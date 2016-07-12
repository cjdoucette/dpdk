/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2010-2014 Intel Corporation. All rights reserved.
 *   All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <string.h>

#include <rte_common.h>
#include <rte_log.h>
#include <rte_memory.h>
#include <rte_malloc.h>
#include <rte_cycles.h>
#include <rte_prefetch.h>
#include <rte_branch_prediction.h>
#include <rte_mbuf.h>

#include "rte_sched_gk.h"
#include "rte_bitmap.h"
#include "rte_sched_common.h"
#include "rte_approx.h"
#include "rte_reciprocal.h"

#ifdef __INTEL_COMPILER
#pragma warning(disable:2259) /* conversion may lose significant bits */
#endif

#ifdef RTE_SCHED_GK_VECTOR
#include <rte_vect.h>

#if defined(__SSE4__)
#define SCHED_GK_VECTOR_SSE4
#endif

#endif

#define RTE_SCHED_GK_TB_RATE_CONFIG_ERR          (1e-7)
#define RTE_SCHED_GK_WRR_SHIFT                   3
#define RTE_SCHED_GK_BMP_POS_INVALID             UINT32_MAX

/* Scaling for cycles_per_byte calculation
 * Chosen so that minimum rate is 480 bit/sec
 */
#define RTE_SCHED_GK_TIME_SHIFT		      8

struct rte_sched_gk_subport {
	/* Token bucket (TB) */
	uint64_t tb_time; /* time of last update */
	uint32_t tb_period;
	uint32_t tb_credits_per_period;
	uint32_t tb_size;
	uint32_t tb_credits;

	/* Statistics */
	struct rte_sched_gk_subport_stats stats;
};

/*
 * Path through the scheduler hierarchy used by the scheduler enqueue
 * operation to identify the destination queue for the current
 * packet. Stored in the field pkt.hash.sched of struct rte_mbuf of
 * each packet, typically written by the classification stage and read
 * by scheduler enqueue.
 */
struct rte_sched_gk_port_hierarchy {
	uint32_t queue;                /**< Queue ID */
	uint32_t subport;                /**< Subport ID */
};

struct rte_sched_gk_port {
	/* User parameters */
	uint32_t n_subports_per_port;
	uint32_t rate;
	uint32_t mtu;
	uint32_t frame_overhead;
	uint16_t qsize;
#ifdef RTE_SCHED_GK_RED
	struct rte_red_config red_config[e_RTE_METER_COLORS];
#endif

	/* Timing */
	uint64_t time_cpu_cycles;     /* Current CPU time measured in CPU cyles */
	uint64_t time_cpu_bytes;      /* Current CPU time measured in bytes */
	uint64_t time;                /* Current NIC TX time measured in bytes */
	struct rte_reciprocal inv_cycles_per_byte; /* CPU cycles per byte */

	/* Bitmap */
	struct rte_bitmap *bmp;

	struct rte_mbuf **pkts_out;
	uint32_t n_pkts_out;

	/* Large data structures */
	struct rte_sched_gk_subport *subport;
	uint8_t *bmp_array;
	struct rte_mbuf **queue_array;
	uint8_t memory[0] __rte_cache_aligned;
} __rte_cache_aligned;

enum rte_sched_gk_port_array {
	e_RTE_SCHED_GK_PORT_ARRAY_SUBPORT = 0,
	e_RTE_SCHED_GK_PORT_ARRAY_PIPE,
	e_RTE_SCHED_GK_PORT_ARRAY_QUEUE,
	e_RTE_SCHED_GK_PORT_ARRAY_QUEUE_EXTRA,
	e_RTE_SCHED_GK_PORT_ARRAY_PIPE_PROFILES,
	e_RTE_SCHED_GK_PORT_ARRAY_BMP_ARRAY,
	e_RTE_SCHED_GK_PORT_ARRAY_QUEUE_ARRAY,
	e_RTE_SCHED_GK_PORT_ARRAY_TOTAL,
};

static int
rte_sched_gk_port_check_params(struct rte_sched_gk_port_params *params)
{
	uint32_t i, j;

	if (params == NULL)
		return -1;

	/* socket */
	if ((params->socket < 0) || (params->socket >= RTE_MAX_NUMA_NODES))
		return -3;

	/* rate */
	if (params->rate == 0)
		return -4;

	/* mtu */
	if (params->mtu == 0)
		return -5;

	/* n_subports_per_port: non-zero, limited to 16 bits, power of 2 */
	if (params->n_subports_per_port == 0 ||
	    params->n_subports_per_port > 1u << 16 ||
	    !rte_is_power_of_2(params->n_subports_per_port))
		return -6;

	/* qsize: non-zero, power of 2,
	 * no bigger than 32K (due to 16-bit read/write pointers)
	 */
	uint16_t qsize = params->qsize[i];

	if (qsize == 0 || !rte_is_power_of_2(qsize))
		return -8;

	return 0;
}

static uint32_t
rte_sched_gk_port_get_array_base(struct rte_sched_gk_port_params *params, enum rte_sched_gk_port_array array)
{
	uint32_t n_subports_per_port = params->n_subports_per_port;
	uint32_t size_subport = n_subports_per_port * sizeof(struct rte_sched_gk_subport);
	/* XXX Replace with real number of queues. */
	uint32_t size_bmp_array = rte_bitmap_get_memory_footprint(4096);
	uint32_t size_queue_array;

	uint32_t base, i;

	size_queue_array = params->qsize[i] * sizeof(struct rte_mbuf *):

	base = 0;

	if (array == e_RTE_SCHED_GK_PORT_ARRAY_SUBPORT)
		return base;
	base += RTE_CACHE_LINE_ROUNDUP(size_subport);

	if (array == e_RTE_SCHED_GK_PORT_ARRAY_BMP_ARRAY)
		return base;
	base += RTE_CACHE_LINE_ROUNDUP(size_bmp_array);

	if (array == e_RTE_SCHED_GK_PORT_ARRAY_QUEUE_ARRAY)
		return base;
	base += RTE_CACHE_LINE_ROUNDUP(size_queue_array);

	return base;
}

uint32_t
rte_sched_gk_port_get_memory_footprint(struct rte_sched_gk_port_params *params)
{
	return 0;
	uint32_t size0, size1;
	int status;

	status = rte_sched_gk_port_check_params(params);
	if (status != 0) {
		RTE_LOG(NOTICE, SCHED,
			"Port scheduler params check failed (%d)\n", status);

		return 0;
	}

	size0 = sizeof(struct rte_sched_gk_port);
	size1 = rte_sched_gk_port_get_array_base(params, e_RTE_SCHED_GK_PORT_ARRAY_TOTAL);

	return size0 + size1;
}

static inline uint64_t
rte_sched_gk_time_ms_to_bytes(uint32_t time_ms, uint32_t rate)
{
	uint64_t time = time_ms;

	time = (time * rate) / 1000;

	return time;
}

struct rte_sched_gk_port *
rte_sched_gk_port_config(struct rte_sched_gk_port_params *params)
{
	struct rte_sched_gk_port *port = NULL;
	uint32_t cycles_per_byte;

	/* XXX change according to port->memory
	 * Allocate memory to store the data structures */
	port = rte_zmalloc("qos_params", sizeof(*port), RTE_CACHE_LINE_SIZE);
	if (port == NULL)
		return NULL;

	/* User parameters */
	port->n_subports_per_port = params->n_subports_per_port;
	port->rate = params->rate;
	port->mtu = params->mtu + params->frame_overhead;
	port->frame_overhead = params->frame_overhead;
	port->qsize = params->qsize;

	/* Timing */
	port->time_cpu_cycles = rte_get_tsc_cycles();
	port->time_cpu_bytes = 0;
	port->time = 0;

	cycles_per_byte = (rte_get_tsc_hz() << RTE_SCHED_GK_TIME_SHIFT)
		/ params->rate;
	port->inv_cycles_per_byte = rte_reciprocal_value(cycles_per_byte);

	port->pkts_out = NULL;
	port->n_pkts_out = 0;

	/* XXX what is this? Large data structures */
	port->subport = (struct rte_sched_gk_subport *)
		(port->memory + rte_sched_gk_port_get_array_base(params,
							      e_RTE_SCHED_GK_PORT_ARRAY_SUBPORT));

	return port;
}

void
rte_sched_gk_port_free(struct rte_sched_gk_port *port)
{
	unsigned int queue;

	/* Check user parameters */
	if (port == NULL)
		return;

	/* Free enqueued mbufs */
/*
	for (queue = 0; queue < RTE_SCHED_GK_TRAFFIC_CLASSES_PER_PIPE; queue++) {
		struct rte_mbuf **mbufs = rte_sched_gk_port_qbase(port, queue);
		unsigned int i;

		for (i = 0; i < rte_sched_gk_port_qsize(port, queue); i++)
			rte_pktmbuf_free(mbufs[i]);
	}

*/
	rte_free(port);
}

static void
rte_sched_gk_port_log_subport_config(struct rte_sched_gk_port *port, uint32_t i)
{
	struct rte_sched_gk_subport *s = port->subport + i;

	RTE_LOG(DEBUG, SCHED, "Low level config for subport %u:\n"
		"    Token bucket: period = %u, credits per period = %u, size = %u\n"
		i,

		/* Token bucket */
		s->tb_period,
		s->tb_credits_per_period,
		s->tb_size,
}

int
rte_sched_gk_subport_config(struct rte_sched_gk_port *port,
	uint32_t subport_id,
	struct rte_sched_gk_subport_params *params)
{
	struct rte_sched_gk_subport *s;
	uint32_t i;

	/* Check user parameters */
	if (port == NULL ||
	    subport_id >= port->n_subports_per_port ||
	    params == NULL)
		return -1;

	if (params->tb_rate == 0 || params->tb_rate > port->rate)
		return -2;

	if (params->tb_size == 0)
		return -3;

	s = port->subport + subport_id;

	/* Token Bucket (TB) */
	if (params->tb_rate == port->rate) {
		s->tb_credits_per_period = 1;
		s->tb_period = 1;
	} else {
		double tb_rate = ((double) params->tb_rate) / ((double) port->rate);
		double d = RTE_SCHED_GK_TB_RATE_CONFIG_ERR;

		rte_approx(tb_rate, d, &s->tb_credits_per_period, &s->tb_period);
	}

	s->tb_size = params->tb_size;
	s->tb_time = port->time;
	s->tb_credits = s->tb_size / 2;

	rte_sched_gk_port_log_subport_config(port, subport_id);

	return 0;
}

void
rte_sched_gk_port_pkt_write(struct rte_mbuf *pkt, uint32_t subport, 
			uint32_t queue)
{
	struct rte_sched_gk_port_hierarchy *sched
		= (struct rte_sched_gk_port_hierarchy *) &pkt->hash.sched;

	RTE_BUILD_BUG_ON(sizeof(*sched) > sizeof(pkt->hash.sched));

	sched->subport = subport;
	sched->queue = queue;
}

void
rte_sched_gk_port_pkt_read_tree_path(const struct rte_mbuf *pkt,
				  uint32_t *subport, uint32_t *queue)
{
	const struct rte_sched_gk_port_hierarchy *sched
		= (const struct rte_sched_gk_port_hierarchy *) &pkt->hash.sched;

	*subport = sched->subport;
	*queue = sched->queue;
}

int
rte_sched_gk_subport_read_stats(struct rte_sched_gk_port *port,
			     uint32_t subport_id,
			     struct rte_sched_gk_subport_stats *stats,
			     uint32_t *tc_ov)
{
	struct rte_sched_gk_subport *s;

	/* Check user parameters */
	if (port == NULL || subport_id >= port->n_subports_per_port ||
	    stats == NULL || tc_ov == NULL)
		return -1;

	s = port->subport + subport_id;

	/* Copy subport stats and clear */
	memcpy(stats, &s->stats, sizeof(struct rte_sched_gk_subport_stats));
	memset(&s->stats, 0, sizeof(struct rte_sched_gk_subport_stats));

	/* Subport TC ovesubscription status */
	*tc_ov = s->tc_ov;

	return 0;
}

#ifdef RTE_SCHED_GK_COLLECT_STATS

static inline void
rte_sched_gk_port_update_subport_stats(struct rte_sched_gk_port *port, uint32_t i, struct rte_mbuf *pkt)
{
	struct rte_sched_gk_subport *s = port->subport + i;
	uint32_t pkt_len = pkt->pkt_len;

	s->stats.n_pkts_tc += 1;
	s->stats.n_bytes_tc += pkt_len;
}

static inline void
rte_sched_gk_port_update_subport_stats_on_drop(struct rte_sched_gk_port *port,
						uint32_t i,
						struct rte_mbuf *pkt, __rte_unused uint32_t red)
{
	struct rte_sched_gk_subport *s = port->subport + i;
	uint32_t pkt_len = pkt->pkt_len;

	s->stats.n_pkts_dropped += 1;
	s->stats.n_bytes_dropped += pkt_len;
}

#endif

static inline uint32_t
rte_sched_gk_port_enqueue_qptrs_prefetch0(struct rte_sched_gk_port *port,
				       struct rte_mbuf *pkt)
{
	struct rte_sched_gk_queue *q;
#ifdef RTE_SCHED_GK_COLLECT_STATS
	struct rte_sched_gk_queue_extra *qe;
#endif
	uint32_t subport, queue;

	rte_sched_gk_port_pkt_read_tree_path(pkt, &subport, &queue);

	/* XXX Assume only one queue for now. */
	q = port->queue;
	rte_prefetch0(q);
#ifdef RTE_SCHED_GK_COLLECT_STATS
	qe = port->queue_extra;
	rte_prefetch0(qe);
#endif

	return 0;
}

static inline void
rte_sched_gk_port_enqueue_qwa_prefetch0(struct rte_sched_gk_port *port,
				     uint32_t qindex, struct rte_mbuf **qbase)
{
	struct rte_sched_gk_queue *q;
	struct rte_mbuf **q_qw;
	uint16_t qsize;

	q = port->queue + qindex;
	qsize = rte_sched_gk_port_qsize(port, qindex);
	q_qw = qbase + (q->qw & (qsize - 1));

	rte_prefetch0(q_qw);
	rte_bitmap_prefetch0(port->bmp, qindex);
}

static inline int
rte_sched_gk_port_enqueue_qwa(struct rte_sched_gk_port *port, uint32_t qindex,
			   struct rte_mbuf **qbase, struct rte_mbuf *pkt)
{
	struct rte_sched_gk_queue *q;
	uint16_t qsize;
	uint16_t qlen;

	q = port->queue + qindex;
	qsize = rte_sched_gk_port_qsize(port, qindex);
	qlen = q->qw - q->qr;

	/* Drop the packet (and update drop stats) when queue is full */
	if (unlikely(rte_sched_gk_port_red_drop(port, pkt, qindex, qlen) ||
		     (qlen >= qsize))) {
		rte_pktmbuf_free(pkt);
#ifdef RTE_SCHED_GK_COLLECT_STATS
		rte_sched_gk_port_update_subport_stats_on_drop(port, qindex, pkt,
							    qlen < qsize);
		rte_sched_gk_port_update_queue_stats_on_drop(port, qindex, pkt,
							  qlen < qsize);
#endif
		return 0;
	}

	/* Enqueue packet */
	qbase[q->qw & (qsize - 1)] = pkt;
	q->qw++;

	/* Activate queue in the port bitmap */
	rte_bitmap_set(port->bmp, qindex);

	/* Statistics */
#ifdef RTE_SCHED_GK_COLLECT_STATS
	rte_sched_gk_port_update_subport_stats(port, qindex, pkt);
	rte_sched_gk_port_update_queue_stats(port, qindex, pkt);
#endif

	return 1;
}


/*
 * The enqueue function implements a 4-level pipeline with each stage
 * processing two different packets. The purpose of using a pipeline
 * is to hide the latency of prefetching the data structures. The
 * naming convention is presented in the diagram below:
 *
 *   p00  _______   p10  _______   p20  _______   p30  _______
 * ----->|       |----->|       |----->|       |----->|       |----->
 *       |   0   |      |   1   |      |   2   |      |   3   |
 * ----->|_______|----->|_______|----->|_______|----->|_______|----->
 *   p01            p11            p21            p31
 *
 */

/*
 * XXX Rewrite this function using the Gatekeeper queues instead of the
 * DPDK QoS queues. This includes getting the queue index for each packet,
 * getting the queue that corresponds to that index, and inserting the
 * packet. Prefetching should be used in the same way as DPDK QoS.
 */
int
rte_sched_gk_port_enqueue(struct rte_sched_gk_port *port, struct rte_mbuf **pkts,
		       uint32_t n_pkts)
{
	struct rte_mbuf *pkt00, *pkt01, *pkt10, *pkt11, *pkt20, *pkt21,
		*pkt30, *pkt31, *pkt_last;
	struct rte_mbuf **q00_base, **q01_base, **q10_base, **q11_base,
		**q20_base, **q21_base, **q30_base, **q31_base, **q_last_base;
	uint32_t q00, q01, q10, q11, q20, q21, q30, q31, q_last;
	uint32_t r00, r01, r10, r11, r20, r21, r30, r31, r_last;
	uint32_t result, i;

	result = 0;

	/*
	 * Less then 6 input packets available, which is not enough to
	 * feed the pipeline
	 */
	if (unlikely(n_pkts < 6)) {
		struct rte_mbuf **q_base[5];
		uint32_t q[5];

		/* Prefetch the mbuf structure of each packet */
		for (i = 0; i < n_pkts; i++)
			rte_prefetch0(pkts[i]);

		/* Prefetch the queue structure for each queue */
		for (i = 0; i < n_pkts; i++)
			/*
			 * XXX Rewrite function to get queue index -- should be
			 * queue that corresponds to priority of packet.
			 */
			q[i] = rte_sched_gk_port_enqueue_qptrs_prefetch0(port,
								      pkts[i]);

		/* Prefetch the write pointer location of each queue */
		for (i = 0; i < n_pkts; i++) {
			/*
			 * XXX Rewrite function to get queue based on
			 * queue index. Again, should be really easy,
			 * but need to preserve prefetching of write
			 * location of each queue.
			 */
			q_base[i] = rte_sched_gk_port_qbase(port, q[i]);
			rte_sched_gk_port_enqueue_qwa_prefetch0(port, q[i],
							     q_base[i]);
		}

		/* Write each packet to its queue */
		for (i = 0; i < n_pkts; i++)
			/*
			 * XXX Rewrite function to enqueue. Need to decide
			 * whether queue will be array or LL. Should keep
			 * RED for priority packets. Should keep bitmap
			 * for priority packets.
			 */
			result += rte_sched_gk_port_enqueue_qwa(port, q[i],
							     q_base[i], pkts[i]);

		return result;
	}

	/* XXX Preserve the following prefetching, using same functions. */

	/* Feed the first 3 stages of the pipeline (6 packets needed) */
	pkt20 = pkts[0];
	pkt21 = pkts[1];
	rte_prefetch0(pkt20);
	rte_prefetch0(pkt21);

	pkt10 = pkts[2];
	pkt11 = pkts[3];
	rte_prefetch0(pkt10);
	rte_prefetch0(pkt11);

	q20 = rte_sched_gk_port_enqueue_qptrs_prefetch0(port, pkt20);
	q21 = rte_sched_gk_port_enqueue_qptrs_prefetch0(port, pkt21);

	pkt00 = pkts[4];
	pkt01 = pkts[5];
	rte_prefetch0(pkt00);
	rte_prefetch0(pkt01);

	q10 = rte_sched_gk_port_enqueue_qptrs_prefetch0(port, pkt10);
	q11 = rte_sched_gk_port_enqueue_qptrs_prefetch0(port, pkt11);

	q20_base = rte_sched_gk_port_qbase(port, q20);
	q21_base = rte_sched_gk_port_qbase(port, q21);
	rte_sched_gk_port_enqueue_qwa_prefetch0(port, q20, q20_base);
	rte_sched_gk_port_enqueue_qwa_prefetch0(port, q21, q21_base);

	/* Run the pipeline */
	for (i = 6; i < (n_pkts & (~1)); i += 2) {
		/* Propagate stage inputs */
		pkt30 = pkt20;
		pkt31 = pkt21;
		pkt20 = pkt10;
		pkt21 = pkt11;
		pkt10 = pkt00;
		pkt11 = pkt01;
		q30 = q20;
		q31 = q21;
		q20 = q10;
		q21 = q11;
		q30_base = q20_base;
		q31_base = q21_base;

		/* Stage 0: Get packets in */
		pkt00 = pkts[i];
		pkt01 = pkts[i + 1];
		rte_prefetch0(pkt00);
		rte_prefetch0(pkt01);

		/* Stage 1: Prefetch queue structure storing queue pointers */
		q10 = rte_sched_gk_port_enqueue_qptrs_prefetch0(port, pkt10);
		q11 = rte_sched_gk_port_enqueue_qptrs_prefetch0(port, pkt11);

		/* Stage 2: Prefetch queue write location */
		q20_base = rte_sched_gk_port_qbase(port, q20);
		q21_base = rte_sched_gk_port_qbase(port, q21);
		rte_sched_gk_port_enqueue_qwa_prefetch0(port, q20, q20_base);
		rte_sched_gk_port_enqueue_qwa_prefetch0(port, q21, q21_base);

		/* Stage 3: Write packet to queue and activate queue */
		r30 = rte_sched_gk_port_enqueue_qwa(port, q30, q30_base, pkt30);
		r31 = rte_sched_gk_port_enqueue_qwa(port, q31, q31_base, pkt31);
		result += r30 + r31;
	}

	/*
	 * Drain the pipeline (exactly 6 packets).
	 * Handle the last packet in the case
	 * of an odd number of input packets.
	 */
	pkt_last = pkts[n_pkts - 1];
	rte_prefetch0(pkt_last);

	q00 = rte_sched_gk_port_enqueue_qptrs_prefetch0(port, pkt00);
	q01 = rte_sched_gk_port_enqueue_qptrs_prefetch0(port, pkt01);

	q10_base = rte_sched_gk_port_qbase(port, q10);
	q11_base = rte_sched_gk_port_qbase(port, q11);
	rte_sched_gk_port_enqueue_qwa_prefetch0(port, q10, q10_base);
	rte_sched_gk_port_enqueue_qwa_prefetch0(port, q11, q11_base);

	r20 = rte_sched_gk_port_enqueue_qwa(port, q20, q20_base, pkt20);
	r21 = rte_sched_gk_port_enqueue_qwa(port, q21, q21_base, pkt21);
	result += r20 + r21;

	q_last = rte_sched_gk_port_enqueue_qptrs_prefetch0(port, pkt_last);

	q00_base = rte_sched_gk_port_qbase(port, q00);
	q01_base = rte_sched_gk_port_qbase(port, q01);
	rte_sched_gk_port_enqueue_qwa_prefetch0(port, q00, q00_base);
	rte_sched_gk_port_enqueue_qwa_prefetch0(port, q01, q01_base);

	r10 = rte_sched_gk_port_enqueue_qwa(port, q10, q10_base, pkt10);
	r11 = rte_sched_gk_port_enqueue_qwa(port, q11, q11_base, pkt11);
	result += r10 + r11;

	q_last_base = rte_sched_gk_port_qbase(port, q_last);
	rte_sched_gk_port_enqueue_qwa_prefetch0(port, q_last, q_last_base);

	r00 = rte_sched_gk_port_enqueue_qwa(port, q00, q00_base, pkt00);
	r01 = rte_sched_gk_port_enqueue_qwa(port, q01, q01_base, pkt01);
	result += r00 + r01;

	if (n_pkts & 1) {
		r_last = rte_sched_gk_port_enqueue_qwa(port, q_last, q_last_base, pkt_last);
		result += r_last;
	}

	return result;
}

static inline void
grinder_credits_update(struct rte_sched_gk_port *port, uint32_t pos)
{
	struct rte_sched_gk_subport *subport = grinder->subport;
	uint64_t n_periods;

	/* Subport TB */
	n_periods = (port->time - subport->tb_time) / subport->tb_period;
	subport->tb_credits += n_periods * subport->tb_credits_per_period;
	subport->tb_credits = rte_sched_min_val_2_u32(subport->tb_credits, subport->tb_size);
	subport->tb_time += n_periods * subport->tb_period;
}

static inline int
grinder_credits_check(struct rte_sched_gk_port *port, uint32_t pos)
{
	struct rte_sched_gk_subport *subport = grinder->subport;
	struct rte_mbuf *pkt = grinder->pkt;
	uint32_t pkt_len = pkt->pkt_len + port->frame_overhead;
	uint32_t subport_tb_credits = subport->tb_credits;
	int enough_credits;

	/* Check queue credits */
	enough_credits = (pkt_len <= subport_tb_credits);

	if (!enough_credits)
		return 0;

	/* Update port credits */
	subport->tb_credits -= pkt_len;

	return 1;
}

static inline int
grinder_schedule(struct rte_sched_gk_port *port, uint32_t pos)
{
	struct rte_mbuf *pkt = grinder->pkt;
	uint32_t pkt_len = pkt->pkt_len + port->frame_overhead;

	/*
	 * XXX Need to keep track of subport and its credits without
	 * associating it with a grinder. Then we can check the subport's
	 * credits directly against the packet. So, need to update this
	 * function to do that and then update credits as needed.
	 */
	if (!grinder_credits_check(port, pos))
		return 0;

	/* Advance port time */
	/* XXX Clearly, this is needed. */
	port->time += pkt_len;

	/* Send packet */
	port->pkts_out[port->n_pkts_out++] = pkt;
	/*
	 * XXX If we want to keep the bitmap functionality, then we need
	 * to make sure we clear it below (if the queue is empty?). But
	 * otherwise, I don't think we need anything below.
	 */
	queue->qr++;
	grinder->wrr_tokens[grinder->qpos] += pkt_len * grinder->wrr_cost[grinder->qpos];
	if (queue->qr == queue->qw) {
		uint32_t qindex = grinder->qindex[grinder->qpos];

		rte_bitmap_clear(port->bmp, qindex);
		grinder->qmask &= ~(1 << grinder->qpos);
		grinder->wrr_mask[grinder->qpos] = 0;
		rte_sched_gk_port_set_queue_empty_timestamp(port, qindex);
	}

	/* Reset pipe loop detection */
	/* XXX Still unclear whether the following are needed. */
	port->pipe_loop = RTE_SCHED_GK_PIPE_INVALID;
	grinder->productive = 1;

	return 1;
}

#ifdef SCHED_GK_VECTOR_SSE4

static inline int
grinder_pipe_exists(struct rte_sched_gk_port *port, uint32_t base_pipe)
{
	__m128i index = _mm_set1_epi32(base_pipe);
	__m128i pipes = _mm_load_si128((__m128i *)port->grinder_base_bmp_pos);
	__m128i res = _mm_cmpeq_epi32(pipes, index);

	pipes = _mm_load_si128((__m128i *)(port->grinder_base_bmp_pos + 4));
	pipes = _mm_cmpeq_epi32(pipes, index);
	res = _mm_or_si128(res, pipes);

	if (_mm_testz_si128(res, res))
		return 0;

	return 1;
}

#else

static inline int
grinder_pipe_exists(struct rte_sched_gk_port *port, uint32_t base_pipe)
{
	uint32_t i;

	for (i = 0; i < RTE_SCHED_PORT_N_GRINDERS; i++) {
		if (port->grinder_base_bmp_pos[i] == base_pipe)
			return 1;
	}

	return 0;
}

#endif /* RTE_SCHED_GK_OPTIMIZATIONS */

static inline void
grinder_prefetch_mbuf(struct rte_sched_gk_port *port, uint32_t pos)
{
	struct rte_sched_gk_grinder *grinder = port->grinder + pos;
	uint32_t qpos = grinder->qpos;
	struct rte_mbuf **qbase = grinder->qbase[qpos];
	uint16_t qsize = grinder->qsize;
	uint16_t qr = grinder->queue[qpos]->qr & (qsize - 1);

	grinder->pkt = qbase[qr];
	rte_prefetch0(grinder->pkt);

	if (unlikely((qr & 0x7) == 7)) {
		uint16_t qr_next = (grinder->queue[qpos]->qr + 1) & (qsize - 1);

		rte_prefetch0(qbase + qr_next);
	}
}

static inline uint32_t
grinder_handle(struct rte_sched_gk_port *port, uint32_t pos)
{
	struct rte_sched_gk_grinder *grinder = port->grinder + pos;

	switch (grinder->state) {
	case e_GRINDER_PREFETCH_PIPE:
	{
		if (grinder_next_pipe(port, pos)) {
			grinder_prefetch_pipe(port, pos);
			port->busy_grinders++;

			grinder->state = e_GRINDER_PREFETCH_TC_QUEUE_ARRAYS;
			return 0;
		}

		return 0;
	}

	case e_GRINDER_PREFETCH_TC_QUEUE_ARRAYS:
	{
		struct rte_sched_gk_pipe *pipe = grinder->pipe;

		grinder->pipe_params = port->pipe_profiles + pipe->profile;
		grinder_prefetch_tc_queue_arrays(port, pos);
		grinder_credits_update(port, pos);

		grinder->state = e_GRINDER_PREFETCH_MBUF;
		return 0;
	}

	case e_GRINDER_PREFETCH_MBUF:
	{
		grinder_prefetch_mbuf(port, pos);

		grinder->state = e_GRINDER_READ_MBUF;
		return 0;
	}

	case e_GRINDER_READ_MBUF:
	{
		uint32_t result = 0;

		result = grinder_schedule(port, pos);

		/*
		 * XXX Figure out whether next packet is in current queue,
		 * or which queue it's in. Prefetch it. Probably ignore
		 * the rest of what's below.
		 */

		/* Look for next packet within the same TC */
		if (result && grinder->qmask) {
			grinder_wrr(port, pos);
			grinder_prefetch_mbuf(port, pos);

			return 1;
		}
		grinder_wrr_store(port, pos);

		/* Look for another active TC within same pipe */
		if (grinder_next_tc(port, pos)) {
			grinder_prefetch_tc_queue_arrays(port, pos);

			grinder->state = e_GRINDER_PREFETCH_MBUF;
			return result;
		}

		/* XXX Should check to see if we need this. */
		if (grinder->productive == 0 &&
		    port->pipe_loop == RTE_SCHED_GK_PIPE_INVALID)
			port->pipe_loop = grinder->pindex;

		grinder_evict(port, pos);

		/* Look for another active pipe */
		if (grinder_next_pipe(port, pos)) {
			grinder_prefetch_pipe(port, pos);

			grinder->state = e_GRINDER_PREFETCH_TC_QUEUE_ARRAYS;
			return result;
		}

		/* No active pipe found */
		port->busy_grinders--;

		grinder->state = e_GRINDER_PREFETCH_PIPE;
		return result;
	}

	default:
		rte_panic("Algorithmic error (invalid state)\n");
		return 0;
	}
}

static inline void
rte_sched_gk_port_time_resync(struct rte_sched_gk_port *port)
{
	uint64_t cycles = rte_get_tsc_cycles();
	uint64_t cycles_diff = cycles - port->time_cpu_cycles;
	uint64_t bytes_diff;

	/* Compute elapsed time in bytes */
	bytes_diff = rte_reciprocal_divide(cycles_diff << RTE_SCHED_GK_TIME_SHIFT,
					   port->inv_cycles_per_byte);

	/* Advance port time */
	port->time_cpu_cycles = cycles;
	port->time_cpu_bytes += bytes_diff;
	if (port->time < port->time_cpu_bytes)
		port->time = port->time_cpu_bytes;

	/* Reset pipe loop detection */
	port->pipe_loop = RTE_SCHED_GK_PIPE_INVALID;
}

static inline int
rte_sched_gk_port_exceptions(struct rte_sched_gk_port *port, int second_pass)
{
	int exceptions;

	/* Check if any exception flag is set */
	exceptions = (second_pass && port->busy_grinders == 0) ||
		(port->pipe_exhaustion == 1);

	/* Clear exception flags */
	port->pipe_exhaustion = 0;

	return exceptions;
}

int
rte_sched_gk_port_dequeue(struct rte_sched_gk_port *port, struct rte_mbuf **pkts, uint32_t n_pkts)
{
	uint32_t i, count;

	port->pkts_out = pkts;
	port->n_pkts_out = 0;

	/*
	 * XXX I think we need this for timing information for filling up
	 * and removing tokens from the bucket. But, find where port->time*
	 * and port->pipe_loop is read after this point to be sure it's needed.
	 */
	rte_sched_gk_port_time_resync(port);

	/* Take each queue in the grinder one step further */
	for (i = 0, count = 0; ; i++)  {
		count += grinder_handle(port, i & (RTE_SCHED_PORT_N_GRINDERS - 1));
		if ((count == n_pkts) ||
		    rte_sched_gk_port_exceptions(port, i >= RTE_SCHED_PORT_N_GRINDERS)) {
			break;
		}
	}

	return count;
}
