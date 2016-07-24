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

#include <rte_cycles.h>
#include <rte_ethdev.h>

#include "main.h"

#define BURST_TX_DRAIN_US 100

#define GK_REQ_PKT	0
#define GK_CAP_PKT	1

/*
 * Path through the scheduler hierarchy used by the scheduler enqueue
 * operation to identify the destination queue for the current
 * packet. Stored in the field pkt.hash.sched of struct rte_mbuf of
 * each packet, typically written by the classification stage and read
 * by scheduler enqueue.
 */
struct rte_sched_port_hierarchy {
	uint16_t queue:2;                /**< Queue ID (0 .. 3) */
	uint16_t traffic_class:2;        /**< Traffic class ID (0 .. 3)*/
	uint32_t color:2;                /**< Color */
	uint16_t unused:10;
	uint16_t subport;                /**< Subport ID */
	uint32_t pipe;		         /**< Pipe ID */
};

static inline int
get_pkt_sched(struct rte_mbuf *m, uint32_t *type, uint32_t *queue)
{
	/* uint16_t *pdata = rte_pktmbuf_mtod(m, uint16_t *); */

	struct rte_sched_port_hierarchy *sched;
	sched = (struct rte_sched_port_hierarchy *)&m->hash.sched;

	/* XXX Replace with lookup and hash. */
	*type = ((uint64_t)m + rte_rand() % 100) < 5 ? GK_REQ_PKT : GK_CAP_PKT;
	*queue = *type == GK_REQ_PKT ? 0 : rte_rand() % 4096;

	/* Don't use pipe, color, or traffic_class. */
	sched->subport = *type;
	sched->queue = *queue;

	return 0;
}

/*
 * Send the packets to an output interface.
 */
static inline void
send_burst(struct gk_data *gk, struct dst_queues *dst_queues)
{
	uint16_t ret;

	do {
		ret = rte_eth_tx_burst(gk->tx_port, gk->tx_queue,
			dst_queues->pkts_out, dst_queues->n_pkts_out);

		/* We cannot drop the packets, so re-send. */
		dst_queues->n_pkts_out -= ret;
		dst_queues->pkts_out += ret;
	} while (dst_queues->n_pkts_out);
}

static int
req_enqueue(__attribute__((unused)) struct req_queue *req_queue,
	__attribute__((unused)) struct rte_mbuf **mbufs,
	__attribute__((unused)) uint32_t num_pkts)
{
	return 0;
}

static int
req_dequeue(__attribute__((unused)) struct req_queue *req_queue,
	__attribute__((unused)) struct rte_mbuf **mbufs,
	__attribute__((unused)) uint32_t num_pkts)
{
	return 0;
}

void
req_thread(struct gk_data *gk, struct req_queue *req_queue)
{
	struct rte_mbuf *mbufs[gk->qos_enqueue_size];

	while (1) {
		uint32_t nb_pkt;
		int ret;

		ret = rte_ring_sc_dequeue_bulk(gk->req_rx_ring,
			(void **)mbufs, gk->qos_enqueue_size);
		if (likely(ret == 0))
			/* XXX Catch return value and maintain stats. */
			req_enqueue(req_queue, mbufs, gk->qos_enqueue_size);

		nb_pkt = req_dequeue(req_queue, mbufs, gk->qos_dequeue_size);
#if USE_TX_THREADS
		if (likely(nb_pkt > 0))
			while (rte_ring_sp_enqueue_bulk(gk->req_tx_ring,
				(void **)mbufs, nb_pkt) != 0);
#endif
	}
}

static inline void
pkt_read_tree_path(const struct rte_mbuf *pkt, uint32_t *type, uint32_t *queue)
{
	const struct rte_sched_port_hierarchy *sched
		= (const struct rte_sched_port_hierarchy *) &pkt->hash.sched;

	/* XXX Is this needed? */
	*type = sched->subport;

	*queue = sched->queue;
}

/*
 * Based on rte_sched_port_enqueue_qptrs_prefetch0().
 * This version is simpler, since there are no subports,
 * pipes, or traffic classes, makingt the queue ID the
 * same as the offset into the queue array.
 */
static inline uint32_t
enqueue_qptrs_prefetch0(struct dst_queues *dst_queues, struct rte_mbuf *pkt)
{
	uint32_t type, qindex;
	/* XXX Should we still store the type and queue with each packet? */
	pkt_read_tree_path(pkt, &type, &qindex);
	rte_prefetch0(dst_queues->queue + qindex);
	return qindex;
}

/*
 * Based on rte_sched_port_qbase().
 * This version is simpler, since we don't have to break
 * the queue index down into the pipe index and queue position.
 */
static inline struct rte_mbuf **
dst_qbase(struct dst_queues *dst_queues, uint32_t qindex)
{
	return dst_queues->queue_array + (qindex * dst_queues->qsize);
}

/*
 * Based on rte_sched_port_enqueue_qwa_prefetch0().
 */
static inline void
enqueue_qwa_prefetch0(struct dst_queues *dst_queues, uint32_t qindex,
	struct rte_mbuf **qbase)
{
	struct gk_queue *q;
	struct rte_mbuf **q_qw;

	q = dst_queues->queue + qindex;
	q_qw = qbase + (q->qw & (dst_queues->qsize - 1));

	rte_prefetch0(q_qw);
	rte_bitmap_prefetch0(dst_queues->bmp, qindex);
}

/*
 * Based on rte_sched_port_enqueue_qwa().
 */
static inline int
enqueue_qwa(struct dst_queues *dst_queues, uint32_t qindex,
	struct rte_mbuf **qbase, struct rte_mbuf *pkt)
{
	struct gk_queue *q;
	uint16_t qlen;

	q = dst_queues->queue + qindex;
	qlen = q->qw - q->qr;

	/* XXX Add RED drop here. */
	if (unlikely(qlen >= dst_queues->qsize)) {
		rte_pktmbuf_free(pkt);
		return 0;
	}

	/* Enqueue packet. */
	qbase[q->qw & (dst_queues->qsize - 1)] = pkt;
	q->qw++;

	/* Activate queue in the bitmap. */
	rte_bitmap_set(dst_queues->bmp, qindex);

	return 1;
}

static int
dst_enqueue(struct dst_queues *dst_queues, struct rte_mbuf **pkts,
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
	 * Less then 6 input packets available, which
	 * is not enough to feed the pipeline.
	 */
	if (unlikely(n_pkts < 6)) {
		struct rte_mbuf **q_base[5];
		uint32_t q[5];

		/* Prefetch the mbuf structure of each packet. */
		for (i = 0; i < n_pkts; i++)
			rte_prefetch0(pkts[i]);

		/* Prefetch the queue metadata structure for each queue. */
		for (i = 0; i < n_pkts; i++)
			q[i] = enqueue_qptrs_prefetch0(dst_queues, pkts[i]);

		/* Prefetch the write pointer location of each queue. */
		for (i = 0; i < n_pkts; i++) {
			q_base[i] = dst_qbase(dst_queues, q[i]);
			enqueue_qwa_prefetch0(dst_queues, q[i], q_base[i]);
		}

		/* Write each packet to its queue */
		for (i = 0; i < n_pkts; i++)
			result += enqueue_qwa(dst_queues, q[i],
				q_base[i], pkts[i]);

		return result;
	}

	/* Feed the first 3 stages of the pipeline (6 packets needed). */
	pkt20 = pkts[0];
	pkt21 = pkts[1];
	rte_prefetch0(pkt20);
	rte_prefetch0(pkt21);

	pkt10 = pkts[2];
	pkt11 = pkts[3];
	rte_prefetch0(pkt10);
	rte_prefetch0(pkt11);

	q20 = enqueue_qptrs_prefetch0(dst_queues, pkt20);
	q21 = enqueue_qptrs_prefetch0(dst_queues, pkt21);

	pkt00 = pkts[4];
	pkt01 = pkts[5];
	rte_prefetch0(pkt00);
	rte_prefetch0(pkt01);

	q10 = enqueue_qptrs_prefetch0(dst_queues, pkt10);
	q11 = enqueue_qptrs_prefetch0(dst_queues, pkt11);

	q20_base = dst_qbase(dst_queues, q20);
	q21_base = dst_qbase(dst_queues, q21);
	enqueue_qwa_prefetch0(dst_queues, q20, q20_base);
	enqueue_qwa_prefetch0(dst_queues, q21, q21_base);

	/* Run the pipeline. */
	for (i = 6; i < (n_pkts & (~1)); i += 2) {
		/* Propagate stage inputs. */
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

		/* Stage 0: Get packets in. */
		pkt00 = pkts[i];
		pkt01 = pkts[i + 1];
		rte_prefetch0(pkt00);
		rte_prefetch0(pkt01);

		/* Stage 1: Prefetch queue structure storing queue pointers. */
		q10 = enqueue_qptrs_prefetch0(dst_queues, pkt10);
		q11 = enqueue_qptrs_prefetch0(dst_queues, pkt11);

		/* Stage 2: Prefetch queue write location. */
		q20_base = dst_qbase(dst_queues, q20);
		q21_base = dst_qbase(dst_queues, q21);
		enqueue_qwa_prefetch0(dst_queues, q20, q20_base);
		enqueue_qwa_prefetch0(dst_queues, q21, q21_base);

		/* Stage 3: Write packet to queue and activate queue. */
		r30 = enqueue_qwa(dst_queues, q30, q30_base, pkt30);
		r31 = enqueue_qwa(dst_queues, q31, q31_base, pkt31);
		result += r30 + r31;
	}

	/*
	 * Drain the pipeline (exactly 6 packets).
	 * Handle the last packet in the case
	 * of an odd number of input packets.
	 */
	pkt_last = pkts[n_pkts - 1];
	rte_prefetch0(pkt_last);

	q00 = enqueue_qptrs_prefetch0(dst_queues, pkt00);
	q01 = enqueue_qptrs_prefetch0(dst_queues, pkt01);

	q10_base = dst_qbase(dst_queues, q10);
	q11_base = dst_qbase(dst_queues, q11);
	enqueue_qwa_prefetch0(dst_queues, q10, q10_base);
	enqueue_qwa_prefetch0(dst_queues, q11, q11_base);

	r20 = enqueue_qwa(dst_queues, q20, q20_base, pkt20);
	r21 = enqueue_qwa(dst_queues, q21, q21_base, pkt21);
	result += r20 + r21;

	q_last = enqueue_qptrs_prefetch0(dst_queues, pkt_last);

	q00_base = dst_qbase(dst_queues, q00);
	q01_base = dst_qbase(dst_queues, q01);
	enqueue_qwa_prefetch0(dst_queues, q00, q00_base);
	enqueue_qwa_prefetch0(dst_queues, q01, q01_base);

	r10 = enqueue_qwa(dst_queues, q10, q10_base, pkt10);
	r11 = enqueue_qwa(dst_queues, q11, q11_base, pkt11);
	result += r10 + r11;

	q_last_base = dst_qbase(dst_queues, q_last);
	enqueue_qwa_prefetch0(dst_queues, q_last, q_last_base);

	r00 = enqueue_qwa(dst_queues, q00, q00_base, pkt00);
	r01 = enqueue_qwa(dst_queues, q01, q01_base, pkt01);
	result += r00 + r01;

	if (n_pkts & 1) {
		r_last = enqueue_qwa(dst_queues, q_last, q_last_base, pkt_last);
		result += r_last;
	}

	return result;
}

static void
__dst_dequeue(struct dst_queues *dst_queues, uint32_t num_pkts)
{
	struct gk_queue *q = dst_queues->queue + dst_queues->cur_queue;
	struct rte_mbuf **qbase = dst_qbase(dst_queues, dst_queues->cur_queue);
	uint16_t qr;

	do {
		/* XXX Check credits. */
		// grinder_credits_check().

		qr = q->qr & (dst_queues->qsize - 1);

		/* Advance port time. */
		dst_queues->time += qbase[qr]->pkt_len +
			dst_queues->frame_overhead;

		/* Queue packet. */
		dst_queues->pkts_out[dst_queues->n_pkts_out] = qbase[qr];

		/* Update number of packets queued. */
		dst_queues->n_pkts_out++;
		/* Update number of slots left. */
		num_pkts--;
		/* Update queue metadata. */
		q->qr++;
	} while (q->qr != q->qw && num_pkts > 0);

	if (q->qr == q->qw) {
		rte_bitmap_clear(dst_queues->bmp, dst_queues->cur_queue);
		/* XXX rte_sched_port_set_queue_empty_timestamp(). */
	} 
}

/*
 * XXX Implement prefetching using grinders, as in rte_sched_port_dequeue().
 */
static uint32_t
dst_dequeue(struct dst_queues *dst_queues, const uint32_t num_pkts)
{
	uint32_t i = 0;

	dst_queues->n_pkts_out = 0;
	while (dst_queues->n_pkts_out < num_pkts &&
		i < dst_queues->num_queues) {

		if (rte_bitmap_get(dst_queues->bmp, dst_queues->cur_queue) != 0)
			__dst_dequeue(dst_queues,
				num_pkts - dst_queues->n_pkts_out);

		dst_queues->cur_queue = (dst_queues->cur_queue + 1) &
					(dst_queues->num_queues - 1);
		i++;
	}

        return dst_queues->n_pkts_out;
}

void
dst_thread(struct gk_data *gk, struct dst_queues *dst_queues)
{
	struct rte_mbuf *en_mbufs[gk->qos_enqueue_size];
	struct rte_mbuf *de_mbufs[gk->qos_dequeue_size];

	dst_queues->pkts_out = de_mbufs;

	while (1) {
		uint32_t nb_pkt;
		int ret;

		/*
		 * XXX Only dequeues when at least gk->qos_enqueue_size
		 * packets are available. Use *_burst() to dequeue
		 * up to a given number.
		 */
		ret = rte_ring_sc_dequeue_bulk(gk->dst_rx_ring,
			(void **)en_mbufs, gk->qos_enqueue_size);
		if (likely(ret == 0)) {
			/* XXX Maintain stats. */
			nb_pkt = dst_enqueue(dst_queues, en_mbufs,
				gk->qos_enqueue_size);
			(void)nb_pkt;
		}

		dst_dequeue(dst_queues, gk->qos_dequeue_size);
		send_burst(gk, dst_queues);
	}
}

void
rx_thread(struct gk_data *gk)
{
	uint32_t nb_rx;
	struct rte_mbuf *rx_mbufs[gk->rx_burst_size] __rte_cache_aligned;

	struct rte_mbuf *req_mbufs[gk->rx_burst_size] __rte_cache_aligned;
	struct rte_mbuf *dst_mbufs[gk->rx_burst_size] __rte_cache_aligned;
	uint32_t nb_req = 0;
	uint32_t nb_dst = 0;

	uint32_t type;
	uint32_t queue;

	while (1) {
		nb_rx = rte_eth_rx_burst(gk->rx_port, gk->rx_queue,
			rx_mbufs, gk->rx_burst_size);

		if (likely(nb_rx != 0)) {
			uint32_t i;

			for (i = 0; i < nb_rx; i++) {
				get_pkt_sched(rx_mbufs[i], &type, &queue);

				if (type == GK_REQ_PKT)
					req_mbufs[nb_req++] = rx_mbufs[i];
				else if (type == GK_CAP_PKT)
					dst_mbufs[nb_dst++] = rx_mbufs[i];	
				else
					rte_pktmbuf_free(rx_mbufs[i]);
			}

			/* Enqueue request packets for further processing. */
			if (unlikely(rte_ring_sp_enqueue_bulk(gk->req_rx_ring,
				(void **)req_mbufs, nb_req) != 0)) {
				for (i = 0; i < nb_req; i++)
					rte_pktmbuf_free(req_mbufs[i]);
			}

			/* Enqueue priority packets for further processing. */
			if (unlikely(rte_ring_sp_enqueue_bulk(gk->dst_rx_ring,
				(void **)dst_mbufs, nb_dst) != 0)) {
				for (i = 0; i < nb_dst; i++)
					rte_pktmbuf_free(dst_mbufs[i]);
			}
		}
	}
}
