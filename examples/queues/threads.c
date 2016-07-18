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

static inline int
get_pkt_sched(struct rte_mbuf *m, uint32_t *subport, uint32_t *queue)
{
	uint16_t *pdata = rte_pktmbuf_mtod(m, uint16_t *);
	*subport = *pdata % 2;
	*queue = 0;
	return 0;
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

	uint32_t subport;
	uint32_t queue;

	while (1) {
		/* For now, assume only one queue is used. */
		nb_rx = rte_eth_rx_burst(gk->rx_port, gk->rx_queue,
			rx_mbufs, gk->rx_burst_size);

		if (likely(nb_rx != 0)) {
			uint32_t i;

			for (i = 0; i < nb_rx; i++) {
				get_pkt_sched(rx_mbufs[i], &subport, &queue);

				if (subport == 0)
					req_mbufs[nb_req++] = rx_mbufs[i];
				else
					dst_mbufs[nb_dst++] = rx_mbufs[i];	
				/*
				 * XXX Probably still needed for queue,
				 * but could be done later.
				 */
				//rte_sched_gk_port_pkt_write(rx_mbufs[i],
				//	subport, queue);
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

/*
 * Send the packet to an output interface
 * For performance reasons this function returns the number
 * of packets dropped, not sent, so 0 means that all packets
 * were sent successfully.
 */
#if 0
static inline void
app_send_burst(struct app_conf *app_conf, struct queues_conf *conf)
{
	struct rte_mbuf **mbufs;
	uint32_t n, ret;

	mbufs = (struct rte_mbuf **)conf->m_table;
	n = conf->n_mbufs;

	do {
		ret = rte_eth_tx_burst(app_conf->tx_port, app_conf->tx_queue,
			mbufs, (uint16_t)n);

		/* we cannot drop the packets, so re-send */
		/* update number of packets to be sent */
		n -= ret;
		mbufs = (struct rte_mbuf **)&mbufs[ret];
	} while (n);
}
#endif

#if 0
/* Send the packet to an output interface */
static void
app_send_packets(struct app_conf *app_conf, struct queues_conf *conf,
	struct rte_mbuf **mbufs, uint32_t nb_pkt)
{
	uint32_t i, len;

	len = conf->n_mbufs;
	for(i = 0; i < nb_pkt; i++) {
		conf->m_table[len] = mbufs[i];
		len++;
		/* enough pkts to be sent */
		if (unlikely(len == conf->tx_burst_size)) {
			conf->n_mbufs = len;
			app_send_burst(app_conf, conf);
			len = 0;
		}
	}

	conf->n_mbufs = len;
}
#endif

void
req_tx_thread(struct gk_data *gk,
	__attribute__((unused)) struct req_queue *req_queue)
{
	struct rte_mbuf *mbufs[gk->qos_dequeue_size];
	int ret;
	const uint64_t drain_tsc = (rte_get_tsc_hz() + US_PER_S - 1) /
		US_PER_S * BURST_TX_DRAIN_US;

	while (1) {
		ret = rte_ring_sc_dequeue_bulk(gk->req_tx_ring,
			(void **)mbufs, gk->qos_dequeue_size);
		if (likely(ret == 0)) {
			/* XXX Shouldn't we send as many as we dequeued? */
#if 0
			req_send_packets(gk, req_queue, mbufs,
				gk->qos_dequeue_size);
#endif
			gk->counter = 0; /* reset empty read loop counter */
		}

		gk->counter++;

		/* Drain ring and TX queues after some timeout. */
		if (unlikely(gk->counter > drain_tsc)) {
			/* Check if there are packets left to be transmitted. */
			if (gk->n_mbufs != 0) {
#if 0
				app_send_burst(gk, req_queue);
#endif
				gk->n_mbufs = 0;
			}
			gk->counter = 0;
		}
	}
}

void
dst_tx_thread(struct gk_data *gk,
	__attribute__((unused)) struct dst_queues *dst_queues)
{
	struct rte_mbuf *mbufs[gk->qos_dequeue_size];
	int ret;
	const uint64_t drain_tsc = (rte_get_tsc_hz() + US_PER_S - 1) /
		US_PER_S * BURST_TX_DRAIN_US;

	while (1) {
		ret = rte_ring_sc_dequeue_bulk(gk->dst_tx_ring,
			(void **)mbufs, gk->qos_dequeue_size);
		if (likely(ret == 0)) {
			/* XXX Shouldn't we send as many as we dequeued? */
#if 0
			app_send_packets(gk, dst_queues, mbufs,
				gk->qos_dequeue_size);
#endif
			gk->counter = 0; /* reset empty read loop counter */
		}

		gk->counter++;

		/* Drain ring and TX queues after some timeout. */
		if (unlikely(gk->counter > drain_tsc)) {
			/* Check if there are packets left to be transmitted. */
			if (gk->n_mbufs != 0) {
				//app_send_burst(gk, dst_queues);
				gk->n_mbufs = 0;
			}
			gk->counter = 0;
		}
	}
}

#if 0
static inline uint32_t
enqueue_qptrs_prefetch0(struct queues_conf *conf, struct rte_mbuf *pkt)
{
	struct rte_sched_queue *q;
	uint32_t subport, pipe, traffic_class, queue, qindex;

	rte_sched_port_pkt_read_tree_path(pkt, &subport, &pipe, &traffic_class, &queue);

	qindex = rte_sched_port_qindex(port, subport, pipe, traffic_class, queue);
	q = port->queue + qindex;
	rte_prefetch0(q);
	return qindex;
	return 0;
}
#endif

#if 0
static inline struct rte_mbuf **
qbase(struct queues_conf *conf, uint32_t qindex)
{
	uint32_t pindex = qindex >> 4;
	uint32_t qpos = qindex & 0xF;

	return (port->queue_array + pindex *
		port->qsize_sum + port->qsize_add[qpos]);
	return NULL;
}
#endif

#if 0
static inline void
enqueue_qwa_prefetch0(struct queues_conf *conf, uint32_t qindex,
	struct rte_mbuf **qbase)
{
	struct rte_sched_queue *q;
	struct rte_mbuf **q_qw;
	uint16_t qsize;

	q = port->queue + qindex;
	qsize = rte_sched_port_qsize(port, qindex);
	q_qw = qbase + (q->qw & (qsize - 1));

	rte_prefetch0(q_qw);
	rte_bitmap_prefetch0(port->bmp, qindex);
}
#endif

#if 0
static inline int
enqueue_qwa(struct queues_conf *port, uint32_t qindex,
	struct rte_mbuf **qbase, struct rte_mbuf *pkt)
{
	struct rte_sched_queue *q;
	uint16_t qsize;
	uint16_t qlen;

	q = port->queue + qindex;
	qsize = rte_sched_port_qsize(port, qindex);
	qlen = q->qw - q->qr;

	/* Drop the packet (and update drop stats) when queue is full */
	if (unlikely(rte_sched_port_red_drop(port, pkt, qindex, qlen) ||
		     (qlen >= qsize))) {
		rte_pktmbuf_free(pkt);
#ifdef RTE_SCHED_COLLECT_STATS
		rte_sched_port_update_subport_stats_on_drop(port, qindex, pkt,
							    qlen < qsize);
		rte_sched_port_update_queue_stats_on_drop(port, qindex, pkt,
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
#ifdef RTE_SCHED_COLLECT_STATS
	rte_sched_port_update_subport_stats(port, qindex, pkt);
	rte_sched_port_update_queue_stats(port, qindex, pkt);
#endif

	return 1;
	return 0;
}
#endif

#if 0
static int
enqueue(struct queues_conf *conf, struct rte_mbuf **pkts, uint32_t n_pkts)
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
			q[i] = enqueue_qptrs_prefetch0(conf, pkts[i]);

		/* Prefetch the write pointer location of each queue */
		for (i = 0; i < n_pkts; i++) {
			q_base[i] = qbase(conf, q[i]);
			enqueue_qwa_prefetch0(conf, q[i], q_base[i]);
		}

		/* Write each packet to its queue */
		for (i = 0; i < n_pkts; i++)
			result += enqueue_qwa(conf, q[i], q_base[i], pkts[i]);

		return result;
	}

	/* Feed the first 3 stages of the pipeline (6 packets needed) */
	pkt20 = pkts[0];
	pkt21 = pkts[1];
	rte_prefetch0(pkt20);
	rte_prefetch0(pkt21);

	pkt10 = pkts[2];
	pkt11 = pkts[3];
	rte_prefetch0(pkt10);
	rte_prefetch0(pkt11);

	q20 = enqueue_qptrs_prefetch0(conf, pkt20);
	q21 =_enqueue_qptrs_prefetch0(conf, pkt21);

	pkt00 = pkts[4];
	pkt01 = pkts[5];
	rte_prefetch0(pkt00);
	rte_prefetch0(pkt01);

	q10 = enqueue_qptrs_prefetch0(conf, pkt10);
	q11 = enqueue_qptrs_prefetch0(conf, pkt11);

	q20_base = qbase(conf, q20);
	q21_base = qbase(conf, q21);
	enqueue_qwa_prefetch0(conf, q20, q20_base);
	enqueue_qwa_prefetch0(conf, q21, q21_base);

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
		q10 = enqueue_qptrs_prefetch0(conf, pkt10);
		q11 = enqueue_qptrs_prefetch0(conf, pkt11);

		/* Stage 2: Prefetch queue write location */
		q20_base = qbase(conf, q20);
		q21_base = qbase(conf, q21);
		enqueue_qwa_prefetch0(conf, q20, q20_base);
		enqueue_qwa_prefetch0(conf, q21, q21_base);

		/* Stage 3: Write packet to queue and activate queue */
		r30 = enqueue_qwa(conf, q30, q30_base, pkt30);
		r31 = enqueue_qwa(conf, q31, q31_base, pkt31);
		result += r30 + r31;
	}

	/*
	 * Drain the pipeline (exactly 6 packets).
	 * Handle the last packet in the case
	 * of an odd number of input packets.
	 */
	pkt_last = pkts[n_pkts - 1];
	rte_prefetch0(pkt_last);

	q00 = enqueue_qptrs_prefetch0(conf, pkt00);
	q01 = enqueue_qptrs_prefetch0(conf, pkt01);

	q10_base = qbase(conf, q10);
	q11_base = qbase(conf, q11);
	enqueue_qwa_prefetch0(conf, q10, q10_base);
	enqueue_qwa_prefetch0(conf, q11, q11_base);

	r20 = enqueue_qwa(conf, q20, q20_base, pkt20);
	r21 = enqueue_qwa(conf, q21, q21_base, pkt21);
	result += r20 + r21;

	q_last = enqueue_qptrs_prefetch0(conf, pkt_last);

	q00_base = qbase(conf, q00);
	q01_base = qbase(conf, q01);
	enqueue_qwa_prefetch0(conf, q00, q00_base);
	enqueue_qwa_prefetch0(conf, q01, q01_base);

	r10 = enqueue_qwa(conf, q10, q10_base, pkt10);
	r11 = enqueue_qwa(conf, q11, q11_base, pkt11);
	result += r10 + r11;

	q_last_base = qbase(conf, q_last);
	enqueue_qwa_prefetch0(conf, q_last, q_last_base);

	r00 = enqueue_qwa(conf, q00, q00_base, pkt00);
	r01 = enqueue_qwa(conf, q01, q01_base, pkt01);
	result += r00 + r01;

	if (n_pkts & 1) {
		r_last = enqueue_qwa(conf, q_last, q_last_base, pkt_last);
		result += r_last;
	}

	return result;
}
#endif

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
		if (likely(nb_pkt > 0))
			while (rte_ring_sp_enqueue_bulk(gk->req_tx_ring,
				(void **)mbufs, nb_pkt) != 0);
	}
}

static int
dst_enqueue(__attribute__((unused)) struct dst_queues *dst_queues,
	__attribute__((unused)) struct rte_mbuf **mbufs,
	__attribute__((unused)) uint32_t num_pkts)
{
	return 0;
}

static int
dst_dequeue(__attribute__((unused)) struct dst_queues *dst_queues,
	__attribute__((unused)) struct rte_mbuf **mbufs,
	__attribute__((unused)) uint32_t num_pkts)
{
	return 0;
}

void
dst_thread(struct gk_data *gk, struct dst_queues *dst_queues)
{
	struct rte_mbuf *mbufs[gk->qos_enqueue_size];

	while (1) {
		uint32_t nb_pkt;
		int ret;

		ret = rte_ring_sc_dequeue_bulk(gk->dst_rx_ring,
			(void **)mbufs, gk->qos_enqueue_size);
		if (likely(ret == 0))
			/* XXX Catch return value and maintain stats. */
			dst_enqueue(dst_queues, mbufs, gk->qos_enqueue_size);

		nb_pkt = dst_dequeue(dst_queues, mbufs, gk->qos_dequeue_size);
		if (likely(nb_pkt > 0))
			while (rte_ring_sp_enqueue_bulk(gk->dst_tx_ring,
				(void **)mbufs, nb_pkt) != 0);
	}
}
