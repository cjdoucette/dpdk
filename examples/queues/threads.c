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
#include "dst.h"
#include "req.h"

void
req_thread(struct gk_data *gk, struct req_queue *req_queue)
{
	struct rte_mbuf *en_mbufs[gk->qos_enqueue_size];
	struct rte_mbuf *de_mbufs[gk->qos_dequeue_size];

	req_queue->pkts_out = de_mbufs;
	req_queue->n_pkts_out = 0;

	while (1) {
		uint32_t nb_pkt;
		int ret;

		/*
		 * XXX Only dequeues when at least gk->qos_enqueue_size
		 * packets are available. Use *_burst() to dequeue
		 * up to a given number.
		 */
		ret = rte_ring_sc_dequeue_bulk(gk->req_rx_ring,
			(void **)en_mbufs, gk->qos_enqueue_size);

		if (likely(ret == 0)) {
			/* XXX Maintain stats. */
			nb_pkt = req_enqueue(req_queue, en_mbufs,
				gk->qos_enqueue_size);
			(void)nb_pkt;
		}

		req_dequeue(req_queue, gk->qos_dequeue_size);
		req_send_burst(gk, req_queue);
	}
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
		dst_send_burst(gk, dst_queues);
	}
}

void
rx_thread(struct gk_data *gk)
{
	uint32_t nb_rx;
	struct rte_mbuf *rx_mbufs[gk->rx_burst_size] __rte_cache_aligned;

	struct rte_mbuf *req_mbufs[gk->rx_burst_size] __rte_cache_aligned;
	struct rte_mbuf *dst_mbufs[gk->rx_burst_size] __rte_cache_aligned;

	uint32_t type;
	uint32_t queue;

	while (1) {
		uint32_t nb_req = 0;
		uint32_t nb_dst = 0;
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
#if 0
			if (unlikely(rte_ring_sp_enqueue_bulk(gk->dst_rx_ring,
				(void **)dst_mbufs, nb_dst) != 0)) {
				for (i = 0; i < nb_dst; i++)
					rte_pktmbuf_free(dst_mbufs[i]);
			}
#endif
			for (i = 0; i < nb_dst; i++)
				rte_pktmbuf_free(dst_mbufs[i]);
		}
	}
}
