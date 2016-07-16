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

#include <stdint.h>

#include <rte_log.h>
#include <rte_mbuf.h>
#include <rte_malloc.h>
#include <rte_cycles.h>
#include <rte_ethdev.h>
#include <rte_memcpy.h>
#include <rte_byteorder.h>
#include <rte_branch_prediction.h>

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
rx_thread(struct queues_conf *conf)
{
	uint32_t nb_rx;
	struct rte_mbuf *rx_mbufs[conf->rx_burst_size] __rte_cache_aligned;

	struct rte_mbuf *req_mbufs[conf->rx_burst_size] __rte_cache_aligned;
	struct rte_mbuf *pri_mbufs[conf->rx_burst_size] __rte_cache_aligned;
	uint32_t nb_req = 0;
	uint32_t nb_pri = 0;

	uint32_t subport;
	uint32_t queue;

	while (1) {
		/* For now, assume only one queue is used. */
		nb_rx = rte_eth_rx_burst(conf->rx_port, conf->rx_queue,
			rx_mbufs, conf->rx_burst_size);

		if (likely(nb_rx != 0)) {
			uint32_t i;

			for (i = 0; i < nb_rx; i++) {
				get_pkt_sched(rx_mbufs[i], &subport, &queue);

				if (subport == 0)
					req_mbufs[nb_req++] = rx_mbufs[i];
				else
					pri_mbufs[nb_pri++] = rx_mbufs[i];	
				/*
				 * XXX Probably still needed for queue,
				 * but could be done later.
				 */
				//rte_sched_gk_port_pkt_write(rx_mbufs[i],
				//	subport, queue);
			}

			/* Enqueue request packets for further processing. */
			if (unlikely(rte_ring_sp_enqueue_bulk(conf->rx_ring,
				(void **)req_mbufs, nb_req) != 0)) {
				for (i = 0; i < nb_req; i++)
					rte_pktmbuf_free(req_mbufs[i]);
			}

			/* Enqueue priority packets for further processing. */
			if (unlikely(rte_ring_sp_enqueue_bulk(conf->rx_ring,
				(void **)pri_mbufs, nb_pri) != 0)) {
				for (i = 0; i < nb_pri; i++)
					rte_pktmbuf_free(pri_mbufs[i]);
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
static inline void
app_send_burst(struct queues_conf *conf)
{
	struct rte_mbuf **mbufs;
	uint32_t n, ret;

	mbufs = (struct rte_mbuf **)conf->m_table;
	n = conf->n_mbufs;

	do {
		ret = rte_eth_tx_burst(conf->tx_port, conf->tx_queue, mbufs,
			(uint16_t)n);

		/* we cannot drop the packets, so re-send */
		/* update number of packets to be sent */
		n -= ret;
		mbufs = (struct rte_mbuf **)&mbufs[ret];
	} while (n);
}


/* Send the packet to an output interface */
static void
app_send_packets(struct queues_conf *conf, struct rte_mbuf **mbufs,
	uint32_t nb_pkt)
{
	uint32_t i, len;

	len = conf->n_mbufs;
	for(i = 0; i < nb_pkt; i++) {
		conf->m_table[len] = mbufs[i];
		len++;
		/* enough pkts to be sent */
		if (unlikely(len == conf->tx_burst_size)) {
			conf->n_mbufs = len;
			app_send_burst(conf);
			len = 0;
		}
	}

	conf->n_mbufs = len;
}

void
req_tx_thread(struct queues_conf *conf)
{
	struct rte_mbuf *mbufs[conf->qos_dequeue_size];
	int ret;
	const uint64_t drain_tsc = (rte_get_tsc_hz() + US_PER_S - 1) /
				   US_PER_S * BURST_TX_DRAIN_US;

	while (1) {
		ret = rte_ring_sc_dequeue_bulk(conf->tx_ring,
			(void **)mbufs, conf->qos_dequeue_size);
		if (likely(ret == 0)) {
			/* XXX Shouldn't we send as many as we dequeued? */
			app_send_packets(conf, mbufs, conf->qos_dequeue_size);
			conf->counter = 0; /* reset empty read loop counter */
		}

		conf->counter++;

		/* Drain ring and TX queues after some timeout. */
		if (unlikely(conf->counter > drain_tsc)) {
			/* Check if there are packets left to be transmitted. */
			if (conf->n_mbufs != 0) {
				app_send_burst(conf);

				conf->n_mbufs = 0;
			}
			conf->counter = 0;
		}
	}
}

void
pri_tx_thread(struct queues_conf *conf)
{
	struct rte_mbuf *mbufs[conf->qos_dequeue_size];
	int ret;
	const uint64_t drain_tsc = (rte_get_tsc_hz() + US_PER_S - 1) /
				   US_PER_S * BURST_TX_DRAIN_US;

	while (1) {
		ret = rte_ring_sc_dequeue_bulk(conf->tx_ring,
			(void **)mbufs, conf->qos_dequeue_size);
		if (likely(ret == 0)) {
			/* XXX Shouldn't we send as many as we dequeued? */
			app_send_packets(conf, mbufs, conf->qos_dequeue_size);
			conf->counter = 0; /* reset empty read loop counter */
		}

		conf->counter++;

		/* Drain ring and TX queues after some timeout. */
		if (unlikely(conf->counter > drain_tsc)) {
			/* Check if there are packets left to be transmitted. */
			if (conf->n_mbufs != 0) {
				app_send_burst(conf);
				conf->n_mbufs = 0;
			}
			conf->counter = 0;
		}
	}
}

static int
req_enqueue(__attribute__((unused)) struct rte_mbuf **mbufs,
	    __attribute__((unused)) uint32_t num_pkts)
{
	return 0;
}

static int
req_dequeue(__attribute__((unused)) struct rte_mbuf **mbufs,
            __attribute__((unused)) uint32_t num_pkts)
{
	return 0;
}

void
req_thread(struct queues_conf *conf)
{
	struct rte_mbuf *mbufs[conf->qos_enqueue_size];

	while (1) {
		uint32_t nb_pkt;
		int ret;

		ret = rte_ring_sc_dequeue_bulk(conf->rx_ring,
			(void **)mbufs, conf->qos_enqueue_size);
		if (likely(ret == 0))
			/* XXX Catch return value and maintain stats. */
			req_enqueue(mbufs, conf->qos_enqueue_size);

		nb_pkt = req_dequeue(mbufs, conf->qos_dequeue_size);
		if (likely(nb_pkt > 0))
			while (rte_ring_sp_enqueue_bulk(conf->tx_ring,
				(void **)mbufs, nb_pkt) != 0);
	}
}

static int
pri_enqueue(__attribute__((unused)) struct rte_mbuf **mbufs,
	    __attribute__((unused)) uint32_t num_pkts)
{
	return 0;
}

static int
pri_dequeue(__attribute__((unused)) struct rte_mbuf **mbufs,
	    __attribute__((unused)) uint32_t num_pkts)
{
	return 0;
}

void
pri_thread(struct queues_conf *conf)
{
	struct rte_mbuf *mbufs[conf->qos_enqueue_size];

	while (1) {
		uint32_t nb_pkt;
		int ret;

		ret = rte_ring_sc_dequeue_bulk(conf->rx_ring,
			(void **)mbufs, conf->qos_enqueue_size);
		if (likely(ret == 0))
			/* XXX Catch return value and maintain stats. */
			pri_enqueue(mbufs, conf->qos_enqueue_size);

		nb_pkt = pri_dequeue(mbufs, conf->qos_dequeue_size);
		if (likely(nb_pkt > 0))
			while (rte_ring_sp_enqueue_bulk(conf->tx_ring,
				(void **)mbufs, nb_pkt) != 0);
	}
}
