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

#ifndef _MAIN_H_
#define _MAIN_H_

#include <rte_reciprocal.h>
#include "bitmap.h"

#define RTE_LOGTYPE_APP RTE_LOGTYPE_USER1

#define USE_TX_THREADS	1

#ifndef APP_MAX_LCORE
#define APP_MAX_LCORE 64
#endif

struct gk_data {
	struct rte_mempool *mbuf_pool;

	/* Rings for RX, worker, and TX lcores to communicate if needed. */
	struct rte_ring *req_rx_ring;
	struct rte_ring *req_tx_ring;

	struct rte_ring *dst_rx_ring;
	struct rte_ring *dst_tx_ring;

	/* Array of packets that are going to be transmitted next. */
	struct rte_mbuf **m_table;

	/* Number of packets to be transmitted next. */
	uint32_t n_mbufs;
	/*
	 * The tx thread transmits packets once a threshold is reached,
	 * so this counter is used to make sure that even if the
	 * threshold is not reached, after some timeout the packets are sent.
	 */
	uint32_t counter;
	/* CPU socket ID. */
	int32_t socket;
	/* Output port rate. */
	uint64_t rate;
	/* Frame overhead for Ethernet frames. */
	uint32_t frame_overhead;

	/* Maximum number of packets read from device at once. */
	uint16_t rx_burst_size;
	/* Maximum number of packets written to/read from an RX ring. */
	uint16_t qos_enqueue_size;
	/* Maximum number of packets written to/read from a TX ring. */
	uint16_t qos_dequeue_size;
	/* Maximum number of packets written device at once. */
	uint16_t tx_burst_size;

	uint16_t rx_ring_size;
	uint16_t tx_ring_size;

	uint16_t rx_queue_size;
	uint16_t tx_queue_size;

	uint8_t rx_pthresh;
	uint8_t rx_hthresh;
	uint8_t rx_wthresh;

	uint8_t tx_pthresh;
	uint8_t tx_hthresh;
	uint8_t tx_wthresh;

	uint8_t rx_port;
	uint8_t tx_port;
	uint16_t rx_queue;
	uint16_t tx_queue;

	uint8_t tx_req_core;
	uint8_t tx_dst_core;
	uint8_t wk_req_core;
	uint8_t wk_dst_core;
	uint8_t rx_core;
};

struct gk_conf {
	uint32_t mbuf_pool_size;

	uint32_t frame_overhead;

	uint16_t rx_burst_size;
	uint16_t qos_enqueue_size;
	uint16_t qos_dequeue_size;
	uint16_t tx_burst_size;

	uint16_t rx_ring_size;
	uint16_t tx_ring_size;

	uint16_t rx_queue_size;
	uint16_t tx_queue_size;

	uint8_t rx_pthresh; /**< Ring prefetch threshold. */
	uint8_t rx_hthresh; /**< Ring host threshold. */
	uint8_t rx_wthresh; /**< Ring writeback threshold. */

	uint8_t tx_pthresh; /**< Ring prefetch threshold. */
	uint8_t tx_hthresh; /**< Ring host threshold. */
	uint8_t tx_wthresh; /**< Ring writeback threshold. */
	
	uint8_t rx_port;
	uint8_t tx_port;
	uint16_t rx_queue;
	uint16_t tx_queue;

	uint8_t tx_req_core;
	uint8_t tx_dst_core;
	uint8_t wk_req_core;
	uint8_t wk_dst_core;
	uint8_t rx_core;
};

struct queues_conf {
	/* Token bucket. */
	uint32_t tb_period;
	uint32_t tb_credits_per_period;
	uint32_t tb_size;

	uint32_t mtu;

	/* Queue base calculation */
	uint16_t qsize;
	uint16_t num_queues;
};

struct gk_queue {
	uint16_t qw;
	uint16_t qr;
};

struct dst_queues {
	uint64_t rate;
	uint32_t mtu;
	uint16_t qsize;
	uint16_t num_queues;

	/* Token bucket. */
	uint64_t tb_time;
	uint32_t tb_period;
	uint32_t tb_credits_per_period;
	uint32_t tb_size;
	uint32_t tb_credits;

	/* Current CPU time measured in CPU cyles */
	uint64_t time_cpu_cycles;
	/* Current CPU time measured in bytes */
	uint64_t time_cpu_bytes;
	/* Current NIC TX time measured in bytes */
	uint64_t time;
	/* CPU cycles per byte */
	struct rte_reciprocal inv_cycles_per_byte;

	struct rte_mbuf **pkts_out;
	uint32_t n_pkts_out;

	struct gk_queue *queue;
	uint8_t *bmp_array;
	struct rte_mbuf **queue_array;
	struct rte_bitmap *bmp;
	uint8_t memory[0] __rte_cache_aligned;
} __rte_cache_aligned;

struct req_queue {
	uint64_t rate;
	uint32_t mtu;
	uint16_t qsize;
	uint16_t num_queues;

	/* Token bucket. */
	uint64_t tb_time;
	uint32_t tb_period;
	uint32_t tb_credits_per_period;
	uint32_t tb_size;
	uint32_t tb_credits;

	/* Current CPU time measured in CPU cyles */
	uint64_t time_cpu_cycles;
	/* Current CPU time measured in bytes */
	uint64_t time_cpu_bytes;
	/* Current NIC TX time measured in bytes */
	uint64_t time;
	/* CPU cycles per byte */
	struct rte_reciprocal inv_cycles_per_byte;

	struct rte_mbuf **pkts_out;
	uint32_t n_pkts_out;

	struct gk_queue *queue;
	uint8_t *bmp_array;
	struct rte_mbuf **queue_array;
	struct rte_bitmap *bmp;
	uint8_t memory[0] __rte_cache_aligned;
} __rte_cache_aligned;

int gk_init(struct gk_conf *gk_conf, struct gk_data *gk,
	unsigned rx_burst_size);

int queues_init(struct gk_data *gk, struct queues_conf *req_conf,
	struct queues_conf *dst_conf, struct req_queue **req_queue,
	struct dst_queues **dst_queues);

void rx_thread(struct gk_data *gk);
void req_thread(struct gk_data *gk, struct req_queue *req_queue);
void dst_thread(struct gk_data *gk, struct dst_queues *dst_queues);
void req_tx_thread(struct gk_data *gk, struct req_queue *req_queue);
void dst_tx_thread(struct gk_data *gk, struct dst_queues *dst_queues);

#endif /* _MAIN_H_ */
