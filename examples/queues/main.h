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

#define RTE_LOGTYPE_APP RTE_LOGTYPE_USER1

#ifndef APP_MAX_LCORE
#define APP_MAX_LCORE 64
#endif

struct app_conf {
	struct rte_mempool *mbuf_pool;

	uint32_t mbuf_pool_size;
	uint16_t rx_queue_size;
	uint16_t tx_queue_size;

	uint8_t tx_req_core;
	uint8_t tx_pri_core;
	uint8_t worker_req_core;
	uint8_t worker_pri_core;
	uint8_t rx_core;
	uint8_t rx_pthresh; /**< Ring prefetch threshold. */
	uint8_t rx_hthresh; /**< Ring host threshold. */
	uint8_t rx_wthresh; /**< Ring writeback threshold. */

	uint8_t tx_pthresh; /**< Ring prefetch threshold. */
	uint8_t tx_hthresh; /**< Ring host threshold. */
	uint8_t tx_wthresh; /**< Ring writeback threshold. */
};

struct queues_conf {
	/* Rings for RX, worker, and TX lcores to communicate if needed. */
	struct rte_ring *rx_ring;
	struct rte_ring *tx_ring;
	
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

	int32_t socket;
	uint32_t rate;

	uint32_t mtu;
	uint32_t frame_overhead;

	/* Token bucket. */
	uint64_t tb_time;

	uint32_t tb_period;
	uint32_t tb_credits_per_period;

	uint32_t tb_size;
	uint32_t tb_credits;

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
	uint16_t rx_queue;
	uint16_t tx_queue;

	uint8_t rx_port;
	uint8_t tx_port;
};

int queues_init(struct app_conf *app_conf, struct queues_conf *req_conf,
	struct queues_conf *pri_conf);

void rx_thread(struct queues_conf *conf);
void req_thread(struct queues_conf *conf);
void pri_thread(struct queues_conf *conf);
void req_tx_thread(struct queues_conf *conf);
void pri_tx_thread(struct queues_conf *conf);

#endif /* _MAIN_H_ */
