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

#include <rte_mbuf.h>
#include <rte_malloc.h>
#include <rte_sched.h>

#include "main.h"

#define RX_PORT		0
#define TX_PORT		0
#define RX_QUEUE	0
#define TX_QUEUE	0

#define RX_RING_SIZE	(8 * 1024)
#define TX_RING_SIZE	(8 * 1024)

#define QUEUES_MTU	(6 + 6 + 4 + 4 + 2 + 1500)
#define QUEUES_FRAME_OVERHEAD	RTE_SCHED_FRAME_OVERHEAD_DEFAULT

/* Number of packets to read and write from and to the NIC. */
#define MAX_PKT_RX_BURST	64
#define MAX_PKT_TX_BURST	64
#define QOS_PKT_ENQUEUE	64
#define QOS_PKT_DEQUEUE	32

static struct queues_conf req_conf = {
	/* These structures will be set by queue_conf_init(). */
	.rx_ring = NULL,
	.tx_ring = NULL,	
	.m_table = NULL,

	.n_mbufs = 0,
	.counter = 0,

	/* The socket will be set by queue_conf_init(). */
	.socket = 0,

	.rate = 0, // XXX
	.mtu = QUEUES_MTU,
	.frame_overhead = QUEUES_FRAME_OVERHEAD,

	.tb_time = 0, // XXX
	.tb_period = 0,
	.tb_credits_per_period = 0,
	.tb_size = 0,
	.tb_credits = 0,

	.rx_burst_size = MAX_PKT_RX_BURST,
	.qos_enqueue_size = QOS_PKT_ENQUEUE,
	.qos_dequeue_size = QOS_PKT_DEQUEUE,
	.tx_burst_size = MAX_PKT_TX_BURST,

	.rx_ring_size = RX_RING_SIZE,
	.tx_ring_size = TX_RING_SIZE,
	.rx_queue = RX_QUEUE,
	.tx_queue = TX_QUEUE,
	.rx_port = RX_PORT,
	.tx_port = TX_PORT,
};

static struct queues_conf pri_conf = {
	/* These structures will be set by queue_conf_init(). */
	.rx_ring = NULL,
	.tx_ring = NULL,	
	.m_table = NULL,

	.n_mbufs = 0,
	.counter = 0,

	/* The socket will be set by queue_conf_init(). */
	.socket = 0,

	.rate = 0, // XXX
	.mtu = QUEUES_MTU,
	.frame_overhead = QUEUES_FRAME_OVERHEAD,

	.tb_time = 0, // XXX
	.tb_period = 0,
	.tb_credits_per_period = 0,
	.tb_size = 0,
	.tb_credits = 0,

	.rx_burst_size = MAX_PKT_RX_BURST,
	.qos_enqueue_size = QOS_PKT_ENQUEUE,
	.qos_dequeue_size = QOS_PKT_DEQUEUE,
	.tx_burst_size = MAX_PKT_TX_BURST,

	.rx_ring_size = RX_RING_SIZE,
	.tx_ring_size = TX_RING_SIZE,
	.rx_queue = RX_QUEUE,
	.tx_queue = TX_QUEUE,
	.rx_port = RX_PORT,
	.tx_port = TX_PORT,
};

/* Configurable number of RX/TX ring descriptors. */
#define RX_DESC_DEFAULT 128
#define TX_DESC_DEFAULT 256

#define TX_REQ_CORE 0
#define TX_PRI_CORE 0
#define WORKER_REQ_CORE 1
#define WORKER_PRI_CORE 1
#define RX_CORE 0

#define NB_MBUF	(2 * 1024)

#define RX_PTHRESH 8 /* Default values of RX prefetch threshold reg. */
#define RX_HTHRESH 8 /* Default values of RX host threshold reg. */
#define RX_WTHRESH 4 /* Default values of RX write-back threshold reg. */

#define TX_PTHRESH 36 /* Default values of TX prefetch threshold reg. */
#define TX_HTHRESH 0  /* Default values of TX host threshold reg. */
#define TX_WTHRESH 0  /* Default values of TX write-back threshold reg. */

static struct app_conf app_conf = {
	/* Mbuf pool will be initialized by app_conf_init(). */
	.mbuf_pool = NULL,

	.mbuf_pool_size = NB_MBUF,
	.rx_queue_size = RX_DESC_DEFAULT,
	.tx_queue_size = TX_DESC_DEFAULT,

	.tx_req_core = TX_REQ_CORE,
	.tx_pri_core = TX_PRI_CORE,
	.worker_req_core = WORKER_REQ_CORE,
	.worker_pri_core = WORKER_PRI_CORE,
	.rx_core = RX_CORE,

	.rx_pthresh = RX_PTHRESH,
	.rx_hthresh = RX_HTHRESH,
	.rx_wthresh = RX_WTHRESH,

	.tx_pthresh = TX_PTHRESH,
	.tx_hthresh = TX_HTHRESH,
	.tx_wthresh = TX_WTHRESH,
};

static int
main_loop(void *arg)
{
	struct app_conf *app_conf = (struct app_conf *)arg;
	uint32_t lcore_id = rte_lcore_id();

	if (lcore_id == app_conf->rx_core) {
		RTE_LOG(INFO, APP, "lcoreid %u reading port %"PRIu8"\n",
			lcore_id, req_conf.rx_port);
		/*
		 * XXX Assume we're using the requests configuration
		 * until we add more threads or more queues.
		 */
		rx_thread(&req_conf);
	}
	if (lcore_id == app_conf->worker_req_core) {
		RTE_LOG(INFO, APP, "lcoreid %u req scheduling\n", lcore_id);
		req_thread(&req_conf);
	}
	if (lcore_id == app_conf->worker_pri_core) {
		RTE_LOG(INFO, APP, "lcoreid %u pri scheduling\n", lcore_id);
		pri_thread(&pri_conf);
	}
	if (lcore_id == app_conf->tx_req_core) {
		req_conf.m_table = rte_malloc("req_table",
			sizeof(struct rte_mbuf *) * req_conf.tx_burst_size,
			RTE_CACHE_LINE_SIZE);
		if (req_conf.m_table == NULL)
			rte_panic("unable to allocate req memory buffer\n");
		RTE_LOG(INFO, APP, "lcoreid %u req writing port %"PRIu8"\n",
			lcore_id, req_conf.tx_port);
		req_tx_thread(&req_conf);
	}
	if (lcore_id == app_conf->tx_pri_core) {
		pri_conf.m_table = rte_malloc("pri_table",
			sizeof(struct rte_mbuf *) * pri_conf.tx_burst_size,
			RTE_CACHE_LINE_SIZE);
		if (pri_conf.m_table == NULL)
			rte_panic("unable to allocate pri memory buffer\n");
		RTE_LOG(INFO, APP, "lcoreid %u pri writing port %"PRIu8"\n",
			lcore_id, pri_conf.tx_port);
		pri_tx_thread(&pri_conf);
	}

	RTE_LOG(INFO, APP, "lcore %u has nothing to do\n", lcore_id);
	return 0;
}

int
main(int argc, char **argv)
{
	int ret;

	ret = rte_eal_init(argc, argv);
	if (ret < 0)
		return -1;

	argc -= ret;
	argv += ret;

	ret = queues_init(&app_conf, &req_conf, &pri_conf);
	if (ret < 0)
		return -1;

	rte_eal_mp_remote_launch(main_loop, &app_conf, CALL_MASTER);
	return 0;
}
