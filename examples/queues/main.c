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

#include <stdint.h>
#include <inttypes.h>
#include <rte_eal.h>
#include <rte_ethdev.h>
#include <rte_cycles.h>
#include <rte_lcore.h>

#include "main.h"
#include "dst.h"
#include "req.h"

#define	DEFAULT_TB_PERIOD		10
#define	DEFAULT_TB_CREDITS_PER_PERIOD	500
#define DEFAULT_TB_SIZE			5000

#define DEFAULT_QUEUE_SIZE	256
#define NUM_QUEUES_DST		4096

#define GK_REQ_QUEUE_MAX_LENGTH	512

#define QUEUES_MTU	(6 + 6 + 4 + 4 + 2 + 1500)
#define QUEUES_FRAME_OVERHEAD	RTE_SCHED_FRAME_OVERHEAD_DEFAULT

static struct queues_conf req_conf = {
	.tb_period = DEFAULT_TB_PERIOD,
	.tb_credits_per_period = DEFAULT_TB_CREDITS_PER_PERIOD,
	.tb_size = DEFAULT_TB_SIZE,

	.mtu = QUEUES_MTU,
	.frame_overhead = QUEUES_FRAME_OVERHEAD,

	.qsize = GK_REQ_QUEUE_MAX_LENGTH,
	.num_queues = GK_NUM_REQ_PRIORITIES,
};

static struct queues_conf dst_conf = {
	.tb_period = DEFAULT_TB_PERIOD,
	.tb_credits_per_period = DEFAULT_TB_CREDITS_PER_PERIOD,
	.tb_size = DEFAULT_TB_SIZE,

	.mtu = QUEUES_MTU,
	.frame_overhead = QUEUES_FRAME_OVERHEAD,

	.qsize = DEFAULT_QUEUE_SIZE,
	.num_queues = NUM_QUEUES_DST,
};

/* Number of packets to read and write from and to the NIC. */
#define MAX_PKT_RX_BURST	64
#define QOS_PKT_ENQUEUE		64
#define QOS_PKT_DEQUEUE		64
#define MAX_PKT_TX_BURST	32

#define RX_DESC_DEFAULT 128
#define TX_DESC_DEFAULT 256

#define RX_PTHRESH 8 /* Default values of RX prefetch threshold reg. */
#define RX_HTHRESH 8 /* Default values of RX host threshold reg. */
#define RX_WTHRESH 4 /* Default values of RX write-back threshold reg. */

#define TX_PTHRESH 36 /* Default values of TX prefetch threshold reg. */
#define TX_HTHRESH 0  /* Default values of TX host threshold reg. */
#define TX_WTHRESH 0  /* Default values of TX write-back threshold reg. */

#define RX_PORT		0
#define TX_PORT		0
#define RX_QUEUE	0
#define TX_QUEUE	0

#define WK_REQ_CORE	10
#define WK_DST_CORE	11
#define RX_CORE		12

#define RX_RING_SIZE	512
#define TX_RING_SIZE	512

#define RX_QUEUE_SIZE	128
#define TX_QUEUE_SIZE	512


#define NUM_MBUFS 8191
#define MBUF_CACHE_SIZE 250

static struct gk_conf gk_conf = {
	.mbuf_pool_size = NUM_MBUFS,
	.frame_overhead = QUEUES_FRAME_OVERHEAD,

	.rx_burst_size = MAX_PKT_RX_BURST,
	.qos_enqueue_size = QOS_PKT_ENQUEUE,
	.qos_dequeue_size = QOS_PKT_DEQUEUE,
	.tx_burst_size = MAX_PKT_TX_BURST,

	.rx_ring_size = RX_RING_SIZE,
	.tx_ring_size = TX_RING_SIZE,

	.rx_queue_size = RX_QUEUE_SIZE,
	.tx_queue_size = TX_QUEUE_SIZE,

	.rx_pthresh = RX_PTHRESH,
	.rx_hthresh = RX_HTHRESH,
	.rx_wthresh = RX_WTHRESH,

	.tx_pthresh = TX_PTHRESH,
	.tx_hthresh = TX_HTHRESH,
	.tx_wthresh = TX_WTHRESH,

	.rx_port = RX_PORT,
	.tx_port = TX_PORT,
	.rx_queue = RX_QUEUE,
	.tx_queue = TX_QUEUE,

	.wk_req_core = WK_REQ_CORE,
	.wk_dst_core = WK_DST_CORE,
	.rx_core = RX_CORE,
};

struct gk_data gk;
struct dst_queues *dst_queues;
struct req_queue *req_queue;

static int
main_loop(void *arg)
{
	struct gk_data *gk = (struct gk_data *)arg;
	uint32_t lcore_id = rte_lcore_id();

	if (lcore_id == gk->rx_core) {
		RTE_LOG(INFO, APP, "lcoreid %u reading port %"PRIu8"\n",
			lcore_id, gk->rx_port);
		rx_thread(gk);
	}
	if (lcore_id == gk->wk_req_core) {
		RTE_LOG(INFO, APP, "lcoreid %u req scheduling\n", lcore_id);
		req_thread(gk, req_queue);
	}
	if (lcore_id == gk->wk_dst_core) {
		RTE_LOG(INFO, APP, "lcoreid %u dst scheduling\n", lcore_id);
		dst_thread(gk, dst_queues);
	}

	RTE_LOG(INFO, APP, "lcore %u has nothing to do\n", lcore_id);
	return 0;
}

static const struct rte_eth_conf port_conf_default = {
	.rxmode = { .max_rx_pkt_len = ETHER_MAX_LEN, },
};

static inline int
port_init(struct gk_data *gk)
{
	int ret = rte_eth_dev_configure(gk->rx_port, 1, 1, &port_conf_default);
	if (ret < 0) {
		printf("Cannot configure device: "
			"err=%d, port=%"PRIu8"\n", ret, gk->rx_port);
		return -1;
	}

	ret = rte_eth_rx_queue_setup(gk->rx_port, gk->rx_queue,
		gk->rx_queue_size,
		rte_eth_dev_socket_id(gk->rx_port), NULL, gk->mbuf_pool);
	if (ret < 0) {
		printf("rte_eth_tx_queue_setup: err=%d, port=%"PRIu8"\n",
			ret, gk->rx_port);
		return -1;
	}

	ret = rte_eth_tx_queue_setup(gk->tx_port, gk->tx_queue,
		gk->tx_queue_size,
		rte_eth_dev_socket_id(gk->tx_port), NULL);
	if (ret < 0) {
		printf("rte_eth_tx_queue_setup: err=%d, "
			"port=%"PRIu8" queue=%d\n", ret, gk->tx_port,
			gk->tx_queue);
		return -1;
	}

	ret = rte_eth_dev_start(gk->rx_port);
	if (ret < 0) {
		printf("rte_pmd_port_start: err=%d, port=%"PRIu8"\n",
			ret, gk->rx_port);
		return -1;
	}
	rte_eth_promiscuous_enable(gk->rx_port);

	if (gk->rx_port != gk->tx_port) {
		int ret = rte_eth_dev_start(gk->tx_port);
		if (ret < 0)
			rte_exit(EXIT_FAILURE, "rte_pmd_port_start: "
				"err=%d, port=%"PRIu8"\n", ret, gk->tx_port);
		rte_eth_promiscuous_enable(gk->tx_port);
	}

	return 0;
}

int
main(int argc, char **argv)
{
	/* init EAL */
	int ret = rte_eal_init(argc, argv);
	unsigned i;

	if (ret < 0)
		rte_exit(EXIT_FAILURE, "Error with EAL initialization\n");
	argc -= ret;
	argv += ret;

	ret = gk_init(&gk_conf, &gk, MBUF_CACHE_SIZE / 4);
	if (ret < 0)
		return -1;

	dst_queues = dst_queues_init(&gk, &dst_conf);
	if (dst_queues == NULL)
		return -1;

	req_queue = req_queue_init(&gk, &req_conf);
	if (req_queue == NULL)
		return -1;

	if (port_init(&gk) < 0)
		return -1;

	rte_eal_mp_remote_launch(main_loop, &gk, CALL_MASTER);

	RTE_LCORE_FOREACH_SLAVE(i) {
		if (rte_eal_wait_lcore(i) < 0)
			return -1;
	}

	return 0;
}
