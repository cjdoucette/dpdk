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
#include <memory.h>
#include <unistd.h>

#include <rte_log.h>
#include <rte_mbuf.h>
#include <rte_debug.h>
#include <rte_ethdev.h>
#include <rte_mempool.h>
#include <rte_cycles.h>
#include <rte_string_fns.h>

#include "main.h"

#define RATE	32000

uint32_t app_numa_mask = 0;

#define MAX_NAME_LEN 32

static const struct rte_eth_conf port_conf = {
	.rxmode = {
		.max_rx_pkt_len = ETHER_MAX_LEN,
		.split_hdr_size = 0,
		.header_split   = 0, /**< Header Split disabled */
		.hw_ip_checksum = 0, /**< IP checksum offload disabled */
		.hw_vlan_filter = 0, /**< VLAN filtering disabled */
		.jumbo_frame    = 0, /**< Jumbo Frame Support disabled */
		.hw_strip_crc   = 0, /**< CRC stripped by hardware */
	},
	.txmode = {
		.mq_mode = ETH_DCB_NONE,
	},
};

#define SYS_CPU_DIR "/sys/devices/system/cpu/cpu%u/topology/"

#if 0
static uint32_t
app_cpu_core_count(void)
{
	int i, len;
	char path[PATH_MAX];
	uint32_t ncores = 0;

	for (i = 0; i < APP_MAX_LCORE; i++) {
		len = snprintf(path, sizeof(path), SYS_CPU_DIR, i);
		if (len <= 0 || (unsigned)len >= sizeof(path))
			continue;

		if (access(path, F_OK) == 0)
			ncores++;
	}

	return ncores;
}
#endif

static int
init_port(struct app_conf *app_conf, uint8_t portid, struct rte_mempool *mp)
{
	int ret;
	struct rte_eth_link link;
	struct rte_eth_rxconf rx_conf;
	struct rte_eth_txconf tx_conf;

	rx_conf.rx_thresh.pthresh = RX_PTHRESH;
	rx_conf.rx_thresh.hthresh = RX_HTHRESH;
	rx_conf.rx_thresh.wthresh = RX_WTHRESH;
	rx_conf.rx_free_thresh = 32;
	rx_conf.rx_drop_en = 0;

	tx_conf.tx_thresh.pthresh = TX_PTHRESH;
	tx_conf.tx_thresh.hthresh = TX_HTHRESH;
	tx_conf.tx_thresh.wthresh = TX_WTHRESH;
	tx_conf.tx_free_thresh = 0;
	tx_conf.tx_rs_thresh = 0;
	tx_conf.txq_flags = ETH_TXQ_FLAGS_NOMULTSEGS | ETH_TXQ_FLAGS_NOOFFLOADS;

	/* init port */
	RTE_LOG(INFO, APP, "Initializing port %"PRIu8"... ", portid);
	fflush(stdout);
	ret = rte_eth_dev_configure(portid, 1, 1, &port_conf);
	if (ret < 0)
		rte_exit(EXIT_FAILURE, "Cannot configure device: "
				"err=%d, port=%"PRIu8"\n", ret, portid);

	/* XXX Initialize one RX queue. This will change. */
	fflush(stdout);
	ret = rte_eth_rx_queue_setup(portid, 0, app_conf->rx_queue_size,
		rte_eth_dev_socket_id(portid), &rx_conf, mp);
	if (ret < 0)
		rte_exit(EXIT_FAILURE, "rte_eth_tx_queue_setup: "
				"err=%d, port=%"PRIu8"\n", ret, portid);

	/* XXX Initialize one TX queue. This will change. */
	fflush(stdout);
	ret = rte_eth_tx_queue_setup(portid, 0, app_conf->tx_queue_size,
		rte_eth_dev_socket_id(portid), &tx_conf);
	if (ret < 0)
		rte_exit(EXIT_FAILURE, "rte_eth_tx_queue_setup: err=%d, "
				"port=%"PRIu8" queue=%d\n", ret, portid, 0);

	/* Start device */
	ret = rte_eth_dev_start(portid);
	if (ret < 0)
		rte_exit(EXIT_FAILURE, "rte_pmd_port_start: "
				"err=%d, port=%"PRIu8"\n", ret, portid);

	printf("done: ");

	/* get link status */
	rte_eth_link_get(portid, &link);
	if (link.link_status) {
		printf(" Link Up - speed %u Mbps - %s\n",
			(uint32_t) link.link_speed,
			(link.link_duplex == ETH_LINK_FULL_DUPLEX) ?
			("full-duplex") : ("half-duplex\n"));
	} else {
		printf(" Link Down\n");
	}
	rte_eth_promiscuous_enable(portid);

	return 0;
}

#if 0
struct rte_sched_gk_port_params port_params = {
	.name = "port_scheduler_0",
	.socket = 0, /* computed */
	.rate = 0, /* computed */
	.mtu = 6 + 6 + 4 + 4 + 2 + 1500,
	.frame_overhead = RTE_SCHED_GK_FRAME_OVERHEAD_DEFAULT,
	.n_subports_per_port = 1,
	.qsize = 256,
};
#endif

static void
init_link(struct queues_conf *req_conf, struct queues_conf *pri_conf,
	uint32_t portid)
{
	struct rte_eth_link link;
	rte_eth_link_get((uint8_t)portid, &link);
	/* XXX Assume requests and priorities come from same port. */
	req_conf->rate = (uint64_t)link.link_speed * 1000 * 1000 / 8;
	pri_conf->rate = (uint64_t)link.link_speed * 1000 * 1000 / 8;
}

int queues_init(struct app_conf *app_conf, struct queues_conf *req_conf,
	struct queues_conf *pri_conf)
{
	uint32_t socket;
	char ring_name[MAX_NAME_LEN];
	char pool_name[MAX_NAME_LEN];

	if (rte_eth_dev_count() == 0)
		rte_exit(EXIT_FAILURE, "No Ethernet port - bye\n");

	/* Initialize request packet flow. */
	socket = rte_lcore_to_socket_id(app_conf->rx_core);

	snprintf(ring_name, MAX_NAME_LEN, "ring-rx-%u", app_conf->rx_core);
	req_conf->rx_ring = rte_ring_create(ring_name, req_conf->ring_size,
		socket, RING_F_SP_ENQ | RING_F_SC_DEQ);

	snprintf(ring_name, MAX_NAME_LEN, "ring-req-%u", app_conf->tx_req_core);
	req_conf->tx_ring = rte_ring_create(ring_name, req_conf->ring_size,
		socket, RING_F_SP_ENQ | RING_F_SC_DEQ);

	snprintf(ring_name, MAX_NAME_LEN, "ring-pri-%u", app_conf->tx_pri_core);
	req_conf->tx_ring = rte_ring_create(ring_name, pri_conf->ring_size,
		socket, RING_F_SP_ENQ | RING_F_SC_DEQ);

	/*
	 * Create the mbuf pools for each RX port.
	 * XXX When we use multiple queues, we need multiple mbuf pools.
	 * For now, just use one for both requests and priority packets.
	 */
	snprintf(pool_name, MAX_NAME_LEN, "mbuf_pool");
	app_conf->mbuf_pool = rte_pktmbuf_pool_create(pool_name,
		app_conf->mbuf_pool_size, req_conf->rx_burst_size * 4, 0,
		RTE_MBUF_DEFAULT_BUF_SIZE,
		rte_eth_dev_socket_id(req_conf->rx_port));
	if (app_conf->mbuf_pool == NULL)
		rte_exit(EXIT_FAILURE, "Cannot init mbuf pool\n");

	/* XXX When we have multiple queues, update this. */
	init_port(app_conf, req_conf->rx_port, app_conf->mbuf_pool);
	init_link(req_conf, pri_conf, req_conf->rx_port);

	RTE_LOG(INFO, APP, "time stamp clock running at %" PRIu64 " Hz\n",
			 rte_get_timer_hz());

	RTE_LOG(INFO, APP, "Ring sizes: NIC RX = %u, Mempool = %d "
		"REQ queue = %u, PRI queue = %u. NIC TX = %u\n",
		app_conf->rx_queue_size, app_conf->mbuf_pool_size,
		req_conf->ring_size, pri_conf->ring_size,
		app_conf->tx_queue_size);

	RTE_LOG(INFO, APP, "Req burst sizes: RX read = %hu, RX write = %hu,\n"
		"             Worker read/QoS enqueue = %hu,\n"
		"             QoS dequeue = %hu, Worker write = %hu\n",
		req_conf->rx_burst_size, req_conf->ring_burst_size,
		req_conf->ring_burst_size,
		req_conf->qos_burst_size, req_conf->tx_burst_size);

	RTE_LOG(INFO, APP, "Req burst sizes: RX read = %hu, RX write = %hu,\n"
		"             Worker read/QoS enqueue = %hu,\n"
		"             QoS dequeue = %hu, Worker write = %hu\n",
		pri_conf->rx_burst_size, pri_conf->ring_burst_size,
		pri_conf->ring_burst_size,
		pri_conf->qos_burst_size, pri_conf->tx_burst_size);

	RTE_LOG(INFO, APP, "NIC thresholds RX (p = %hhu, h = %hhu, w = %hhu),"
				 "TX (p = %hhu, h = %hhu, w = %hhu)\n",
		app_conf->rx_pthresh, app_conf->rx_hthresh,
		app_conf->rx_wthresh, app_conf->tx_pthresh,
		app_conf->tx_hthresh, app_conf->tx_wthresh);

	return 0;
}
