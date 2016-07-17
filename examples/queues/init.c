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

#include <rte_ethdev.h>
#include <rte_malloc.h>
#include <rte_sched.h>
#include <rte_cycles.h>

#include "main.h"

/* Configurable number of RX/TX ring descriptors. */
#define RX_DESC_DEFAULT 128
#define TX_DESC_DEFAULT 256

#define RX_PORT		0
#define TX_PORT		0
#define RX_QUEUE	0
#define TX_QUEUE	0

#define RX_RING_SIZE	(8 * 1024)
#define TX_RING_SIZE	(8 * 1024)

#define QUEUES_MTU (6 + 6 + 4 + 4 + 2 + 1500)
#define QUEUES_FRAME_OVERHEAD	RTE_SCHED_FRAME_OVERHEAD_DEFAULT

/* Number of packets to read and write from and to the NIC. */
#define MAX_PKT_RX_BURST 64
#define MAX_PKT_TX_BURST 64
#define QOS_PKT_ENQUEUE 64
#define QOS_PKT_DEQUEUE 32

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

#define MAX_NAME_LEN 32
#define SYS_CPU_DIR "/sys/devices/system/cpu/cpu%u/topology/"

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

	rx_conf.rx_thresh.pthresh = app_conf->rx_pthresh;
	rx_conf.rx_thresh.hthresh = app_conf->rx_hthresh;
	rx_conf.rx_thresh.wthresh = app_conf->rx_wthresh;
	rx_conf.rx_free_thresh = 32;
	rx_conf.rx_drop_en = 0;

	tx_conf.tx_thresh.pthresh = app_conf->tx_pthresh;
	tx_conf.tx_thresh.hthresh = app_conf->tx_hthresh;
	tx_conf.tx_thresh.wthresh = app_conf->tx_wthresh;
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

static void
app_conf_init(struct app_conf *app_conf, uint32_t rx_burst_size,
	uint8_t rx_port)
{
	char pool_name[MAX_NAME_LEN];

	/*
	 * Create the mbuf pools for each RX port.
	 * XXX When we use multiple queues, we need multiple mbuf pools.
	 * For now, just use one for both requests and priority packets.
	 */
	app_conf->mbuf_pool_size = NB_MBUF;
	snprintf(pool_name, MAX_NAME_LEN, "mbuf_pool");
	app_conf->mbuf_pool = rte_pktmbuf_pool_create(pool_name,
		app_conf->mbuf_pool_size, 4 * rx_burst_size, 0,
		RTE_MBUF_DEFAULT_BUF_SIZE,
		rte_eth_dev_socket_id(rx_port));
	if (app_conf->mbuf_pool == NULL)
		rte_exit(EXIT_FAILURE, "Cannot init mbuf pool\n");

	app_conf->rx_queue_size = RX_DESC_DEFAULT;
	app_conf->tx_queue_size = TX_DESC_DEFAULT;

	app_conf->tx_req_core = TX_REQ_CORE;
	app_conf->tx_pri_core = TX_PRI_CORE;
	app_conf->worker_req_core = WORKER_REQ_CORE;
	app_conf->worker_pri_core = WORKER_PRI_CORE;
	app_conf->rx_core = RX_CORE;

	app_conf->rx_pthresh = RX_PTHRESH;
	app_conf->rx_hthresh = RX_HTHRESH;
	app_conf->rx_wthresh = RX_WTHRESH;

	app_conf->tx_pthresh = TX_PTHRESH;
	app_conf->tx_hthresh = TX_HTHRESH;
	app_conf->tx_wthresh = TX_WTHRESH;
}

static void
queue_conf_init(struct queues_conf *conf, const char *name, unsigned rx_core)
{
	char ring_name[MAX_NAME_LEN];
	uint32_t socket = rte_lcore_to_socket_id(rx_core);

	conf->rx_ring_size = RX_RING_SIZE;
	conf->tx_ring_size = TX_RING_SIZE;

	snprintf(ring_name, MAX_NAME_LEN, "ring-%s-rx", name);
	conf->rx_ring = rte_ring_create(ring_name, conf->rx_ring_size,
		socket, RING_F_SP_ENQ | RING_F_SC_DEQ);

	snprintf(ring_name, MAX_NAME_LEN, "ring-%s-tx", name);
	conf->tx_ring = rte_ring_create(ring_name, conf->tx_ring_size,
		socket, RING_F_SP_ENQ | RING_F_SC_DEQ);

	conf->rx_burst_size = MAX_PKT_RX_BURST;
	conf->qos_enqueue_size = QOS_PKT_ENQUEUE;
	conf->qos_dequeue_size = QOS_PKT_DEQUEUE;
	conf->tx_burst_size = MAX_PKT_TX_BURST;

	conf->m_table = rte_malloc(name,
		sizeof(struct rte_mbuf *) * conf->tx_burst_size,
		RTE_CACHE_LINE_SIZE);
	if (conf->m_table == NULL)
		rte_panic("cannot allocate memory buffer for %s\n", name);

	conf->n_mbufs = 0;

	conf->counter = 0;
	conf->socket = socket;

	conf->rate = 0; // XXX computed?
	conf->mtu = QUEUES_MTU;
	conf->frame_overhead = QUEUES_FRAME_OVERHEAD;

	conf->tb_time = 0; // XXX
	conf->tb_period = 0; // XXX
	conf->tb_credits_per_period = 0; // XXX
	conf->tb_size = 0; // XXX
	conf->tb_credits = 0; // XXX

	/* XXX These could change once we add more queues. */
	conf->rx_queue = RX_QUEUE;
	conf->tx_queue = TX_QUEUE;

	/*
	 * XXX TX should change according to configuration, and RX should
	 * change according to port in main.c.
	 */
	conf->rx_port = RX_PORT;
	conf->tx_port = TX_PORT;
}

int queues_init(struct app_conf *app_conf, struct queues_conf *req_conf,
	struct queues_conf *pri_conf)
{
	if (rte_eth_dev_count() == 0)
		rte_exit(EXIT_FAILURE, "No Ethernet port - bye\n");

	/* XXX Separate mbuf pool and remove these parameters. */
	app_conf_init(app_conf, MAX_PKT_RX_BURST, RX_PORT);

	queue_conf_init(req_conf, "req", app_conf->rx_core);
	queue_conf_init(pri_conf, "pri", app_conf->rx_core);

	/* XXX When we have multiple queues, update this. */
	init_port(app_conf, req_conf->rx_port, app_conf->mbuf_pool);
	init_link(req_conf, pri_conf, req_conf->rx_port);

	RTE_LOG(INFO, APP, "time stamp clock running at %" PRIu64 " Hz\n",
			 rte_get_timer_hz());

	RTE_LOG(INFO, APP, "Ring sizes: NIC RX = %u, Mempool = %d "
		"REQ queue = %u, PRI queue = %u. NIC TX = %u\n",
		app_conf->rx_queue_size, app_conf->mbuf_pool_size,
		req_conf->rx_ring_size, pri_conf->rx_ring_size,
		app_conf->tx_queue_size);

	RTE_LOG(INFO, APP, "Req burst sizes: RX read = %hu, RX write = %hu,\n"
		"Worker read/QoS enqueue = %hu,\n"
		"QoS dequeue = %hu, Worker write = %hu\n",
		req_conf->rx_burst_size, req_conf->qos_enqueue_size,
		req_conf->qos_enqueue_size,
		req_conf->qos_dequeue_size, req_conf->tx_burst_size);

	RTE_LOG(INFO, APP, "Req burst sizes: RX read = %hu, RX write = %hu,\n"
		"Worker read/QoS enqueue = %hu,\n"
		"QoS dequeue = %hu, Worker write = %hu\n",
		pri_conf->rx_burst_size, pri_conf->qos_enqueue_size,
		pri_conf->qos_enqueue_size,
		pri_conf->qos_dequeue_size, pri_conf->tx_burst_size);

	RTE_LOG(INFO, APP, "NIC thresholds RX (p = %hhu, h = %hhu, w = %hhu),"
		"TX (p = %hhu, h = %hhu, w = %hhu)\n",
		app_conf->rx_pthresh, app_conf->rx_hthresh,
		app_conf->rx_wthresh, app_conf->tx_pthresh,
		app_conf->tx_hthresh, app_conf->tx_wthresh);

	return 0;
}
