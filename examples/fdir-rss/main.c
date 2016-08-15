/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2010-2015 Intel Corporation. All rights reserved.
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
#include <inttypes.h>
#include <rte_eal.h>
#include <rte_ethdev.h>
#include <rte_cycles.h>
#include <rte_lcore.h>
#include <rte_mbuf.h>
#include <arpa/inet.h>

#define RX_RING_SIZE 128
#define TX_RING_SIZE 512

#define NUM_MBUFS 8191
#define MBUF_CACHE_SIZE 250
#define BURST_SIZE 32

#define APP_RETA_SIZE_MAX	(ETH_RSS_RETA_SIZE_512 / RTE_RETA_GROUP_SIZE)
#define APP_NUM_QUEUES		2

#define LCORE_ID_Q0	10
#define LCORE_ID_Q1	11
#define LCORE_ID_Q2	12

static const struct rte_eth_conf port_conf_default = {
	.rxmode = {
		.mq_mode = ETH_MQ_RX_RSS,
		.max_rx_pkt_len = ETHER_MAX_LEN, /* Def max frame length. */
		.split_hdr_size = 0,
		.header_split   = 0, /**< Header Split disabled. */
		.hw_ip_checksum = 0, /**< IP checksum offload disabled. */
		.hw_vlan_filter = 1, /**< VLAN filtering enabled. */
		.hw_vlan_strip  = 1, /**< VLAN strip enabled. */
		.hw_vlan_extend = 0, /**< Extended VLAN disabled. */
		.jumbo_frame    = 0, /**< Jumbo Frame Support disabled. */
		.hw_strip_crc   = 0, /**< CRC stripping by hardware disabled. */
	},

	.rx_adv_conf = {
		.rss_conf = {
			//.rss_key = NULL,
			//.rss_key_len = 40,
			.rss_hf = ETH_RSS_IP,
		},
	},

	.fdir_conf = {
		.mode = RTE_FDIR_MODE_PERFECT,
		.pballoc = RTE_FDIR_PBALLOC_64K,
		.status = RTE_FDIR_REPORT_STATUS,
		.drop_queue = 127,
		.mask = {
			.ipv4_mask = {
				.dst_ip = 0xFFFFFFFF,
				.proto = 0xFF,
			},
			.dst_port_mask = 0xFFFF,
		},
	},
};

#define TCP_BGP_PORT	179
#define KNI_IP_ADDR	"192.168.57.12"

static void
add_fdir_filter(uint8_t port_id)
{
	struct rte_eth_fdir_filter entry;
	uint32_t kni_ip_addr;
	int ret = 0;

	ret = rte_eth_dev_filter_supported(port_id, RTE_ETH_FILTER_FDIR);
	if (ret < 0) {
		printf("flow director is not supported on port %u.\n",
			port_id);
		return;
	}
	memset(&entry, 0, sizeof(struct rte_eth_fdir_filter));

	ret = inet_pton(AF_INET, KNI_IP_ADDR, &kni_ip_addr);
	if (ret == 0) {
		printf("%s is not a valid address in AF_INET\n", KNI_IP_ADDR);
		return;
	} else if (ret == -1) {
		perror("inet_pton");
		return;
	}

	entry.input.flow_type = RTE_ETH_FLOW_NONFRAG_IPV4_TCP;
	entry.input.flow.ip4_flow.dst_ip = kni_ip_addr;
	entry.input.flow.udp4_flow.dst_port = rte_cpu_to_be_16(TCP_BGP_PORT);

	entry.input.flow_ext.is_vf = 0;

	entry.action.behavior = RTE_ETH_FDIR_ACCEPT;
	entry.action.flex_off = 0;  /* Use 0 by default. */
	entry.action.report_status = RTE_ETH_FDIR_REPORT_ID;
	entry.action.rx_queue = 0;

	/* XXX Probably need to increment with every filter. */
	entry.soft_id = 0;

	ret = rte_eth_dev_filter_ctrl(port_id, RTE_ETH_FILTER_FDIR,
		RTE_ETH_FILTER_ADD, &entry);

/*
	ret = rte_eth_dev_filter_ctrl(res->port_id, RTE_ETH_FILTER_FDIR,
		RTE_ETH_FILTER_DELETE, &entry);
	ret = rte_eth_dev_filter_ctrl(res->port_id, RTE_ETH_FILTER_FDIR,
		RTE_ETH_FILTER_UPDATE, &entry);
*/

	if (ret < 0)
		printf("flow director programming error: (%s)\n",
			strerror(-ret));
}

/*
 * Initialises a given port using global settings and with the rx buffers
 * coming from the mbuf_pool passed as parameter
 */
static inline int
port_init(uint8_t port, struct rte_mempool *mbuf_pool)
{
	struct rte_eth_conf port_conf = port_conf_default;
	const uint16_t rx_rings = 3, tx_rings = 1;
	int retval;
	uint16_t q;

	if (port >= rte_eth_dev_count())
		return -1;

	retval = rte_eth_dev_configure(port, rx_rings, tx_rings, &port_conf);
	if (retval != 0)
		return retval;

	for (q = 0; q < rx_rings; q++) {
		retval = rte_eth_rx_queue_setup(port, q, RX_RING_SIZE,
				rte_eth_dev_socket_id(port), NULL, mbuf_pool);
		if (retval < 0)
			return retval;
	}

	for (q = 0; q < tx_rings; q++) {
		retval = rte_eth_tx_queue_setup(port, q, TX_RING_SIZE,
				rte_eth_dev_socket_id(port), NULL);
		if (retval < 0)
			return retval;
	}

	retval  = rte_eth_dev_start(port);
	if (retval < 0)
		return retval;

	struct ether_addr addr;

	rte_eth_macaddr_get(port, &addr);
	printf("Port %u MAC: %02"PRIx8" %02"PRIx8" %02"PRIx8
			" %02"PRIx8" %02"PRIx8" %02"PRIx8"\n",
			(unsigned)port,
			addr.addr_bytes[0], addr.addr_bytes[1],
			addr.addr_bytes[2], addr.addr_bytes[3],
			addr.addr_bytes[4], addr.addr_bytes[5]);

	rte_eth_promiscuous_enable(port);
	return 0;
}

static void
rss_setup(uint8_t portid)
{
	struct rte_eth_dev_info dev_info;
	struct rte_eth_rss_reta_entry64 reta_conf[APP_RETA_SIZE_MAX];
	uint16_t queues[APP_NUM_QUEUES];
	uint32_t i;
	int status;

	/* For this sample application, just assign queues 1 and 2. */
	queues[0] = 1;
	queues[1] = 2;

	/* Get RETA size. */
	memset(&dev_info, 0, sizeof(dev_info));
	rte_eth_dev_info_get(portid, &dev_info);

	if (dev_info.reta_size == 0)
		rte_panic("port %u: RSS setup error (null RETA size)\n",
			portid);

	if (dev_info.reta_size > ETH_RSS_RETA_SIZE_512)
		rte_panic("port %u: RSS setup error (RETA size too big)\n",
			portid);

	/* Setup RETA contents. */
	memset(reta_conf, 0, sizeof(reta_conf));

	for (i = 0; i < dev_info.reta_size; i++)
		reta_conf[i / RTE_RETA_GROUP_SIZE].mask = UINT64_MAX;

	for (i = 0; i < dev_info.reta_size; i++) {
		uint32_t reta_id = i / RTE_RETA_GROUP_SIZE;
		uint32_t reta_pos = i % RTE_RETA_GROUP_SIZE;
		uint32_t rss_qs_pos = i % APP_NUM_QUEUES; 


		printf("queue: %hu\n", queues[rss_qs_pos]);
		reta_conf[reta_id].reta[reta_pos] = queues[rss_qs_pos];
	}

	/* RETA update */
	status = rte_eth_dev_rss_reta_update(portid,
		reta_conf,
		dev_info.reta_size);
	if (status != 0)
		rte_panic("port %u: RSS setup error (RETA update failed)\n",
			portid);
}

static void
rx_packets(uint8_t portid, uint16_t queueid)
{
	printf("lcore listening on queue %hu\n", queueid);
	for (;;) {
		struct rte_mbuf *bufs[BURST_SIZE];
		const uint16_t nb_rx = rte_eth_rx_burst(portid, queueid,
				bufs, BURST_SIZE);
		uint16_t buf;

		if (unlikely(nb_rx == 0))
			continue;

		printf("queue %hu received %hu packets\n", queueid, nb_rx);
		for (buf = 0; buf < nb_rx; buf++) {
			struct rte_mbuf *mbuf = bufs[buf];
			//unsigned int len = rte_pktmbuf_data_len(mbuf);
			//rte_pktmbuf_dump(stdout, mbuf, len);
			rte_pktmbuf_free(mbuf);
		}
	}
}

static int
main_loop(__rte_unused void *arg)
{
	const unsigned lcore_id = rte_lcore_id();
	uint8_t portid = 0;
	uint16_t queueid = 0;

	switch (lcore_id) {
	case LCORE_ID_Q0:
		queueid = 0;
		break;
	case LCORE_ID_Q1:
		queueid = 1;
		break;
	case LCORE_ID_Q2:
		queueid = 2;
		break;
	default:
		return 0;
	}

	rx_packets(portid, queueid);
	return 0;
}

/* Main function, does initialisation and calls the per-lcore functions */
int
main(int argc, char *argv[])
{
	struct rte_mempool *mbuf_pool;
	uint8_t portid = 0;
	unsigned i;

	/* init EAL */
	int ret = rte_eal_init(argc, argv);

	if (ret < 0)
		rte_exit(EXIT_FAILURE, "Error with EAL initialization\n");
	argc -= ret;
	argv += ret;

	mbuf_pool = rte_pktmbuf_pool_create("MBUF_POOL",
		NUM_MBUFS, MBUF_CACHE_SIZE, 0,
		RTE_MBUF_DEFAULT_BUF_SIZE, rte_socket_id());
	if (mbuf_pool == NULL)
		rte_exit(EXIT_FAILURE, "Cannot create mbuf pool\n");

	/* initialize all ports */
	if (port_init(portid, mbuf_pool) != 0)
		rte_exit(EXIT_FAILURE, "Cannot init port %"PRIu8"\n",
				portid);

	add_fdir_filter(portid);
	rss_setup(portid);

	if (rte_lcore_count() != 3)
		printf("\nWARNING: app needs 3 lcores: 10, 11, and 12\n");

	/* Launch per-lcore function on every lcore */
	rte_eal_mp_remote_launch(main_loop, NULL, CALL_MASTER);

	RTE_LCORE_FOREACH_SLAVE(i) {
		if (rte_eal_wait_lcore(i) < 0)
			return -1;
	}

	return 0;
}
