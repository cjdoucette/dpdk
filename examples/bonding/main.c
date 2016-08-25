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

#include "rte_eth_bond.h"

#define NUM_RX_QUEUES	3
#define FILTER_QUEUE	0

#define RTE_RX_DESC_DEFAULT 128
#define RTE_TX_DESC_DEFAULT 512

#define NUM_MBUFS 8191
#define MBUF_CACHE_SIZE 250
#define BURST_SIZE 32

#define APP_RETA_SIZE_MAX	(ETH_RSS_RETA_SIZE_512 / RTE_RETA_GROUP_SIZE)
#define APP_NUM_QUEUES		2

#define LCORE_ID_Q0	10
#define LCORE_ID_Q1	11
#define LCORE_ID_Q2	12

static const struct rte_eth_conf port_conf = {
	.rxmode = {
		.mq_mode = ETH_MQ_RX_RSS,
		.max_rx_pkt_len = ETHER_MAX_LEN, /* Def max frame length. */
		.split_hdr_size = 0,
		.header_split   = 0, /**< Header Split disabled. */
		.hw_ip_checksum = 0, /**< IP checksum offload disabled. */
		.hw_vlan_filter = 0, /**< VLAN filtering disabled. */
		.hw_vlan_strip  = 1, /**< VLAN strip disabled. */
		.hw_vlan_extend = 0, /**< Extended VLAN disabled. */
		.jumbo_frame    = 0, /**< Jumbo Frame Support disabled. */
		.hw_strip_crc   = 0, /**< CRC stripping by hardware disabled. */
	},
	.rx_adv_conf = {
		.rss_conf = {
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

#define PRINT_MAC(addr)		printf("%02"PRIx8":%02"PRIx8":%02"PRIx8 \
		":%02"PRIx8":%02"PRIx8":%02"PRIx8,	\
		addr.addr_bytes[0], addr.addr_bytes[1], addr.addr_bytes[2], \
		addr.addr_bytes[3], addr.addr_bytes[4], addr.addr_bytes[5])

static void
slave_port_init(uint8_t portid, struct rte_mempool *mbuf_pool)
{
	int retval;
	int i;

	if (portid >= rte_eth_dev_count())
		rte_exit(EXIT_FAILURE, "Invalid port\n");

	retval = rte_eth_dev_configure(portid, NUM_RX_QUEUES, 1, &port_conf);
	if (retval != 0)
		rte_exit(EXIT_FAILURE, "port %u: config failed (res=%d)\n",
			portid, retval);

	/* RX setup. */
	for (i = 0; i < NUM_RX_QUEUES; i++) {
		retval = rte_eth_rx_queue_setup(portid, i, RTE_RX_DESC_DEFAULT,
			rte_eth_dev_socket_id(portid), NULL, mbuf_pool);
		if (retval < 0)
			rte_exit(retval,
				"port %u: RX queue %d setup failed (res=%d)",
				portid, i, retval);
	}

	/* TX setup. */
	retval = rte_eth_tx_queue_setup(portid, 0, RTE_TX_DESC_DEFAULT,
		rte_eth_dev_socket_id(portid), NULL);
	if (retval < 0)
		rte_exit(retval, "port %u: TX queue 0 setup failed (res=%d)",
			portid, retval);

	retval = rte_eth_dev_start(portid);
	if (retval < 0)
		rte_exit(retval, "Start port %d failed (res=%d)",
			portid, retval);

	struct ether_addr addr;
	rte_eth_macaddr_get(portid, &addr);
	printf("Port %u MAC: ", (unsigned)portid);
	PRINT_MAC(addr);
	printf("\n");
	rte_eth_promiscuous_enable(portid);
}

static uint8_t
bond_port_init(struct rte_mempool *mbuf_pool)
{
	uint8_t bond_port;
	int retval;
	int i;

	retval = rte_eth_bond_create("bond0", BONDING_MODE_ROUND_ROBIN,
		0 /*SOCKET_ID_ANY*/);
	if (retval < 0)
		rte_exit(EXIT_FAILURE, "Faled to create bond port\n");

	bond_port = (uint8_t)retval;

	retval = rte_eth_dev_configure(bond_port, NUM_RX_QUEUES, 1, &port_conf);
	if (retval != 0)
		rte_exit(EXIT_FAILURE, "port %u: config failed (res=%d)\n",
			bond_port, retval);

	for (i = 0; i < NUM_RX_QUEUES; i++) {
		retval = rte_eth_rx_queue_setup(bond_port, i,
			RTE_RX_DESC_DEFAULT, rte_eth_dev_socket_id(bond_port),
			NULL, mbuf_pool);
		if (retval < 0)
			rte_exit(retval,
				"bport %u: RX queue %d setup failed (res=%d)",
				bond_port, i, retval);
	}

	/* TX setup. */
	retval = rte_eth_tx_queue_setup(bond_port, 0, RTE_TX_DESC_DEFAULT,
		rte_eth_dev_socket_id(bond_port), NULL);
	if (retval < 0)
		rte_exit(retval, "port %u: TX queue 0 setup failed (res=%d)",
			bond_port, retval);

	if (rte_eth_bond_slave_add(bond_port, 0) == -1)
		rte_exit(-1, "adding slave 0 to bond (%u) failed\n",bond_port);
	if (rte_eth_bond_slave_add(bond_port, 1) == -1)
		rte_exit(-1, "adding slave 1 to bond (%u) failed\n",bond_port);

	retval = rte_eth_dev_start(bond_port);;
	if (retval < 0)
		rte_exit(retval, "Start port %d failed (res=%d)",
			bond_port, retval);

	rte_eth_promiscuous_enable(bond_port);

	struct ether_addr addr;
	rte_eth_macaddr_get(bond_port, &addr);
	printf("Bond port %u MAC: ", (unsigned)bond_port);
		PRINT_MAC(addr);
	printf("\n");

	return bond_port;
}

#define TCP_PORT 179
#define IP_ADDR	"192.168.57.12"

static inline int
add_ntuple_filter(uint8_t port)
{
	int ret = 0;
	uint16_t dport = htons(TCP_PORT);
	uint32_t ntuple_ip_addr;

	ret = inet_pton(AF_INET, IP_ADDR, &ntuple_ip_addr);
	if (ret <= 0) {
		if (ret == 0) {
			printf("Error: %s is not in presentation format\n",
				IP_ADDR);
		} else if (ret == -1) {
			perror("inet_pton");
		}
		return ret;
	}

	struct rte_eth_ntuple_filter filter = {
		.flags = RTE_5TUPLE_FLAGS,
		.dst_ip = ntuple_ip_addr, /* Big endian */
		.dst_ip_mask = UINT32_MAX, /* Enable */
		.src_ip = 0,
		.src_ip_mask = 0, /* Disable */
		.dst_port = dport,
		.dst_port_mask = UINT16_MAX, /* Disable */
		.src_port = 0,
		.src_port_mask = 0, /* Disable */
		.proto = 0,
		.proto_mask = 0, /* Disable */
		.tcp_flags = 0,
		.priority = 1, /* Lowest */
		.queue = FILTER_QUEUE,
	};

	return rte_eth_dev_filter_ctrl(port,
			RTE_ETH_FILTER_NTUPLE,
			RTE_ETH_FILTER_ADD,
			&filter);
}

static inline void
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

	ret = inet_pton(AF_INET, IP_ADDR, &kni_ip_addr);
	if (ret == 0) {
		printf("%s is not a valid address in AF_INET\n", IP_ADDR);
		return;
	} else if (ret == -1) {
		perror("inet_pton");
		return;
	}

	entry.input.flow_type = RTE_ETH_FLOW_NONFRAG_IPV4_TCP;
	entry.input.flow.ip4_flow.dst_ip = kni_ip_addr;
	entry.input.flow.udp4_flow.dst_port = rte_cpu_to_be_16(TCP_PORT);

	entry.input.flow_ext.is_vf = 0;

	entry.action.behavior = RTE_ETH_FDIR_ACCEPT;
	entry.action.flex_off = 0;  /* Use 0 by default. */
	entry.action.report_status = RTE_ETH_FDIR_REPORT_ID;
	entry.action.rx_queue = FILTER_QUEUE;

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

static  __attribute__((noreturn)) void
lcore_main(void)
{
	printf("\nCore %u forwarding packets. [Ctrl+C to quit]\n",
			rte_lcore_id());
	while (1) {
		uint8_t port = 2;
		uint16_t queue;
		for (queue = 0; queue < NUM_RX_QUEUES; queue++) {
			struct rte_mbuf *bufs[BURST_SIZE];
			const uint16_t nb_rx = rte_eth_rx_burst(port,
				queue, bufs, BURST_SIZE);
			uint16_t buf;

			if (unlikely(nb_rx == 0))
				continue;

			printf("port %hhu queue %hu rcvd %hu pkts\n",
				port, queue, nb_rx);
			for (buf = 0; buf < nb_rx; buf++) {
				struct rte_mbuf *mbuf = bufs[buf];
				unsigned int len;

				len = rte_pktmbuf_data_len(mbuf);
				rte_pktmbuf_dump(stdout, mbuf, len);
				rte_pktmbuf_free(mbuf);
			}
		}
	}
}

/* Main function, does initialisation and calls the per-lcore functions */
int
main(int argc, char *argv[])
{
	struct rte_mempool *mbuf_pool;
	uint8_t bond_port = 0;

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
	slave_port_init(0, mbuf_pool);
	slave_port_init(1, mbuf_pool);
	bond_port = bond_port_init(mbuf_pool);
	rss_setup(bond_port);
//	add_fdir_filter(bond_port);
	add_ntuple_filter(bond_port);

	lcore_main();
	return 0;
}
