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

static const struct rte_eth_conf port_conf_default = {
	.rxmode = {
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

#if 0
#define IPV4_ADDR_TO_UINT(ip_addr, ip) \
do { \
	if ((ip_addr).family == AF_INET) \
		(ip) = (ip_addr).addr.ipv4.s_addr; \
	else { \
		printf("invalid parameter.\n"); \
		return; \
	} \
} while (0)
#endif

static void
add_fdir_filter(uint8_t port_id)
{
	struct rte_eth_fdir_filter entry;
	uint32_t kni_ip_addr;
#if 0
	uint8_t flexbytes[RTE_ETH_FDIR_MAX_FLEXLEN];
	char *end;
	unsigned long vf_id;
#endif
	int ret = 0;

	ret = rte_eth_dev_filter_supported(port_id, RTE_ETH_FILTER_FDIR);
	if (ret < 0) {
		printf("flow director is not supported on port %u.\n",
			port_id);
		return;
	}
#if 0
	memset(flexbytes, 0, sizeof(flexbytes));
#endif
	memset(&entry, 0, sizeof(struct rte_eth_fdir_filter));

#if 0
	if (fdir_conf.mode ==  RTE_FDIR_MODE_PERFECT_MAC_VLAN) {
		if (strcmp(res->mode_value, "MAC-VLAN")) {
			printf("Please set mode to MAC-VLAN.\n");
			return;
		}
	} else if (fdir_conf.mode ==  RTE_FDIR_MODE_PERFECT_TUNNEL) {
		if (strcmp(res->mode_value, "Tunnel")) {
			printf("Please set mode to Tunnel.\n");
			return;
		}
	} else {
		if (strcmp(res->mode_value, "IP")) {
			printf("Please set mode to IP.\n");
			return;
		}
		entry.input.flow_type = str2flowtype(res->flow_type);
	}
#endif

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

//	ret = rte_eth_dev_filter_ctrl(1, RTE_ETH_FILTER_NTUPLE,
//		RTE_ETH_FILTER_ADD, &bgp_filter);

#if 0
	ret = parse_flexbytes(res->flexbytes_value,
					flexbytes,
					RTE_ETH_FDIR_MAX_FLEXLEN);
	if (ret < 0) {
		printf("error: Cannot parse flexbytes input.\n");
		return;
	}

	switch (entry.input.flow_type) {
	case RTE_ETH_FLOW_FRAG_IPV4:
	case RTE_ETH_FLOW_NONFRAG_IPV4_OTHER:
		entry.input.flow.ip4_flow.proto = res->proto_value;
	case RTE_ETH_FLOW_NONFRAG_IPV4_UDP:
	case RTE_ETH_FLOW_NONFRAG_IPV4_TCP:
		IPV4_ADDR_TO_UINT(res->ip_dst,
			entry.input.flow.ip4_flow.dst_ip);
		IPV4_ADDR_TO_UINT(res->ip_src,
			entry.input.flow.ip4_flow.src_ip);
		entry.input.flow.ip4_flow.tos = res->tos_value;
		entry.input.flow.ip4_flow.ttl = res->ttl_value;
		/* need convert to big endian. */
		entry.input.flow.udp4_flow.dst_port =
				rte_cpu_to_be_16(res->port_dst);
		entry.input.flow.udp4_flow.src_port =
				rte_cpu_to_be_16(res->port_src);
		break;
	case RTE_ETH_FLOW_NONFRAG_IPV4_SCTP:
		IPV4_ADDR_TO_UINT(res->ip_dst,
			entry.input.flow.sctp4_flow.ip.dst_ip);
		IPV4_ADDR_TO_UINT(res->ip_src,
			entry.input.flow.sctp4_flow.ip.src_ip);
		entry.input.flow.ip4_flow.tos = res->tos_value;
		entry.input.flow.ip4_flow.ttl = res->ttl_value;
		/* need convert to big endian. */
		entry.input.flow.sctp4_flow.dst_port =
				rte_cpu_to_be_16(res->port_dst);
		entry.input.flow.sctp4_flow.src_port =
				rte_cpu_to_be_16(res->port_src);
		entry.input.flow.sctp4_flow.verify_tag =
				rte_cpu_to_be_32(res->verify_tag_value);
		break;
	case RTE_ETH_FLOW_FRAG_IPV6:
	case RTE_ETH_FLOW_NONFRAG_IPV6_OTHER:
		entry.input.flow.ipv6_flow.proto = res->proto_value;
	case RTE_ETH_FLOW_NONFRAG_IPV6_UDP:
	case RTE_ETH_FLOW_NONFRAG_IPV6_TCP:
		IPV6_ADDR_TO_ARRAY(res->ip_dst,
			entry.input.flow.ipv6_flow.dst_ip);
		IPV6_ADDR_TO_ARRAY(res->ip_src,
			entry.input.flow.ipv6_flow.src_ip);
		entry.input.flow.ipv6_flow.tc = res->tos_value;
		entry.input.flow.ipv6_flow.hop_limits = res->ttl_value;
		/* need convert to big endian. */
		entry.input.flow.udp6_flow.dst_port =
				rte_cpu_to_be_16(res->port_dst);
		entry.input.flow.udp6_flow.src_port =
				rte_cpu_to_be_16(res->port_src);
		break;
	case RTE_ETH_FLOW_NONFRAG_IPV6_SCTP:
		IPV6_ADDR_TO_ARRAY(res->ip_dst,
			entry.input.flow.sctp6_flow.ip.dst_ip);
		IPV6_ADDR_TO_ARRAY(res->ip_src,
			entry.input.flow.sctp6_flow.ip.src_ip);
		entry.input.flow.ipv6_flow.tc = res->tos_value;
		entry.input.flow.ipv6_flow.hop_limits = res->ttl_value;
		/* need convert to big endian. */
		entry.input.flow.sctp6_flow.dst_port =
				rte_cpu_to_be_16(res->port_dst);
		entry.input.flow.sctp6_flow.src_port =
				rte_cpu_to_be_16(res->port_src);
		entry.input.flow.sctp6_flow.verify_tag =
				rte_cpu_to_be_32(res->verify_tag_value);
		break;
	case RTE_ETH_FLOW_L2_PAYLOAD:
		entry.input.flow.l2_flow.ether_type =
			rte_cpu_to_be_16(res->ether_type);
		break;
	default:
		break;
	}

	if (fdir_conf.mode ==  RTE_FDIR_MODE_PERFECT_MAC_VLAN)
		(void)rte_memcpy(&entry.input.flow.mac_vlan_flow.mac_addr,
				 &res->mac_addr,
				 sizeof(struct ether_addr));

	if (fdir_conf.mode ==  RTE_FDIR_MODE_PERFECT_TUNNEL) {
		(void)rte_memcpy(&entry.input.flow.tunnel_flow.mac_addr,
				 &res->mac_addr,
				 sizeof(struct ether_addr));
		entry.input.flow.tunnel_flow.tunnel_type =
			str2fdir_tunneltype(res->tunnel_type);
		entry.input.flow.tunnel_flow.tunnel_id =
			rte_cpu_to_be_32(res->tunnel_id_value);
	}

	(void)rte_memcpy(entry.input.flow_ext.flexbytes,
		   flexbytes,
		   RTE_ETH_FDIR_MAX_FLEXLEN);

	entry.input.flow_ext.vlan_tci = rte_cpu_to_be_16(res->vlan_value);
#endif

	entry.input.flow_ext.is_vf = 0;

	entry.action.behavior = RTE_ETH_FDIR_ACCEPT;
	entry.action.flex_off = 0;  /* Use 0 by default. */
	entry.action.report_status = RTE_ETH_FDIR_REPORT_ID;
	entry.action.rx_queue = 127;

	entry.soft_id = 0;

/*
	if (!strcmp(res->pf_vf, "pf"))
	else if (!strncmp(res->pf_vf, "vf", 2)) {
		struct rte_eth_dev_info dev_info;

		memset(&dev_info, 0, sizeof(dev_info));
		rte_eth_dev_info_get(res->port_id, &dev_info);
		errno = 0;
		vf_id = strtoul(res->pf_vf + 2, &end, 10);
		if (errno != 0 || *end != '\0' || vf_id >= dev_info.max_vfs) {
			printf("invalid parameter %s.\n", res->pf_vf);
			return;
		}
		entry.input.flow_ext.is_vf = 1;
		entry.input.flow_ext.dst_id = (uint16_t)vf_id;
	} else {
		printf("invalid parameter %s.\n", res->pf_vf);
		return;
	}
*/
	ret = rte_eth_dev_filter_ctrl(port_id, RTE_ETH_FILTER_FDIR,
		RTE_ETH_FILTER_ADD, &entry);

/*
	else if (!strcmp(res->ops, "del"))
		ret = rte_eth_dev_filter_ctrl(res->port_id, RTE_ETH_FILTER_FDIR,
					     RTE_ETH_FILTER_DELETE, &entry);
	else
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
	const uint16_t rx_rings = 1, tx_rings = 1;
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

/*
 * Main thread that does the work, reading from INPUT_PORT
 * and writing to OUTPUT_PORT
 */
static  __attribute__((noreturn)) void
lcore_main(void)
{
	uint8_t port = 0;

	if (rte_eth_dev_socket_id(port) > 0 &&
			rte_eth_dev_socket_id(port) !=
					(int)rte_socket_id())
		printf("WARNING, port %u is on remote NUMA node to "
				"polling thread.\n\tPerformance will "
				"not be optimal.\n", port);

	printf("\nCore %u forwarding packets. [Ctrl+C to quit]\n",
			rte_lcore_id());
	for (;;) {
		struct rte_mbuf *bufs[BURST_SIZE];
		const uint16_t nb_rx = rte_eth_rx_burst(port, 0,
				bufs, BURST_SIZE);
		uint16_t buf;

		if (unlikely(nb_rx == 0))
			continue;

		for (buf = 0; buf < nb_rx; buf++) {
			struct rte_mbuf *mbuf = bufs[buf];
			unsigned int len = rte_pktmbuf_data_len(mbuf);
			rte_pktmbuf_dump(stdout, mbuf, len);
			rte_pktmbuf_free(mbuf);
		}
	}
}

/* Main function, does initialisation and calls the per-lcore functions */
int
main(int argc, char *argv[])
{
	struct rte_mempool *mbuf_pool;
	uint8_t portid = 0;

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

	if (rte_lcore_count() > 1)
		printf("\nWARNING: Too much enabled lcores - "
			"App uses only 1 lcore\n");

	/* call lcore_main on master core only */
	lcore_main();
	return 0;
}
