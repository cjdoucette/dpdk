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

#define RX_RING_SIZE 128
#define TX_RING_SIZE 512

#define NUM_MBUFS 8191
#define MBUF_CACHE_SIZE 250
#define BURST_SIZE 64

static const struct rte_eth_conf port_conf_default = {
	.rxmode = { .max_rx_pkt_len = RTE_ETHER_MAX_LEN, },
};

static struct rte_ether_addr s_addr;

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

	struct rte_ether_addr addr;

	rte_eth_macaddr_get(port, &addr);
	printf("Port %u MAC: %02"PRIx8" %02"PRIx8" %02"PRIx8
			" %02"PRIx8" %02"PRIx8" %02"PRIx8"\n",
			(unsigned)port,
			addr.addr_bytes[0], addr.addr_bytes[1],
			addr.addr_bytes[2], addr.addr_bytes[3],
			addr.addr_bytes[4], addr.addr_bytes[5]);
	if (port == 0) {
		memcpy(&s_addr, &addr, sizeof(s_addr));
	}

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
	uint8_t port = 1;
	char buffer[64];
	FILE *f = fopen("/tmp/arp", "r");
	struct rte_ether_addr d_addr;

	if (rte_eth_dev_socket_id(port) > 0 &&
			rte_eth_dev_socket_id(port) !=
					(int)rte_socket_id())
		printf("WARNING, port %u is on remote NUMA node to "
				"polling thread.\n\tPerformance will "
				"not be optimal.\n", port);

	if (fgets(buffer, 64, f) == NULL) {
		printf("WARNING, couldn't read ARP result\n");
	}

	printf("ARP result read: %s\n", buffer);
	fclose(f);

	sscanf(buffer, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
		&d_addr.addr_bytes[0],
		&d_addr.addr_bytes[1],
		&d_addr.addr_bytes[2],
		&d_addr.addr_bytes[3],
		&d_addr.addr_bytes[4],
		&d_addr.addr_bytes[5]);

	for (;;) {
		struct rte_mbuf *bufs[BURST_SIZE];
		struct rte_mbuf *tx_bufs[BURST_SIZE];
		const uint16_t nb_rx = rte_eth_rx_burst(port, 0,
				bufs, BURST_SIZE);
		uint16_t nb_tx = 0;
		struct rte_ether_hdr *eth_hdr;
		struct rte_ipv6_hdr *ip6_hdr;
		struct rte_ipv4_hdr *ip4_hdr;
		struct rte_tcp_hdr *tcp_hdr = NULL;
		struct rte_udp_hdr *udp_hdr = NULL;
		uint16_t buf;

		if (unlikely(nb_rx == 0))
			continue;

		for (buf = 0; buf < nb_rx; buf++) {
			struct rte_mbuf *mbuf = bufs[buf];
		//	unsigned int len = rte_pktmbuf_data_len(mbuf);
	        	eth_hdr = rte_pktmbuf_mtod(mbuf, struct rte_ether_hdr *);
			memcpy(&eth_hdr->d_addr, &d_addr,
				sizeof(eth_hdr->d_addr));
			memcpy(&eth_hdr->s_addr, &s_addr,
				sizeof(eth_hdr->s_addr));
			switch (rte_be_to_cpu_16(eth_hdr->ether_type)) {
			case RTE_ETHER_TYPE_IPV4: {
				uint32_t new_dst_addr;
				uint32_t new_src_addr;
				ip4_hdr = rte_pktmbuf_mtod_offset(mbuf,
					struct rte_ipv4_hdr *,
					sizeof(*eth_hdr));
			       	new_dst_addr = ntohl(ip4_hdr->dst_addr);
				if (ntohl(ip4_hdr->dst_addr) != 0xAC1F0350 ||
						ntohl(ip4_hdr->src_addr) !=
							0xAC1F03C8) {
					rte_pktmbuf_free(mbuf);
					break;
				}
				new_dst_addr &= 0xFFFF0000;
				new_dst_addr |= 0x0000005E;
				ip4_hdr->dst_addr = htonl(new_dst_addr);
			       	new_src_addr = ntohl(ip4_hdr->src_addr);
				new_src_addr &= 0xFFFF0000;
				new_src_addr |= 0x0000012B;
				ip4_hdr->src_addr = htonl(new_src_addr);
				if (ip4_hdr->next_proto_id == IPPROTO_TCP) {
					tcp_hdr = (struct rte_tcp_hdr *)&ip4_hdr[1];
					ip4_hdr->hdr_checksum = 0;
					tcp_hdr->cksum = 0;
					tcp_hdr->cksum = rte_ipv4_udptcp_cksum(ip4_hdr,
						tcp_hdr);
					ip4_hdr->hdr_checksum =
						rte_ipv4_cksum(ip4_hdr);
				} else if (ip4_hdr->next_proto_id == IPPROTO_UDP) {
					udp_hdr = (struct rte_udp_hdr *)&ip4_hdr[1];
					ip4_hdr->hdr_checksum = 0;
					udp_hdr->dgram_cksum = 0;
					udp_hdr->dgram_cksum = rte_ipv4_udptcp_cksum(ip4_hdr,
						udp_hdr);
					ip4_hdr->hdr_checksum =
						rte_ipv4_cksum(ip4_hdr);
				} else {
					ip4_hdr->hdr_checksum = 0;
					ip4_hdr->hdr_checksum =
						rte_ipv4_cksum(ip4_hdr);
				}
				tx_bufs[nb_tx++] = mbuf;
				break;
			}
			case RTE_ETHER_TYPE_IPV6:
				ip6_hdr = rte_pktmbuf_mtod_offset(mbuf,
					struct rte_ipv6_hdr *,
					sizeof(*eth_hdr));

				if (ip6_hdr->src_addr[0] == 0xfe || ip6_hdr->dst_addr[0] == 0xfe) {
					rte_pktmbuf_free(mbuf);
					break;
				}

				ip6_hdr->dst_addr[7] = 0x00;
				ip6_hdr->dst_addr[14] = 0x56;
				ip6_hdr->dst_addr[15] = 0x78;

				ip6_hdr->src_addr[7] = 0x01;
				ip6_hdr->src_addr[14] = 0x14;
				ip6_hdr->src_addr[15] = 0x39;
				if (ip6_hdr->proto == IPPROTO_TCP) {
					tcp_hdr = (struct rte_tcp_hdr *)&ip6_hdr[1];
					tcp_hdr->cksum = 0;
					tcp_hdr->cksum = rte_ipv6_udptcp_cksum(ip6_hdr,
						tcp_hdr);
				} else if (ip6_hdr->proto == IPPROTO_UDP) {
					udp_hdr = (struct rte_udp_hdr *)&ip6_hdr[1];
					udp_hdr->dgram_cksum = 0;
					udp_hdr->dgram_cksum = rte_ipv6_udptcp_cksum(ip6_hdr,
						udp_hdr);
				}
				tx_bufs[nb_tx++] = mbuf;
				break;
			default:
				rte_pktmbuf_free(mbuf);
				break;
			}
		}
		rte_eth_tx_burst(0, 0, tx_bufs, nb_tx);
	}
}

/* Main function, does initialisation and calls the per-lcore functions */
int
main(int argc, char *argv[])
{
	struct rte_mempool *mbuf_pool;
	uint8_t rx_portid = 1;
	uint8_t tx_portid = 0;

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

	if (port_init(tx_portid, mbuf_pool) != 0)
		rte_exit(EXIT_FAILURE, "Cannot init port %"PRIu8"\n",
				tx_portid);

	if (port_init(rx_portid, mbuf_pool) != 0)
		rte_exit(EXIT_FAILURE, "Cannot init port %"PRIu8"\n",
				rx_portid);

	lcore_main();
	return 0;
}
