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
#include <stdbool.h>
#include <inttypes.h>
#include <rte_arp.h>
#include <rte_eal.h>
#include <rte_ethdev.h>
#include <rte_cycles.h>
#include <rte_lcore.h>
#include <rte_mbuf.h>
#include <rte_hash.h>

#define RX_RING_SIZE 128
#define TX_RING_SIZE 512

#define NUM_MBUFS 8191
#define MBUF_CACHE_SIZE 250
#define BURST_SIZE 32


static const struct rte_eth_conf port_conf_default = {
	.rxmode = { .max_rx_pkt_len = ETHER_MAX_LEN, },
};

static struct rte_mempool *mbuf_pool;
static unsigned nb_ports;

static void xmit_arp_req(const uint8_t port, const uint32_t ip,
			 const struct ether_addr *ha);

/*
 *	IP addresses per port
 */

static uint8_t num_ip_addrs[5];
static uint32_t ip_addrs[5][5];

static bool
port_has_ip(const uint8_t port, const uint32_t ip)
{
	uint8_t i;
	uint8_t n = num_ip_addrs[port];
	for (i = 0; i < n; i++)
		if (ip == ip_addrs[port][i])
			return true;
	return false;
}

static uint32_t
port_get_ip(const uint8_t port, uint8_t index)
{
	if (num_ip_addrs[port] > index)
		return ip_addrs[port][index];
	return 0;
}

/*
 *	ARP cache
 */

struct arp_cache_entry {
	struct ether_addr	ha;
	time_t			ts;
	bool			stale;
};

#ifdef RTE_MACHINE_CPUFLAG_SSE4_2
#include <rte_hash_crc.h>
#define DEFAULT_HASH_FUNC       rte_hash_crc
#else
#include <rte_jhash.h>
#define DEFAULT_HASH_FUNC       rte_jhash
#endif

#define ARP_CACHE_TIMEOUT 7200
#define ARP_CACHE_ENTRIES 1024
#define ARP_MEMPOOL_CACHE_SIZE 73
static struct rte_hash_parameters arp_cache_params = {
	.name = "arp_cache",
	.entries = ARP_CACHE_ENTRIES,
	.reserved = 0,
	.key_len = sizeof(uint32_t),
	.hash_func = DEFAULT_HASH_FUNC,
	.hash_func_init_val = 0,
	.socket_id = 0,
	.extra_flag = 0,
};

static struct rte_hash *arp_cache = NULL;
static struct rte_mempool *arp_cache_entry_pool;

static void
arp_cache_entry_init(struct rte_mempool *mp,
		     __attribute__((unused)) void *opaque_arg,
		     void *_m,
		     __attribute__((unused)) unsigned i)
{
	struct rte_mbuf *m = _m;
	uint32_t arp_cache_entry_size, buf_len, priv_size;

	priv_size = rte_pktmbuf_priv_size(mp);
	arp_cache_entry_size = sizeof(struct arp_cache_entry);
	buf_len = rte_pktmbuf_data_room_size(mp);

	RTE_ASSERT(priv_size == 0);
	RTE_ASSERT(mp->elt_size >= arp_cache_entry_size);
	(void)arp_cache_entry_size;

	/* start of buffer is after mbuf structure and priv data */
	m->priv_size = priv_size;
	m->buf_addr = (char *)m;
	m->buf_physaddr = rte_mempool_virt2phy(mp, m);
	m->buf_len = (uint16_t)buf_len;

	/* No need for room between start of buffer and data. */
	m->data_off = 0;

	/* Initialize some constant fields. */
	m->pool = mp;
	m->nb_segs = 1;
	m->port = 0xff;
}

static int
init_arp_cache(void)
{
	arp_cache_params.socket_id = rte_socket_id();
	arp_cache = rte_hash_create(&arp_cache_params);
	if (arp_cache == NULL)
		return -1;

	/*
	 * Size of mempool should be n = (power of 2) - 1, and the size
	 * of the mempool cache should be c such that n % c == 0.
	 */
	arp_cache_entry_pool = rte_mempool_create("arp_pool",
		ARP_CACHE_ENTRIES / 2 - 1, sizeof(struct arp_cache_entry),
		ARP_MEMPOOL_CACHE_SIZE, 0,
		NULL, NULL,
		arp_cache_entry_init, NULL,
		rte_socket_id(), 0);
	if (arp_cache_entry_pool == NULL) {
		rte_hash_free(arp_cache);
		return -1;
	}

	/*
	 * TODO: pre-load cache with entries from config file.
	 * See examples/ipv4_multicast/main.c for an example.
	 */

	return 0;
}

/* Not multi-thread safe. */
static int
arp_cache_put(const uint32_t ip, const struct ether_addr *ha)
{
	struct arp_cache_entry *arp_entry;
	time_t update_time;
	int ret;

	update_time = time(NULL);

	ret = rte_hash_lookup_data(arp_cache, &ip, (void **)&arp_entry);
	if (ret == 0) {
		arp_entry->ts = update_time;
		arp_entry->stale = false;
	} else if (ret == -ENOENT) {
		ret = rte_mempool_sc_get(arp_cache_entry_pool,
					 (void **)&arp_entry);
		if (ret == -ENOENT) {
			printf("\nWARNING: no memory left in ARP cache entry pool; ARP entry not added\n");
			return -1;
		}
		arp_entry->ts = update_time;
		arp_entry->stale = false;
		ether_addr_copy(ha, &arp_entry->ha);
		ret = rte_hash_add_key_data(arp_cache, &ip, arp_entry);
		if (ret != 0) {
			printf("\nWARNING: invalid parameters; ARP entry not added\n");
			rte_mempool_sp_put(arp_cache_entry_pool, arp_entry);
			return -1;
		}
	} else if (ret == -EINVAL) {
		printf("\nWARNING: invalid parameters; could not lookup ARP entry\n");
		return -1;
	}

	return 0;
}

static inline const struct ether_addr *
arp_cache_get(const uint8_t port, const uint32_t ip)
{
	struct arp_cache_entry *arp_entry;
	int ret = rte_hash_lookup_data(arp_cache, &ip, (void **)&arp_entry);
	if (ret == 0) {
		time_t now = time(NULL);
		if (now - arp_entry->ts > ARP_CACHE_TIMEOUT) {
			arp_entry->stale = true;
			/* TODO: let caller know they should re-try later. */
			xmit_arp_req(port, ip, &arp_entry->ha);
			return NULL;
		}
		return &arp_entry->ha;
	} else if (ret == -ENOENT) {
		/* TODO: let caller know they should re-try later. */
		xmit_arp_req(port, ip, NULL);
		return NULL;
	} else if (ret == -EINVAL) {
		printf("\nWARNING: invalid parameters; could not lookup ARP entry\n");
		return NULL;
	}

	RTE_ASSERT(false);
	return NULL;
}

/*
 * Initialises a given port using global settings and with the rx buffers
 * coming from the mbuf_pool passed as parameter
 */
static inline int
port_init(uint8_t port)
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

	/* Set IP address(es) for this port. */
	if (port == 0) {
		/* 192.168.56.10 */
		ip_addrs[port][0] = 3232249866;
		num_ip_addrs[port] = 1;
	} else if (port == 1) {
		/* 192.168.57.10 */
		ip_addrs[port][0] = 3232250122;
		num_ip_addrs[port] = 1;
	}
	return 0;
}

static void
xmit_arp_req(const uint8_t port, const uint32_t ip, const struct ether_addr *ha)
{
	struct rte_mbuf *created_pkt;
	struct ether_hdr *eth_hdr;
	struct arp_hdr *arp_hdr;
	size_t pkt_size;

	created_pkt = rte_pktmbuf_alloc(mbuf_pool);
	pkt_size = sizeof(struct ether_hdr) + sizeof(struct arp_hdr);
	created_pkt->data_len = pkt_size;
	created_pkt->pkt_len = pkt_size;

	/* Set-up Ethernet header. */
	eth_hdr = rte_pktmbuf_mtod(created_pkt, struct ether_hdr *);
	rte_eth_macaddr_get(port, &eth_hdr->s_addr);
	if (ha == NULL)
		/* Broadcast. */
		memset(&eth_hdr->d_addr, 0xFF, ETHER_ADDR_LEN);
	else
		/* Unicast to previously-known entry. */
		ether_addr_copy(ha, &eth_hdr->d_addr);
	eth_hdr->ether_type = rte_cpu_to_be_16(ETHER_TYPE_ARP);

	/* Set-up ARP header. */
	arp_hdr = (struct arp_hdr *)((char *)eth_hdr + sizeof(struct ether_hdr));
	arp_hdr->arp_hrd = rte_cpu_to_be_16(ARP_HRD_ETHER);
	arp_hdr->arp_pro = rte_cpu_to_be_16(ETHER_TYPE_IPv4);
	arp_hdr->arp_hln = ETHER_ADDR_LEN;
	arp_hdr->arp_pln = sizeof(uint32_t);
	arp_hdr->arp_op = rte_cpu_to_be_16(ARP_OP_REQUEST);
	rte_eth_macaddr_get(port, &arp_hdr->arp_data.arp_sha);
	arp_hdr->arp_data.arp_sip = port_get_ip(port, 0);
	memset(&arp_hdr->arp_data.arp_tha, 0, ETHER_ADDR_LEN);
	arp_hdr->arp_data.arp_tip = ip;

	rte_eth_tx_burst(port, 0, &created_pkt, 1);
}

/*
 * Returns true if the packet should be transmitted, false if it should
 * not be transmitted (and can be freed).
 */
static bool
process_arp(uint8_t port, struct ether_hdr *eth_hdr, struct arp_hdr *arp_hdr,
	    struct ether_addr *port_ether_addr)
{
	uint32_t target_ip_be_32;

	/* We only support ARP for resolving IPv4 --> Ethernet addresses. */
	if (unlikely(arp_hdr->arp_hrd != rte_cpu_to_be_16(ARP_HRD_ETHER) ||
		     arp_hdr->arp_pro != rte_cpu_to_be_16(ETHER_TYPE_IPv4) ||
		     arp_hdr->arp_hln != ETHER_ADDR_LEN ||
		     arp_hdr->arp_pln != sizeof(uint32_t)))
		return false;

	/* Check if we have this IP address. */
	target_ip_be_32 = arp_hdr->arp_data.arp_tip;
	if (!port_has_ip(port, rte_be_to_cpu_32(target_ip_be_32)))
		return false;

	/* Update cache with source resolution, regardless of operation. */
	arp_cache_put(arp_hdr->arp_data.arp_sip, &arp_hdr->arp_data.arp_sha);

	switch (rte_be_to_cpu_16(arp_hdr->arp_op)) {
	case ARP_OP_REQUEST:
		/* Set-up Ethernet header. */
		ether_addr_copy(&eth_hdr->s_addr, &eth_hdr->d_addr);
		ether_addr_copy(port_ether_addr, &eth_hdr->s_addr);

		/* Set-up ARP header. */
		arp_hdr->arp_op = rte_cpu_to_be_16(ARP_OP_REPLY);
		ether_addr_copy(&arp_hdr->arp_data.arp_sha,
				&arp_hdr->arp_data.arp_tha);
		arp_hdr->arp_data.arp_tip = arp_hdr->arp_data.arp_sip;
		ether_addr_copy(port_ether_addr, &arp_hdr->arp_data.arp_sha);
		arp_hdr->arp_data.arp_sip = target_ip_be_32;
		return true;
	case ARP_OP_REPLY:
		/*
		 * No further action required. Could check to make sure
		 * arp_hdr->arp_data.arp_tha is equal to port_ether_addr,
		 * but there's nothing that can be done if it's wrong anyway.
		 */
		return false;
	default:
		return false;
	}

	RTE_ASSERT(false);
}

static inline size_t
get_vlan_offset(struct ether_hdr *eth_hdr, uint16_t *proto)
{
	size_t vlan_offset = 0;

	if (rte_cpu_to_be_16(ETHER_TYPE_VLAN) == *proto) {
		struct vlan_hdr *vlan_hdr = (struct vlan_hdr *)(eth_hdr + 1);

		vlan_offset = sizeof(struct vlan_hdr);
		*proto = vlan_hdr->eth_proto;

		if (rte_cpu_to_be_16(ETHER_TYPE_VLAN) == *proto) {
			vlan_hdr = vlan_hdr + 1;

			*proto = vlan_hdr->eth_proto;
			vlan_offset += sizeof(struct vlan_hdr);
		}
	}
	return vlan_offset;
}

static void
process_rx(uint8_t port, struct rte_mbuf **mbufs, const uint16_t nb_rx)
{
	uint16_t i;
	for (i = 0; i < nb_rx; i++) {
		struct ether_addr port_ether_addr;
		struct ether_hdr *eth_hdr;
		uint16_t ether_type, offset;

		eth_hdr = rte_pktmbuf_mtod(mbufs[i], struct ether_hdr *);

		/*
		 * Ensure this frame was sent to the right place. For ARP,
		 * this should probably just check that the destination
		 * MAC address is the broadcast address, but we'll check
		 * for both since it will probably be useful later.
		 */
		rte_eth_macaddr_get(port, &port_ether_addr);
		if (!is_broadcast_ether_addr(&eth_hdr->d_addr) &&
		    !is_same_ether_addr(&eth_hdr->d_addr, &port_ether_addr))
			goto free_mbuf;

		/* Calculate offset to skip over VLAN header, if present. */
		ether_type = eth_hdr->ether_type;
		offset = get_vlan_offset(eth_hdr, &ether_type);

		switch (rte_be_to_cpu_16(ether_type)) {
		case ETHER_TYPE_ARP: {
			struct arp_hdr *arp_hdr = (struct arp_hdr *)
				((char *)(eth_hdr + 1) + offset);

			if (process_arp(port, eth_hdr, arp_hdr,
					&port_ether_addr)) {
				const uint16_t nb_tx = rte_eth_tx_burst(port,
						0, &mbufs[i], 1);
				if (unlikely(nb_tx < 1))
					goto free_mbuf;
				continue;
			}

			goto free_mbuf;
		}
		default:
			goto free_mbuf;
		}

free_mbuf:
		rte_pktmbuf_free(mbufs[i]);
	}
}

/*
 * Main thread that does the work, reading from INPUT_PORT
 * and writing to OUTPUT_PORT
 */
static  __attribute__((noreturn)) void
lcore_main(void)
{
	uint8_t port;

	for (port = 0; port < nb_ports; port++)
		if (rte_eth_dev_socket_id(port) > 0 &&
				rte_eth_dev_socket_id(port) !=
						(int)rte_socket_id())
			printf("WARNING, port %u is on remote NUMA node to "
					"polling thread.\n\tPerformance will "
					"not be optimal.\n", port);

	printf("\nCore %u forwarding packets. [Ctrl+C to quit]\n",
			rte_lcore_id());
	for (;;) {
		for (port = 0; port < nb_ports; port++) {
			struct rte_mbuf *bufs[BURST_SIZE];
			const uint16_t nb_rx = rte_eth_rx_burst(port, 0,
					bufs, BURST_SIZE);

			if (unlikely(nb_rx == 0))
				continue;

			process_rx(port, bufs, nb_rx);
		}
	}
}

/* Main function, does initialisation and calls the per-lcore functions */
int
main(int argc, char *argv[])
{
	uint8_t portid;

	/* init EAL */
	int ret = rte_eal_init(argc, argv);

	if (ret < 0)
		rte_exit(EXIT_FAILURE, "Error with EAL initialization\n");
	argc -= ret;
	argv += ret;

	nb_ports = rte_eth_dev_count();
	if (nb_ports < 2 || (nb_ports & 1))
		rte_exit(EXIT_FAILURE, "Error: number of ports must be even\n");

	mbuf_pool = rte_pktmbuf_pool_create("MBUF_POOL",
		NUM_MBUFS * nb_ports, MBUF_CACHE_SIZE, 0,
		RTE_MBUF_DEFAULT_BUF_SIZE, rte_socket_id());
	if (mbuf_pool == NULL)
		rte_exit(EXIT_FAILURE, "Cannot create mbuf pool\n");

	/* initialize all ports */
	for (portid = 0; portid < nb_ports; portid++)
		if (port_init(portid) != 0)
			rte_exit(EXIT_FAILURE, "Cannot init port %"PRIu8"\n",
					portid);

	ret = init_arp_cache();
	if (ret != 0)
		rte_exit(EXIT_FAILURE, "Cannot build the ARP cache\n");

	if (rte_lcore_count() > 1)
		printf("\nWARNING: Too much enabled lcores - "
			"App uses only 1 lcore\n");

	/* call lcore_main on master core only */
	lcore_main();
	return 0;
}
