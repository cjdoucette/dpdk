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

#include <stdbool.h>
#include <rte_ethdev.h>

#define RX_RING_SIZE 128
#define TX_RING_SIZE 512

#define NUM_MBUFS 8191
#define MBUF_CACHE_SIZE 250
#define BURST_SIZE 32

#define VLAN_TAG 1000

/* Set to 1 to enable VLAN stripping in hardware. */
#define HW_STRIP_VLAN_TAG 0

/* Set to 1 to enable VLAN insertion in hardware. */
#define HW_INSERT_VLAN_TAG 0

/* Set to 1 to enable VLAN stripping in software. */
#define SW_STRIP_VLAN_TAG 0

/* Set to 1 to enable VLAN insertion in software. */
#define SW_INSERT_VLAN_TAG 1

/*
 * If we know that we're going to insert an egress VLAN tag, we
 * can keep the ingress VLAN tag while processing the packet and
 * then just overwrite the tag at the end. This avoids copying
 * memory back and forth.
 */
#define KEEP_AND_OVERWRITE_VLAN 0

static const struct rte_eth_conf port_conf_default = {
	.rxmode = { .max_rx_pkt_len = ETHER_MAX_LEN, },
};

static unsigned nb_ports;

#define RX_PORT	0
#define TX_PORT	1

static inline int
vlan_id_is_invalid(uint16_t vlan_id)
{
	if (vlan_id < 4096)
		return 0;
	printf("Invalid vlan_id %d (must be < 4096)\n", vlan_id);
	return 1;
}

#if 0
static void
rx_vlan_strip_set(uint8_t port_id, int on)
{
        int diag;
        int vlan_offload;

	if (!rte_eth_dev_is_valid_port(port_id))
                return;

        vlan_offload = rte_eth_dev_get_vlan_offload(port_id);

        if (on)
                vlan_offload |= ETH_VLAN_STRIP_OFFLOAD;
        else
                vlan_offload &= ~ETH_VLAN_STRIP_OFFLOAD;

        diag = rte_eth_dev_set_vlan_offload(port_id, vlan_offload);
        if (diag < 0)
                printf("rx_vlan_strip_set(port_pi=%d, on=%d) failed "
               "diag=%d\n", port_id, on, diag);
}

static void
rx_vlan_strip_set_on_queue(uint8_t port_id, uint16_t queue_id, int on)
{
        int diag;

	if (!rte_eth_dev_is_valid_port(port_id))
                return;

        diag = rte_eth_dev_set_vlan_strip_on_queue(port_id, queue_id, on);
        if (diag < 0)
                printf("rx_vlan_strip_set_on_queue(port_pi=%d, queue_id=%d, on=%d) failed "
               "diag=%d\n", port_id, queue_id, on, diag);
}

static int
tx_vlan_pvid_set(uint8_t port_id, uint16_t vlan_id, int on)
{
	if (!rte_eth_dev_is_valid_port(port_id)) {
		printf("err here\n");
                return -1;
	}

        return rte_eth_dev_set_vlan_pvid(port_id, vlan_id, on);
}
#endif

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

	/*
	 * The calls below show how the port VLAN stripping and insertion
	 * can be configured before the device is started
	 */
	if (port == RX_PORT && HW_STRIP_VLAN_TAG) {
		port_conf.rxmode.hw_vlan_strip = 1;
	}
	if (port == TX_PORT && HW_INSERT_VLAN_TAG) {
		port_conf.txmode.hw_vlan_insert_pvid = 1;
		port_conf.txmode.pvid = 0x03e8;
	}

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

	/*
	 * The calls below show how the port VLAN stripping and insertion
	 * can be configured *after* rte_eth_dev_start() has been called.
	 */
#if 0
	if (port == RX_PORT && HW_STRIP_VLAN_TAG) {
		/*
		 * XXX In some cases, the order in which these calls
		 * are invoked can matter. See the README for more
		 * information.
		 */
		rx_vlan_strip_set(port, 1);
		rx_vlan_strip_set_on_queue(port, 0, 1);
	}
	if (port == TX_PORT && HW_INSERT_VLAN_TAG) {
		tx_vlan_pvid_set(port, VLAN_TAG, 1);
	}
#endif 

	return 0;
}

static void
tx_packets(const uint8_t port, struct rte_mbuf **bufs, const uint16_t n)
{
	const uint16_t nb_tx = rte_eth_tx_burst(port, 0, bufs, n);
	if (unlikely(nb_tx < n)) {
		uint16_t buf;
		for (buf = nb_tx; buf < n; buf++)
			rte_pktmbuf_free(bufs[buf]);
	}
}

static void
insert_tx_packets(const uint8_t port, struct rte_mbuf **bufs, const uint16_t n)
{
	uint16_t nb_tx, i;
	for (i = 0; i < n; i++) {
		/* Insert VLAN tag. */
		bufs[i]->ol_flags |= PKT_TX_VLAN_PKT;
		bufs[i]->vlan_tci = VLAN_TAG;
		if (rte_vlan_insert(&bufs[i]) != 0) {
			/* XXX Would probably want to handle this. */
			continue;
		}
	}

        nb_tx = rte_eth_tx_burst(port, 0, bufs, n);
	if (unlikely(nb_tx < n)) {
		uint16_t buf;
		for (buf = nb_tx; buf < n; buf++)
			rte_pktmbuf_free(bufs[buf]);
	}
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
process_vlan(const uint8_t port, struct rte_mbuf **mbufs, const uint16_t nb_rx)
{
	struct ether_hdr *eth_hdr;
	uint16_t ether_type, offset, i;

	struct rte_mbuf *tx[nb_rx];
	uint16_t nb_tx = 0;

	for (i = 0; i < nb_rx; i++) {

		eth_hdr = rte_pktmbuf_mtod(mbufs[i], struct ether_hdr *);
		ether_type = eth_hdr->ether_type;
		if (ether_type == rte_cpu_to_be_16(ETHER_TYPE_VLAN)) {
			if (SW_STRIP_VLAN_TAG) {
				if (rte_vlan_strip(mbufs[i]) != 0) {
					rte_pktmbuf_free(mbufs[i]);
					continue;
				}
				ether_type = eth_hdr->ether_type;
			}
		}

		/* Read in next ether_type after VLAN, if present. */
		offset = get_vlan_offset(eth_hdr, &ether_type);

		/* Process rest packet in some way. */
		switch (rte_be_to_cpu_16(ether_type)) {
		case ETHER_TYPE_ARP: {
			struct arp_hdr *arp_hdr = (struct arp_hdr *)
				((char *)(eth_hdr + 1) + offset);
			(void)arp_hdr;
			/* ... */
			break;
		}
		/* ... */
		default:
			break;
		}

		/* Add VLAN tag by insertion or overwriting. */
		if (ether_type == rte_cpu_to_be_16(ETHER_TYPE_VLAN) &&
		    SW_INSERT_VLAN_TAG) {
			/* Overwrite old VLAN tag with new one. */
			struct vlan_hdr *vlan_hdr;
			vlan_hdr = (struct vlan_hdr *)(eth_hdr + 1);
			vlan_hdr->vlan_tci = rte_cpu_to_be_16(VLAN_TAG);
		} else if (SW_INSERT_VLAN_TAG) {
			/* Insert VLAN tag. */
			mbufs[i]->ol_flags |= PKT_TX_VLAN_PKT;
			mbufs[i]->vlan_tci = VLAN_TAG;
			if (rte_vlan_insert(&mbufs[i]) != 0) {
				rte_pktmbuf_free(mbufs[i]);
				continue;
			}
		}

		tx[nb_tx++] = mbufs[i];
	}

	(void)port;
	tx_packets(TX_PORT, tx, nb_tx);
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

			uint16_t buf;

			if (unlikely(nb_rx == 0))
				continue;

			for (buf = 0; buf < nb_rx; buf++) {
				struct rte_mbuf *mbuf = bufs[buf];
				unsigned int len = rte_pktmbuf_data_len(mbuf);
				rte_pktmbuf_dump(stdout, mbuf, len);
				rte_pktmbuf_free(mbuf);
			}

			if (HW_STRIP_VLAN_TAG && HW_INSERT_VLAN_TAG)
				/*
				 * You could do some processing on the
				 * packets first, but then just transmit them.
				 * The hardware will take care of removing
				 * the VLAN tag before we see the packets, and
				 * will add a tag after we're done.
				 */
				tx_packets(TX_PORT, bufs, nb_rx);
			else if (HW_STRIP_VLAN_TAG && SW_INSERT_VLAN_TAG)
				insert_tx_packets(TX_PORT, bufs, nb_rx);
			else
				/* Do some processing on the packets while
				 * keeping the VLAN header in mind. We can
				 * either remove the VLAN header before
				 * processing, or keep it in and then
				 * overwrite it with a new tag before tx.
				 */
				process_vlan(port, bufs, nb_rx);
		}
	}
}

/* Main function, does initialisation and calls the per-lcore functions */
int
main(int argc, char *argv[])
{
	struct rte_mempool *mbuf_pool;

	/* init EAL */
	int ret = rte_eal_init(argc, argv);

	if (ret < 0)
		rte_exit(EXIT_FAILURE, "Error with EAL initialization\n");
	argc -= ret;
	argv += ret;

	if (vlan_id_is_invalid(VLAN_TAG))
		rte_exit(EXIT_FAILURE, "Error with VLAN tag\n");

	nb_ports = rte_eth_dev_count();
	if (nb_ports != 2)
		rte_exit(EXIT_FAILURE, "Need two ports\n");

	mbuf_pool = rte_pktmbuf_pool_create("MBUF_POOL",
		NUM_MBUFS * nb_ports, MBUF_CACHE_SIZE, 0,
		RTE_MBUF_DEFAULT_BUF_SIZE, rte_socket_id());
	if (mbuf_pool == NULL)
		rte_exit(EXIT_FAILURE, "Cannot create mbuf pool\n");

	/* initialize all ports */
	if (port_init(RX_PORT, mbuf_pool) != 0)
		rte_exit(EXIT_FAILURE, "Cannot init port %"PRIu8"\n", RX_PORT);

	if (port_init(TX_PORT, mbuf_pool) != 0)
		rte_exit(EXIT_FAILURE, "Cannot init port %"PRIu8"\n", TX_PORT);

	if (rte_lcore_count() > 1)
		printf("\nWARNING: Too much enabled lcores - "
			"App uses only 1 lcore\n");

	/* call lcore_main on master core only */
	lcore_main();
	return 0;
}
