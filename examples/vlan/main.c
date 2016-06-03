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
#include <rte_eal.h>
#include <rte_ether.h>
#include <rte_ethdev.h>
#include <rte_cycles.h>
#include <rte_lcore.h>
#include <rte_mbuf.h>

#define RX_RING_SIZE 128
#define TX_RING_SIZE 512

#define NUM_MBUFS 8191
#define MBUF_CACHE_SIZE 250
#define BURST_SIZE 32

#define VLAN_TAG 1000

/*
 * The code can ignore needing to strip or insert VLAN tags, because
 * the hardware will take care of it. VLAN stripping and tagging can
 * be configured separately, but this sample application enables
 * both or disables both.
 */
#define USE_VLAN_HARDWARE_OFFLOAD 1

/*
 * The code can be configured to insert a VLAN tag on transmission
 * if this flag is set to 1. If it is, then the tag specified by
 * VLAN_TAG will be used.
 */
#define INSERT_VLAN_TAG 1

/*
 * If we know that we're going to insert an egress VLAN tag, we
 * can keep the ingress VLAN tag while processing the packet and
 * then just overwrite the tag at the end. This avoids copying
 * memory back and forth.
 */
#define KEEP_AND_OVERWRITE_VLAN 1

static const struct rte_eth_conf port_conf_default = {
	.rxmode = { .max_rx_pkt_len = ETHER_MAX_LEN, },
};

static unsigned nb_ports;

static inline int
vlan_id_is_invalid(uint16_t vlan_id)
{
	if (vlan_id < 4096)
		return 0;
	printf("Invalid vlan_id %d (must be < 4096)\n", vlan_id);
	return 1;
}

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

static void
tx_vlan_pvid_set(uint8_t port_id, uint16_t vlan_id, int on)
{
	if (!rte_eth_dev_is_valid_port(port_id))
                return;

        rte_eth_dev_set_vlan_pvid(port_id, vlan_id, on);
}

static void
vlan_tpid_set(uint8_t port_id, enum rte_vlan_type vlan_type, uint16_t tp_id)
{
        int diag;

	if (!rte_eth_dev_is_valid_port(port_id))
                return;

        diag = rte_eth_dev_set_vlan_ether_type(port_id, vlan_type, tp_id);
        if (diag == 0)
                return;

        printf("tx_vlan_tpid_set(port_pi=%d, vlan_type=%d, tpid=%d) failed "
               "diag=%d\n",
               port_id, vlan_type, tp_id, diag);
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

	/*
	 * TODO Is it better to enable/disable hardware VLAN stripping/insertion
	 * using rte_eth_dev_configure() or below? Can either be used? The
	 * insert options claim to only be available for i40e NICs.
	 */
	if (USE_VLAN_HARDWARE_OFFLOAD) {
		port_conf.rxmode.hw_vlan_strip = 1;
		if (INSERT_VLAN_TAG) {
			port_conf.txmode.hw_vlan_insert_pvid = 1;
			port_conf.txmode.pvid = VLAN_TAG;
		}
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
	 * TODO Is it better to enable/disable hardware VLAN stripping/insertion
	 * here or above? Can either be used? If here, which of the following
	 * calls are necessary?
	 */
	if (USE_VLAN_HARDWARE_OFFLOAD) {
		rx_vlan_strip_set(port, 1);
		rx_vlan_strip_set_on_queue(port, 0, 1);
		if (INSERT_VLAN_TAG) {
			tx_vlan_pvid_set(port, VLAN_TAG, 1);
			vlan_tpid_set(port, ETH_VLAN_TYPE_INNER, VLAN_TAG);
		}
	}

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
		bool vlan_present = false;

		eth_hdr = rte_pktmbuf_mtod(mbufs[i], struct ether_hdr *);
		ether_type = eth_hdr->ether_type;
		if (ether_type == rte_cpu_to_be_16(ETHER_TYPE_VLAN)) {
			vlan_present = true;
			if (!KEEP_AND_OVERWRITE_VLAN) {
				if (rte_vlan_strip(mbufs[i]) != 0) {
					rte_pktmbuf_free(mbufs[i]);
					continue;
				}
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
		if (vlan_present) {
			if (KEEP_AND_OVERWRITE_VLAN) {
				/* Overwrite old VLAN tag with new one. */
				struct vlan_hdr *vlan_hdr;
				vlan_hdr = (struct vlan_hdr *)(eth_hdr + 1);
				vlan_hdr->vlan_tci = rte_cpu_to_be_16(VLAN_TAG);
			} else if (INSERT_VLAN_TAG) {
				/* Insert VLAN tag. */
				mbufs[i]->ol_flags |= PKT_TX_VLAN_PKT;
				mbufs[i]->vlan_tci = VLAN_TAG;
				if (rte_vlan_insert(&mbufs[i]) != 0) {
					rte_pktmbuf_free(mbufs[i]);
					continue;
				}
			}
		}

		tx[nb_tx++] = mbufs[i];
	}

	tx_packets(port, tx, nb_tx);
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

			if (USE_VLAN_HARDWARE_OFFLOAD)
				/*
				 * You could do some processing on the
				 * packets first, but then just transmit them.
				 * The hardware will take care of removing
				 * the VLAN tag before we see the packets, and
				 * will add a tag after we're done.
				 */
				tx_packets(port, bufs, nb_rx);
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
	uint8_t portid;

	/* init EAL */
	int ret = rte_eal_init(argc, argv);

	if (ret < 0)
		rte_exit(EXIT_FAILURE, "Error with EAL initialization\n");
	argc -= ret;
	argv += ret;

	if (vlan_id_is_invalid(VLAN_TAG))
		rte_exit(EXIT_FAILURE, "Error with VLAN tag\n");

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
		if (port_init(portid, mbuf_pool) != 0)
			rte_exit(EXIT_FAILURE, "Cannot init port %"PRIu8"\n",
					portid);

	if (rte_lcore_count() > 1)
		printf("\nWARNING: Too much enabled lcores - "
			"App uses only 1 lcore\n");

	/* call lcore_main on master core only */
	lcore_main();
	return 0;
}
