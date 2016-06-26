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

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <sys/queue.h>
#include <stdarg.h>
#include <errno.h>
#include <getopt.h>

#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/if.h>
#include <linux/if_tun.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <rte_common.h>
#include <rte_log.h>
#include <rte_memory.h>
#include <rte_memcpy.h>
#include <rte_memzone.h>
#include <rte_eal.h>
#include <rte_per_lcore.h>
#include <rte_launch.h>
#include <rte_atomic.h>
#include <rte_lcore.h>
#include <rte_branch_prediction.h>
#include <rte_interrupts.h>
#include <rte_pci.h>
#include <rte_debug.h>
#include <rte_ether.h>
#include <rte_ethdev.h>
#include <rte_ring.h>
#include <rte_log.h>
#include <rte_mempool.h>
#include <rte_mbuf.h>
#include <rte_string_fns.h>
#include <rte_cycles.h>
#include <rte_malloc.h>
#include <rte_kni.h>

#include "kni.h"
#include "cli.h"
#include "nl.h"
#include "fib.h"
#include "common.h"

#define KNI_IP_ADDR	"192.168.57.12"
#define TCP_BGP_PORT	179

/* Size of the data buffer in each mbuf */
#define MBUF_DATA_SZ (MAX_PACKET_SZ + RTE_PKTMBUF_HEADROOM)

/* Number of mbufs in mempool that is created */
#define NB_MBUF                 (8192 * 16)

/* How many packets to attempt to read from NIC in one go */
#define PKT_BURST_SZ            32

/* How many objects (mbufs) to keep in per-lcore mempool cache */
#define MEMPOOL_CACHE_SZ        PKT_BURST_SZ

/* Number of RX ring descriptors */
#define NB_RXD                  128

/* Number of TX ring descriptors */
#define NB_TXD                  512

struct kni_port_params *kni_port_params_array[RTE_MAX_ETHPORTS];
static rte_atomic32_t kni_stop = RTE_ATOMIC32_INIT(0);

uint32_t ports_mask = 0;
int promiscuous_on = 1;
struct rte_mempool *pktmbuf_pool = NULL;

struct rte_eth_conf port_conf = {
	.rxmode = {
		.header_split = 0,      /* Header Split disabled */
		.hw_ip_checksum = 0,    /* IP checksum offload disabled */
		.hw_vlan_filter = 0,    /* VLAN filtering disabled */
		.jumbo_frame = 0,       /* Jumbo Frame Support disabled */
		.hw_strip_crc = 0,      /* CRC stripped by hardware */
	},
	.txmode = {
		.mq_mode = ETH_MQ_TX_NONE,
	},
};

uint64_t dest_eth_addr[RTE_MAX_ETHPORTS];
struct ether_addr ports_eth_addr[RTE_MAX_ETHPORTS];

/* TODO: needed? Socket for receiving updates from BGP daemon. */
//static struct mnl_socket *nl;

static void
kni_burst_free_mbufs(struct rte_mbuf **pkts, unsigned num)
{
	unsigned i;

	if (pkts == NULL)
		return;

	for (i = 0; i < num; i++) {
		rte_pktmbuf_free(pkts[i]);
		pkts[i] = NULL;
	}
}

/**
 * Interface to burst rx and enqueue mbufs into rx_q
 */
static void
kni_ingress(struct kni_port_params *p)
{
	uint8_t i, port_id;
	unsigned nb_rx, num;
	uint32_t nb_kni;
	struct rte_mbuf *pkts_burst[PKT_BURST_SZ];

	if (p == NULL)
		return;

	nb_kni = p->nb_kni;
	port_id = p->port_id;
	for (i = 0; i < nb_kni; i++) {
		/* Burst rx from eth */
		nb_rx = rte_eth_rx_burst(port_id, 0, pkts_burst, PKT_BURST_SZ);
		if (unlikely(nb_rx > PKT_BURST_SZ)) {
			RTE_LOG(ERR, APP, "Error receiving from eth\n");
			return;
		}
		if (nb_rx > 0)
			printf("port %hhu received %u packets\n", port_id, nb_rx);
		/* Burst tx to kni */
		num = rte_kni_tx_burst(p->kni[i], pkts_burst, nb_rx);

		rte_kni_handle_request(p->kni[i]);
		if (unlikely(num < nb_rx)) {
			/* Free mbufs not tx to kni interface */
			kni_burst_free_mbufs(&pkts_burst[num], nb_rx - num);
		}
	}
}

/**
 * Interface to dequeue mbufs from tx_q and burst tx
 */
static void
kni_egress(struct kni_port_params *p)
{
	uint8_t i, port_id;
	unsigned nb_tx, num;
	uint32_t nb_kni;
	struct rte_mbuf *pkts_burst[PKT_BURST_SZ];
	struct rte_mbuf *pkts_to_send[PKT_BURST_SZ];

	if (p == NULL)
		return;

	nb_kni = p->nb_kni;
	port_id = p->port_id;
	for (i = 0; i < nb_kni; i++) {
		struct ether_hdr *eth_hdr;
		uint8_t to_send = 0;
		uint8_t j;

		/* Burst rx from kni */
		num = rte_kni_rx_burst(p->kni[i], pkts_burst, PKT_BURST_SZ);
		if (unlikely(num > PKT_BURST_SZ)) {
			RTE_LOG(ERR, APP, "Error receiving from KNI\n");
			return;
		}

		for (j = 0; j < num; j++) {
			eth_hdr = rte_pktmbuf_mtod(pkts_burst[i],
						   struct ether_hdr *);
			if (rte_be_to_cpu_16(eth_hdr->ether_type) ==
			    ETHER_TYPE_EXPER) {
				handle_nlmsg((struct nlmsghdr *)(eth_hdr + 1));
				rte_pktmbuf_free(pkts_burst[i]);
			} else
				pkts_to_send[to_send++] = pkts_burst[i];
		}

		/* Burst tx to eth */
		nb_tx = rte_eth_tx_burst(port_id, 0, pkts_to_send,
					 (uint16_t)to_send);
		if (unlikely(nb_tx < to_send))
			/* Free mbufs not tx to NIC */
			kni_burst_free_mbufs(&pkts_to_send[nb_tx],
					     to_send - nb_tx);
	}
}

static int
main_loop(__rte_unused void *arg)
{
	uint8_t i, nb_ports = rte_eth_dev_count();
	int32_t f_stop;
	const unsigned lcore_id = rte_lcore_id();
	enum lcore_rxtx {
		LCORE_NONE,
		LCORE_RX,
		LCORE_TX,
		LCORE_MAX
	};
	enum lcore_rxtx flag = LCORE_NONE;

	for (i = 0; i < nb_ports; i++) {
		if (!kni_port_params_array[i])
			continue;
		if (kni_port_params_array[i]->lcore_rx == (uint8_t)lcore_id) {
			flag = LCORE_RX;
			break;
		} else if (kni_port_params_array[i]->lcore_tx ==
						(uint8_t)lcore_id) {
			flag = LCORE_TX;
			break;
		}
	}

	if (flag == LCORE_RX) {
		RTE_LOG(INFO, APP, "Lcore %u is reading from port %d\n",
					kni_port_params_array[i]->lcore_rx,
					kni_port_params_array[i]->port_id);
		while (1) {
			f_stop = rte_atomic32_read(&kni_stop);
			if (f_stop)
				break;
			kni_ingress(kni_port_params_array[i]);
		}
	} else if (flag == LCORE_TX) {
		RTE_LOG(INFO, APP, "Lcore %u is writing to port %d\n",
					kni_port_params_array[i]->lcore_tx,
					kni_port_params_array[i]->port_id);
		while (1) {
			f_stop = rte_atomic32_read(&kni_stop);
			if (f_stop)
				break;
			kni_egress(kni_port_params_array[i]);
		}
	} else
		RTE_LOG(INFO, APP, "Lcore %u has nothing to do\n", lcore_id);

	return 0;
}

/* Initialise a single port on an Ethernet device */
static void
init_port(uint8_t port)
{
	int ret;

	/* Initialise device and RX/TX queues */
	RTE_LOG(INFO, APP, "Initialising port %u ...\n", (unsigned)port);
	fflush(stdout);
	ret = rte_eth_dev_configure(port, 1, 1, &port_conf);
	if (ret < 0)
		rte_exit(EXIT_FAILURE, "Could not configure port%u (%d)\n",
		            (unsigned)port, ret);

	ret = rte_eth_rx_queue_setup(port, 0, NB_RXD,
		rte_eth_dev_socket_id(port), NULL, pktmbuf_pool);
	if (ret < 0)
		rte_exit(EXIT_FAILURE, "Could not setup up RX queue for "
				"port%u (%d)\n", (unsigned)port, ret);

	ret = rte_eth_tx_queue_setup(port, 0, NB_TXD,
		rte_eth_dev_socket_id(port), NULL);
	if (ret < 0)
		rte_exit(EXIT_FAILURE, "Could not setup up TX queue for "
				"port%u (%d)\n", (unsigned)port, ret);

	ret = rte_eth_dev_start(port);
	if (ret < 0)
		rte_exit(EXIT_FAILURE, "Could not start port%u (%d)\n",
						(unsigned)port, ret);

	if (promiscuous_on)
		rte_eth_promiscuous_enable(port);
}

/* Check the link status of all ports in up to 9s, and print them finally */
static void
check_all_ports_link_status(uint8_t port_num, uint32_t port_mask)
{
#define CHECK_INTERVAL 100 /* 100ms */
#define MAX_CHECK_TIME 90 /* 9s (90 * 100ms) in total */
	uint8_t portid, count, all_ports_up, print_flag = 0;
	struct rte_eth_link link;

	printf("\nChecking link status\n");
	fflush(stdout);
	for (count = 0; count <= MAX_CHECK_TIME; count++) {
		all_ports_up = 1;
		for (portid = 0; portid < port_num; portid++) {
			if ((port_mask & (1 << portid)) == 0)
				continue;
			memset(&link, 0, sizeof(link));
			rte_eth_link_get_nowait(portid, &link);
			/* print link status if flag set */
			if (print_flag == 1) {
				if (link.link_status)
					printf("Port %d Link Up - speed %u "
						"Mbps - %s\n", (uint8_t)portid,
						(unsigned)link.link_speed,
				(link.link_duplex == ETH_LINK_FULL_DUPLEX) ?
					("full-duplex") : ("half-duplex\n"));
				else
					printf("Port %d Link Down\n",
						(uint8_t)portid);
				continue;
			}
			/* clear all_ports_up flag if any link down */
			if (link.link_status == ETH_LINK_DOWN) {
				all_ports_up = 0;
				break;
			}
		}
		/* after finally printing all link status, get out */
		if (print_flag == 1)
			break;

		if (all_ports_up == 0) {
			printf(".");
			fflush(stdout);
			rte_delay_ms(CHECK_INTERVAL);
		}

		/* set the print_flag if all ports up or timeout */
		if (all_ports_up == 1 || count == (MAX_CHECK_TIME - 1)) {
			print_flag = 1;
			printf("done\n");
		}
	}
}

static int
bgp_receiver(__rte_unused void *arg)
{
	uint8_t i, nb_ports = rte_eth_dev_count();
	int32_t f_stop;
	const unsigned lcore_id = rte_lcore_id();
	enum lcore_rxtx {
		LCORE_NONE,
		LCORE_RX,
		LCORE_TX,
		LCORE_MAX
	};
	enum lcore_rxtx flag = LCORE_NONE;

	for (i = 0; i < nb_ports; i++) {
		if (!kni_port_params_array[i])
			continue;
		if (kni_port_params_array[i]->lcore_rx == (uint8_t)lcore_id) {
			flag = LCORE_RX;
			break;
		} else if (kni_port_params_array[i]->lcore_tx ==
						(uint8_t)lcore_id) {
			flag = LCORE_TX;
			break;
		}
	}

	if (flag == LCORE_RX) {
		RTE_LOG(INFO, APP, "Lcore %u is reading from port %d\n",
					kni_port_params_array[i]->lcore_rx,
					kni_port_params_array[i]->port_id);
		while (1) {
			f_stop = rte_atomic32_read(&kni_stop);
			if (f_stop)
				break;
			kni_ingress(kni_port_params_array[i]);
		}
	} else if (flag == LCORE_TX) {
		RTE_LOG(INFO, APP, "Lcore %u is writing to port %d\n",
					kni_port_params_array[i]->lcore_tx,
					kni_port_params_array[i]->port_id);
		while (1) {
			f_stop = rte_atomic32_read(&kni_stop);
			if (f_stop)
				break;
			kni_egress(kni_port_params_array[i]);
		}
	} else
		RTE_LOG(INFO, APP, "Lcore %u has nothing to do\n", lcore_id);

	return 0;


}

static int
remote_launch_except_special(int (*f)(void *), void *arg,
			     enum rte_rmt_call_master_t call_master,
			     int special_lcore)
{
	int lcore_id;
	int master = rte_get_master_lcore();

	/* Check state of lcores. */
	RTE_LCORE_FOREACH_SLAVE(lcore_id) {
		if (lcore_config[lcore_id].state != WAIT)
			return -EBUSY;
	}

	/* Send messages to cores, launching special function. */
	RTE_LCORE_FOREACH_SLAVE(lcore_id) {
		if (lcore_id != special_lcore)
			rte_eal_remote_launch(f, arg, lcore_id);
	}
	rte_eal_remote_launch(&bgp_receiver, arg, special_lcore);

	if (call_master == CALL_MASTER) {
		lcore_config[master].ret = f(arg);
		lcore_config[master].state = FINISHED;
	}

	return 0;
}

static int
setup_bgp_filter(void)
{
	/* XXX In an environment with more than one queue, we should create
	 * additional RX queues for the port on which BGP packets will
	 * be received (via rte_eth_dev_configure()), and use that queue
	 * identifier here. Then, use rte_eal_remote_launch() to start
	 * an lcore that polls that queue.
	 */
	struct rte_eth_ntuple_filter bgp_filter = {
		.flags = RTE_2TUPLE_FLAGS,
		.dst_ip = 0, /* Set below. */
		.dst_ip_mask = UINT32_MAX,
		.src_ip = 0,
		.src_ip_mask = 0,
		.dst_port = TCP_BGP_PORT,
		.dst_port_mask = UINT16_MAX,
		.src_port = 0,
		.src_port_mask = 0,
		.proto = IPPROTO_TCP,
		.proto_mask = UINT8_MAX,
		.tcp_flags = 0,
		.priority = 1, /* Lowest */
		.queue = 0,
	};

	uint32_t kni_ip_addr;
	int ret = inet_pton(AF_INET, KNI_IP_ADDR, &kni_ip_addr);
	if (ret == 0) {
		printf("%s is not a valid address in AF_INET\n", KNI_IP_ADDR);
		return -1;
	} else if (ret == -1) {
		perror("inet_pton");
		return -1;
	}

	bgp_filter.dst_ip = kni_ip_addr;
	return rte_eth_dev_filter_ctrl(1, RTE_ETH_FILTER_NTUPLE,
				       RTE_ETH_FILTER_ADD, &bgp_filter);
}

/* Initialise ports/queues etc. and start main loop on each core */
int
main(int argc, char** argv)
{
	int ret;
	uint8_t nb_sys_ports, port;
	unsigned i;

	/* Initialise EAL */
	ret = rte_eal_init(argc, argv);
	if (ret < 0)
		rte_exit(EXIT_FAILURE, "Could not initialise EAL (%d)\n", ret);
	argc -= ret;
	argv += ret;

	/* Parse application arguments (after the EAL ones) */
	ret = parse_args(argc, argv);
	if (ret < 0)
		rte_exit(EXIT_FAILURE, "Could not parse input parameters\n");

	/* Create the mbuf pool */
	pktmbuf_pool = rte_pktmbuf_pool_create("mbuf_pool", NB_MBUF,
		MEMPOOL_CACHE_SZ, 0, MBUF_DATA_SZ, rte_socket_id());
	if (pktmbuf_pool == NULL) {
		rte_exit(EXIT_FAILURE, "Could not initialise mbuf pool\n");
		return -1;
	}

	/* Get number of ports found in scan */
	nb_sys_ports = rte_eth_dev_count();
	if (nb_sys_ports == 0)
		rte_exit(EXIT_FAILURE, "No supported Ethernet device found\n");

	/* Check if the configured port ID is valid */
	for (i = 0; i < RTE_MAX_ETHPORTS; i++)
		if (kni_port_params_array[i] && i >= nb_sys_ports)
			rte_exit(EXIT_FAILURE, "Configured invalid "
						"port ID %u\n", i);

	/* Initialize KNI subsystem */
	init_kni();

	/* Initialise each port */
	for (port = 0; port < nb_sys_ports; port++) {
		/* Skip ports that are not enabled */
		if (!(ports_mask & (1 << port)))
			continue;
		init_port(port);

		if (port >= RTE_MAX_ETHPORTS)
			rte_exit(EXIT_FAILURE, "Can not use more than "
				"%d ports for kni\n", RTE_MAX_ETHPORTS);

		kni_alloc(port);
	}
	check_all_ports_link_status(nb_sys_ports, ports_mask);

	if (setup_bgp_filter()) {
		printf("could not initialize BGP filter\n");
		return -1;
	}

	/* Initialize dst MACs for all ports. */
	for (port = 0; port < RTE_MAX_ETHPORTS; port++) {
		//dest_eth_addr[port] =
		//	ETHER_LOCAL_ADMIN_ADDR + ((uint64_t)port << 40);
		//rte_eth_macaddr_get(port, &ports_eth_addr[port]);
	}

	/*
	 * To make this application NUMA-aware, the l3fwd application
	 * uses the parameter to this setup function as a socket ID
	 * for separate pools of memory and LPM lookup tables, per-lcore.
	 * For this example, we'll just use one pool of packets and
	 * one lookup table.
	 */
	fib_setup(0);

	/* Launch per-lcore function on every lcore */

	/*
	 * This would launch same loop on every lcore:
	 *   rte_eal_mp_remote_launch(main_loop, NULL, CALL_MASTER);
	 *
	 * But for this application we want most lcores running the same
	 * loop, but need one lcore to run a different loop. No easy way
	 * to do this using that function, so we use our own version.
	 */
	remote_launch_except_special(main_loop, NULL, CALL_MASTER,
				     rte_get_next_lcore(0, 1, 0));

	RTE_LCORE_FOREACH_SLAVE(i) {
		if (rte_eal_wait_lcore(i) < 0)
			return -1;
	}

	kni_release(nb_sys_ports);

	return 0;
}
