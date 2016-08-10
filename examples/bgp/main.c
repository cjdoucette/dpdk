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

#include <arpa/inet.h>

#include <rte_ethdev.h>
#include <rte_cycles.h>
#include <rte_kni.h>

#include "kni.h"
#include "cli.h"
#include "nl.h"
#include "fib.h"
#include "common.h"

/* BGP task application parameters. */
#define PORT		0
#define KNI_IP_ADDR	"192.168.57.12"
#define TCP_BGP_PORT	179

/* Parameters for the mbuf pool. */
#define MBUF_DATA_SZ		(MAX_PACKET_SZ + RTE_PKTMBUF_HEADROOM)
#define NB_MBUF                 (8192 * 16)
#define MEMPOOL_CACHE_SZ        PKT_BURST_SZ

/* RX/TX parameters. */
#define PKT_BURST_SZ            32
#define NB_RXD                  128
#define NB_TXD                  512

struct kni_port_params *kni_port_params_array[RTE_MAX_ETHPORTS];
static rte_atomic32_t kni_stop = RTE_ATOMIC32_INIT(0);

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

	rte_eth_promiscuous_enable(port);
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
setup_bgp_filter(uint8_t port_id)
{
	struct rte_eth_fdir_filter entry;
	uint32_t kni_ip_addr;
	int ret = 0;

	ret = rte_eth_dev_filter_supported(port_id, RTE_ETH_FILTER_FDIR);
	if (ret < 0) {
		printf("flow director is not supported on port %u.\n",
			port_id);
		return -1;
	}
	memset(&entry, 0, sizeof(struct rte_eth_fdir_filter));

	ret = inet_pton(AF_INET, KNI_IP_ADDR, &kni_ip_addr);
	if (ret == 0) {
		printf("%s is not a valid address in AF_INET\n", KNI_IP_ADDR);
		return -1;
	} else if (ret == -1) {
		perror("inet_pton");
		return -1;
	}

	entry.input.flow_type = RTE_ETH_FLOW_NONFRAG_IPV4_TCP;
	entry.input.flow.ip4_flow.dst_ip = kni_ip_addr;
	entry.input.flow.udp4_flow.dst_port = rte_cpu_to_be_16(TCP_BGP_PORT);

	entry.input.flow_ext.is_vf = 0;

	entry.action.behavior = RTE_ETH_FDIR_ACCEPT;
	entry.action.flex_off = 0;  /* Use 0 by default. */
	entry.action.report_status = RTE_ETH_FDIR_REPORT_ID;
	entry.action.rx_queue = 127;

	entry.soft_id = 0;

	ret = rte_eth_dev_filter_ctrl(port_id, RTE_ETH_FILTER_FDIR,
		RTE_ETH_FILTER_ADD, &entry);

/*
	ret = rte_eth_dev_filter_ctrl(res->port_id, RTE_ETH_FILTER_FDIR,
		RTE_ETH_FILTER_DELETE, &entry);
	ret = rte_eth_dev_filter_ctrl(res->port_id, RTE_ETH_FILTER_FDIR,
		RTE_ETH_FILTER_UPDATE, &entry);
*/

	if (ret < 0) {
		printf("flow director programming error: (%s)\n",
			strerror(-ret));
		return -1;
	}

	return 0;
}

/* Initialise ports/queues etc. and start main loop on each core */
int
main(int argc, char** argv)
{
	unsigned i;
	int ret;

	/* Initialise EAL */
	ret = rte_eal_init(argc, argv);
	if (ret < 0)
		rte_exit(EXIT_FAILURE, "Could not initialise EAL (%d)\n", ret);
	argc -= ret;
	argv += ret;

	/* Parse application arguments (after the EAL ones). */
	ret = parse_args(argc, argv);
	if (ret < 0)
		rte_exit(EXIT_FAILURE, "Could not parse input parameters\n");

	/* Create the mbuf pool. */
	pktmbuf_pool = rte_pktmbuf_pool_create("mbuf_pool", NB_MBUF,
		MEMPOOL_CACHE_SZ, 0, MBUF_DATA_SZ, rte_socket_id());
	if (pktmbuf_pool == NULL) {
		rte_exit(EXIT_FAILURE, "Could not initialise mbuf pool\n");
		return -1;
	}

	init_kni();
	init_port(PORT);
	kni_alloc(PORT);

	if (setup_bgp_filter(PORT)) {
		printf("could not initialize BGP filter\n");
		return -1;
	}

	/*
	 * To make this application NUMA-aware, the l3fwd application
	 * uses the parameter to this setup function as a socket ID
	 * for separate pools of memory and LPM lookup tables, per-lcore.
	 * For this example, we'll just use one pool of packets and
	 * one lookup table.
	 */
	fib_setup(PORT);

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

	kni_release(PORT);

	return 0;
}
