/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2010-2016 Intel Corporation. All rights reserved.
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

#include "fib.h"

#if 0
struct fib_entry {
	uint32_t ip;
	uint8_t  depth;
	uint8_t  if_out;
};

static struct fib_entry fib_entry_array[] = {
	{IPv4(1, 1, 1, 0), 24, 0},
	{IPv4(2, 1, 1, 0), 24, 1},
	{IPv4(3, 1, 1, 0), 24, 2},
	{IPv4(4, 1, 1, 0), 24, 3},
	{IPv4(5, 1, 1, 0), 24, 4},
	{IPv4(6, 1, 1, 0), 24, 5},
	{IPv4(7, 1, 1, 0), 24, 6},
	{IPv4(8, 1, 1, 0), 24, 7},
};

#define FIB_NUM_ROUTES	(sizeof(fib_entry_array) / sizeof(fib_entry_array[0]))
#endif

#define FIB_MAX_RULES		1024
#define FIB_NUMBER_TBL8S	(1 << 8)

struct rte_lpm *fib = NULL;

#if 0
int
fib_main_loop(__attribute__((unused)) void *dummy)
{
	struct rte_mbuf *pkts_burst[MAX_PKT_BURST];
	unsigned lcore_id;
	uint64_t prev_tsc, diff_tsc, cur_tsc;
	int i, nb_rx;
	uint8_t portid, queueid;
	struct lcore_conf *qconf;
	const uint64_t drain_tsc = (rte_get_tsc_hz() + US_PER_S - 1) /
		US_PER_S * BURST_TX_DRAIN_US;

	prev_tsc = 0;

	lcore_id = rte_lcore_id();
	qconf = &lcore_conf[lcore_id];

	if (qconf->n_rx_queue == 0) {
		RTE_LOG(INFO, L3FWD, "lcore %u has nothing to do\n", lcore_id);
		return 0;
	}

	RTE_LOG(INFO, L3FWD, "entering main loop on lcore %u\n", lcore_id);

	for (i = 0; i < qconf->n_rx_queue; i++) {

		portid = qconf->rx_queue_list[i].port_id;
		queueid = qconf->rx_queue_list[i].queue_id;
		RTE_LOG(INFO, L3FWD,
			" -- lcoreid=%u portid=%hhu rxqueueid=%hhu\n",
			lcore_id, portid, queueid);
	}

	while (!force_quit) {

		cur_tsc = rte_rdtsc();

		/*
		 * TX burst queue drain
		 */
		diff_tsc = cur_tsc - prev_tsc;
		if (unlikely(diff_tsc > drain_tsc)) {

			for (i = 0; i < qconf->n_tx_port; ++i) {
				portid = qconf->tx_port_id[i];
				if (qconf->tx_mbufs[portid].len == 0)
					continue;
				send_burst(qconf,
					qconf->tx_mbufs[portid].len,
					portid);
				qconf->tx_mbufs[portid].len = 0;
			}

			prev_tsc = cur_tsc;
		}

		/*
		 * Read packet from RX queues
		 */
		for (i = 0; i < qconf->n_rx_queue; ++i) {
			portid = qconf->rx_queue_list[i].port_id;
			queueid = qconf->rx_queue_list[i].queue_id;
			nb_rx = rte_eth_rx_burst(portid, queueid, pkts_burst,
				MAX_PKT_BURST);
			if (nb_rx == 0)
				continue;

#if defined(__SSE4_1__)
			fib_send_packets(nb_rx, pkts_burst, portid, qconf);
#else
			fib_no_opt_send_packets(nb_rx, pkts_burst,
						portid, qconf);
#endif /* __SSE_4_1__ */
		}
	}

	return 0;
}
#endif

void
fib_setup(const int socketid)
{
	struct rte_lpm_config config_ipv4;
	char s[64];

	config_ipv4.max_rules = FIB_MAX_RULES;
	config_ipv4.number_tbl8s = FIB_NUMBER_TBL8S;
	config_ipv4.flags = 0;
	snprintf(s, sizeof(s), "BGP_FIB_%d", socketid);
	fib = rte_lpm_create(s, socketid, &config_ipv4);
	if (fib == NULL)
		rte_exit(EXIT_FAILURE,
			 "Unable to create the FIB on socket %d\n",
			 socketid);
}

int
fib_add_route(uint32_t ip, uint8_t depth, uint8_t if_out)
{
	return rte_lpm_add(fib, ip, depth, if_out);
}

int
fib_del_route(uint32_t ip, uint8_t depth)
{
	return rte_lpm_delete(fib, ip, depth);
}
