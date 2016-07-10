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

#include <string.h>
#include <locale.h>
#include <unistd.h>
#include <getopt.h>

#include <rte_string_fns.h>

#include "main.h"

#define MAX_OPT_VALUES 8
#define SYS_CPU_DIR "/sys/devices/system/cpu/cpu%u/topology/"
#define APP_MASTER_CORE	0

static uint64_t app_used_core_mask = 0;
static uint64_t app_used_rx_port_mask = 0;
static uint64_t app_used_tx_port_mask = 0;

static const char usage[] =
	"                                                                               \n"
	"    %s <APP PARAMS>                                                            \n"
	"                                                                               \n"
	"Application mandatory parameters:                                              \n"
	"    --pfc \"RX PORT, TX PORT, RX LCORE, WT LCORE\" : Packet flow configuration \n"
	"           multiple pfc can be configured in command line                      \n"
;

/* display usage */
static void
app_usage(const char *prgname)
{
	printf(usage, prgname);
}

static inline int str_is(const char *str, const char *is)
{
	return strcmp(str, is) == 0;
}

/* returns core mask used by DPDK */
static uint64_t
app_eal_core_mask(void)
{
	uint32_t i;
	uint64_t cm = 0;
	struct rte_config *cfg = rte_eal_get_configuration();

	for (i = 0; i < APP_MAX_LCORE; i++) {
		if (cfg->lcore_role[i] == ROLE_RTE)
			cm |= (1ULL << i);
	}

	cm |= (1ULL << cfg->master_lcore);

	return cm;
}


/* returns total number of cores presented in a system */
static uint32_t
app_cpu_core_count(void)
{
	int i, len;
	char path[PATH_MAX];
	uint32_t ncores = 0;

	for (i = 0; i < APP_MAX_LCORE; i++) {
		len = snprintf(path, sizeof(path), SYS_CPU_DIR, i);
		if (len <= 0 || (unsigned)len >= sizeof(path))
			continue;

		if (access(path, F_OK) == 0)
			ncores++;
	}

	return ncores;
}

/* returns:
	 number of values parsed
	-1 in case of error
*/
static int
app_parse_opt_vals(const char *conf_str, char separator, uint32_t n_vals, uint32_t *opt_vals)
{
	char *string;
	int i, n_tokens;
	char *tokens[MAX_OPT_VALUES];

	if (conf_str == NULL || opt_vals == NULL || n_vals == 0 || n_vals > MAX_OPT_VALUES)
		return -1;

	/* duplicate configuration string before splitting it to tokens */
	string = strdup(conf_str);
	if (string == NULL)
		return -1;

	n_tokens = rte_strsplit(string, strnlen(string, 32), tokens, n_vals, separator);

	if (n_tokens > MAX_OPT_VALUES)
		return -1;

	for (i = 0; i < n_tokens; i++)
		opt_vals[i] = (uint32_t)atol(tokens[i]);

	free(string);

	return n_tokens;
}

static int
app_parse_flow_conf(const char *conf_str)
{
	int ret;
	uint32_t vals[5];
	struct flow_conf *pconf;
	uint64_t mask;

	ret = app_parse_opt_vals(conf_str, ',', 6, vals);
	if (ret < 4 || ret > 5)
		return ret;

	pconf = &qos_conf[nb_pfc];

	pconf->rx_port = (uint8_t)vals[0];
	pconf->tx_port = (uint8_t)vals[1];
	pconf->rx_core = (uint8_t)vals[2];
	pconf->wt_core = (uint8_t)vals[3];
	if (ret == 5)
		pconf->tx_core = (uint8_t)vals[4];
	else
		pconf->tx_core = pconf->wt_core;

	if (pconf->rx_core == pconf->wt_core) {
		RTE_LOG(ERR, APP, "pfc %u: rx thread and worker thread cannot share same core\n", nb_pfc);
		return -1;
	}

	if (pconf->rx_port >= RTE_MAX_ETHPORTS) {
		RTE_LOG(ERR, APP, "pfc %u: invalid rx port %"PRIu8" index\n",
				nb_pfc, pconf->rx_port);
		return -1;
	}
	if (pconf->tx_port >= RTE_MAX_ETHPORTS) {
		RTE_LOG(ERR, APP, "pfc %u: invalid tx port %"PRIu8" index\n",
				nb_pfc, pconf->tx_port);
		return -1;
	}

	mask = 1lu << pconf->rx_port;
	if (app_used_rx_port_mask & mask) {
		RTE_LOG(ERR, APP, "pfc %u: rx port %"PRIu8" is used already\n",
				nb_pfc, pconf->rx_port);
		return -1;
	}
	app_used_rx_port_mask |= mask;

	mask = 1lu << pconf->tx_port;
	if (app_used_tx_port_mask & mask) {
		RTE_LOG(ERR, APP, "pfc %u: port %"PRIu8" is used already\n",
				nb_pfc, pconf->tx_port);
		return -1;
	}
	app_used_tx_port_mask |= mask;

	mask = 1lu << pconf->rx_core;
	app_used_core_mask |= mask;

	mask = 1lu << pconf->wt_core;
	app_used_core_mask |= mask;

	mask = 1lu << pconf->tx_core;
	app_used_core_mask |= mask;

	nb_pfc++;

	return 0;
}

/*
 * Parses the argument given in the command line of the application,
 * calculates mask for used cores and initializes EAL with calculated core mask
 */
int
app_parse_args(int argc, char **argv)
{
	int opt, ret;
	int option_index;
	const char *optname;
	char *prgname = argv[0];
	uint32_t i, nb_lcores;

	static struct option lgopts[] = {
		/*
		 * Packet flow configuration: which ports and lcores to
		 * use for receiving, transmitting, working.
		 */
		{ "pfc", 1, 0, 0 },
		{ NULL,  0, 0, 0 }
	};

	/* initialize EAL first */
	ret = rte_eal_init(argc, argv);
	if (ret < 0)
		return -1;

	argc -= ret;
	argv += ret;

	/* set en_US locale to print big numbers with ',' */
	setlocale(LC_NUMERIC, "en_US.utf-8");

	while ((opt = getopt_long(argc, argv, "i",
		lgopts, &option_index)) != EOF) {

			switch (opt) {
			/* long options */
			case 0:
				optname = lgopts[option_index].name;
				if (str_is(optname, "pfc")) {
					ret = app_parse_flow_conf(optarg);
					if (ret) {
						RTE_LOG(ERR, APP, "Invalid pipe configuration %s\n", optarg);
						return -1;
					}
					break;
				}
				break;

			default:
				app_usage(prgname);
				return -1;
			}
	}

	/* check master core index validity */
	for(i = 0; i <= APP_MASTER_CORE; i++) {
		if (app_used_core_mask & (1u << APP_MASTER_CORE)) {
			RTE_LOG(ERR, APP, "Master core index is not configured properly\n");
			app_usage(prgname);
			return -1;
		}
	}
	app_used_core_mask |= 1u << APP_MASTER_CORE;

	if ((app_used_core_mask != app_eal_core_mask()) ||
			(APP_MASTER_CORE != rte_get_master_lcore())) {
		RTE_LOG(ERR, APP, "EAL core mask not configured properly, must be %" PRIx64
				" instead of %" PRIx64 "\n" , app_used_core_mask, app_eal_core_mask());
		return -1;
	}

	if (nb_pfc == 0) {
		RTE_LOG(ERR, APP, "Packet flow not configured!\n");
		app_usage(prgname);
		return -1;
	}

	/* sanity check for cores assignment */
	nb_lcores = app_cpu_core_count();

	for(i = 0; i < nb_pfc; i++) {
		if (qos_conf[i].rx_core >= nb_lcores) {
			RTE_LOG(ERR, APP, "pfc %u: invalid RX lcore index %u\n", i + 1,
					qos_conf[i].rx_core);
			return -1;
		}
		if (qos_conf[i].wt_core >= nb_lcores) {
			RTE_LOG(ERR, APP, "pfc %u: invalid WT lcore index %u\n", i + 1,
					qos_conf[i].wt_core);
			return -1;
		}
		uint32_t rx_sock = rte_lcore_to_socket_id(qos_conf[i].rx_core);
		uint32_t wt_sock = rte_lcore_to_socket_id(qos_conf[i].wt_core);
		if (rx_sock != wt_sock) {
			RTE_LOG(ERR, APP, "pfc %u: RX and WT must be on the same socket\n", i + 1);
			return -1;
		}
	}

	return 0;
}
