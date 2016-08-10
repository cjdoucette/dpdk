#include <rte_string_fns.h>
#include <rte_malloc.h>
#include <getopt.h>

#include "kni.h"
#include "common.h"
#include "cli.h"

/* Display usage instructions */
static void
print_usage(const char *prgname)
{
	RTE_LOG(INFO, APP, "\nUsage: %s [EAL options] -- -p PORTMASK -P "
		   "[--config (port,lcore_rx,lcore_tx,lcore_kthread...)"
		   "[,(port,lcore_rx,lcore_tx,lcore_kthread...)]]\n"
		   "    -p PORTMASK: hex bitmask of ports to use\n"
		   "    -P : enable promiscuous mode\n"
		   "    --config (port,lcore_rx,lcore_tx,lcore_kthread...): "
		   "port and lcore configurations\n",
	           prgname);
}

static int
parse_config(const char *arg)
{
	const char *p, *p0 = arg;
	char s[256], *end;
	unsigned size;
	enum fieldnames {
		FLD_PORT = 0,
		FLD_LCORE_RX,
		FLD_LCORE_TX,
		_NUM_FLD = KNI_MAX_KTHREAD + 3,
	};
	int i, j, nb_token;
	char *str_fld[_NUM_FLD];
	unsigned long int_fld[_NUM_FLD];
	uint8_t port_id, nb_kni_port_params = 0;

	memset(&kni_port_params_array, 0, sizeof(kni_port_params_array));
	while (((p = strchr(p0, '(')) != NULL) &&
		nb_kni_port_params < RTE_MAX_ETHPORTS) {
		p++;
		if ((p0 = strchr(p, ')')) == NULL)
			goto fail;
		size = p0 - p;
		if (size >= sizeof(s)) {
			printf("Invalid config parameters\n");
			goto fail;
		}
		snprintf(s, sizeof(s), "%.*s", size, p);
		nb_token = rte_strsplit(s, sizeof(s), str_fld, _NUM_FLD, ',');
		if (nb_token <= FLD_LCORE_TX) {
			printf("Invalid config parameters\n");
			goto fail;
		}
		for (i = 0; i < nb_token; i++) {
			errno = 0;
			int_fld[i] = strtoul(str_fld[i], &end, 0);
			if (errno != 0 || end == str_fld[i]) {
				printf("Invalid config parameters\n");
				goto fail;
			}
		}

		i = 0;
		port_id = (uint8_t)int_fld[i++];
		if (port_id >= RTE_MAX_ETHPORTS) {
			printf("Port ID %d could not exceed the maximum %d\n",
						port_id, RTE_MAX_ETHPORTS);
			goto fail;
		}
		if (kni_port_params_array[port_id]) {
			printf("Port %d has been configured\n", port_id);
			goto fail;
		}
		kni_port_params_array[port_id] =
			rte_zmalloc("KNI_port_params",
				    sizeof(struct kni_port_params), RTE_CACHE_LINE_SIZE);
		kni_port_params_array[port_id]->port_id = port_id;
		kni_port_params_array[port_id]->lcore_rx =
					(uint8_t)int_fld[i++];
		kni_port_params_array[port_id]->lcore_tx =
					(uint8_t)int_fld[i++];
		if (kni_port_params_array[port_id]->lcore_rx >= RTE_MAX_LCORE ||
		kni_port_params_array[port_id]->lcore_tx >= RTE_MAX_LCORE) {
			printf("lcore_rx %u or lcore_tx %u ID could not "
						"exceed the maximum %u\n",
				kni_port_params_array[port_id]->lcore_rx,
				kni_port_params_array[port_id]->lcore_tx,
						(unsigned)RTE_MAX_LCORE);
			goto fail;
		}
		for (j = 0; i < nb_token && j < KNI_MAX_KTHREAD; i++, j++)
			kni_port_params_array[port_id]->lcore_k[j] =
						(uint8_t)int_fld[i];
		kni_port_params_array[port_id]->nb_lcore_k = j;
	}

	return 0;

fail:
	for (i = 0; i < RTE_MAX_ETHPORTS; i++) {
		if (kni_port_params_array[i]) {
			rte_free(kni_port_params_array[i]);
			kni_port_params_array[i] = NULL;
		}
	}

	return -1;
}

#define CMDLINE_OPT_CONFIG  "config"

/* Parse the arguments given in the command line of the application */
int
parse_args(int argc, char **argv)
{
	int opt, longindex, ret = 0;
	const char *prgname = argv[0];
	static struct option longopts[] = {
		{CMDLINE_OPT_CONFIG, required_argument, NULL, 0},
		{NULL, 0, NULL, 0}
	};

	/* Disable printing messages within getopt() */
	opterr = 0;

	/* Parse command line */
	while ((opt = getopt_long(argc, argv, "p:P", longopts,
						&longindex)) != EOF) {
		switch (opt) {
		case 0:
			if (!strncmp(longopts[longindex].name,
				     CMDLINE_OPT_CONFIG,
				     sizeof(CMDLINE_OPT_CONFIG))) {
				ret = parse_config(optarg);
				if (ret) {
					printf("Invalid config\n");
					print_usage(prgname);
					return -1;
				}
			}
			break;
		default:
			print_usage(prgname);
			rte_exit(EXIT_FAILURE, "Invalid option specified\n");
		}
	}

	return ret;
}


