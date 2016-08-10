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

#include <rte_ethdev.h>
#include <rte_malloc.h>
#include <rte_cycles.h>

#include "main.h"
#include "dst.h"

//#define SYS_CPU_DIR "/sys/devices/system/cpu/cpu%u/topology/"

static const struct rte_eth_conf port_conf = {
	.rxmode = {
		.max_rx_pkt_len = ETHER_MAX_LEN,
		.split_hdr_size = 0,
		.header_split   = 0, /**< Header Split disabled */
		.hw_ip_checksum = 0, /**< IP checksum offload disabled */
		.hw_vlan_filter = 0, /**< VLAN filtering disabled */
		.jumbo_frame    = 0, /**< Jumbo Frame Support disabled */
		.hw_strip_crc   = 0, /**< CRC stripped by hardware */
	},
	.txmode = {
		.mq_mode = ETH_DCB_NONE,
	},
};

#if 0
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
#endif

static int
port_init(struct gk_data *gk)
{
	int ret;
	struct rte_eth_link link;
	struct rte_eth_rxconf rx_conf;
	struct rte_eth_txconf tx_conf;

	rx_conf.rx_thresh.pthresh = gk->rx_pthresh;
	rx_conf.rx_thresh.hthresh = gk->rx_hthresh;
	rx_conf.rx_thresh.wthresh = gk->rx_wthresh;
	rx_conf.rx_free_thresh = 32;
	rx_conf.rx_drop_en = 0;

	tx_conf.tx_thresh.pthresh = gk->tx_pthresh;
	tx_conf.tx_thresh.hthresh = gk->tx_hthresh;
	tx_conf.tx_thresh.wthresh = gk->tx_wthresh;
	tx_conf.tx_free_thresh = 0;
	tx_conf.tx_rs_thresh = 0;
	tx_conf.txq_flags = ETH_TXQ_FLAGS_NOMULTSEGS | ETH_TXQ_FLAGS_NOOFFLOADS;

	/* init port */
	RTE_LOG(INFO, APP, "Initializing port %"PRIu8"... ", gk->rx_port);
	fflush(stdout);
	ret = rte_eth_dev_configure(gk->rx_port, 1, 1, &port_conf);
	if (ret < 0) {
		printf("Cannot configure device: "
			"err=%d, port=%"PRIu8"\n", ret, gk->rx_port);
		return -1;
	}

	fflush(stdout);
	ret = rte_eth_rx_queue_setup(gk->rx_port, gk->rx_queue,
		gk->rx_queue_size, gk->socket, &rx_conf, gk->mbuf_pool);
	if (ret < 0) {
		printf("rte_eth_tx_queue_setup: err=%d, port=%"PRIu8"\n",
			ret, gk->rx_port);
		return -1;
	}

	fflush(stdout);
	ret = rte_eth_tx_queue_setup(gk->tx_port, gk->tx_queue,
		gk->tx_queue_size, gk->socket, &tx_conf);
	if (ret < 0) {
		printf("rte_eth_tx_queue_setup: err=%d, "
			"port=%"PRIu8" queue=%d\n", ret, gk->tx_port,
			gk->tx_queue);
		return -1;
	}

	ret = rte_eth_dev_start(gk->rx_port);
	if (ret < 0) {
		printf("rte_pmd_port_start: err=%d, port=%"PRIu8"\n",
			ret, gk->rx_port);
		return -1;
	}

	rte_eth_link_get(gk->rx_port, &link);
	if (link.link_status) {
		printf(" RX link Up - speed %u Mbps - %s\n",
			(uint32_t)link.link_speed,
			(link.link_duplex == ETH_LINK_FULL_DUPLEX) ?
			("full-duplex") : ("half-duplex\n"));
	} else {
		printf(" RX link Down\n");
	}
	rte_eth_promiscuous_enable(gk->rx_port);

	if (gk->rx_port != gk->tx_port) {
		ret = rte_eth_dev_start(gk->tx_port);
		if (ret < 0)
			rte_exit(EXIT_FAILURE, "rte_pmd_port_start: "
				"err=%d, port=%"PRIu8"\n", ret, gk->tx_port);
		rte_eth_link_get(gk->rx_port, &link);
		if (link.link_status) {
			printf(" TX link Up - speed %u Mbps - %s\n",
				(uint32_t)link.link_speed,
				(link.link_duplex == ETH_LINK_FULL_DUPLEX) ?
				("full-duplex") : ("half-duplex\n"));
		} else {
			printf(" TX link Down\n");
		}
		rte_eth_promiscuous_enable(gk->tx_port);
	}

	return 0;
}

enum dst_queues_pos {
	e_DST_QUEUES_QUEUE_META,
	e_DST_QUEUES_BITMAP,
	e_DST_QUEUES_QUEUE_ARRAY,
	e_DST_QUEUES_TOTAL
};

static uint32_t
dst_queues_offset(struct queues_conf *conf, enum dst_queues_pos pos)
{
	uint32_t n_queues = conf->num_queues;
	uint32_t size_queue_meta = n_queues * sizeof(struct gk_queue);
	uint32_t size_bmp_array = rte_bitmap_get_memory_footprint(n_queues);
	uint32_t size_queue = conf->qsize * sizeof(struct rte_mbuf *);
	uint32_t size_queue_array = n_queues * size_queue;

	/*
	 * Add the size of each queue's metadata,
	 * the bitmap array, and the actual queues.
	 */
	uint32_t base = 0;

	if (pos == e_DST_QUEUES_QUEUE_META)
		return base;
	base += RTE_CACHE_LINE_ROUNDUP(size_queue_meta);

	if (pos == e_DST_QUEUES_BITMAP)
		return base;
	base += RTE_CACHE_LINE_ROUNDUP(size_bmp_array);

	if (pos == e_DST_QUEUES_QUEUE_ARRAY)
		return base;
	base += RTE_CACHE_LINE_ROUNDUP(size_queue_array);

	return base;
}

static inline uint32_t
dst_queues_mem_size(struct queues_conf *conf)
{
	return sizeof(struct dst_queues) +
		dst_queues_offset(conf, e_DST_QUEUES_TOTAL);
}

static struct dst_queues *
dst_queues_init(struct gk_data *gk, struct queues_conf *conf)
{
	struct dst_queues *dst_queues = NULL;
	uint32_t mem_size, bmp_mem_size, cycles_per_byte;

	mem_size = dst_queues_mem_size(conf);
	if (mem_size == 0)
		return NULL;

	dst_queues = rte_zmalloc("dst_queues", mem_size, RTE_CACHE_LINE_SIZE);
	if (dst_queues == NULL) {
		printf("Could not allocate dst_queues\n");
		return NULL;
	}

	/* User parameters. */
	dst_queues->rate = 0;
	dst_queues->mtu = conf->mtu;
	dst_queues->frame_overhead = conf->frame_overhead;
	dst_queues->qsize = conf->qsize;
	dst_queues->num_queues = conf->num_queues;
	dst_queues->cur_queue = 0;

	/* Token bucket. */
	dst_queues->tb_time = 0;
	/* XXX Enforce that this is 95%. */
	dst_queues->tb_period = conf->tb_period;
	dst_queues->tb_credits_per_period = conf->tb_credits_per_period;
	dst_queues->tb_size = conf->tb_size;
	dst_queues->tb_credits = conf->tb_size / 2;

	/* Timing. */
	dst_queues->time_cpu_cycles = rte_get_tsc_cycles();
	dst_queues->time_cpu_bytes = 0;
	dst_queues->time = 0;

	cycles_per_byte = (rte_get_tsc_hz() << RTE_SCHED_TIME_SHIFT)
		/ gk->rate;
	dst_queues->inv_cycles_per_byte = rte_reciprocal_value(cycles_per_byte);

	dst_queues->pkts_out = NULL;
	dst_queues->n_pkts_out = 0;

	dst_queues->queue = (struct gk_queue *)
		(dst_queues->memory +
		dst_queues_offset(conf, e_DST_QUEUES_QUEUE_META));

	dst_queues->bmp_array = dst_queues->memory +
		dst_queues_offset(conf, e_DST_QUEUES_BITMAP);

	dst_queues->queue_array = (struct rte_mbuf **)
		(dst_queues->memory +
		dst_queues_offset(conf, e_DST_QUEUES_QUEUE_ARRAY));

	bmp_mem_size = rte_bitmap_get_memory_footprint(conf->num_queues);
	dst_queues->bmp = rte_bitmap_init(conf->num_queues,
		dst_queues->bmp_array, bmp_mem_size);
	if (dst_queues->bmp == NULL) {
		printf("bitmap init error\n");
		return NULL;
	}

	return dst_queues;
}

enum req_queue_pos {
	e_REQ_QUEUE_BITMAP,
	e_REQ_QUEUE_TOTAL
};

static uint32_t
req_queue_offset(const uint16_t num_priorities, enum req_queue_pos pos)
{
	uint32_t size_bmp_array =
		rte_bitmap_get_memory_footprint(num_priorities);

	/*
	 * Add the size of each queue's metadata,
	 * the bitmap array, and the actual queues.
	 */
	uint32_t base = 0;

	if (pos == e_REQ_QUEUE_BITMAP)
		return base;
	base += RTE_CACHE_LINE_ROUNDUP(size_bmp_array);

	return base;
}

static inline uint32_t
req_queue_mem_size(const uint16_t num_priorities)
{
	return sizeof(struct req_queue) +
		req_queue_offset(num_priorities, e_REQ_QUEUE_TOTAL);
}

static struct req_queue *
req_queue_init(struct gk_data *gk, struct queues_conf *conf)
{
	struct req_queue *req_queue = NULL;
	uint32_t mem_size, bmp_mem_size, cycles_per_byte;

	mem_size = req_queue_mem_size(conf->num_queues);
	if (mem_size == 0)
		return NULL;

	req_queue = rte_zmalloc("req_queue", mem_size, RTE_CACHE_LINE_SIZE);
	if (req_queue == NULL) {
		printf("Could not allocate req_queue\n");
		return NULL;
	}

	/* User parameters. */
	req_queue->rate = 0;
	req_queue->mtu = conf->mtu;
	req_queue->frame_overhead = conf->frame_overhead;
	req_queue->qsize = conf->qsize;
	req_queue->length = 0;
	req_queue->num_priorities = conf->num_queues;
	req_queue->highest_priority = 0;
	req_queue->lowest_priority = conf->num_queues;

	/* Token bucket. */
	req_queue->tb_time = 0;
	/* XXX Enforce that this is 5%. */
	req_queue->tb_period = conf->tb_period;
	req_queue->tb_credits_per_period = conf->tb_credits_per_period;
	req_queue->tb_size = conf->tb_size;
	req_queue->tb_credits = conf->tb_size / 2;

	/* Timing. */
	req_queue->time_cpu_cycles = rte_get_tsc_cycles();
	req_queue->time_cpu_bytes = 0;
	req_queue->time = 0;

	cycles_per_byte = (rte_get_tsc_hz() << RTE_SCHED_TIME_SHIFT)
		/ gk->rate;
	req_queue->inv_cycles_per_byte = rte_reciprocal_value(cycles_per_byte);

	req_queue->pkts_out = NULL;
	req_queue->n_pkts_out = 0;

	req_queue->head = NULL;

	req_queue->bmp_array = req_queue->memory +
		req_queue_offset(conf->num_queues, e_DST_QUEUES_BITMAP);

	bmp_mem_size =
		rte_bitmap_get_memory_footprint(req_queue->num_priorities);
	req_queue->bmp = rte_bitmap_init(req_queue->num_priorities,
		req_queue->bmp_array, bmp_mem_size);
	if (req_queue->bmp == NULL) {
		printf("bitmap init error\n");
		return NULL;
	}

	return req_queue;
}

int
gk_init(struct gk_conf *gk_conf, struct gk_data *gk, unsigned rx_burst_size)
{
	struct rte_eth_link link;
	uint32_t socket = rte_lcore_to_socket_id(gk_conf->rx_core);

	gk->mbuf_pool = rte_pktmbuf_pool_create("mbuf_pool",
		gk_conf->mbuf_pool_size, 4 * rx_burst_size, 0,
		RTE_MBUF_DEFAULT_BUF_SIZE,
		rte_eth_dev_socket_id(gk_conf->rx_port));
	if (gk->mbuf_pool == NULL) {
		printf("Could not allocate mbuf pool\n");
		return -1;
	}

	gk->req_rx_ring = rte_ring_create("req_rx_ring", gk_conf->rx_ring_size,
		socket, RING_F_SP_ENQ | RING_F_SC_DEQ);
	if (gk->req_rx_ring == NULL) {
		printf("Could not allocate req rx ring\n");
		return -1;
	}

	gk->req_tx_ring = rte_ring_create("req_tx_ring", gk_conf->tx_ring_size,
		socket, RING_F_SP_ENQ | RING_F_SC_DEQ);
	if (gk->req_tx_ring == NULL) {
		printf("Could not allocate req tx ring\n");
		return -1;
	}

	gk->dst_rx_ring = rte_ring_create("dst_rx_ring", gk_conf->rx_ring_size,
		socket, RING_F_SP_ENQ | RING_F_SC_DEQ);
	if (gk->dst_rx_ring == NULL) {
		printf("Could not allocate dst rx ring\n");
		return -1;
	}

	gk->dst_tx_ring = rte_ring_create("dst_tx_ring", gk_conf->tx_ring_size,
		socket, RING_F_SP_ENQ | RING_F_SC_DEQ);
	if (gk->dst_tx_ring == NULL) {
		printf("Could not allocate dst tx ring\n");
		return -1;
	}

	gk->m_table = rte_malloc("m_table",
		sizeof(struct rte_mbuf *) * gk_conf->tx_burst_size,
		RTE_CACHE_LINE_SIZE);
	if (gk->m_table == NULL) {
		printf("Could not allocate m_table\n");
		return -1;
	}
	gk->n_mbufs = 0;

	gk->counter = 0;
	gk->socket = socket;

	/* Convert from Mbps to Bps. */
	rte_eth_link_get(gk->rx_port, &link);
	gk->rate = (uint64_t)link.link_speed * 1000 * 1000 / 8;

	gk->frame_overhead = gk_conf->frame_overhead;

	gk->rx_burst_size = gk_conf->rx_burst_size;
	gk->qos_enqueue_size = gk_conf->qos_enqueue_size;
	gk->qos_dequeue_size = gk_conf->qos_dequeue_size;
	gk->tx_burst_size = gk_conf->tx_burst_size;

	gk->rx_ring_size = gk_conf->rx_ring_size;
	gk->tx_ring_size = gk_conf->tx_ring_size;

	gk->rx_queue_size = gk_conf->rx_queue_size;
	gk->tx_queue_size = gk_conf->tx_queue_size;

	gk->rx_pthresh = gk_conf->rx_pthresh;
	gk->rx_hthresh = gk_conf->rx_hthresh;
	gk->rx_wthresh = gk_conf->rx_wthresh;

	gk->tx_pthresh = gk_conf->tx_pthresh;
	gk->tx_hthresh = gk_conf->tx_hthresh;
	gk->tx_wthresh = gk_conf->tx_wthresh;

	gk->rx_port = gk_conf->rx_port;
	gk->tx_port = gk_conf->tx_port;
	gk->rx_queue = gk_conf->rx_queue;
	gk->tx_queue = gk_conf->tx_queue;

	gk->wk_req_core = gk_conf->wk_req_core;
	gk->wk_dst_core = gk_conf->wk_dst_core;
	gk->rx_core = gk_conf->rx_core;

	return 0;
}

int
queues_init(struct gk_data *gk, struct queues_conf *req_conf,
	struct queues_conf *dst_conf, struct req_queue **req_queue,
	struct dst_queues **dst_queues)
{
	if (rte_eth_dev_count() == 0) {
		printf("No Ethernet port\n");
		return -1;
	}

	*dst_queues = dst_queues_init(gk, dst_conf);
	if (dst_queues == NULL)
		return -1;

	*req_queue = req_queue_init(gk, req_conf);
	if (req_queue == NULL)
		return -1;

	if (port_init(gk) < 0)
		return -1;

#if 0
	RTE_LOG(INFO, APP, "time stamp clock running at %" PRIu64 " Hz\n",
			 rte_get_timer_hz());

	RTE_LOG(INFO, APP, "Ring sizes: NIC RX = %u, Mempool = %d "
		"REQ queue = %u, DST queue = %u. NIC TX = %u\n",
		gk->rx_queue_size, gk->mbuf_pool_size,
		gk->rx_ring_size, gl->rx_ring_size,
		gk->tx_queue_size);

	RTE_LOG(INFO, APP, "Req burst sizes: RX read = %hu, RX write = %hu,\n"
		"Worker read/QoS enqueue = %hu,\n"
		"QoS dequeue = %hu, Worker write = %hu\n",
		req_conf->rx_burst_size, req_conf->qos_enqueue_size,
		req_conf->qos_enqueue_size,
		req_conf->qos_dequeue_size, req_conf->tx_burst_size);

	RTE_LOG(INFO, APP, "Req burst sizes: RX read = %hu, RX write = %hu,\n"
		"Worker read/QoS enqueue = %hu,\n"
		"QoS dequeue = %hu, Worker write = %hu\n",
		pri_conf->rx_burst_size, pri_conf->qos_enqueue_size,
		pri_conf->qos_enqueue_size,
		pri_conf->qos_dequeue_size, pri_conf->tx_burst_size);

	RTE_LOG(INFO, APP, "NIC thresholds RX (p = %hhu, h = %hhu, w = %hhu),"
		"TX (p = %hhu, h = %hhu, w = %hhu)\n",
		app_conf->rx_pthresh, app_conf->rx_hthresh,
		app_conf->rx_wthresh, app_conf->tx_pthresh,
		app_conf->tx_hthresh, app_conf->tx_wthresh);
#endif
	return 0;
}
