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

#include <rte_cycles.h>
#include <rte_ethdev.h>

#include "dst.h"

/*
 * Send the packets to an output interface.
 */
void
dst_send_burst(struct gk_data *gk, struct dst_queues *dst_queues)
{
	uint16_t ret;

	do {
		ret = rte_eth_tx_burst(gk->tx_port, gk->tx_queue,
			dst_queues->pkts_out, dst_queues->n_pkts_out);

		/* We cannot drop the packets, so re-send. */
		dst_queues->n_pkts_out -= ret;
		dst_queues->pkts_out += ret;
	} while (dst_queues->n_pkts_out);
}

/*
 * Based on rte_sched_port_enqueue_qptrs_prefetch0().
 * This version is simpler, since there are no subports,
 * pipes, or traffic classes, makingt the queue ID the
 * same as the offset into the queue array.
 */
static inline uint32_t
enqueue_qptrs_prefetch0(struct dst_queues *dst_queues, struct rte_mbuf *pkt)
{
	uint32_t type, qindex;
	/* XXX Should we still store the type and queue with each packet? */
	pkt_read_tree_path(pkt, &type, &qindex);
	rte_prefetch0(dst_queues->queue + qindex);
	return qindex;
}

/*
 * Based on rte_sched_port_qbase().
 * This version is simpler, since we don't have to break
 * the queue index down into the pipe index and queue position.
 */
static inline struct rte_mbuf **
dst_qbase(struct dst_queues *dst_queues, uint32_t qindex)
{
	return dst_queues->queue_array + (qindex * dst_queues->qsize);
}

/*
 * Based on rte_sched_port_enqueue_qwa_prefetch0().
 */
static inline void
enqueue_qwa_prefetch0(struct dst_queues *dst_queues, uint32_t qindex,
	struct rte_mbuf **qbase)
{
	struct gk_queue *q;
	struct rte_mbuf **q_qw;

	q = dst_queues->queue + qindex;
	q_qw = qbase + (q->qw & (dst_queues->qsize - 1));

	rte_prefetch0(q_qw);
	rte_bitmap_prefetch0(dst_queues->bmp, qindex);
}

/*
 * Based on rte_sched_port_enqueue_qwa().
 */
static inline int
enqueue_qwa(struct dst_queues *dst_queues, uint32_t qindex,
	struct rte_mbuf **qbase, struct rte_mbuf *pkt)
{
	struct gk_queue *q;
	uint16_t qlen;

	q = dst_queues->queue + qindex;
	qlen = q->qw - q->qr;

	/* XXX Add RED drop here. */
	if (unlikely(qlen >= dst_queues->qsize)) {
		rte_pktmbuf_free(pkt);
		return 0;
	}

	/* Enqueue packet. */
	qbase[q->qw & (dst_queues->qsize - 1)] = pkt;
	q->qw++;

	/* Activate queue in the bitmap. */
	rte_bitmap_set(dst_queues->bmp, qindex);

	return 1;
}

int
dst_enqueue(struct dst_queues *dst_queues, struct rte_mbuf **pkts,
	uint32_t n_pkts)
{
	struct rte_mbuf *pkt00, *pkt01, *pkt10, *pkt11, *pkt20, *pkt21,
		*pkt30, *pkt31, *pkt_last;
	struct rte_mbuf **q00_base, **q01_base, **q10_base, **q11_base,
		**q20_base, **q21_base, **q30_base, **q31_base, **q_last_base;
	uint32_t q00, q01, q10, q11, q20, q21, q30, q31, q_last;
	uint32_t r00, r01, r10, r11, r20, r21, r30, r31, r_last;
	uint32_t result, i;

	result = 0;

	/*
	 * Less then 6 input packets available, which
	 * is not enough to feed the pipeline.
	 */
	if (unlikely(n_pkts < 6)) {
		struct rte_mbuf **q_base[5];
		uint32_t q[5];

		/* Prefetch the mbuf structure of each packet. */
		for (i = 0; i < n_pkts; i++)
			rte_prefetch0(pkts[i]);

		/* Prefetch the queue metadata structure for each queue. */
		for (i = 0; i < n_pkts; i++)
			q[i] = enqueue_qptrs_prefetch0(dst_queues, pkts[i]);

		/* Prefetch the write pointer location of each queue. */
		for (i = 0; i < n_pkts; i++) {
			q_base[i] = dst_qbase(dst_queues, q[i]);
			enqueue_qwa_prefetch0(dst_queues, q[i], q_base[i]);
		}

		/* Write each packet to its queue */
		for (i = 0; i < n_pkts; i++)
			result += enqueue_qwa(dst_queues, q[i],
				q_base[i], pkts[i]);

		return result;
	}

	/* Feed the first 3 stages of the pipeline (6 packets needed). */
	pkt20 = pkts[0];
	pkt21 = pkts[1];
	rte_prefetch0(pkt20);
	rte_prefetch0(pkt21);

	pkt10 = pkts[2];
	pkt11 = pkts[3];
	rte_prefetch0(pkt10);
	rte_prefetch0(pkt11);

	q20 = enqueue_qptrs_prefetch0(dst_queues, pkt20);
	q21 = enqueue_qptrs_prefetch0(dst_queues, pkt21);

	pkt00 = pkts[4];
	pkt01 = pkts[5];
	rte_prefetch0(pkt00);
	rte_prefetch0(pkt01);

	q10 = enqueue_qptrs_prefetch0(dst_queues, pkt10);
	q11 = enqueue_qptrs_prefetch0(dst_queues, pkt11);

	q20_base = dst_qbase(dst_queues, q20);
	q21_base = dst_qbase(dst_queues, q21);
	enqueue_qwa_prefetch0(dst_queues, q20, q20_base);
	enqueue_qwa_prefetch0(dst_queues, q21, q21_base);

	/* Run the pipeline. */
	for (i = 6; i < (n_pkts & (~1)); i += 2) {
		/* Propagate stage inputs. */
		pkt30 = pkt20;
		pkt31 = pkt21;
		pkt20 = pkt10;
		pkt21 = pkt11;
		pkt10 = pkt00;
		pkt11 = pkt01;
		q30 = q20;
		q31 = q21;
		q20 = q10;
		q21 = q11;
		q30_base = q20_base;
		q31_base = q21_base;

		/* Stage 0: Get packets in. */
		pkt00 = pkts[i];
		pkt01 = pkts[i + 1];
		rte_prefetch0(pkt00);
		rte_prefetch0(pkt01);

		/* Stage 1: Prefetch queue structure storing queue pointers. */
		q10 = enqueue_qptrs_prefetch0(dst_queues, pkt10);
		q11 = enqueue_qptrs_prefetch0(dst_queues, pkt11);

		/* Stage 2: Prefetch queue write location. */
		q20_base = dst_qbase(dst_queues, q20);
		q21_base = dst_qbase(dst_queues, q21);
		enqueue_qwa_prefetch0(dst_queues, q20, q20_base);
		enqueue_qwa_prefetch0(dst_queues, q21, q21_base);

		/* Stage 3: Write packet to queue and activate queue. */
		r30 = enqueue_qwa(dst_queues, q30, q30_base, pkt30);
		r31 = enqueue_qwa(dst_queues, q31, q31_base, pkt31);
		result += r30 + r31;
	}

	/*
	 * Drain the pipeline (exactly 6 packets).
	 * Handle the last packet in the case
	 * of an odd number of input packets.
	 */
	pkt_last = pkts[n_pkts - 1];
	rte_prefetch0(pkt_last);

	q00 = enqueue_qptrs_prefetch0(dst_queues, pkt00);
	q01 = enqueue_qptrs_prefetch0(dst_queues, pkt01);

	q10_base = dst_qbase(dst_queues, q10);
	q11_base = dst_qbase(dst_queues, q11);
	enqueue_qwa_prefetch0(dst_queues, q10, q10_base);
	enqueue_qwa_prefetch0(dst_queues, q11, q11_base);

	r20 = enqueue_qwa(dst_queues, q20, q20_base, pkt20);
	r21 = enqueue_qwa(dst_queues, q21, q21_base, pkt21);
	result += r20 + r21;

	q_last = enqueue_qptrs_prefetch0(dst_queues, pkt_last);

	q00_base = dst_qbase(dst_queues, q00);
	q01_base = dst_qbase(dst_queues, q01);
	enqueue_qwa_prefetch0(dst_queues, q00, q00_base);
	enqueue_qwa_prefetch0(dst_queues, q01, q01_base);

	r10 = enqueue_qwa(dst_queues, q10, q10_base, pkt10);
	r11 = enqueue_qwa(dst_queues, q11, q11_base, pkt11);
	result += r10 + r11;

	q_last_base = dst_qbase(dst_queues, q_last);
	enqueue_qwa_prefetch0(dst_queues, q_last, q_last_base);

	r00 = enqueue_qwa(dst_queues, q00, q00_base, pkt00);
	r01 = enqueue_qwa(dst_queues, q01, q01_base, pkt01);
	result += r00 + r01;

	if (n_pkts & 1) {
		r_last = enqueue_qwa(dst_queues, q_last, q_last_base, pkt_last);
		result += r_last;
	}

	return result;
}

static inline int
credits_check(struct dst_queues *dst_queues, struct rte_mbuf *pkt)
{
	uint32_t pkt_len = pkt->pkt_len + dst_queues->frame_overhead;

	if (pkt_len > dst_queues->tb_credits)
		return 0;

	dst_queues->tb_credits -= pkt_len;
	return 1;
}

static void
__dst_dequeue(struct dst_queues *dst_queues, uint32_t num_pkts)
{
	struct gk_queue *q = dst_queues->queue + dst_queues->cur_queue;
	struct rte_mbuf **qbase = dst_qbase(dst_queues, dst_queues->cur_queue);
	uint16_t qr;

	do {
		qr = q->qr & (dst_queues->qsize - 1);

		/* Check credits. */
		if (!credits_check(dst_queues, qbase[qr]))
			return;

		/* Queue packet. */
		dst_queues->pkts_out[dst_queues->n_pkts_out++] = qbase[qr];

		/* Advance port time. */
		dst_queues->time += qbase[qr]->pkt_len +
			dst_queues->frame_overhead;

		/* Update number of slots left. */
		num_pkts--;
		/* Update queue metadata. */
		q->qr++;
	} while (q->qr != q->qw && num_pkts > 0);

	if (q->qr == q->qw) {
		rte_bitmap_clear(dst_queues->bmp, dst_queues->cur_queue);
		/* XXX rte_sched_port_set_queue_empty_timestamp(). */
	} 
}

static inline void
time_resync(struct dst_queues *dst_queues)
{
	uint64_t cycles = rte_get_tsc_cycles();
	uint64_t cycles_diff = cycles - dst_queues->time_cpu_cycles;
	uint64_t bytes_diff;

	/* Compute elapsed time in bytes. */
	bytes_diff = rte_reciprocal_divide(cycles_diff << RTE_SCHED_TIME_SHIFT,
		dst_queues->inv_cycles_per_byte);

	/* Advance port time. */
	dst_queues->time_cpu_cycles = cycles;
	dst_queues->time_cpu_bytes += bytes_diff;
	if (dst_queues->time < dst_queues->time_cpu_bytes)
		dst_queues->time = dst_queues->time_cpu_bytes;
}

static inline void
credits_update(struct dst_queues *dst_queues)
{
	uint64_t n_periods = (dst_queues->time - dst_queues->tb_time) /
		dst_queues->tb_period;
	dst_queues->tb_credits += n_periods * dst_queues->tb_credits_per_period;
	dst_queues->tb_credits = min_val_2_u32(dst_queues->tb_credits,
		dst_queues->tb_size);
	dst_queues->tb_time += n_periods * dst_queues->tb_period;
}

/*
 * XXX Implement prefetching using grinders, as in rte_sched_port_dequeue().
 */
uint32_t
dst_dequeue(struct dst_queues *dst_queues, const uint32_t num_pkts)
{
	uint32_t i = 0;

	time_resync(dst_queues);

	/*
	 * XXX When is the best time to do this? DPDK sched does
	 * this for every packet that it sees while doing prefetching.
	 */
	credits_update(dst_queues);

	dst_queues->n_pkts_out = 0;
	while (dst_queues->n_pkts_out < num_pkts &&
		i < dst_queues->num_queues) {

		if (rte_bitmap_get(dst_queues->bmp, dst_queues->cur_queue) != 0)
			__dst_dequeue(dst_queues,
				num_pkts - dst_queues->n_pkts_out);

		dst_queues->cur_queue = (dst_queues->cur_queue + 1) &
					(dst_queues->num_queues - 1);
		i++;
	}

        return dst_queues->n_pkts_out;
}
