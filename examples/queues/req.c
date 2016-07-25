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

#include "req.h"

void
req_send_burst(struct gk_data *gk, struct req_queue *req_queue)
{
	uint16_t ret;

	do {
		ret = rte_eth_tx_burst(gk->tx_port, gk->tx_queue,
			req_queue->pkts_out, req_queue->n_pkts_out);

		/* We cannot drop the packets, so re-send. */
		req_queue->n_pkts_out -= ret;
		req_queue->pkts_out += ret;
	} while (req_queue->n_pkts_out);
}

/* XXX Dummy implementation. */
static inline uint8_t
req_get_priority(struct rte_mbuf *pkt)
{
	return ((*(uint8_t *)pkt + rte_rand()) % 62) + 2;
}

static struct priority_ll *
first_pkt_of_priority(struct priority_ll *pll, uint8_t priority)
{
	while (pll->next != NULL && pll->next->priority == priority)
		pll = pll->next;
	return pll;
}

static void
insert_new_priority_req(struct req_queue *req_queue, struct priority_ll *pll)
{
	uint8_t next, prev;

	req_queue->priorities[pll->priority] = pll;
	pll->next = NULL;
	pll->prev = NULL;

	/* This is the first packet in the queue. */
	if (req_queue->length == 0) {
		req_queue->head = pll;
		req_queue->highest_priority = pll->priority;
		return;
	}

	/* Update head of queue. */
	if (pll->priority > req_queue->highest_priority) {
		req_queue->head = pll;
		req_queue->highest_priority = pll->priority;
	}

	/* Search for the next node in the queue. */
	for (next = pll->priority + 1; next < GK_NUM_REQ_PRIORITIES; next++) {
		if (rte_bitmap_get(req_queue->bmp, next) != 0) {
			pll->next = req_queue->priorities[next];
			break;
		}
	}

	/* Found the next node in the queue. */
	if (pll->next != NULL) {
		pll->prev = pll->next->prev;
		pll->next->prev = pll;
		if (pll->prev != NULL)
			pll->prev->next = pll;
		return;
	}

	/*
	 * Need to look backwards for previous node, and
	 * then get the first packet in that priority. We
	 * know there must be a previous node because the
	 * queue has at least one packet in it.
	 */
	for (prev = pll->priority - 1; prev >= 2; prev--)
		if (rte_bitmap_get(req_queue->bmp, prev) != 0)
			break;
	pll->prev = first_pkt_of_priority(req_queue->priorities[prev], prev);
	pll->prev->next = pll;
}

static inline void
insert_req(struct priority_ll *last_req_of_pri, struct priority_ll *new_req)
{
	if (last_req_of_pri->prev != NULL)
		last_req_of_pri->prev->next = new_req;
	new_req->prev = last_req_of_pri->prev;

	if (last_req_of_pri->next != NULL)
		last_req_of_pri->next->prev = new_req;
	new_req->next = last_req_of_pri;
}

int
req_enqueue(struct req_queue *req_queue, struct rte_mbuf **mbufs,
	uint32_t num_pkts)
{
	struct priority_ll *pll;
	uint32_t added = 0;
	uint32_t i;

	for (i = 0; i < num_pkts; i++) {
		pll = (struct priority_ll *)rte_pktmbuf_prepend(mbufs[i],
			sizeof(*pll));
		/*
		 * XXX Does mbuf pool creation size guarantee
		 * this should never happen?
		 */
		if (!pll) {
			rte_pktmbuf_free(mbufs[i]);
			continue;
		}

		pll->mbuf = mbufs[i];
		pll->priority = req_get_priority(mbufs[i]);

		/* Insert request of a priority we don't yet have. */
		if (req_queue->priorities[pll->priority] == NULL) {
			insert_new_priority_req(req_queue, pll);
			rte_bitmap_set(req_queue->bmp, pll->priority);
		} else
			/* Append request to end of appropriate priority. */
			insert_req(req_queue->priorities[pll->priority], pll);

		req_queue->length++;
		added++;
	}

	return added;
}

uint32_t
req_dequeue(struct req_queue *req_queue, const uint32_t num_pkts)
{
	struct priority_ll *head = req_queue->head;
	struct rte_mbuf *pkt;

	req_queue->n_pkts_out = 0;
	while (req_queue->n_pkts_out < num_pkts && head != NULL) {

		/* XXX Check credits. */

		pkt = head->mbuf;

		/* Remove request from queue. */
		head = head->prev;
		head->next = NULL;
		req_queue->length--;

		/* Remove extra space in mbuf. */
		if (rte_pktmbuf_adj(pkt, sizeof(*head)) == NULL) {
			rte_panic("bug in request queue: should be able to remove the extra linked list data in a packet, but the removal failed\n");;
			continue;
		}

		/* Advance port time. */
		req_queue->time += pkt->pkt_len + req_queue->frame_overhead;

		/* Queue packet for transmission. */
		req_queue->pkts_out[req_queue->n_pkts_out] = pkt;
		req_queue->n_pkts_out++;
	}

	req_queue->head = head;
	return req_queue->n_pkts_out;
}
