# Queues Test Application

This sample application demonstrates how a request priority queue and per-destination queues can be created in DPDK.

## Setup

Issue the `make` command:

    $ make

The application can be run with:

    $ sudo ./build/queues -c 0x7

## Known Issues

Although the enqueuing algorithm for the per-destination queues is similar to the built-in DPDK scheduler's enqueuing algorithm and therefore prefetching was preserved, dequeuing for the per-destination queues needs to be made more efficient. Prefetching and accessing the active queues bitmap using a slab, rather than one bit a time, could speed it up.

Both enqueuing and dequeuing in the request priority queue could benefit from some performance analysis and prefetching as well.

Additionally, the per-destination queues should be RED queues.

The algorithm for recognizing a first-time request and also updating the priority of repeated requests needs to be implemented.

The algorithm for recognizing whether a packet is a request or already has a capability needs to be implemented.

The application needs to be tested at high packet processing speeds to test its performance.

The request and granted threads dequeue from their rings using 64 packets at a time, and won't dequeue if there are fewer than 64. So we should explore the tradeoffs of using variant that will dequeue up to 64.
