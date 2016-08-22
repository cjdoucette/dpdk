# Queues Test Application

This sample application demonstrates how a request priority queue and per-destination queues can be created in DPDK.

## Setup

Issue the `make` command:

    $ make

The application can be run with:

    $ sudo ./build/queues -c 0x1c00 -b 83:00.0 -b 85:00.0 -b 85:00.1 --socket-mem 256 --file-prefix queues

An example `pktgen` command that would generate packets to this application is:

    $ sudo ./pktgen -c 0xe000 --socket-mem 256 --file-prefix pg -b 83:00.0 -b 83:00.1 -b 85:00.0 -- -T -P -m "[14:15].0"

## Known Issues

The request and granted threads dequeue from their rings using 64 packets at a time, and won't dequeue if there are fewer than 64. So we should explore the tradeoffs of using the variant that will dequeue up to 64. For example, it may make sense to dequeue any requests that are waiting (`\*\_burst`), but wait until there are 64 packets for the granted queues (`\*\_bulk`).

## To Be Implemented in Future Versions

Although the enqueuing algorithm for the per-destination queues is similar to the built-in DPDK scheduler's enqueuing algorithm and therefore prefetching was preserved, dequeuing for the per-destination queues needs to be made more efficient.

Prefetching and accessing the active queues bitmap using a slab, rather than one bit a time, could speed it up.

Both enqueuing and dequeuing in the request priority queue could benefit from some performance analysis and prefetching as well.

The algorithm for recognizing a first-time request and also updating the priority of repeated requests needs to be implemented.

The algorithm for recognizing whether a packet is a request or already has a capability needs to be implemented.
