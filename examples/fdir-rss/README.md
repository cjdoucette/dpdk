# Flow Director and RSS Application

This sample application demonstrates how to run Flow Director to direct packets, and then to let RSS take any unrecognized packets and load balance them across the available queues.

The application uses Flow Director to accept BGP packets on queue 0, and uses RSS to randomly assign all other packets to queues 1 and 2.

## Setup

Go to the `examples/fdir-rss` directory:

    $ cd examples/fdir-rss

And issue the `make` command:

    $ make

The application can be run with:

    $ sudo ./build/fdir-rss -c 0x1c00 -b 83:00.0 -b 85:00.0 -b 85:00.1 --socket-mem 256 --file-prefix fdir-rss

Note: the application should be run with lcores 10, 11, and 12 to work properly.

## Packet Generation

You can use the `pktgen` application to generate BGP packets. In a third terminal, issue this command:

    $ sudo ./pktgen -c 0xe000 --socket-mem 256 --file-prefix pg -b 83:00.0 -b 83:00.1 -b 85:00.0 -- -T -P -m "[14:15].0"

And when `pktgen` starts, change the packets in this way:

    Pktgen> set 0 count 1
    Pktgen> set 0 dport 179
    Pktgen> set ip dst 0 192.168.57.12

Then send a BGP packet with:

    Pktgen> start 0

This packet should be sent to queue 0.

You can then change the destination IP address to see that the packet will be sent to either queue 1 or queue 2.

## Known Issues

Changing the source and destination ports doesn't seem to affect the RSS assignment.
