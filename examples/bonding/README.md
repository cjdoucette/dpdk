# Ethernet Bonding Application

This sample application demonstrates how to bond multiple Ethernet ports.

This example enables RSS on the bond port, and sets up multiple queues on the bond port to show RSS working. Configuring multiple queues on the slave ports does not seem to be necessary.

This example also enables Flow Director on port 1. Since bonded ports do not support Flow Director, currently the only way to use Flow Director is to set it up on a specific port (or set of ports) and have an lcore listen on that port and thread. When a packet comes in (in this case, a BGP packet), the slave port using Flow Director seems to be prioritized.

## Setup

Go to the `examples/bonding` directory:

    $ cd examples/bonding

And issue the `make` command:

    $ make

The application can be run with:

    $ sudo ./build/bonding -c 0x400 -b 85:00.0 -b 85:00.1 --socket-mem 256 --file-prefix bonding

Note: the application should be run with lcore 10 to work properly.

## Packet Generation

You can use the `pktgen` application to generate packets. In a third terminal, issue this command:

    $ sudo ./pktgen -c 0xe000 --socket-mem 256 --file-prefix pg -b 83:00.0 -b 83:00.1 -b 85:00.0 -- -T -P -m "[14:15].0"

And when `pktgen` starts, change the packets in this way:

    Pktgen> set 0 count 100

Then send packets with:

    Pktgen> start 0

This packet should be sent to queue 0.

You could also start `pktgen` with the other port and observe that the behavior is the same since the ports are bonded:

    $ sudo ./pktgen -c 0xe000 --socket-mem 256 --file-prefix pg -b 83:00.0 -b 83:00.1 -b 85:00.1 -- -T -P -m "[14:15].0"

You can change the destination IP address using the following command:

    Pktgen> set ip dst 0 192.168.57.12

By changing the destination IP address (perhaps a few times), you should be able to see the packets being directed to a different queue by RSS.

To test the Flow Director, you can change the packets to be BGP packets (with the following destination IP address and destination port):

    Pktgen> set ip dst 0 192.168.57.12
    Pktgen> set 0 dport 179

## Known Issues

This bonding example uses round-robin mode (mode 0), but in production we should use the 802.3AD mode (mode 4).
