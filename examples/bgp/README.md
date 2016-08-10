# BGP Test Application

This sample application demonstrates how to let a BGP daemon running userspace interact with a routing table in DPDK.

This sample consists of four components:

 * The use of DPDK's filtering system (formerly called flow director) to put BGP packets (TCP packets with port 179) in a queue that is only served by a control lcore.
 * The use of DPDK's kernel-NIC interface (KNI) to forward and receive BGP traffic to and from a BGP daemon listening in userspace.
 * The use of Netlink messages so that the BGP daemon can edit the routing table.
 * The use of DPDK's LPM module as a forwarding information base.

## Setup

From inside the `dpdk` directory, go to the kernel modules directory:

    $ cd build/kmod

Then insert the KNI module, adjusting the kernel argumetns as needed:

    $ sudo insmod rte_kni.ko kthread_mode=single

Then go to the `examples/bgp` directory:

    $ cd ../../examples/bgp

And issue the `make` command:

    $ make

The application can be run with:

    $ sudo ./build/bgp -c 0x3 -b 83:00.1 -b 85:00.0 -b 85:00.1 --socket-mem 256 --file-prefix bgp -- --config "(0,0,1,2)"

In a separate terminal, configure the newly-created kernel interface:

    $ sudo ifconfig vEth0_0 192.168.57.12 netmask 255.255.255.0 up

In this second terminal, go to the `examples/bgp` directory and build the sample BGP daemon:

    $ gcc -Wall -Wextra bgp_daemon.c -lmnl -o bgp_daemon

And then run it:

    $ sudo ./bgp_daemon

This daemon will send an `RTM_NEWROUTE` message to the `bgp` application through the KNI, triggering an entry to be added to the LPM table. The daemon will also listen for BGP packets that are passed through the KNI.

You can then generate packets using the instructions below, observing that packets are passed to the DPDK port, queued to the control lcore, and sent to the BGP daemon.

## Packet Generation

You can use the `pktgen` application to generate BGP packets. In a third terminal, issue this command:

    $ sudo ./pktgen -c 70 --socket-mem 256 --file-prefix pg -b 83:00.0 -b 83:00.1 -b 85:00.1 -- -T -P -m "[5:6].0"

And when `pktgen` starts, change the packets in this way:

    Pktgen> set 0 count 1
    Pktgen> set 0 dport 179
    Pktgen> set ip dst 0 192.168.57.12

Then send a BGP packet with:

    Pktgen> start 0

## Known Issues

The allocation of threads needs to be done dynamically.

The only way I could get the fake BGP daemon to receive packets was to have it receive *all* TCP packets and drop non-BGP packets.
