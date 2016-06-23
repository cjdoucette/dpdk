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

    $ sudo ./build/bgp -c 0x3 -n 4 -- -P -p 0x2 --config "(1,0,1,2)"

In a separate terminal in the VM, configure the newly-created kernel interface:

    $ sudo ifconfig vEth1_0 192.168.57.12 netmask 255.255.255.0 up

In this second terminal, go to the `examples/bgp` directory and build the sample BGP daemon:

    $ gcc -Wall -Wextra bgp_daemon.c -lmnl -o bgp_daemon

And then run it:

    $ sudo ./bgp_daemon

This daemon will send an `RTM_NEWROUTE` message to the `bgp` application through the KNI, triggering an entry to be added to the LPM table. The daemon will also listen for BGP packets that are passed through the KNI.

You can then generate packets using the instructions below, observing that packets are passed to the DPDK port, queued to the control lcore, and sent to the BGP daemon.

## Packet Generation

The sample stream file `bgp_to_dpdk` can be opened in Ostinato and run to cause the DPDK code to forward the packet to the BGP daemon. Note that this is not a valid BGP stream -- it would need to initialize a TCP connection and send actual BGP messages. Instead, it's just a packet that uses TCP and port 179 (BGP) to demonstrate how to direct BGP flows and deliver them to the BGP daemon.

## Known Issues

The first issue is that the sample packet, `bgp_to_dpdk`, uses the broadcast Ethernet address for its destination MAC address. This is because even after enabling promiscuous mode on all ports (DPDK and kernel), I could not get a packet to go all the way from the client, through the DPDK port, and onto the kernel port. The packet seemed to get lost in the kernel, and was never delivered to the application. This could have to do with the fact that the application is using a raw socket to listen for *all* TCP packets. In any case, once a real BGP daemon is deployed, we'll need to make sure that with promiscuous mode enabled the BGP packet is able to make it from peer --> DPDK --> daemon without the broadcast destination address.

The other issue is that since the virtual NIC in VirtualBox does not support multiple receiving queues, I haven't fully tested the ability of the flow director (just referred to as a "filter" in DPDK now) to assign to one of many queues on a port, where a special lcore is pinned to that queue.
