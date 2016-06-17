# ARP Test Application

This sample application demonstrates how to create and use an ARP cache in DPDK. It supports ARP requests and replies by hashing IP addresses and storing the associated hardware address, port number, and time of last update.

The application should be run with at least two lcores. The main lcore accepts packets and, for ARP requests and replies, adds update messages to a DPDK ring. The second lcore polls the ring for new updates and also periodically scans the ARP cache to mark entries as stale and eventually remove stale entries.

## Setup

The sample application is setup to have two hard-coded IP addresses, one for each port. This is so that when ARP requests are received, the code can verify that the target IP address matches an IP address that has been assigned to that port.

The sample application also adds a sample entry to the ARP cache upon startup.

From inside the `examples/arp` directory, just issue the `make` command:

    $ make

The application can be run with (using two lcores):

    $ sudo ./build/arp -c 0x3

## Packet Generation

The sample stream file `arp_req_to_dpdk` can be opened in Ostinato and run to cause the ARP cache to receive the request, update the ARP cache with information from the sender, and send a reply. The sample stream file `arp_repy_to_dpdk` can be opened in Ostinato and run to cause the ARP cache to add an entry. Change the MAC addresses and IP addresses in the stream files according to your local settings.

## Known Issues

This code should probably make sure that when it receives an ARP packet, it makes sure the sender's IP address is in the same subnet.
