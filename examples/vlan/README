# VLAN Test Application

This sample application demonstrates how to configure DPDK ports to strip and insert VLAN (802.1q) tags in hardware and in software. Its basic functionality is to echo packets back out the same port, but will strip the VLAN tag off of incoming VLAN packets.

The application can optionally be configured to insert a VLAN tag, and do this VLAN stripping/insertion in hardware or software. Additionally, if we know that we're going to receive VLAN packets and add our own VLAN tag on transmission, then the application can be configured to keep the VLAN header while processing the packet and simply overwrite the tag on transmission.

## Setup

To make DPDK insert a VLAN tag, set the `INSERT_VLAN_TAG` macro to 1 and set `VLAN_TAG` to the desired value.

To make DPDK configure ports to strip and insert VLAN tags in hardware, make sure the `USE_VLAN_HARDWARE_OFFLOAD` macro is set to 1. To test the functionality in software, set it to 0.

If a VLAN packet is going to be processed and then another tag is going to be applied, it makes sense to work around the VLAN header and overwrite the tag later. To enable this functionality, as opposed to removing the header completely, processing the packet, and then re-adding the header, make sure the `KEEP_AND_OVERWRITE_VLAN` macro is set to 1.

From inside the directory, just issue the `make` command:

    $ make

The application can be run with:

    $ sudo ./build/vlan

## Packet Generation

The sample stream file, `arp_to_dpdk`, can be opened in Ostinato and run. Change the MAC addresses in the stream file according to your local settings.

## Known Issues

This code has not yet been tested with VLAN packets.
