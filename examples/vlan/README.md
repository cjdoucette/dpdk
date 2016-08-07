# VLAN Test Application

This sample application demonstrates how to configure DPDK ports to strip and insert VLAN (802.1q) tags in hardware and in software. It assumes that incoming packets will have a VLAN tag and outgoing packets should have a VLAN tag.

The application can be configured to strip and insert VLAN tags in both hardware in software.

## Setup

To make DPDK strip and insert VLAN tags in hardware only, then set the macros in main.c to:

    #define HW_STRIP_VLAN_TAG 1
    #define HW_INSERT_VLAN_TAG 1
    #define SW_STRIP_VLAN_TAG 0
    #define SW_INSERT_VLAN_TAG 0

*Note: VLAN insertion in hardware is not currently supported, so in this mode the application will only strip the VLAN tag.*

To make DPDK strip VLAN tags in hardware and insert VLAN tags in software, then set the macros in main.c to:

    #define HW_STRIP_VLAN_TAG 1
    #define HW_INSERT_VLAN_TAG 0
    #define SW_STRIP_VLAN_TAG 0
    #define SW_INSERT_VLAN_TAG 1

To make DPDK strip and insert VLAN tags in software only, then set the macros in main.c to:

    #define HW_STRIP_VLAN_TAG 0
    #define HW_INSERT_VLAN_TAG 0
    #define SW_STRIP_VLAN_TAG 1
    #define SW_INSERT_VLAN_TAG 1

To make DPDK insert VLAN tags in software only (by not stripping the VLAN tag, but instead overwriting it), then set the macros in main.c to:

    #define HW_STRIP_VLAN_TAG 0
    #define HW_INSERT_VLAN_TAG 0
    #define SW_STRIP_VLAN_TAG 0
    #define SW_INSERT_VLAN_TAG 1

From inside the directory, just issue the `make` command:

    $ make

The application can be run (on the XIA server) with:

    $ sudo ./build/vlan -c 0x60 --socket-mem 256 --file-prefix vlan -b 83:00.0 -b 83:00.1

To send a VLAN packet, use `pktgen` in a separate terminal and run:

    $ sudo ./pktgen -c 7 --socket-mem 256 --file-prefix pg1 -b 83:00.1 -b 85:00.0 -b 85:00.1 -- -T -P -m "[1:2].0"

When in `pktgen`, issue these commands:

    Pktgen> vlan 0 enable
    Pktgen> set 0 vlanid 999
    Pktgen> set 0 count 1
    Pktgen> start 0

To see the packets that are transmitted (to check where the VLAN tag was inserted), in a separate terminal run the `print` application:

    $ sudo ./build/print -c 0x10 --socket-mem 256 --file-prefix print -b 83:00.0 -b 85:00.0 -b 85:00.1

## Known Issues

DPDK does not support VLAN insertion using the ixgbe driver.
