# Ethernet Bonding Application

This sample application demonstrates how to bond multiple Ethernet ports.

This example enables RSS on the bond port, and sets up multiple queues on the bond port to show RSS working. Configuring multiple queues on the slave ports does not seem to be necessary.

This example also shows how you can enable a Flow Director rule or ntuple filter rule on the bonded port.

Finally, this example enables VLAN stripping to show that bonded ports support VLAN stripping.

## Setup

Go to the `examples/bonding` directory:

    $ cd examples/bonding

And issue the `make` command:

    $ make

The application can be run with:

    $ sudo ./build/bonding -c 0x400 -b 85:00.0 -b 85:00.1 --socket-mem 256 --file-prefix bonding

## Packet Generation

### Generally Testing Bonded Ports

You can use the `pktgen` application to generate packets. In a third terminal, issue this command:

    $ sudo ./pktgen -c 0xe000 --socket-mem 256 --file-prefix pg -b 83:00.0 -b 85:00.0 -b 83:00.1 -- -T -P -m "[14:15].0"

And when `pktgen` starts, change the packets in this way:

    Pktgen> set 0 count 100

Then send packets with:

    Pktgen> start 0

You could also start `pktgen` with the other port and observe that the behavior is the same since the ports are bonded:

    $ sudo ./pktgen -c 0xe000 --socket-mem 256 --file-prefix pg -b 83:00.0 -b 83:00.1 -b 85:00.1 -- -T -P -m "[14:15].0"

### Testing RSS

You can change the destination IP address using the following command:

    Pktgen> set ip dst 0 192.168.57.12

By changing the destination IP address (perhaps a few times), you should be able to see the packets being directed to a different queue by RSS.

### Testing Flow Director and ntuple Filter

To test the filters, you can change the packets to be BGP packets (with the following destination IP address and destination port):

    Pktgen> set ip dst 0 192.168.57.12
    Pktgen> set 0 dport 179

BGP is just used as an example in this application for packets to filter.

### Testing VLAN Stripping

To send VLAN packets with VLAN ID 999 (0x03e7), use these commands:

    Pktgen> vlan 0 enable
    Pktgen> set 0 vlanid 999

When the packets arrived on the bonded port, you will see that it has been stripped off.

## Known Issues

This bonding example uses round-robin mode (mode 0), but in production we should use the 802.3AD mode (mode 4).
