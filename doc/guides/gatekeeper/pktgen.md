# Generating Packets on DPDK Interfaces

This document describes how to install and use the DPDK pktgen program to generate packets on the XIA server.

## Setting up Pktgen

First, obtain the pktgen source code and build it:

    $ git clone http://dpdk.org/git/apps/pktgen-dpd
    $ cd pktgen-dpdk
    $ make

Once it compiles, do a setup step:

    $ sudo -E ./setup.sh

The application should be ready to run at this point, but I also encountered the need to perform the following step (still in the pktgen-dpdk directory):

    $ cp Pktgen.lua app/app/x86_64-native-linuxapp-gcc/

## Running Two Instances of Pktgen

To see how two pktgen applications can send a receive from each other, open two terminals, go to the pktgen directory, and in the first one run:

    $ cd app/app/x86_64-native-linuxapp-gcc/
    $ sudo ./pktgen -c 7 --socket-mem 256 --file-prefix pg1 -b 85:00.0 -- -T -P -m "[1:2].0"

The "-c 7" option specifies which lcores are available for use, which is lcores 0, 1, and 2. The "-m [1:2].0" part specifies that lcores 1 and 2 will handle rx/tx on port 0, and lcore 0 is automatically assigned to the pktgen program for displaying statistics.

The "--socket-mem 256" option puts a limit on the memory used, which is often needed when multiple DPDK applications are run at the same time. The "--file-prefix pg1" option specifies a special file prefix to use for this DPDK application's meta information, which again is needed if there are multiple DPDK applications running (which we will have in the next part).

The "-b 85:00.0" option blacklists the NIC in physical location 85:00.0 from being used in this application. In this demonstration, this will be the receiving NIC that we will setup in the next part.

The "-T" option gives the display statistics some color and the "-P" makes all ports run in promiscuous mode.

Once that's running, do the following in the second terminal:

    $ cd app/app/x86_64-native-linuxapp-gcc/
    $ sudo ./pktgen -c 70 --socket-mem 256 --file-prefix pg2 -b 83:00.0 -- -T -P -m "[5:6].1"

This does the same as the first command, but instead allows lcores 5, 6, and 7 to be used, uses a different file prefix, and blacklists the transmitting NIC device from used.

More information about the command-line parameters is here: <http://pktgen.readthedocs.io/en/latest/usage_pktgen.html>.

Once the applications are running, go to the first terminal (running on port 0). You can start packets flowing on port 0 using:

    $ Pktgen> start 0

When you do this, you should see the second terminal's statistics being updated, as port 1 is receiving the packets from port 0. You could also start packets flowing on port 1 (from the other terminal) and see the packets being received on port 0.

You can stop packets flowing with:

    $ Pktgen> stop 0

And quit with:

    $ Pktgen> quit

## Running Pktgen with Another DPDK Application

We can also run pktgen on port 0 and observe the packets using a separate DPDK application running on port 1.

To use pktgen on port 0, use the same setup as above, with this command:

    $ sudo ./pktgen -c 7 --socket-mem 256 --file-prefix pg1 -b 85:00.0 -- -T -P -m "[1:2].0"

To get an application that will run on port 1, you can get the print application from my DPDK repository:

    $ git clone http://github.com/cjdoucette/dpdk

Use the same steps as in the guide setup.md to compile DPDK. Then you can compile the print application:

    $ cd examples/print
    $ make

The print application automatically runs on port 1, and can be run with this command:

    $ sudo ./build/print -c 0x10 --socket-mem 256 --file-prefix print -b 83:00.0

## Port Information

It appears that the ports, physical locations, and addresses on the XIA server are:

* port 0: location 83:00.0; MAC address: e8:ea:6a:06:1f:7c
* port 2: location 83:00.1; MAC address: e8:ea:6a:06:1f:7d

* port 1: location 85:00.0; MAC address: e8:ea:6a:06:21:b2
* port 3: location 85:00.1; MAC address: e8:ea:6a:06:21:b3

Port 0 is connected to port 1, and port 2 is connected to port 3.

## Known Issues

When trying to run the same demonstrations above, but with ports 2 and 3, I get a segmentation fault when trying to use port 3. Port 3 by itself works, but if you try to run it while blacklisting any of ports 0, 1, or 2, the segmentation fault happens.
