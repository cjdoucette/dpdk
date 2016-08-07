# Generating Packets on DPDK Interfaces

This document describes how to install and use the DPDK `pktgen` program to generate packets on the XIA server.

## Server Physical Setup

The XIA server has four DPDK-enabled ports that can be used to send and receive packets. The diagram below shows the four ports (along with their PCI identifiers):

    +-----------------------------------------+
    |  -----               -----              |
    |  | o | A (83:00.0)   | o | B (83:00.1)  |
    |  --|--               --|--              |
    |    |                   |                |
    |  --|--               --|--              |
    |  | o | C (85:00.0)   | o | D (85:00.1)  |
    |  -----               -----              |
    +-----------------------------------------+

Note that ports A and C are connected, and ports B and D are connected.

When you run multiple DPDK applications at the same time (such as two instances of `pktgen`, or `pktgen` and another DPDK application, you need to *blacklist* the PCI devices, or ports in this case, that you don't want to use. Depending on how you blacklist the devices, the port numbers that are assigned by DPDK will change.

For example, if you blacklist ports A, B, and C in one instance of `pktgen`, then port D is known as port 0 in that instance. If, at the same time, you blacklist ports A, C, and D in another instance of `pktgen`, then in that instance port B is known as port 0. To avoid ambiguity, in this document I will refer to the ports using these letter designations (A, B, C, D) instead of by numbers.

## Setting up `pktgen`

First, obtain the `pktgen` source code and build it:

    $ git clone http://dpdk.org/git/apps/pktgen-dpd
    $ cd pktgen-dpdk
    $ make

Once it compiles, do a setup step:

    $ sudo -E ./setup.sh

The application should be ready to run at this point, but I also encountered the need to perform the following step (still in the `pktgen-dpdk` directory):

    $ cp Pktgen.lua app/app/x86_64-native-linuxapp-gcc/

## Running Two Instances of `pktgen`

This demonstration will show how we can generate packets on port A and send them to port C. To begin, open two terminals.

In the first terminal, go to the `dpdk-pktgen` directory and run:

    $ cd app/app/x86_64-native-linuxapp-gcc/
    $ sudo ./pktgen -c 7 --socket-mem 256 --file-prefix pg1 -b 83:00.1 -b 85:00.0 -b 85:00.1 -- -T -P -m "[1:2].0" 

The "-c 7" option specifies which lcores are available for use, which is lcores 0, 1, and 2. The "-m [1:2].0" part specifies that lcores 1 and 2 will handle rx/tx on port 0, and lcore 0 is automatically assigned to the `pktgen` program for displaying statistics.

The "--socket-mem 256" option puts a limit on the memory used, which is often needed when multiple DPDK applications are run at the same time. The "--file-prefix pg1" option specifies a special file prefix to use for this DPDK application's meta information, which again is needed if there are multiple DPDK applications running (which we will have in the next part).

The "-b" options blacklist different ports -- in other words, excludes those ports from being used in the application. In this command, we only want to use port A (at PCI location 83:00.0), so we blacklist the other three ports: B (83:00.1), C (85:00.0), and D (85:00.1).

The "-T" option gives the display statistics some color and the "-P" makes all ports run in promiscuous mode.

In the second terminal, run the same command, but with different lcores and blacklisted ports:

    $ cd app/app/x86_64-native-linuxapp-gcc/
    $ sudo ./pktgen -c 70 --socket-mem 256 --file-prefix pg2 -b 83:00.0 -b 83:00.1 -b 85:00.1 -- -T -P -m "[5:6].0"

This does the same as the first command, but instead allows lcores 5, 6, and 7 to be used, uses a different file prefix, and blacklists all ports except for port C.

More information about the command-line parameters is here: <http://pktgen.readthedocs.io/en/latest/usage_pktgen.html>.

Once the applications are running, go to the terminal running `pktgen` on port A. You can start packets flowing using:

    $ Pktgen> start 0

Remember that port numbering always starts from 0 within an application, so sinc we blacklisted all other ports, port 0 means port A.

When you do this, you should see the second terminal's statistics being updated. You could also start packets flowing on the second terminal (port C) and see the packets being received on port A -- on the second terminal, port C will also be called port 0, since that is the only active port in that application.

You can stop packets flowing with:

    $ Pktgen> stop 0

And quit with:

    $ Pktgen> quit

## Running `pktgen` with Another DPDK Application

We can also run `pktgen` on port A and observe the packets using a separate DPDK application running on port C.

To use `pktgen` on port A, use the same setup as above with this command:

    $ sudo ./pktgen -c 7 --socket-mem 256 --file-prefix pg1 -b 83:00.1 -b 85:00.0 -b 85:00.1 -- -T -P -m "[1:2].0" 

To get an application that will run on port C, you can get the `print` application from my DPDK repository:

    $ git clone http://github.com/cjdoucette/dpdk

Use the same steps as in the guide setup.md to compile DPDK. Then you can compile the `print` application:

    $ cd examples/print
    $ make

Then `print` can be run with this command:

    $ sudo ./build/print -c 0x10 --socket-mem 256 --file-prefix print -b 83:00.0 -b 83:00.1 -b 85:00.1

Back in the `pktgen` application, you could use `start 0` to send a lot of packets. A better demonstration is to send one packet (an ARP packet) using:

    $ send arp req 0

You should see the packet printed by the `print` application.
