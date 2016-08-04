# Gatekeeper DPDK Configuration

## First Time Setup

There is some setup that is required that only needs to be done once, such as installing packages and installing modules. I've already done this setup on the XIA server.

### Install Dependencies

    # apt-get update

Not all of the following packages are needed, but they are helpful in DPDK development and debugging:

    # apt-get -y -q install git clang doxygen hugepages build-essential linux-headers-`uname -r` libpcap-dev tshark

### Get Source Code

Obtain the DPDK source code; here's one example to get the copy that Cody has been working on:

    $ git clone https://github.com/cjdoucette/dpdk

### Setup Environmental Variables and Build

    $ cd dpdk
    $ export RTE_SDK=`pwd`
    $ export RTE_TARGET=x86_64-native-linuxapp-gcc

    $ make config T=${RTE_TARGET}
    $ make

### Install Kernel Modules

    # modprobe uio
    # insmod ${RTE_SDK}/build/kmod/igb_uio.ko

The following makes the uio and igb\_uio installations persist across reboots:

    # ln -s ${RTE_SDK}/build/kmod/igb_uio.ko /lib/modules/`uname -r`
    # depmod -a
    # echo "uio" | sudo tee -a /etc/modules
    # echo "igb_uio" | sudo tee -a /etc/modules

## Setup After Every Reboot

After every reboot, the hugepages have to be setup with the following steps:

    $ HUGEPAGE_MOUNT=/mnt/huge
    # echo 1024 | sudo tee /sys/kernel/mm/hugepages/hugepages-2048kB/nr_hugepages
    # mkdir ${HUGEPAGE_MOUNT}
    # mount -t hugetlbfs nodev ${HUGEPAGE_MOUNT}

## Setup for Individual Users

Assuming the first-time setup and setup after every reboot have been completed (as described above), individual users can set up their own local copy of DPDK using these steps:

### Get Source Code

Obtain the DPDK source code; here's one example to get the copy that Cody has been working on:

    $ git clone https://github.com/cjdoucette/dpdk

### Setup Environmental Variables and Build

    $ cd dpdk
    $ export RTE_SDK=`pwd`
    $ export RTE_TARGET=x86_64-native-linuxapp-gcc

    $ make config T=${RTE_TARGET}
    $ make

### Add Environmenttal Variables to .profile

To make the environmental variables be set at each login, use these commands:

    $ echo "export RTE_SDK=${RTE_SDK}" >> ${HOME}/.profile
    $ echo "export RTE_TARGET=${RTE_TARGET}" >> ${HOME}/.profile

### Add Symlink to Make Examples Compile

    $ ln -s ${RTE_SDK}/build ${RTE_SDK}/${RTE_TARGET}

## Configuring NICs and Hugepages

You may want to change the status of the NICs and reconfigure hugepages while the system is running. The following commands provide information to do so.

### Configuring NICs

NICs can either be bound to the kernel ("ixgbe" driver) or DPDK ("igb\_uio" driver). When the NIC is associated with the kernel driver, you can refer to it by its interface name. When it's not, you need to use an identifier that represents the physical location of the NIC.

To see the status of the NICs, go to the `dpdk/tools` directory. Then issue:

    # ./dpdk_nic_bind.py --status

To bind a NIC to DPDK, find the name of the NIC you want to change (in this case "enp131s0f0") and issue the following command:

    # ./dpdk_nic_bind.py --bind=igb_uio enp131s0f0

To unbind a device, get the physical location identifier from the status command and issue this command:

    # ./dpdk_nic_bind.py --unbind 0000:83:00.0

To bind a device to the kernel, issue this command:

    # ./dpdk_nic_bind.py --bind=ixgbe 0000:83:00.0

### Configuring Hugepages

When DPDK programs fail, they may not properly release the pages that they were allocated. You can check how many hugepages you've allocated and how many are free by executing this command:

    $ grep -i huge /proc/meminfo

To free all of the pages, you can force them to release by unmounting them and remounting them:

    # umount /mnt/huge
    # mount -t hugetlbfs nodev /mnt/huge
