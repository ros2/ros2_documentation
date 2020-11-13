.. redirect-from::

    Building-Realtime-rt_preempt-kernel-for-ROS-2

Building realtime Linux for ROS 2 [community-contributed]
=========================================================

This tutorial begins with a clean Ubuntu 20.04.1 install on Intel x86_64. Actual kernel is 5.4.0-52-generic, but we will install the Latest Stable RT_PREEMPT Version. To build the kernel you need at least 30GB free disk space.

Check https://wiki.linuxfoundation.org/realtime/start for the latest stable version, at this time it is "Latest Stable Version 5.4-rt". If we click on the `link <http://cdn.kernel.org/pub/linux/kernel/projects/rt/5.4/>`_, we get the exact version, it is patch-5.4.74-rt41.patch.gz .

.. image:: https://i.imgur.com/hu4Q04b.png
   :target: https://i.imgur.com/hu4Q04b.png
   :alt: eclipse-1

We create a directory in our home dir with

.. code-block:: bash

   mkdir ~/kernel

and switch into it with

.. code-block:: bash

   cd ~/kernel

We can go with a browser to https://mirrors.edge.kernel.org/pub/linux/kernel/v5.x/ and see if the version is there, then download it with

.. code-block:: bash

   wget https://mirrors.edge.kernel.org/pub/linux/kernel/v5.x/linux-5.4.74.tar.gz

unpack it with

.. code-block:: bash

   tar -xzf linux-5.4.74.tar.gz

download rt_preempt patch with

.. code-block:: bash

   wget http://cdn.kernel.org/pub/linux/kernel/projects/rt/5.4/patch-5.4.74-rt41.patch.gz

unpack it with

.. code-block:: bash

   gunzip patch-5.4.74-rt41.patch.gz

Then switch into the linux directory with

.. code-block:: bash

   cd linux-5.4.74/

and patch the kernel with the realtime patch

.. code-block:: bash

   patch -p1 < ../patch-5.4.74-rt41.patch

We simply want to use the config of our Ubuntu installation, so we get the Ubuntu config with

.. code-block:: bash

   cp /boot/config-5.4.0-52-generic .config

We need some tools to build kernel, install them with

.. code-block:: bash

   sudo apt-get build-dep linux
   sudo apt-get install libncurses-dev flex bison openssl libssl-dev dkms libelf-dev libudev-dev libpci-dev libiberty-dev autoconf fakeroot

To enable all Ubuntu configurations, we simply use

.. code-block:: bash

   yes '' | make oldconfig

Then we need to enable rt_preempt in the kernel. We call

.. code-block:: bash

   make menuconfig

and set the following

.. code-block:: bash

  # Enable CONFIG_PREEMPT_RT
   -> General Setup
    -> Preemption Model (Fully Preemptible Kernel (Real-Time))
     (X) Fully Preemptible Kernel (Real-Time)

  # Enable CONFIG_HIGH_RES_TIMERS
   -> General setup
    -> Timers subsystem
     [*] High Resolution Timer Support

  # Enable CONFIG_NO_HZ_FULL
   -> General setup
    -> Timers subsystem
     -> Timer tick handling (Full dynticks system (tickless))
      (X) Full dynticks system (tickless)

  # Set CONFIG_HZ_1000
   -> Processor type and features
    -> Timer frequency (1000 HZ)
     (X) 1000 HZ

  # Set CPU_FREQ_DEFAULT_GOV_PERFORMANCE [=y]
   ->  Power management and ACPI options
    -> CPU Frequency scaling
     -> CPU Frequency scaling (CPU_FREQ [=y])
      -> Default CPUFreq governor (<choice> [=y])
       (X) performance

Save and exit menuconfig and run

.. code-block:: bash

   make -j `nproc` deb-pkg

After the build is finished check the debian packages

.. code-block:: bash

   ls ../*deb
   ../linux-headers-5.4.74-rt41_5.4.74-rt41-1_amd64.deb  ../linux-image-5.4.74-rt41-dbg_5.4.74-rt41-1_amd64.deb
   ../linux-image-5.4.74-rt41_5.4.74-rt41-1_amd64.deb    ../linux-libc-dev_5.4.74-rt41-1_amd64.deb

Then we install all kernel debian packages

.. code-block:: bash

   sudo dpkg -i ../*.deb

Reboot the system and check the new kernel version

.. code-block:: bash

   sudo reboot
   uname -a
   Linux ros2host 5.4.74-rt41 #1 SMP PREEMPT_RT Wed Nov 11 23:40:27 CET 2020 x86_64 xx
   
