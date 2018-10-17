
This Tutorial begins with a clean ubuntu 16.04.2 install. Actual kernel is 4.13.0-38-generic, but we will install another one.

If you are a company or rich person :) using rt_preempt, check https://wiki.linuxfoundation.org/realtime/rtl/blog#preempt-rt-history 

Check on https://wiki.linuxfoundation.org/realtime/start what the latest stable version is, at this time it is  Latest Stable Version 4.9-rt. If we click on the link, we get the exact version, it is patch-4.9.84-rt62.patch.gz 


.. image:: https://i.imgur.com/bAMOzbt.png
   :target: https://i.imgur.com/bAMOzbt.png
   :alt: eclipse-1


We create a directory in our home dir with 

.. code-block::

   mkdir ~/kernel

and switch into it with

.. code-block::

   cd ~/kernel

We can go with a browser to https://mirrors.edge.kernel.org/pub/linux/kernel/v4.x/ and see if the version is there, then download it with

.. code-block::

   wget https://mirrors.edge.kernel.org/pub/linux/kernel/v4.x/linux-4.9.84.tar.gz

unpack it with

.. code-block::

   tar -xzf linux-4.9.84.tar.gz

rename it to the same name with postfix of the patch version

.. code-block::

   mv linux-4.9.84 linux-4.9.84-rt62

download rt_preempt patch with

.. code-block::

   wget ftp.ntu.edu.tw/linux/kernel/projects/rt/4.9/older/patch-4.9.84-rt62.patch.gz

unpack it with

.. code-block::

   gunzip patch-4.9.84-rt62.patch.gz

Then switch into the linux directory with

.. code-block::

   cd linux-4.9.84-rt62/

and patch the kernel with the realtime patch

.. code-block::

   patch -p1 < ../patch-4.9.84-rt62.patch


.. image:: https://i.imgur.com/u1VFptM.png
   :target: https://i.imgur.com/u1VFptM.png
   :alt: eclipse-1


We simply wanna use the config of our ubuntu installation, so we use the ubuntu config with

.. code-block::

   cp /boot/config-4.13.0-38-generic .config

To enable all ubuntu-configurations, we simply use

.. code-block::

   yes '' | make oldconfig

We need some tools, install them with

.. code-block::

   sudo apt install libncurses5-dev build-essential libssl-dev ccache

Then we need to enable rt_preempt in the kernel. We call 

.. code-block::

   make menuconfig

and choose under “Processor Type and Features”  ---  “Preemption Model”  --- “Fully Preemptible kernel (RT)” 


.. image:: https://i.imgur.com/Jg5zX6G.png
   :target: https://i.imgur.com/Jg5zX6G.png
   :alt: eclipse-1


Exit menuconfig and run

.. code-block::

   make

You could use “make -j4” if you got 4-cpu-cores to build faster.

Then we need to build the kernel modules with

.. code-block::

   sudo make modules_install

Then we install the kernel to /boot and update grub with

.. code-block::

   sudo make install


.. image:: https://i.imgur.com/Y5ihCXd.png
   :target: https://i.imgur.com/Y5ihCXd.png
   :alt: eclipse-1

