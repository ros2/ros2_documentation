
Installing ROS 2 on Linux
=========================

This page explains how to install ROS 2 on Linux from a pre-built binary package.

As of Beta 2 there are also `Debian packages <Linux-Install-Debians>` available.

System Requirements
-------------------

We support Ubuntu Linux Bionic Beaver (18.04) and Ubuntu Xenial Xerus (16.04) on 64-bit x86 and 64-bit ARM.

Note: Ardent and beta versions supported Ubuntu Xenial Xerus 16.04.

Add the ROS 2 apt repository
----------------------------

See the `development instructions <linux-dev-add-ros2-repo>`.

Downloading ROS 2
-----------------


* Go `the releases page <https://github.com/ros2/ros2/releases>`_
* Download the latest package for Linux; let's assume that it ends up at ``~/Downloads/ros2-crystal-linux-x86_64.tar.bz2``.

  * Note: there may be more than one binary download option which might cause the file name to differ.

* 
  Unpack it:

  .. code-block:: bash

       mkdir -p ~/ros2_install
       cd ~/ros2_install
       tar xf ~/Downloads/ros2-crystal-linux-x86_64.tar.bz2

Installing and initializing rosdep
----------------------------------

.. code-block:: bash

       sudo apt install -y python-rosdep
       rosdep init
       rosdep update


Installing the missing dependencies
-----------------------------------

Set your rosdistro according to the release you downloaded.

.. code-block:: bash

       CHOOSE_ROS_DISTRO=crystal # or bouncy
       rosdep install --from-paths ros2-linux/share --ignore-src --rosdistro $CHOOSE_ROS_DISTRO -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 osrf_testing_tools_cpp poco_vendor rmw_connext_cpp rosidl_typesupport_connext_c rosidl_typesupport_connext_cpp rti-connext-dds-5.3.1 tinyxml_vendor tinyxml2_vendor urdfdom urdfdom_headers"

#. *Optional*\ : if you want to use the ROS 1<->2 bridge, then you must also install ROS 1.
   Follow the normal install instructions: http://wiki.ros.org/kinetic/Installation/Ubuntu

Installing the python3 libraries
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

       sudo apt install -y libpython3-dev


Install additional DDS implementations (optional)
-------------------------------------------------

ROS 2 builds on top of DDS.
It is compatible with multiple DDS or RTPS (the DDS wire protocol) vendors.

The package you downloaded has been built with optional support for multiple vendors: eProsima FastRTPS, Adlink OpenSplice, and (as of ROS 2 Bouncy) RTI Connext as the middleware options.
Run-time support for eProsima's Fast RTPS is included bundled by default.
If you would like to use one of the other vendors you will need to install their software separately.

Adlink OpenSplice
^^^^^^^^^^^^^^^^^

To use OpenSplice you can install a Debian package built by OSRF.

Crystal and above:

.. code-block:: bash

       sudo apt update && sudo apt install -q -y \
           libopensplice69

Bouncy and earlier:

.. code-block:: bash

       sudo apt update && sudo apt install -q -y \
           libopensplice69


RTI Connext
^^^^^^^^^^^

To use RTI Connext you will need to have obtained a license from RTI.
Add the following line to your ``.bashrc`` file pointing to your copy of the license (and source it).

.. code-block:: bash

   export RTI_LICENSE_FILE=path/to/rti_license.dat

You can install a Debian package of RTI Connext built by OSRF.
You will need to accept a license from RTI.

.. code-block:: bash

       sudo apt update && sudo apt install -q -y \
           rti-connext-dds-5.3.1


If you want to install the Connext DDS-Security plugins please refer to `this page <Install-Connext-Security-Plugins>`

Try some examples
-----------------

In one terminal, source the setup file and then run a ``talker``\ :

.. code-block:: bash

   . ~/ros2_install/ros2-linux/setup.bash
   ros2 run demo_nodes_cpp talker

In another terminal source the setup file and then run a ``listener``\ :

.. code-block:: bash

   . ~/ros2_install/ros2-linux/setup.bash
   ros2 run demo_nodes_cpp listener

You should see the ``talker`` saying that it's ``Publishing`` messages and the ``listener`` saying ``I heard`` those messages.
Hooray!

If you have installed support for an optional vendor, see `this page <Working-with-multiple-RMW-implementations>` for details on how to use that vendor.

See the `demos <Tutorials>` for other things to try, including how to `run the talker-listener example in Python <Python-Programming>`.

ROS 1 bridge
^^^^^^^^^^^^

If you have ROS 1 installed, you can try the ROS 1 bridge, by first sourcing your ROS 1 setup file; we'll assume that it's ``/opt/ros/melodic/setup.bash``.

If you haven't already, start a roscore:

.. code-block:: bash

   . /opt/ros/melodic/setup.bash
   roscore


In another terminal, start the bridge:

.. code-block:: bash

   . /opt/ros/melodic/setup.bash
   . ~/ros2_install/ros2-linux/setup.bash
   ros2 run ros1_bridge dynamic_bridge

For more information on the bridge, read the `tutorial <https://github.com/ros2/ros1_bridge/blob/master/README>`__.
