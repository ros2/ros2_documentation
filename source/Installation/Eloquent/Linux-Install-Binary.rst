Installing ROS 2 on Linux
=========================

.. contents:: Table of Contents
   :depth: 2
   :local:

This page explains how to install ROS 2 on Linux from a pre-built binary package.

As of Beta 2 there are also `Debian packages <Linux-Install-Debians>` available.

System Requirements
-------------------

We support Ubuntu Linux Bionic Beaver (18.04) and Ubuntu Xenial Xerus (16.04) on 64-bit x86 and 64-bit ARM.

Note: Ardent and beta versions supported Ubuntu Xenial Xerus 16.04.

Add the ROS 2 apt repository
----------------------------

.. include:: ../_Apt-Repositories.rst

Downloading ROS 2
-----------------


* Go `the releases page <https://github.com/ros2/ros2/releases>`_
* Download the latest package for Linux; let's assume that it ends up at ``~/Downloads/ros2-eloquent-linux-x86_64.tar.bz2``.

  * Note: there may be more than one binary download option which might cause the file name to differ.

*
  Unpack it:

  .. code-block:: bash

       mkdir -p ~/ros2_eloquent
       cd ~/ros2_eloquent
       tar xf ~/Downloads/ros2-eloquent-linux-x86_64.tar.bz2

Installing and initializing rosdep
----------------------------------

.. code-block:: bash

       sudo apt update
       sudo apt install -y python-rosdep
       sudo rosdep init
       rosdep update


Installing the missing dependencies
-----------------------------------

Set your rosdistro according to the release you downloaded.

.. code-block:: bash

       rosdep install --from-paths ros2-linux/share --ignore-src --rosdistro eloquent -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 osrf_testing_tools_cpp poco_vendor rmw_connext_cpp rosidl_typesupport_connext_c rosidl_typesupport_connext_cpp rti-connext-dds-5.3.1 tinyxml_vendor tinyxml2_vendor urdfdom urdfdom_headers"

#. *Optional*\ : if you want to use the ROS 1<->2 bridge, then you must also install ROS 1.
   Follow the normal install instructions: http://wiki.ros.org/melodic/Installation/Ubuntu

Installing the python3 libraries
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

       sudo apt install -y libpython3-dev

Install additional DDS implementations (optional)
-------------------------------------------------

ROS 2 builds on top of DDS.
It is compatible with multiple DDS or RTPS (the DDS wire protocol) vendors.

The package you downloaded has been built with optional support for multiple vendors: eProsima FastRTPS, ADLINK OpenSplice, and (as of ROS 2 Bouncy) RTI Connext as the middleware options.
Run-time support for eProsima's Fast RTPS is included bundled by default.
If you would like to use one of the other vendors you will need to install their software separately.

ADLINK OpenSplice
^^^^^^^^^^^^^^^^^

To use OpenSplice you can install a Debian package built by OSRF.

.. code-block:: bash

   sudo apt update && sudo apt install -q -y libopensplice69


RTI Connext (version 5.3.1, amd64 only)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To use RTI Connext DDS there are full-suite install options available for `university, purchase or evaluation <../Install-Connext-University-Eval>`
or you can install a libraries-only Debian package of RTI Connext 5.3.1, available from the OSRF Apt repository
under a `non-commercial license <https://www.rti.com/ncl>`__.

To install the libs-only Debian package:

.. code-block:: bash

   sudo apt update && sudo apt install -q -y rti-connext-dds-5.3.1

You will need to accept a license agreement from RTI, and will find an 'rti_license.dat file in the installation.

Add the following line to your ``.bashrc`` file pointing to your copy of the license (and source it).

.. code-block:: bash

   export RTI_LICENSE_FILE=path/to/rti_license.dat

All options need you to source the setup file to set the ``NDDSHOME`` environment variable:

.. code-block:: bash

   cd /opt/rti.com/rti_connext_dds-5.3.1/resource/scripts && source ./rtisetenv_x64Linux3gcc5.4.0.bash; cd -

Note: the above may need modification to match your RTI installation location

If you want to install the Connext DDS-Security plugins please refer to `this page <../Install-Connext-Security-Plugins>`.


Environment setup
-----------------

Sourcing the setup script
^^^^^^^^^^^^^^^^^^^^^^^^^

Set up your environment by sourcing the following file.

.. code-block:: bash

  . ~/ros2_eloquent/ros2-linux/setup.bash

Installing python3 argcomplete (optional)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

ROS 2 command line tools use argcomplete for autocompletion.
So if you want autocompletion, installing argcomplete is necessary.

.. code-block:: bash

   sudo apt install python3-argcomplete

Try some examples
-----------------

In one terminal, source the setup file and then run a ``talker``:

.. code-block:: bash

   . ~/ros2_eloquent/ros2-linux/setup.bash
   ros2 run demo_nodes_cpp talker

In another terminal source the setup file and then run a ``listener``:

.. code-block:: bash

   . ~/ros2_eloquent/ros2-linux/setup.bash
   ros2 run demo_nodes_cpp listener

You should see the ``talker`` saying that it's ``Publishing`` messages and the ``listener`` saying ``I heard`` those messages.
Hooray!

If you have installed support for an optional vendor, see `this page </Tutorials/Working-with-multiple-RMW-implementations>` for details on how to use that vendor.

See the `demos </Tutorials>` for other things to try, including how to `run the talker-listener example in Python </Tutorials/Python-Programming>`.


Using the ROS 1 bridge
----------------------

If you have ROS 1 installed, you can try the ROS 1 bridge, by first sourcing your ROS 1 setup file.
We'll assume that it is ``/opt/ros/melodic/setup.bash`` in the following.

If you haven't already, start a roscore:

.. code-block:: bash

   . /opt/ros/melodic/setup.bash
   roscore


In another terminal, start the bridge:

.. code-block:: bash

   . /opt/ros/melodic/setup.bash
   . ~/ros2_eloquent/ros2-linux/setup.bash
   ros2 run ros1_bridge dynamic_bridge

For more information on the bridge, read the `tutorial <https://github.com/ros2/ros1_bridge/blob/master/README.md>`__.

Build your own packages
-----------------------

If you would like to build your own packages, refer to the tutorial `"Using Colcon to build packages" </Tutorials/Colcon-Tutorial>`.

Troubleshooting
---------------

Troubleshooting techniques can be found `here </Troubleshooting>`.

Uninstall
---------

1. If you installed your workspace with colcon as instructed above, "uninstalling" could be just a matter of opening a new terminal and not sourcing the workspace's ``setup`` file.
   This way, your environment will behave as though there is no Eloquent install on your system.

2. If you're also trying to free up space, you can delete the entire workspace directory with:

   .. code-block:: bash

    rm -rf ~/ros2_eloquent
