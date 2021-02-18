Installing ROS 2 on Linux
=========================

.. contents:: Table of Contents
   :depth: 2
   :local:

This page explains how to install ROS 2 on Linux from a pre-built binary package.

There are also `Debian packages <Linux-Install-Debians>` available.

System Requirements
-------------------

We support Ubuntu Linux Bionic Beaver (18.04) and Ubuntu Xenial Xerus (16.04) on 64-bit x86 and 64-bit ARM.

Note: Ardent and beta versions supported Ubuntu Xenial Xerus 16.04.

Add the ROS 2 apt repository
----------------------------

.. include:: _Apt-Repositories.rst

Downloading ROS 2
-----------------


* Go `the releases page <https://github.com/ros2/ros2/releases>`_
* Download the latest package for Linux; let's assume that it ends up at ``~/Downloads/ros2-crystal-linux-x86_64.tar.bz2``.

  * Note: there may be more than one binary download option which might cause the file name to differ.

*
  Unpack it:

  .. code-block:: bash

       mkdir -p ~/ros2_crystal
       cd ~/ros2_crystal
       tar xf ~/Downloads/ros2-crystal-linux-x86_64.tar.bz2

.. _linux-install-binary-install-missing-dependencies:

Installing and initializing rosdep
----------------------------------

.. code-block:: bash

       sudo apt update
       sudo apt install -y python-rosdep
       sudo rosdep init # if already initialized you may continue
       rosdep update


Installing the missing dependencies
-----------------------------------

Set your rosdistro according to the release you downloaded.

.. code-block:: bash

       CHOOSE_ROS_DISTRO=crystal # or bouncy
       rosdep install --from-paths ros2-linux/share --ignore-src --rosdistro $CHOOSE_ROS_DISTRO -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 osrf_testing_tools_cpp poco_vendor rmw_connext_cpp rosidl_typesupport_connext_c rosidl_typesupport_connext_cpp rti-connext-dds-5.3.1 tinyxml_vendor tinyxml2_vendor urdfdom urdfdom_headers"

#. *Optional*\ : if you want to use the ROS 1<->2 bridge, then you must also install ROS 1.
   Follow the normal install instructions: https://wiki.ros.org/kinetic/Installation/Ubuntu

Installing the python3 libraries
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

       sudo apt install -y libpython3-dev

Install additional DDS implementations (optional)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you would like to use another DDS or RTPS vendor besides the default, eProsima's Fast RTPS, you can find instructions `here <DDS-Implementations>`.

Environment setup
-----------------

Sourcing the setup script
^^^^^^^^^^^^^^^^^^^^^^^^^

Set up your environment by sourcing the following file.

.. code-block:: bash

  . ~/ros2_crystal/ros2-linux/setup.bash

Install argcomplete (optional)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

ROS 2 command line tools use argcomplete to autocompletion. So if you want autocompletion, installing argcomplete is necessary.

Ubuntu 18.04
~~~~~~~~~~~~

.. code-block:: bash

   sudo apt install python3-argcomplete

Ubuntu 16.04 (argcomplete >= 0.8.5)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To install ``argcomplete`` on Ubuntu 16.04 (Xenial), you'll need to use pip, because the version available through ``apt`` will not work due to a bug in that version of ``argcomplete``:

.. code-block:: bash

   sudo apt install python3-pip
   sudo pip3 install argcomplete

Try some examples
-----------------

In one terminal, source the setup file and then run a C++ ``talker``:

.. code-block:: bash

   . ~/ros2_crystal/ros2-linux/setup.bash
   ros2 run demo_nodes_cpp talker

In another terminal source the setup file and then run a Python ``listener``:

.. code-block:: bash

   . ~/ros2_crystal/ros2-linux/setup.bash
   ros2 run demo_nodes_py listener

You should see the ``talker`` saying that it's ``Publishing`` messages and the ``listener`` saying ``I heard`` those messages.
This verifies both the C++ and Python APIs are working properly.
Hooray!

See the `tutorials and demos </Tutorials>` for other things to try.

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
   . ~/ros2_crystal/ros2-linux/setup.bash
   ros2 run ros1_bridge dynamic_bridge

For more information on the bridge, read the `tutorial <https://github.com/ros2/ros1_bridge/blob/master/README.md>`__.

Build your own packages
-----------------------

If you would like to build your own packages, refer to the tutorial `"Using Colcon to build packages" </Tutorials/Colcon-Tutorial>`.

Uninstall
---------

1. If you installed your workspace with colcon as instructed above, "uninstalling" could be just a matter of opening a new terminal and not sourcing the workspace's ``setup`` file.
   This way, your environment will behave as though there is no Crystal install on your system.

2. If you're also trying to free up space, you can delete the entire workspace directory with:

   .. code-block:: bash

    rm -rf ~/ros2_crystal
