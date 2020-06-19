Installing ROS 2 on Linux
=========================

.. contents:: Table of Contents
   :depth: 2
   :local:

This page explains how to install ROS 2 on Linux from a pre-built binary package.

There are also `Debian packages <Linux-Install-Debians>` available.

System Requirements
-------------------

We currently support Ubuntu Linux Focal Fossa (20.04) 64-bit x86 and 64-bit ARM.
The Rolling Ridley distribution will change target platforms from time to time as new platforms are selected for development.
Most people will want to use a stable ROS distribution.

Add the ROS 2 apt repository
----------------------------

.. include:: ../_Apt-Repositories.rst

Downloading ROS 2
-----------------

Binary releases of Rolling Ridley are not provided.
Instead you may download nightly `prerelease binaries <Prerelease_binaries>`.

* Download the latest package for Linux; let's assume that it ends up at ``~/Downloads/ros2-package-linux-x86_64.tar.bz2``.

  * Note: there may be more than one binary download option which might cause the file name to differ.

*
  Unpack it:

  .. code-block:: bash

       mkdir -p ~/ros2_rolling
       cd ~/ros2_rolling
       tar xf ~/Downloads/ros2-package-linux-x86_64.tar.bz2

Installing and initializing rosdep
----------------------------------

.. code-block:: bash

       sudo apt update
       sudo apt install -y python3-rosdep
       sudo rosdep init
       rosdep update


Installing the missing dependencies
-----------------------------------

Set your rosdistro according to the release you downloaded.

.. code-block:: bash

       rosdep install --from-paths ros2-linux/share --ignore-src --rosdistro rolling -y --skip-keys "console_bridge fastcdr fastrtps osrf_testing_tools_cpp poco_vendor rmw_connext_cpp rosidl_typesupport_connext_c rosidl_typesupport_connext_cpp rti-connext-dds-5.3.1 tinyxml_vendor tinyxml2_vendor urdfdom urdfdom_headers"

#. *Optional*\ : if you want to use the ROS 1<->2 bridge, then you must also install ROS 1.
   Follow the normal install instructions: http://wiki.ros.org/noetic/Installation/Ubuntu

Installing the python3 libraries
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

       sudo apt install -y libpython3-dev

Install additional DDS implementations (optional)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you would like to use another DDS or RTPS vendor besides the default, eProsima's Fast RTPS, you can find instructions `here <../DDS-Implementations>`.

Environment setup
-----------------

Sourcing the setup script
^^^^^^^^^^^^^^^^^^^^^^^^^

Set up your environment by sourcing the following file.

.. code-block:: bash

  . ~/ros2_rolling/ros2-linux/setup.bash

Installing python3 argcomplete (optional)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

ROS 2 command line tools use argcomplete for autocompletion.
So if you want autocompletion, installing argcomplete is necessary.

.. code-block:: bash

   sudo apt install python3-argcomplete


Try some examples
-----------------

In one terminal, source the setup file and then run a C++ ``talker``:

.. code-block:: bash

   . ~/ros2_rolling/ros2-linux/setup.bash
   ros2 run demo_nodes_cpp talker

In another terminal source the setup file and then run a Python ``listener``:

.. code-block:: bash

   . ~/ros2_rolling/ros2-linux/setup.bash
   ros2 run demo_nodes_py listener

You should see the ``talker`` saying that it's ``Publishing`` messages and the ``listener`` saying ``I heard`` those messages.
This verifies both the C++ and Python APIs are working properly.
Hooray!

See the `tutorials and demos </Tutorials>` for other things to try.

Using the ROS 1 bridge
----------------------

If you have ROS 1 installed, you can try the ROS 1 bridge, by first sourcing your ROS 1 setup file.
We'll assume that it is ``/opt/ros/noetic/setup.bash`` in the following.

If you haven't already, start a roscore:

.. code-block:: bash

   . /opt/ros/noetic/setup.bash
   roscore


In another terminal, start the bridge:

.. code-block:: bash

   . /opt/ros/noetic/setup.bash
   . ~/ros2_rolling/ros2-linux/setup.bash
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
   This way, your environment will behave as though there is no Rolling install on your system.

2. If you're also trying to free up space, you can delete the entire workspace directory with:

   .. code-block:: bash

    rm -rf ~/ros2_rolling
