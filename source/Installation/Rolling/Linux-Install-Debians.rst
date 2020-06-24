Installing ROS 2 via Debian Packages
====================================

.. contents:: Table of Contents
   :depth: 2
   :local:

Debian packages for ROS 2 Rolling Ridley are currently available for Ubuntu Focal.
The Rolling Ridley distribution will change target platforms from time to time as new platforms are selected for development.
The target platforms are defined in `REP 2000 <https://github.com/ros-infrastructure/rep/blob/master/rep-2000.rst>`__
Most people will want to use a stable ROS distribution.

Resources
---------

* Status Page:

  * ROS 2 Rolling (Ubuntu Focal): `amd64 <http://repo.ros2.org/status_page/ros_rolling_default.html>`__\ , `arm64 <http://repo.ros2.org/status_page/ros_rolling_ubv8.html>`__
* `Jenkins Instance <http://build.ros2.org/>`__
* `Repositories <http://repo.ros2.org>`__


Setup Locale
------------
Make sure you have a locale which supports ``UTF-8``.
If you are in a minimal environment, such as a docker container, the locale may be something minimal like POSIX.
We test with the following settings.
It should be fine if you're using a different UTF-8 supported locale.

.. code-block:: bash

   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   export LANG=en_US.UTF-8

Setup Sources
-------------

.. include:: ../_Apt-Repositories.rst

.. _Rolling_linux-install-debians-install-ros-2-packages:

Install ROS 2 packages
----------------------

Update your apt repository caches after setting up the repositories.

.. code-block:: bash

   sudo apt update

Desktop Install (Recommended): ROS, RViz, demos, tutorials.

.. code-block:: bash

   sudo apt install ros-rolling-desktop

ROS-Base Install (Bare Bones): Communication libraries, message packages, command line tools.
No GUI tools.

.. code-block:: bash

   sudo apt install ros-rolling-ros-base

See specific sections below for how to also install the :ref:`ros1_bridge <Rolling_linux-ros1-add-pkgs>`, :ref:`TurtleBot packages <Rolling_linux-ros1-add-pkgs>`, or :ref:`alternative RMW packages <Rolling_linux-install-additional-rmw-implementations>`.

Environment setup
-----------------

Sourcing the setup script
^^^^^^^^^^^^^^^^^^^^^^^^^

Set up your environment by sourcing the following file.

.. code-block:: bash

   source /opt/ros/rolling/setup.bash

Install argcomplete (optional)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

ROS 2 command line tools use argcomplete to autocompletion.
So if you want autocompletion, installing argcomplete is necessary.

.. code-block:: bash

   sudo apt install python3-argcomplete

Try some examples
-----------------

If you installed ``ros-rolling-desktop`` above you can try some examples.

In one terminal, source the setup file and then run a C++ ``talker``\ :

.. code-block:: bash

   source /opt/ros/rolling/setup.bash
   ros2 run demo_nodes_cpp talker

In another terminal source the setup file and then run a Python ``listener``\ :

.. code-block:: bash

   source /opt/ros/rolling/setup.bash
   ros2 run demo_nodes_py listener

You should see the ``talker`` saying that it's ``Publishing`` messages and the ``listener`` saying ``I heard`` those messages.
This verifies both the C++ and Python APIs are working properly.
Hooray!

See the `tutorials and demos </Tutorials>` for other things to try.

.. _Rolling_linux-install-additional-rmw-implementations:

Install additional RMW implementations (optional)
-------------------------------------------------

By default the RMW implementation ``Fast RTPS`` is used.
``Cyclone DDS`` is also installed.

To install support for ``RTI Connext``:

.. code-block:: bash

   sudo apt update
   sudo apt install ros-rolling-rmw-connext-cpp # for RTI Connext (requires license agreement)

By setting the environment variable ``RMW_IMPLEMENTATION=rmw_connext_cpp`` you can switch to use RTI Connext instead.

You can also install `the Connext DDS-Security plugins <../DDS-Implementations/Install-Connext-Security-Plugins>` or use the `University, purchase or evaluation <../DDS-Implementations/Install-Connext-University-Eval>` options for RTI Connext.

.. _Rolling_linux-ros1-add-pkgs:

Install additional packages using ROS 1 packages
------------------------------------------------

The ``ros1_bridge`` as well as the TurtleBot demos are using ROS 1 packages.
To be able to install them please start by adding the ROS 1 sources as documented `here <http://wiki.ros.org/Installation/Ubuntu?distro=melodic>`__.

If you're using Docker for isolation you can start with the image ``ros:noetic`` or ``osrf/ros:noetic-desktop``.
This will also avoid the need to setup the ROS sources as they will already be integrated.

Now you can install the remaining packages:

.. code-block:: bash

   sudo apt update
   sudo apt install ros-rolling-ros1-bridge

The turtlebot2 packages are not currently available in Rolling.

Build your own packages
-----------------------

If you would like to build your own packages, refer to the tutorial `"Using Colcon to build packages" </Tutorials/Colcon-Tutorial>`.

Troubleshooting
---------------

Troubleshooting techniques can be found `here </Troubleshooting>`.

Uninstall
---------

If you need to uninstall ROS 2 or switch to a source-based install once you
have already installed from binaries, run the following command:

.. code-block:: bash

  sudo apt remove ros-rolling-* && sudo apt autoremove
