.. redirect-from::

   Installation/Linux-Install-Debians

Ubuntu (Debian)
===============

.. contents:: Table of Contents
   :depth: 2
   :local:

Debian packages for ROS 2 {DISTRO_TITLE_FULL} are currently available for Ubuntu Jammy.
The Rolling Ridley distribution will change target platforms from time to time as new platforms are selected for development.
The target platforms are defined in `REP 2000 <https://github.com/ros-infrastructure/rep/blob/master/rep-2000.rst>`__
Most people will want to use a stable ROS distribution.

Resources
---------

* Status Page:

  * ROS 2 {DISTRO_TITLE} (Ubuntu Jammy): `amd64 <http://repo.ros2.org/status_page/ros_{DISTRO}_default.html>`__\ , `arm64 <http://repo.ros2.org/status_page/ros_{DISTRO}_ufv8.html>`__
* `Jenkins Instance <http://build.ros2.org/>`__
* `Repositories <http://repo.ros2.org>`__


Set locale
----------

.. include:: ../_Ubuntu-Set-Locale.rst

.. _linux-install-debians-setup-sources:

Setup Sources
-------------

.. include:: ../_Apt-Repositories.rst

.. _linux-install-debians-install-ros-2-packages:

Install ROS 2 packages
----------------------

Update your apt repository caches after setting up the repositories.

.. code-block:: bash

   sudo apt update

Desktop Install (Recommended): ROS, RViz, demos, tutorials.

.. code-block:: bash

   sudo apt install ros-{DISTRO}-desktop

Desktop-Full Install: ROS, RViz, demos, tutorials, simulation, perception.

.. code-block:: bash

   sudo apt install ros-{DISTRO}-desktop-full

ROS-Base Install (Bare Bones): Communication libraries, message packages, command line tools.
No GUI tools.

.. code-block:: bash

   sudo apt install ros-{DISTRO}-ros-base

Environment setup
-----------------

Sourcing the setup script
^^^^^^^^^^^^^^^^^^^^^^^^^

Set up your environment by sourcing the following file.

.. code-block:: bash

   source /opt/ros/{DISTRO}/setup.bash

Try some examples
-----------------

Talker-listener
^^^^^^^^^^^^^^^

If you installed ``ros-{DISTRO}-desktop`` above you can try some examples.

In one terminal, source the setup file and then run a C++ ``talker``\ :

.. code-block:: bash

   source /opt/ros/{DISTRO}/setup.bash
   ros2 run demo_nodes_cpp talker

In another terminal source the setup file and then run a Python ``listener``\ :

.. code-block:: bash

   source /opt/ros/{DISTRO}/setup.bash
   ros2 run demo_nodes_py listener

You should see the ``talker`` saying that it's ``Publishing`` messages and the ``listener`` saying ``I heard`` those messages.
This verifies both the C++ and Python APIs are working properly.
Hooray!

Differential drive robot
^^^^^^^^^^^^^^^^^^^^^^^^

If you installed ``ros-{DISTRO}-desktop-full`` above you can try some simulation examples.

In one terminal, source the setup file and then launch a simulated differential drive robot:

.. code-block:: bash

   source /opt/ros/{DISTRO}/setup.bash
   ros2 launch ros_ign_gazebo_demos diff_drive.launch.py

The Gazebo simulator and RViz visualization should come up.

In another terminal source the setup file and then send a command to a robot:

.. code-block:: bash

   source /opt/ros/{DISTRO}/setup.bash
   ros2 topic pub /model/vehicle_blue/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.05}}"

You should see the vehicle start moving, and its odometry be updated on RViz.
This verifies that simulation and visualization are working properly.
Hooray! You can try more simulation demos `here <https://github.com/gazebosim/ros_gz/tree/ros2/ros_ign_gazebo_demos>`__.

Next steps after installing
---------------------------
Continue with the :doc:`tutorials and demos <../../Tutorials>` to configure your environment, create your own workspace and packages, and learn ROS 2 core concepts.

Using the ROS 1 bridge
----------------------
The ROS 1 bridge can connect topics from ROS 1 to ROS 2 and vice-versa. See the dedicated `documentation <https://github.com/ros2/ros1_bridge/blob/master/README.md>`__ on how to build and use the ROS 1 bridge.

Additional RMW implementations (optional)
-----------------------------------------
The default middleware that ROS 2 uses is ``Fast DDS``, but the middleware (RMW) can be replaced at runtime.
See the :doc:`guide <../../How-To-Guides/Working-with-multiple-RMW-implementations>` on how to work with multiple RMWs.

Troubleshooting
---------------

Troubleshooting techniques can be found :doc:`here <../../How-To-Guides/Installation-Troubleshooting>`.

Uninstall
---------

If you need to uninstall ROS 2 or switch to a source-based install once you
have already installed from binaries, run the following command:

.. code-block:: bash

  sudo apt remove ~nros-{DISTRO}-* && sudo apt autoremove

You may also want to remove the repository:

.. code-block:: bash

  sudo rm /etc/apt/sources.list.d/ros2.list
  sudo apt update
  sudo apt autoremove
  # Consider upgrading for packages previously shadowed.
  sudo apt upgrade
