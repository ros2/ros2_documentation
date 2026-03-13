Installation (Ubuntu)
=====================

**Goal:** Install the ``mvsim`` package on Ubuntu and verify it works.

**Tutorial level:** Advanced

**Time:** 10 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

`MVSim <https://mvsimulator.readthedocs.io/>`__ (MultiVehicle Simulator) is a lightweight, open-source simulator for mobile robots.
It provides 2D physics-based simulation with 3D visualization, supporting differential drive and Ackermann vehicles,
multiple sensor types (LiDAR, cameras, IMU, GPS), and native ROS 2 integration via standard message types.

MVSim is licensed under the BSD 3-clause license.

Prerequisites
-------------

It is recommended to understand basic ROS principles covered in the beginner :doc:`../../../../Tutorials`.
In particular, :doc:`../../../Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace` is a useful prerequisite.

You should have a working ROS 2 installation.
Follow the :doc:`ROS 2 install instructions <../../../../Installation>` if needed.

Tasks
-----

1 Install ``mvsim``
^^^^^^^^^^^^^^^^^^^

You can either install the released binary package or build from sources.

.. tabs::

    .. group-tab:: Install from ROS binary packages

        Run the following command in a terminal:

        .. code-block:: console

            $ sudo apt install ros-{DISTRO}-mvsim

    .. group-tab:: Build from sources

        Create a ROS 2 workspace if you don't already have one:

        .. code-block:: console

            $ mkdir -p ~/ros2_ws/src

        Source the ROS 2 environment:

        .. code-block:: console

            $ source /opt/ros/{DISTRO}/setup.bash

        Clone the MVSim repository:

        .. code-block:: console

            $ cd ~/ros2_ws/src
            $ git clone https://github.com/MRPT/mvsim.git --recursive

        Install dependencies using ``rosdep``:

        .. code-block:: console

            $ cd ~/ros2_ws
            $ rosdep install --from-paths src --ignore-src -r -y

        Build the package:

        .. code-block:: console

            $ colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

        Source the workspace:

        .. code-block:: console

            $ source install/setup.bash

2 Verify the installation
^^^^^^^^^^^^^^^^^^^^^^^^^

Check that the ``mvsim`` CLI is available:

.. code-block:: console

    $ mvsim --version

You should see the installed version number printed to the terminal.

.. warning::

   The ``mvsim`` package provides two executables:

   - ``mvsim``: the main CLI tool for running the simulator standalone
   - ``mvsim_node``: a ROS 2 node wrapper for running the simulator and connect it to other ROS 2 nodes

3 Launch a demo
^^^^^^^^^^^^^^^

To quickly verify everything is working, launch the warehouse demo with ROS 2:

.. code-block:: console

    $ ros2 launch mvsim demo_warehouse.launch.py

You should see the MVSim GUI window open with a Jackal robot in a warehouse environment.
Use the keyboard (W/A/S/D keys) to drive the robot.

Summary
-------

You have installed MVSim and verified it works by launching a demo world.
In the next tutorial, you will learn how to launch different demo scenarios and interact with them through ROS 2 topics.
