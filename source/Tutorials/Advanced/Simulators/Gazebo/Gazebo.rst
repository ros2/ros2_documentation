.. redirect-from::

    Tutorials/Simulators/Ignition/Setting-up-a-Robot-Simulation-Ignition
    Tutorials/Advanced/Simulators/Ignition
    Tutorials/Advanced/Simulators/Gazebo

Setting up a robot simulation (Gazebo)
======================================

**Goal:** Launch a Simulation with Gazebo and ROS 2

**Tutorial level:** Advanced

**Time:** 20 minutes

.. contents:: Contents
   :depth: 2
   :local:

Prerequisites
-------------

First of all you should install ROS 2 and Gazebo.
You have two options:

 - Install from deb packages. To check which versions are available from deb packages please check this `table <https://github.com/gazebosim/ros_ign>`__.
 - Compile from sources:

   - :doc:`ROS 2 install instructions <../../../../Installation>`
   - `Gazebo install instructions <https://gazebosim.org/docs>`__

Tasks
-----

1 Launch the simulation
^^^^^^^^^^^^^^^^^^^^^^^

In this demo you are going to simulate a simple diff drive robot in Gazebo.
You are going to use one of the worlds defined in the Gazebo examples called
`visualize_lidar.sdf <https://github.com/gazebosim/gz-sim/blob/main/examples/worlds/visualize_lidar.sdf>`__.
To run this example you should execute the following command in a terminal:


.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        ign gazebo -v 4 -r visualize_lidar.sdf

.. image:: Image/gazebo_diff_drive.png

When the simulation is running you can check the topics provided by Gazebo with the ``ign`` command line tool:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        ign topic -l

      Which should show:

      .. code-block:: console

        /clock
        /gazebo/resource_paths
        /gui/camera/pose
        /gui/record_video/stats
        /model/vehicle_blue/odometry
        /model/vehicle_blue/tf
        /stats
        /world/visualize_lidar_world/clock
        /world/visualize_lidar_world/dynamic_pose/info
        /world/visualize_lidar_world/pose/info
        /world/visualize_lidar_world/scene/deletion
        /world/visualize_lidar_world/scene/info
        /world/visualize_lidar_world/state
        /world/visualize_lidar_world/stats

Since you have not launched an ROS 2 nodes yet, the output from ``ros2 topic list``
should be free of any robot topics:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        ros2 topic list

      Which should show:

      .. code-block:: console

        /parameter_events
        /rosout

2 Configuring ROS 2
^^^^^^^^^^^^^^^^^^^

To be able to communicate our simulation with ROS 2 you need to use a package called ``ros_gz_bridge``.
This package provides a network bridge which enables the exchange of messages between ROS 2 and Gazebo Transport.
You can install this package by typing:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        sudo apt-get install ros-{DISTRO}-ros-ign-bridge

At this point you are ready to launch a bridge from ROS to Gazebo.
In particular you are going to create a bridge for the topic ``/model/vehicle_blue/cmd_vel``:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        source /opt/ros/{DISTRO}/setup.bash
        ros2 run ros_gz_bridge parameter_bridge /model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist

For more details about the ``ros_gz_bridge`` please check this `README <https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge>`__ .

Once the bridge is running the robot is able to follow your motor commands.
There are two options:

* Send a command to the topic using ``ros2 topic pub``

 .. tabs::

    .. group-tab:: Linux

       .. code-block:: console

        ros2 topic pub /model/vehicle_blue/cmd_vel geometry_msgs/Twist "linear: { x: 0.1 }"

* ``teleop_twist_keyboard`` package. This node takes keypresses from the keyboard and publishes them as Twist messages. You can install it typing:

 .. tabs::

    .. group-tab:: Linux

       .. code-block:: console

         sudo apt-get install ros-{DISTRO}-teleop-twist-keyboard

 The default topic where ``teleop_twist_keyboard`` is publishing Twist messages is ``/cmd_vel`` but you can remap this
 topic to make use of the topic used in the bridge:

 .. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        source /opt/ros/{DISTRO}/setup.bash
        ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/model/vehicle_blue/cmd_vel

      Which will show:

      .. code-block:: console

        This node takes keypresses from the keyboard and publishes them
        as Twist messages. It works best with a US keyboard layout.
        ---------------------------
        Moving around:
           u    i    o
           j    k    l
           m    ,    .

        For Holonomic mode (strafing), hold down the shift key:
        ---------------------------
           U    I    O
           J    K    L
           M    <    >

        t : up (+z)
        b : down (-z)

        anything else : stop

        q/z : increase/decrease max speeds by 10%
        w/x : increase/decrease only linear speed by 10%
        e/c : increase/decrease only angular speed by 10%

        CTRL-C to quit

        currently:	speed 0.5	turn 1.0

3 Visualizing lidar data in ROS 2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The diff drive robot has a lidar.
To send the data generated by Gazebo to ROS 2, you need to launch another bridge.
In the case the data from the lidar is provided in the Gazebo Transport topic ``/lidar2``, which you are going to remap in the bridge.
This topic will be available under the topic ``/lidar_scan``:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        source /opt/ros/{DISTRO}/setup.bash
        ros2 run ros_gz_bridge parameter_bridge /lidar2@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan --ros-args -r /lidar2:=/laser_scan

To visualize the data from the lidar in ROS 2 you can use Rviz2:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        source /opt/ros/{DISTRO}/setup.bash
        rviz2

Then you need to configure the ``fixed frame``:

.. image:: Image/fixed_frame.png

And then click in the button "Add" to include a display to visualize the lidar:

.. image:: Image/add_lidar.png

Now you should see the data from the lidar in Rviz2:

.. image:: Image/rviz2.png

Summary
-------

In this tutorial, you launched a robot simulation with Gazebo, launched
bridges with actuators and sensors, visualized data from a sensor, and moved a diff drive robot.
