Getting started with MVSim
==========================

**Goal:** Launch MVSim demo worlds both standalone and with ROS 2, and learn how to interact with simulated robots.

**Tutorial level:** Advanced

**Time:** 20 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

MVSim ships with a collection of demo worlds that showcase different features such as multi-robot simulation,
sensor configurations, terrain types, human actors, articulated vehicles and environment layouts.
You can run these demos either as a standalone application using the ``mvsim`` CLI,
or as a ROS 2 node that publishes sensor data and accepts velocity commands through standard ROS 2 topics.

.. image:: Image/mvsim_demos_screenshot.png
   :alt: MVSim demo screenshots

Prerequisites
-------------

You should have MVSim installed following the :doc:`Installation-Ubuntu` tutorial.

Tasks
-----

1 Launch demo worlds with the standalone CLI
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

MVSim includes a standalone launcher that does not require ROS 2.
This is useful for quickly testing world files or for non-ROS use cases.

To launch the warehouse demo:

.. code-block:: console

    $ mvsim launch ~/ros2_ws/src/mvsim/mvsim_tutorial/demo_warehouse.world.xml

If you installed from binary packages, the demo files are typically found under
``/opt/ros/{DISTRO}/share/mvsim/mvsim_tutorial/``.

Some other demo worlds you can try:

- ``demo_turtlebot_world.world.xml`` -- A TurtleBot3 in a classic ROS-style environment with obstacles.
- ``demo_2robots.world.xml`` -- Two robots navigating among furniture blocks.
- ``demo_elevation_map.world.xml`` -- A Jackal robot driving over terrain with elevation data.
- ``demo_greenhouse.world.xml`` -- A complex greenhouse environment demonstrating XML loops for procedural content.

2 Controlling the robot
^^^^^^^^^^^^^^^^^^^^^^^^

Once a world is running, you can control the robot using:

- **Keyboard:** Press W/S to move forward/backward, A/D to turn left/right, and spacebar to stop.
  If the world has multiple robots, click on a robot in the GUI to select it before using keyboard controls.
- **Joystick:** If a gamepad is connected, it will be automatically detected.

.. image:: Image/mvsim_gui_controls.jpg
   :alt: MVSim GUI control reference

The GUI also provides controls for camera view, simulation speed, and visualization options.
You can toggle orthographic/perspective view and enable visualization of sensor data directly in the 3D window.

3 Launch with ROS 2
^^^^^^^^^^^^^^^^^^^^^

To launch MVSim as a ROS 2 node, use the provided launch files:

.. code-block:: console

    $ source /opt/ros/{DISTRO}/setup.bash
    $ ros2 launch mvsim demo_warehouse.launch.py

This starts the simulator and creates ROS 2 topics for each vehicle and sensor.

4 Inspect ROS 2 topics
^^^^^^^^^^^^^^^^^^^^^^^^

With the demo running, open a new terminal and list the available topics:

.. code-block:: console

    $ ros2 topic list

You should see topics such as:

- ``/robot1/cmd_vel`` -- Send ``geometry_msgs/msg/Twist`` commands to control the robot.
- ``/robot1/odom`` -- Odometry from wheel encoders (``nav_msgs/msg/Odometry``).
- ``/robot1/base_pose_ground_truth`` -- Perfect ground truth pose.
- ``/robot1/<sensor_name>`` -- Sensor-specific topics (e.g., ``/robot1/lidar1_points`` for 3D LiDAR point clouds, ``/robot1/laser1`` for 2D scans).
- ``/tf`` and ``/tf_static`` -- TF2 transforms following `REP-105 <https://www.ros.org/reps/rep-0105.html>`__ (``map`` → ``odom`` → ``base_link``).

You can send velocity commands from the command line:

.. code-block:: console

    $ ros2 topic pub /robot1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.3}}"

Or use ``teleop_twist_keyboard`` for interactive control:

.. code-block:: console

    $ ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/robot1/cmd_vel

5 Visualize in RViz2
^^^^^^^^^^^^^^^^^^^^

You can visualize MVSim sensor data in RViz2.
Some launch files include an ``use_rviz`` option:

.. code-block:: console

    $ ros2 launch mvsim demo_warehouse.launch.py use_rviz:=True

Alternatively, open RViz2 manually and add displays for the topics of interest (e.g., ``LaserScan``, ``PointCloud2``, ``Image``, ``Odometry``).

.. image:: Image/mvsim_depth_camera_demo.png
   :alt: MVSim depth camera visualization

6 Headless mode
^^^^^^^^^^^^^^^

For CI pipelines or remote servers without a display, MVSim supports headless operation:

.. code-block:: console

    $ ros2 launch mvsim demo_warehouse.launch.py headless:=True

This runs the full simulation without opening a GUI window.

Summary
-------

In this tutorial, you launched MVSim demo worlds both standalone and with ROS 2.
You learned how to control robots with keyboard and ROS 2 topics, inspect the published topics, and visualize data in RViz2.
The next tutorial covers how to define your own worlds with custom robots and sensors.
