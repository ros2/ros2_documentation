Setting up a robot simulation (Advanced)
========================================

**Goal:** Extend a robot simulation with an obstacle avoider node.

**Tutorial level:** Advanced

**Time:** 20 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

In this tutorial you will extend the package created in the first part of the tutorial: :doc:`./Setting-Up-Simulation-Webots-Basic`.
The aim is to implement a ROS 2 node that avoids obstacles using the robot's distance sensors.
This tutorial focuses on using robot devices with the ``webots_ros2_driver`` interface.

Prerequisites
-------------

This is a continuation of the first part of the tutorial: :doc:`./Setting-Up-Simulation-Webots-Basic`.
It is mandatory to start with the first part to set up the custom packages and necessary files.

Tasks
-----

1 Updating ``my_robot.urdf``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

As mentioned in :doc:`./Setting-Up-Simulation-Webots-Basic`, ``webots_ros2_driver`` contains plugins to interface most of Webots devices with ROS 2 directly.
These plugins can be loaded using the ``<device>`` tag in the URDF file of the robot.
The ``reference`` attribute should match the Webots device ``name`` parameter.
The list of all existing interfaces and the corresponding parameters can be found `on the devices reference page <https://github.com/cyberbotics/webots_ros2/wiki/References-Devices>`_.
For available devices that are not configured in the URDF file, the interface will be automatically created and default values will be used for ROS parameters (e.g. ``update rate``, ``topic name``, and ``frame name``).

In ``my_robot.urdf`` replace the whole contents with:

.. tabs::

    .. group-tab:: Python

        .. literalinclude:: Code/my_robot_with_sensors_python.urdf
            :language: xml

    .. group-tab:: C++

        .. literalinclude:: Code/my_robot_with_sensors_cpp.urdf
            :language: xml


In addition to your custom plugin, the ``webots_ros2_driver`` will parse the ``<device>`` tags referring to the **DistanceSensor** nodes and use the standard parameters in the ``<ros>`` tags to enable the sensors and name their topics.

2 Creating a ROS node to avoid obstacles
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. tabs::

    .. group-tab:: Python

        The robot will use a standard ROS node to detect the wall and send motor commands to avoid it.
        In the ``my_package/my_package/`` folder, create a file named ``obstacle_avoider.py`` with this code:

        .. literalinclude:: Code/obstacle_avoider.py
            :language: python

        This node will create a publisher for the command and subscribe to the sensors topics here:

        .. literalinclude:: Code/obstacle_avoider.py
            :language: python
            :dedent: 8
            :lines: 14-17

        When a measurement is received from the left sensor it will be copied to a member field:

        .. literalinclude:: Code/obstacle_avoider.py
            :language: python
            :dedent: 4
            :lines: 19-20

        Finally, a message will be sent to the ``/cmd_vel`` topic when a measurement from the right sensor is received.
        The ``command_message`` will register at least a forward speed in ``linear.x`` in order to make the robot move when no obstacle is detected.
        If any of the two sensors detect an obstacle, ``command_message`` will also register a rotational speed in ``angular.z`` in order to make the robot turn right.

        .. literalinclude:: Code/obstacle_avoider.py
            :language: python
            :dedent: 4
            :lines: 22-32

    .. group-tab:: C++

        The robot will use a standard ROS node to detect the wall and send motor commands to avoid it.
        In the ``my_package/include/my_package`` folder, create a header file named ``ObstacleAvoider.hpp`` with this code:

        .. literalinclude:: Code/ObstacleAvoider.hpp
            :language: cpp

        In the ``my_package/src`` folder, create a source file named ``ObstacleAvoider.cpp`` with this code:

        .. literalinclude:: Code/ObstacleAvoider.cpp
            :language: cpp

        This node will create a publisher for the command and subscribe to the sensors topics here:

        .. literalinclude:: Code/ObstacleAvoider.cpp
            :language: cpp
            :lines: 6-16

        When a measurement is received from the left sensor it will be copied to a member field:

        .. literalinclude:: Code/ObstacleAvoider.cpp
            :language: cpp
            :lines: 19-22

        Finally, a message will be sent to the ``/cmd_vel`` topic when a measurement from the right sensor is received.
        The ``command_message`` will register at least a forward speed in ``linear.x`` in order to make the robot move when no obstacle is detected.
        If any of the two sensors detect an obstacle, ``command_message`` will also register a rotational speed in ``angular.z`` in order to make the robot turn right.

        .. literalinclude:: Code/ObstacleAvoider.cpp
            :language: cpp
            :lines: 24-38


3 Updating additional files
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You have to modify these two other files to launch your new node.

.. tabs::

    .. group-tab:: Python

        Edit ``setup.py`` and replace ``'console_scripts'`` with:

        .. literalinclude:: Code/setup_sensor.py
            :language: python
            :dedent: 8
            :lines: 24-27

        This will add an entry point for the ``obstacle_avoider`` node.

    .. group-tab:: C++

        Edit ``CMakeLists.txt`` and add the compilation and installation of the ``obstacle_avoider``:

        .. literalinclude:: Code/CMakeLists_sensor.txt
            :language: cmake


Go to the file ``robot_launch.py`` and replace ``def generate_launch_description():`` with:

.. literalinclude:: Code/robot_launch_sensor.py
    :language: python
    :lines: 11-44

This will create an ``obstacle_avoider`` node that will be included in the ``LaunchDescription``.

4 Test the obstacle avoidance code
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Launch the simulation from a terminal in your ROS 2 workspace:

.. tabs::

    .. group-tab:: Linux

        From a terminal in your ROS 2 workspace run:

        .. code-block:: console

            colcon build
            source install/local_setup.bash
            ros2 launch my_package robot_launch.py

    .. group-tab:: Windows

        From a terminal in your WSL ROS 2 workspace run:

        .. code-block:: console

            colcon build
            export WEBOTS_HOME=/mnt/c/Program\ Files/Webots
            source install/local_setup.bash
            ros2 launch my_package robot_launch.py

        Be sure to use the ``/mnt`` prefix in front of your path to the Webots installation folder to access the Windows file system from WSL.

    .. group-tab:: macOS

        In a terminal of the host machine (not in the VM), if not done already, specify the Webots installation folder (e.g. ``/Applications/Webots.app``) and start the server using the following commands:

        .. code-block:: console

            export WEBOTS_HOME=/Applications/Webots.app
            python3 local_simulation_server.py

        Note that the server keeps running once the ROS 2 nodes are ended.
        You don't need to restart it every time you want to launch a new simulation.
        From a terminal in the Linux VM in your ROS 2 workspace, build and launch your custom package with:

        .. code-block:: console

            cd ~/ros2_ws
            colcon build
            source install/local_setup.bash
            ros2 launch my_package robot_launch.py

Your robot should go forward and before hitting the wall it should turn clockwise.
You can press ``Ctrl+F10`` in Webots or go to the ``View`` menu, ``Optional Rendering`` and ``Show DistanceSensor Rays`` to display the range of the distance sensors of the robot.

.. image:: Image/Robot_turning_clockwise.png

Summary
-------

In this tutorial, you extended the basic simulation with a obstacle avoider ROS 2 node that publishes velocity commands based on the distance sensor values of the robot.

Next steps
----------

You might want to improve the plugin or create new nodes to change the behavior of the robot.
Taking inspiration from these previous tutorials could be a starting point:

* :doc:`../../Recording-A-Bag-From-Your-Own-Node-Py`.

* :doc:`../../../Intermediate/Tf2/Tf2-Main`.
