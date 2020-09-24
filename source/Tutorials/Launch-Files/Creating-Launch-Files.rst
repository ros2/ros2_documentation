.. _ROS2Launch:

Creating a launch file
======================

**Goal:** Create a launch file to run a complex ROS 2 system.

**Tutorial level:** Beginner

**Time:** 10 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

In the tutorials up until now, you have been opening new terminals for every new node you run.
As you create more complex systems with more and more nodes running simultaneously, opening terminals and reentering configuration details becomes tedious.

Launch files allow you to start up and configure a number of executables containing ROS 2 nodes simultaneously.

Running a single launch file with the ``ros2 launch`` command will start up your entire system - all nodes and their configurations - at once.


Prerequisites
-------------

This tutorial uses the :ref:`rqt_graph and turtlesim <Turtlesim>` packages.

You will also need to use a text editor of your preference.

As always, don’t forget to source ROS 2 in :ref:`every new terminal you open <ConfigROS2>`.


Tasks
-----

1 Setup
^^^^^^^

Create a launch file named ``turtlesim_mimic_launch.xml`` or ``turtlesim_mimic_launch.py``, depending if you're using XML or Python launch files.
Open it in your preferred text editor.

2 Write the launch file
^^^^^^^^^^^^^^^^^^^^^^^

Let’s put together a ROS 2 launch file using the ``turtlesim`` package and its executables.

Copy and paste the complete code into the ``turtlesim_mimic_launch.py`` file:

.. tabs::

  .. group-tab:: XML launch file

    .. code-block:: xml

        <launch>
          <node pkg="turtlesim" exec="turtlesim_node" name="sim" namespace="turtlesim1"/>
          <node pkg="turtlesim" exec="turtlesim_node" name="sim" namespace="turtlesim2"/>
          <node pkg="turtlesim" exec="mimic" name="mimic">
            <remap from="/input/pose" to="/turtlesim1/turtle1/pose"/>
            <remap from="/output/cmd_vel" to="/turtlesim2/turtle1/cmd_vel"/>
          </node>
        </launch>

  .. group-tab:: Python launch file, Foxy and newer

    .. code-block:: python

        from launch import LaunchDescription
        from launch_ros.actions import Node

        def generate_launch_description():
            return LaunchDescription([
                Node(
                    package='turtlesim',
                    namespace='turtlesim1',
                    executable='turtlesim_node',
                    name='sim'
                ),
                Node(
                    package='turtlesim',
                    namespace='turtlesim2',
                    executable='turtlesim_node',
                    name='sim'
                ),
                Node(
                    package='turtlesim',
                    executable='mimic',
                    name='mimic',
                    remappings=[
                        ('/input/pose', '/turtlesim1/turtle1/pose'),
                        ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
                    ]
                )
            ])

  .. group-tab:: Python launch file, Eloquent and older

    .. code-block:: python

         from launch import LaunchDescription
         from launch_ros.actions import Node

         def generate_launch_description():
             return LaunchDescription([
                 Node(
                     package='turtlesim',
                     node_namespace='turtlesim1',
                     node_executable='turtlesim_node',
                     node_name='sim'
                 ),
                 Node(
                     package='turtlesim',
                     node_namespace='turtlesim2',
                     node_executable='turtlesim_node',
                     node_name='sim'
                 ),
                 Node(
                     package='turtlesim',
                     node_executable='mimic',
                     node_name='mimic',
                     remappings=[
                         ('/input/pose', '/turtlesim1/turtle1/pose'),
                         ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
                     ]
                 )
             ])


2.1 Examine the launch file
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Launch files begin with a launch description:

.. code-block:: xml

  <launch>

  </launch>

.. code-block:: python

  def generate_launch_description():
    return LaunchDescription([

    ])

The launch description is a collection of actions that describes the system to be run.
In this case, there is a system of three nodes, all from the ``turtlesim`` package.
The goal of the system is to launch two turtlesim windows, and have one turtle mimic the movements of the other.

The first two actions in the launch description launch two turtlesim windows:

.. tabs::

  .. group-tab:: XML launch file

    .. code-block:: xml

      <node pkg="turtlesim" exec="turtlesim_node" name="sim" namespace="turtlesim1"/>
      <node pkg="turtlesim" exec="turtlesim_node" name="sim" namespace="turtlesim2"/>

  .. group-tab:: Python, Foxy and newer

    .. code-block:: python

           Node(
               package='turtlesim',
               namespace='turtlesim1',
               executable='turtlesim_node',
               name='sim'
           ),
           Node(
               package='turtlesim',
               namespace='turtlesim2',
               executable='turtlesim_node',
               name='sim'
           ),

  .. group-tab:: Python, Eloquent and older

    .. code-block:: python

            Node(
                package='turtlesim',
                node_namespace='turtlesim1',
                node_executable='turtlesim_node',
                node_name='sim'
            ),
            Node(
                package='turtlesim',
                node_namespace='turtlesim2',
                node_executable='turtlesim_node',
                node_name='sim'
            ),

Note the only difference between the two nodes is their namespace values.
Unique namespaces allow the system to start two simulators without node name nor topic name conflicts.

Both turtles in this system receive commands over the same topic and publish their pose over the same topic.
Without unique namespaces, there would be no way to distinguish between messages meant for one turtle or the other.

The final node is also from the ``turtlesim`` package, but a different executable: ``mimic``.

.. tabs::


  .. group-tab:: XML launch file

    .. code-block:: xml

          <node pkg="turtlesim" exec="mimic" name="mimic">
            <remap from="/input/pose" to="/turtlesim1/turtle1/pose"/>
            <remap from="/output/cmd_vel" to="/turtlesim2/turtle1/cmd_vel"/>
          </node>

  .. group-tab:: Python, Foxy and newer

    .. code-block:: python

          Node(
              package='turtlesim',
              executable='mimic',
              name='mimic',
              remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
              ]
          )

  .. group-tab:: Python, Eloquent and older

    .. code-block:: python

            Node(
                package='turtlesim',
                node_executable='mimic',
                node_name='mimic',
                remappings=[
                  ('/input/pose', '/turtlesim1/turtle1/pose'),
                  ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
                ]
            )


This node has added configuration details in the form of remappings.

``mimic``'s ``/input/pose`` topic is remapped to ``/turtlesim1/turtle1/pose`` and it's ``/output/cmd_vel`` topic to ``/turtlesim2/turtle1/cmd_vel``.
This means ``mimic`` will subscribe to ``/turtlesim1/sim``'s pose topic and republish it for ``/turtlesim2/sim``'s velocity command topic to subscribe to.
In other words, ``turtlesim2`` will mimic ``turtlesim1``'s movements.


3 ros2 launch
^^^^^^^^^^^^^

Open in a terminal the directory where you created the launch file, then you can run the following command:

.. code-block:: console

  ros2 launch turtlesim_mimic_launch.py

.. note::

  It is possible to launch a launch file directly (as we do above), or provided by a package.
  When it is provided by a package, the syntax is:

  .. code-block:: console

      ros2 launch <package_name> <launch_file_name>

  You will learn more about :ref:`creating packages <CreatePkg>` in a later tutorial.

Two turtlesim windows will open, and you will see the following ``[INFO]`` messages telling you which nodes your launch file has started:

.. code-block:: console

  [INFO] [launch]: Default logging verbosity is set to INFO
  [INFO] [turtlesim_node-1]: process started with pid [11714]
  [INFO] [turtlesim_node-2]: process started with pid [11715]
  [INFO] [mimic-3]: process started with pid [11716]

To see the system in action, open a new terminal and run the ``ros2 topic pub`` command on the ``/turtlesim1/turtle1/cmd_vel`` topic to get the first turtle moving:

.. code-block:: console

  ros2 topic pub -r 1 /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"

You will see both turtles following the same path.

.. image:: mimic.png

4 Introspect the system with rqt_graph
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

While the system is still running, open a new terminal and run ``rqt_graph`` to get a better idea of the relationship between the nodes in your launch file.

Run the command:

.. code-block:: console

  rqt_graph

.. image:: mimic_graph.png

A hidden node (the ``ros2 topic pub`` command you ran) is publishing data to the ``/turtlesim1/turtle1/cmd_vel`` topic on the left, which the ``/turtlesim1/sim`` node is subscribed to.
The rest of the graph shows what was described earlier: ``mimic`` is subscribed to ``/turtlesim1/sim``'s pose topic, and publishes to ``/turtlesim2/sim``'s velocity command topic.

5 Make your launch file configurable with arguments and substitutions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

We're going to now edit the same launch file we previously created to make it more configurable:

.. tabs::

  .. group-tab:: XML launch file

    .. code-block:: xml

        <launch>
          <arg name="turtlesim_ns1" default="turtlesim1" description="namespace of one of the turtlesim to run"/>
          <arg name="turtlesim_ns2" default="turtlesim2" description="namespace of the other turtlesim to run"/>
          <arg name="mimic_name" default="mimic" description="name of the mimicking node"/>

          <node pkg="turtlesim" exec="turtlesim_node" name="sim" namespace="$(var turtlesim_ns1)"/>
          <node pkg="turtlesim" exec="turtlesim_node" name="sim" namespace="$(var turtlesim_ns2)"/>
          <node pkg="turtlesim" exec="mimic" name="$(var mimic_name)">
            <remap from="/input/pose" to="/$(var turtlesim_ns1)/turtle1/pose"/>
            <remap from="/output/cmd_vel" to="/$(var turtlesim_ns2)/turtle1/cmd_vel"/>
          </node>
        </launch>

  .. group-tab:: Python launch file, Foxy and newer

    .. code-block:: python

        from launch import LaunchDescription
        from launch.actions import DeclareLaunchArgument
        from launch.substitutions import LaunchConfiguration
        from launch_ros.actions import Node

        def generate_launch_description():
            return LaunchDescription([
                DeclareLaunchArgument(
                  'turtlesim_ns1',
                  default_value='turtlesim1',
                  description='namespace of one of the turtlesim to run',
                ),
                DeclareLaunchArgument(
                  'turtlesim_ns2',
                  default_value='turtlesim2',
                  description='namespace of the other turtlesim to run',
                ),
                DeclareLaunchArgument(
                  'mimic_name',
                  default_value='mimic',
                  description='name of the mimicking node',
                ),
                Node(
                    package='turtlesim',
                    namespace=LaunchConfiguration('turtlesim_ns1'),
                    executable='turtlesim_node',
                    name='sim'
                ),
                Node(
                    package='turtlesim',
                    namespace=LaunchConfiguration('turtlesim_ns2'),
                    executable='turtlesim_node',
                    name='sim'
                ),
                Node(
                    package='turtlesim',
                    executable='mimic',
                    name=LaunchConfiguration('mimic_name'),
                    remappings=[
                        ('input/pose', [LaunchConfiguration('turtlesim_ns1'), '/turtle1/pose']),
                        ('output/cmd_vel', [LaunchConfiguration('turtlesim_ns2'), '/turtle1/cmd_vel']),
                    ]
                )
            ])

  .. group-tab:: Python launch file, Eloquent and older

    .. code-block:: python

        from launch import LaunchDescription
        from launch.actions import DeclareLaunchArgument
        from launch.substitutions import LaunchConfiguration
        from launch_ros.actions import Node

        def generate_launch_description():
            return LaunchDescription([
                DeclareLaunchArgument(
                  'turtlesim_ns1',
                  default_value='turtlesim1',
                  description='namespace of one of the turtlesim to run',
                ),
                DeclareLaunchArgument(
                  'turtlesim_ns2',
                  default_value='turtlesim2',
                  description='namespace of the other turtlesim to run',
                ),
                DeclareLaunchArgument(
                  'mimic_name',
                  default_value='mimic',
                  description='name of the mimicking node',
                ),
                Node(
                    package='turtlesim',
                    node_namespace=LaunchConfiguration('turtlesim_ns1'),
                    node_executable='turtlesim_node',
                    node_name='sim'
                ),
                Node(
                    package='turtlesim',
                    node_namespace=LaunchConfiguration('turtlesim_ns2'),
                    node_executable='turtlesim_node',
                    node_name='sim'
                ),
                Node(
                    package='turtlesim',
                    node_executable='mimic',
                    node_name=LaunchConfiguration('mimic_name'),
                    remappings=[
                        ('input/pose', [LaunchConfiguration('turtlesim_ns1'), '/turtle1/pose']),
                        ('output/cmd_vel', [LaunchConfiguration('turtlesim_ns2'), '/turtle1/cmd_vel']),
                    ]
                )
            ])

In this example, we're first declaring three arguments, the namespace of each turtlesim and the node name of the mimic node:

.. tabs::

  .. group-tab:: XML launch file

    .. code-block:: xml

          <arg name="turtlesim_ns1" default="turtlesim1" description="namespace of one of the turtlesim to run"/>
          <arg name="turtlesim_ns2" default="turtlesim2" description="namespace of the other turtlesim to run"/>
          <arg name="mimic_name" default="mimic" description="name of the mimicking node"/>

  .. group-tab:: Python launch file

    .. code-block:: python

                DeclareLaunchArgument(
                  'turtlesim_ns1',
                  default_value='turtlesim1',
                  description='namespace of one of the turtlesim to run',
                ),
                DeclareLaunchArgument(
                  'turtlesim_ns2',
                  default_value='turtlesim2',
                  description='namespace of the other turtlesim to run',
                ),
                DeclareLaunchArgument(
                  'mimic_name',
                  default_value='mimic',
                  description='name of the mimicking node',
                ),

Each of the declared launch arguments have a name.
A default value and a description can optionally be provided.

It's possible to introspect what arguments a launch file declare:

.. code-block:: console

  ros2 launch -s turtlesim_mimic_launch.py

The provided arguments are being queried in the rest of the launch files using substitutions:

.. tabs::

  .. group-tab:: XML launch file

    .. code-block:: xml

        <launch>
          <arg name="turtlesim_ns1" default="turtlesim1" description="namespace of one of the turtlesim to run"/>
          <arg name="turtlesim_ns2" default="turtlesim2" description="namespace of the other turtlesim to run"/>
          <arg name="mimic_name" default="mimic" description="name of the mimicking node"/>

          <node pkg="turtlesim" exec="turtlesim_node" name="sim" namespace="$(var turtlesim_ns1)"/>
          <node pkg="turtlesim" exec="turtlesim_node" name="sim" namespace="$(var turtlesim_ns2)"/>
          <node pkg="turtlesim" exec="mimic" name="$(var mimic_name)">
            <remap from="/input/pose" to="/$(var turtlesim_ns1)/turtle1/pose"/>
            <remap from="/output/cmd_vel" to="/$(var turtlesim_ns2)/turtle1/cmd_vel"/>
          </node>
        </launch>

  .. group-tab:: Python launch file, Foxy and newer

    .. code-block:: python

                Node(
                    ...,
                    namespace=LaunchConfiguration('turtlesim_ns1'),
                    ...,
                ),
                Node(
                    ...,
                    namespace=LaunchConfiguration('turtlesim_ns2'),
                    ...,
                ),
                Node(
                    ...,
                    name=LaunchConfiguration('mimic_name'),
                    remappings=[
                        ('input/pose', [LaunchConfiguration('turtlesim_ns1'), '/turtle1/pose']),
                        ('output/cmd_vel', [LaunchConfiguration('turtlesim_ns2'), '/turtle1/cmd_vel']),
                    ]
                )
            ])

  .. group-tab:: Python launch file, Eloquent and older

    .. code-block:: python

                Node(
                    ...,
                    namespace=LaunchConfiguration('turtlesim_ns1'),
                    ...,
                ),
                Node(
                    ...,
                    namespace=LaunchConfiguration('turtlesim_ns2'),
                    ...,
                ),
                Node(
                    ...,
                    name=LaunchConfiguration('mimic_name'),
                    remappings=[
                        ('input/pose', [LaunchConfiguration('turtlesim_ns1'), '/turtle1/pose']),
                        ('output/cmd_vel', [LaunchConfiguration('turtlesim_ns2'), '/turtle1/cmd_vel']),
                    ]
                )
            ])

Now that the launch file is configurable, we can run it twice without collisions between the topics and node names!
In the following example, we will launch two turtlesims that draw a circle, and other two turtlesims that draw a square.

.. code-block:: console

  # In one terminal
  ros2 launch turtlesim_mimic_launch.py turtlesim_ns1:=drawing_circles_1 turtlesim_ns2:=drawing_circles_2 mimic_name:=mimic_circles
  ros2 topic pub -r 1 /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"

.. code-block:: console
  # In another terminal
  ros2 launch turtlesim_mimic_launch.py turtlesim_ns1:=drawing_squares_1 turtlesim_ns2:=drawing_squares_2 mimic_name:=mimic_squares
  ros2 run turtlesim draw_square --ros-args -r __ns:=/drawing_squares_1


6 Reusing an existing launch files and using conditions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

We will now reuse our last iteration of the ``turtlesim_mimic_launch.py/xml``, and create a new launch file that either draws a circle or an square based on a condition.

Create a new launch file named ``turtlesim_draw_launch.py/xml`` in the same directory that the one created before and copy the following

.. tabs::

  .. group-tab:: XML launch file

    .. code-block:: xml

        <launch>
            <arg name="mode" default="square"
              description="can be 'square' or 'circle', to make the turtle draw those figures"/>
            <arg name="turtlesim_ns1" default="turtlesim1" description="namespace of one of the turtlesim to run"/>
            <arg name="turtlesim_ns2" default="turtlesim2" description="namespace of the other turtlesim to run"/>
            <arg name="mimic_name" default="mimic" description="name of the mimicking node"/>

            <include file="$(dirname)/turtlesim_launch.xml"/>  <!-- arguments are passed automatically-->
            <node pkg="turtlesim" exec="draw_square" name="square_drawer" namespace="$(var turtlesim_ns1)"
                if="$(eval '\'$(var mode)\' == \'square\'')"/>
            <executable cmd="$(find-pkg-prefix ros2cli)/bin/ros2 topic pub -r 1 /$(var turtlesim_ns1)/turtle1/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}'"
                if="$(eval '\'$(var mode)\' == \'circle\'')"/>
        </launch>

  .. group-tab:: Python launch file, Foxy and newer

    .. code-block:: python

        Please write code here

  .. group-tab:: Python launch file, Eloquent and older

    .. code-block:: python

        Please write code here

COMPLETE THE EXAMPLE

TBD: Conditions look too ugly in XML (those scape characters!), we should improve them.
TBD2: Using forward slashes should work in XML launch files, even on Windows. I'm not sure if that is working.

Summary
-------

Launch files simplify running complex systems with many nodes and specific configuration details.
You can create launch files using XML or Python, and run them using the ``ros2 launch`` command.

Next steps
----------

In the next tutorial, :ref:`ROS2Bag`, you'll learn about another helpful tool, ``ros2bag``.
