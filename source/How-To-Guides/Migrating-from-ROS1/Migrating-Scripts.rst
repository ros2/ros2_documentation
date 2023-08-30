Migrating Scripts
=================

ROS CLI
-------

In ROS 1 there were individual commands for performing various actions, like ``rosrun``, ``rosparam``, etc.

In ROS 2, there is a single top-level commands called ``ros2``, and all of the actions are sub-commands of that, like ``ros2 run``, ``ros2 param``, etc.

ROS CLI arguments
-----------------

In ROS 1, arguments to nodes were provided directly on the command-line.

ROS 2 arguments should be scoped with ``--ros-args`` and a trailing ``--`` (the trailing double dash may be elided if no arguments follow it).

Remapping names is similar to ROS 1, taking on the form ``from:=to``, except that it must be preceded by a ``--remap`` (or ``-r``) flag.
For example:

.. code-block:: bash

   ros2 run some_package some_ros_executable --ros-args -r foo:=bar

We use a similar syntax for parameters, using the ``--param`` (or ``-p``) flag:

.. code-block:: bash

   ros2 run some_package some_ros_executable --ros-args -p my_param:=value

Note, this is different than using a leading underscore in ROS 1.

To change a node name use ``__node`` (the ROS 1 equivalent is ``__name``):

.. code-block:: bash

   ros2 run some_package some_ros_executable --ros-args -r __node:=new_node_name

Note the use of the ``-r`` flag.
The same remap flag is needed for changing the namespace ``__ns``:

.. code-block:: bash

   ros2 run some_package some_ros_executable --ros-args -r __ns:=/new/namespace

There is no equivalent in ROS 2 for the following ROS 1 keys:

- ``__log`` (but ``--log-config-file`` can be used to provide a logger configuration file)
- ``__ip``
- ``__hostname``
- ``__master``

For more information, see the `design document <https://design.ros2.org/articles/ros_command_line_arguments.html>`_.

Quick reference
~~~~~~~~~~~~~~~

+------------+-------------+----------------+
| Feature    | ROS 1       | ROS 2          |
+============+=============+================+
| remapping  | foo:=bar    | -r foo:=bar    |
+------------+-------------+----------------+
| parameters | _foo:=bar   | -p foo:=bar    |
+------------+-------------+----------------+
| node name  | __name:=foo | -r __node:=foo |
+------------+-------------+----------------+
| namespace  | __ns:=foo   | -r __ns:=foo   |
+------------+-------------+----------------+
