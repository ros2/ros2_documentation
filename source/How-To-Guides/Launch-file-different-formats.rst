.. redirect-from::

  Guides/Launch-file-different-formats

Using XML, YAML, and Python for ROS 2 Launch Files
==================================================

.. contents:: Table of Contents
   :depth: 1
   :local:

ROS 2 launch files can be written in XML, YAML, and Python.
This guide shows how to use these different formats to accomplish the same task, as well as has some discussion on when to use each format.

Launch file examples
--------------------

Below is a launch file implemented in XML, YAML, and Python.
Each launch file performs the following actions:

* Setup command line arguments with defaults
* Include another launch file
* Include another launch file in another namespace
* Start a node and setting its namespace
* Start a node, setting its namespace, and setting parameters in that node (using the args)
* Create a node to remap messages from one topic to another

.. tabs::

   .. group-tab:: XML

      .. literalinclude:: launch/different_formats_launch.xml
        :language: xml

   .. group-tab:: YAML

      .. literalinclude:: launch/different_formats_launch.yaml
        :language: yaml

   .. group-tab:: Python

      .. literalinclude:: launch/different_formats_launch.py
        :language: python


Using the Launch files from the command line
--------------------------------------------

Launching
^^^^^^^^^

Any of the launch files above can be run with ``ros2 launch``.
To try them locally, you can either create a new package and use

.. code-block:: console

  $ ros2 launch <package_name> <launch_file_name>

or run the file directly by specifying the path to the launch file

.. code-block:: console

  $ ros2 launch <path_to_launch_file>

Setting arguments
^^^^^^^^^^^^^^^^^

To set the arguments that are passed to the launch file, you should use ``key:=value`` syntax.
For example, you can set the value of ``background_r`` in the following way:

.. code-block:: console

  $ ros2 launch <package_name> <launch_file_name> background_r:=255

or

.. code-block:: console

  $ ros2 launch <path_to_launch_file> background_r:=255

Controlling the turtles
^^^^^^^^^^^^^^^^^^^^^^^

To test that the remapping is working, you can control the turtles by running the following command in another terminal:

.. code-block:: console

  $ ros2 run turtlesim turtle_teleop_key --ros-args --remap __ns:=/turtlesim1


.. _launch-file-different-formats-which:

XML, YAML, or Python: Which should I use?
-----------------------------------------

.. note::

  Launch files in ROS 1 were written in XML, so XML may be the most familiar to people coming from ROS 1.
  To see what's changed, you can visit :doc:`Migrating-from-ROS1/Migrating-Launch-Files`.

For most applications the choice of which ROS 2 launch format comes down to developer preference.
However, if your launch file requires flexibility that you cannot achieve with XML or YAML, you can use Python to write your launch file.
Using Python for ROS 2 launch is more flexible because of following two reasons:

* Python is a scripting language, and thus you can leverage the language and its libraries in your launch files.
* `ros2/launch <https://github.com/ros2/launch>`_ (general launch features) and `ros2/launch_ros <https://github.com/ros2/launch_ros>`_ (ROS 2 specific launch features) are written in Python and thus you have lower level access to launch features that may not be exposed by XML and YAML.

That being said, a launch file written in Python may be more complex and verbose than one in XML or YAML.
