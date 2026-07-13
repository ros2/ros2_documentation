.. redirect-from::

    Tutorials/Launch/CLI-Intro

.. _ROS2Launch:

Launching nodes - how-to
=========================

A launch file lets you start multiple nodes simultaneously with a single command, without opening a separate terminal for each.
This article shows you how to run an existing launch file using ``ros2 launch``.
It uses the Turtlesim package to demonstrate the process.

**Area: ROS-tutorials | Content-type: how-to | Experience: beginner**

.. contents:: Contents
   :depth: 2
   :local:

Summary
-------

Use ``ros2 launch <package> <launch_file>`` to start multiple nodes at once from a single launch file.
Running a single launch file starts all nodes and their configurations at once, rather than requiring a separate terminal for each.

Background
----------

As your system grows to include more nodes running simultaneously, opening separate terminals and re-entering configuration details for each becomes impractical.
Launch files let you start and configure any number of executables containing ROS nodes at once.

Prerequisites
-------------

:doc:`Install ROS <../../../Installation/>` before following these steps.

The commands here assume you followed the binary packages installation guide for your operating system (deb packages for Linux).
If you built from source, the path to your setup files will likely be different, and the ``sudo apt install ros-<distro>-<package>`` command will not be available.

If you are not already familiar with the shell on Linux, `this tutorial <https://www.linux.com/training-tutorials/bash-101-working-cli/>`__ will help.

.. note::
    Source ROS in every new terminal you open.
    See :doc:`Configuring environment <../Configuring-ROS2-Environment>`.

Steps
-----

1 Run a launch file
^^^^^^^^^^^^^^^^^^^

To run a launch file, use:

.. code-block:: console

   $ ros2 launch <package_name> <launch_file_name>

For example, to run the ``multisim.launch.py`` file from the ``turtlesim`` package, run:

.. code-block:: console

   $ ros2 launch turtlesim multisim.launch.py

This runs the following launch file:

.. literalinclude:: launch/multisim.launch.py
   :language: python

.. note::

   The launch file above is written in Python, but you can also use XML and YAML to create launch files.
   You can see a comparison of these different ROS launch formats in :doc:`../../../How-To-Guides/Launch-file-different-formats`.

Two Turtlesim windows open, one for each node defined in the launch file:

.. image:: images/turtlesim_multisim.png

2 (Optional) Control the launched nodes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Nodes launched with ``ros2 launch`` behave like any other ROS nodes.
To make the two turtles drive in opposite directions, open two additional terminals and run the following commands.

In the second terminal:

.. code-block:: console

   $ ros2 topic pub  /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

In the third terminal:

.. code-block:: console

   $ ros2 topic pub  /turtlesim2/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"

You should see something like the following:

.. image:: images/turtlesim_multisim_spin.png

Related content
---------------

* :doc:`ROS launch tutorials <../../Intermediate/Launch/Launch-Main>`
* :doc:`Launch file formats <../../../How-To-Guides/Launch-file-different-formats>`

FAQs
----

Can I write a launch file in a language other than Python?
   Yes. ROS supports launch files written in Python, XML, and YAML.
   For a comparison of these formats, see :doc:`../../../How-To-Guides/Launch-file-different-formats`.

How do I write my own launch file?
   For a full guide on writing launch files, see the :doc:`ROS launch tutorials <../../Intermediate/Launch/Launch-Main>`.

Can I pass arguments to a launch file?
   Yes. Use ``ros2 launch <package> <launch_file> <arg>:=<value>`` to pass arguments at the command line.
   Arguments must be declared within the launch file itself.
