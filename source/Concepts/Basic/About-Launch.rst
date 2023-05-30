Launch
======

.. contents:: Table of Contents
   :local:

A ROS 2 system typically consists of many nodes running across many different processes (and even different machines).
While it is possible to run each of these nodes separately, it gets cumbersome quite quickly.

The launch system in ROS 2 is meant to automate the running of many nodes with a single command.
It helps the user describe the configuration of their system and then executes it as described.
The configuration of the system includes what programs to run, where to run them, what arguments to pass them, and ROS-specific conventions which make it easy to reuse components throughout the system by giving them each a different configuration.
It is also responsible for monitoring the state of the processes launched, and reporting and/or reacting to changes in the state of those processes.

All of the above is specified in a launch file, which can be written in Python, XML, or YAML.
This launch file can then be run using the ``ros2 launch`` command, and all of the nodes specified will be run.

The `design document <https://design.ros2.org/articles/roslaunch.html>`__ details the goal of the design of ROS 2's launch system (not all functionality is currently available).
