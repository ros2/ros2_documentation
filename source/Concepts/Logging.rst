.. redirect-from::

    Logging

About logging and logger configuration
======================================

.. contents:: Table of Contents
   :depth: 2
   :local:


Overview
--------

The logging functionality currently supported is:


* Client libraries (``rclcpp`` and ``rclpy``) using a common logging library to provide:

  * Log calls with a variety of filters.
  * Hierarchy of loggers.
  * Loggers associated with nodes that automatically use the node's name and namespace.

* Console output.

  * File output and functionality akin to `rosout <https://wiki.ros.org/rosout>`__ for remote consumption of messages is forthcoming.

* Programmatic configuration of logger levels.

  * Launch-time configuration of the default logger level is supported; config files and external configuration at run-time is forthcoming.

Logger concepts
---------------

Log messages have a severity level associated with them: ``DEBUG``, ``INFO``, ``WARN``, ``ERROR`` or ``FATAL``, in ascending order.

A logger will only process log messages with severity at or higher than a specified level chosen for the logger.

Each node (in ``rclcpp`` and ``rclpy``) has a logger associated with it that automatically includes the node's name and namespace.
If the node's name is externally remapped to something other than what is defined in the source code, it will be reflected in the logger name.
Non-node loggers can also be created that use a specific name.

Logger names represent a hierarchy.
If the level of a logger named "abc.def" is unset, it will defer to the level of its parent named "abc", and if that level is also unset, the default logger level will be used.
When the level of logger "abc" is changed, all of its descendants (e.g. "abc.def", "abc.ghi.jkl") will have their level impacted unless their level has been explicitly set.

Logging usage
-------------

.. tabs::

  .. group-tab:: C++

    * See the `logging demo <../Tutorials/Logging-and-logger-configuration>` for example usage.
    * See the `rclcpp documentation <https://docs.ros2.org/latest/api/rclcpp/logging_8hpp.html>`__ for an extensive list of functionality.

  .. group-tab:: Python

    * See the `rclpy examples <https://github.com/ros2/examples/blob/master/rclpy/services/minimal_client/examples_rclpy_minimal_client/client.py>`__ for example usage of a node's logger.
    * See the `rclpy tests <https://github.com/ros2/rclpy/blob/master/rclpy/test/test_logging.py>`__ for example usage of keyword arguments (e.g. ``skip_first``, ``once``).
