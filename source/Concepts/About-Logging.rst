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

Log files
---------

Log output files written to a directory that defaults to ``~/.ros/log`` but can be changed through enviroment variables.  The filenames follow this pattern:

  ``<exe>_<pid>_<milliseconds-since-epoch>.log``

These are created per operating system process.
The external lib logging using ``spdlog`` is what writes these files in the current implementation.

Screen log output
-----------------

The screen output is written from the handler within ``rcutils``.
The screen output to ``stdout`` and ``stderr`` can originate from many different processes when ROS nodes are started through the launch system.
For this reason, the launch system itself consumes ``stderr`` and ``stdout`` from each process and writes to its own ``stderr`` and ``stdout`` streams to interleave the log messages from the various processes.

Logging from a ROS Node
-----------------------

When logging from within a C++ ROS Node one should use the interface defined in the ``rclcpp`` library.

Initialization of the ROS Logging system
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

When a ``rclcpp::Node`` is created the global Context object is created and initialized.
As part of this process, the logging system is initialized.
One of the steps of initializing the logging system is setting the global output handler, which is a function that is called as the final step to write out a log.
The code within this function defines where the log string and data go.
``rclcpp`` defines an output handler which locks a global logging mutex and then calls ``rcl_logging_multiple_output_handler``, which is defined in ``rcl/logging.hpp``.

One implication of this is that in any ROS process based on ``rclcpp`` it is single-threaded during logging.
To combat the performance implications of this you can use throttling.
This is necessary because the logging system writes to file streams (stderr, stdout, normal files) which cannot be done from multiple threads in a sane way.
A possible technique to log from a thread you need to be lock-free would be to use internal queues to send data to another thread that then logs your data.

Compiling out logging
^^^^^^^^^^^^^^^^^^^^^

To compile wihtout logging set ``RCLCPP_LOGGING_ENABLED=0`` and ``RCLCPP_LOG_MIN_SEVERITY=RCLCPP_LOG_MIN_SEVERITY_NONE``.

Will this only work with a source build of ros2 or can this be done per project that depends on ros2?

rclcpp ``Logger`` class
^^^^^^^^^^^^^^^^^^^^^^^

Stores a name string.
If this is created from the Node the string is the name of the node, otherwise, it is whatever string you pass into the function ``get_logger``.
Logger object is a required first argument for the ``RCLCPP_`` macros.

``RCLCPP`` macros
^^^^^^^^^^^^^^^^^

Defined by generated header in rclcpp (rclcpp/resource/logging.hpp.em).
This header is generated as much of the content is repeated over and over with a small difference for the combinations of level and features.
The first argument is required to be of the type ``rclcpp::Logger``.
The ``rclcpp::Logger`` is used to get the name of the logger to pass to the ``RCUTILS_`` macro it uses.
This is the interface that users of rclcpp are expected to use and implements several features on top of the ``RCUTILS_LOG_<LEVEL>[_<FEATURE(s)>]_NAMED`` logging macros.
The name argument for the ``RCUTILS`` macro that is used comes from the ``rclcpp::Logger``.

The structure of the macro is ``RCLCPP_<LEVEL>[_<FEATURE(s)>](logger, ...)``.
The possible severity levels are ``DEBUG``, ``INFO``, ``WARN``, ``ERROR``, or ``FATAL``.
The possible feature combinations are ``ONCE``, ``EXPRESSION``, ``FUNCTION``, ``SKIPFIRST``, ``THROTTLE``, ``SKIPFIRST_THROTTLE``, ``STREAM``, ``STREAM_ONCE``, ``STREAM_EXPRESSION``, ``STREAM_FUNCTION``, ``STREAM_SKIPFIRST``, ``STREAM_THROTTLE``, ``STREAM_SKIPFIRST_THROTTLE``.

.. list-table:: RCLCPP logging macro FEATURES
   :widths: 10 90
   :header-rows: 1

   * - FEATURE
     - Description
   * - ONCE
     - All subsequent log calls except the first one are being ignored.
   * - EXPRESSION
     - Log calls are being ignored when the expression evaluates to false.
   * - FUNCTION
     - Log calls are being ignored when the function returns false.
   * - SKIPFIRST
     - The first log call is being ignored but all subsequent calls are being processed.
   * - THROTTLE
     - Log calls are being ignored if the last logged message is not longer ago than the specified duration.
   * - STREAM
     - Uses a ``std::stringstream`` to construct the logged string.

Logging from within ROS 2
-------------------------

Configuration of the logging system and handling of logging output is defined in ``rcl/logging.h``.  Macros used by rclcpp for logging are defined in the C utility package ``rcutils``.

rcl/logging.h
^^^^^^^^^^^^^

Configuration of the logging system can come from process arguments.
The available arguments are defined in ``rcl/arguments.h``.
See `logging demo <../Tutorials/Logging-and-logger-configuration.html#logger-level-configuration-command-line>` for example usage.

.. list-table:: Global arguments for logging
   :widths: 10 20 70
   :header-rows: 1

   * - Argument
     - Default
     - Description
   * - --log-level
     - Unset
     - The ROS flag that precedes the ROS logging level to set.
   * - --log-config-file
     - Unset
     - The ROS flag that precedes the name of a configuration file to configure logging in external logger.
   * - stdout-logs
     - enabled
     - The suffix of the ROS flag to enable or disable stdout logging.  must be preceded with --enable- or --disable-.
   * - rosout-logs
     - enabled
     - The suffix of the ROS flag to enable or disable rosout logging.  must be preceded with --enable- or --disable-.
   * - external-lib-logs
     - enabled
     - The suffix of the ROS flag to enable or disable external library logging.  must be preceded with --enable- or --disable-.

``rcl_logging_multiple_output_handler`` is a function that is called by the logging system that then, in turn, calls logging handlers for each of the three types of outputs (depending on if they are enabled): stdout, rosout, and external-lib.

stdout logging
^^^^^^^^^^^^^^

This is defined by the function ``rcutils_logging_console_output_handler`` and handles sending logs to stdout and stderr.

rosout logging
^^^^^^^^^^^^^^

Publishes a ``rcl_interfaces::Log`` message to the topic /node-name/rosout.

external-lib logging
^^^^^^^^^^^^^^^^^^^^

This feature uses an external logging library to consume the output of the logging system using the package ``rcl_logging``.
There is currently only one external library interface defined and that is ``rcl_logging_spdlog``.
The external-lib implementation is set during the CMake configure step of the package ``rcl``.
It can be overridden with the variable ``RCL_LOGGING_IMPLEMENTATION``.
The ``rcl_logging`` README says there is a ``rcl_logging_log4cxx`` implementation, however that does not appear to exist yet.

rcl_logging_spdlog configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The current implementation of ``rcl_logging_spdlog`` cannot be configured with a config file.

spdlog logging
^^^^^^^^^^^^^^

This is what writes the log files.

RCUTILS internals
-----------------

``rcutils`` contains macros and functions that define some of the non-ros low-level logging behavior in C.

rcutils/logging.h
^^^^^^^^^^^^^^^^^

This header defines various internal functions used by the logging system that can be configured via environment variables.

RCUTILS_CONSOLE_OUTPUT_FORMAT enviroment variable
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The ``RCUTILS_CONSOLE_OUTPUT_FORMAT`` environment variable can be used to set the output format of messages logged to the console.

.. list-table:: RCUTILS_CONSOLE_OUTPUT_FORMAT tokens
   :widths: 10 90
   :header-rows: 1

   * - Token
     - Description
   * - file_name
     - the full file name of the caller including the path
   * - function_name
     - the function name of the caller
   * - line_number
     - the line number of the caller
   * - message
     - the message string after it has been formatted
   * - name
     - the full logger name
   * - severity
     - the name of the severity level, e.g. `INFO`
   * - time
     - the timestamp of log message in floating point seconds
   * - time_as_nanoseconds
     - the timestamp of log message in integer nanoseconds

The format string can use these tokens by referencing them in curly brackets,
e.g. ``"[{severity}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"``.
Any number of tokens can be used.
The limit of the format string is 2048 characters.

RCUTILS_COLORIZED_OUTPUT environment variable
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The ``RCUTILS_COLORIZED_OUTPUT`` environment variable allows configuring if colors are used or not.

.. list-table:: RCUTILS_COLORIZED_OUTPUT values
   :widths: 10 90
   :header-rows: 1

   * - Value
     - Description
   * - 1
     - Force using colours.
   * - 0
     - Don't use colours.

If it is unset, colors are used depending on if the target stream is a terminal or not.
See `isatty` documentation.

rcutils/logging_macros.h
^^^^^^^^^^^^^^^^^^^^^^^^

This is a generated header file as much of the content is repeated over and over for the various combinations.
The structure of the macros is ``RCUTILS_LOG_<LEVEL>[_FEATURE(s)][_NAMED]``.
The named variants are the ones used by rclcpp and the name comes from the Logger object.
The levels are the same as in rclcpp.
The possible feature combinations are ``ONCE``, ``EXPRESSION``, ``FUNCTION``, ``SKIPFIRST``, ``THROTTLE``, ``SKIPFIRST_THROTTLE``.
These features are implemented with a set of even more general macros.
Normal users of ros2 should not need to understand how this implementation works or use this interface.

Logging from a Python ROS Node
------------------------------

The Python interfaces are built on top of ``rclcpp`` and therefore share the same functionality.

Logging from Python launch file
-------------------------------

TODO

Logging usage
-------------

.. tabs::

  .. group-tab:: C++

    * See the `logging demo <../Tutorials/Logging-and-logger-configuration>` for example usage.
    * See the `rclcpp documentation <https://docs.ros2.org/latest/api/rclcpp/logging_8hpp.html>`__ for an extensive list of functionality.

  .. group-tab:: Python

    * See the `rclpy examples <https://github.com/ros2/examples/blob/master/rclpy/services/minimal_client/examples_rclpy_minimal_client/client.py>`__ for example usage of a node's logger.
    * See the `rclpy tests <https://github.com/ros2/rclpy/blob/master/rclpy/test/test_logging.py>`__ for example usage of keyword arguments (e.g. ``skip_first``, ``once``).
