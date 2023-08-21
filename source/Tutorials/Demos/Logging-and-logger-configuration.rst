.. redirect-from::

    Logging-and-logger-configuration
    Tutorials/Logging-and-logger-configuration

Logging
=======

.. contents:: Table of Contents
   :depth: 2
   :local:

See `the logging page <../../Concepts/Intermediate/About-Logging>` for details on available functionality.

Using log statements in code
----------------------------

Basic logging
^^^^^^^^^^^^^

The following code will output a log message from a ROS 2 node at ``DEBUG`` severity:

.. tabs::

    .. group-tab:: C++

        .. code-block:: C++

            // printf style
            RCLCPP_DEBUG(node->get_logger(), "My log message %d", 4);

            // C++ stream style
            RCLCPP_DEBUG_STREAM(node->get_logger(), "My log message " << 4);

    .. group-tab:: Python

        .. code-block:: python

            node.get_logger().debug('My log message %d' % (4))

Note that in both cases, no trailing newline is added, as the logging infrastructure will automatically add one.

Logging only the first time
^^^^^^^^^^^^^^^^^^^^^^^^^^^

The following code will output a log message from a ROS 2 node at ``INFO`` severity, but only the first time it is hit:

.. tabs::

    .. group-tab:: C++

        .. code-block:: C++

            // printf style
            RCLCPP_INFO_ONCE(node->get_logger(), "My log message %d", 4);

            // C++ stream style
            RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "My log message " << 4);

    .. group-tab:: Python

        .. code-block:: python

            num = 4
            node.get_logger().info(f'My log message {num}', once=True)

Logging all but the first time
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The following code will output a log message from a ROS 2 node at ``WARN`` severity, but not the very first time it is hit:

.. tabs::

    .. group-tab:: C++

        .. code-block:: C++

            // printf style
            RCLCPP_WARN_SKIPFIRST(node->get_logger(), "My log message %d", 4);

            // C++ stream style
            RCLCPP_WARN_STREAM_SKIPFIRST(node->get_logger(), "My log message " << 4);

    .. group-tab:: Python

        .. code-block:: python

            num = 4
            node.get_logger().warning('My log message {0}'.format(num), skip_first=True)

Logging throttled
^^^^^^^^^^^^^^^^^

The following code will output a log message from a ROS 2 node at ``ERROR`` severity, but no more than once per second.

The interval parameter specifying milliseconds between messages should have an integer data type so it can be converted to a ``rcutils_duration_value_t`` (an ``int64_t``):

.. tabs::

    .. group-tab:: C++

        .. code-block:: C++

            // printf style
            RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "My log message %d", 4);

            // C++ stream style
            RCLCPP_ERROR_STREAM_THROTTLE(node->get_logger(), *node->get_lock(), 1000, "My log message " << 4);

            // For now, use the nanoseconds() method to use an existing rclcpp::Duration value, see https://github.com/ros2/rclcpp/issues/1929
            RCLCPP_ERROR_STREAM_THROTTLE(node->get_logger(), *node->get_clock(), msg_interval.nanoseconds()/1000000, "My log message " << 4);

    .. group-tab:: Python

        .. code-block:: python

            num = 4
            node.get_logger().error(f'My log message {num}', throttle_duration_sec=1)

Logging throttled all but the first time
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The following code will output a log message from a ROS 2 node at ``DEBUG`` severity, no more than once per second, skipping the very first time it is hit:

.. tabs::

    .. group-tab:: C++

        .. code-block:: C++

            // printf style
            RCLCPP_DEBUG_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "My log message %d", 4);

            RCLCPP_DEBUG_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "My log message " << 4);

    .. group-tab:: Python

        .. code-block:: python

            num = 4
            node.get_logger().debug(f'My log message {num}', skip_first=True, throttle_duration_sec=1.0)

Logging demo
------------

In this `demo <https://github.com/ros2/demos/tree/{REPOS_FILE_BRANCH}/logging_demo>`_, different types of log calls are shown and the severity level of different loggers is configured locally and externally.

Start the demo with:

.. code-block:: bash

   ros2 run logging_demo logging_demo_main

Over time you will see output from various log calls with different properties.
To start with you will only see output from log calls with severity ``INFO`` and above (``WARN``, ``ERROR``, ``FATAL``).
Note that the first message will only be logged once, though the line is reached on each iteration, as that is a property of the log call used for that message.

Logging directory configuration
-------------------------------

The logging directory can be configured through two environment variables: ``ROS_LOG_DIR`` and ``ROS_HOME``.
The logic is as follows:

* Use ``$ROS_LOG_DIR`` if ``ROS_LOG_DIR`` is set and not empty.
* Otherwise, use ``$ROS_HOME/log``, using ``~/.ros`` for ``ROS_HOME`` if not set or if empty.

For example, to set the logging directory to ``~/my_logs``:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

      export ROS_LOG_DIR=~/my_logs
      ros2 run logging_demo logging_demo_main

  .. group-tab:: macOS

    .. code-block:: bash

      export ROS_LOG_DIR=~/my_logs
      ros2 run logging_demo logging_demo_main

  .. group-tab:: Windows

    .. code-block:: bash

      set "ROS_LOG_DIR=~/my_logs"
      ros2 run logging_demo logging_demo_main

You will then find the logs under ``~/my_logs/``.

Alternatively, you can set ``ROS_HOME`` and the logging directory will be relative to it (``$ROS_HOME/log``).
``ROS_HOME`` is intended to be used by anything that needs a base directory.
Note that ``ROS_LOG_DIR`` has to be either unset or empty.
For example, with ``ROS_HOME`` set to ``~/my_ros_home``:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

      export ROS_HOME=~/my_ros_home
      ros2 run logging_demo logging_demo_main

  .. group-tab:: macOS

    .. code-block:: bash

      export ROS_HOME=~/my_ros_home
      ros2 run logging_demo logging_demo_main

  .. group-tab:: Windows

    .. code-block:: bash

      set "ROS_HOME=~/my_ros_home"
      ros2 run logging_demo logging_demo_main

You will then find the logs under ``~/my_ros_home/log/``.

Logger level configuration: programmatically
--------------------------------------------

After 10 iterations the level of the logger will be set to ``DEBUG``, which will cause additional messages to be logged.

Some of these debug messages cause additional functions/expressions to be evaluated, which were previously skipped as ``DEBUG`` log calls were not enabled.
See `the source code <https://github.com/ros2/demos/blob/{REPOS_FILE_BRANCH}/logging_demo/src/logger_usage_component.cpp>`__ of the demo for further explanation of the calls used, and see the rclcpp logging documentation for a full list of supported logging calls.

Logger level configuration: externally
--------------------------------------

In the future there will be a generalized approach to external configuration of loggers at runtime (similar to how `rqt_logger_level <https://wiki.ros.org/rqt_logger_level>`__ in ROS 1 allows logger configuration via remote procedural calls).
**This concept is not yet officially supported in ROS 2.**
In the meantime, this demo provides an **example** service that can be called externally to request configuration of logger levels for known names of loggers in the process.

The demo previously started is already running this example service.
To set the level of the demo's logger back to ``INFO``\ , call the service with:

.. code-block:: bash

   ros2 service call /config_logger logging_demo/srv/ConfigLogger "{logger_name: 'logger_usage_demo', level: INFO}"

This service call will work on any logger that is running in the process provided that you know its name.
This includes the loggers in the ROS 2 core, such as ``rcl`` (the common client library package).
To enable debug logging for ``rcl``, call:

.. code-block:: bash

   ros2 service call /config_logger logging_demo/srv/ConfigLogger "{logger_name: 'rcl', level: DEBUG}"

You should see debug output from ``rcl`` start to show.

Using the logger config component
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The server that responds to the logger configuration requests has been developed as a component so that it may be added to an existing composition-based system.
For example, if you are using `a container to run your nodes <../Intermediate/Composition>`, to be able to configure your loggers you only need to request that it additionally load the ``logging_demo::LoggerConfig`` component into the container.

As an example, if you want to debug the ``composition::Talker`` demo, you can start the talker as normal with:

Shell 1:

.. code-block:: bash

   ros2 run rclcpp_components component_container

Shell 2:

.. code-block:: bash

   ros2 component load /ComponentManager composition composition::Talker

And then when you want to enable debug logging, load the ``LoggerConfig`` component with:

Shell 2

.. code-block:: bash

   ros2 component load /ComponentManager logging_demo logging_demo::LoggerConfig

And finally, configure all unset loggers to the debug severity by addressing the empty-named logger.
Note that loggers that have been specifically configured to use a particular severity will not be affected by this call.

Shell 2:

.. code-block:: bash

   ros2 service call /config_logger logging_demo/srv/ConfigLogger "{logger_name: '', level: DEBUG}"

You should see debug output from any previously unset loggers in the process start to appear, including from the ROS 2 core.

Logger level configuration: command line
----------------------------------------

As of the Bouncy ROS 2 release, the severity level for loggers that have not had their severity set explicitly can be configured from the command line.
Restart the demo including the following command line argument:


.. code-block:: bash

   ros2 run logging_demo logging_demo_main --ros-args --log-level debug

This configures the default severity for any unset logger to the debug severity level.
You should see debug output from loggers from the demo itself and from the ROS 2 core.

As of the Galactic ROS 2 release, the severity level for individual loggers can be configured from the command-line.
Restart the demo including the following command line arguments:

.. tabs::

  .. group-tab:: Galactic and newer

    .. code-block:: bash

       ros2 run logging_demo logging_demo_main --ros-args --log-level logger_usage_demo:=debug


Console output formatting
^^^^^^^^^^^^^^^^^^^^^^^^^

If you would like more or less verbose formatting, you can use RCUTILS_CONSOLE_OUTPUT_FORMAT environment variable.
For example, to additionally get the timestamp and location of the log calls, stop the demo and restart it with the environment variable set:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

      export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"
      ros2 run logging_demo logging_demo_main

  .. group-tab:: macOS

    .. code-block:: bash

      export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"
      ros2 run logging_demo logging_demo_main

  .. group-tab:: Windows

    .. code-block:: bash

      set "RCUTILS_CONSOLE_OUTPUT_FORMAT=[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"
      ros2 run logging_demo logging_demo_main

You should see the timestamp in seconds and the function name, filename and line number additionally printed with each message.
*The ``time`` option is only supported as of the ROS 2 Bouncy release.*

Console output colorizing
^^^^^^^^^^^^^^^^^^^^^^^^^

By default, the output is colorized when it's targeting a terminal.
If you would like to force enabling or disabling it, you can use the ``RCUTILS_COLORIZED_OUTPUT`` environment variable.
For example:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

      export RCUTILS_COLORIZED_OUTPUT=0  # 1 for forcing it
      ros2 run logging_demo logging_demo_main

  .. group-tab:: macOS

    .. code-block:: bash

      export RCUTILS_COLORIZED_OUTPUT=0  # 1 for forcing it
      ros2 run logging_demo logging_demo_main

  .. group-tab:: Windows

    .. code-block:: bash

      set "RCUTILS_COLORIZED_OUTPUT=0" :: 1 for forcing it
      ros2 run logging_demo logging_demo_main

You should see that debug, warn, error and fatal logs aren't colorized now.

.. note::

   In Linux and MacOS forcing colorized output means that if you redirect the output to a file, the ansi escape color codes will appear on it.
   In windows the colorization method relies on console APIs.
   If it is forced you will get a new warning saying that colorization failed.
   The default behavior already checks if the output is a console or not, so forcing colorization is not recommended.

Default stream for console output
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In Foxy and later, the output from all debug levels goes to stderr by default.  It is possible to force all output to go to stdout by setting the ``RCUTILS_LOGGING_USE_STDOUT`` environment variable to ``1``.
For example:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

      export RCUTILS_LOGGING_USE_STDOUT=1

  .. group-tab:: macOS

    .. code-block:: bash

      export RCUTILS_LOGGING_USE_STDOUT=1

  .. group-tab:: Windows

    .. code-block:: bash

      set "RCUTILS_LOGGING_USE_STDOUT=1"


Line buffered console output
^^^^^^^^^^^^^^^^^^^^^^^^^^^^


By default, all logging output is unbuffered.
You can force it to be buffered by setting the ``RCUTILS_LOGGING_BUFFERED_STREAM`` environment variable to 1.
For example:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

      export RCUTILS_LOGGING_BUFFERED_STREAM=1

  .. group-tab:: macOS

    .. code-block:: bash

      export RCUTILS_LOGGING_BUFFERED_STREAM=1

  .. group-tab:: Windows

    .. code-block:: bash

      set "RCUTILS_LOGGING_BUFFERED_STREAM=1"

Then run:

.. code-block:: bash

    ros2 run logging_demo logging_demo_main
