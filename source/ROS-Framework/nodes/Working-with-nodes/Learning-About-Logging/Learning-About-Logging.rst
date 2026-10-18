.. _ROS2CliLogging:

Learning about CLI logging - tutorial
=====================================

Log messages report what a node is doing, and each message has a severity level.
This article teaches you to write a small battery monitor node that logs at different severities, then use the ROS CLI to show or hide those messages.
A hands-on exercise shows why ``DEBUG`` is hidden by default and how ``--log-level`` reveals it.

**Area: Nodes, Framework | Content-type: tutorial | Experience: beginner**

.. contents:: Contents
   :depth: 2
   :local:

Summary
-------

ROS nodes use log messages to report status and problems.
Severity levels, from least to most severe, are ``DEBUG``, ``INFO``, ``WARN``, ``ERROR``, and ``FATAL``.
By default, a node prints ``INFO`` and above.
Use ``--ros-args --log-level`` to change which severities are shown when you run a node.

For more information, see :doc:`About logging <../../About-Logging/About-Logging>`.

Prerequisites
-------------

* Make sure you understand the concepts covered in :doc:`Learning about nodes <../Understanding-ROS2-Nodes/Understanding-ROS2-Nodes>`.
* You need a workspace and a basic understanding of how to build a Python package.
  See :doc:`Creating a workspace <../../../client-libraries/Working-with-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace>` and :doc:`Creating a package <../../../client-libraries/Working-with-Client-Libraries/Creating-Your-First-ROS2-Package>`.
* You need to be familiar with ROS publishers and subscribers.
  See :doc:`Writing a simple publisher and subscriber (Python) <../../../client-libraries/Working-with-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber>`.

Steps
-----

.. note::
    Make sure to source ROS in every new terminal you open.

    For more information, see :doc:`Configuring environment <../../../../Get-Started/Configuring-ROS2-Environment>`.

1 Create a package
^^^^^^^^^^^^^^^^^^

Create a package to hold the battery monitor node that you are going to build.

#. Open a terminal, source your ROS installation, and navigate to the ``src`` directory of your workspace.

#. Create a Python package for the battery monitor:

   .. code-block:: console

     $ ros2 pkg create --build-type ament_python --license Apache-2.0 battery_logging --dependencies rclpy

   Your terminal returns a message confirming that the ``battery_logging`` package and its files were created.

2 Write the battery monitor node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Write a node that logs at each severity level as a simulated battery drains.
This node gives you controlled messages to inspect with the CLI in the later steps.

#. In the package module directory ``battery_logging/battery_logging``, create a file named ``battery_monitor.py``.

#. In the ``battery_monitor.py`` file, include the following code:

   .. code-block:: python

     import rclpy
     from rclpy.node import Node


     class BatteryMonitor(Node):

         def __init__(self):
             super().__init__('battery_monitor')
             self.level = 100
             self.warn_threshold = 30
             self.error_threshold = 10
             self.warned = False
             self.errored = False

             self.get_logger().info(
                 f'Battery monitor started, initial level {self.level}%')
             self.timer = self.create_timer(0.5, self.timer_callback)

         def timer_callback(self):
             self.get_logger().debug(f'Battery level: {self.level}%')

             if self.level <= 0:
                 self.get_logger().fatal(
                     'Battery depleted, shutting down battery monitor')
                 raise SystemExit

             if self.level <= self.error_threshold and not self.errored:
                 self.get_logger().error(
                     f'Battery critically low at {self.level}%, stop immediately')
                 self.errored = True
             elif self.level <= self.warn_threshold and not self.warned:
                 self.get_logger().warning(
                     'Battery low, please return to dock')
                 self.warned = True

             self.level -= 5


     def main(args=None):
         rclpy.init(args=args)
         node = BatteryMonitor()
         try:
             rclpy.spin(node)
         except (SystemExit, KeyboardInterrupt):
             pass
         finally:
             node.get_logger().info('Battery monitor shutting down')
             node.destroy_node()
             if rclpy.ok():
                 rclpy.shutdown()


     if __name__ == '__main__':
         main()

The node does the following:

* On startup, it logs ``INFO`` with the initial battery level.
* Every 0.5 second, it logs ``DEBUG`` with the current reading and drains the battery by 5 %.
* The first time the level reaches 30 % or below, it logs ``WARN``.
* The first time the level reaches 10 % or below, it logs ``ERROR``.
* When the level reaches 0 %, it logs ``FATAL``, then the node shuts down and logs ``INFO`` on the way out.

``WARN`` and ``ERROR`` are printed only the first time each threshold is crossed, so the terminal stays readable.
``DEBUG`` is printed on every timer callback.
Because it is verbose, it is hidden by default.

3 Add an entry point
^^^^^^^^^^^^^^^^^^^^

ROS needs an entry point to ensure that ``ros2 run`` can start your node by name.

#. Navigate one level up to the package root, where ``setup.py`` and ``package.xml`` were created.

#. Open ``setup.py``, then replace the console script entry:

   .. code-block:: python

     entry_points={
         'console_scripts': [
             'battery_monitor = battery_logging.battery_monitor:main',
         ],
     },

#. Save the file.

4 Build and source the workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Build the package to install the new entry point, then source the workspace overlay so that ``ros2 run`` can find it.

Return to the root of your workspace, then run the following:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      $ cd ~/ros2_ws
      $ colcon build --packages-select battery_logging
      $ source install/setup.bash

  .. group-tab:: macOS

    .. code-block:: console

      $ cd ~/ros2_ws
      $ colcon build --packages-select battery_logging
      $ source install/setup.bash

  .. group-tab:: Windows

    .. code-block:: console

      $ cd \dev\ros2_ws
      $ colcon build --packages-select battery_logging
      $ call install\setup.bat

5 Run the node with the default log level
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Run the node with no log-level override.
That shows which severities appear under the default filter.

.. code-block:: console

  $ ros2 run battery_logging battery_monitor

The node runs until the battery reaches 0 % and then shuts down.
You should see output similar to:

.. code-block:: console

  [INFO] [battery_monitor]: Battery monitor started, initial level 100%
  [WARN] [battery_monitor]: Battery low, please return to dock
  [ERROR] [battery_monitor]: Battery critically low at 10%, stop immediately
  [FATAL] [battery_monitor]: Battery depleted, shutting down battery monitor
  [INFO] [battery_monitor]: Battery monitor shutting down

Notice that there are no ``DEBUG`` lines for each battery reading.
By default, ROS shows ``INFO`` and higher (``WARN``, ``ERROR``, ``FATAL``).
``DEBUG`` is below that default, so the per-callback readings stay hidden.

6 Reveal DEBUG logs with the CLI
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

When you ran the node with no log-level override, it created a ``DEBUG`` message for each battery reading, but the default log level hid those messages.
To print them, set the log level to ``debug`` when you start the node.

#. Run the ``battery_monitor`` with the arguments that allow you to see its ``DEBUG`` messages without also printing messages from internal library loggers:

   .. code-block:: console

     $ ros2 run battery_logging battery_monitor --ros-args --log-level battery_monitor:=debug

   Now you should also see a stream of lines such as:

   .. code-block:: console

     [DEBUG] [battery_monitor]: Battery level: 100%
     [DEBUG] [battery_monitor]: Battery level: 95%
     [DEBUG] [battery_monitor]: Battery level: 90%

   The node was already creating those messages.
   The CLI log level only controls whether they are printed.

#. Now to set a global log level to ``debug``:

   .. code-block:: console

     $ ros2 run battery_logging battery_monitor --ros-args --log-level debug

   A global ``debug`` setting applies to every logger in the process, not only ``battery_monitor``.
   That includes internal ROS libraries such as ``rcl``, so you may see many extra ``DEBUG`` lines that are unrelated to the battery monitor.

#. If you want global ``DEBUG`` for your own code, set ``rcl`` back to ``info`` to hide the ``rcl`` lines.

   Other library loggers may still print.

   .. code-block:: console

     $ ros2 run battery_logging battery_monitor --ros-args --log-level debug --log-level rcl:=info

7 Choose an appropriate severity
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The CLI only controls which messages are shown.
The severity you pick in code is what operators see by default.

#. In ``battery_monitor.py``, change the low-battery call from ``warning`` to ``debug``:

   .. code-block:: python

     self.get_logger().debug(
         'Battery low, please return to dock')

#. Return to the root of your workspace, then rebuild the package and source the overlay.

#. Run the node with no log-level override:

   .. code-block:: console

     $ ros2 run battery_logging battery_monitor

   The battery still crosses 30 %, but the low-battery message no longer appears.
   A real problem logged at ``DEBUG`` stays hidden until someone sets the log level to ``debug``.
   If you put too many routine messages at ``INFO``, operators may struggle to spot the important ones among them.

#. In ``battery_monitor.py``, change that call back to ``warning``, then rebuild the package and source the overlay.

Use this node as a model for when each level fits:

* ``DEBUG``: frequent detail that helps while diagnosing a problem, but is too verbose for normal operation.
* ``INFO``: routine lifecycle or status that operators usually want to see.
* ``WARN``: something is wrong, but the system can continue if the operator responds.
* ``ERROR``: a serious fault that needs immediate attention.
* ``FATAL``: the node cannot continue.

Logging ``FATAL`` does not stop the process by itself.
This example raises ``SystemExit`` after the fatal message so that the node shuts down cleanly.

8 Inspect the same logs in ``rqt_console``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Log messages are not limited to the terminal.
They are also published on the ``/rosout`` topic, so you can view and filter them in a GUI with :doc:`Using rqt_console <../Using-Rqt-Console/Using-Rqt-Console>`.

#. In a new terminal, start ``rqt_console``:

   .. code-block:: console

     $ ros2 run rqt_console rqt_console

#. In another terminal, run the battery monitor with ``--log-level battery_monitor:=debug``.

#. In ``rqt_console``, exclude ``Debug`` and confirm that the remaining messages match the terminal output from the default log-level run.

Next steps
----------

* Read more about severity, logger names, and configuration in :doc:`About logging <../../About-Logging/About-Logging>`.
* Read more about logger services, formatting, and per-logger CLI options in :doc:`Logging and logger configuration <../../../../Developer-Tools/Introspection-and-analysis/Logging-and-logger-configuration>`.

Related content
---------------

More articles:

* :doc:`About logging <../../About-Logging/About-Logging>`
* :doc:`Using rqt_console <../Using-Rqt-Console/Using-Rqt-Console>`
* :doc:`Logging and logger configuration <../../../../Developer-Tools/Introspection-and-analysis/Logging-and-logger-configuration>`
* :doc:`Learning about nodes <../Understanding-ROS2-Nodes/Understanding-ROS2-Nodes>`
* :doc:`Writing a simple publisher and subscriber (Python) <../../../client-libraries/Working-with-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber>`

FAQs
----

Why do I not see DEBUG messages when I first run the battery monitor?
   By default, ROS shows log severities of ``INFO`` and above.
   ``DEBUG`` is below that default, so those messages are generated but not printed.
   Run the node with ``--ros-args --log-level battery_monitor:=debug`` to show them.

What is the difference between WARN, ERROR, and FATAL?
   ``WARN`` means there is a problem the operator can still recover from, such as a low battery.
   ``ERROR`` means a serious fault that needs immediate action.
   ``FATAL`` means the node cannot continue.
   In this tutorial, the node logs ``FATAL`` at 0 % and then shuts itself down.

Does logging FATAL automatically stop my node?
   No.
   ``FATAL`` is only a severity level for the message.
   Your code must still stop timers, destroy the node, or shut down ``rclpy`` if the process should exit.

Why do I see many rcl DEBUG lines when I use ``--log-level debug``?
   A global ``debug`` level applies to more loggers than your node alone, including internal ``rcl`` loggers.
   Narrow the setting with ``--log-level battery_monitor:=debug``, or hide ``rcl`` ``DEBUG`` messages with ``--log-level debug --log-level rcl:=info``.

How do I change the log level when I run a node?
   Pass ROS arguments after ``--ros-args``.
   For example: ``ros2 run battery_logging battery_monitor --ros-args --log-level battery_monitor:=debug``.
