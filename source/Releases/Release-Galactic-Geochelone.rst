.. _upcoming-release:

.. _galactic-release:

.. move this directive when next release page is created

ROS 2 Galactic Geochelone (codename 'galactic'; May, 2021)
==========================================================

.. contents:: Table of Contents
   :depth: 2
   :local:

*Galactic Geochelone* is the seventh release of ROS 2.

Supported Platforms
-------------------

TBD

Installation
------------

TBD

New features in this ROS 2 release
----------------------------------

Ability to specify per-logger log levels
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

It is now possible to specify different logging levels for different loggers on the command line:

.. code-block:: bash

   ros2 run demo_nodes_cpp talker --ros-args --log-level WARN --log-level talker:=DEBUG

The above command sets a global log level of WARN, but sets the log level of the talker node messages to DEBUG.
The ``--log-level`` command-line option can be passed an arbitrary number of times to set different log levels for each logger.

Ability to configure logging directory through environment variables
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

It is now possible to configure the logging directory through two environment variables: ``ROS_LOG_DIR`` and ``ROS_HOME``.
The logic is as follows:

* Use ``$ROS_LOG_DIR`` if ``ROS_LOG_DIR`` is set and not empty.
* Otherwise, use ``$ROS_HOME/log``, using ``~/.ros`` for ``ROS_HOME`` if not set or if empty.

Thus the default value stays the same: ``~/.ros/log``.

Related PRs: `ros2/rcl_logging#53 <https://github.com/ros2/rcl_logging/pull/53>`_ and `ros2/launch#460 <https://github.com/ros2/launch/pull/460>`_.

Changes since the Foxy release
------------------------------

nav2
^^^^

Changes include, but are not limited to, a number of stability improvements, new plugins, interface changes, costmap filters.
See `Migration Guides <https://navigation.ros.org/migration/Foxy.html>`_ for full list

tf2_ros Python split out of tf2_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The Python code that used to live in tf2_ros has been moved into its own package named tf2_ros_py.
Any existing Python code that depends on tf2_ros will continue to work, but the package.xml of those packages should be amended to ``exec_depend`` on tf2_ros_py.

rclcpp
^^^^^^

Change in spin_until_future_complete template parameters
""""""""""""""""""""""""""""""""""""""""""""""""""""""""

The first template parameter of ``Executor::spin_until_future_complete`` was the future result type ``ResultT``, and the method only accepted a ``std::shared_future<ResultT>``.
In order to accept other types of futures (e.g.: ``std::future``), that parameter was changed to the future type itself.

In places where a ``spin_until_future_complete`` call was relying on template argument deduction, no change is needed.
If not, this is an example diff:

.. code-block:: dpatch

   std::shared_future<MyResultT> future;
   ...
   -executor.spin_until_future_complete<MyResultT>(future);
   +executor.spin_until_future_complete<std::shared_future<MyResultT>>(future);


For more details, see `ros2/rclcpp#1160 <https://github.com/ros2/rclcpp/pull/1160>`_.
For an example of the needed changes in user code, see `ros-visualization/interactive_markers#72 <https://github.com/ros-visualization/interactive_markers/pull/72>`_.

Change in default ``/clock`` subscription QoS profile
"""""""""""""""""""""""""""""""""""""""""""""""""""""

The default was changed from a reliable communication with history depth 10 to a best effort communication with history depth 1.
See `ros2/rclcpp#1312 <https://github.com/ros2/rclcpp/pull/1312>`_.

Waitable API
""""""""""""

Waitable API was modified to avoid issues with the ``MultiThreadedExecutor``.
This only affects users implementing a custom waitable.
See `ros2/rclcpp#1241 <https://github.com/ros2/rclcpp/pull/1241>`_ for more details.

rclpy
^^^^^^

Removal of deprecated Node.set_parameters_callback
""""""""""""""""""""""""""""""""""""""""""""""""""

Can be replaced with ``Node.add_on_set_parameters_callback``.
See `ros2/rclpy#504 <https://github.com/ros2/rclpy/pull/504>`_ for some examples.

rclcpp_action
^^^^^^^^^^^^^

Action client goal response callback signature changed
""""""""""""""""""""""""""""""""""""""""""""""""""""""

The goal response callback should now take a shared pointer to a goal handle, instead of a future.

For `example <https://github.com/ros2/examples/pull/291>`_, old signature:

.. code-block:: c++

   void goal_response_callback(std::shared_future<GoalHandleFibonacci::SharedPtr> future)

New signature:

.. code-block:: c++

   void goal_response_callback(GoalHandleFibonacci::SharedPtr goal_handle)

Related PR: `ros2/rclcpp#1311 <https://github.com/ros2/rclcpp/pull/1311>`_

rosidl_typesupport_introspection_c
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

API break in function that gets an element from an array
""""""""""""""""""""""""""""""""""""""""""""""""""""""""

The signature of the function was changed because it was semantically different to all the other functions used to get an element from an array or sequence.
This only affects authors of rmw implementations using the introspection typesupport.

For further details, see `ros2/rosidl#531 <https://github.com/ros2/rosidl/pull/531>`_.

Known Issues
------------

Timeline before the release
---------------------------

    Mon. March 22, 2021 - Alpha
        Preliminary testing and stabilization of ROS Core [1]_ packages.

    Mon. April 5, 2021 - Freeze
        API and feature freeze for ROS Core [1]_ packages in Rolling Ridley.
        Note that this includes ``rmw``, which is a recursive dependency of ``ros_core``.
        Only bug fix releases should be made after this point.
        New packages can be released independently.

    Mon. April 19, 2021 - Branch
        Branch from Rolling Ridley.
        ``rosdistro`` is reopened for Rolling PRs for ROS Core [1]_ packages.
        Galactic development shifts from ``ros-rolling-*`` packages to ``ros-galactic-*`` packages.

    Mon. April 26, 2021 - Beta
        Updated releases of ROS Desktop [2]_ packages available.
        Call for general testing.

    Mon. May 17, 2021 - RC
        Release Candidate packages are built.
        Updated releases of ROS Desktop [2]_ packages available.

    Thu. May 20, 2021 - Distro Freeze
        Freeze rosdistro.
        No PRs for Galactic on the ``rosdistro`` repo will be merged (reopens after the release announcement).

    Sun. May 23, 2021 - General Availability
        Release announcement.
        ``rosdistro`` is reopened for Galactic PRs.

.. [1] The ``ros_core`` variant is described in `REP 2001 (ros-core) <https://www.ros.org/reps/rep-2001.html#ros-core>`_.
.. [2] The ``desktop`` variant is described in `REP 2001 (desktop-variants) <https://www.ros.org/reps/rep-2001.html#desktop-variants>`_.
