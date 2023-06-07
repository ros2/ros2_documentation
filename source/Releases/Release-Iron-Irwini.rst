.. _latest_release:

.. _iron-release:

Iron Irwini (``iron``)
======================

.. toctree::
   :hidden:

   Iron-Irwini-Complete-Changelog

.. contents:: Table of Contents
   :depth: 2
   :local:

*Iron Irwini* is the ninth release of ROS 2.
What follows is highlights of the important changes and features in Iron Irwini since the last release.
For a list of all of the changes since Humble, see the :doc:`long form changelog <Iron-Irwini-Complete-Changelog>`.

Supported Platforms
-------------------

Iron Irwini is primarily supported on the following platforms:

Tier 1 platforms:

* Ubuntu 22.04 (Jammy): ``amd64`` and ``arm64``
* Windows 10 (Visual Studio 2019): ``amd64``

Tier 2 platforms:

* RHEL 9: ``amd64``

Tier 3 platforms:

* macOS: ``amd64``
* Debian Bullseye: ``amd64``

For more information about RMW implementations, compiler / interpreter versions, and system dependency versions see `REP 2000 <https://www.ros.org/reps/rep-2000.html>`__.

Installation
------------

`Install Iron Irwini <../../iron/Installation.html>`__

New features in this ROS 2 release
----------------------------------

API documentation generation for Python packages
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

ROS 2 has had automatic API documentation for C++ packages for several releases, e.g. https://docs.ros.org/en/rolling/p/rclcpp/generated/index.html.
Iron adds automatic API documentation for Python packages as well, e.g. https://docs.ros.org/en/rolling/p/rclpy/rclpy.html.

See https://github.com/ros-infrastructure/rosdoc2/pull/28, https://github.com/ros-infrastructure/rosdoc2/pull/49, https://github.com/ros-infrastructure/rosdoc2/pull/51, and https://github.com/ros-infrastructure/rosdoc2/pull/52 for more details.

Service introspection
^^^^^^^^^^^^^^^^^^^^^

It is now possible to enable service introspection on a per-service basis.
When enabled, this allows users to see the metadata associated with the client requesting a service, the server accepting the request, the server sending the response, and the client accepting the response.
Optionally, the contents of the client/server requests/responses can also be introspected.
All of the information is published on a hidden topic generated from the name of the service.
So if the service is called ``/myservice``, then the information will be published on ``/myservice/_service_event``.

Note that this functionality is disabled by default; to enable it, users must call ``configure_introspection`` after creating a server client or server.
There are examples showing how to do this in https://github.com/ros2/demos/tree/iron/demo_nodes_cpp/src/services (C++) and https://github.com/ros2/demos/blob/iron/demo_nodes_py/demo_nodes_py/services/introspection.py (Python).

See `REP 2012 <https://github.com/ros-infrastructure/rep/pull/360>`__ and the tracking bug at https://github.com/ros2/ros2/issues/1285 for more information.

Pre and post set parameter callback support
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For many releases now, users could register a callback to be called when parameters on a node were changed by an external entity (like ``ros2 param set``).
This callback could examine the changed parameter types and values, and reject the whole lot if one of them didn't meet certain criteria.
However, it could not modify the parameter list, nor should it have modified state (since there might be other callbacks after the set one that would reject the parameters).

This release adds in a pre and post callback.
The callbacks are called in this order:

* The "pre" set parameter callback, which can modify the list of parameters based on arbitrary criteria.
* The "set" parameter callback, which cannot modify the list and should only accept or reject the parameters based on their type and value (this is the existing callback).
* The "post" set parameter callback, which can make state changes based on parameters and is only called if the previous two callbacks are successful.

There are examples of this in action in https://github.com/ros2/demos/blob/iron/demo_nodes_cpp/src/parameters/set_parameters_callback.cpp (C++) and https://github.com/ros2/demos/blob/iron/demo_nodes_py/demo_nodes_py/parameters/set_parameters_callback.py (Python).

See https://github.com/ros2/rclcpp/pull/1947, https://github.com/ros2/rclpy/pull/966, and https://github.com/ros2/demos/pull/565 for more information.

Improved discovery options
^^^^^^^^^^^^^^^^^^^^^^^^^^

Previous ROS 2 versions offered limited discovery options.
The default behavior for DDS based RMW implementations was to discover any node reachable via multicast.
It could be limited to the same machine by setting the environment variable ``ROS_LOCALHOST_ONLY``, but any additional configuration required configuring the middleware directly, usually via middleware specific XML files and environment variables.
ROS Iron retains the same default discovery behavior, but deprecates ``ROS_LOCALHOST_ONLY`` in favor of more granular options.

* ``ROS_AUTOMATIC_DISCOVERY_RANGE`` controls how far ROS nodes will try to discover each other. Valid options are:

  * ``SUBNET`` - The default, and for DDS-based middlewares it will discover any node reachable via multicast.
  * ``LOCALHOST`` - Will only try to discover other nodes on the same machine.
  * ``OFF`` - Will not attempt to discover any other nodes automatically, even on the same machine.
  * ``SYSTEM_DEFAULT`` - Will not change any discovery settings.  This is useful when you already have custom settings for your middleware and don't want ROS to change them.

* ``ROS_STATIC_PEERS`` - A semicolon (``;``) separated list of addresses that ROS should try to discover nodes on.  This allows the user to connect to nodes on specifc machines (as long as their discovery range is not set to ``OFF``).

For example, you might have several robots with ``ROS_AUTOMATIC_DISCOVERY_RANGE`` set to ``LOCALHOST`` so they don't communicate with each other.
When you want to connect RViz to one of them, you add it's address to ``ROS_STATIC_PEERS`` in your terminal.
Now you can use ROS 2 CLI and visualization tools to interact with the robot.

See https://github.com/ros2/ros2/issues/1359 for more information about this feature.

Matched events
^^^^^^^^^^^^^^

In addition to QoS events, matched events can be generated when any publisher and subscription establishes or drops the connection between them.
Users can provide each publisher and subscription with callback functions that are triggered by matched events and handle them in a way they see fit, similar to how messages received on a topic are handled.

* publisher: this event happens when it finds a subscription which matches the topic and has compatible QoS or a connected subscription is disconnected.
* subscription: this event happens when it finds a publisher which matches the topic and has compatible QoS or a connected publisher is disconnected.

See the tracking issue at https://github.com/ros2/rmw/issues/330 for more information.

* C++ Demo of Matched Events: https://github.com/ros2/demos/blob/iron/demo_nodes_cpp/src/events/matched_event_detect.cpp
* Python Demo of Matched Events: https://github.com/ros2/demos/blob/iron/demo_nodes_py/demo_nodes_py/events/matched_event_detect.py

External configuration services of loggers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

It is now possible to configure node logger levels remotely via a service.
When the ``enable_logger_service`` option is enabled during node creation, the ``set_logger_levels`` and ``get_logger_levels`` services will be available.

Be advised that the ``enable_logger_service`` option is disabled by default, so the user needs to enable this option on node creation.

See https://github.com/ros2/ros2/issues/1355 for more information.

Type Description Distribution
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

It is now possible to communicate information about the types of ROS 2 messages, so that systems with potentially-different types of the same name may discover their compatibility more transparently.
This umbrella of capabilities, which is defined by a subset of REP-2011: Evolving Message Types, has had many parts land in Iron.

First, the introduction of the new package `type_description_interfaces <https://index.ros.org/p/type_description_interfaces/github-ros2-rcl_interfaces/#iron>`__ provides a common way to communicate the descriptions of ROS 2 communication interface types (msg, srv, action).

Next, a method to hash type descriptions has been decided on, the ROS Interface Hashing Standard (RIHS) - starting with the first version RIHS01.
RIHS hashes are automatically calculated for all compiled ROS types at build time, and baked into the generated code so that they can be inspected.
These hashes are also communicated automatically during discovery, and included in ``rmw_topic_endpoint_info_t`` for graph introspection queries such as ``get_publishers_info_by_topic``.

The full ``TypeDescription`` data structure, as well as the raw source text (such as ``.msg`` file) that were used to generate it are now baked in by default to the message libraries, so they can be used by ``typesupport`` or end users.
While we expect this data to provide value to most users, some users trying to minimize bytes in their install space can disable the feature when building ROS 2 Core by defining the CMake variable ``ROSIDL_GENERATOR_C_DISABLE_TYPE_DESCRIPTION_CODEGEN``.

Finally, the new service ``type_description_interfaces/GetTypeDescription.srv`` has been defined to allow nodes, on encountering an unknown RIHS type hash, to request the full definition from the node advertising that type.
Work is in progress to provide this feature natively in ROS 2 Nodes, as an optional switch on node construction.
This feature has not yet shipped, but is expected to be backported into Iron sometime mid-2023.
Meanwhile, user nodes could implement this service indepedently, using the stable service interface.

See `REP 2011 <https://github.com/ros-infrastructure/rep/pull/358>`__ for the design proposal.
See `Type Description Distribution <https://github.com/ros2/ros2/issues/1159>`__ for tracking development on the feature set.

Dynamic Types and Dynamic Messages
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Alongside the type description distribution feature mentioned above, is the ability to construct and access dynamically created types at runtime (i.e., dynamic types).
This feature is available in Iron for Fast DDS and ``rcl``, with new ``rmw`` interfaces for supporting the taking of messages as dynamic messages (i.e., messages built from or following the structure of the dynamic type).

First, utilities were introduced into `rosidl <https://index.ros.org/r/rosidl/github-ros2-rosidl/#iron>`__ to aid in the construction and manipulation of type descriptions.

Next, the `rosidl_dynamic_typesupport <https://index.ros.org/r/rosidl_dynamic_typesupport/github-ros2-rosidl_dynamic_typesupport/#iron>`__ package was written and provides a middleware-agnostic interface to construct dynamic types and dynamic messages at runtime.
Types can be constructed at runtime either programmatically, or by parsing a ``type_description_interfaces/TypeDescription`` message.

.. note::

   The ``rosidl_dynamic_typesupport`` library requires serialization support libraries to implement the middleware-specific dynamic type behavior.
   A serialization support library for Fast DDS was implemented in `rosidl_dynamic_typesupport_fastrtps <https://index.ros.org/r/rosidl_dynamic_typesupport_fastrtps/github-ros2-rosidl_dynamic_typesupport_fastrtps/#iron>`__.
   Ideally more middlewares will implement support libraries, expanding the number of middlewares that support this feature.

Finally, to support the use of dynamic types and dynamic messages, new methods were added to `rmw <https://index.ros.org/r/rmw/github-ros2-rmw/#iron>`__ and `rcl <https://index.ros.org/r/rcl/github-ros2-rcl/#iron>`__ that support:

- The ability to obtain of middleware-specific serialization support
- The ability to construct message type support at runtime that use dynamic types
- The ability to take dynamic messages using dynamic type

Work is in progress to enable the use of dynamic types to create subscriptions in the client libraries (see the ``rclcpp`` issue below), though it is uncertain when the feature will land or be backported.
This will allow users to subscribe to topics whose type descriptions are only known at runtime.
In the meantime, users may write their own subscriptions that subscribe to dynamic types by using the new ``rmw`` and ``rcl`` features introduced as part of this feature set.

See `REP 2011 <https://github.com/ros-infrastructure/rep/pull/358>`__ for the design proposal.
See `Dynamic Subscription <https://github.com/ros2/ros2/issues/1374>`__ for tracking development on the feature set, with `rclcpp <https://github.com/ros2/rclcpp/pull/2176>`__ needing the bulk of the work.

``launch``
^^^^^^^^^^

``PythonExpression`` now supports importing modules
"""""""""""""""""""""""""""""""""""""""""""""""""""

It is now possible to have a launch ``PythonExpression`` import modules before performing the evaluation.
This can be useful for pulling in additional functionality to be used when evaluating an expression.

See https://github.com/ros2/launch/pull/655 for more information.

``ReadyToTest`` can be called from an event handler
"""""""""""""""""""""""""""""""""""""""""""""""""""

It is now possible to register an event handler that uses ``ReadyToTest`` in its output.
This can be useful for doing things like downloading an asset before allowing a test to run.

See https://github.com/ros2/launch/pull/665 for more information.

Addition of ``AnySubstitution`` and ``AllSubstitution``
"""""""""""""""""""""""""""""""""""""""""""""""""""""""

It is now possible to specify a substitution to happen when any of the input arguments are true (``AnySubstitution``), or when all of the input arguments are true (``AllSubstitution``).

See https://github.com/ros2/launch/pull/649 for more details.

Addition of a new substitution to get the launch logging directory
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

It is now possible to use a substitution called ``LaunchLogDir`` to get the current logging directory for launch.

See https://github.com/ros2/launch/pull/652 for more details.

``launch_ros``
^^^^^^^^^^^^^^

Add a ``LifecycleTransition`` action
""""""""""""""""""""""""""""""""""""

It is now possible to send a transition signal to a lifecycle node via the new ``LifeCycleTransition`` action.

See https://github.com/ros2/launch_ros/pull/317 for more information.

Add a ``SetROSLogDir`` action
"""""""""""""""""""""""""""""

It is now possible to configure the directory used for logging via the ``SetROSLogDir`` action.

See https://github.com/ros2/launch_ros/pull/325 for more information.

Ability to specify a condition to a ``ComposableNode``
""""""""""""""""""""""""""""""""""""""""""""""""""""""

It is now possible to specify a condition that must be satisfied in order for a ``ComposableNode`` to be inserted into its container.

See https://github.com/ros2/launch_ros/pull/311 for more information.

``launch_testing``
^^^^^^^^^^^^^^^^^^

Timeout for process startup is now configurable
"""""""""""""""""""""""""""""""""""""""""""""""

Prior to this release, the ``ReadyToTest`` action would wait exactly 15 seconds for processes to start up.
If the processes took longer than that, they would fail.
There is now a new decorator called ``ready_to_test_action_timeout`` that allows the user to configure the amount of time to wait for the processes to start.

See https://github.com/ros2/launch/pull/625 for more information.

``rclcpp``
^^^^^^^^^^

Addition of a new paradigm for handling ``Node`` and ``LifecycleNode``
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

The ``Node`` and ``LifecycleNode`` classes are related in that they both provide the same base set of methods (though ``LifecycleNode`` provides additional methods as well).
Due to various implementation considerations, they are not derived from a common base class.

This has led to some trouble for downstream code that wants to accept either a ``Node`` or a ``LifecycleNode``.
One solution is to have two method signatures, one that accepts a ``Node`` and one that accepts a ``LifecycleNode``.
The other, recommended solution is to have a method that accepts the "node interfaces" pointers that can be accessed from both classes, e.g.

.. code-block:: C++

   void do_thing(rclcpp::node_interfaces::NodeGraphInterface graph)
   {
     fprintf(stderr, "Doing a thing\n");
   }

   void do_thing(rclcpp::Node::SharedPtr node)
   {
     do_thing(node->get_node_graph_interface());
   }

   void do_thing(rclcpp::LifecycleNode::SharedPtr node)
   {
     do_thing(node->get_node_graph_interface());
   }

This works, but can get a bit unwieldy when many node interfaces are needed.
To make this a bit better, there is now a new ``NodeInterfaces`` class that can be constructed to contain the interfaces, and then be used by other code.

There are examples on how to use this in https://github.com/ros2/rclcpp/pull/2041.

Introduction of a new executor type: the Events Executor
""""""""""""""""""""""""""""""""""""""""""""""""""""""""

The ``EventsExecutor`` from iRobot has been merged into the main ``rclcpp`` codebase.
This alternative executor implementation uses event-driven callbacks from the middleware implementations to fire callbacks at the ``rclcpp`` layer.
In addition to the push-based model, the ``EventsExecutor`` also moves timer management into a separate thread, which can allow for more accurate results and lower overhead, especially with many timers.

The ``EventsExecutor`` has a substantial set of documentation and use-in-practice that make it a strong candidate for inclusion in the ``rclcpp`` codebase.
For information about the initial implementation proposal as well as performance benchmarks, see https://discourse.ros.org/t/ros2-middleware-change-proposal/15863.
For more information about the design, see the design PR: https://github.com/ros2/design/pull/305.

Since the API is the same, trying the ``EventsExecutor`` is as straightforward as replacing your current Executor implementation (eg. ``SingleThreadedExecutor``):

.. code-block:: C++

    #include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
    using rclcpp::experimental::executors::EventsExecutor;

    EventsExecutor executor;
    executor.add_node(node);
    executor.spin();

**Note** The ``EventsExecutor`` and ``TimersManager`` are currently in the ``experimental`` namespace.
While it has been used as a standalone implementation for some time https://github.com/irobot-ros/events-executor, it was decided to use the ``experimental`` namespace for at least one release to give latitude in changing the API within the release.
Use caution as it will not be subject to the same API/ABI guarantees that the non-experimental code has.

``rclpy``
^^^^^^^^^

Ability to wait for another node to join the graph
""""""""""""""""""""""""""""""""""""""""""""""""""

It is now possible to wait for another node to join the network graph with code like the following:

.. code-block:: Python

  node.wait_for_node('/fully_qualified_node_name')

See https://github.com/ros2/rclpy/pull/930 for more information.

Implementation of ``AsyncParameterClient``
""""""""""""""""""""""""""""""""""""""""""

``rclpy`` now has an ``AsyncParameterClient`` class, bringing it to feature parity with ``rclcpp``.
This class is used to perform parameter actions on a remote node without blocking the calling node.

See https://github.com/ros2/rclpy/pull/959 for more information and examples.

Subscription callbacks can now optionally get the message info
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

It is now possible to register for a subscription callback with a function signature that takes both the message, and the message info, like:

.. code-block:: Python

  def msg_info_cb(msg, msg_info):
      print('Message info:', msg_info)

  node.create_subscription(msg_type=std_msgs.msg.String, topic='/chatter', qos_profile=10, callback=msg_info_cb)

The message info structure contains various pieces of information like the sequence number of the message, the source and received timestamps, and the GID of the publisher.

See https://github.com/ros2/rclpy/pull/922 for more information.

Optional argument that hides assertions for messages class
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
All message classes now include a new optional argument that allows the hiding of assertions for each field type from the message.
By default, assertions are hidden, which provides a performance improvement during runtime.
In order to enable the assertions for development/debugging purposes, you are given two choices:

1. Define the environment variable ``ROS_PYTHON_CHECK_FIELDS`` to ``'1'`` (this would affect all the messages in your project):

.. code-block:: Python

  import os
  from std_msgs.msg import String

  os.environ['ROS_PYTHON_CHECK_FIELDS'] = '1'
  new_message=String()

2. Select the specific behavior for a single message by explicitly defining the new argument in the constructor:

.. code-block:: Python

  from std_msgs.msg import String

  new_message=String(check_fields=True)

See https://github.com/ros2/rosidl_python/pull/194 for more information.

``ros2param``
^^^^^^^^^^^^^

Option to timeout when waiting for a node with ``ros2 param``
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

It is now possible to have the various ``ros2 param`` commands timeout by passing ``--timeout`` to the command.

See https://github.com/ros2/ros2cli/pull/802 for more information.

Deprecated options were removed
""""""""""""""""""""""""""""""""

``--output-dir`` and ``--print`` options with ``dump`` command have been removed.

See https://github.com/ros2/ros2cli/pull/824 for more information.

``ros2topic``
^^^^^^^^^^^^^

``now`` as keyword for ``builtin_interfaces.msg.Time`` and ``auto`` for ``std_msgs.msg.Header``
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

``ros2 topic pub`` now allows to set a ``builtin_interfaces.msg.Time`` message to the current time via the ``now`` keyword.
Similarly, a ``std_msg.msg.Header`` message will be automatically generated when passed the keyword ``auto``.
This behavior matches that of ROS 1's ``rostopic`` (http://wiki.ros.org/ROS/YAMLCommandLine#Headers.2Ftimestamps)

Related PR: `ros2/ros2cli#749 <https://github.com/ros2/ros2cli/pull/749>`_

``ros2 topic pub`` can be configured to wait a maximum amount of time
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

The command ``ros2 topic pub -w 1`` will wait for at least that number of subscribers before publishing a message.
This release adds in a ``--max-wait-time`` option so that the command will only wait a maximum amount of time before quitting if no subscribers are seen.

See https://github.com/ros2/ros2cli/pull/800 for more information.

``ros2 topic echo`` can be configured to wait a maximum amount of time
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

The command ``ros2 topic echo`` now accepts a ``--timeout`` option, which controls the maximum amount of time that the command will wait for a publication to happen.

See https://github.com/ros2/ros2cli/pull/792 for more information.

Deprecated option was removed
"""""""""""""""""""""""""""""

``--lost-messages`` option with ``echo`` command has been removed.

See https://github.com/ros2/ros2cli/pull/824 for more information.

Changes since the Humble release
--------------------------------

Change to the default console logging file flushing behavior
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This specifically applies to the default ``spdlog`` based logging backend in ROS 2, implemented in ``rcl_logging_spdlog``.
Log file flushing was changed to flush every time an "error" log message is used, e.g. each ``RCLCPP_ERROR()`` call, and also periodically every five seconds.

Previously, ``spdlog`` was used without configuring anything other than creating the sink for logging to a file.

We tested the change and did not find that the CPU overhead was significant, even on machines with slow disks (e.g. sd cards).
However, if this change is causing you problems, you can get the old behavior by setting the ``RCL_LOGGING_SPDLOG_EXPERIMENTAL_OLD_FLUSHING_BEHAVIOR=1`` environment variable.

Later we would like to have support for a full configuration file (see: https://github.com/ros2/rcl_logging/issues/92), giving you more flexibility in how the logging is done, but that is work that is only planned right now.

  Therefore, **this environment variable should be considered experimental and subject to removal without deprecation in the future**, when we add config file support for the ``rcl_logging_spdlog`` logging backend.

See this pull request for more details about the change: https://github.com/ros2/rcl_logging/pull/95

``ament_cmake_auto``
^^^^^^^^^^^^^^^^^^^^

Include dependencies are now marked as SYSTEM
"""""""""""""""""""""""""""""""""""""""""""""

When using ``ament_auto_add_executable`` or ``ament_auto_add_library``, dependencies are now automatically added as ``SYSTEM``.
This means that warnings in the header files of the dependencies will not be reported.

See https://github.com/ament/ament_cmake/pull/385 for more details.

``ament_cmake_nose``
^^^^^^^^^^^^^^^^^^^^

Package has been deprecated and removed
"""""""""""""""""""""""""""""""""""""""

The Python ``nose`` package has long been deprecated.
Since none of the open-source packages currently released into Humble or Rolling currently depend on it, this release deprecates and removes the ament wrapper around it.

See https://github.com/ament/ament_cmake/pull/415 for more information.

``ament_lint``
^^^^^^^^^^^^^^

Files can be excluded from linter checks
""""""""""""""""""""""""""""""""""""""""

Certain files can now be excluded from linter checks by setting the ``AMENT_LINT_AUTO_FILE_EXCLUDE`` CMake variable before calling ``ament_lint_auto_find_test_dependencies``.

See https://github.com/ament/ament_lint/pull/386 for more information.

``camera_info_manager``
^^^^^^^^^^^^^^^^^^^^^^^

Lifecycle node support
""""""""""""""""""""""

``camera_info_manager`` now supports lifecycle nodes in additional to regular ROS 2 nodes.

See https://github.com/ros-perception/image_common/pull/190 for more information.

``launch``
^^^^^^^^^^

``LaunchConfigurationEquals`` and ``LaunchConfigurationNotEquals`` are deprecated
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

The ``LaunchConfigurationEquals`` and ``LaunchConfigurationNotEquals`` conditions are deprecated, and will be removed in a future release.
Instead, the more universal ``Equals`` and ``NotEquals`` substitutions should be used instead.

See https://github.com/ros2/launch/pull/649 for more details.

``launch_ros``
^^^^^^^^^^^^^^

Renamed classes which used ``Ros`` in the name to use ``ROS`` in line with PEP8
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Classes that were changed:

* ``launch_ros.actions.RosTimer`` -> ``launch_ros.actions.ROSTimer``
* ``launch_ros.actions.PushRosNamespace`` -> ``launch.actions.PushROSNamespace``

The old class names are still there, but will be deprecated.

See https://github.com/ros2/launch_ros/pull/326 for more information.

``launch_xml``
^^^^^^^^^^^^^^

Expose ``emulate_tty`` to XML frontend
""""""""""""""""""""""""""""""""""""""

It has been possible for several releases to have the ``launch`` Python code use pseudo-terminals to emulate a TTY (and hence do things like print colors).
That functionality is now available in the XML frontend by passing the ``emulate_tty`` argument to an executable command.

See https://github.com/ros2/launch/pull/669 for more information.

Expose ``sigterm_timeout`` and ``sigkill_timeout`` to XML frontend
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

It has been possible for several releases to configure the maximum timeout value for the SIGTERM and SIGKILL signals in the ``launch`` Python code.
That functionality is now available in the XML frontend by passing the ``sigterm_timeout`` or ``sigkill_timeout`` argument to an executable command.

See https://github.com/ros2/launch/pull/667 for more information.

``launch_yaml``
^^^^^^^^^^^^^^^

Expose ``emulate_tty`` to YAML frontend
"""""""""""""""""""""""""""""""""""""""

It has been possible for several releases to have the ``launch`` Python code use pseudo-terminals to emulate a TTY (and hence do things like print colors).
That functionality is now available in the YAML frontend by passing the ``emulate_tty`` argument to an executable command.

See https://github.com/ros2/launch/pull/669 for more information.

Expose ``sigterm_timeout`` and ``sigkill_timeout`` to YAML frontend
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

It has been possible for several releases to configure the maximum timeout value for the SIGTERM and SIGKILL signals in the ``launch`` Python code.
That functionality is now available in the YAML frontend by passing the ``sigterm_timeout`` or ``sigkill_timeout`` argument to an executable command.

See https://github.com/ros2/launch/pull/667 for more information.

``message_filters``
^^^^^^^^^^^^^^^^^^^

New approximate time policy
"""""""""""""""""""""""""""

Add in a simpler approximate time policy called ``ApproximateEpsilonTime``.
This time policy works like ``ExactTime``, but allows timestamps being within a epsilon tolerance.
See https://github.com/ros2/message_filters/pull/84 for more information.

New upsampling time policy
""""""""""""""""""""""""""

Adds in a new time policy called ``LatestTime``.
It can synchronize up to 9 messages by their rates with upsampling via zero-order-hold.
See https://github.com/ros2/message_filters/pull/73 for more information.

``rcl_yaml_param_parser``
^^^^^^^^^^^^^^^^^^^^^^^^^

Support for YAML ``!!str`` syntax in parameter files
""""""""""""""""""""""""""""""""""""""""""""""""""""

It is now possible to force the ROS parameter file parser to interpret a field as a string using the YAML ``!!str`` syntax.
See https://github.com/ros2/rcl/pull/999 for more information.

``rclcpp``
^^^^^^^^^^

Default number of threads for multi-threaded executor has been changed
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

If the user doesn't specify otherwise, the default number of threads for the multi-threaded executor will be set to the number of CPUs on the machine.
If the underlying OS doesn't support getting this information, it will be set to 2.

See https://github.com/ros2/rclcpp/pull/2032 for more information.

A warning is now printed when QoS of KEEP_LAST is specified with a depth of 0
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Specifying a QoS of KEEP_LAST with a depth of 0 is a nonsensical arrangement, since the entity wouldn't be able to send or receive any data.
``rclcpp`` will now print a warning if this combination is specified, but will still continue on and let the underlying middleware choose a sane value (generally a depth of 1).

See https://github.com/ros2/rclcpp/pull/2048 for more information.

Deprecated ``RCLCPP_SCOPE_EXIT`` macro was removed
""""""""""""""""""""""""""""""""""""""""""""""""""

In Humble, the macro ``RCLCPP_SCOPE_EXIT`` was deprecated in favor of ``RCPPUTILS_SCOPE_EXIT``.
In Iron, the ``RCLCPP_SCOPE_EXIT`` macro has been completely removed.

``rclpy``
^^^^^^^^^

Default number of threads for multi-threaded executor has been changed
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

If the user doesn't specify otherwise, the default number of threads for the multi-threaded executor will be set to the number of CPUs on the machine.
If the underlying OS doesn't support getting this information, it will be set to 2.

See https://github.com/ros2/rclpy/pull/1031 for more information.

A warning is now printed when QoS of KEEP_LAST is specified with a depth of 0
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Specifying a QoS of KEEP_LAST with a depth of 0 is a nonsensical arrangement, since the entity wouldn't be able to send or receive any data.
``rclpy`` will now print a warning if this combination is specified, but will still continue on and let the underlying middleware choose a sane value (generally a depth of 1).

See https://github.com/ros2/rclpy/pull/1048 for more information.

Time and Duration no longer raise exception when compared to another type
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

It is now possible to compare ``rclpy.time.Time`` and ``rclpy.duration.Duration`` to other types without getting exceptions.
If the types are not comparable, the comparison returns ``False``.
Note that this is a behavior change from previous releases.

.. code-block:: Python

  print(None in [rclpy.time.Time(), rclpy.duration.Duration()])  # Prints "False" instead of raising TypeError

See https://github.com/ros2/rclpy/pull/1007 for more information.

``rcutils``
^^^^^^^^^^^

Improve the performance of message logging
""""""""""""""""""""""""""""""""""""""""""

The code used to output a log message when ``RCUTILS_LOG_*`` or ``RCLCPP_*`` was optimized to reduce overhead.
These log messages should now be more efficient, though they should still not be called at high rates.
See https://github.com/ros2/rcutils/pull/381, https://github.com/ros2/rcutils/pull/372, https://github.com/ros2/rcutils/pull/369, and https://github.com/ros2/rcutils/pull/367 for more information.

Deprecated ``rcutils/get_env.h`` header was removed
"""""""""""""""""""""""""""""""""""""""""""""""""""

In Humble, the header ``rcutils/get_env.h`` was deprecated in favor of ``rcutils/env.h``.
In Iron, the ``rcutils/get_env.h`` header been completely removed.

``rmw``
^^^^^^^

Change the GID storage to 16 bytes
""""""""""""""""""""""""""""""""""

The GID in the RMW layer is meant to be a globally unique identifier for writers in the ROS graph.
Previously, this was erroneously set to 24 bytes based on a bug in an old RMW implementation.
But the ``rmw`` package should define this, and all of the implementations should conform to that.
Thus, this release defines it as 16 bytes (the DDS standard), and changes all implementations to use that definition.

See https://github.com/ros2/rmw/pull/345 and the (closed, but relevant) https://github.com/ros2/rmw/pull/328 for more information.

``rmw_dds_common``
^^^^^^^^^^^^^^^^^^

Change the GID storage to 16 bytes
""""""""""""""""""""""""""""""""""

Along with the change in the ``rmw`` layer, change the message that sends out GID information to 16 bytes.

See https://github.com/ros2/rmw_dds_common/pull/68 for more information.

``ros2topic``
^^^^^^^^^^^^^

``ros2 topic hz/bw/pub`` now respect ``use_sim_time``
"""""""""""""""""""""""""""""""""""""""""""""""""""""

When running under simulation, the ROS 2 ecosystem generally gets its time from a ``/clock`` topic published by the simulator (rather than using the system clock).
ROS 2 nodes are typically informed of this change by setting the ``use_sim_time`` parameter on the node.
The node created by the ``ros2 topic`` commands ``hz``, ``bw``, and ``pub`` now respect that parameter and will use simulation time as appropriate.

See https://github.com/ros2/ros2cli/pull/754 for more information.

``rosbag2``
^^^^^^^^^^^

Change default bag file type to ``mcap``
""""""""""""""""""""""""""""""""""""""""

Prior to this release, by default rosbag2 would record data into sqlite3 databases.
During testing, it was found that in many cases this was not performant enough and lacked certain features desirable for offline processing.

To meet these needs, a new bag format (influenced by the original ROS 1 bag file format) called ``mcap`` was developed.
This bag file format has many of the missing features from the sqlite3 file format, and should also be more performant.

This release switches to using ``mcap`` as the default file format for writing new bags.
The old ``sqlite3`` file format is still available and can be selected by the user for writing if desired.
This release also allows playing back data from either the ``sqlite3`` file format or the ``mcap`` file format.

See https://github.com/ros2/rosbag2/pull/1160 for more information.

Store message definitions in bag files with SQLite3 plugin
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Now we support saving message definitions to the ``sqlite3`` database file in the same format
as we are saving it to the ``mcap`` file.
This opens an opportunity for third-party tools to have
the ability to deserialize rosbag2 files without having the correct version of all the original
.msg files on the machine that is decoding the bag file recorded with ``sqlite3`` plugin.

See https://github.com/ros2/rosbag2/issues/782 and https://github.com/ros2/rosbag2/pull/1293 for
more information.


New playback and recording controls
"""""""""""""""""""""""""""""""""""

Several pull requests have been added to enhance the user's control over playback of bags.
Pull request `960 <https://github.com/ros2/rosbag2/pull/960>`_ adds the ability to play bag for
a specified number of seconds.
And pull request `1005 <https://github.com/ros2/rosbag2/pull/1005>`_ allows to play bag until specified timestamp.
Another pull request `1007 <https://github.com/ros2/rosbag2/pull/1007>`_ adds the ability to
stop playback remotely via service call.
Stop will unpause player if it was in pause mode, stop playback and force exit from play() method if it was in progress.

Managing recording via service calls
""""""""""""""""""""""""""""""""""""

There are new options to control the recording process from remote nodes.
The pull request `1131 <https://github.com/ros2/rosbag2/pull/1131>`_ adds the ability to pause and
resume recording via service calls.
Another pull request `1115 <https://github.com/ros2/rosbag2/pull/1115>`_ adds the ability to split
bags during recording by sending service call.

Filtering topics via regular expression during playback
"""""""""""""""""""""""""""""""""""""""""""""""""""""""

Users sometimes need to replay only a subset of topics from recorded bags and the following two pull request
adds such capability.
Pull request `1034 <https://github.com/ros2/rosbag2/pull/1034>`_ adds a new option
``--topics-regex`` that allows filtering topics via regular expressions.
The ``--topics-regex`` option accepts multiple regular expressions separated by space.
And pull request `1046 <https://github.com/ros2/rosbag2/pull/1046>`_ adds the ability to exclude some
certain topics from being replayed by providing regular expression in a new ``--exclude``
(and ``-x``) option.

Allow plugins to register their own CLI verb arguments
""""""""""""""""""""""""""""""""""""""""""""""""""""""

Pull request `1209 <https://github.com/ros2/rosbag2/pull/1209>`_ adds the ability for ``rosbag2`` plugins to
register an optional Python entrypoint providing plugin-specific CLI argument values.
As a result the command line option ``--storage-preset-profile`` for ``ros2 bag record`` verb will have
different valid options depending on the underlying storage plugin.

Other changes
"""""""""""""

The pull request `1038 <https://github.com/ros2/rosbag2/pull/1038>`_ adds the ability to record
any key/value pair in 'custom' field in metadata.yaml file.
It is useful when users need to save some hardware specific id or coordinates where the recording was captured.
And pull request `1180 <https://github.com/ros2/rosbag2/pull/1180>`_ adds an option to change the underlying
node name for the recorder via providing the new command line ``--node-name`` option.
This option might be used for creating remote distributed recording with multiple rosbag2 recorder instances.
It provides the ability to send service calls for managing the recording process to the dedicated
rosbag2 recorder instances.

``rosidl_python``
^^^^^^^^^^^^^^^^^

Modification of content of ``__slots__`` attribute
""""""""""""""""""""""""""""""""""""""""""""""""""

So far, the attribute ``__slots__`` from the python message classes, have been used as the member that contains the field names of the message.
In Iron, this attribute no longer contains only the field names from the message structure, but the field names for all the class members.
Therefore, users shouldn't rely on this attribute to retrieve the field names information, instead, users should retrieve it using the method ``get_field_and_field_types()``.

See https://github.com/ros2/rosidl_python/pull/194 for more information.

``rviz``
^^^^^^^^

Map display can now be shown as binary
""""""""""""""""""""""""""""""""""""""

The RViz map display can now display the map as binary, with a settable threshold.
This is useful in some cases to inspect maps or in combination with planners that have a settable threshold.

See https://github.com/ros2/rviz/pull/846 for more information.

Camera display plugin respects the ROI in the CameraInfo message
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

The CameraDisplay plugin now honors the region-of-interest (ROI) settings in the CameraInfo message, if it is provided.
This accounts for the fact that an image was cropped by the camera driver to reduce the bandwidth.

See https://github.com/ros2/rviz/pull/864 for more information.

Binary STL files from SOLIDWORKS work without error
"""""""""""""""""""""""""""""""""""""""""""""""""""

A change was made to the STL loader such that it accepts binary STL files from SOLIDWORKS that have the word "solid" in them.
This technically violates the STL specification, but is common enough that a special case is added to handle these files.

See https://github.com/ros2/rviz/pull/917 for more information.

``tracetools``
^^^^^^^^^^^^^^

Tracing instrumentation is now included by default on Linux
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

The ROS 2 core has had tracing instrumentation for a while now.
However, it was compiled out by default.
To get the instrumentation, the LTTng tracer had to be manually installed before rebuilding ROS 2 from source.
In Iron, the tracing instrumentation and tracepoints are included by default; the LTTng tracer is therefore now a ROS 2 dependency.

Note that this only applies to Linux.

See https://github.com/ros2/ros2_tracing/pull/31 and https://github.com/ros2/ros2/issues/1177 for more information.
See :doc:`this how-to guide to remove the instrumentation (or add the instrumentation with Humble and older) <../How-To-Guides/Building-ROS-2-with-Tracing-Instrumentation>`.

New tracepoints for ``rclcpp`` intra-process are added
""""""""""""""""""""""""""""""""""""""""""""""""""""""

New tracepoints have been added to support ``rclcpp`` intra-process communication.
This allows the evaluation of the time between the message publishing and the callback start in intra-process communication.

See https://github.com/ros2/ros2_tracing/pull/30 and https://github.com/ros2/rclcpp/pull/2091 for more information.

Known Issues
------------

* ``rmw_connextdds`` does not work with Windows Binary release packages.
  RTI is not longer distributing ``RTI ConnextDDS 6.0.1`` which was used by the packaging jobs to create the binaries for Windows.
  Instead they now distribute ``RTI ConnextDDS 6.1.0`` which is ABI incompatible with the generated binaries.
  The solution is to rely on source builds of ROS 2 and ``rmw_connextdds`` on Windows.

* ``sros2`` on Windows requires users to downgrade the ``cryptography`` python module to ``cryptography==38.0.4`` as discussed `here <https://github.com/ros2/sros2/issues/285>`_.

* ``ros1_bridge`` does not work with ROS Noetic packages from `upstream Ubuntu <https://packages.ubuntu.com/jammy/ros-core-dev>`_.  The suggested workaround is to build ROS Noetic from sources, then build the ``ros1_bridge`` using that.

Release Timeline
----------------

    November, 2022 - Platform decisions
        REP 2000 is updated with the target platforms and major dependency versions.

    By January, 2023 - Rolling platform shift
        Build farm is updated with the new platform versions and dependency versions for Iron Irwini (if necessary).

    Mon. April 10, 2023 - Alpha + RMW freeze
        Preliminary testing and stabilization of ROS Base [1]_ packages, and API and feature freeze for RMW provider packages.

    Mon. April 17, 2023 - Freeze
        API and feature freeze for ROS Base [1]_ packages in Rolling Ridley.
        Only bug fix releases should be made after this point.
        New packages can be released independently.

    Mon. April 24, 2023 - Branch
        Branch from Rolling Ridley.
        ``rosdistro`` is reopened for Rolling PRs for ROS Base [1]_ packages.
        Iron development shifts from ``ros-rolling-*`` packages to ``ros-iron-*`` packages.

    Mon. May 1, 2023 - Beta
        Updated releases of ROS Desktop [2]_ packages available.
        Call for general testing.

    Mon. May 15, 2023 - Release Candidate
        Release Candidate packages are built.
        Updated releases of ROS Desktop [2]_ packages available.

    Thu. May 18, 2023 - Distro Freeze
        Freeze rosdistro.
        No PRs for Iron on the ``rosdistro`` repo will be merged (reopens after the release announcement).

    Tue. May 23, 2023 - General Availability
        Release announcement.
        ``rosdistro`` is reopened for Iron PRs.

.. [1] The ``ros_base`` variant is described in `REP 2001 (ros-base) <https://www.ros.org/reps/rep-2001.html#ros-base>`_.
.. [2] The ``desktop`` variant is described in `REP 2001 (desktop-variants) <https://www.ros.org/reps/rep-2001.html#desktop-variants>`_.

Development progress
--------------------

For progress on the development and release of Iron Irwini, see `the tracking GitHub issue <https://github.com/ros2/ros2/issues/1298>`__.

For the broad process followed by Iron Irwini, see the :doc:`process description page <Release-Process>`.
