.. redirect-from::

  Feature-Ideas

.. _FeatureIdeas:

Feature Ideas
=============

.. contents:: Table of Contents
   :depth: 2
   :local:

The following are feature ideas in no specific order.
This list contains features that we think are important and can make for good contributions to ROS 2.
:doc:`Please get in touch with us <../Contact>` before digging into a new feature.
We can offer guidance, and connect you with other developers.

Design / Concept
----------------

* IDL format

  * Leverage new features like grouping constants into enums
  * Extend usage to ``.idl`` files with just constants and/or declare parameters with ranges
  * Revisit constraints of IDL interface naming, see `ros2/design#220 <https://github.com/ros2/design/pull/220>`_

* Create migration plan for ROS 1 -> ROS 2 transition
* Uniqueness of node names, see `ros2/design#187 <https://github.com/ros2/design/issues/187>`_
* Specific "API" of a node in terms of topics / services / etc. in a descriptive format, see `ros2/design#266 <https://github.com/ros2/design/pull/266>`_

Infrastructure and tools
------------------------

* Building

  * Consolidate https://build.ros2.org and https://ci.ros2.org
  * Provision macOS
  * Windows and macOS packages
  * Support profiles in ``colcon``

* Documentation

  * Deprecate https://design.ros2.org.  Content should move to either an REP, to https://github.com/ros2/ros2_documentation, or be removed.
  * Fix per-package documentation builder to be able to document build artifacts, i.e. messages, services, actions, etc.
  * Make https://docs.ros.org/en/ros2_documentation automatically rebuild on changes to https://github.com/ros2/ros2_documentation.
  * ``ament`` documentation
  * Add documentation examples of using ROS 2 with Jupyter notebooks.
  * Add documentation for implementing a new RMW.
  * Provide three different kinds of content:

    * "demos" to show features and cover them with tests
    * "examples" to show a simple/minimalistic usage which might have multiple ways to do something
    * "tutorials" which contain more comments and anchors for the wiki (teaching one recommended way)

New features
------------

The trailing stars indicate the rough effort: 1 star for small, 2 stars for medium, 3 stars for large.


* Logging improvements [\* / \*\*]

  * Configuration specified in a file
  * Per-logger configuration (enabling e.g. ``rqt_logger_level``)

* Time related

  * Support rate and sleep based on clock

* Additional Graph API features [\*\* / \*\*\*]

  * Introspect QoS setting for all (especially remote) topics
  * a la ROS 1 Master API: https://wiki.ros.org/ROS/Master_API
  * Event-based notification
  * Requires knowledge of the rmw interface which needs to be extended

* Executor

  * Performance improvements (mostly around the waitset)
  * Deterministic ordering (fair scheduling)
  * Decouple waitables

* Message generation

  * Catch-up message generation for languages not supported out-of-the-box
  * Mangle field names in message to avoid language specific keywords
  * Improve generator performance by running them in the same Python interpreter

* Launch

  * Support for launching multi-node executables (i.e. manual composition)
  * Extend launch XML/YAML support: events and event handlers, tag namespaces and aliasing

* Rosbag

  * Support recording services (and actions)

* ros1_bridge

  * Support bridging actions

* RMW configuration

  * Unified standard way of configuring the middleware

* Remapping [\*\* / \*\*\*]

  * Dynamic remapping and aliasing through a Service interface

* Type masquerading [\*\*\*]

  * a la ROS 1's message traits: https://wiki.ros.org/roscpp/Overview/MessagesSerializationAndAdaptingTypes
  * Requires knowledge of the typesupport system

* Expand on real-time safety [\*\*\*]

  * For services, clients, and parameters
  * Expose more quality of service parameters related to real-time performance
  * Real-time-safe intra-process messaging

* Multi-robot supporting features and demos [\*\*\*]

  * Undesired that all nodes across all robots share the same domain (and discover each other)
  * Design how to “partition” the system

* Support more DDS / RTPS implementations:

  * RTI Connext DDS Micro (implemented, not enabled by default or officially supported).

* Security improvements:

  * More granularity in security configuration (allow authentication only, authentication and encryption, etc.) [\*]
  * Integrate DDS-Security logging plugin (unified way to aggregate security events and report them to the users through a ROS interface) [\*\*]
  * Key storage security (right now, keys are just stored in the filesystem) [\*\*]
  * More user friendly interface (make it easier to specify security config). Maybe a Qt GUI? This GUI could also assist in distributing keys somehow. [\*\*\*]
  * A way to say "please secure this running system" with some UI that would auto-generate keys and policies for everything that is currently running. [\*\*\*]
  * If there are hardware-specific features for securing keys or accelerating encryption/signing messages, that could be interesting to add to DDS/RTPS implementations that don't use it already. [\*\*\*]

Reducing Technical Debt
-----------------------

* Fix flaky tests on https://ci.ros2.org/view/nightly.
* Ability to run (all) unit tests with tools e.g. valgrind, clang-tidy, clang static analysis (scan-build), ASAN, TSAN, UBSAN, etc.
* API review, specifically user-facing APIs in rclcpp and rclpy
* Refactor the rclcpp API into separate packages focused on a single aspect, rclcpp should afterward still provide the combined user-facing API
* Revisit message allocators, consider using std::polymorphic_allocator to address problems
* Synchronize / reconcile `design docs <https://design.ros2.org>`__ with the implementation.
* Address / classify pending tickets
* Address TODOs in code / docs
* Remove tinyxml as a dependency
