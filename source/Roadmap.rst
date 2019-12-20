.. _Roadmap:

Roadmap
=======

.. contents:: Table of Contents
   :depth: 2
   :local:

Please see the page of the :ref:`upcoming distribution <upcoming-release>` for more information what is planned to be part of that release.

Please see the :ref:`Distributions page <Releases>` for the timeline of and information about future distributions.

For more information on the design of ROS 2 please see `design.ros2.org <http://design.ros2.org>`__.
The core code for ROS 2 is on the `ros2 github organization <https://github.com/ros2>`__.
The Discourse forum/mailing list for discussing ROS 2 design is `ng-ros <https://discourse.ros.org/c/ng-ros>`__.
Questions should be asked on `ROS answers <https://answers.ros.org>`__\ , make sure to include at least the ``ros2`` tag and the rosdistro version you are running, e.g. ``ardent``.

Feature ideas in no specific order
----------------------------------

Design / Concept
~~~~~~~~~~~~~~~~

* IDL format

  * Leverage new features like grouping constants into enums
  * Extend usage to ``.idl`` files with just constants and/or declare parameters with ranges
  * Revisit constraints of IDL interface naming, see `ros2/design#220 <https://github.com/ros2/design/pull/220>`_

* Progress on migration plan
* Reconsider 1-to-1 mapping of ROS nodes to DDS participants, see `ros2/design#250 <https://github.com/ros2/design/pull/250>`_
* Uniqueness of node names, see `ros2/design#187 <https://github.com/ros2/design/issues/187>`_
* Specific "API" of a node in terms of topics / services / etc in a descriptive format, see `ros2/design#266 <https://github.com/ros2/design/pull/266>`_

Infrastructure and tools
~~~~~~~~~~~~~~~~~~~~~~~~

* Building

  * Consolidate build.ros2.org and ci.ros2.org
  * Provision macOS and Windows machines
  * Windows and Mac OS packages
  * Support profiles in ``colcon``

* Documentation

  * Improve documentation platform
  * Support for ``doc`` jobs on the `ROS 2 buildfarm <http://build.ros2.org>`__
  * Consider consolidating with design.ros2.org
  * Provide three different kinds of content:

    * "demos" to show features and cover them with tests
    * "examples" to show a simple/minimalistic usage which might have multiple ways to do something
    * "tutorials" which contain more comments and anchors for the wiki (teaching one recommended way)

New features
~~~~~~~~~~~~

The trailing stars indicate the rough effort: 1 star for small, 2 stars for medium, 3 stars for large.


* Logging improvements [\* / \*\*]

  * Configuration specified in a file
  * Per-logger configuration (enabling e.g. ``rqt_logger_level``)

* Time related

  * Support rate and sleep based on clock

* Parameters

  * enforce type

* Additional Graph API features [\*\* / \*\*\*]

  * Introspect QoS setting for all (especially remote) topics
  * a la ROS 1 Master API: http://wiki.ros.org/ROS/Master_API
  * Event-based notification
  * Requires knowledge of the rmw interface which needs to be extended

* Executor

  * Performance improvements (mostly around the waitset)
  * Deterministic ordering (fair scheduling)
  * Work with callback groups
  * Decouple waitables

* Message generation

  * Catch-up message generation for languages not support out-of-the-box
  * Mangle field names in message to avoid language specific keywords
  * Improve generator performance by running them in the same Python interpreter

* Launch

  * Support use case of using ``xacro`` to perform substitutions before passing the result containing parameters
  * Use pytest for launch testing
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

  * a la ROS 1's message traits: http://wiki.ros.org/roscpp/Overview/MessagesSerializationAndAdaptingTypes
  * Requires knowledge of the typesupport system

* Expand on real-time safety [\*\*\*]

  * With FastRTPS
  * For services, clients, and parameters
  * Expose more quality of service parameters related to real-time performance
  * Real-time-safe intra-process messaging

* Multi-robot supporting features and demos [\*\*\*]

  * Undesired that all nodes across all robots share the same domain (and discover each other)
  * Design how to “partition” the system

* Implement C client library ``rclc`` [\*\*]
* Support more DDS / RTPS implementations:

  * Connext 6, see `ros2/rmw_connext#375 <https://github.com/ros2/rmw_connext/issues/375>`_
  * Connext dynamic [\*]
  * RTI's micro implementation [\*]

* security improvements:

  * more granularity in security configuration (allow authentication only, authentication and encryption, etc) [\*]
  * integrate DDS-Security logging plugin (unified way to aggregate security events and report them to the users through a ROS interface) [\*\*]
  * key storage security (right now, keys are just stored in the filesystem) [\*\*]
  * more user friendly interface (make it easier to specify security config). Maybe a Qt GUI? This GUI could also assist in distributing keys somehow. [\*\*\*]
  * A way to say "please secure this running system" with some UI that would auto-generate keys and policies for everything that is currently running. [\*\*\*]
  * If there are hardware-specific features for securing keys or accelerating encryption/signing messages, that could be interesting to add to DDS/RTPS implementations that don't use it already. [\*\*\*]

Port of existing ROS 1 functionality
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* Perception metapackage

  * Image pipeline

* MoveIt

  * Moveit Maintainers are tracking: https://discourse.ros.org/t/moveit-maintainer-meeting-recap-july-25th-2018/5504

* RQt

  * convert more plugins [\* each when dependencies are available]

Reducing Technical Debt
~~~~~~~~~~~~~~~~~~~~~~~

* Extend testing and resolve bugs in the current code base

  * Waitset inconsistency
  * Multi-threading problems with components

* Fix flaky tests.
* Ability to run (all) unit tests with tools e.g. valgrind
* API review, specifically user facing API in rclcpp and rclpy
* Refactor the rclcpp API into separate packages focused on a single aspect, rclcpp should afterward still provide the combined user facing API
* Revisit message allocators, consider using std::polymorphic_allocator to address problems

* Modernization

  * Support/use exporting CMake targets (rather than using CMake variables like ``*_INCLUDE_DIRS``, ``*_LIBRARIES``)
  * Use C++17 filesystem features rather than custom code
  * Use pybind11 for rclpy
  * Move to f-strings in Python code
  * Use setup.cfg files for Python packages

* Synchronize / reconcile design docs with the implementation.

  * Pre-release retrospective review (APIs, docs, etc.)

* Address / classify pending tickets
* Address TODOs in code / docs
