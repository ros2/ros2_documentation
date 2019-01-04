
Roadmap
=======

.. contents:: Table of Contents
   :depth: 2
   :local:

For more information on the design of ROS 2 please see `design.ros2.org <http://design.ros2.org>`__.
The core code for ROS 2 is on the `ros2 github organization <https://github.com/ros2>`__.
The Discourse forum/mailing list for discussing ROS 2 design is `ng-ros <https://discourse.ros.org/c/ng-ros>`__.
Questions should be asked on `ROS answers <https://answers.ros.org>`__\ , make sure to include at least the ``ros2`` tag and the rosdistro version you are running, e.g. ``ardent``.

Planned upcoming releases
-------------------------

This is a list of the features targeted for development in the future.

*Subject to change.*

Next release - D-turtle (June 2019)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The `D-turtle meta ticket <https://github.com/ros2/ros2/issues/607>`__ on GitHub will in the near future enumerate the ongoing high level tasks as well as references specific tickets with more details.

Future (in no specific order)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Design / Concept
~~~~~~~~~~~~~~~~

* Support for non-ASCII strings in messages / services
* Progress on migration plan
* Reconsider 1-to-1 mapping of ROS nodes to DDS participants
* optional XML or YAML frontend for Python-based launch

Infrastructure and tools
~~~~~~~~~~~~~~~~~~~~~~~~

* Building

  * Support to generate "fat" packages / archives
  * Windows and Mac OS packages

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


* Actions in Python

* Logging improvements [\* / \*\*]

  * Configuration specified in a file
  * C++ stream operators
  * Colorize console output

* Parameters

  * set individual parameters via command line arguments (instead of passing a yaml file)
  * Specify the value range
  * Define read-only parameters

* Additional Graph API features [\*\* / \*\*\*]

  * a la ROS 1 Master API: http://wiki.ros.org/ROS/Master_API
  * Event-based notification
  * Requires knowledge of the rmw interface which needs to be extended

* Remapping [\*\* / \*\*\*]

  * Dynamic remapping and aliasing through a Service interface

* Type masquerading [\*\*\*]

  * a la ROS 1's message traits: http://wiki.ros.org/roscpp/Overview/MessagesSerializationAndAdaptingTypes
  * Requires knowledge of the typesupport system

* Expand on real-time safety [\*\*\*]

  * With FastRTPS
  * For services, clients, and parameters
  * Support deterministic ordering of executables in Executor (fair scheduling)
  * Expose more quality of service parameters related to real-time performance
  * Real-time-safe intra-process messaging

* Multi-robot supporting features and demos [\*\*\*]

  * Undesired that all nodes across all robots share the same domain (and discover each other)
  * Design how to “partition” the system

* Implement C client library ``rclc`` [\*\*]
* Support more DDS / RTPS implementations:

  * Connext dynamic [\*]
  * RTI's micro implementation [\*]
  * Eclipse Cyclone DDS (former ADLINK OpenSplice) [\*]

* security improvements:

  * more granularity in security configuration (allow authentication only, authentication and encryption, etc) [\*]
  * extend access control permission generation to support services [\*]
  * integrate DDS-Security logging plugin (unified way to aggregate security events and report them to the users through a ROS interface) [\*\*]
  * key storage security (right now, keys are just stored in the filesystem) [\*\*]
  * more user friendly interface (make it easier to specify security config). Maybe a Qt GUI? This GUI could also assist in distributing keys somehow. [\*\*\*]
  * A way to say "please secure this running system" with some UI that would auto-generate keys and policies for everything that is currently running. [\*\*\*]
  * If there are hardware-specific features for securing keys or accelerating encryption/signing messages, that could be interesting to add to DDS/RTPS implementations that don't use it already. [\*\*\*]

Port of existing ROS 1 functionality
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* Perception metapackage

  * Image pipeline
  * Improvements to the intra process comm. to reduce latency / overhead

* MoveIt

  * Needs Actions
  * Moveit Maintainers are tracking: https://discourse.ros.org/t/moveit-maintainer-meeting-recap-july-25th-2018/5504

* RQt

  * convert more plugins [\* each when dependencies are available]

* Diagnostics

Reducing Technical Debt
~~~~~~~~~~~~~~~~~~~~~~~

* Extend testing and resolve bugs in the current code base

  * Waitset inconsistency
  * Multi-threading problems with components
  * Reduce overhead / latency of intra-process communication

* Fix flaky tests.
* Ability to run (all) unit tests with tools e.g. valgrind
* API review
* Synchronize / reconcile design docs with the implementation.

  * Pre-release retrospective review (APIs, docs, etc.)

* Address / classify pending tickets
* Address TODOs in code / docs

Past releases
-------------

See `list of releases <Releases>`.
