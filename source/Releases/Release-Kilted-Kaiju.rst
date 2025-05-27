.. _latest-release:

.. _kilted-release:

Kilted Kaiju (codename 'kilted'; May, 2025)
===========================================

.. toctree::
   :hidden:

   Kilted-Kaiju-Complete-Changelog

.. contents:: Table of Contents
   :depth: 2
   :local:

*Kilted Kaiju* is the eleventh release of ROS 2.
What follows is highlights of the important changes and features in Kilted Kaiju since the last release.
For a list of all of the changes since Jazzy, see the :doc:`long form changelog <Kilted-Kaiju-Complete-Changelog>`

Supported Platforms
-------------------

Kilted Kaiju is primarily supported on the following platforms:

Tier 1 platforms:

* Ubuntu 24.04 (Noble): ``amd64`` and ``arm64``
* Windows 10 (Visual Studio 2019): ``amd64``

Tier 2 platforms:

* RHEL 9: ``amd64``

Tier 3 platforms:

* macOS: ``amd64``
* Debian Bookworm: ``amd64``

For more information about RMW implementations, compiler / interpreter versions, and system dependency versions see `REP 2000 <https://www.ros.org/reps/rep-2000.html>`__.

Installation
------------

`Install Kilted Kaiju <../../kilted/Installation.html>`__

Supported Gazebo Release
------------------------
For Kilted Kaiju, the recommended Gazebo release is `Ionic <https://gazebosim.org/docs/ionic/ros_installation>`__.

New features in this ROS 2 release
----------------------------------

``ament_cmake_ros``
^^^^^^^^^^^^^^^^^^^

Add rmw_test_fixture for supporting RMW-isolated testing
""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Included two new packages which provide an extensible mechanism for creating a test fixture for RMW-based communication isolation.
It is modeled closely after the rmw and rmw_implementation API.

The ``rmw_test_fixture`` package currently provides only the API, which could be implemented by an RMW provider for configuring their RMW for a test to run.

The ``rmw_test_fixture_implementation`` package provides the entry point for discovering, loading, and invoking the appropriate extension.

See https://github.com/ros2/ament_cmake_ros/pull/21 for more details.

``common_interfaces``
^^^^^^^^^^^^^^^^^^^^^

New nav_msgs/Goals message
""""""""""""""""""""""""""

A new message type, {interface(nav_msgs/msg/Goals)}, has been introduced to support an array of navigation goals within the nav_msgs package.

See https://github.com/ros2/common_interfaces/pull/269 for more details.

``ros2cli``
^^^^^^^^^^^

Action introspection
""""""""""""""""""""

This allows to instrospect an action with the command line.
Using ``ros2cli`` tools: ``ros2 action echo <action name>``.

See https://github.com/ros2/ros2cli/pull/978 for more information.
Visit :ref:`ros2 action echo <understanding-actions-ros2-action-echo>` and :doc:`Action Introspection <../Tutorials/Demos/Action-Introspection>` to learn more about this feature.

``rclcpp``
^^^^^^^^^^

Action generic client
"""""""""""""""""""""

Support action generic client, this is used to support actions in rosbag2.

See https://github.com/ros2/rclcpp/pull/2759 for more details.

``rclpy``
^^^^^^^^^

Static Type Checking
""""""""""""""""""""

Added static type hints to ``ActionClient`` and ``ActionServer``.

See https://github.com/ros2/rclpy/pull/1349 for more details.

Add support for `generics <https://typing.python.org/en/latest/reference/generics.html>`_ in ``pub/sub/client/server/actions``, ``Future/Task``, and ``Parameter``.

``Publisher``, ``Subscription``, ``Server``, ``Task``, and ``Parameter`` should need no updates to add support for generics.

``Client`` will need to be updated to resemble the following to get the improved type checking.

.. code-block:: python

    self._get_parameter_client: Client[GetParameters.Request,
                                       GetParameters.Response] = self.node.create_client(
                                        GetParameters, '/get_parameters',
                                        qos_profile=qos_profile, callback_group=callback_group)

``ActionClient`` will need to be updated to resemble the following to get the improved type checking.

.. code-block:: python

    ac: ActionClient[Fibonacci.Goal,
                     Fibonacci.Result,
                     Fibonacci.Feedback] = ActionClient(self.node, Fibonacci, 'fibonacci')

``Future`` will need to be updated to resemble the following to get the improved type checking.

.. code-block:: python

    log_msgs_future: Future[bool] = Future()

See https://github.com/ros2/rclpy/pull/1239, https://github.com/ros2/rclpy/pull/1275, https://github.com/ros2/rclpy/pull/1246, and https://github.com/ros2/rclpy/pull/1254/files for more details.

Various other small improvements and corrections have also been made throughout all of ``rclpy``.

Python types can be statically checked using `ament_mypy <https://github.com/ament/ament_lint/tree/kilted/ament_mypy>`_ which wraps `mypy <https://www.mypy-lang.org/>`_.

EventsExecutor
""""""""""""""

Support an experimental events executor for ``rclpy``, which is a port of the original ``rclcpp`` events executor concept.

See https://github.com/ros2/rclpy/pull/1391 for more details.

``Rosbag2``
^^^^^^^^^^^

Action introspection Rosbag2 support
""""""""""""""""""""""""""""""""""""

Allow to record and play actions from a rosbag.

See https://github.com/ros2/rosbag2/pull/1955 for more information.
Design document https://github.com/ros2/rosbag2/pull/1928.
Visit :ref:`Managing Action Data <record-play-data-action>` to learn more about this feature.

Progress bar for ``ros2 bag play``
""""""""""""""""""""""""""""""""""

Added a progress bar for ``ros2 bag play`` CLI, showing the bag time and duration, similar to
what is seen in ROS 1.

See https://github.com/ros2/rosbag2/pull/1836 for more details.

Added support for replaying multiple bags with ``ros2 bag play`` CLI
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

To replay multiple bags, use the new ``-i, --input`` CLI option:

.. code-block:: console

    $ ros2 bag play -i bag1 -i bag2 -i bag3 [storage_id]

See https://github.com/ros2/rosbag2/pull/1848 for more information.

Added support for replaying messages chronologically based on their publication timestamp
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

This is exposed through ``ros2 bag play`` with a new ``--message-order {received,sent}`` option.
The default behavior is to play messages in the order they were received.

See https://github.com/ros2/rosbag2/pull/1876 for more information.

Make snapshot writing into a new file each time it is triggered
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

See https://github.com/ros2/rosbag2/pull/1842 for more details.

New ``--sort`` CLI option in the ``ros2 bag info`` command
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

With new ``--sort`` CLI option user will be able to sort topics, services and actions by name, topic type or number of recorded messages.

See https://github.com/ros2/rosbag2/pull/1804 for more details.

Show size contribution of each topic with ``ros2 bag info``
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

With new ``--size-contribution`` option together with ``ros2 bag info -v`` user will be able to see the size contribution of each topic in the bag file.

See https://github.com/ros2/rosbag2/pull/1726 for more information.

Added ``--log-level`` option to ``ros2 bag play`` and ``ros2 bag record`` to allow printing debug messages
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

See https://github.com/ros2/rosbag2/pull/1625 for more details.

``rosidl_rust``
^^^^^^^^^^^^^^^

Added ``rosidl_rust``
"""""""""""""""""""""

A Rust idl generator was added to the list of default code generators.

See https://github.com/ros2/ros2/pull/1674 for more details.

``ros2``
^^^^^^^^

Switch to using Pixi/Conda for Windows
""""""""""""""""""""""""""""""""""""""

This allows to easily manage dependencies, and to update them in the future.
The installation process is significantly simplified.
Instead of dozens of steps to install dependencies, it is just a couple of commands.
It is much easier to update dependencies.
The dependencies are installed in individual workspaces, with no “global” installation.

See https://github.com/ros2/ci/pull/802 and https://github.com/ros2/ros2/pull/1642 for more details.
Visit :doc:`Windows source install instructions <../Installation/Alternatives/Windows-Development-Setup>` to install it on Windows.

Support topic instances in DDS topics
"""""""""""""""""""""""""""""""""""""

Topic instances are a way of multiplexing the transmission of updates of several objects of the same logical kind over the same resource, i.e. the topic.

See https://github.com/ros2/ros2/issues/1538 for more information.
You can also check the documentation: https://github.com/ros2/design/pull/340/files.

Changes since the Jazzy release
-------------------------------

``common_interfaces``
^^^^^^^^^^^^^^^^^^^^^

Added NV12 to pixel formats
"""""""""""""""""""""""""""

Added NV12 to pixel formats, which is a common output format of hardware-accelerated decoders.

See https://github.com/ros2/common_interfaces/pull/253 for more details.

``rclcpp``
^^^^^^^^^^

Consistent behavior for Subordinate nodes
"""""""""""""""""""""""""""""""""""""""""

Inconsistent behavior of subordinate nodes was fixed.
The subordinate node is a secondary node associated with a primary node, that shares the same underlying context and resources while maintaining a separate name and namespace.
The behavioral modification may affect existing applications relying on the previous implementation:

1. Generic clients created from a subordinate node now correctly respect the subordinate node's sub-namespace
2. Parameters obtained using a subordinate node now correctly use the (parent) node's ``rclcpp::node_interfaces::NodeParametersInterface``

See https://github.com/ros2/rclcpp/pull/2822 for more details.

``rmw_connextdds_cpp``
^^^^^^^^^^^^^^^^^^^^^^

Version bumped to 7.3
"""""""""""""""""""""

The RTI Connext DDS version was bumped to 7.3.0.

See https://github.com/ros2/ci/pull/811 for more details.

``Connextmicro``
^^^^^^^^^^^^^^^^

deprecated Connextmicro
"""""""""""""""""""""""

The RTI Connext Micro RMW package, ``rmw_connextddsmicro``, is going to stop receiving updates in Kilted Kaiju, and be removed in a future ROS 2 release.

See https://github.com/ros2/rmw_connextdds/pull/182 for more information.

``rosidl_dynamic_typesupport``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Removing support for float128
"""""""""""""""""""""""""""""

Removed support for float128 because there are inconsistencies in the definition.

See https://github.com/ros2/rosidl_dynamic_typesupport/issues/11 for more details.

``rmw_fastrtps_cpp``
^^^^^^^^^^^^^^^^^^^^

Renaming package from fastrtps to fastdds
"""""""""""""""""""""""""""""""""""""""""

``fastrtps`` was renamed to ``fastdds``.
The names of the rmw implementations stay the same.
XML Profile ENV strings will change.

See https://github.com/ros2/ros2/pull/1641 for more details.

ament_target_dependencies is deprecated
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The CMake macro ``ament_target_dependencies()`` has been deprecated in favor of ``target_link_libraries()`` with modern CMake targets.
The macro still works, but it emits a CMake deprecation warning at build time like this:

.. code-block::

    CMake Deprecation Warning at [...]/ament_cmake_target_dependencies/share/ament_cmake_target_dependencies/cmake/ament_target_dependencies.cmake:89 (message):
    ament_target_dependencies() is deprecated.  Use target_link_libraries()
    with modern CMake targets instead.  Try replacing this call with:

        target_link_libraries([...] PUBLIC
        [...]
        )

Try replacing the ``ament_target_dependencies()`` call with the  ``target_link_libraries()`` call suggested by the warning.
For more information see `ament/ament_cmake#572 <https://github.com/ament/ament_cmake/pull/572>`__ and `ament/ament_cmake#292 <https://github.com/ament/ament_cmake/issues/292>`__.

Note that the deprecation warning suggests a call to ``target_link_libraries`` with a scope keyword like ``PUBLIC``, ``PRIVATE``, or ``INTERFACE``.
If you have already used ``target_link_libraries()`` on this target without a scope keyword, then you must remove the scope keyword from the suggested call.
This can happen when using some ``ament_cmake`` macros, like ``ament_add_gtest``, which use the plain version of ``target_link_libraries`` internally.
You will know if you need to remove the keyword because CMake will emit an error saying:

.. code-block::

   All uses of target_link_libraries with a target must be either all-keyword or all-plain.

For more information, see `ament/ament_cmake#580 <https://github.com/ament/ament_cmake/issues/580>`__.


``launch``
^^^^^^^^^^

``PathJoinSubstitution``
""""""""""""""""""""""""

``PathJoinSubstitution`` now supports concatenating strings or substitutions into a single path component.
For example:

.. code-block:: python

    PathJoinSubstitution(['robot_description', 'urdf', [LaunchConfiguration('model'), '.xacro']])

If the ``model`` launch configuration was set to ``my_model``, this would result in a path equal to:

.. code-block:: python

    'robot_description/urdf/my_model.xacro'

For more information, see `ros2/launch#835 <https://github.com/ros2/launch/issues/835>`__ and `ros2/launch#838 <https://github.com/ros2/launch/pull/838>`__.

``rmw_zenoh_cpp``
^^^^^^^^^^^^^^^^^

``Tier 1``
""""""""""

The ``rmw_zenoh_cpp`` is now considered Tier 1.
There are many PRs (summarized in `ros2/rmw_zenoh#265 <https://github.com/ros2/rmw_zenoh/issues/265>`__) in the ROS 2 core packages, such as:

  * Make the rmw pass all core tests.
  * Implement and document security
  * Make it work in the Tier 1 platforms.
  * Added Quality declarations
  * Added to REP 2005
  * A dedicated nightly CI job
  * Among others

For more information see https://github.com/ros2/rmw_zenoh/issues/265.

Development progress
--------------------

For progress on the development of Kiltled Kaiju, see `this project board <https://github.com/orgs/ros2/projects/63>`__.

For the broad process followed by Kilted Kaiju, see the :doc:`process description page <Release-Process>`.

Release Timeline
----------------

    December, 2024 - Platform decisions
        REP 2000 is updated with the target platforms and major dependency versions.

    Mon. April 7, 2025 - Alpha + RMW freeze
        Preliminary testing and stabilization of ROS Base [1]_ packages, and API and feature freeze for RMW provider packages.

    Mon. April 14, 2025 - Freeze
        API and feature freeze for ROS Base [1]_ packages in Rolling Ridley.
        Only bug fix releases should be made after this point.
        New packages can be released independently.

    Mon. April 21, 2025 - Branch
        Branch from Rolling Ridley.
        ``rosdistro`` is reopened for Rolling PRs for ROS Base [1]_ packages.
        Kilted development shifts from ``ros-rolling-*`` packages to ``ros-kilted-*`` packages.

    Mon. April 28, 2025 - Beta
        Updated releases of ROS Desktop [2]_ packages available.
        Call for general testing.

    Thu, May 1, 2025 - Kick off of Tutorial Party
        Tutorials hosted at https://github.com/ros2/kilted_tutorial_party are open for community testing.

    Mon. May 12, 2025 - Release Candidate
        Release Candidate packages are built.
        Updated releases of ROS Desktop [2]_ packages available.

    Mon. May 19, 2025 - Distro Freeze
        Freeze all Kilted branches on all `ROS 2 desktop packages <https://www.ros.org/reps/rep-2001.html#kilted-kaiju-may-2025-november-2026>`__ and ``rosdistro``.
        No pull requests for any ``kilted`` branch or targeting ``kilted/distribution.yaml`` in ``rosdistro`` repo will be merged.

    Fri. May 23, 2025 - General Availability
        Release announcement.
        `ROS 2 desktop packages <https://www.ros.org/reps/rep-2001.html#kilted-kaiju-may-2025-november-2026>`__ source freeze is lifted and ``rosdistro`` is reopened for Kilted pull requests.

.. [1] The ``ros_base`` variant is described in `REP 2001 (ros-base) <https://www.ros.org/reps/rep-2001.html#ros-base>`_.
.. [2] The ``desktop`` variant is described in `REP 2001 (desktop-variants) <https://www.ros.org/reps/rep-2001.html#desktop-variants>`_.
