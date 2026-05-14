.. _upcoming-release:

.. _lyrical-release:

Lyrical Luth (codename 'lyrical'; May, 2026)
============================================

.. toctree::
   :hidden:

   Lyrical-Luth-Complete-Changelog

.. contents:: Table of Contents
   :depth: 2
   :local:

*Lyrical Luth* is the twelfth release of ROS 2.

Release Timeline
----------------

**As soon as possible** - Migrate ROS Rolling to ROS Lyrical's target platforms
    * RHEL 10 + Ubuntu 26.04: Migrate as soon as core packages successfully build on both platforms.
    * Windows 11: Migrate as soon as we have a green build

**Mon. April 13, 2026** - Alpha + RMW freeze (*Delayed; was originally April 6th*)
    * Preliminary testing of ROS Base packages
    * API and feature freeze for RMW provider packages.

**Mon. April 20, 2026** - Freeze (*Delayed; was originally April 13th*)
    * API and feature freeze for ROS Base packages in Rolling Ridley.
    * Only bug fix releases should be made after this point.
    * New packages can be released.

**Mon. April 21, 2026** - Branch (*Delayed; was originally April 20th*)
    * Branch from Rolling Ridley
    * ``rosdistro`` is reopened for Rolling PRs for ROS Base packages.
    * Lyrical development shifts from ``ros-rolling-*`` packages to ``ros-lyrical-*`` packages.

**Mon. April 27, 2026** - Beta
    * Updated releases of ROS Desktop packages available.
    * Call for general testing.

**Thu, April 30, 2026** - Kick off Tutorial Party
    * Open up tutorials for community testing.

**Mon. May 11, 2026** - Release Candidate
    * Build release candidate packages up to ROS Desktop

**Mon. May 18, 2026** - Distro Freeze
    * Freeze all Lyrical branches on all ROS desktop packages
    * No pull requests for any Lyrical branch or targeting ``lyrical/distribution.yaml`` in ``rosdistro`` repo will be merged.

**Friday May 22nd, 2026** - General Availability
    * Release announcement.
    * ROS desktop packages source freeze is lifted and ``rosdistro`` is reopened for Lyrical pull requests.

For progress on the development of Lyrical Luth, see `this project board <https://github.com/orgs/ros2/projects/70>`__.
For the broad process followed by Lyrical Luth, see the :doc:`process description page <Release-Process>`.

Supported Platforms
-------------------

ROS Lyrical supports the following platforms according to `the platform support tiers <../The-ROS2-Project/Platform-Support-Tiers>`:

+--------------+-------------------+-------------------+---------------+-------------------+-----------+-----------------+----------------+
| Architecture | Ubuntu Resolute   | Ubuntu Noble*     | Windows 11    | RHEL 10           | macOS     | Debian Trixie*  | OpenEmbedded / |
|              | (26.04)           | (24.04)           | (VS2022)      |                   |           | (13)            | Yocto Project  |
+==============+===================+===================+===============+===================+===========+=================+================+
| amd64        | Tier 1 [d][a]     | Tier 3            | Tier 1 [a]    | Tier 2 [d][a]     | Tier 3    | Tier 3          | Tier 3         |
+--------------+-------------------+-------------------+---------------+-------------------+-----------+-----------------+----------------+
| arm64        | Tier 1 [d][a]     | Tier 3            |               |                   |           | Tier 3          | Tier 3         |
+--------------+-------------------+-------------------+---------------+-------------------+-----------+-----------------+----------------+
| arm32        | Tier 3            | Tier 3            |               |                   |           | Tier 3          | Tier 3         |
+--------------+-------------------+-------------------+---------------+-------------------+-----------+-----------------+----------------+

* ``*`` Early EOL per `the platform EOL policy <../The-ROS2-Project/Platform-EOL-Policy>`
    * Ubuntu Noble is supported until ``2029-06-01``
    * Debian Trixie is supported until ``2028-08-09``
* ``[d]`` You may install ROS Lyrical on this platform using Distribution-specific packaegs (Debian, RPM, etc.).
* ``[a]`` You may install ROS Lyrical by downloading an archive containing pre-built packages for all packages in the `ROS Lyrical ros2.repos file <https://github.com/ros2/ros2/blob/lyrical/ros2.repos>`__

To use ROS Lyrical on any Tier 3 platform, you must build ROS Lyrical from source.

Minimum Language Requirements
-----------------------------

* `C++20 <https://discourse.openrobotics.org/t/ros-2-lyrical-c-version/52551>`__
* C17
* Python 3.12 - 3.14

Dependency Requirements
-----------------------

TODO - this section will show a table of important system package versions across supported platforms.

Middleware Implementation support
---------------------------------

The default middleware in ROS Lyrical is **rmw_fastrtps_cpp**.

+---------------------------+-------------------------+---------------+----------------------------+-------------------------------+
| Middleware Library        | Middleware Provider     | Support Level | Platforms                  | Architectures                 |
+===========================+=========================+===============+============================+===============================+
| rmw_fastrtps_cpp          | eProsima Fast-DDS       | Tier 1        | All Platforms              | All Architectures             |
+---------------------------+-------------------------+---------------+----------------------------+-------------------------------+
| rmw_connextdds            | RTI Connext             | Tier 1        | Ubuntu, Windows, and macOS | All Architectures except arm64|
+---------------------------+-------------------------+---------------+----------------------------+-------------------------------+
| rmw_cyclonedds_cpp        | Eclipse Cyclone DDS     | Tier 1        | All Platforms              | All Architectures             |
+---------------------------+-------------------------+---------------+----------------------------+-------------------------------+
| rmw_zenoh_cpp             | Eclipse Zenoh           | Tier 1        | All Platforms              | All Architectures             |
+---------------------------+-------------------------+---------------+----------------------------+-------------------------------+
| rmw_fastrtps_dynamic_cpp  | eProsima Fast-DDS       | Tier 2        | All Platforms              | All Architectures             |
+---------------------------+-------------------------+---------------+----------------------------+-------------------------------+
| rmw_gurumdds_cpp          | GurumNetworks GurumDDS  | Tier 3        | Ubuntu and Windows         | All Architectures except arm32|
+---------------------------+-------------------------+---------------+----------------------------+-------------------------------+

Middleware implementation support is dependent upon the platform support tier.
For example, a Tier 1 middleware implementation on a Tier 2 platform will only receive Tier 2 support.

Installation
------------

TODO

New Features in Lyrical
-----------------------

This section highlights some of the new features and changes in ROS Lyrical.
For all changes, see the :doc:`full ROS Lyrical changelog <Lyrical-Luth-Complete-Changelog>`.

Callback Group Events executor (``rclcpp``)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Looking for better executor performance?
Check out the new Callback Group Events Executor.
Like its predecessor the ``EventsExecutor``, the ``EventsCBGExecutor`` uses an events queue to process ready entities.
However, ``EventsCBGExecutor`` adds support for multiple sources of ROS time and multiple threads.
Compared to the Single and Multithreaded executors, the ``EventsCBGExecutor`` uses 10% to 15% less CPU.

Try it out by instantiating ``rclcpp::executors::EventsCBGExecutor``;

.. code-block:: c++

    #include <rclcpp/rclcpp.hpp>

    // ... class MyNode ...

    int main(int argc, char ** argv)
    {
      rclcpp::init(argc, argv);
      auto node = std::make_shared<MyNode>();
      rclcpp::executors::EventsCBGExecutor executor;
      executor.add_node(node);
      executor.spin();
      rclcpp::shutdown();
      return 0;
    }

Using composable nodes?
Launch a component container with the ``EventsCBGExecutor`` using the new ``--executor-type`` argument.

.. code-block:: console

    ros2 run rclcpp_components component_container --executor-type events-cbg

.. code-block:: xml

    <?xml version="1.0" encoding="UTF-8"?>
    <launch>
      <node_container pkg="rclcpp_components" exec="component_container" name="my_node_container" namespace="" args="--executor-type events-cbg">
        <!-- Your composable nodes here -->
      </node_container>
    </launch>

For more info, see `ros2/rclcpp#3097 <https://github.com/ros2/rclcpp/pull/3097>`_, `ros2/rclcpp#3134 <https://github.com/ros2/rclcpp/pull/3134>`_, and `ros2/rclcpp#3137 <https://github.com/ros2/rclcpp/pull/3137>`_.

Parameter range descriptors check bounds for integer and double arrays (``rclcpp``)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Do your nodes have integer or double arrays?
Do you need to constrain the values in those arrays?
``rclcpp`` nodes now validate range constraints on these arrays.
Using the code below, the node will only allow setting ``my_integer_array`` to a list containing even integers between 2 and 10 (inclusive).

.. code-block:: c++

    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.integer_range.resize(1);
    auto & integer_range = descriptor.integer_range.at(0);
    integer_range.from_value = 2;
    integer_range.to_value = 10;
    integer_range.step = 2;
    node->declare_parameter("my_integer_array", std::vector<int64_t>{2, 4, 6, 8, 10}, descriptor);

See `ros2/rclcpp#2828 <https://github.com/ros2/rclcpp/pull/2828>`_ for more info.

``AsyncNode`` lets you use ``asyncio`` (``rclpy``)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Want to use ``asyncio`` and ``rclpy`` at the same time?
Check out the new ``AsyncNode`` class.
This node runs an ``asyncio`` event loop.
Call ``await`` on any ``asyncio`` operation from any subscription, service, and timer callback.
Try ``await client.call(request)`` to wait for service calls, and the sim-time aware ``await clock.sleep(...)``.
This class uses significantly less CPU compared to the default ``SingleThreadedExecutor``.

.. code-block:: python

    import asyncio
    import rclpy
    from rclpy.experimental import AsyncNode

    class HelloWorldNode(AsyncNode):
        def __init__(self):
            super().__init__('hello_world_node')
            self._timer = self.create_timer(5.0, self._cb)

        async def _cb(self):
            self.get_logger().info('Hello')
            await self.get_clock().sleep(1.0)
            self.get_logger().info('World!')

    async def _main():
        with rclpy.init():
            await HelloWorldNode().run()

    if __name__ == '__main__':
        asyncio.run(_main())

See the :doc:`Writing an async node with asyncio <../Tutorials/Intermediate/Writing-An-Async-Node-With-Asyncio-Python>` tutorial and `ros2/rclpy#1620 <https://github.com/ros2/rclpy/pull/1620>`_ for more details.

Publish messages without copying data using ``rosidl::Buffer``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Are you publishing data on ROS topics, but using the data elsewhere, like a GPU?
Tired of copying data out of the GPU before publishing just to copy it back into the GPU in the subscriber?
``rosidl::Buffer`` publish and subscribe ROS messages without moving the data from elsewhere.

All ``uint8[]`` fields now have the type ``rosidl::Buffer<uint8_t>`` in C++ instead of  ``std::vector<uint8_t>``.
Define your ROS messages with ``uint8[]`` fields and install an appropriate ``rosidl::BufferBackend`` implementation.
Note that only publishers and subscribers using ``rmw_fastrtps_cpp`` may use this feature for now, but `support in Zenoh is comming <https://github.com/ros2/rmw_zenoh/pull/930>`_.

Using a custom hardware accelerator or machine learning library?
You can benefit from this too.
See :doc:`../Tutorials/Advanced/Writing-a-Buffer-Backend` to learn how to implement your own ``rosidl::BufferBackend``.

See :doc:`../Concepts/Intermediate/About-Buffer-Backends` and :doc:`../How-To-Guides/Using-Buffer-Backends` for more details.

Use YAML tags in parameter files
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Tired of ``rcl`` interpretting ambiguous YAML parameter values as the wrong type?
Prevent that by specifying the type using YAML tags.

.. code-block:: yaml

  my_node:
    ros__parameters:
      string_param: !!str true
      bool_param: !!bool yes
      int_param: !!int 0
      float_param: !!float 10
      seq_param: !!seq [10, 0, -10]
      map_param: !!map {str: string, bool: true, int: 10, float: 1.1}

See `ros2/rcl#1275 <https://github.com/ros2/rcl/pull/1275>`_ for more info.

* https://github.com/ros2/rcl/commit/b7d6d69e670aa97bf69a6b92d12321ed31e68a4c

More logging options in launch files
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Have different kinds of messages in your log files that you want to log with different severity?
Specify the log level using the the new ``level`` argument on the ``log`` action.
Alternatively, use the new ``log_debug``, ``log_info``, ``log_warning``, or ``log_error`` actions.

.. code-block:: xml

    <?xml version="1.0" encoding="UTF-8"?>
    <launch>
      <log level="INFO" message="Hello world! (log level=INFO)" />
      <log_debug message="Hello world debug!" />
      <log_info message="Hello world!" />
      <log_warning message="Hello world warning!" />
      <log_error message="Hello world error!" />
    </launch>


For more info see `ros2/launch#866 <https://github.com/ros2/launch/pull/866>`_

New substitutions in XML and YAML launch files
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Use XML or YAML launch files?
:doc:`Substitutions <../Tutorials/Intermediate/Launch/Using-Substitutions>` make your launch files evaluate variables at launch time.
Launch frontends (the things that make it possible to use XML and YAML launch files) may now use ``string-join`` and ``path-join`` substitutions.

.. code-block:: xml

    <?xml version="1.0" encoding="UTF-8"?>
    <launch>
      <log_info message="Check out $(string-join . https://docs ros org)"/>
      <log_info message="Don't forget to source /$(path-join opt ros lyrical $(string-join . setup bash))"/>
    </launch>


See `ros2/launch#857 <https://github.com/ros2/launch/pull/857>`_  and `ros2/launch#943 <https://github.com/ros2/launch/pull/943>`_ for more info.

Choose ROS logging backend at runtime
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Have you ever needed to use ROS with another framework that has its own logging system?
ROS supports replacing its logging backend, but previously you had to rebuild ``rcl`` from source change it.
Now you can change the logging implementation at runtime!
Set the `RCL_LOGGING_IMPLEMENTATION` environment variable to switch between logging backends.
Valid values are:

* ``rcl_logging_spdlog``
* ``rcl_loggin_noop``
* or your own custom logging implementation!

If not specified, ROS uses ``rcl_logging_spdlog`` by default.

See `ros2/rcl#1178 <https://github.com/ros2/rcl/issues/1178>`_, `ros2/rcl#1276 <https://github.com/ros2/rcl/pull/1276>`_, and `ros2/rcl_logging#135 <https://github.com/ros2/rcl_logging/pull/135>`_ for more details.

Control bag recording remotely using ROS services
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Want to remotely control bag recording?
Use ``rosbag2``'s new services to:

* start recording ``~/record``
* stop recording ``~/stop``
* start topic discovery ``~/start_discovery``
* stop topic discovery ``~/stop_discovery``
* query discovery state ``~/is_discovery_running``

.. code-block:: bash

    ros2 bag record --all -o /tmp/my_awesome_bag

    # In another terminal, stop the existing recording
    ros2 service call /rosbag2_recorder/stop rosbag2_interfaces/srv/Stop "{}"
    # Start recording again at a new location
    ros2 service call /rosbag2_recorder/record rosbag2_interfaces/srv/Record "{uri: 'file:///tmp/my_awesome_bag_2'}"


See `ros2/rosbag2#2248 <https://github.com/ros2/rosbag2/pull/2248>`__ for more details.

Control bag Playback and Recording using Python
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Want to control bag playback and reporting programmatically?
Previously Python users had to rely on blocking command-line-style helpers.
Python users can now programmatically control playback and recording with APIs for: pause, resume, stop, seek, play-next, spin control, and wait helpers.

Recording example:

.. code-block:: python

    import rclpy
    import rosbag2_py

    with rclpy.init():
        # Configure storage and initialize the recorder
        storage_opts = rosbag2_py.StorageOptions(uri='/tmp/my_awesome_bag', storage_id='mcap')
        record_opts = rosbag2_py.RecordOptions()
        record_opts.all_topics = True

        recorder = rosbag2_py.Recorder(storage_opts, record_opts)

        # Start the ROS node spinner and kick off the recording thread
        recorder.start_spin()
        recorder.record()

        # Pause the recording and verify the current state
        recorder.pause()
        print(recorder.is_paused())

        # Terminate recording and shut down the spinner threads
        recorder.stop()
        recorder.stop_spin()

Playback example:

.. code-block:: python

    import rclpy
    import rosbag2_py

    with rclpy.init():
        # Configure storage and initialize the player
        storage_opts = rosbag2_py.StorageOptions(uri='/tmp/my_awesome_bag', storage_id='mcap')
        play_opts = rosbag2_py.PlayOptions()
        play_opts.start_paused = True

        player = rosbag2_py.Player(storage_opts, play_opts)

        # Start the ROS node spinner and kick off the playback thread
        player.start_spin()
        player.play()

        # Verify playback startup and retrieve bag timing metadata
        print(player.wait_for_playback_to_start(1.0))
        print(player.wait_for_playback_to_start_exclusively(1.0))
        print(player.get_starting_time())
        print(player.get_playback_duration())

        # Control the playback state and step through messages manually
        print(player.is_paused())
        print(player.play_next())
        player.resume()
        player.pause()
        player.seek(0)

        # Await playback completion and terminate the worker threads
        print(player.wait_for_playback_to_finish(1.0))
        print(player.wait_for_playback_to_finish_exclusively(1.0))
        player.stop()
        player.stop_spin()

See `ros2/rosbag2#2047 <https://github.com/ros2/rosbag2/pull/2047>`_, `ros2/rosbag2#2062 <https://github.com/ros2/rosbag2/pull/2062>`_, `ros2/rosbag2#2061 <https://github.com/ros2/rosbag2/pull/2061>`_, and `ros2/rosbag2#2095 <https://github.com/ros2/rosbag2/pull/2095>`_ for more details.

Circular recording by bag split
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Recording data on your robot with limited disk space?
Try the new ``--max-bag-files`` option.
It limits the maximum number of bag files stored on disk by automatically deleting the oldest split files as new ones are created.

.. code-block:: bash

    # Max bag size: 100MB
    ros2 bag record --all --max-bag-size 100000000 --max-bag-files 5

See `ros2/rosbag2#2218 <https://github.com/ros2/rosbag2/pull/2218>`__ for more details.

More descriptive bag split names
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Do you having trouble finding bag splits?
``rosbag2`` now names bag splits so that each is self-descriptive and chronologically traceable.

.. code-block:: text

   {counter}_{prefix}_{timestamp}.{extension}

* **counter**: split index (integer starting from 0, *not zero-padded*)
* **prefix**: derived from the bag directory name, with any default timestamp suffix removed
* **timestamp**: local time at file creation, formatted as ``YYYY_MM_DD-HH_MM_SS``
* **extension**: bag file extension. e.g., ``.mcap``, ``.db3``

See `ros2/rosbag2#2265 <https://github.com/ros2/rosbag2/pull/2265>`__ for more details.

``rosbag2`` message-loss observability
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Say you've built a robust system for recording data on your robot, but there is a problem.
How do you *know* there's a problem?
With ``rosbag2``'s new message-loss observability of course!

``rosbag2`` now collects message-loss statistics from the transport layer and recorder internals.
It publishes incremental per-topic loss events on the ``events/rosbag2_messages_lost`` topic.
Control the publishing rate using ``--stats_max_publishing_rate``.

.. code-block:: bash

    # Publish message loss statistics at most 10 Hz
    ros2 bag record --all --stats_max_publishing_rate 10 -o /tmp/my_awesome_bag
    # If all is going well, you should see no output from this command
    ros2 topic echo /events/rosbag2_messages_lost

See `ros2/rosbag2#2039 <https://github.com/ros2/rosbag2/pull/2039>`__, `ros2/rosbag2#2144 <https://github.com/ros2/rosbag2/pull/2144>`__, and `ros2/rosbag2#2150 <https://github.com/ros2/rosbag2/pull/2150>`__ for more details.


``fish`` shell support
^^^^^^^^^^^^^^^^^^^^^^

Do you enjoy `fish shell <https://fishshell.com/>`_?
Do you want to use it with ROS?
Now you can!
Try out the new ``setup.fish`` script.

.. code-block:: shell

    source /opt/ros/lyrical/setup.fish

See `ros2/ros2cli#1211 <https://github.com/ros2/ros2cli/pull/1211>`_ and `ament/ament_package#164 <https://github.com/ament/ament_package/pull/164>`_ for more info.
To use ``fish`` shell with ``colcon``, check out `@Sunrisepeak's <https://github.com/Sunrisepeak>`_ `colcon-fish package <https://github.com/ros-x/colcon-fish>`_.

``ros2 param get`` a parameter from all nodes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Are you using simulated time?
How do you know if all of your nodes are using simulated time?
Use ``ros2 param get <parmeter name>`` to get a parameter value from all nodes.

.. code-block:: shell

    ros2 param get use_sim_time


See `ros2/ros2cli#1174 <https://github.com/ros2/ros2cli/pull/1174>`_ for more info.

``ros2 param`` get and set multipe parameters on one node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Want to get and set multiple parameters on one node at the same time?
Use ``ros2 param get <node name> <param1> <param2> ...`` to get multiple values from a single node.

.. code-block:: console

    $ ros2 param get /robot_state_publisher frame_prefix ignore_timestamp publish_frequency
    frame_prefix:
      String value is: 
    ignore_timestamp:
      Boolean value is: False
    publish_frequency:
      Double value is: 20.0

Use ``ros2 param set <node name> <param1> <value1> <param2> <value2> ...`` to set multiple values on a single node.

.. code-block:: console

    $ ros2 param set /robot_state_publisher frame_prefix foo ignore_timestamp True publish_frequency 10.0
    frame_prefix: Set parameter successful
    ignore_timestamp: Set parameter successful
    publish_frequency: Set parameter successful


See `ros2/ros2cli#1203 <https://github.com/ros2/ros2cli/pull/1203>`_ and `ros2/ros2cli#1204 <https://github.com/ros2/ros2cli/pull/1204>`_ for more details.

Actions, Services, and Environment variables in ``ros2 doctor --report``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``ros2 doctor --report`` now includes information about Actions, Services, and ROS-related environment variables.
Include this report in your github issues, or AI prompts, to debug problems faster.

.. code-block:: console

    $ ros2 doctor --report

      ACTION LIST
    action                 : none
    action server count    : 0
    action client count    : 0

      ROS ENVIRONMENT
    ROS environment variables        : ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET, ROS_DISTRO=lyrical
    rcutils environment variables    : 
    rmw environment variables        : 
    # ...

      SERVICE LIST
    service          : /dummy_joint_states/describe_parameters
    service count    : 1
    client count     : 0
    service          : /dummy_joint_states/get_parameter_types
    service count    : 1
    client count     : 0
    # ...


For more information see `ros2/ros2cli#1059 <https://github.com/ros2/ros2cli/pull/1059>`_, `ros2/ros2cli#1076 <https://github.com/ros2/ros2cli/pull/1076>`_, and `ros2/ros2cli#1045 <https://github.com/ros2/ros2cli/pull/1045>`_.

``ros2 service info --verbose``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Are you debugging mismatched QoS settings between ROS clients and ROS services?
Try out the new ``--verbose`` option to ``ros2 service info``.
Like ``ros2 topic info``, this flag outputs detailed information about clients and services to help you troubleshoot issues.

.. code-block:: console

    $ ros2 service info --verbose /robot_state_publisher/list_parameters
    Type: rcl_interfaces/srv/ListParameters
    Clients count: 0
    Services count: 1
    Node name: robot_state_publisher
    Node namespace: /
    Service type: rcl_interfaces/srv/ListParameters
    Service type hash: RIHS01_3e6062bfbb27bfb8730d4cef2558221f51a11646d78e7bb30a1e83afac3aad9d
    Endpoint type: SERVER
    Endpoint count: 2
    GIDs:
    - Request Reader : 01.0f.c8.b6.fe.43.b3.44.00.00.00.00.00.00.0e.04
    - Response Writer : 01.0f.c8.b6.fe.43.b3.44.00.00.00.00.00.00.0f.03
    QoS profiles:
    - Request Reader :
          Reliability: RELIABLE
          History (Depth): KEEP_LAST (1000)
          Durability: VOLATILE
          Lifespan: Infinite
          Deadline: Infinite
          Liveliness: AUTOMATIC
          Liveliness lease duration: Infinite
    - Response Writer :
          Reliability: RELIABLE
          History (Depth): KEEP_LAST (1000)
          Durability: VOLATILE
          Lifespan: Infinite
          Deadline: Infinite
          Liveliness: AUTOMATIC
          Liveliness lease duration: Infinite

See `ros2/ros2cli#916 <https://github.com/ros2/ros2cli/pull/916>` for more info.

``ros2 topic bw`` multiple topics at once
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Trying to figure out which topics are using most of your network bandwidth?
Now you can use ``ros2 topic bw`` with multiple topics at the same time.
Pass multiple topics by name:

.. code-block:: bash

    ros2 topic bw /tf /joint_states

Or pass ``--all`` to watch bandwidth statistics in real time.

.. image:: images/ros2_topic_bw_all.gif

See `ros2/ros2cli#1124 <https://github.com/ros2/ros2cli/pull/1124>`_ and `ros2/ros2cli#1130 <https://github.com/ros2/ros2cli/pull/1130>`_ for more info.

URDF improvements
^^^^^^^^^^^^^^^^^

URDF released a few new features:

* Quaternions
* Capsule geometry
* Accelreation, decelleration, and jerk limits

Add ``version="1.2"`` to your robot description to start using them.

.. code-block:: xml

    <?xml version="1.0" ?>
    <robot name="simple_capsule_arm" version="1.2">

      <link name="link1">
        <visual>
          <origin xyz="0 0 0.25" quat_xyzw="0 0 0 1"/>
          <geometry>
            <capsule radius="0.1" length="0.5"/>
          </geometry>
        </visual>
        <collision>
          <origin xyz="0 0 0.25" quat_xyzw="0 0 0 1"/>
          <geometry>
            <capsule radius="0.1" length="0.5"/>
          </geometry>
        </collision>
      </link>

      <joint name="joint1" type="revolute">
        <!-- ... -->
        <!-- Using quaternion for a 90-degree pitch rotation (y-axis) -->
        <origin xyz="0 0 0.5" quat_xyzw="0 0.7071068 0 0.7071068"/>
        <!-- Demonstrating new and existing limits -->
        <limit lower="-1.57" upper="1.57" effort="100.0" velocity="2.0" acceleration="10.0" deceleration="5.0" jerk="50.0"/>
      </joint>

      <!-- ... -->
    </robot>

See `ros/urdfdom#235 <https://github.com/ros/urdfdom/pull/235>`_, `ros/urdfdom#238 <https://github.com/ros/urdfdom/pull/238>`_, and `ros/urdfdom#212 <https://github.com/ros/urdfdom/pull/212>`_ for more info.

Note that the Robot Model plugin `does not yet support capsule geometry <https://github.com/ros2/rviz/issues/1734>`_.
Please consider opening a pull request for this feature!

``robot_state_publisher`` can read the robot description from a topic
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Most of the time in a ROS system the ``robot_state_publisher`` node does two things:

* It publishes the ``robot_description`` on a topic, and
* It publishes TF transforms given joint positions.

If you have ever tried to add ROS interfaces to a framework with its own internal robot model, you may have wished these were two separate utilies.
Now they can be!
Set the ``use_robot_description_topic`` parameter to ``true`` to make ``robot_state_publisher`` subscribe to the ``robot_description`` topic instead of publishing it.
Then, make the other robot framework publish its own robot description on that topic.

See `ros/robot_state_publisher#234 <https://github.com/ros/robot_state_publisher/pull/234>`_ for more info.

RViz resource retriever service
"""""""""""""""""""""""""""""""

No longer need to install robot_description on the same machine you launch RViz.
Use resource_retriever_service to publish meshes etc from the robot itself

* https://github.com/ros2/rviz/commit/7190270e0c32f531a6d4b12a3623bc0cace3e9e0

Beware the wisdom of (insert discourse post here about robots should be clients)

CMake improvements
^^^^^^^^^^^^^^^^^^

ament_python_install_package
""""""""""""""""""""""""""""

* https://github.com/ament/ament_cmake/commit/abd86f5a12eb085cd975ad9e02ff6b66cc372170

Install a Python package multiple times with the same name; allows rosidl_generate_interfaces and ament_python_install_package in the same package.
Useful for existing packages that combine code and interface files.
Note: new packages should consider message, service, and action definitions live in their own package so that downstream users can use messages without depending on your code

ament_ros_defaults
""""""""""""""""""

* https://github.com/ros2/ament_cmake_ros/commit/6a84b6f31dc047adfe525fa0d01af4eef8652c35

Get default CMake settings for a given ROS distro - C17; c++ 20 etc

Utility Improvements
^^^^^^^^^^^^^^^^^^^^

Get client and server info
""""""""""""""""""""""""""

* https://github.com/ros2/rclcpp/commit/63bdf2add403ac38ff51969acf02919911e89724
* https://github.com/ros2/rcl/commit/4e0829cedd4ff9d50fb8de40f7b351bcfaa2317a

Add examples for all client libraries

rcpputils
"""""""""

New thread naming utilities to aid in debugging
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* https://github.com/ros2/rcpputils/commit/8e29c4c16857656244e7fd9d5450569d0208a0ab

rcutils
"""""""

New rcutils_strnlen
~~~~~~~~~~~~~~~~~~~

* https://github.com/ros2/rcutils/commit/ead8874667205525a2d16f5adb694ab0dd01a325

base64 functions
~~~~~~~~~~~~~~~~

* https://github.com/ros2/rcutils/commit/f32243642b1d95f2dd2ea42ef468ccb377df0e13

rcl changes
"""""""""""

New RCL APIs
~~~~~~~~~~~~

* https://github.com/ros2/rcl/commit/d290ab955ceb57fae6c76f96bdb5b649a5e3f4bd
* https://github.com/ros2/rcl/commit/819c78db03753216dd5b1ac38be96ad811bb6cad
* https://github.com/ros2/rcl/commit/be6ba458057f6a1cb48a5bed007a9be70d986e76
* https://github.com/ros2/rcl/commit/5990da469eff7071913cca793bf7f7b5f7979873
* https://github.com/ros2/rcl/commit/33ad697c5386ffd89cea386a2b530649fdb4e5fd

class_loader arguments to constructors
""""""""""""""""""""""""""""""""""""""

* https://github.com/ros/class_loader/commit/5c279488cb9fd4b168502c5ffdd533fabab20168

Must specialize class_loader::InterfaceTraits to specify constructor arguments

Show loader creating class with arguments; plugins no longer need to be default constructible

``ament_cmake``
^^^^^^^^^^^^^^^

Allow multiple ``ament_python_install_package()`` calls per package
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
This enables shipping a single package with both python code and generated interfaces (from ``rosidl_generate_interfaces()``).
Source directories from each call are merged at install time, with the last call winning on file conflicts.

See https://github.com/ament/ament_cmake/pull/587 for more details.

``ament_mypy``
^^^^^^^^^^^^^^^

Has new ``--ament-strict`` option to allow more strict type checking.
The ROS core is slowly being switch to this stricter standard to help prevent regressions in Python packages.

``class_loader``
^^^^^^^^^^^^^^^^

Add support for passing arguments to constructors.
As a result, users can now create plugins that are not default constructible, removing the need for initialize method.

See https://github.com/ros/class_loader/pull/223 for more details.

``plugin_lib``
^^^^^^^^^^^^^^

Add support for passing arguments to constructors.

See https://github.com/ros/pluginlib/pull/291 for more details.

``image_transport``
^^^^^^^^^^^^^^^^^^^

``image_transport`` now supports lifecycle nodes.

See https://github.com/ros-perception/image_common/pull/352 for more details.

``point_cloud_transport``
^^^^^^^^^^^^^^^^^^^^^^^^^

``point_cloud_transport`` now supports lifecycle nodes.

See https://github.com/ros-perception/point_cloud_transport/pull/109 for more details.

``rcl_logging_implementation``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A new ``rcl_logging_implementation`` package has been introduced to allow users to select the logging backend implementation at runtime without rebuilding ``rcl``.
Users can set the ``RCL_LOGGING_IMPLEMENTATION`` environment variable to switch between available logging backends (e.g., ``rcl_logging_spdlog``, ``rcl_logging_noop``, or custom implementations).
If not specified, ``rcl_logging_spdlog`` is used by default.

See https://github.com/ros2/rcl/issues/1178, https://github.com/ros2/rcl/pull/1276, and https://github.com/ros2/rcl_logging/pull/135 for more details.

``rclcpp``
^^^^^^^^^^

* `Unified component container interface <https://github.com/ros2/rclcpp/pull/3134>`__ -
  ``component_container`` is now the single entrypoint for launching both regular and
  isolated component containers with all types of executors.

* Added ``disable_callbacks()`` and ``enable_callbacks()`` APIs to ``SubscriptionBase``.
  By design the subscription itself is a shared pointer and its callbacks are propagated to
  the executor, making it difficult to manage subscription lifecycle from the application
  side.
  When a subscription is deleted at the application layer, it may still be referenced by
  the node and executor, causing callbacks to be invoked unexpectedly.
  This can lead to undefined behavior when the callback accesses resources that have been
  cleaned up.
  The new APIs allow users to disable subscription callbacks at runtime without destroying
  the subscription, preventing unexpected callback invocations.

  See `ros2/rclcpp#2984 <https://github.com/ros2/rclcpp/issues/2984>`__ and
  `ros2/rclcpp#2985 <https://github.com/ros2/rclcpp/pull/2985>`__ for more details.

``rclpy``
^^^^^^^^^

Native asyncio support with ``AsyncNode`` (experimental)
""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Added ``AsyncNode``, a new node type that runs on the ``asyncio`` event loop.
Subscription, service, and timer callbacks can now ``await`` any asyncio operation.
The new async ``client.call(request)`` and sim-time aware ``clock.sleep(...)`` are awaitable from any asyncio task.
CPU usage is significantly reduced compared to the ``SingleThreadedExecutor``.

See the :doc:`Writing an async node with asyncio <../Tutorials/Intermediate/Writing-An-Async-Node-With-Asyncio-Python>` tutorial and https://github.com/ros2/rclpy/pull/1620 for more details.

Static typing stablization
""""""""""""""""""""""""""

All methods and classes now have proper type hints.

``rosbag2``
^^^^^^^^^^^

``Rosbag2`` received several new recording, playback, observability, and API improvements.

* Added support for repeating transient-local messages when recording bags.
  Users can configure selected transient-local topics with ``--repeat-transient-local`` and
  an optional per-topic depth, or use ``--repeat-all-transient-local`` to automatically apply
  the behavior to topics whose publishers offer ``TRANSIENT_LOCAL`` durability QoS.
  This helps preserve latched/static state, such as maps or static transforms, across bag
  splits and snapshots.

  See `ros2/rosbag2#2385 <https://github.com/ros2/rosbag2/pull/2385>`__,
  `ros2/rosbag2#2386 <https://github.com/ros2/rosbag2/pull/2386>`__,
  `ros2/rosbag2#2387 <https://github.com/ros2/rosbag2/pull/2387>`__, and
  `ros2/rosbag2#2391 <https://github.com/ros2/rosbag2/pull/2391>`__ for more details.

* Added recorder service APIs for starting and stopping recording, starting and stopping topic
  discovery, and querying whether discovery is running.
  This makes it possible to control a recorder process remotely through ROS services instead
  of only through the command-line process lifecycle.

  See `ros2/rosbag2#2248 <https://github.com/ros2/rosbag2/pull/2248>`__ for more details.

* Added scheduled service operations for coordinated recording and playback workflows.
  Record, play, resume, and split operations can now be scheduled by time, with support for
  node time, publish time, and receive time modes where applicable.
  Service responses were also extended with explicit return codes and error strings.

  See `ros2/rosbag2#2330 <https://github.com/ros2/rosbag2/pull/2330>`__ and
  `ros2/rosbag2#2357 <https://github.com/ros2/rosbag2/pull/2357>`__ for more details.

* Added circular logging by split count.
  When recording with bag splitting enabled, ``--max-bag-files`` limits the maximum number of
  bag files stored on disk by automatically deleting the oldest split files as new ones are
  created.

  See `ros2/rosbag2#2218 <https://github.com/ros2/rosbag2/pull/2218>`__ for more details.

* Added a time-bounded snapshot cache option, ``--max-cache-duration``.
  This allows snapshot-mode recording to bound the cached data by duration in addition to
  existing cache controls.

  See `ros2/rosbag2#2289 <https://github.com/ros2/rosbag2/pull/2289>`__ for more details.

* Added message-loss observability during recording.
  ``Rosbag2`` can now collect message-loss statistics from the transport layer and recorder
  internals, and publish incremental per-topic loss events on the predefined
  ``events/rosbag2_messages_lost`` topic.
  The publishing rate can be configured with ``--stats_max_publishing_rate``.

  See `ros2/rosbag2#2039 <https://github.com/ros2/rosbag2/pull/2039>`__,
  `ros2/rosbag2#2144 <https://github.com/ros2/rosbag2/pull/2144>`__, and
  `ros2/rosbag2#2150 <https://github.com/ros2/rosbag2/pull/2150>`__ for more details.

* Expanded the ``rosbag2_py`` player and recorder APIs.
  Python users can now programmatically control playback and recording with APIs such as
  pause, resume, stop, seek, play-next, spin control, and wait helpers, instead of relying
  only on blocking command-line-style helpers.

  See `ros2/rosbag2#2047 <https://github.com/ros2/rosbag2/pull/2047>`__ and
  `ros2/rosbag2#2062 <https://github.com/ros2/rosbag2/pull/2062>`__ for more details.

* Added APIs for querying player timing metadata.
  Users can now query the player's starting time and playback duration, and Python readers
  can access the send timestamp when reading serialized messages.

  See `ros2/rosbag2#2061 <https://github.com/ros2/rosbag2/pull/2061>`__ and
  `ros2/rosbag2#2095 <https://github.com/ros2/rosbag2/pull/2095>`__ for more details.

* Improved message definition resolution for recorded data.
  ``Rosbag2`` can now find message definitions in nested subdirectories, better resolve inner
  message definitions for service and action introspection data, and support relative includes
  in local IDL message definitions.

  See `ros2/rosbag2#2041 <https://github.com/ros2/rosbag2/pull/2041>`__,
  `ros2/rosbag2#2052 <https://github.com/ros2/rosbag2/pull/2052>`__,
  `ros2/rosbag2#2055 <https://github.com/ros2/rosbag2/pull/2055>`__, and
  `ros2/rosbag2#2241 <https://github.com/ros2/rosbag2/pull/2241>`__ for more details.

* Updated split bag file naming to include the split index, bag name, and timestamp, making
  split files easier to identify and sort.

  See `ros2/rosbag2#2265 <https://github.com/ros2/rosbag2/pull/2265>`__ for more details.

* Enabled the ``rosbag2_performance_benchmarking`` package to be built by default, making the
  benchmarking tools easier to use for recorder and player performance evaluation.

  See `ros2/rosbag2#2093 <https://github.com/ros2/rosbag2/pull/2093>`__ for more details.

``rosidl_buffer``
^^^^^^^^^^^^^^^^^

Added ``rosidl::Buffer<T>``, a generated C++ container for variable-length primitive array fields such as ``uint8[]``.
It behaves like ``std::vector<T>`` with the default CPU backend, while allowing backend plugins to provide externally managed storage such as GPU memory.

The first RMW integration is for topic publish/subscribe with ``rmw_fastrtps_cpp``.

See :doc:`../Concepts/Intermediate/About-Buffer-Backends` and :doc:`../How-To-Guides/Using-Buffer-Backends` for more details.

``rosidl_buffer_backend``
^^^^^^^^^^^^^^^^^^^^^^^^^

Added the ``rosidl::BufferBackend`` plugin interface for packages that implement storage and transport backends for ``rosidl::Buffer`` fields.
Backend plugins provide descriptor message type support, build per-endpoint descriptors, reconstruct buffers on the receiving side, and participate in endpoint discovery.

Backends are discovered through ``pluginlib`` and registered by RMW automatically.
See :doc:`../Tutorials/Advanced/Writing-a-Buffer-Backend` for the backend implementer guide.

``rosidl_python``
^^^^^^^^^^^^^^^^^

Passing in Python ``set`` objects into array or sequence fields is now deprecated.
Instead pass in something that implements ``collections.abc.Sequence`` most commonly a ``list``, ``tuple``, or a ``numpy.ndarray``. To be removed in ROS M.

All generated messages, services, and actions are fully statically typed.
