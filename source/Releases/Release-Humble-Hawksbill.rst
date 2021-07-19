.. _upcoming-release:

.. _humble-release:

.. move this directive when next release page is created

ROS 2 Humble Hawksbill (codename 'humble'; May, 2022)
=====================================================

.. contents:: Table of Contents
   :depth: 2
   :local:

*Humble Hawksbill* is the eighth release of ROS 2.
What follows is highlights of the important changes and features in Humble Hawksbill since the last release.

Supported Platforms
-------------------

Humble Hawksbill is primarily supported on the following platforms:

Tier 1 platforms:

TBD

Tier 2 platforms:

TBD

Tier 3 platforms:

TBD

For more information about RMW implementations, compiler / interpreter versions, and system dependency versions see `REP 2000 <https://www.ros.org/reps/rep-2000.html>`__.

Installation
------------

To come.

New features in this ROS 2 release
----------------------------------

``ros_args`` attribute & ``ros_arguments`` parameter for nodes in launch files
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

It is now possible to provide `ROS-specific node arguments <../Guides/Node-arguments>` directly, without needing to use ``args`` with a leading ``--ros-args`` flag:

.. tabs::

   .. group-tab:: XML

    .. code-block:: xml

      <launch>
        <node pkg="demo_nodes_cpp" exec="talker" ros_args="--log-level debug" />
      </launch>

   .. group-tab:: YAML

      .. code-block:: yaml

        launch:
        - node:
            pkg: demo_nodes_cpp
            exec: talker
            ros_args: '--log-level debug'

The corresponding parameter for the ``Node`` action in Python launch files is ``ros_arguments``:

.. code-block:: python

  from launch import LaunchDescription
  import launch_ros.actions

  def generate_launch_description():
      return LaunchDescription([
          launch_ros.actions.Node(
              package='demo_nodes_cpp',
              executable='talker',
              ros_arguments=['--log-level', 'debug'],
          ),
      ])

Related PRs: `ros2/launch_ros#249 <https://github.com/ros2/launch_ros/pull/249>`_ and `ros2/launch_ros#253 <https://github.com/ros2/launch_ros/pull/253>`_.

Changes since the Galactic release
----------------------------------

rclcpp
^^^^^^

Support Type Adaption for Publishers and Subscriptions
""""""""""""""""""""""""""""""""""""""""""""""""""""""

After defining a type adapter, custom data structures can be used directly by publishers and subscribers, which helps to avoid additional work for the programmer and potential sources of errors.
This is especially useful when working with complex data types, such as when converting OpenCV's ``cv::Map`` to ROS's ``sensor_msgs/msg/Image`` type.

Here is an example of a type adapter that converts ``std_msgs::msg::String`` to ``std::string``:

.. code-block:: cpp

   template<>
   struct rclcpp::TypeAdapter<
      std::string,
      std_msgs::msg::String
   >
   {
     using is_specialized = std::true_type;
     using custom_type = std::string;
     using ros_message_type = std_msgs::msg::String;

     static
     void
     convert_to_ros_message(
       const custom_type & source,
       ros_message_type & destination)
     {
       destination.data = source;
     }

     static
     void
     convert_to_custom(
       const ros_message_type & source,
       custom_type & destination)
     {
       destination = source.data;
     }
   };

And an example of how the type adapter can be used:

.. code-block:: cpp

   using MyAdaptedType = TypeAdapter<std::string, std_msgs::msg::String>;

   // Publish a std::string
   auto pub = node->create_publisher<MyAdaptedType>(...);
   std::string custom_msg = "My std::string"
   pub->publish(custom_msg);

   // Pass a std::string to a subscription's callback
   auto sub = node->create_subscription<MyAdaptedType>(
     "topic",
     10,
     [](const std::string & msg) {...});

To learn more, see the `publisher <https://github.com/ros2/examples/blob/b83b18598b198b4a5ba44f9266c1bb39a393fa17/rclcpp/topics/minimal_publisher/member_function_with_type_adapter.cpp>`_ and `subscription <https://github.com/ros2/examples/blob/b83b18598b198b4a5ba44f9266c1bb39a393fa17/rclcpp/topics/minimal_subscriber/member_function_with_type_adapter.cpp>`_) examples, as well as a more complex `demo <https://github.com/ros2/demos/pull/482>`_.
For more details, see `REP 2007 <https://ros.org/reps/rep-2007.html>`_.

ros2cli
^^^^^^^

``ros2 topic pub`` will wait for one matching subscription when using ``--times/--once/-1``
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

When using ``--times/--once/-1`` flags, ``ros2 topic pub`` will wait for one matching subscription to be found before starting to publish.
This avoids the issue of the ros2cli node starting to publish before discovering a matching subscription, which results in some of the first messages being lost.
This is particularly unexpected when using a reliable qos profile.

The number of matching subscriptions to wait before starting publishing can be configured with the ``-w/--wait-matching-subscriptions`` flags, e.g.:

```
ros2 topic pub -1 -w 3 /chatter std_msgs/msg/String "{data: 'foo'}"
```

to wait for three matching subscriptions before starting to publish.

``-w`` can also be used independently of ``--times/--once/-1`` but it only defaults to one when combined with them, otherwise the ``-w`` default is zero.

See https://github.com/ros2/ros2cli/pull/642 for more details.

``ros2 param dump`` default output changed
""""""""""""""""""""""""""""""""""""""""""

  * ``--print`` option for dump command was `deprecated <https://github.com/ros2/ros2cli/pull/638>`_.

    It prints to stdout by default:

    .. code-block:: bash

      ros2 param dump /my_node_name

  * ``--output-dir`` option for dump command was `deprecated <https://github.com/ros2/ros2cli/pull/638>`_.

    To dump parameters to a file, run:

    .. code-block:: bash

      ros2 param dump /my_node_name > my_node_name.yaml

robot_state_publisher
^^^^^^^^^^^^^^^^^^^^^

Removal of deprecated ``use_tf_static`` parameter
"""""""""""""""""""""""""""""""""""""""""""""""""

The deprecated ``use_tf_static`` parameter has been removed from ``robot_state_publisher``.
This means that static transforms are unconditionally published to the ``/tf_static`` topic, and that the static transforms are published in a ``transient_local`` Quality of Service.
This was the default behavior, and the behavior which the ``tf2_ros::TransformListener`` class expected before, so most code will not have to be changed.
Any code that was relying on ``robot_state_publisher`` to periodically publish static transforms to ``/tf`` will have to be updated to subscribe to ``/tf_static`` as a ``transient_local`` subscription instead.

Known Issues
------------

To come.

Release Timeline
----------------

    Mon. March 21, 2022 - Alpha + RMW freeze
        Preliminary testing and stabilization of ROS Base [1]_ packages, and API and feature freeze for RMW provider packages.

    Mon. April 4, 2022 - Freeze
        API and feature freeze for ROS Base [1]_ packages in Rolling Ridley.
        Only bug fix releases should be made after this point.
        New packages can be released independently.

    Mon. April 18, 2022 - Branch
        Branch from Rolling Ridley.
        ``rosdistro`` is reopened for Rolling PRs for ROS Base [1]_ packages.
        Humble development shifts from ``ros-rolling-*`` packages to ``ros-humble-*`` packages.

    Mon. April 25, 2022 - Beta
        Updated releases of ROS Desktop [2]_ packages available.
        Call for general testing.

    Mon. May 16, 2022 - Release Candidate
        Release Candidate packages are built.
        Updated releases of ROS Desktop [2]_ packages available.

    Thu. May 19, 2022 - Distro Freeze
        Freeze rosdistro.
        No PRs for Humble on the ``rosdistro`` repo will be merged (reopens after the release announcement).

    Mon. May 23, 2022 - General Availability
        Release announcement.
        ``rosdistro`` is reopened for Humble PRs.

.. [1] The ``ros_base`` variant is described in `REP 2001 (ros-base) <https://www.ros.org/reps/rep-2001.html#ros-base>`_.
.. [2] The ``desktop`` variant is described in `REP 2001 (desktop-variants) <https://www.ros.org/reps/rep-2001.html#desktop-variants>`_.
