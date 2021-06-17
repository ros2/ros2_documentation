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

To come.

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

Known Issues
------------

To come.

Release Timeline
----------------

To come.
