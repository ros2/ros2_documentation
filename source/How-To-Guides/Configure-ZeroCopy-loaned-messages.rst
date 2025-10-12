.. redirect-from::

  How-To-Guides/Disabling-ZeroCopy-loaned-messages

Configure Zero Copy Loaned Messages
===================================

.. contents:: Contents
   :depth: 1
   :local:

Overview
--------

ROS 2 loaned messages and zero copy data sharing are mechanisms designed to improve performance by minimizing data copying.
When using loaned messages, the RMW middleware can allocate and manage message memory, allowing publishers and subscribers to share data buffers directly.
This reduces the overhead associated with memory allocation and data copying, leading to lower latency and higher throughput.
Zero copy data sharing is particularly beneficial in high-performance applications where large amounts of data need to be transmitted efficiently.

See more details for `Loaned Messages <https://design.ros2.org/articles/zero_copy.html>`__ article for details on how loaned messages work.

RMW Support
-----------

Loaned messages require RMW implementation support.

.. list-table::  Loaned Messages Support Status
   :widths: 25 25 25

   * - RMW Implementation
     - Support Status
     - Documentation
   * - rmw_fastrtps
     - supported
     - `Enable Zero Copy Data Sharing <https://github.com/ros2/rmw_fastrtps?tab=readme-ov-file#enable-zero-copy-data-sharing>`__
   * - rmw_connextdds
     - not supported
     - N.A
   * - rmw_cyclonedds
     - not supported
     - N.A

Installing the demo
-------------------

See the :doc:`installation instructions <../../Installation>` for details on installing ROS 2.

If you've installed ROS 2 from packages, ensure that you have ``ros-{DISTRO}-demo-nodes-cpp`` installed.
If you downloaded the archive or built ROS 2 from source, it will already be part of the installation.

Using Loaned Messages
---------------------

Loaned messages on the publisher are used by default when the underlying RMW implementation supports them.
If the RMW implementation does not support loaned messages, the messages will be allocated with the allocator instance provided by the publisher.
The `talker_loaned_message example <https://github.com/ros2/demos/blob/{REPOS_FILE_BRANCH}/demo_nodes_cpp/src/topics/talker_loaned_message.cpp>`__ demonstrates how to create a ROS 2 publisher that uses loaned messages to publish data efficiently without copying the message data.

.. code-block:: c++

    #include <chrono>
    #include <cstdio>
    #include <memory>
    #include <utility>

    #include "rclcpp/rclcpp.hpp"
    #include "rclcpp_components/register_node_macro.hpp"

    #include "std_msgs/msg/float64.hpp"
    #include "std_msgs/msg/string.hpp"

    #include "demo_nodes_cpp/visibility_control.h"

    using namespace std::chrono_literals;

    namespace demo_nodes_cpp
    {
    // Create a Talker class that subclasses the generic rclcpp::Node base class.
    // The main function below will instantiate the class as a ROS node.
    class LoanedMessageTalker : public rclcpp::Node
    {
    public:
      DEMO_NODES_CPP_PUBLIC
      explicit LoanedMessageTalker(const rclcpp::NodeOptions & options)
      : Node("loaned_message_talker", options)
      {
        // Create a function for when messages are to be sent.
        setvbuf(stdout, NULL, _IONBF, BUFSIZ);

        // We differentiate in this demo between two fundamental message types - POD and non-POD
        // PODs are plain old data types, meaning all the data of its type is encapsulated within
        // the structure and does not require any heap allocation or dynamic resizing.
        // non-PODs are essentially the opposite where the data size changes during runtime.
        // All containers (including Strings) are such non-PODs.
        // Most middlewares won't be able to loan non-POD datatypes.
        // We thus feature two publishers in this demo where both, a POD and non-POD message
        // will be used to publish data.
        // The take-away for this is that the rclcpp API for message loaning can cope with
        // either POD and non-POD transparently.
        auto publish_message =
          [this]() -> void
          {
            // We loan a message here and don't allocate the memory on the stack.
            // For middlewares which support message loaning, this means the middleware
            // completely owns the memory for this message.
            // This enables a zero-copy message transport for middlewares with shared memory
            // capabilities.
            // If the middleware doesn't support this, the loaned message will be allocated
            // with the allocator instance provided by the publisher.
            auto pod_loaned_msg = pod_pub_->borrow_loaned_message();
            auto pod_msg_data = static_cast<double>(count_);
            pod_loaned_msg.get().data = pod_msg_data;
            RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", pod_msg_data);
            // As the middleware might own the memory allocated for this message,
            // a call to publish explicitly transfers ownership back to the middleware.
            // The loaned message instance is thus no longer valid after a call to publish.
            pod_pub_->publish(std::move(pod_loaned_msg));

            // Similar as in the above case, we ask the middleware to loan a message.
            // As most likely the middleware won't be able to loan a message for a non-POD
            // data type, the memory for the message will be allocated on the heap within
            // the scope of the `LoanedMessage` instance.
            // After the call to `publish()`, the message will be correctly allocated.
            auto non_pod_loaned_msg = non_pod_pub_->borrow_loaned_message();
            auto non_pod_msg_data = "Hello World: " + std::to_string(count_);
            non_pod_loaned_msg.get().data = non_pod_msg_data;
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", non_pod_msg_data.c_str());
            non_pod_pub_->publish(std::move(non_pod_loaned_msg));
            count_++;
          };

        // Create a publisher with a custom Quality of Service profile.
        rclcpp::QoS qos(rclcpp::KeepLast(7));
        pod_pub_ = this->create_publisher<std_msgs::msg::Float64>("chatter_pod", qos);
        non_pod_pub_ = this->create_publisher<std_msgs::msg::String>("chatter", qos);

        // Use a timer to schedule periodic message publishing.
        timer_ = this->create_wall_timer(1s, publish_message);
      }

    private:
      size_t count_ = 1;
      rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pod_pub_;
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr non_pod_pub_;
      rclcpp::TimerBase::SharedPtr timer_;
    };

    }  // namespace demo_nodes_cpp

This example tries to loan two types of messages from the RMW implementation with calling ``borrow_loaned_message()``.
The one is a Plain Old Data (POD) message type, ``std_msgs::msg::Float64``, and the other is a non-Plain Old Data (POD) message type, ``std_msgs::msg::String``.
The requirements for loaned messages are that the message type is a Plain Old Data (POD) type for `rmw_fastrtps <https://github.com/ros2/rmw_fastrtps>`__ as shown below.

We can run the demo by running the ``ros2 run demo_nodes_cpp talker_loaned_message`` executable (don't forget to source the setup file first):

.. code-block:: console

    $ ros2 run demo_nodes_cpp talker_loaned_message
    [INFO] [1741063656.446278828] [loaned_message_talker]: Publishing: '1.000000'
    [INFO] [1741063656.446705580] [rclcpp]: Currently used middleware cannot loan messages. Local allocator will be used.
    [INFO] [1741063656.446754794] [loaned_message_talker]: Publishing: 'Hello World: 1'
    [INFO] [1741063657.446232119] [loaned_message_talker]: Publishing: '2.000000'
    [INFO] [1741063657.446401820] [loaned_message_talker]: Publishing: 'Hello World: 2'
    [INFO] [1741063658.446217220] [loaned_message_talker]: Publishing: '3.000000'
    [INFO] [1741063658.446383011] [loaned_message_talker]: Publishing: 'Hello World: 3'
    [...]

If the RMW implementation does not support loaned messages, all the messages will be allocated with the allocator instance provided by the publisher.
We can try that by executing ``RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run demo_nodes_cpp talker_loaned_message``.

.. code-block:: console

    $ RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run demo_nodes_cpp talker_loaned_message
    [INFO] [1741064109.676860153] [rclcpp]: Currently used middleware cannot loan messages. Local allocator will be used.
    [INFO] [1741064109.677043250] [loaned_message_talker]: Publishing: '1.000000'
    [INFO] [1741064109.677185724] [rclcpp]: Currently used middleware cannot loan messages. Local allocator will be used.
    [INFO] [1741064109.677224058] [loaned_message_talker]: Publishing: 'Hello World: 1'
    [INFO] [1741064110.676842111] [loaned_message_talker]: Publishing: '2.000000'
    [INFO] [1741064110.677008774] [loaned_message_talker]: Publishing: 'Hello World: 2'
    [INFO] [1741064111.676779850] [loaned_message_talker]: Publishing: '3.000000'
    [INFO] [1741064111.676937613] [loaned_message_talker]: Publishing: 'Hello World: 3'
    [...]

As we can see, both messages are published successfully, but the messages are allocated with the local allocator instance provided by the publisher because the RMW implementation does not support loaned messages.

How to disable Loaned Messages
------------------------------

Publishers
~~~~~~~~~~

By default, *Loaned Messages* will try to borrow the memory from underlying middleware if it supports *Loaned Messages*.
The ``ROS_DISABLE_LOANED_MESSAGES`` environment variable can be used to disable *Loaned Messages*, and fallback to normal publisher behavior, without any code changes or middleware configuration.
You can set the environment variable with the following command:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        $ export ROS_DISABLE_LOANED_MESSAGES=1

      To maintain this setting between shell sessions, you can add the command to your shell startup script:

      .. code-block:: console

        $ echo "export ROS_DISABLE_LOANED_MESSAGES=1" >> ~/.bashrc

   .. group-tab:: macOS

      .. code-block:: console

        $ export ROS_DISABLE_LOANED_MESSAGES=1

      To maintain this setting between shell sessions, you can add the command to your shell startup script:

      .. code-block:: console

        $ echo "export ROS_DISABLE_LOANED_MESSAGES=1" >> ~/.bash_profile

   .. group-tab:: Windows

      .. code-block:: console

        $ set ROS_DISABLE_LOANED_MESSAGES=1

      If you want to make this permanent between shell sessions, also run:

      .. code-block:: console

        $ setx ROS_DISABLE_LOANED_MESSAGES 1


Subscriptions
~~~~~~~~~~~~~

Currently using *Loaned Messages* is not safe on subscription, see more details in `rmw issue <https://github.com/ros2/rmw_cyclonedds/issues/469>`_ and `rclcpp issue <https://github.com/ros2/rclcpp/issues/2401>`_.
Because of this, by default *Loaned Messages* is ``disabled`` on subscription with `Set disable loan to on by default <https://github.com/ros2/rcl/pull/1110>`_ even though underlying middleware supports that.
To enable *Loaned Messages* on subscription, you need to set the environment variable ``ROS_DISABLE_LOANED_MESSAGES`` to ``0`` explicitly.

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        $ export ROS_DISABLE_LOANED_MESSAGES=0

      To maintain this setting between shell sessions, you can add the command to your shell startup script:

      .. code-block:: console

        $ echo "export ROS_DISABLE_LOANED_MESSAGES=0" >> ~/.bashrc

   .. group-tab:: macOS

      .. code-block:: console

        $ export ROS_DISABLE_LOANED_MESSAGES=0

      To maintain this setting between shell sessions, you can add the command to your shell startup script:

      .. code-block:: console

        $ echo "export ROS_DISABLE_LOANED_MESSAGES=0" >> ~/.bash_profile

   .. group-tab:: Windows

      .. code-block:: console

        $ set ROS_DISABLE_LOANED_MESSAGES=0

      If you want to make this permanent between shell sessions, also run:

      .. code-block:: console

        $ setx ROS_DISABLE_LOANED_MESSAGES 0
