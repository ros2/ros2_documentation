.. redirect-from::

    Tutorials/Content-Filtering-Subscription

Creating a content filtering subscription
=========================================

**Goal:** Create a content filtering subscription.

**Tutorial level:** Advanced

**Time:** 15 minutes

.. contents:: Table of Contents
   :depth: 1
   :local:

Overview
--------

ROS 2 applications typically consist of topics to transmit data from publishers to subscriptions.
Basically, subscriptions receive all published data from publishers on the topic.
But sometimes, a subscription might be interested in only a subset of the data which is being sent by publishers.
A content filtering subscription allows to receive only the data of interest for the application.

In this demo, we'll be highlighting how to create a content filtering subscription and how they work.

RMW Support
-----------

Content filtering subscriptions require RMW implementation support.

.. list-table::  Content-Filtering-Subscription Support Status
   :widths: 25 25

   * - rmw_fastrtps
     - supported
   * - rmw_connextdds
     - supported
   * - rmw_cyclonedds
     - not supported

Currently all RMW implementations that support content filtering subscriptions are `DDS <https://www.omg.org/omg-dds-portal/>`__ based.
That means that the supported filtering expressions and parameters are also dependent on `DDS <https://www.omg.org/omg-dds-portal/>`__, you can refer to `DDS specification <https://www.omg.org/spec/DDS/1.4/PDF>`__ ``Annex B - Syntax for Queries and Filters`` for details.

Installing the demo
-------------------

See the :doc:`installation instructions <../../Installation>` for details on installing ROS 2.

If you've installed ROS 2 from packages, ensure that you have ``ros-{DISTRO}-demo-nodes-cpp`` installed.
If you downloaded the archive or built ROS 2 from source, it will already be part of the installation.

Temperature filtering demo
--------------------------

This demo shows how a content filtering subscription can be used to only receive temperature values that are out of the acceptable temperature range, detecting emergencies.
The content filtering subscription filters out the uninteresting temperature data, so that the subscription callback is not issued.

ContentFilteringPublisher:

https://github.com/ros2/demos/blob/{REPOS_FILE_BRANCH}/demo_nodes_cpp/src/topics/content_filtering_publisher.cpp

.. code-block:: c++

    #include <chrono>
    #include <cstdio>
    #include <memory>
    #include <utility>

    #include "rclcpp/rclcpp.hpp"
    #include "rclcpp_components/register_node_macro.hpp"

    #include "std_msgs/msg/float32.hpp"

    #include "demo_nodes_cpp/visibility_control.h"

    using namespace std::chrono_literals;

    namespace demo_nodes_cpp
    {
    // The simulated temperature data starts from -100.0 and ends at 150.0 with a step size of 10.0
    constexpr std::array<float, 3> TEMPERATURE_SETTING {-100.0f, 150.0f, 10.0f};

    // Create a ContentFilteringPublisher class that subclasses the generic rclcpp::Node base class.
    // The main function below will instantiate the class as a ROS node.
    class ContentFilteringPublisher : public rclcpp::Node
    {
    public:
      DEMO_NODES_CPP_PUBLIC
      explicit ContentFilteringPublisher(const rclcpp::NodeOptions & options)
      : Node("content_filtering_publisher", options)
      {
        // Create a function for when messages are to be sent.
        setvbuf(stdout, NULL, _IONBF, BUFSIZ);
        auto publish_message =
          [this]() -> void
          {
            msg_ = std::make_unique<std_msgs::msg::Float32>();
            msg_->data = temperature_;
            temperature_ += TEMPERATURE_SETTING[2];
            if (temperature_ > TEMPERATURE_SETTING[1]) {
              temperature_ = TEMPERATURE_SETTING[0];
            }
            RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", msg_->data);
            // Put the message into a queue to be processed by the middleware.
            // This call is non-blocking.
            pub_->publish(std::move(msg_));
          };
        // Create a publisher with a custom Quality of Service profile.
        // Uniform initialization is suggested so it can be trivially changed to
        // rclcpp::KeepAll{} if the user wishes.
        // (rclcpp::KeepLast(7) -> rclcpp::KeepAll() fails to compile)
        rclcpp::QoS qos(rclcpp::KeepLast{7});
        pub_ = this->create_publisher<std_msgs::msg::Float32>("temperature", qos);

        // Use a timer to schedule periodic message publishing.
        timer_ = this->create_wall_timer(1s, publish_message);
      }

    private:
      float temperature_ = TEMPERATURE_SETTING[0];
      std::unique_ptr<std_msgs::msg::Float32> msg_;
      rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
      rclcpp::TimerBase::SharedPtr timer_;
    };

    }  // namespace demo_nodes_cpp

The content filter is defined in the subscription side, publishers don't need to be configured in any special way to allow content filtering.
The ``ContentFilteringPublisher`` node publishes simulated temperature data starting from -100.0 and ending at 150.0 with a step size of 10.0 every second.

We can run the demo by running the ``ros2 run demo_nodes_cpp content_filtering_publisher`` executable (don't forget to source the setup file first):

.. code-block:: bash

    $ ros2 run demo_nodes_cpp content_filtering_publisher
    [INFO] [1651094594.822753479] [content_filtering_publisher]: Publishing: '-100.000000'
    [INFO] [1651094595.822723857] [content_filtering_publisher]: Publishing: '-90.000000'
    [INFO] [1651094596.822752996] [content_filtering_publisher]: Publishing: '-80.000000'
    [INFO] [1651094597.822752475] [content_filtering_publisher]: Publishing: '-70.000000'
    [INFO] [1651094598.822721485] [content_filtering_publisher]: Publishing: '-60.000000'
    [INFO] [1651094599.822696188] [content_filtering_publisher]: Publishing: '-50.000000'
    [INFO] [1651094600.822699217] [content_filtering_publisher]: Publishing: '-40.000000'
    [INFO] [1651094601.822744113] [content_filtering_publisher]: Publishing: '-30.000000'
    [INFO] [1651094602.822694805] [content_filtering_publisher]: Publishing: '-20.000000'
    [INFO] [1651094603.822735805] [content_filtering_publisher]: Publishing: '-10.000000'
    [INFO] [1651094604.822722094] [content_filtering_publisher]: Publishing: '0.000000'
    [INFO] [1651094605.822699960] [content_filtering_publisher]: Publishing: '10.000000'
    [INFO] [1651094606.822748946] [content_filtering_publisher]: Publishing: '20.000000'
    [INFO] [1651094607.822694017] [content_filtering_publisher]: Publishing: '30.000000'
    [INFO] [1651094608.822708798] [content_filtering_publisher]: Publishing: '40.000000'
    [INFO] [1651094609.822692417] [content_filtering_publisher]: Publishing: '50.000000'
    [INFO] [1651094610.822696426] [content_filtering_publisher]: Publishing: '60.000000'
    [INFO] [1651094611.822751913] [content_filtering_publisher]: Publishing: '70.000000'
    [INFO] [1651094612.822692231] [content_filtering_publisher]: Publishing: '80.000000'
    [INFO] [1651094613.822745549] [content_filtering_publisher]: Publishing: '90.000000'
    [INFO] [1651094614.822701982] [content_filtering_publisher]: Publishing: '100.000000'
    [INFO] [1651094615.822691465] [content_filtering_publisher]: Publishing: '110.000000'
    [INFO] [1651094616.822649070] [content_filtering_publisher]: Publishing: '120.000000'
    [INFO] [1651094617.822693616] [content_filtering_publisher]: Publishing: '130.000000'
    [INFO] [1651094618.822691832] [content_filtering_publisher]: Publishing: '140.000000'
    [INFO] [1651094619.822688452] [content_filtering_publisher]: Publishing: '150.000000'
    [INFO] [1651094620.822645327] [content_filtering_publisher]: Publishing: '-100.000000'
    [INFO] [1651094621.822689219] [content_filtering_publisher]: Publishing: '-90.000000'
    [INFO] [1651094622.822694292] [content_filtering_publisher]: Publishing: '-80.000000'
    [...]

ContentFilteringSubscriber:

https://github.com/ros2/demos/blob/{REPOS_FILE_BRANCH}/demo_nodes_cpp/src/topics/content_filtering_subscriber.cpp

.. code-block:: c++

    #include "rclcpp/rclcpp.hpp"
    #include "rclcpp_components/register_node_macro.hpp"
    #include "rcpputils/join.hpp"

    #include "std_msgs/msg/float32.hpp"

    #include "demo_nodes_cpp/visibility_control.h"

    namespace demo_nodes_cpp
    {
    // Emergency temperature data less than -30 or greater than 100
    constexpr std::array<float, 2> EMERGENCY_TEMPERATURE {-30.0f, 100.0f};

    // Create a ContentFilteringSubscriber class that subclasses the generic rclcpp::Node base class.
    // The main function below will instantiate the class as a ROS node.
    class ContentFilteringSubscriber : public rclcpp::Node
    {
    public:
      DEMO_NODES_CPP_PUBLIC
      explicit ContentFilteringSubscriber(const rclcpp::NodeOptions & options)
      : Node("content_filtering_subscriber", options)
      {
        setvbuf(stdout, NULL, _IONBF, BUFSIZ);
        // Create a callback function for when messages are received.
        auto callback =
          [this](const std_msgs::msg::Float32 & msg) -> void
          {
            if (msg.data < EMERGENCY_TEMPERATURE[0] || msg.data > EMERGENCY_TEMPERATURE[1]) {
              RCLCPP_INFO(
                this->get_logger(),
                "I receive an emergency temperature data: [%f]", msg.data);
            } else {
              RCLCPP_INFO(this->get_logger(), "I receive a temperature data: [%f]", msg.data);
            }
          };

        // Initialize a subscription with a content filter to receive emergency temperature data that
        // are less than -30 or greater than 100.
        rclcpp::SubscriptionOptions sub_options;
        sub_options.content_filter_options.filter_expression = "data < %0 OR data > %1";
        sub_options.content_filter_options.expression_parameters = {
          std::to_string(EMERGENCY_TEMPERATURE[0]),
          std::to_string(EMERGENCY_TEMPERATURE[1])
        };

        sub_ = create_subscription<std_msgs::msg::Float32>("temperature", 10, callback, sub_options);

        if (!sub_->is_cft_enabled()) {
          RCLCPP_WARN(
            this->get_logger(), "Content filter is not enabled since it's not supported");
        } else {
          RCLCPP_INFO(
            this->get_logger(),
            "subscribed to topic \"%s\" with content filter options \"%s, {%s}\"",
            sub_->get_topic_name(),
            sub_options.content_filter_options.filter_expression.c_str(),
            rcpputils::join(sub_options.content_filter_options.expression_parameters, ", ").c_str());
        }
      }

    private:
      rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
    };

    }  // namespace demo_nodes_cpp

To enable content filtering, applications can set the filtering expression and the expression parameters in ``SubscriptionOptions``.
The application can also check if content filtering is enabled on the subscription.

In this demo, the ``ContentFilteringSubscriber`` node creates a content filtering subscription that receives a message only if the temperature value is less than -30.0 or greater than 100.0.

As commented before, content filtering subscription support depends on the RMW implementation.
Applications can use the ``is_cft_enabled`` method to check if content filtering is actually enabled on the subscription.

To test content filtering subscription, let's run it:

.. code-block:: bash

    $ ros2 run demo_nodes_cpp content_filtering_subscriber
    [INFO] [1651094590.682660703] [content_filtering_subscriber]: subscribed to topic "/temperature" with content filter options "data < %0 OR data > %1, {-30.000000, 100.000000}"
    [INFO] [1651094594.823805294] [content_filtering_subscriber]: I receive an emergency temperature data: [-100.000000]
    [INFO] [1651094595.823419993] [content_filtering_subscriber]: I receive an emergency temperature data: [-90.000000]
    [INFO] [1651094596.823410859] [content_filtering_subscriber]: I receive an emergency temperature data: [-80.000000]
    [INFO] [1651094597.823350377] [content_filtering_subscriber]: I receive an emergency temperature data: [-70.000000]
    [INFO] [1651094598.823282657] [content_filtering_subscriber]: I receive an emergency temperature data: [-60.000000]
    [INFO] [1651094599.823297857] [content_filtering_subscriber]: I receive an emergency temperature data: [-50.000000]
    [INFO] [1651094600.823355597] [content_filtering_subscriber]: I receive an emergency temperature data: [-40.000000]
    [INFO] [1651094615.823315377] [content_filtering_subscriber]: I receive an emergency temperature data: [110.000000]
    [INFO] [1651094616.823258458] [content_filtering_subscriber]: I receive an emergency temperature data: [120.000000]
    [INFO] [1651094617.823323525] [content_filtering_subscriber]: I receive an emergency temperature data: [130.000000]
    [INFO] [1651094618.823315527] [content_filtering_subscriber]: I receive an emergency temperature data: [140.000000]
    [INFO] [1651094619.823331424] [content_filtering_subscriber]: I receive an emergency temperature data: [150.000000]
    [INFO] [1651094620.823271748] [content_filtering_subscriber]: I receive an emergency temperature data: [-100.000000]
    [INFO] [1651094621.823343550] [content_filtering_subscriber]: I receive an emergency temperature data: [-90.000000]
    [INFO] [1651094622.823286326] [content_filtering_subscriber]: I receive an emergency temperature data: [-80.000000]
    [INFO] [1651094623.823371031] [content_filtering_subscriber]: I receive an emergency temperature data: [-70.000000]
    [INFO] [1651094624.823333112] [content_filtering_subscriber]: I receive an emergency temperature data: [-60.000000]
    [INFO] [1651094625.823266469] [content_filtering_subscriber]: I receive an emergency temperature data: [-50.000000]
    [INFO] [1651094626.823284093] [content_filtering_subscriber]: I receive an emergency temperature data: [-40.000000]

You should see a message showing the content filtering options used and logs for each message received only if the temperature value is less than -30.0 or greater than 100.0.

If content filtering is not supported by the RMW implementation, the subscription will still be created without content filtering enabled.
We can try that by executing ``RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run demo_nodes_cpp content_filtering_publisher``.

.. code-block:: bash

    $ RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run demo_nodes_cpp content_filtering_subscriber
    [WARN] [1651096637.893842072] [content_filtering_subscriber]: Content filter is not enabled since it is not supported
    [INFO] [1651096641.246043703] [content_filtering_subscriber]: I receive an emergency temperature data: [-100.000000]
    [INFO] [1651096642.245833527] [content_filtering_subscriber]: I receive an emergency temperature data: [-90.000000]
    [INFO] [1651096643.245743471] [content_filtering_subscriber]: I receive an emergency temperature data: [-80.000000]
    [INFO] [1651096644.245833932] [content_filtering_subscriber]: I receive an emergency temperature data: [-70.000000]
    [INFO] [1651096645.245916679] [content_filtering_subscriber]: I receive an emergency temperature data: [-60.000000]
    [INFO] [1651096646.245861895] [content_filtering_subscriber]: I receive an emergency temperature data: [-50.000000]
    [INFO] [1651096647.245946352] [content_filtering_subscriber]: I receive an emergency temperature data: [-40.000000]
    [INFO] [1651096648.245934569] [content_filtering_subscriber]: I receive a temperature data: [-30.000000]
    [INFO] [1651096649.245877906] [content_filtering_subscriber]: I receive a temperature data: [-20.000000]
    [INFO] [1651096650.245939068] [content_filtering_subscriber]: I receive a temperature data: [-10.000000]
    [INFO] [1651096651.245911450] [content_filtering_subscriber]: I receive a temperature data: [0.000000]
    [INFO] [1651096652.245879830] [content_filtering_subscriber]: I receive a temperature data: [10.000000]
    [INFO] [1651096653.245858329] [content_filtering_subscriber]: I receive a temperature data: [20.000000]
    [INFO] [1651096654.245916370] [content_filtering_subscriber]: I receive a temperature data: [30.000000]
    [INFO] [1651096655.245933741] [content_filtering_subscriber]: I receive a temperature data: [40.000000]
    [INFO] [1651096656.245833975] [content_filtering_subscriber]: I receive a temperature data: [50.000000]
    [INFO] [1651096657.245971483] [content_filtering_subscriber]: I receive a temperature data: [60.000000]

You can see the message ``Content filter is not enabled`` because underlying RMW implementation does not support the feature, but the demo still successfully creates the normal subscription to receive all temperature data.

Related content
---------------

- `content filtering examples <https://github.com/ros2/examples/blob/{REPOS_FILE_BRANCH}/rclcpp/topics/minimal_subscriber/content_filtering.cpp>`__ that covers all interfaces for content filtering subscription.
- `content filtering design PR <https://github.com/ros2/design/pull/282>`__
