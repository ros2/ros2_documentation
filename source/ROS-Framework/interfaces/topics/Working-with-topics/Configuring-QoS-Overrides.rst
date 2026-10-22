.. meta::
   :contentType: tutorial
   :experience: intermediate
   :area: interfaces, framework
   :distribution: {DISTRO}
   :product: {PRODUCT}

Configuring QoS overrides using parameters — tutorial
=====================================================

.. short-description::
   Quality of Service (QoS) overrides allow configuring publisher and subscription QoS profiles at node startup without modifying application source code.
   This tutorial demonstrates how to enable QoS overrides in C++ and Python nodes and configure them using parameter files.
   After following these steps, you will be able to inspect, configure, and validate QoS policies at runtime.

.. showmeta::
   :order: area, contentType, experience
   :labels: area=Area, contentType=Content type, experience=Level

**Goal:** Enable and configure Quality of Service (QoS) overrides for publishers and subscriptions using ROS 2 parameters.

**Tutorial level:** Intermediate

**Time:** 15 minutes

.. contents:: Table of Contents
   :depth: 2
   :local:

Overview
--------

In ROS 2, Quality of Service (QoS) profiles determine the frequency, reliability, and overall performance of communication between publishers and subscribers on a ROS topic.
By default, developers specify QoS settings directly in their application source code when creating publishers and subscriptions.
However, hard-coded QoS settings make it difficult to adapt nodes to different deployment environments.
For example, a robot might use reliable communications over a wired gigabit network during development, but require best-effort communications over a lossy wireless connection in deployment.

To address this challenge, ROS 2 provides **QoS overrides**.
QoS overrides allow developers to expose publisher and subscription QoS configurations as read-only ROS 2 parameters under a hierarchical parameter prefix (``qos_overrides.<topic>...``).
Users can then change QoS policies at node startup using YAML parameter files or command-line arguments without recompiling or altering the node source code.

Prerequisites
-------------

This tutorial assumes you have a :doc:`working ROS 2 installation <../../../../Get-Started/Installation>`.
The commands in this tutorial use the ``quality_of_service_demo_cpp`` and ``quality_of_service_demo_py`` packages included with the ROS 2 demos repository.

You can verify that these demo packages are available on your system by checking their executables:

.. code-block:: console

   $ ros2 pkg executables quality_of_service_demo_cpp
   $ ros2 pkg executables quality_of_service_demo_py

Enabling QoS overrides in code
------------------------------

To allow users to override QoS settings, the node author must opt in when constructing the publisher or subscription.
This opt-in process does not change the QoS policy itself, but instead provides a way for users to change the policy via a parameter file specified at execution time.
You can enable default reconfigurable policies (history, depth, and reliability) or specify a custom list of policy kinds with an optional validation callback.

.. tabs::

   .. group-tab:: C++

      In C++, include the ``rclcpp/qos_overriding_options.hpp`` header.
      Set the ``qos_overriding_options`` field on ``rclcpp::PublisherOptions`` or ``rclcpp::SubscriptionOptions``.

      .. code-block:: cpp

         #include "rclcpp/logging.hpp"
         #include "rclcpp/node.hpp"
         #include "rclcpp/publisher.hpp"
         #include "rclcpp/publisher_options.hpp"
         #include "rclcpp/qos.hpp"
         #include "rclcpp/qos_overriding_options.hpp"
         #include "rclcpp/subscription.hpp"
         #include "rclcpp/subscription_options.hpp"
         #include "std_msgs/msg/string.hpp"
         #include "std_msgs/msg/string.hpp"

         class QosConfigurableNode : public rclcpp::Node
         {
         public:
           QosConfigurableNode()
           : Node("qos_configurable_node")
           {
             // Configure publisher options to allow overriding default QoS policies (history, depth, and reliability)
             rclcpp::PublisherOptions pub_opts;
             pub_opts.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies(
               [](const rclcpp::QoS & qos) {
                 rclcpp::QosCallbackResult result;
                 if (qos.depth() > 10u) {
                   result.successful = false;
                   result.reason = "history depth must be 10 or less";
                   return result;
                 }
                 result.successful = true;
                 return result;
               });

             pub_ = this->create_publisher<std_msgs::msg::String>("chatter", 10, pub_opts);

             // Configure subscription options to allow overriding default QoS policies (history, depth, and reliability)
             rclcpp::SubscriptionOptions sub_opts;
             sub_opts.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();

             sub_ = this->create_subscription<std_msgs::msg::String>(
               "chatter",
               10,
               [this](std_msgs::msg::String::ConstSharedPtr msg) {
                 RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
               },
               sub_opts);
           }

         private:
           rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
           rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
         };

   .. group-tab:: Python

      In Python, import ``QoSOverridingOptions`` from ``rclpy.qos_overriding_options``.
      Pass ``qos_overriding_options`` when calling ``create_publisher`` or ``create_subscription``.

      .. code-block:: python

         import rclpy
         from rclpy.node import Node
         from rclpy.qos_overriding_options import QoSOverridingOptions, QosCallbackResult
         from std_msgs.msg import String


         class QosConfigurableNode(Node):

             def __init__(self):
                 super().__init__('qos_configurable_node')

                 # Enable QoS overrides with default policies (history, depth, reliability) for the publisher
                 self.pub = self.create_publisher(
                     String,
                     'chatter',
                     10,
                     # Adding the following line makes the QoS configurable at run time.
                     qos_overriding_options=QoSOverridingOptions.with_default_policies(
                         callback=self.validate_qos,
                     ),
                 )

                 # Enable QoS overrides with default policies (history, depth, reliability) for the subscription
                 self.sub = self.create_subscription(
                     String,
                     'chatter',
                     self.listener_callback,
                     10,
                     # Adding the following line makes the QoS configurable at run time.
                     qos_overriding_options=QoSOverridingOptions.with_default_policies(),
                 )

             def listener_callback(self, msg):
                 self.get_logger().info(f'Received: "{msg.data}"')

             def validate_qos(self, qos):
                 result = QosCallbackResult()
                 if qos.depth > 10:
                     result.successful = False
                     result.reason = 'history depth must be 10 or less'
                     return result
                 result.successful = True
                 return result

Validation callbacks
^^^^^^^^^^^^^^^^^^^^

A validation callback lets the node author verify whether the user-supplied QoS profile meets application safety requirements.
The callback receives the final resolved ``QoS`` profile and returns a result containing a boolean flag ``successful`` and a text ``reason``.
If the callback returns ``successful = false``, the node initialization fails with an exception (``InvalidQosOverridesException`` in C++, ``InvalidQosOverridesError`` in Python), preventing accidental misconfigurations.

Entity identifiers
^^^^^^^^^^^^^^^^^^

When a single node hosts multiple publishers or subscriptions on the same topic, pass an optional ``id`` or ``entity_id`` to distinguish them.
For example, supplying ``entity_id="fast_stream"`` renames the parameter group from ``publisher`` to ``publisher_fast_stream``.

QoS parameter hierarchy
-----------------------

When QoS overrides are enabled, the node declares read-only parameters under a hierarchical naming pattern:

.. code-block:: text

   qos_overrides.<topic_name>.<entity_type>[_<id>].<policy_name>

* ``<topic_name>``: The topic name associated with the publisher or subscription.
* ``<entity_type>``: Either ``publisher`` or ``subscription``.
* ``_<id>``: An optional suffix appended when an entity identifier is specified.
* ``<policy_name>``: The specific QoS policy to override.

The following table summarizes the supported QoS policies and their parameter types:

.. list-table:: Supported QoS override parameters
   :widths: 20 20 20 40
   :header-rows: 1

   * - Policy
     - Parameter Type
     - Scope
     - Possible Values
   * - ``history``
     - string
     - Publisher & Subscription
     - ``system_default``, ``keep_last``, ``keep_all``
   * - ``depth``
     - integer
     - Publisher & Subscription
     - Positive integer queue size (e.g., ``10``)
   * - ``reliability``
     - string
     - Publisher & Subscription
     - ``system_default``, ``reliable``, ``best_effort``
   * - ``durability``
     - string
     - Publisher & Subscription
     - ``system_default``, ``transient_local``, ``volatile``
   * - ``deadline``
     - integer or duration
     - Publisher & Subscription
     - Duration in nanoseconds, or dictionary with ``sec`` and ``nsec``
   * - ``lifespan``
     - integer or duration
     - Publisher only
     - Duration in nanoseconds, or dictionary with ``sec`` and ``nsec``
   * - ``liveliness``
     - string
     - Publisher & Subscription
     - ``system_default``, ``automatic``, ``manual_by_topic``
   * - ``liveliness_lease_duration``
     - integer or duration
     - Publisher & Subscription
     - Duration in nanoseconds, or dictionary with ``sec`` and ``nsec``

.. note::

   These parameters are declared as read-only.
   They cannot be modified at runtime using ``ros2 param set``.
   They must be supplied at node startup using parameter files or command-line arguments.

Creating a parameter overrides file
-----------------------------------

Create a YAML file named ``qos_overrides.yaml`` with your desired QoS configurations:

.. code-block:: yaml

   qos_overrides_talker:
     ros__parameters:
       qos_overrides:
         /qos_overrides_chatter:
           publisher:
             reliability: best_effort
             depth: 5

   qos_overrides_listener:
     ros__parameters:
       qos_overrides:
         /qos_overrides_chatter:
           subscription:
             reliability: best_effort
             depth: 5

Running the demonstration
-------------------------

Step 1: Inspect default parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Start the talker node in a terminal without any parameter overrides:

.. tabs::

   .. group-tab:: Python

      .. code-block:: console

         $ ros2 run quality_of_service_demo_py qos_overrides_talker

   .. group-tab:: C++

      .. code-block:: console

         $ ros2 run quality_of_service_demo_cpp qos_overrides_talker

In a second terminal, inspect the declared QoS parameters:

.. code-block:: console

   $ ros2 param list /qos_overrides_talker

You will see parameters under the ``qos_overrides`` namespace:

.. code-block:: text

   /qos_overrides_talker:
     qos_overrides./qos_overrides_chatter.publisher.depth
     qos_overrides./qos_overrides_chatter.publisher.history
     qos_overrides./qos_overrides_chatter.publisher.reliability
     ...

You can view the default values with ``ros2 param dump``:

.. code-block:: console

   $ ros2 param dump /qos_overrides_talker

The output reveals default values:

.. code-block:: yaml

   /qos_overrides_talker:
     ros__parameters:
       qos_overrides:
         /qos_overrides_chatter:
           publisher:
             depth: 10
             history: keep_last
             reliability: reliable

Stop the node using ``Ctrl-C``.

Step 2: Apply the parameter overrides
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Start the talker and listener passing the parameter file with ``--params-file``:

In terminal 1:

.. tabs::

   .. group-tab:: Python

      .. code-block:: console

         $ ros2 run quality_of_service_demo_py qos_overrides_talker --ros-args --params-file qos_overrides.yaml

   .. group-tab:: C++

      .. code-block:: console

         $ ros2 run quality_of_service_demo_cpp qos_overrides_talker --ros-args --params-file qos_overrides.yaml

In terminal 2:

.. tabs::

   .. group-tab:: Python

      .. code-block:: console

         $ ros2 run quality_of_service_demo_py qos_overrides_listener --ros-args --params-file qos_overrides.yaml

   .. group-tab:: C++

      .. code-block:: console

         $ ros2 run quality_of_service_demo_cpp qos_overrides_listener --ros-args --params-file qos_overrides.yaml

In a third terminal, verify that the endpoint QoS profile has been updated to best-effort:

.. code-block:: console

   $ ros2 topic info -v /qos_overrides_chatter

The output confirms that the publisher and subscription are operating with the overridden settings:

.. code-block:: text

   Type: example_interfaces/msg/String

   Publisher count: 1

   Node name: qos_overrides_talker
   Node namespace: /
   Topic type: example_interfaces/msg/String
   Endpoint type: PUBLISHER
   QoS profile:
     Reliability: BEST_EFFORT
     History (Depth): UNKNOWN
     Durability: VOLATILE
     Lifespan: Infinite
     Deadline: Infinite
     Liveliness: AUTOMATIC
     Liveliness lease duration: Infinite

   Subscription count: 1

   Node name: qos_overrides_listener
   Node namespace: /
   Topic type: example_interfaces/msg/String
   Endpoint type: SUBSCRIPTION
   QoS profile:
     Reliability: BEST_EFFORT
     History (Depth): UNKNOWN
     Durability: VOLATILE
     Lifespan: Infinite
     Deadline: Infinite
     Liveliness: AUTOMATIC
     Liveliness lease duration: Infinite

Step 3: Test validation callbacks
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The example demo enforces a maximum history depth of 10.
Edit ``qos_overrides.yaml`` and change the publisher depth to ``11``:

.. code-block:: yaml

   qos_overrides_talker:
     ros__parameters:
       qos_overrides:
         /qos_overrides_chatter:
           publisher:
             depth: 11

Now launch the talker node again:

.. code-block:: console

   $ ros2 run quality_of_service_demo_py qos_overrides_talker --ros-args --params-file qos_overrides.yaml

The node will reject the configuration and terminate with an error:

.. code-block:: text

   rclpy.qos_overriding_options.InvalidQosOverridesError: Provided QoS overrides for publisher `/qos_overrides_chatter` with id `None`, are not valid: expected qos depth less than 10

This mechanism ensures that invalid or incompatible QoS combinations are rejected at startup before communication begins.

Summary
-------

In this tutorial, you learned:

* How to enable QoS overrides in C++ using ``rclcpp::QosOverridingOptions`` and in Python using ``rclpy.qos_overriding_options.QoSOverridingOptions``.
* How ROS 2 organizes override parameters under the ``qos_overrides`` namespace.
* How to override publisher and subscription QoS policies at startup using a YAML parameter file.
* How to verify endpoint QoS profiles with ``ros2 topic info -v``.
* How validation callbacks enforce application safety constraints at initialization.

Related Content
---------------

* :doc:`About Quality of Service settings <../About-Quality-of-Service-Settings>`
* :doc:`Using quality-of-service settings for lossy networks — tutorial <Quality-of-Service>`
* :doc:`Overriding QoS policies for recording and playback </Developer-Tools/Debugging/Testing/Overriding-QoS-Policies-For-Recording-And-Playback>`
