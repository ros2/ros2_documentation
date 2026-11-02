.. meta::
   :contentType: tutorial
   :experience: intermediate
   :area: nodes, framework
   :distribution: {DISTRO}
   :product: {PRODUCT}

Using Subordinate Nodes (C++) — tutorial
========================================

.. short-description::
   Subordinate nodes (or sub-nodes) extend a node's namespace to create modular scoped entities without introducing separate nodes into the ROS graph.
   This tutorial explains how to create and use subordinate nodes with ``rclcpp`` in C++.

.. showmeta::
   :order: area, contentType, experience
   :labels: area=Area, contentType=Content type, experience=Level

**Goal:** Learn how to create and use subordinate nodes (``create_sub_node``) in ``rclcpp`` to organize topics and services hierarchically.

**Tutorial level:** Intermediate

**Time:** 10 minutes

.. contents:: Table of Contents
   :depth: 2
   :local:

Summary
-------

* A **subordinate node** (or **sub-node**) is a secondary node instance created from an existing ``rclcpp::Node`` via ``create_sub_node()``.
* Sub-nodes share the parent node's underlying context, executor callbacks, parameter interface, logging interface, and ROS graph identity.
* Sub-nodes define an additional relative **sub-namespace** that scopes entities (publishers, subscriptions, service clients, and service servers) created through them.
* In ROS 1, developers often used ``ros::NodeHandle`` instances to scope namespaces for internal helper classes; in ROS 2, ``create_sub_node()`` fulfills a similar role for C++ components within a node.

Overview
--------

In ROS 2, a standard node represents a single entity in the computation graph.
However, complex nodes frequently contain internal helper libraries, device sub-drivers, or modular algorithms that need to publish or subscribe to topics under their own scoped namespace.

Creating a full separate node for each internal subcomponent incurs overhead: each node has its own middleware participant and discovery entity, requires separate executor management or thread scheduling, and clutters the node graph.

To solve this, ``rclcpp::Node`` provides the ``create_sub_node()`` method.
A subordinate node allows internal components within a single ROS 2 node process to encapsulate their own relative topic and service namespaces while reusing all of the parent node's underlying resources.

How Subordinate Nodes Work
--------------------------

Namespaces and naming
^^^^^^^^^^^^^^^^^^^^^

When you create a sub-node, you provide a relative sub-namespace string:

.. code-block:: c++

   auto node = std::make_shared<rclcpp::Node>("my_node", "/my_ns");
   auto sub_node = node->create_sub_node("sub_ns");

The node provides three namespace-related query methods:

* ``get_namespace()``: Returns the parent node's base namespace (for example, ``"/my_ns"``).
* ``get_sub_namespace()``: Returns the accumulated relative sub-namespace (for example, ``"sub_ns"``).
* ``get_effective_namespace()``: Returns the concatenation of the base namespace and the accumulated sub-namespace (for example, ``"/my_ns/sub_ns"``).

Sub-nodes can also be nested hierarchically:

.. code-block:: c++

   auto sub_node2 = sub_node->create_sub_node("nested");
   // sub_node2->get_sub_namespace()       -> "sub_ns/nested"
   // sub_node2->get_effective_namespace() -> "/my_ns/sub_ns/nested"

Entity scoping rules
^^^^^^^^^^^^^^^^^^^^

When you use a sub-node to create publishers, subscriptions, service clients, or service servers:

* **Relative names**: Any relative topic or service name (a name not starting with ``/``) is resolved relative to the sub-node's *effective namespace*.
  For example, creating a publisher for ``"chatter"`` on a sub-node with effective namespace ``"/my_ns/sub_ns"`` results in the topic ``"/my_ns/sub_ns/chatter"``.
* **Absolute names**: Any absolute name (starting with ``/``) bypasses all namespaces and resolves directly to the global name, identical to standard ROS 2 naming behavior.
* **Relative requirement**: The sub-namespace passed to ``create_sub_node()`` must be relative (it cannot start with ``/`` and cannot be empty).
  Passing an absolute or empty sub-namespace throws ``rclcpp::exceptions::NameValidationError``.

Shared resources vs. Scoped behavior
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The following table summarizes what is shared with the parent node and what is scoped by the subordinate node:

.. list-table::
   :header-rows: 1
   :widths: 35 65

   * - Aspect
     - Behavior
   * - **Graph identity**
     - Shared.
       The sub-node shares the parent's ``rclcpp::node_interfaces::NodeBaseInterface``.
       It does not appear as a separate node in ``ros2 node list``.
   * - **Node name**
     - Shared.
       ``sub_node->get_name()`` returns the parent node's name.
   * - **Executor and callbacks**
     - Shared.
       Callbacks registered on sub-node entities are dispatched by the parent node's executor.
       You spin the parent node, not the sub-node.
   * - **Parameters**
     - Shared.
       The sub-node shares the parent's ``rclcpp::node_interfaces::NodeParametersInterface``.
       Declaring or querying parameters operates on the parent node's parameter store.
   * - **Logging**
     - Shared.
       The sub-node shares the parent's ``rclcpp::node_interfaces::NodeLoggingInterface`` and parent logger.
   * - **Clock and timers**
     - Shared.
       Timers and clock interfaces are shared with the parent node.
   * - **Entity namespaces**
     - Scoped.
       Relative publishers, subscriptions, and services use the sub-node's effective namespace.

C++ Example
-----------

The following example demonstrates creating a parent node and two subordinate nodes to publish telemetry from two sensors:

.. code-block:: c++

   #include <chrono>
   #include <memory>
   #include <string>

   #include "rclcpp/rclcpp.hpp"
   #include "std_msgs/msg/string.hpp"

   using namespace std::chrono_literals;

   class RobotSystemNode : public rclcpp::Node
   {
   public:
     RobotSystemNode()
     : Node("robot_system", "robot")
     {
       // Create subordinate nodes for distinct subsystems
       imu_subnode_ = this->create_sub_node("imu");
       camera_subnode_ = this->create_sub_node("camera");

       RCLCPP_INFO(
         this->get_logger(),
         "Parent node namespace: %s, effective: %s",
         this->get_namespace(),
         this->get_effective_namespace().c_str());

       RCLCPP_INFO(
         this->get_logger(),
         "IMU sub-node sub_ns: %s, effective: %s",
         imu_subnode_->get_sub_namespace().c_str(),
         imu_subnode_->get_effective_namespace().c_str());

       RCLCPP_INFO(
         this->get_logger(),
         "Camera sub-node sub_ns: %s, effective: %s",
         camera_subnode_->get_sub_namespace().c_str(),
         camera_subnode_->get_effective_namespace().c_str());

       // Create publishers via subordinate nodes using relative topic names
       // Resolved topics will be /robot/imu/data and /robot/camera/status
       imu_pub_ = imu_subnode_->create_publisher<std_msgs::msg::String>("data", 10);
       camera_pub_ = camera_subnode_->create_publisher<std_msgs::msg::String>("status", 10);

       timer_ = this->create_wall_timer(
         1000ms, std::bind(&RobotSystemNode::publish_data, this));
     }

   private:
     void publish_data()
     {
       auto imu_msg = std_msgs::msg::String();
       imu_msg.data = "imu_ok";
       imu_pub_->publish(imu_msg);

       auto cam_msg = std_msgs::msg::String();
       cam_msg.data = "camera_streaming";
       camera_pub_->publish(cam_msg);
     }

     rclcpp::Node::SharedPtr imu_subnode_;
     rclcpp::Node::SharedPtr camera_subnode_;
     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr imu_pub_;
     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr camera_pub_;
     rclcpp::TimerBase::SharedPtr timer_;
   };

   int main(int argc, char * argv[])
   {
     rclcpp::init(argc, argv);
     auto node = std::make_shared<RobotSystemNode>();
     rclcpp::spin(node);
     rclcpp::shutdown();
     return 0;
   }

Verifying Behavior with CLI Tools
---------------------------------

When running this node, you can verify how subordinate nodes appear in the ROS 2 graph using the standard command-line tools:

1. List the active nodes:

   .. code-block:: console

      $ ros2 node list
      /robot/robot_system

   Notice that only the parent node ``/robot/robot_system`` appears in the graph.
   The subordinate nodes do not register as separate nodes.

2. List the active topics:

   .. code-block:: console

      $ ros2 topic list
      /parameter_events
      /robot/camera/status
      /robot/imu/data
      /rosout

   The topics published by the subordinate nodes are correctly scoped under ``/robot/camera/`` and ``/robot/imu/`` according to each sub-node's effective namespace.

When to Use Subordinate Nodes
-----------------------------

* **Use subordinate nodes** when you are developing modular C++ classes or internal helper libraries within a single executable and want each component to manage its own scoped topic or service namespace without the overhead of additional ROS 2 nodes.
* **Use composable nodes (components)** when your components represent independent units of computation that should be independently discoverable, dynamically loaded into component containers at launch or runtime, or composed into separate processes.
* **Use standalone nodes** when components require independent operating system processes, distinct lifecycles, or physical isolation across machines.
