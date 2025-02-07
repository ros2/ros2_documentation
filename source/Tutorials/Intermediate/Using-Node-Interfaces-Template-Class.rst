Using the Node Interfaces Template Class (C++)
==============================================

**Goal:** Learn how to access ``Node`` information using ``rclcpp::NodeInterfaces<>``

**Tutorial level:** Intermediate

**Time:** 10 minutes

.. contents:: Table of Contents
   :depth: 2
   :local:


Overview
--------

The ``rclcpp::NodeInterfaces<>`` template class provides a compact and efficient way to manage Node Interfaces in ROS 2. This is particularly useful when working with different types of ``Nodes``, such as ``rclcpp::Node`` and ``rclcpp_lifecycle::LifecycleNode``, which do not share the same inheritance tree.


1 Accessing Node Information with a ``SharedPtr``
-------------------------------------------------

In the example below, we create a simple ``Node`` called ``Simple_Node`` and define a function ``node_info`` that accepts a ``SharedPtr`` to the ``Node``. The function retrieves and prints the name of the ``Node``.

.. code-block:: c++

    #include <memory>
    #include "rclcpp/rclcpp.hpp"

    void node_info(rclcpp::Node::SharedPtr node)
    {
      RCLCPP_INFO(node->get_logger(), "Node name: %s", node->get_name());
    }

    class SimpleNode : public rclcpp::Node
    {
    public:
      SimpleNode()
      : Node("Simple_Node")
      {
      }
    };

    int main(int argc, char * argv[])
    {
      rclcpp::init(argc, argv);
      auto node = std::make_shared<SimpleNode>();
      node_info(*node);
      rclcpp::spin(node);
      rclcpp::shutdown();
      return 0;
    }

Output:

.. code-block:: console

    [INFO] [Simple_Node]: Node name: Simple_Node

While this approach works well for arguments of type ``rclcpp::Node``, it does not work for other node types like ``rclcpp_lifecycle::LifecycleNode``. This is because they do not share the same inheritance tree.

To make the code more flexible and compatible with different node types, we use ``rclcpp::NodeInterfaces<>``.

2 Using ``rclcpp::NodeInterfaces<>``
------------------------------------

The recommended way of accessing a ``Node`` type's information is through the ``Node Interfaces``.

Below, similar to the previous example, we create a simple node of type ``rclcpp_lifecycle::LifecycleNode``.

.. code-block:: c++

    #include <memory>
    #include <string>
    #include <thread>
    #include "lifecycle_msgs/msg/transition.hpp"
    #include "rclcpp/rclcpp.hpp"
    #include "rclcpp_lifecycle/lifecycle_node.hpp"
    #include "rclcpp_lifecycle/lifecycle_publisher.hpp"
    #include "rclcpp/node_interfaces/node_interfaces.hpp"

    void node_info(
        rclcpp::node_interfaces::NodeInterfaces<
        rclcpp::node_interfaces::NodeBaseInterface,
        rclcpp::node_interfaces::NodeLoggingInterface> interfaces)
    {
      auto base_interface = interfaces.get_node_base_interface();
      auto logging_interface = interfaces.get_node_logging_interface();
      RCLCPP_INFO(logging_interface->get_logger(), "Node name: %s", base_interface->get_name());
    }

    class SimpleLifeCycleNode : public rclcpp_lifecycle::LifecycleNode
    {
    public:
      explicit SimpleLifeCycleNode(const std::string & node_name, bool intra_process_comms = false)
      : rclcpp_lifecycle::LifecycleNode(node_name,
        rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
      {}
    };

    int main(int argc, char * argv[])
    {
      rclcpp::init(argc, argv);
      rclcpp::executors::SingleThreadedExecutor exe;
      std::shared_ptr<SimpleLifeCycleNode> lc_node =
        std::make_shared<SimpleLifeCycleNode>("Simple_LifeCycle_Node");
      exe.add_node(lc_node->get_node_base_interface());
      node_info(*lc_node);
      exe.spin();
      rclcpp::shutdown();

      return 0;
    }

Output:

.. code-block:: console

    [INFO] [Simple_LifeCycle_Node]: Node name: Simple_LifeCycle_Node

2.1 Examine the code
~~~~~~~~~~~~~~~~~~~~

.. code-block:: c++

    void node_info(
    rclcpp::node_interfaces::NodeInterfaces<
        rclcpp::node_interfaces::NodeBaseInterface,
        rclcpp::node_interfaces::NodeLoggingInterface> interfaces)
    {
      auto base_interface = interfaces.get_node_base_interface();
      auto logging_interface = interfaces.get_node_logging_interface();
      RCLCPP_INFO(logging_interface->get_logger(), "Node name: %s", base_interface->get_name());
    }

Instead of accepting a ``SharedPtr``, this function takes a reference to a ``rclcpp::node_interfaces::NodeInterfaces`` object. Another advantage of using this approach is the support for implicit conversion of node-like objects.
This means that it is possible to directly pass any node-like object to a function expecting a ``rclcpp::node_interfaces::NodeInterfaces`` object.

It extracts:

* ``NodeBaseInterface`` Provides basic node functionalities.
* ``NodeLoggingInterface`` Enables logging.

Then, it retrieves and prints the node name.

.. code-block:: c++

    class SimpleLifeCycleNode : public rclcpp_lifecycle::LifecycleNode
    {
    public:
      explicit SimpleLifeCycleNode(const std::string & node_name, bool intra_process_comms = false)
      : rclcpp_lifecycle::LifecycleNode(node_name,
        rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
      {}
    };




Next, we create a ``rclcpp_lifecycle::LifecycleNode`` class. This class often includes functions for the state transitions  ``Unconfigured``, ``Inactive``, ``Active``, and ``Finalized``. However, we have not included any of them for demonstration purposes.

.. code-block:: c++

    int main(int argc, char * argv[])
    {
      rclcpp::init(argc, argv);
      rclcpp::executors::SingleThreadedExecutor exe;
      std::shared_ptr<SimpleLifeCycleNode> lc_node =
        std::make_shared<SimpleLifeCycleNode>("Simple_LifeCycle_Node");
      exe.add_node(lc_node->get_node_base_interface());
      node_info(*lc_node);
      exe.spin();
      rclcpp::shutdown();

      return 0;
    }


In the main function, a ``SharedPtr`` for the ``LifecycleNode`` is created, and the function declared above with the ``LifecycleNode`` as an argument is called.

.. note:: The ``SharedPtr`` needs to be dereferenced as the template accepts a reference to the ``NodeT`` object.
