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

Not all ROS Nodes are created equally!
The ``rclcpp::Node`` and ``rclcpp_lifecycle::LifecycleNode`` classes do not share an inheritance tree, which means ROS developers can run into compile time type issues when they want to write a function that takes in a ROS node pointer as an argument.
To address this issue, ``rclcpp`` includes the ``rclcpp::NodeInterfaces<>`` template type that should be used as the preferred convention for passing for both conventional and lifecycle nodes to functions.
This `ROSCon 2023 lightning talk <https://vimeo.com/879001243#t=16m0s>`_ summarizes the issue and remedy succinctly.
The following tutorial will show you how to use ``rclcpp::NodeInterfaces<>`` as reliable and compact interface for all ROS node types.

The ``rclcpp::NodeInterfaces<>`` template class provides a compact and efficient way to manage Node Interfaces in ROS 2.
This is particularly useful when working with different types of ``Nodes``, such as ``rclcpp::Node`` and ``rclcpp_lifecycle::LifecycleNode``, which do not share the same inheritance tree.

1 Accessing Node Information with a ``SharedPtr``
-------------------------------------------------

In the example below, we create a simple ``Node`` called ``Simple_Node`` and define a function ``node_info`` that accepts a ``SharedPtr`` to the ``Node``.
The function retrieves and prints the name of the ``Node``.

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
      SimpleNode(const std::string & node_name)
      : Node(node_name)
      {
      }
    };

    int main(int argc, char * argv[])
    {
      rclcpp::init(argc, argv);
      auto node = std::make_shared<SimpleNode>("Simple_Node");
      node_info(*node);
    }

Output:

.. code-block:: console

    [INFO] [Simple_Node]: Node name: Simple_Node

While this approach works well for arguments of type ``rclcpp::Node``, it does not work for other node types like ``rclcpp_lifecycle::LifecycleNode``.

2 Explicitly pass ``rclcpp::node_interfaces``
---------------------------------------------

A more robust approach, applicable to all node types, is to explicitly pass ``rclcpp::node_interfaces`` as function arguments, as demonstrated in the example below.
In the example that follows, we create function called ``node_info`` that take as arguments two ``rclcpp::node_interfaces``, ``NodeBaseInterface`` and ``NodeLoggingInterface`` and prints the ``Node`` name.
We then create two nodes of type ``rclcpp_lifecycle::LifecycleNode`` and ``rclcpp::Node`` and pass their interfaces in ``node_info``.

.. code-block:: c++

    void node_info(std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> base_interface,
                   std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> logging_interface)
    {
      RCLCPP_INFO(logging_interface->get_logger(), "Node name: %s", base_interface->get_name());
    }

    class SimpleNode : public rclcpp::Node
    {
    public:
      SimpleNode(const std::string & node_name)
      : Node(node_name)
      {
      }
    };

    class LifecycleTalker : public rclcpp_lifecycle::LifecycleNode
    {
    public:
      explicit LifecycleTalker(const std::string & node_name, bool intra_process_comms = false)
      : rclcpp_lifecycle::LifecycleNode(node_name,
          rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
      {}
    }

    int main(int argc, char * argv[])
    {
      rclcpp::init(argc, argv);
      rclcpp::executors::SingleThreadedExecutor exe;
      auto node = std::make_shared<SimpleNode>("Simple_Node");
      auto lc_node = std::make_shared<LifecycleTalker>("Simple_LifeCycle_Node");
      node_info(node->get_node_base_interface(),node->get_node_logging_interface());
      node_info(lc_node->get_node_base_interface(),lc_node->get_node_logging_interface());
    }

Output:

.. code-block:: console

    [INFO] [Simple_Node]: Node name: Simple_Node
    [INFO] [Simple_LifeCycle_Node]: Node name: Simple_LifeCycle_Node

As functions grow in complexity, the number of ``rclcpp::node_interfaces`` arguments also increases, leading to readability and compactness issues.
To make the code more flexible and compatible with different node types, we use ``rclcpp::NodeInterfaces<>``.

3 Using ``rclcpp::NodeInterfaces<>``
------------------------------------

The recommended way of accessing a ``Node`` type's information is through the ``Node Interfaces``.

Below, similar to the previous example, a ``rclcpp_lifecycle::LifecycleNode`` and a ``rclcpp::Node`` are created.

.. code-block:: c++

    #include <memory>
    #include <string>
    #include <thread>
    #include "lifecycle_msgs/msg/transition.hpp"
    #include "rclcpp/rclcpp.hpp"
    #include "rclcpp_lifecycle/lifecycle_node.hpp"
    #include "rclcpp_lifecycle/lifecycle_publisher.hpp"
    #include "rclcpp/node_interfaces/node_interfaces.hpp"

    using MyNodeInterfaces =
      rclcpp::node_interfaces::NodeInterfaces<rclcpp::node_interfaces::NodeBaseInterface, rclcpp::node_interfaces::NodeLoggingInterface>;

    void node_info(MyNodeInterfaces interfaces)
    {
      auto base_interface = interfaces.get_node_base_interface();
      auto logging_interface = interfaces.get_node_logging_interface();
      RCLCPP_INFO(logging_interface->get_logger(), "Node name: %s", base_interface->get_name());
    }

    class SimpleNode : public rclcpp::Node
    {
    public:
      SimpleNode(const std::string & node_name)
      : Node(node_name)
      {
      }
    };

    class LifecycleTalker : public rclcpp_lifecycle::LifecycleNode
    {
    public:
      explicit LifecycleTalker(const std::string & node_name, bool intra_process_comms = false)
      : rclcpp_lifecycle::LifecycleNode(node_name,
          rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
      {}
    }

    int main(int argc, char * argv[])
    {
      rclcpp::init(argc, argv);
      rclcpp::executors::SingleThreadedExecutor exe;
      auto node = std::make_shared<SimpleNode>("Simple_Node");
      auto lc_node = std::make_shared<LifecycleTalker>("Simple_LifeCycle_Node");
      node_info(*node);
      node_info(*lc_node);
    }

Output:

.. code-block:: console

    [INFO] [Simple_Node]: Node name: Simple_Node
    [INFO] [Simple_LifeCycle_Node]: Node name: Simple_LifeCycle_Node

3.1 Examine the code
~~~~~~~~~~~~~~~~~~~~

.. code-block:: c++

    using MyNodeInterfaces =
      rclcpp::node_interfaces::NodeInterfaces<rclcpp::node_interfaces::NodeBaseInterface, rclcpp::node_interfaces::NodeLoggingInterface>;

    void node_info(MyNodeInterfaces interfaces)
    {
      auto base_interface = interfaces.get_node_base_interface();
      auto logging_interface = interfaces.get_node_logging_interface();
      RCLCPP_INFO(logging_interface->get_logger(), "Node name: %s", base_interface->get_name());
    }

Instead of accepting ``SharedPtr`` or a node interface, this function takes a reference to a ``rclcpp::node_interfaces::NodeInterfaces`` object.
Another advantage of using this approach is the support for implicit conversion of node-like objects.
This means that it is possible to directly pass any node-like object to a function expecting a ``rclcpp::node_interfaces::NodeInterfaces`` object.

It extracts:

* ``NodeBaseInterface`` Provides basic node functionalities.
* ``NodeLoggingInterface`` Enables logging.

Then, it retrieves and prints the node name.

.. code-block:: c++

    class SimpleNode : public rclcpp::Node
    {
    public:
      SimpleNode(const std::string & node_name)
      : Node(node_name)
      {
      }
    };

    class LifecycleTalker : public rclcpp_lifecycle::LifecycleNode
    {
    public:
      explicit LifecycleTalker(const std::string & node_name, bool intra_process_comms = false)
      : rclcpp_lifecycle::LifecycleNode(node_name,
          rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
      {}
    }

Next, we create a ``rclcpp::Node`` as well as a ``rclcpp_lifecycle::LifecycleNode`` class.
The ``rclcpp_lifecycle::LifecycleNode`` class often includes functions for the state transitions  ``Unconfigured``, ``Inactive``, ``Active``, and ``Finalized``.
However, they are not included for demonstration purposes.

.. code-block:: c++

    int main(int argc, char * argv[])
    {
      rclcpp::init(argc, argv);
      rclcpp::executors::SingleThreadedExecutor exe;
      auto node = std::make_shared<SimpleNode>("Simple_Node");
      auto lc_node = std::make_shared<LifecycleTalker>("Simple_LifeCycle_Node");
      node_info(*node);
      node_info(*lc_node);
    }

In the main function, a ``SharedPtr`` to both ``rclcpp_lifecycle::LifecycleNode`` and ``rclcpp::Node`` is created.
The function declared above is called once with each node type as an argument.

.. note:: The ``SharedPtr`` needs to be dereferenced as the template accepts a reference to the ``NodeT`` object.
