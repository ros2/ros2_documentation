Using the Node Interfaces Class (C++)
=====================================

.. contents:: Table of Contents
   :depth: 2
   :local:

Using ``SharedPtr``
-------------------

The ``rclcpp::node_interfaces`` template class provides an easy and compact mechanism for aggregating Node Interfaces of node-like objects.

Lets say that you need to create a function that take as an argument a node object.

In the code below, we create a simple ``Node`` called ``Simple_Node`` and a function ``node_info`` that accepts a ``SharedPtr`` to the ``Node`` and prints the name of the ``Node``.

.. code-block:: c++

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

    private:
    };

    int main(int argc, char * argv[])
    {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleNode>();
    node_info(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
    }

.. code-block:: console
    
    [INFO] [Simple_Node]: Node name: Simple_Node


After running the ``Node``, we can see that we have access to the ``Node`` member functions. 
Unfortunately, this is only possible for ``rclcpp::Node`` and does not work for other types, such as ``rclcpp_lifecycle::LifecycleNode`` since they do not share the same inheritance tree.

Using ``rclcpp::node_interfaces::NodeInterfaces``
-------------------------------------------------

Another way of accessing a Node is through the ``Node Interfaces``.







.. code-block:: c++

    #include "rclcpp/rclcpp.hpp"
    #include "rclcpp/node_interfaces/node_interfaces.hpp"


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
    node_info(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
    }