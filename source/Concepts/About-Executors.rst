.. _Executors:

Executors
=========

.. contents:: Table of Contents
   :local:

Overview
--------

Execution management in ROS 2 is explicated by the concept of Executors.
An Executor uses one or more threads of the underlying operating system to invoke the callbacks of subscriptions, timers, service servers, action servers, etc. on incoming messages and events.
The explicit Executor class (in `executor.hpp <https://github.com/ros2/rclcpp/blob/master/rclcpp/include/rclcpp/executor.hpp>`_ in rclcpp, in `executors.py <https://github.com/ros2/rclpy/blob/master/rclpy/rclpy/executors.py>`_ in rclpy, or in `executor.h <https://github.com/ros2/rclc/blob/master/rclc/include/rclc/executor.h>`_ in rclc) provides more control over execution management than the spin mechanism in ROS 1, although the basic API is very similar.

In the following, we focus on the C++ Client Library *rclcpp*.

Simple use
----------

In the simplest case, the main thread is used for processing the incoming messages and events of a Node by calling the *spin* function as follows:

.. code-block:: cpp

   int main(int argc, char* argv[])
   {
      // Some initialization.
      rclcpp::init(argc, argv);
      ...

      // Instantiate a node.
      rclcpp::Node::SharedPtr node = ...

      // Run the executor. 
      rclcpp::spin(node);
      
      // Shutdown and exit.
      ...
      return 0;
   }

The call to *spin* basically expands to an instatiation and invokation of the Single-Threaded Executor, which is the simples Executor:

.. code-block:: cpp

   rclcpp::executors::SingleThreadedExecutor executor;
   executor.add_node(node);
   executor.spin();

By invoking *spin* of the Executor instance, the current thread starts querying the rcl and middleware layers for incoming messages and other events and calls the corresponding callback functions until the node shuts down. In order not to counteract the QoS settings of the middleware, an incoming message is not stored in a queue on the Client Library layer but kept in the middleware until it is taken for processing by a callback function. (This is a crucial difference to ROS 1.) A *wait set* is used to inform the Executor about available messages on the middleware layer, with one binary flag per queue.

.. image:: images/executors_basic_principle.png

Composition ...

Multiple nodes served by one executor ...

Other Executors
---------------

Add text here ...

Callback groups
---------------

Add text here ...

Scheduling semantics
--------------------

Add text here ...

Outlook
-------

Add text here ...

Further information
-------------------

Add text here ...