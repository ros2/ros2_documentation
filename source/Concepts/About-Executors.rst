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

The Single-Threaded Executor is also used by the container process for :doc:`components <./About-Composition>`, i.e. in all cases where nodes are created and executed without an explicit main function.

Other Executors and advanced use
--------------------------------

Currently, rclcpp provides three Executor types, derived from a shared parent class:

.. code-block:: text

                                      Executor
                                         ^
                                         |
               /-------------------------|----------------------------\
               |                         |                            |
     SingleThreadedExecutor     MultiThreadedExecutor     StaticSingleThreadedExecutor

The *Multi-Threaded Executor* creates a configurable number of threads to allow for processing multiple messages or events in parallel. The *Static Single-Threaded Executor* optimizes the runtime costs for scanning the structure of a node in terms of subscriptions, timers, service servers, action servers, etc. It performs this scan only once when the node is added, while the other two executors regularly scan for such changes. Therefore, the Static Single-Threaded Executor should be used only with nodes that create all subscriptions, timers, etc. during initialization.

All three executors can be used with multiple nodes by calling *add_node* for each node. 

.. code-block:: cpp

   rclcpp::Node::SharedPtr node1 = ...
   rclcpp::Node::SharedPtr node2 = ...
   rclcpp::Node::SharedPtr node3 = ...

   rclcpp::executors::StaticSingleThreadedExecutor executor;
   executor.add_node(node1);
   executor.add_node(node2);
   executor.add_node(node2);
   executor.spin();

In the above example, the one thread of a Static Single-Threaded Executor is used to serve three nodes together. In case of a Multi-Threaded Executor, the actual parallelism depends on the callback groups.

Callback groups
---------------

The rclcpp allows assigning callbacks to different callback groups of a node. Such a callback group can be created by the ``create_callback_group`` function of the Node class. The callback group can be specified when creating a subscription, timer, etc. - for example by the subscription options:

.. code-block:: cpp

   auto my_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

   rclcpp::SubscriptionOptions options;
   options.callback_group = my_callback_group;

   my_subscription = create_subscription<Int32>("/topic", rclcpp::SensorDataQoS(), 
                                                callback, options);

If no callback group TODO ...

Rclcpp differentiates between two types of callback groups, specified by at instantiation time:

* *Mutually exclusive:* The callbacks of this group must not be executed in parallel.
* *Reentrant:* The callbacks of this group may be executed in parallel.

The 

Scheduling semantics
--------------------

Add text here ...

Outlook
-------

Add text here ...

Further information
-------------------

Add text here ...