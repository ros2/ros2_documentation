.. _RealTimeTutorial:

Real-Time Processing with Multiple Threads
==========================================

.. contents:: Table of Contents
   :local:



Introduction
------------

In this tutorial you will learn how to design a ROS 2 application with real-time and non real-time requirements. The simplest approach is to define a ROS 2 nodes and call the corresponding Executor in multiple threads with different real-time priorities. The limitation is that all callbacks (subscription, timers etc.) of the ROS 2 node are processed with the same priority. If you have multiple callbacks within
a ROS 2 node with different timing requirements, e.g. one real-time callback and one non real-time callback, then see Tutorial :doc:`Real-Time Processing with Callback Groups <../Real-Time-Processing-with-Callback-Groups>`.

This tutorial is specific for Linux.


Application
-----------

The application consists of two publishers and two subscriptions. At the subscriber side: one subscription callback shall be processed by a real-time prioritized thread, and the other one shall be processed by a thread with default scheduling priority. 

To demonstrate the real-time capability we also need to provide a test harness: It would require a larger test setup to actually compare the time durations of these two subscriptions. However, with a simple metric we can indirectly demonstrate it. In Linux scheduling, higher prioritized threads could interrupt lower prioritized ones. The interruption leads to a context switch, which can be detected.  This fact can be exploited to show the different thread priorities: We simply count how many times the thread with the subscription was interrupted by other threads. A real-time thread, which has a higher priority than all user threads, will not be interrupted by other user threads. However it can be expected to see some involuntary context switches even with a real-time priority. There might be other real-time threads with higher priority or interrupts preempting the real-time callback execution. Therefore we expect significantly fewer context switched for the real-time subscription callback compared to the non real-time subscription callback.

The complete source file of this tutorial is available `here <https://github.com/ros-realtime/ros2-realtime-examples/tree/rolling/minimal_scheduling/minimal_scheduling_real_time_executor.cpp>`_.


Subscriber configuration
------------------------

First, a subscriber is defined:

.. code-block:: cpp

   class MinimalSubscriber : public rclcpp::Node
   {
   public:
   MinimalSubscriber(const std::string & node_name, const std::string & topic_name)
   : Node(node_name), context_switches_counter_(RUSAGE_THREAD)
   {
      subscription1_ = this->create_subscription<std_msgs::msg::String>(
         topic_name,
         10,
         [this](std_msgs::msg::String::UniquePtr) {
         auto context_switches = context_switches_counter_.get();
         if (context_switches > 0L) {
            RCLCPP_WARN(
               this->get_logger(), "[sub]    Involuntary context switches: '%lu'",
               context_switches);
         } else {
            RCLCPP_INFO(
               this->get_logger(), "[sub]    Involuntary context switches: '%lu'",
               context_switches);
         }
         burn_cpu_cycles(200ms);
         });
   }

   private:
   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription1_;
   ContextSwitchesCounter context_switches_counter_;
   };

To test the real-time setup, the number of preemptions by other kernel processes is used as a metric
(`context_switches_counter_.get`). 

To assign the real-time priority of the thread, the parameters of the scheduler and scheduling priority 
are passed as option, when executing the ROS 2 program: 

.. code-block:: bash
   ros2 run minimal_scheduling minimal_scheduling_real_time_tutorial --sched SCHED_FIFO --priority 80

In the main function, these parameters are parsed using the library function (`SchedOptionReader`).

.. code-block:: cpp

   int main(int argc, char * argv[])
   {
   // Force flush of the stdout buffer.
   setvbuf(stdout, NULL, _IONBF, BUFSIZ);

   auto options_reader = SchedOptionsReader();
   if (!options_reader.read_options(argc, argv)) {
      options_reader.print_usage();
      return 0;
   }
   auto options = options_reader.get_options();

Then, the Publisher and Subscribers are defined. Here, the Publisher will periodically send the two messages, `topic` and `topic_rt`.


.. code-block:: cpp

   rclcpp::init(argc, argv);

   auto node_pub = std::make_shared<MinimalPublisher>();
   auto node_sub = std::make_shared<MinimalSubscriber>("minimal_sub1", "topic");
   auto node_sub_rt = std::make_shared<MinimalSubscriber>("minimal_sub2", "topic_rt");


Executor configuration
----------------------

To configure the execution management in ROS 2, a default Executor and a real-time Executor are defined. At this moment, it is only a name, no real-time criticality is configured. In terms of ROS 2 both are an instance of a Static Single-Threaded-Executor. However, you could also use the MultiThreadedExecutor. In a second step, Publisher node and the non real-time Subscriber are added to the `default_executor`. The real-time Subscription is added to the `realtime_executor`:

.. code-block:: cpp

   rclcpp::executors::StaticSingleThreadedExecutor default_executor;
   rclcpp::executors::StaticSingleThreadedExecutor realtime_executor;

   // the publisher and non real-time subscriber are processed by default_executor
   default_executor.add_node(node_pub);
   default_executor.add_node(node_sub);

   // real-time subscriber is processed by realtime_executor.
   realtime_executor.add_node(node_sub_rt);


Thread configuration
----------------------
The operating system provides multi-threading by means of creating different threads. These threads can be configured in terms of their scheduling policy. Therefore we will now create two threads: one thread in which the `default_executor` will spin; and one in which the `realtime_executor` will spin.

.. code-block:: cpp

   // processing of normal callbacks in first thread
   auto default_thread = std::thread(
      [&]() {
         default_executor.spin();
      });

   // processing of real-time callbacks in second thread
   auto realtime_thread = std::thread(
      [&]() {
         realtime_executor.spin();
      });

   set_thread_scheduling(realtime_thread.native_handle(), options.policy, options.priority);


Then, we assign to the `realtime_thread` the priority, that was passed by the command line argument. We are not assigning any scheduling parameter to the `default_thread`, so that the Publisher and the non real-time Subscription will be processed with default Linux scheduling settings.

Note, that the thread function (only) calls the `spin()`-function of the Executor. This implies that all entities that are managed by the Executor are processed in this thread. This gives us now the opportunity to manage the priority by which these callbacks are processed by the Linux operating system. For example, if we increased the default priority to a higher priority, then all by this Executor managed callbacks would be processed with a higher priority.

The priority of the middleware threads are not changed by this setup. See this `example <https://github.com/ros-realtime/ros2-realtime-examples/tree/rolling/minimal_scheduling/minimal_scheduling_real_time_executor.cpp>`_., how to change the priority of the middleware threads. The trick is to change the priority of the main thread before calling `rclcpp::init(argc,argv)`, in which all middleware threads are created with the same priority as the main thread.


Note also, that only entire nodes can be added to an Executor. Thhis implies that the scheduling policy applies to all entities (timers, subscriptions, etc.) of a node, that were added to the Executor. A differentiation of real-time and non real-time within a node is not possible with this approach. In this case see this tutorial :doc:`Real-Time Processing with Callback Groups <../Real-Time-Processing-with-Callback-Groups>`.

For simplicity, we are using a helper-function `set_thread_scheduling` to set the scheduling parameter of the thread. It calls the POSIX function `pthread_setschedparam` to assign the scheduler type and scheduling priority to a thread:

.. code-block:: cpp

   void set_thread_scheduling(std::thread::native_handle_type thread, int policy, int sched_priority)
   {
      struct sched_param param;
      param.sched_priority = sched_priority;
      auto ret = pthread_setschedparam(thread, policy, &param);
      if (ret > 0) {
         throw std::runtime_error("Couldn't set scheduling priority and policy. Error code: " + std::string(strerror(errno)));
      }
   }

It is important to understand, that there is no real-time Executor in ROS 2, but only by running the `executor.spin()` function inside a real-time Linux thread makes it real-time capable.



Output
------

We run the example with default and a real-time priority (80). The output shows the number of context switches of other kernel processes during computation. The number of context switches of the real-time callback (minimal_sub2) is reduced to zero compared to 8-49 context switched for the non real-time configuration (minimal_sub1).

.. code-block:: bash

   ros2 run minimal_scheduling minimal_scheduling_real_time_tutorial
   [WARN] [1680948979.971439054] [minimal_sub1]: [sub]    Involuntary context switches: '25'
   [WARN] [1680948979.971483357] [minimal_sub2]: [sub]    Involuntary context switches: '20'
   [WARN] [1680948980.473828433] [minimal_sub1]: [sub]    Involuntary context switches: '23'
   [WARN] [1680948980.473872245] [minimal_sub2]: [sub]    Involuntary context switches: '21'
   [WARN] [1680948980.972909968] [minimal_sub1]: [sub]    Involuntary context switches: '26'
   [WARN] [1680948980.973096277] [minimal_sub2]: [sub]    Involuntary context switches: '15'

   ros2 run minimal_scheduling minimal_scheduling_real_time_tutorial --sched SCHED_FIFO --priority 80
   [WARN] [1680947876.099416572] [minimal_sub1]: [sub]    Involuntary context switches: '8'
   [INFO] [1680947876.099471567] [minimal_sub2]: [sub]    Involuntary context switches: '0'
   [WARN] [1680947876.599197932] [minimal_sub1]: [sub]    Involuntary context switches: '49'
   [INFO] [1680947876.599202498] [minimal_sub2]: [sub]    Involuntary context switches: '0'
   [WARN] [1680947877.101378852] [minimal_sub1]: [sub]    Involuntary context switches: '25'
   [INFO] [1680947877.101372018] [minimal_sub2]: [sub]    Involuntary context switches: '0'


