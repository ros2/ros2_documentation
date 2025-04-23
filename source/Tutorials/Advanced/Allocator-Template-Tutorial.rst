.. redirect-from::

    Allocator-Template-Tutorial
    Tutorials/Allocator-Template-Tutorial

Implementing a custom memory allocator
======================================

**Goal:** This tutorial will show how to use a custom memory allocator when writing ROS 2 C++ code.

**Tutorial level:** Advanced

**Time:** 20 minutes

.. contents:: Table of Contents
   :depth: 2
   :local:

This tutorial will teach you how to integrate a custom allocator for publishers and subscribers so that the default heap allocator is never called while your ROS nodes are executing.
The code for this tutorial is available `here <https://github.com/ros2/demos/blob/{REPOS_FILE_BRANCH}/demo_nodes_cpp/src/topics/allocator_tutorial_pmr.cpp>`__.

Background
----------

Suppose you want to write real-time safe code, and you've heard about the many dangers of calling ``new`` during the real-time critical section, because the default heap allocator on most platforms is nondeterministic.

By default, many C++ standard library structures will implicitly allocate memory as they grow, such as ``std::vector``.
However, these data structures also accept an "Allocator" template argument.
If you specify a custom allocator to one of these data structures, it will use that allocator instead of the system allocator to grow or shrink the data structure.
Your custom allocator could have a pool of memory preallocated on the stack, which might be better suited to real-time applications.

In the ROS 2 C++ client library (rclcpp), we are following a similar philosophy to the C++ standard library.
Publishers, subscribers, and the Executor accept an Allocator template parameter that controls allocations made by that entity during execution.

Writing an allocator
--------------------

To write an allocator compatible with ROS 2's allocator interface, your allocator must be compatible with the C++ standard library allocator interface.

Since C++17, the standard library provides something called ``std::pmr::memory_resource``.
This is a class that can be derived from to create a custom allocator that fulfills a minimum set of requirements.

For example, the following declaration for a custom memory resource fulfills the requirements (of course, you would still need to implement the declared functions in this class):

.. code-block:: c++

    class CustomMemoryResource : public std::pmr::memory_resource
    {
    private:
      void * do_allocate(std::size_t bytes, std::size_t alignment) override;

      void do_deallocate(
        void * p, std::size_t bytes,
        std::size_t alignment) override;

      bool do_is_equal(
        const std::pmr::memory_resource & other) const noexcept override;
    };

To learn about the full capabilities of ``std::pmr::memory_resource``, see https://en.cppreference.com/w/cpp/memory/memory_resource.

The full implementation of the custom allocator for this tutorial is in https://github.com/ros2/demos/blob/{REPOS_FILE_BRANCH}/demo_nodes_cpp/src/topics/allocator_tutorial_pmr.cpp.

Writing an example main
-----------------------

Once you have written a valid C++ allocator, you must pass it as a shared pointer to your publisher, subscriber, and executor.
But first, we'll declare a few aliases to shorten the names.

.. code-block:: c++

     using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;
     using Alloc = std::pmr::polymorphic_allocator<void>;
     using MessageAllocTraits =
       rclcpp::allocator::AllocRebind<std_msgs::msg::UInt32, Alloc>;
     using MessageAlloc = MessageAllocTraits::allocator_type;
     using MessageDeleter = rclcpp::allocator::Deleter<MessageAlloc, std_msgs::msg::UInt32>;
     using MessageUniquePtr = std::unique_ptr<std_msgs::msg::UInt32, MessageDeleter>;

Now we can create our resources with the custom allocator:

.. code-block:: c++

     CustomMemoryResource mem_resource{};
     auto alloc = std::make_shared<Alloc>(&mem_resource);
     rclcpp::PublisherOptionsWithAllocator<Alloc> publisher_options;
     publisher_options.allocator = alloc;
     auto publisher = node->create_publisher<std_msgs::msg::UInt32>(
       "allocator_tutorial", 10, publisher_options);

     rclcpp::SubscriptionOptionsWithAllocator<Alloc> subscription_options;
     subscription_options.allocator = alloc;
     auto msg_mem_strat = std::make_shared<
       rclcpp::message_memory_strategy::MessageMemoryStrategy<
         std_msgs::msg::UInt32, Alloc>>(alloc);
     auto subscriber = node->create_subscription<std_msgs::msg::UInt32>(
       "allocator_tutorial", 10, callback, subscription_options, msg_mem_strat);

     std::shared_ptr<rclcpp::memory_strategy::MemoryStrategy> memory_strategy =
       std::make_shared<AllocatorMemoryStrategy<Alloc>>(alloc);

     rclcpp::ExecutorOptions options;
     options.memory_strategy = memory_strategy;
     rclcpp::executors::SingleThreadedExecutor executor(options);

You must also instantiate a custom deleter and allocator for use when allocating messages:

.. code-block:: c++

     MessageDeleter message_deleter;
     MessageAlloc message_alloc = *alloc;
     rclcpp::allocator::set_allocator_for_deleter(&message_deleter, &message_alloc);

Once you've add the node to the executor, it is time to spin.
We'll use the custom allocator to allocate each message:

.. code-block:: c++

     uint32_t i = 0;
     while (rclcpp::ok()) {
       auto ptr = MessageAllocTraits::allocate(message_alloc, 1);
       MessageAllocTraits::construct(message_alloc, ptr);
       MessageUniquePtr msg(ptr, message_deleter);
       msg->data = i;
       ++i;
       publisher->publish(std::move(msg));
       rclcpp::sleep_for(10ms);
       executor.spin_some();
     }

Passing an allocator to the intra-process pipeline
--------------------------------------------------

Even though we instantiated a publisher and subscriber in the same process, we aren't using the intra-process pipeline yet.

The IntraProcessManager is a class that is usually hidden from the user, but in order to pass a custom allocator to it we need to expose it by getting it from the rclcpp Context.
The IntraProcessManager makes use of several standard library structures, so without a custom allocator it will call the default ``new``.

.. code-block:: c++

    auto context = rclcpp::contexts::get_global_default_context();
    auto options = rclcpp::NodeOptions()
      .context(context)
      .use_intra_process_comms(true);
    auto node = rclcpp::Node::make_shared("allocator_example", options);

Make sure to instantiate publishers and subscribers AFTER constructing the node in this way.

Testing and verifying the code
------------------------------

How do you know that your custom allocator is actually getting called?

The obvious thing to do would be to count the calls made to your custom allocator's ``allocate`` and ``deallocate`` functions and compare that to the calls to ``new`` and ``delete``.

Adding counting to the custom allocator is easy:

.. code-block:: c++

     void * do_allocate(std::size_t size, std::size_t alignment) override
     {
       // ...
       num_allocs++;
       // ...
     }

     void do_deallocate(
       void * p, std::size_t bytes,
       std::size_t alignment) override
     {
       // ...
       num_deallocs++;
       // ...
     }

You can also override the global ``new`` and ``delete`` operators:

.. code-block:: c++

     void * operator new(std::size_t size)
     {
       if (is_running) {
         global_runtime_allocs++;
       }
       return std::malloc(size);
     }

     void operator delete(void * ptr, size_t) noexcept
     {
       if (ptr != nullptr) {
         if (is_running) {
           global_runtime_deallocs++;
         }
         std::free(ptr);
       }
     }

     void operator delete(void * ptr) noexcept
     {
       if (ptr != nullptr) {
         if (is_running) {
           global_runtime_deallocs++;
         }
         std::free(ptr);
       }
     }

where the variables we are incrementing are just global static integers, and ``is_running`` is a global static boolean that gets toggled right before the call to ``spin``.

The `example executable <https://github.com/ros2/demos/blob/{REPOS_FILE_BRANCH}/demo_nodes_cpp/src/topics/allocator_tutorial_pmr.cpp>`__ prints the value of the variables.
To run the example executable, use:

.. code-block:: console

     $ ros2 run demo_nodes_cpp allocator_tutorial

or, to run the example with the intra-process pipeline on:

.. code-block:: console

     $ ros2 run demo_nodes_cpp allocator_tutorial intra
     Global new was called 15590 times during spin
     Global delete was called 15590 times during spin
     Allocator new was called 27284 times during spin
     Allocator delete was called 27281 times during spin

We've caught about 2/3 of the allocations/deallocations that happen on the execution path, but where do the remaining 1/3 come from?

As a matter of fact, these allocations/deallocations originate in the underlying DDS implementation used in this example.

Proving this is out of the scope of this tutorial, but you can check out the test for the allocation path that gets run as part of the ROS 2 continuous integration testing, which backtraces through the code and figures out whether certain function calls originate in the rmw implementation or in a DDS implementation:

https://github.com/ros2/realtime_support/blob/{REPOS_FILE_BRANCH}/tlsf_cpp/test/test_tlsf.cpp#L41

Note that this test is not using the custom allocator we just created, but the TLSF allocator (see below).

The TLSF allocator
------------------

ROS 2 offers support for the TLSF (Two Level Segregate Fit) allocator, which was designed to meet real-time requirements:

https://github.com/ros2/realtime_support/tree/{REPOS_FILE_BRANCH}/tlsf_cpp

For more information about TLSF, see `this page via Universitat Politècnica de València <http://www.gii.upv.es/tlsf/>`_.

Note that the TLSF allocator is licensed under a dual-GPL/LGPL license.

A full working example using the TLSF allocator is here:
https://github.com/ros2/realtime_support/blob/{REPOS_FILE_BRANCH}/tlsf_cpp/example/allocator_example.cpp
