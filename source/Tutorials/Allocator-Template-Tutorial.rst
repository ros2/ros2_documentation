.. redirect-from::

    Allocator-Template-Tutorial

Implement a custom memory allocator
===================================

.. contents:: Table of Contents
   :depth: 1
   :local:

This tutorial will teach you how to integrate a custom allocator for publishers and subscribers so that the default heap allocator is never called while your ROS nodes are executing.
The code for this tutorial is available `here <https://github.com/ros2/demos/blob/master/demo_nodes_cpp/src/topics/allocator_tutorial.cpp>`__.

Background
----------

Suppose you want to write real-time safe code, and you've heard about the many dangers of calling "new" during the real-time critical section, because the default heap allocator on most platforms is nondeterministic.

By default, many C++ standard library structures will implicitly allocate memory as they grow, such as ``std::vector``. However, these data structures also accept an "Allocator" template argument. If you specify a custom allocator to one of these data structures, it will use that allocator for you instead of the system allocator to grow or shrink the data structure. Your custom allocator could have a pool of memory preallocated on the stack, which might be better suited to real-time applications.

In the ROS 2 C++ client library (rclcpp), we are following a similar philosophy to the C++ standard library. Publishers, subscribers, and the Executor accept an Allocator template parameter that controls allocations made by that entity during execution.

Writing an allocator
--------------------

To write an allocator compatible with ROS 2's allocator interface, your allocator must be compatible with the C++ standard library allocator interface.

The C++11 library provides something called ``allocator_traits``. The C++11 standard specifies that a custom allocator only needs to fulfil a minimal set of requirements to be used to allocate and deallocate memory in a standard way. ``allocator_traits`` is a generic structure that fills out other qualities of an allocator based on an allocator written with the minimal requirements.

For example, the following declaration for a custom allocator would satisfy ``allocator_traits`` (of course, you would still need to implement the declared functions in this struct):

.. code-block:: c++

   template <class T>
   struct custom_allocator {
     using value_type = T;
     custom_allocator() noexcept;
     template <class U> custom_allocator (const custom_allocator<U>&) noexcept;
     T* allocate (std::size_t n);
     void deallocate (T* p, std::size_t n);
   };

   template <class T, class U>
   constexpr bool operator== (const custom_allocator<T>&, const custom_allocator<U>&) noexcept;

   template <class T, class U>
   constexpr bool operator!= (const custom_allocator<T>&, const custom_allocator<U>&) noexcept;

You could then access other functions and members of the allocator filled in by ``allocator_traits`` like so: ``std::allocator_traits<custom_allocator<T>>::construct(...)``

To learn about the full capabilities of ``allocator_traits``, see https://en.cppreference.com/w/cpp/memory/allocator_traits .

However, some compilers that only have partial C++11 support, such as GCC 4.8, still require allocators to implement a lot of boilerplate code to work with standard library structures such as vectors and strings, because these structures do not use ``allocator_traits`` internally. Therefore, if you're using a compiler with partial C++11 support, your allocator will need to look more like this:

.. code-block:: c++

   template<typename T>
   struct pointer_traits {
     using reference = T &;
     using const_reference = const T &;
   };

   // Avoid declaring a reference to void with an empty specialization
   template<>
   struct pointer_traits<void> {
   };

   template<typename T = void>
   struct MyAllocator : public pointer_traits<T> {
   public:
     using value_type = T;
     using size_type = std::size_t;
     using pointer = T *;
     using const_pointer = const T *;
     using difference_type = typename std::pointer_traits<pointer>::difference_type;

     MyAllocator() noexcept;

     ~MyAllocator() noexcept;

     template<typename U>
     MyAllocator(const MyAllocator<U> &) noexcept;

     T * allocate(size_t size, const void * = 0);

     void deallocate(T * ptr, size_t size);

     template<typename U>
     struct rebind {
       typedef MyAllocator<U> other;
     };
   };

   template<typename T, typename U>
   constexpr bool operator==(const MyAllocator<T> &,
     const MyAllocator<U> &) noexcept;

   template<typename T, typename U>
   constexpr bool operator!=(const MyAllocator<T> &,
     const MyAllocator<U> &) noexcept;

Writing an example main
-----------------------

Once you have written a valid C++ allocator, you must pass it as a shared pointer to your publisher, subscriber, and executor.

.. code-block:: c++

     auto alloc = std::make_shared<MyAllocator<void>>();
     auto publisher = node->create_publisher<std_msgs::msg::UInt32>("allocator_example", 10, alloc);
     auto msg_mem_strat =
       std::make_shared<rclcpp::message_memory_strategy::MessageMemoryStrategy<std_msgs::msg::UInt32,
       MyAllocator<>>>(alloc);
     auto subscriber = node->create_subscription<std_msgs::msg::UInt32>(
       "allocator_example", 10, callback, nullptr, false, msg_mem_strat, alloc);

     std::shared_ptr<rclcpp::memory_strategy::MemoryStrategy> memory_strategy =
       std::make_shared<AllocatorMemoryStrategy<MyAllocator<>>>(alloc);
     rclcpp::executors::SingleThreadedExecutor executor(memory_strategy);

You will also need to use your allocator to allocate any messages that you pass along the execution codepath.

.. code-block:: c++

     auto alloc = std::make_shared<MyAllocator<void>>();

Once you've instantiated the node and added the executor to the node, it's time to spin:

.. code-block:: c++

     uint32_t i = 0;
     while (rclcpp::ok()) {
       msg->data = i;
       i++;
       publisher->publish(msg);
       rclcpp::utilities::sleep_for(std::chrono::milliseconds(1));
       executor.spin_some();
     }

Passing an allocator to the intra-process pipeline
--------------------------------------------------

Even though we instantiated a publisher and subscriber in the same process, we aren't using the intra-process pipeline yet.

The IntraProcessManager is a class that is usually hidden from the user, but in order to pass a custom allocator to it we need to expose it by getting it from the rclcpp Context. The IntraProcessManager makes use of several standard library structures, so without a custom allocator it will call the default new.

.. code-block:: c++

     auto context = rclcpp::contexts::default_context::get_global_default_context();
     auto ipm_state =
       std::make_shared<rclcpp::intra_process_manager::IntraProcessManagerState<MyAllocator<>>>();
     // Constructs the intra-process manager with a custom allocator.
     context->get_sub_context<rclcpp::intra_process_manager::IntraProcessManager>(ipm_state);
     auto node = rclcpp::Node::make_shared("allocator_example", true);

Make sure to instantiate publishers and subscribers AFTER constructing the node in this way.

Testing and verifying the code
------------------------------

How do you know that your custom allocator is actually getting called?

The obvious thing to do would be to count the calls made to your custom allocator's ``allocate`` and ``deallocate`` functions and compare that to the calls to ``new`` and ``delete``.

Adding counting to the custom allocator is easy:

.. code-block:: c++

     T * allocate(size_t size, const void * = 0) {
       // ...
       num_allocs++;
       // ...
     }

     void deallocate(T * ptr, size_t size) {
       // ...
       num_deallocs++;
       // ...
     }

You can also override the global new and delete operators:

.. code-block:: c++

   void operator delete(void * ptr) noexcept {
     if (ptr != nullptr) {
       if (is_running) {
         global_runtime_deallocs++;
       }
       std::free(ptr);
       ptr = nullptr;
     }
   }

   void operator delete(void * ptr, size_t) noexcept {
     if (ptr != nullptr) {
       if (is_running) {
         global_runtime_deallocs++;
       }
       std::free(ptr);
       ptr = nullptr;
     }
   }

where the variables we are incrementing are just global static integers, and ``is_running`` is a global static boolean that gets toggled right before the call to ``spin``.

The `example executable <https://github.com/ros2/demos/blob/master/demo_nodes_cpp/src/topics/allocator_tutorial.cpp>`__ prints the value of the variables. To run the example executable, use:

.. code-block:: bash

   allocator_example

or, to run the example with the intra-process pipeline on:

.. code-block:: bash

   allocator_example intra-process

You should get numbers like:

.. code-block:: bash

   Global new was called 15590 times during spin
   Global delete was called 15590 times during spin
   Allocator new was called 27284 times during spin
   Allocator delete was called 27281 times during spin

We've caught about 2/3 of the allocations/deallocations that happen on the execution path, but where do the remaining 1/3 come from?

As a matter of fact, these allocations/deallocations originate in the underlying DDS implementation used in this example.

Proving this is out of the scope of this tutorial, but you can check out the test for the allocation path that gets run as part of the ROS 2 continuous integration testing, which backtraces through the code and figures out whether certain function calls originate in the rmw implementation or in a DDS implementation:

https://github.com/ros2/realtime_support/blob/master/tlsf_cpp/test/test_tlsf.cpp#L41

Note that this test is not using the custom allocator we just created, but the TLSF allocator (see below).

The TLSF allocator
------------------

ROS 2 offers support for the TLSF (Two Level Segregate Fit) allocator, which was designed to meet real-time requirements:

https://github.com/ros2/realtime_support/tree/master/tlsf_cpp

For more information about TLSF, see http://www.gii.upv.es/tlsf/

Note that the TLSF allocator is licensed under a dual-GPL/LGPL license.

A full working example using the TLSF allocator is here:
https://github.com/ros2/realtime_support/blob/master/tlsf_cpp/example/allocator_example.cpp
