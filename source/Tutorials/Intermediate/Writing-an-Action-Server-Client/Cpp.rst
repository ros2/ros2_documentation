.. redirect-from::

    Tutorials/Actions/Writing-a-Cpp-Action-Server-Client

.. _ActionsCpp:

Writing an action server and client (C++)
=========================================

**Goal:** Implement an action server and client in C++.

**Tutorial level:** Intermediate

**Time:** 15 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

Actions are a form of asynchronous communication in ROS.
*Action clients* send goal requests to *action servers*.
*Action servers* send goal feedback and results to *action clients*.

Prerequisites
-------------

You will need the ``action_tutorials_interfaces`` package and the ``Fibonacci.action``
interface defined in the previous tutorial, :doc:`../Creating-an-Action`.

Tasks
-----

1 Creating the action_tutorials_cpp package
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

As we saw in the :doc:`../../Beginner-Client-Libraries/Creating-Your-First-ROS2-Package` tutorial, we need to create a new package to hold our C++ and supporting code.

1.1 Creating the action_tutorials_cpp package
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Go into the action workspace you created in the :doc:`previous tutorial <../Creating-an-Action>` (remember to source the workspace), and create a new package for the C++ action server:


.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

      cd ~/ros2_ws/src
      ros2 pkg create --dependencies action_tutorials_interfaces rclcpp rclcpp_action rclcpp_components -- action_tutorials_cpp

  .. group-tab:: macOS

    .. code-block:: bash

      cd ~/ros2_ws/src
      ros2 pkg create --dependencies action_tutorials_interfaces rclcpp rclcpp_action rclcpp_components -- action_tutorials_cpp

  .. group-tab:: Windows

    .. code-block:: bash

      cd \dev\ros2_ws\src
      ros2 pkg create --dependencies action_tutorials_interfaces rclcpp rclcpp_action rclcpp_components -- action_tutorials_cpp

1.2 Adding in visibility control
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In order to make the package compile and work on Windows, we need to add in some "visibility control".
For more details, see :ref:`Windows Symbol Visibility in the Windows Tips and Tricks document <Windows_Symbol_Visibility>`.

Open up ``action_tutorials_cpp/include/action_tutorials_cpp/visibility_control.h``, and put the following code in:

.. rli:: https://github.com/ros2/demos/raw/9c4ced3c5be392145312e0c0d3653140a2e29cc0/action_tutorials/action_tutorials_cpp/include/action_tutorials_cpp/visibility_control.h
    :caption: `action_tutorials/action_tutorials_cpp/include/action_tutorials_cpp/visibility_control.h <https://github.com/ros2/demos/blob/9c4ced3c5be392145312e0c0d3653140a2e29cc0/action_tutorials/action_tutorials_cpp/include/action_tutorials_cpp/visibility_control.h>`_
    :language: c++
    :lines: 15-


2 Writing an action server
^^^^^^^^^^^^^^^^^^^^^^^^^^

Let's focus on writing an action server that computes the Fibonacci sequence using the action we created in the :doc:`../Creating-an-Action` tutorial.

2.1 Writing the action server code
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Open up ``action_tutorials_cpp/src/fibonacci_action_server.cpp``, and put the following code in:

.. rli:: https://github.com/ros2/demos/raw/9c4ced3c5be392145312e0c0d3653140a2e29cc0/action_tutorials/action_tutorials_cpp/src/fibonacci_action_server.cpp
    :caption: `action_tutorials/action_tutorials_cpp/src/fibonacci_action_server.cpp <https://github.com/ros2/demos/blob/9c4ced3c5be392145312e0c0d3653140a2e29cc0/action_tutorials/action_tutorials_cpp/src/fibonacci_action_server.cpp>`_
    :language: c++
    :lines: 15-

The first few lines include all of the headers we need to compile.

Next we create a class that is a derived class of ``rclcpp::Node``:

.. rli:: https://github.com/ros2/demos/raw/9c4ced3c5be392145312e0c0d3653140a2e29cc0/action_tutorials/action_tutorials_cpp/src/fibonacci_action_server.cpp
    :language: c++
    :lines: 27

The constructor for the ``FibonacciActionServer`` class initializes the node name as ``fibonacci_action_server``:

.. rli:: https://github.com/ros2/demos/raw/9c4ced3c5be392145312e0c0d3653140a2e29cc0/action_tutorials/action_tutorials_cpp/src/fibonacci_action_server.cpp
    :language: c++
    :lines: 34-35

The constructor also instantiates a new action server:

.. rli:: https://github.com/ros2/demos/raw/9c4ced3c5be392145312e0c0d3653140a2e29cc0/action_tutorials/action_tutorials_cpp/src/fibonacci_action_server.cpp
    :language: c++
    :lines: 39-47

An action server requires 6 things:

1. The templated action type name: ``Fibonacci``.
2. A ROS 2 node to add the action to: ``this``.
3. The action name: ``'fibonacci'``.
4. A callback function for handling goals: ``handle_goal``
5. A callback function for handling cancellation: ``handle_cancel``.
6. A callback function for handling goal accept: ``handle_accept``.

The implementation of the various callbacks is next in the file.
Note that all of the callbacks need to return quickly, otherwise we risk starving the executor.

We start with the callback for handling new goals:

.. rli:: https://github.com/ros2/demos/raw/9c4ced3c5be392145312e0c0d3653140a2e29cc0/action_tutorials/action_tutorials_cpp/src/fibonacci_action_server.cpp
    :language: c++
    :lines: 54-59,66-67

This implementation just accepts all goals.

Next up is the callback for dealing with cancellation:

.. rli:: https://github.com/ros2/demos/raw/9c4ced3c5be392145312e0c0d3653140a2e29cc0/action_tutorials/action_tutorials_cpp/src/fibonacci_action_server.cpp
    :language: c++
    :lines: 70-76

This implementation just tells the client that it accepted the cancellation.

The last of the callbacks accepts a new goal and starts processing it:

.. rli:: https://github.com/ros2/demos/raw/9c4ced3c5be392145312e0c0d3653140a2e29cc0/action_tutorials/action_tutorials_cpp/src/fibonacci_action_server.cpp
    :language: c++
    :lines: 79-84

Since the execution is a long-running operation, we spawn off a thread to do the actual work and return from ``handle_accepted`` quickly.

All further processing and updates are done in the ``execute`` method in the new thread:

.. rli:: https://github.com/ros2/demos/raw/9c4ced3c5be392145312e0c0d3653140a2e29cc0/action_tutorials/action_tutorials_cpp/src/fibonacci_action_server.cpp
    :language: c++
    :lines: 87-121

This work thread processes one sequence number of the Fibonacci sequence every second, publishing a feedback update for each step.
When it has finished processing, it marks the ``goal_handle`` as succeeded, and quits.

We now have a fully functioning action server.  Let's get it built and running.

2.2 Compiling the action server
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In the previous section we put the action server code into place.
To get it to compile and run, we need to do a couple of additional things.

First we need to setup the CMakeLists.txt so that the action server is compiled.
Open up ``action_tutorials_cpp/CMakeLists.txt``, and add the following right after the ``find_package`` calls:

.. code-block:: cmake

  add_library(action_server SHARED
    src/fibonacci_action_server.cpp)
  target_include_directories(action_server PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_definitions(action_server
    PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
  ament_target_dependencies(action_server
    "action_tutorials_interfaces"
    "rclcpp"
    "rclcpp_action"
    "rclcpp_components")
  rclcpp_components_register_node(action_server PLUGIN "action_tutorials_cpp::FibonacciActionServer" EXECUTABLE fibonacci_action_server)
  install(TARGETS
    action_server
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

And now we can compile the package.  Go to the top-level of the ``ros2_ws``, and run:

.. code-block:: bash

  colcon build

This should compile the entire workspace, including the ``fibonacci_action_server`` in the ``action_tutorials_cpp`` package.

2.3 Running the action server
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Now that we have the action server built, we can run it.
Source the workspace we just built (``ros2_ws``), and try to run the action server:

.. code-block:: bash

  ros2 run action_tutorials_cpp fibonacci_action_server

3 Writing an action client
^^^^^^^^^^^^^^^^^^^^^^^^^^

3.1 Writing the action client code
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Open up ``action_tutorials_cpp/src/fibonacci_action_client.cpp``, and put the following code in:

.. rli:: https://github.com/ros2/demos/raw/9c4ced3c5be392145312e0c0d3653140a2e29cc0/action_tutorials/action_tutorials_cpp/src/fibonacci_action_client.cpp
    :caption: `action_tutorials/action_tutorials_cpp/src/fibonacci_action_client.cpp <https://github.com/ros2/demos/blob/9c4ced3c5be392145312e0c0d3653140a2e29cc0/action_tutorials/action_tutorials_cpp/src/fibonacci_action_client.cpp>`_
    :language: c++
    :lines: 15-

The first few lines include all of the headers we need to compile.

Next we create a class that is a derived class of ``rclcpp::Node``:

.. rli:: https://github.com/ros2/demos/raw/9c4ced3c5be392145312e0c0d3653140a2e29cc0/action_tutorials/action_tutorials_cpp/src/fibonacci_action_client.cpp
    :language: c++
    :lines: 29

The constructor for the ``FibonacciActionClient`` class initializes the node name as ``fibonacci_action_client``:

.. rli:: https://github.com/ros2/demos/raw/9c4ced3c5be392145312e0c0d3653140a2e29cc0/action_tutorials/action_tutorials_cpp/src/fibonacci_action_client.cpp
    :language: c++
    :lines: 36-37

The constructor also instantiates a new action client:

.. rli:: https://github.com/ros2/demos/raw/9c4ced3c5be392145312e0c0d3653140a2e29cc0/action_tutorials/action_tutorials_cpp/src/fibonacci_action_client.cpp
    :language: c++
    :lines: 39-44

An action client requires 3 things:

1. The templated action type name: ``Fibonacci``.
2. A ROS 2 node to add the action client to: ``this``.
3. The action name: ``'fibonacci'``.

We also instantiate a ROS timer that will kick off the one and only call to ``send_goal``:

.. rli:: https://github.com/ros2/demos/raw/9c4ced3c5be392145312e0c0d3653140a2e29cc0/action_tutorials/action_tutorials_cpp/src/fibonacci_action_client.cpp
    :language: c++
    :lines: 46-48

When the timer expires, it will call ``send_goal``:

.. rli:: https://github.com/ros2/demos/raw/9c4ced3c5be392145312e0c0d3653140a2e29cc0/action_tutorials/action_tutorials_cpp/src/fibonacci_action_client.cpp
    :language: c++
    :lines: 52-77

This function does the following:

1. Cancels the timer (so it is only called once).
2. Waits for the action server to come up.
3. Instantiates a new ``Fibonacci::Goal``.
4. Sets the response, feedback, and result callbacks.
5. Sends the goal to the server.

When the server receives and accepts the goal, it will send a response to the client.
That response is handled by ``goal_response_callback``:

.. rli:: https://github.com/ros2/demos/raw/9c4ced3c5be392145312e0c0d3653140a2e29cc0/action_tutorials/action_tutorials_cpp/src/fibonacci_action_client.cpp
    :language: c++
    :lines: 84-92

Assuming the goal was accepted by the server, it will start processing.
Any feedback to the client will be handled by the ``feedback_callback``:

.. rli:: https://github.com/ros2/demos/raw/9c4ced3c5be392145312e0c0d3653140a2e29cc0/action_tutorials/action_tutorials_cpp/src/fibonacci_action_client.cpp
    :language: c++
    :lines: 95-105

When the server is finished processing, it will return a result to the client.
The result is handled by the ``result_callback``:

.. rli:: https://github.com/ros2/demos/raw/9c4ced3c5be392145312e0c0d3653140a2e29cc0/action_tutorials/action_tutorials_cpp/src/fibonacci_action_client.cpp
    :language: c++
    :lines: 108-130

We now have a fully functioning action client.  Let's get it built and running.

3.2 Compiling the action client
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In the previous section we put the action client code into place.
To get it to compile and run, we need to do a couple of additional things.

First we need to setup the CMakeLists.txt so that the action client is compiled.
Open up ``action_tutorials_cpp/CMakeLists.txt``, and add the following right after the ``find_package`` calls:

.. code-block:: cmake

  add_library(action_client SHARED
    src/fibonacci_action_client.cpp)
  target_include_directories(action_client PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_definitions(action_client
    PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
  ament_target_dependencies(action_client
    "action_tutorials_interfaces"
    "rclcpp"
    "rclcpp_action"
    "rclcpp_components")
  rclcpp_components_register_node(action_client PLUGIN "action_tutorials_cpp::FibonacciActionClient" EXECUTABLE fibonacci_action_client)
  install(TARGETS
    action_client
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

And now we can compile the package.  Go to the top-level of the ``ros2_ws``, and run:

.. code-block:: bash

  colcon build

This should compile the entire workspace, including the ``fibonacci_action_client`` in the ``action_tutorials_cpp`` package.

3.3 Running the action client
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Now that we have the action client built, we can run it.
First make sure that an action server is running in a separate terminal.
Now source the workspace we just built (``ros2_ws``), and try to run the action client:

.. code-block:: bash

  ros2 run action_tutorials_cpp fibonacci_action_client

You should see logged messages for the goal being accepted, feedback being printed, and the final result.

Summary
-------

In this tutorial, you put together a C++ action server and action client line by line, and configured them to exchange goals, feedback, and results.

Related content
---------------

* There are several ways you could write an action server and client in C++; check out the ``minimal_action_server`` and ``minimal_action_client`` packages in the `ros2/examples <https://github.com/ros2/examples/tree/{REPOS_FILE_BRANCH}/rclcpp>`_ repo.

* For more detailed information about ROS actions, please refer to the `design article <http://design.ros2.org/articles/actions.html>`__.
