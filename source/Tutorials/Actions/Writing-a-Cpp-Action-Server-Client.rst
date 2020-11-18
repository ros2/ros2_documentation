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
interface defined in the previous tutorial, :ref:`ActionCreate`.

Tasks
-----

1 Creating the action_tutorials_cpp package
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

As we saw in the :ref:`CreatePkg` tutorial, we need to create a new package to hold our C++ and supporting code.

1.1 Creating the action_tutorials_cpp package
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Go into the action workspace you created in the previous tutorial, and create a new package for the C++ action server:

(Remember to :ref:`source the workspace from the previous tutorial <ActionCreate>` first.)

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

      cd ~/action_ws/src
      ros2 pkg create --dependencies action_tutorials_interfaces rclcpp rclcpp_action rclcpp_components -- action_tutorials_cpp

  .. group-tab:: macOS

    .. code-block:: bash

      cd ~/action_ws/src
      ros2 pkg create --dependencies action_tutorials_interfaces rclcpp rclcpp_action rclcpp_components -- action_tutorials_cpp

  .. group-tab:: Windows

    .. code-block:: bash

      cd \dev\action_ws\src
      ros2 pkg create --dependencies action_tutorials_interfaces rclcpp rclcpp_action rclcpp_components -- action_tutorials_cpp

1.2 Adding in visibility control
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In order to make the package compile and work on Windows, we need to add in some "visibility control".
For details on why this is needed, see `here <https://docs.microsoft.com/en-us/cpp/cpp/dllexport-dllimport>`_.

Open up ``action_tutorials_cpp/include/action_tutorials_cpp/visibility_control.h``, and put the following code in:

.. code-block:: c++

  #ifndef ACTION_TUTORIALS_CPP__VISIBILITY_CONTROL_H_
  #define ACTION_TUTORIALS_CPP__VISIBILITY_CONTROL_H_

  #ifdef __cplusplus
  extern "C"
  {
  #endif

  // This logic was borrowed (then namespaced) from the examples on the gcc wiki:
  //     https://gcc.gnu.org/wiki/Visibility

  #if defined _WIN32 || defined __CYGWIN__
    #ifdef __GNUC__
      #define ACTION_TUTORIALS_CPP_EXPORT __attribute__ ((dllexport))
      #define ACTION_TUTORIALS_CPP_IMPORT __attribute__ ((dllimport))
    #else
      #define ACTION_TUTORIALS_CPP_EXPORT __declspec(dllexport)
      #define ACTION_TUTORIALS_CPP_IMPORT __declspec(dllimport)
    #endif
    #ifdef ACTION_TUTORIALS_CPP_BUILDING_DLL
      #define ACTION_TUTORIALS_CPP_PUBLIC ACTION_TUTORIALS_CPP_EXPORT
    #else
      #define ACTION_TUTORIALS_CPP_PUBLIC ACTION_TUTORIALS_CPP_IMPORT
    #endif
    #define ACTION_TUTORIALS_CPP_PUBLIC_TYPE ACTION_TUTORIALS_CPP_PUBLIC
    #define ACTION_TUTORIALS_CPP_LOCAL
  #else
    #define ACTION_TUTORIALS_CPP_EXPORT __attribute__ ((visibility("default")))
    #define ACTION_TUTORIALS_CPP_IMPORT
    #if __GNUC__ >= 4
      #define ACTION_TUTORIALS_CPP_PUBLIC __attribute__ ((visibility("default")))
      #define ACTION_TUTORIALS_CPP_LOCAL  __attribute__ ((visibility("hidden")))
    #else
      #define ACTION_TUTORIALS_CPP_PUBLIC
      #define ACTION_TUTORIALS_CPP_LOCAL
    #endif
    #define ACTION_TUTORIALS_CPP_PUBLIC_TYPE
  #endif

  #ifdef __cplusplus
  }
  #endif

  #endif  // ACTION_TUTORIALS_CPP__VISIBILITY_CONTROL_H_

2 Writing an action server
^^^^^^^^^^^^^^^^^^^^^^^^^^

Let's focus on writing an action server that computes the Fibonacci sequence using the action we created in the :ref:`ActionCreate` tutorial.

2.1 Writing the action server code
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Open up ``action_tutorials_cpp/src/fibonacci_action_server.cpp``, and put the following code in:

.. literalinclude:: server.cpp
    :language: c++
    :linenos:

The first few lines include all of the headers we need to compile.

Next we create a class that is a derived class of ``rclcpp::Node``:

.. literalinclude:: server.cpp
    :language: c++
    :lines: 14

The constructor for the ``FibonacciActionServer`` class initializes the node name as ``fibonacci_action_server``:

.. literalinclude:: server.cpp
    :language: c++
    :lines: 21-22

The constructor also instantiates a new action server:

.. literalinclude:: server.cpp
    :language: c++
    :lines: 26-31

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

.. literalinclude:: server.cpp
    :language: c++
    :lines: 37-44

This implementation just accepts all goals.

Next up is the callback for dealing with cancellation:

.. literalinclude:: server.cpp
    :language: c++
    :lines: 46-52

This implementation just tells the client that it accepted the cancellation.

The last of the callbacks accepts a new goal and starts processing it:

.. literalinclude:: server.cpp
    :language: c++
    :lines: 54-59

Since the execution is a long-running operation, we spawn off a thread to do the actual work and return from ``handle_accepted`` quickly.

All further processing and updates are done in the ``execute`` method in the new thread:

.. literalinclude:: server.cpp
    :language: c++
    :lines: 61-95

This work thread processes one sequence number of the Fibonacci sequence every second, publishing a feedback update for each step.
When it has finished processing, it marks the ``goal_handle`` as succeeded, and quits.

We now have a fully functioning action server.  Let's get it built and running.

2.2 Compiling the action server
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In the previous section we put the action server code into place.
To get it to compile and run, we need to do a couple of additional things.

First we need to setup the CMakeLists.txt so that the action server is compiled.
Open up ``action_tutorials_cpp/CMakeLists.txt``, and add the following right after the ``find_package`` calls:

.. code-block:: console

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

And now we can compile the package.  Go to the top-level of the ``action_ws``, and run:

.. code-block:: bash

  colcon build

This should compile the entire workspace, including the ``fibonacci_action_server`` in the ``action_tutorials_cpp`` package.

2.3 Running the action server
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Now that we have the action server built, we can run it.
Source the workspace we just built (``action_ws``), and try to run the action server:

.. code-block:: bash

  ros2 run action_tutorials_cpp fibonacci_action_server

3 Writing an action client
^^^^^^^^^^^^^^^^^^^^^^^^^^

3.1 Writing the action client code
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Open up ``action_tutorials_cpp/src/fibonacci_action_client.cpp``, and put the following code in:

.. literalinclude:: client.cpp
    :language: c++
    :linenos:

The first few lines include all of the headers we need to compile.

Next we create a class that is a derived class of ``rclcpp::Node``:

.. literalinclude:: client.cpp
    :language: c++
    :lines: 15

The constructor for the ``FibonacciActionClient`` class initializes the node name as ``fibonacci_action_client``:

.. literalinclude:: client.cpp
    :language: c++
    :lines: 20-22

The constructor also instantiates a new action client:

.. literalinclude:: client.cpp
    :language: c++
    :lines: 24-26

An action client requires 3 things:

1. The templated action type name: ``Fibonacci``.
2. A ROS 2 node to add the action client to: ``this``.
3. The action name: ``'fibonacci'``.

We also instantiate a ROS timer that will kick off the one and only call to ``send_goal``:

.. literalinclude:: client.cpp
    :language: c++
    :lines: 27-30

When the timer expires, it will call ``send_goal``:

.. literalinclude:: client.cpp
    :language: c++
    :lines: 32-57

This function does the following:

1. Cancels the timer (so it is only called once).
2. Waits for the action server to come up.
3. Instantiates a new ``Fibonacci::Goal``.
4. Sets the response, feedback, and result callbacks.
5. Sends the goal to the server.

When the server receives and accepts the goal, it will send a response to the client.
That response is handled by ``goal_response_callback``:

.. literalinclude:: client.cpp
    :language: c++
    :lines: 62-71

Assuming the goal was accepted by the server, it will start processing.
Any feedback to the client will be handled by the ``feedback_callback``:

.. literalinclude:: client.cpp
    :language: c++
    :lines: 72-83

When the server is finished processing, it will return a result to the client.
The result is handled by the ``result_callback``:

.. literalinclude:: client.cpp
    :language: c++
    :lines: 84-107

We now have a fully functioning action client.  Let's get it built and running.

3.2 Compiling the action client
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In the previous section we put the action client code into place.
To get it to compile and run, we need to do a couple of additional things.

First we need to setup the CMakeLists.txt so that the action client is compiled.
Open up ``action_tutorials_cpp/CMakeLists.txt``, and add the following right after the ``find_package`` calls:

.. code-block:: console

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

And now we can compile the package.  Go to the top-level of the ``action_ws``, and run:

.. code-block:: bash

  colcon build

This should compile the entire workspace, including the ``fibonacci_action_client`` in the ``action_tutorials_cpp`` package.

3.3 Running the action client
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Now that we have the action client built, we can run it.
First make sure that an action server is running in a separate terminal.
Now source the workspace we just built (``action_ws``), and try to run the action client:

.. code-block:: bash

  ros2 run action_tutorials_cpp fibonacci_action_client

You should see logged messages for the goal being accepted, feedback being printed, and the final result.

Summary
-------

In this tutorial, you put together a C++ action server and action client line by line, and configured them to exchange goals, feedback, and results.

Related content
---------------

* There are several ways you could write an action server and client in C++; check out the ``minimal_action_server`` and ``minimal_action_client`` packages in the `ros2/examples <https://github.com/ros2/examples/tree/master/rclcpp>`_ repo.

* For more detailed information about ROS actions, please refer to the `design article <http://design.ros2.org/articles/actions.html>`__.
