.. redirect-from::

    Tutorials/Writing-A-Simple-Cpp-Service-And-Client

.. _CppSrvCli:

Writing a simple service and client (C++)
=========================================

**Goal:** Create and run service and client nodes using C++.

**Tutorial level:** Beginner

**Time:** 20 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

When :doc:`nodes <../Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes>` communicate using :doc:`services <../Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services>`, the node that sends a request for data is called the client node, and the one that responds to the request is the service node.
The structure of the request and response is determined by a ``.srv`` file.

The example used here is a simple integer addition system; one node requests the sum of two integers, and the other responds with the result.


Prerequisites
-------------

In previous tutorials, you learned how to :doc:`create a workspace <./Creating-A-Workspace/Creating-A-Workspace>` and :doc:`create a package <./Creating-Your-First-ROS2-Package>`.

Tasks
-----

1 Create a package
^^^^^^^^^^^^^^^^^^

Open a new terminal and :doc:`source your ROS 2 installation <../Beginner-CLI-Tools/Configuring-ROS2-Environment>` so that ``ros2`` commands will work.

Navigate into the ``ros2_ws`` directory created in a :ref:`previous tutorial <new-directory>`.

Recall that packages should be created in the ``src`` directory, not the root of the workspace.
Navigate into ``ros2_ws/src`` and create a new package:

.. code-block:: console

  ros2 pkg create --build-type ament_cmake --license Apache-2.0 cpp_srvcli --dependencies rclcpp example_interfaces

Your terminal will return a message verifying the creation of your package ``cpp_srvcli`` and all its necessary files and folders.

The ``--dependencies`` argument will automatically add the necessary dependency lines to ``package.xml`` and ``CMakeLists.txt``.
``example_interfaces`` is the package that includes `the .srv file <https://github.com/ros2/example_interfaces/blob/{REPOS_FILE_BRANCH}/srv/AddTwoInts.srv>`__ you will need to structure your requests and responses:

.. code-block:: console

  int64 a
  int64 b
  ---
  int64 sum

The first two lines are the parameters of the request, and below the dashes is the response.

1.1 Update ``package.xml``
~~~~~~~~~~~~~~~~~~~~~~~~~~

Because you used the ``--dependencies`` option during package creation, you don't have to manually add dependencies to ``package.xml`` or ``CMakeLists.txt``.

As always, though, make sure to add the description, maintainer email and name, and license information to ``package.xml``.

.. code-block:: xml

  <description>C++ client server tutorial</description>
  <maintainer email="you@email.com">Your Name</maintainer>
  <license>Apache-2.0</license>


2 Write the service node
^^^^^^^^^^^^^^^^^^^^^^^^

Inside the ``ros2_ws/src/cpp_srvcli/src`` directory, create a new file called ``add_two_ints_server.cpp`` and paste the following code within:

.. code-block:: C++

  #include <cinttypes>
  #include <memory>

  #include "example_interfaces/srv/add_two_ints.hpp"
  #include "rclcpp/rclcpp.hpp"

  using AddTwoInts = example_interfaces::srv::AddTwoInts;
  rclcpp::Node::SharedPtr g_node = nullptr;

  void handle_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<AddTwoInts::Request> request,
    const std::shared_ptr<AddTwoInts::Response> response)
  {
    (void)request_header;
    RCLCPP_INFO(
      g_node->get_logger(),
      "request: %" PRId64 " + %" PRId64, request->a, request->b);
    response->sum = request->a + request->b;
  }

  int main(int argc, char ** argv)
  {
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("minimal_service");
    auto server = g_node->create_service<AddTwoInts>("add_two_ints", handle_service);
    rclcpp::spin(g_node);
    rclcpp::shutdown();
    g_node = nullptr;
    return 0;
  }

2.1 Examine the code
~~~~~~~~~~~~~~~~~~~~

The first ``#include`` statements are your package dependencies.

The ``handle_service`` function adds two integers from the request and gives the sum to the response, while notifying the console of its status using logs.

.. code-block:: C++

  void handle_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<AddTwoInts::Request> request,
    const std::shared_ptr<AddTwoInts::Response> response)
  {
    (void)request_header;
    RCLCPP_INFO(
      g_node->get_logger(),
      "request: %" PRId64 " + %" PRId64, request->a, request->b);
    response->sum = request->a + request->b;
  }

The ``main`` function accomplishes the following, line by line:

* Initializes ROS 2 C++ client library:

  .. code-block:: C++

    rclcpp::init(argc, argv);

* Creates a node named ``minimal_service``:

  .. code-block:: C++

    g_node = rclcpp::Node::make_shared("minimal_service");

* Creates a service named ``add_two_ints`` for that node and automatically advertises it over the networks with the ``handle_service`` method:

  .. code-block:: C++

    auto server = g_node->create_service<AddTwoInts>("add_two_ints", handle_service);

* Spins the node, making the service available.

  .. code-block:: C++

    rclcpp::spin(g_node);

2.2 Add executable
~~~~~~~~~~~~~~~~~~

The ``add_executable`` macro generates an executable you can run using ``ros2 run``.
Add the following code block to ``CMakeLists.txt`` to create an executable named ``server``:

.. code-block:: console

  add_executable(server src/add_two_ints_server.cpp)
  ament_target_dependencies(server rclcpp example_interfaces)

So ``ros2 run`` can find the executable, add the following lines to the end of the file, right before ``ament_package()``:

.. code-block:: console

  install(TARGETS
      server
    DESTINATION lib/${PROJECT_NAME})

You could build your package now, source the local setup files, and run it, but let's create the client node first so you can see the full system at work.

3 Write the client node
^^^^^^^^^^^^^^^^^^^^^^^

Inside the ``ros2_ws/src/cpp_srvcli/src`` directory, create a new file called ``add_two_ints_client.cpp`` and paste the following code within:

.. code-block:: C++

  #include <chrono>
  #include <cinttypes>
  #include <memory>

  #include "example_interfaces/srv/add_two_ints.hpp"
  #include "rclcpp/rclcpp.hpp"

  using AddTwoInts = example_interfaces::srv::AddTwoInts;

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("minimal_client");
    auto client = node->create_client<AddTwoInts>("add_two_ints");
    while (!client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
        return 1;
      }
      RCLCPP_INFO(node->get_logger(), "waiting for service to appear...");
    }
    auto request = std::make_shared<AddTwoInts::Request>();
    request->a = 41;
    request->b = 1;
    auto result_future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node->get_logger(), "service call failed :(");
      client->remove_pending_request(result_future);
      return 1;
    }
    auto result = result_future.get();
    RCLCPP_INFO(
      node->get_logger(), "result of %" PRId64 " + %" PRId64 " = %" PRId64,
      request->a, request->b, result->sum);
    rclcpp::shutdown();
    return 0;
  }


3.1 Examine the code
~~~~~~~~~~~~~~~~~~~~

Similar to the service node, the following lines of code create the node and then create the client for that node:

.. code-block:: C++

  auto node = rclcpp::Node::make_shared("minimal_client");
  auto client = node->create_client<AddTwoInts>("add_two_ints");

Next, the code waits for the service to appear.
The ``while`` loop gives the client 1 second to search for service nodes in the network.
If it can't find any, it will continue waiting.
If the client is canceled (e.g. by you entering ``Ctrl+C`` into the terminal), it will return an error log message stating it was interrupted.

.. code-block:: C++

  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "waiting for service to appear...");
  }

Next, the request is created.
Its structure is defined by the ``.srv`` file mentioned earlier.

.. code-block:: C++

  auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = 41;
  request->b = 1;


Then the client sends its request, and the node spins until it receives its response, or fails.

3.2 Add executable
~~~~~~~~~~~~~~~~~~

Return to ``CMakeLists.txt`` to add the executable and target for the new node.
After removing some unnecessary boilerplate from the automatically generated file, your ``CMakeLists.txt`` should look like this:

.. code-block:: console

  cmake_minimum_required(VERSION 3.5)
  project(cpp_srvcli)

  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(example_interfaces REQUIRED)

  add_executable(server src/add_two_ints_server.cpp)
  ament_target_dependencies(server rclcpp example_interfaces)

  add_executable(client src/add_two_ints_client.cpp)
  ament_target_dependencies(client rclcpp example_interfaces)

  install(TARGETS
    server
    client
    DESTINATION lib/${PROJECT_NAME})

  ament_package()


4 Build and run
^^^^^^^^^^^^^^^

It's good practice to run ``rosdep`` in the root of your workspace (``ros2_ws``) to check for missing dependencies before building:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      rosdep install -i --from-path src --rosdistro {DISTRO} -y

  .. group-tab:: macOS

      rosdep only runs on Linux, so you can skip ahead to next step.

  .. group-tab:: Windows

      rosdep only runs on Linux, so you can skip ahead to next step.


Navigate back to the root of your workspace, ``ros2_ws``, and build your new package:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      colcon build --packages-select cpp_srvcli

  .. group-tab:: macOS

    .. code-block:: console

      colcon build --packages-select cpp_srvcli

  .. group-tab:: Windows

    .. code-block:: console

      colcon build --merge-install --packages-select cpp_srvcli

Open a new terminal, navigate to ``ros2_ws``, and source the setup files:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      source install/setup.bash

  .. group-tab:: macOS

    .. code-block:: console

      . install/setup.bash

  .. group-tab:: Windows

    .. code-block:: console

      call install/setup.bat

Now run the service node:

.. code-block:: console

  ros2 run cpp_srvcli server

The terminal should wait for incoming requests.

Open another terminal, source the setup files from inside ``ros2_ws`` again.
Start the client node, followed by any two integers separated by a space:

.. code-block:: console

  ros2 run cpp_srvcli client

The client sends the request to the service, which computes the sum and returns the result.
The client should receive the following response:

.. code-block:: console

  [INFO] [minimal_client]: result of 41 + 1: 42

Return to the terminal where your service node is running.
You will see that it published log messages when it received the request and the data it received, and the response it sent back:

.. code-block:: console

  [INFO] [minimal_service]: request: 41 + 1

Enter ``Ctrl+C`` in the server terminal to stop the node from spinning.

Summary
-------

You created two nodes to request and respond to data over a service.
You added their dependencies and executables to the package configuration files so that you could build and run them, and see a service/client system at work.

Next steps
----------

In the last few tutorials you've been utilizing interfaces to pass data across topics and services.
Next, you'll learn how to :doc:`create custom interfaces <./Custom-ROS2-Interfaces>`.

Related content
---------------

* There are several ways you could write a service and client in C++; check out the ``minimal_service`` and ``minimal_client`` packages in the `ros2/examples <https://github.com/ros2/examples/tree/{REPOS_FILE_BRANCH}/rclcpp/services>`_ repo.
