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

  $ ros2 pkg create --build-type ament_cmake --license Apache-2.0 cpp_srvcli --dependencies rclcpp example_interfaces

Your terminal will return a message verifying the creation of your package ``cpp_srvcli`` and all its necessary files and folders.

The ``--dependencies`` argument will automatically add the necessary dependency lines to ``package.xml`` and ``CMakeLists.txt``.
``example_interfaces`` is the package that includes `the .srv file <https://github.com/ros2/example_interfaces/blob/{REPOS_FILE_BRANCH}/srv/AddTwoInts.srv>`__ you will need to structure your requests and responses:

.. code-block:: bash

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
Add the following code block to ``CMakeLists.txt`` just below the dependencies to create an executable named ``server``:

.. code-block:: cmake

  add_executable(server src/add_two_ints_server.cpp)
  target_link_libraries(server PUBLIC rclcpp::rclcpp example_interfaces::example_interfaces)

So ``ros2 run`` can find the executable, add the following lines to the end of the file, right before ``ament_package()``:

.. code-block:: cmake

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

  using namespace std::chrono_literals;

  using AddTwoInts = example_interfaces::srv::AddTwoInts;

  class MinimalClient : public rclcpp::Node
  {
  public:
    MinimalClient()
    : Node("minimal_client"),
      count_(0)
    {
      client_ = this->create_client<AddTwoInts>("add_two_ints");
      while (!client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
          throw 1;
        }
        RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
      }

      auto response_received_callback =
        [this](rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future) -> void {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Received response: %ld", response->sum);
      };

      auto timer_callback = [this, response_received_callback]() -> void {
        auto request       = std::make_shared<AddTwoInts::Request>();
        request->a         = ++count_;
        request->b         = 10;
        auto future_result = client_->async_send_request(request, response_received_callback);
        RCLCPP_INFO(
          this->get_logger(), "Async request sent, it was: %ld + %ld. And then exiting", request->a,
          request->b);
      };

      timer_ = this->create_wall_timer(500ms, timer_callback);
    }

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
    int count_;
  };

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    try {
      auto node = std::make_shared<MinimalClient>();
      rclcpp::spin(node);
    } catch (...) {
    }
    rclcpp::shutdown();
    return 0;
  }


3.1 Examine the code
~~~~~~~~~~~~~~~~~~~~

Similar to the nodes in previous examples, the ``MinimalClient`` class inherits from ``Node``.
It has three attributes: a timer object, a service client, and an integer.

All initialization occurs in the constructor.
First, The client for sending requests is created.
Then, the code waits for the service to appear.

The ``while`` loop gives the client 1 second to search for service nodes in the network.
If it can't find any, it will continue waiting.
If the client is canceled (e.g. by you entering ``Ctrl+C`` into the terminal), it logs a message indicating the interruption and raises an exception to exit.


.. code-block:: C++

  client_ = this->create_client<AddTwoInts>("add_two_ints");
  while (!client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
      throw std::runtime_error("client interrupted while waiting for service to appear");
    }
    RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
  }


Two callbacks are defined to handle incoming events: one for the service response and another for the timer.
Both are implemented as lambda functions.

The service response callback takes a parameter containing the response.
It extracts the result object and logs it.

.. code-block:: C++

  auto response_received_callback =
    [this](rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future) -> void {
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Received response: %ld", response->sum);
  };


The timer callback is responsible for making service requests.
To do so, it must capture the previous callback within the lambda’s context, which is why it appears inside the square brackets.

.. code-block:: C++

  auto timer_callback = [this, response_received_callback]() -> void

Next, the request is created.
Its structure is defined by the ``.srv`` file mentioned earlier.

.. code-block:: C++

  auto request       = std::make_shared<AddTwoInts::Request>();
  request->a         = ++count_;
  request->b         = 10;

The client sends its request, logs a message, and exits the callback.

It is important to note that ``async_send_request`` returns immediately, taking only the time required to send the request without waiting for a response.
As a result, the callback execution completes, and the node continues spinning.

Processing the response within this callback is not possible since it has not yet been received.
However, this is not an issue, as response handling is already implemented in the previous callback, which will be triggered upon receiving the response.

.. code-block:: C++

  auto future_result = client_->async_send_request(request, response_received_callback);
  RCLCPP_INFO(
    this->get_logger(), "Async request sent, it was: %ld + %ld. And then exiting", request->a,
    request->b);


Finally, the timer is created:

.. code-block:: C++

  timer_ = this->create_wall_timer(500ms, timer_callback);

The main function initializes rclcpp, creates the node, and starts spinning.

Since the node constructor may throw an exception, a try/catch block is used for handling it.
This exception occurs when the user signals the program to exit, so no additional action is needed, allowing the program to terminate naturally.

.. code-block:: C++

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    try {
      auto node = std::make_shared<MinimalClient>();
      rclcpp::spin(node);
    } catch (...) {
    }
    rclcpp::shutdown();
    return 0;
  }


3.2 Add executable
~~~~~~~~~~~~~~~~~~

Return to ``CMakeLists.txt`` to add the executable and target for the new node.
After removing some unnecessary boilerplate from the automatically generated file, your ``CMakeLists.txt`` should look like this:

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.20)
  project(cpp_srvcli)

  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(example_interfaces REQUIRED)

  add_executable(server src/add_two_ints_server.cpp)
  target_link_libraries(server PUBLIC rclcpp::rclcpp example_interfaces::example_interfaces)

  add_executable(client src/add_two_ints_client.cpp)
  target_link_libraries(client PUBLIC rclcpp::rclcpp example_interfaces::example_interfaces)

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

      $ rosdep install -i --from-path src --rosdistro {DISTRO} -y

  .. group-tab:: macOS

      rosdep only runs on Linux, so you can skip ahead to next step.

  .. group-tab:: Windows

      rosdep only runs on Linux, so you can skip ahead to next step.


Navigate back to the root of your workspace, ``ros2_ws``, and build your new package:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      $ colcon build --packages-select cpp_srvcli

  .. group-tab:: macOS

    .. code-block:: console

      $ colcon build --packages-select cpp_srvcli

  .. group-tab:: Windows

    .. code-block:: console

      $ colcon build --merge-install --packages-select cpp_srvcli

Open a new terminal, navigate to ``ros2_ws``, and source the setup files:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      $ source install/setup.bash

  .. group-tab:: macOS

    .. code-block:: console

      $ . install/setup.bash

  .. group-tab:: Windows

    .. code-block:: console

      $ call install/setup.bat

Now run the service node:

.. code-block:: console

  $ ros2 run cpp_srvcli server

The terminal should wait for incoming requests.

Open another terminal, source the setup files from inside ``ros2_ws`` again.
Start the client node, followed by any two integers separated by a space.
The client sends the request to the service, which computes the sum and returns the result:

.. code-block:: console

  $ ros2 run cpp_srvcli client
  [INFO] [minimal_client]: result of 41 + 1: 42

The client should receive the response above.

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
