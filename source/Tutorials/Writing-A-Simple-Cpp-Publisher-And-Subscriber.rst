.. _CppPubSub:

Writing a simple publisher and subscriber (C++)
===============================================

**Goal:** Create and run a publisher and subscriber node using C++.

**Tutorial level:** Beginner

**Time:** 20 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

:ref:`Nodes <ROS2Nodes>` are executable processes that communicate over the ROS graph.
In this tutorial, the nodes will pass information in the form of string messages to each other over a :ref:`topic <ROS2Topics>`.
The example used here is a simple “talker” and “listener” system; one node publishes data and the other subscribes to the topic so it can receive that data.

The code used in these examples can be found `here <https://github.com/ros2/examples/tree/master/rclcpp/topics>`__.

Prerequisites
-------------

In previous tutorials, you learned how to :ref:`create a workspace <ROS2Workspace>` and :ref:`create a package <CreatePkg>`.

Tasks
-----

1 Create a package
^^^^^^^^^^^^^^^^^^

Open a new terminal and :ref:`source your ROS 2 installation <ConfigROS2>` so that ``ros2`` commands will work.

Navigate into the ``dev_ws`` directory created in a previous tutorial.

Recall that packages should be created in the ``src`` directory, not the root of the workspace.
So, navigate into ``dev_ws/src``, and run the package creation command:

.. code-block:: console

    ros2 pkg create --build-type ament_cmake cpp_pubsub

Your terminal will return a message verifying the creation of your package ``cpp_pubsub`` and all its necessary files and folders.

Navigate into ``dev_ws/src/cpp_pubsub/src``.
Recall that this is the directory in any CMake package where the source files containing executables belong.


2 Write the publisher node
^^^^^^^^^^^^^^^^^^^^^^^^^^

Download the example talker code by entering the following command:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

            wget -O publisher_member_function.cpp https://raw.githubusercontent.com/ros2/examples/master/rclcpp/topics/minimal_publisher/member_function.cpp

   .. group-tab:: macOS

      .. code-block:: console

            wget -O publisher_member_function.cpp https://raw.githubusercontent.com/ros2/examples/master/rclcpp/topics/minimal_publisher/member_function.cpp

   .. group-tab:: Windows

      Right click this link and select Save As ``publisher_member_function.cpp``:

      https://raw.githubusercontent.com/ros2/examples/master/rclcpp/topics/minimal_publisher/member_function.cpp

Now there will be a new file named ``publisher_member_function.cpp``.
Open the file using your preferred text editor.

.. code-block:: C++

    #include <chrono>
    #include <functional>
    #include <memory>
    #include <string>

    #include "rclcpp/rclcpp.hpp"
    #include "std_msgs/msg/string.hpp"

    using namespace std::chrono_literals;

    /* This example creates a subclass of Node and uses std::bind() to register a
    * member function as a callback from the timer. */

    class MinimalPublisher : public rclcpp::Node
    {
      public:
        MinimalPublisher()
        : Node("minimal_publisher"), count_(0)
        {
          publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
          timer_ = this->create_wall_timer(
          500ms, std::bind(&MinimalPublisher::timer_callback, this));
        }

      private:
        void timer_callback()
        {
          auto message = std_msgs::msg::String();
          message.data = "Hello, world! " + std::to_string(count_++);
          RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
          publisher_->publish(message);
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        size_t count_;
      };

      int main(int argc, char * argv[])
      {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<MinimalPublisher>());
        rclcpp::shutdown();
        return 0;
      }

2.1 Examine the code
~~~~~~~~~~~~~~~~~~~~

The top of the code includes the standard C++ headers you will be using.
After the standard C++ headers is the ``rclcpp/rclcpp.hpp`` include which allows you to use the most common pieces of the ROS 2 system.
Last is ``std_msgs/msg/string.hpp``, which includes the built-in message type you will use to publish data.

These lines represent the node’s dependencies.
Recall that dependencies have to be added to ``package.xml`` and ``CMakeLists.txt``, which you’ll do in the next section.

.. code-block:: C++

    #include <chrono>
    #include <functional>
    #include <memory>
    #include <string>

    #include "rclcpp/rclcpp.hpp"
    #include "std_msgs/msg/string.hpp"

    using namespace std::chrono_literals;

The next line creates the node class ``MinimalPublisher`` by inheriting from ``rclcpp::Node``.
Every ``this`` in the code is referring to the node.

.. code-block:: C++

    class MinimalPublisher : public rclcpp::Node

The public constructor names the node ``minimal_publisher`` and initializes ``count_`` to 0.
Inside the constructor, the publisher is initialized with the ``String`` message type, the topic name ``topic``, and the required queue size to limit messages in the event of a backup.
Next, ``timer_`` is initialized, which causes the ``timer_callback`` function to be executed twice a second.

.. code-block:: C++

    public:
      MinimalPublisher()
      : Node("minimal_publisher"), count_(0)
      {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
      }

The ``timer_callback`` function is where the message data is set and the messages are actually published.
The ``RCLCPP_INFO`` macro ensures every published message is printed to the console.

.. code-block:: C++

    private:
      void timer_callback()
      {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
      }

Last is the declaration of the timer, publisher, and counter fields.

.. code-block:: C++

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;

Following the ``MinimalPublisher`` class is ``main``, where the node actually executes.
``rclcpp::init`` initializes ROS 2, and ``rclcpp::spin`` starts processing data from the node, including callbacks from the timer.

.. code-block:: C++

    int main(int argc, char * argv[])
    {
      rclcpp::init(argc, argv);
      rclcpp::spin(std::make_shared<MinimalPublisher>());
      rclcpp::shutdown();
      return 0;
    }

2.2 Add dependencies
~~~~~~~~~~~~~~~~~~~~

Navigate one level back to the ``dev_ws/src/cpp_pubsub`` directory, where the ``CMakeLists.txt`` and ``package.xml`` files have been created for you.

Open ``package.xml`` with your text editor.

As mentioned in the previous tutorial, make sure to fill in the ``<description>``, ``<maintainer>`` and ``<license>`` tags:

.. code-block:: xml

      <description>Examples of minimal publisher/subscriber using rclcpp</description>
      <maintainer email="you@email.com">Your Name</maintainer>
      <license>Apache License 2.0</license>

Add a new line after the ``ament_cmake`` buildtool dependency and paste the following dependencies corresponding to your node’s include statements:

.. code-block:: xml

    <depend>rclcpp</depend>
    <depend>std_msgs</depend>

This declares the package needs ``rclcpp`` and ``std_msgs`` when its code is executed.

Make sure to save the file.

2.3 CMakeLists.txt
~~~~~~~~~~~~~~~~~~

Now open the ``CMakeLists.txt`` file.
Below the existing dependency ``find_package(ament_cmake REQUIRED)``, add the lines:

.. code-block:: console

    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)

After that, add the executable and name it ``talker`` so you can run your node using ``ros2 run``:

.. code-block:: console

    add_executable(talker src/publisher_member_function.cpp)
    ament_target_dependencies(talker rclcpp std_msgs)

Finally, add the ``install(TARGETS…)`` section so ``ros2 run`` can find your executable:

.. code-block:: console

  install(TARGETS
    talker
    DESTINATION lib/${PROJECT_NAME})

You can clean up your ``CMakeLists.txt`` by removing some unnecessary sections and comments, so it looks like this:

.. code-block:: console

  cmake_minimum_required(VERSION 3.5)
  project(cpp_pubsub)

  # Default to C++14
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
  endif()

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)

  add_executable(talker src/publisher_member_function.cpp)
  ament_target_dependencies(talker rclcpp std_msgs)

  install(TARGETS
    talker
    DESTINATION lib/${PROJECT_NAME})

  ament_package()

You could build your package now, source the local setup files, and run it, but let’s create the subscriber node first so you can see the full system at work.

3 Write the subscriber node
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Return to ``dev_ws/src/cpp_pubsub/src`` to create the next node.
Enter the following code in your terminal:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

            wget -O subscriber_member_function.cpp https://raw.githubusercontent.com/ros2/examples/master/rclcpp/topics/minimal_subscriber/member_function.cpp

   .. group-tab:: macOS

      .. code-block:: console

            wget -O subscriber_member_function.cpp https://raw.githubusercontent.com/ros2/examples/master/rclcpp/topics/minimal_subscriber/member_function.cpp

   .. group-tab:: Windows

      Right click this link and select Save As ``subscriber_member_function.cpp``:

      https://raw.githubusercontent.com/ros2/examples/master/rclcpp/topics/minimal_subscriber/member_function.cpp

Entering ``ls`` in the console will now return:

.. code-block:: console

    publisher_member_function.cpp  subscriber_member_function.cpp

Open the ``subscriber_member_function.cpp`` with your text editor.

.. code-block:: C++

    #include <memory>

    #include "rclcpp/rclcpp.hpp"
    #include "std_msgs/msg/string.hpp"
    using std::placeholders::_1;

    class MinimalSubscriber : public rclcpp::Node
    {
      public:
        MinimalSubscriber()
        : Node("minimal_subscriber")
        {
          subscription_ = this->create_subscription<std_msgs::msg::String>(
          "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
        }

      private:
        void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
        {
          RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
        }
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    };

    int main(int argc, char * argv[])
    {
      rclcpp::init(argc, argv);
      rclcpp::spin(std::make_shared<MinimalSubscriber>());
      rclcpp::shutdown();
      return 0;
    }

3.1 Examine the code
~~~~~~~~~~~~~~~~~~~~

The subscriber node’s code is nearly identical to the publisher’s.
Now the node is named ``minimal_subscriber``, and the constructor uses the node’s ``create_subscription`` class to execute the callback.

There is no timer because the subscriber simply responds whenever data is published to the ``topic`` topic.

.. code-block:: C++

    public:
      MinimalSubscriber()
      : Node("minimal_subscriber")
      {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
      }

Recall from the :ref:`topic tutorial <ROS2Topics>` that the topic name and message type used by the publisher and subscriber must match to allow them to communicate.

The ``topic_callback`` function receives the string message data published over the topic, and simply writes it to the console using the ``RCLCPP_INFO`` macro.

The only field declaration in this class is the subscription.

.. code-block:: C++

    private:
      void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
      {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      }
      rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

The ``main`` function is exactly the same, except now it spins the ``MinimalSubscriber`` node.
For the publisher node, spinning meant starting the timer, but for the subscriber it simply means preparing to receive messages whenever they come.

Since this node has the same dependencies as the publisher node, there’s nothing new to add to ``package.xml``.

3.2 CMakeLists.txt
~~~~~~~~~~~~~~~~~~

Reopen ``CMakeLists.txt`` and add the executable and target for the subscriber node below the publisher’s entries.

.. code-block:: console

  add_executable(listener src/subscriber_member_function.cpp)
  ament_target_dependencies(listener rclcpp std_msgs)

  install(TARGETS
    talker
    listener
    DESTINATION lib/${PROJECT_NAME})

Make sure to save the file, and then your pub/sub system should be ready for use.

.. _cpppubsub-build-and-run:

4 Build and run
^^^^^^^^^^^^^^^
You likely already have the ``rclcpp`` and ``std_msgs`` packages installed as part of your ROS 2 system.
It's good practice to run ``rosdep`` in the root of your workspace (``dev_ws``) to check for missing dependencies before building:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

            rosdep install -i --from-path src --rosdistro rolling -y

   .. group-tab:: macOS

      rosdep only runs on Linux, so you can skip ahead to next step.

   .. group-tab:: Windows

      rosdep only runs on Linux, so you can skip ahead to next step.


Still in the root of your workspace, ``dev_ws``, build your new package:

.. code-block:: console

    colcon build --packages-select cpp_pubsub

Open a new terminal, navigate to ``dev_ws``, and source the setup files:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      . install/setup.bash

  .. group-tab:: macOS

    .. code-block:: console

      . install/setup.bash

  .. group-tab:: Windows

    .. code-block:: console

      call install/setup.bat

Now run the talker node:

.. code-block:: console

     ros2 run cpp_pubsub talker

The terminal should start publishing info messages every 0.5 seconds, like so:

.. code-block:: console

    [INFO] [minimal_publisher]: Publishing: "Hello World: 0"
    [INFO] [minimal_publisher]: Publishing: "Hello World: 1"
    [INFO] [minimal_publisher]: Publishing: "Hello World: 2"
    [INFO] [minimal_publisher]: Publishing: "Hello World: 3"
    [INFO] [minimal_publisher]: Publishing: "Hello World: 4"

Open another terminal, source the setup files from inside ``dev_ws`` again, and then start the listener node:

.. code-block:: console

     ros2 run cpp_pubsub listener

The listener will start printing messages to the console, starting at whatever message count the publisher is on at that time, like so:

.. code-block:: console

  [INFO] [minimal_subscriber]: I heard: "Hello World: 10"
  [INFO] [minimal_subscriber]: I heard: "Hello World: 11"
  [INFO] [minimal_subscriber]: I heard: "Hello World: 12"
  [INFO] [minimal_subscriber]: I heard: "Hello World: 13"
  [INFO] [minimal_subscriber]: I heard: "Hello World: 14"

Enter ``Ctrl+C`` in each terminal to stop the nodes from spinning.

Summary
-------

You created two nodes to publish and subscribe to data over a topic.
Before compiling and running them, you added their dependencies and executables to the package configuration files.



Next steps
----------

Next you'll create another simple ROS 2 package using the service/client model.
Again, you can choose to write it in either :ref:`C++ <CppSrvCli>` or :ref:`Python <PySrvCli>`.

Related content
---------------

There are several ways you could write a publisher and subscriber in C++; check out the ``minimal_publisher`` and ``minimal_subscriber`` packages in the `ros2/examples <https://github.com/ros2/examples/tree/master/rclcpp/topics>`_ repo.
