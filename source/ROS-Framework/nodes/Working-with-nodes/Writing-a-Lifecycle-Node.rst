.. meta::
   :contentType: tutorial
   :experience: intermediate
   :area: nodes, framework
   :distribution: {DISTRO}
   :product: {PRODUCT}

Writing a Lifecycle Node (C++) — tutorial
=========================================

.. short-description::
   Lifecycle nodes (also called managed nodes) implement a deterministic state machine to control node execution, resource allocation, and communication.
   This tutorial explains how to implement a lifecycle node in C++ using ``rclcpp_lifecycle``, transition states using the ``ros2 lifecycle`` command-line interface, and automate lifecycle management in launch files.

.. showmeta::
   :order: area, contentType, experience
   :labels: area=Area, contentType=Content type, experience=Level

**Goal:** Learn how to implement a lifecycle node and listener in C++, manage states via the CLI, and automate transitions in launch files.

**Tutorial level:** Intermediate

**Time:** 20 minutes

.. contents:: Table of Contents
   :depth: 2
   :local:

Summary
-------

* A **lifecycle node** (or **managed node**) follows a well-defined state machine with primary states (Unconfigured, Inactive, Active, Finalized) and transition states.
* Inheriting from ``rclcpp_lifecycle::LifecycleNode`` allows nodes to override transition callbacks such as ``on_configure()``, ``on_activate()``, ``on_deactivate()``, ``on_cleanup()``, and ``on_shutdown()``.
* Callback functions return a status of type ``CallbackReturn`` (``SUCCESS``, ``FAILURE``, or ``ERROR``), controlling whether the state machine transitions or remains in its previous state.
* Managed entities like ``rclcpp_lifecycle::LifecyclePublisher`` obey the node lifecycle, transmitting data across the network only when activated.
* Lifecycle nodes automatically publish state change notifications on the ``<node_name>/transition_event`` topic, allowing other nodes to monitor their lifecycle state.
* Node states and transitions can be inspected and triggered dynamically from the command-line interface using ``ros2 lifecycle``.
* Lifecycle state transitions can be orchestrated automatically using ``launch_ros`` actions and event handlers.

Overview
--------

Standard ROS 2 nodes execute their initialization and communication logic immediately upon instantiation.
In complex robotic systems comprising multiple sensors, actuators, and controllers, starting all components simultaneously can lead to race conditions, undefined behaviors, or hardware damage.
For example, a robot controller should not send motor commands until the safety monitor and trajectory planners are fully initialized and calibrated.

To address these challenges, ROS 2 provides managed lifecycle nodes based on the `ROS 2 Node Lifecycle design specification <http://design.ros2.org/articles/node_lifecycle.html>`__.
Lifecycle nodes provide explicit control over when a node is configured, when it starts processing and publishing data, when it pauses, and when it cleans up resources.

The Lifecycle State Machine
^^^^^^^^^^^^^^^^^^^^^^^^^^^

The lifecycle state machine consists of four **primary states** and several **transition states**:

1. **Unconfigured**: The node is instantiated, but its internal state, parameters, and communication channels are not yet configured.
2. **Inactive**: The node has been configured.
   Publishers and timers are created, parameters are validated, and hardware connections can be established.
   However, the node does not execute active computation or transmit messages over the network.
3. **Active**: The node is fully operational.
   Execution loops are running, callbacks are processed, and lifecycle publishers transmit data.
4. **Finalized**: The node has terminated its lifecycle and released all resources, ready for destruction.

Transitions between primary states occur through transition states where user-defined callbacks are executed:

* **Configuring** (calls ``on_configure()``): Transitions the node from **Unconfigured** to **Inactive**.
* **Activating** (calls ``on_activate()``): Transitions the node from **Inactive** to **Active**.
* **Deactivating** (calls ``on_deactivate()``): Transitions the node from **Active** back to **Inactive**.
* **CleaningUp** (calls ``on_cleanup()``): Transitions the node from **Inactive** back to **Unconfigured**, releasing allocated resources.
* **ShuttingDown** (calls ``on_shutdown()``): Transitions the node from **Unconfigured**, **Inactive**, or **Active** to **Finalized**.
* **ErrorProcessing** (calls ``on_error()``): Invoked if any transition callback returns ``ERROR`` or throws an uncaught exception.

All transition callbacks return a ``CallbackReturn`` enum value:

* ``CallbackReturn::SUCCESS``: The transition succeeded, and the node advances to the destination primary state.
* ``CallbackReturn::FAILURE``: The transition failed gracefully, and the node falls back to its previous primary state.
* ``CallbackReturn::ERROR``: An unexpected error occurred, transitioning the node into **ErrorProcessing**.

1 Creating a Package
--------------------

Open a new terminal and source your ROS 2 installation.
Create a new workspace directory and create a package named ``my_lifecycle_pkg``:

.. code-block:: console

   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ros2 pkg create --build-type ament_cmake --dependencies rclcpp rclcpp_lifecycle lifecycle_msgs std_msgs -- my_lifecycle_pkg

The command creates a ``my_lifecycle_pkg`` package directory containing ``package.xml``, ``CMakeLists.txt``, and a ``src`` folder.

1.1 package.xml
^^^^^^^^^^^^^^^

Your ``package.xml`` should declare the build tool and runtime dependencies:

.. code-block:: xml

   <?xml version="1.0"?>
   <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
   <package format="3">
     <name>my_lifecycle_pkg</name>
     <version>0.0.0</version>
     <description>Tutorial package for ROS 2 lifecycle nodes</description>
     <maintainer email="user@todo.todo">user</maintainer>
     <license>Apache-2.0</license>

     <buildtool_depend>ament_cmake</buildtool_depend>

     <depend>rclcpp</depend>
     <depend>rclcpp_lifecycle</depend>
     <depend>lifecycle_msgs</depend>
     <depend>std_msgs</depend>

     <export>
       <build_type>ament_cmake</build_type>
     </export>
   </package>

2 Writing the Lifecycle Talker Node
-----------------------------------

The lifecycle talker inherits from ``rclcpp_lifecycle::LifecycleNode`` instead of ``rclcpp::Node``.
It manages a lifecycle publisher and a wall timer, and overrides the transition callbacks to control when data is sent.

The code below is based on the `lifecycle_talker.cpp <https://github.com/ros2/demos/blob/{REPOS_FILE_BRANCH}/lifecycle/src/lifecycle_talker.cpp>`__ demo in the ``ros2/demos`` repository.

2.1 Source Code
^^^^^^^^^^^^^^^

Create a file named ``lifecycle_talker.cpp`` inside ``my_lifecycle_pkg/src``:

.. code-block:: c++

   #include <chrono>
   #include <cstddef>
   #include <cstdio>
   #include <memory>
   #include <string>
   #include <thread>
   #include <utility>

   #include "lifecycle_msgs/msg/transition.hpp"

   #include "rclcpp/executors/single_threaded_executor.hpp"
   #include "rclcpp/logging.hpp"
   #include "rclcpp/node_options.hpp"
   #include "rclcpp/timer.hpp"
   #include "rclcpp/utilities.hpp"

   #include "rclcpp_lifecycle/lifecycle_node.hpp"
   #include "rclcpp_lifecycle/lifecycle_publisher.hpp"
   #include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
   #include "rclcpp_lifecycle/state.hpp"

   #include "std_msgs/msg/string.hpp"

   using namespace std::chrono_literals;

   class LifecycleTalker : public rclcpp_lifecycle::LifecycleNode
   {
   public:
     explicit LifecycleTalker(const std::string & node_name, bool intra_process_comms = false)
     : rclcpp_lifecycle::LifecycleNode(
         node_name,
         rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
     {
     }

     void publish()
     {
       static size_t count = 0;
       auto msg = std::make_unique<std_msgs::msg::String>();
       msg->data = "Lifecycle HelloWorld #" + std::to_string(++count);

       if (!pub_->is_activated()) {
         RCLCPP_INFO(
           get_logger(), "Lifecycle publisher is currently inactive. Messages are not published.");
       } else {
         RCLCPP_INFO(
           get_logger(), "Lifecycle publisher is active. Publishing: [%s]", msg->data.c_str());
       }

       pub_->publish(std::move(msg));
     }

     rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
     on_configure(const rclcpp_lifecycle::State &) override
     {
       pub_ = this->create_publisher<std_msgs::msg::String>("lifecycle_chatter", 10);
       timer_ = this->create_wall_timer(
         1s, [this]() { return this->publish(); });

       RCLCPP_INFO(get_logger(), "on_configure() is called.");

       return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
     }

     rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
     on_activate(const rclcpp_lifecycle::State & state) override
     {
       LifecycleNode::on_activate(state);

       RCLCPP_INFO(get_logger(), "on_activate() is called.");

       std::this_thread::sleep_for(2s);

       return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
     }

     rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
     on_deactivate(const rclcpp_lifecycle::State & state) override
     {
       LifecycleNode::on_deactivate(state);

       RCLCPP_INFO(get_logger(), "on_deactivate() is called.");

       return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
     }

     rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
     on_cleanup(const rclcpp_lifecycle::State &) override
     {
       timer_.reset();
       pub_.reset();

       RCLCPP_INFO(get_logger(), "on cleanup is called.");

       return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
     }

     rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
     on_shutdown(const rclcpp_lifecycle::State & state) override
     {
       timer_.reset();
       pub_.reset();

       RCLCPP_INFO(get_logger(), "on shutdown is called from state %s.", state.label().c_str());

       return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
     }

   private:
     std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_;
     std::shared_ptr<rclcpp::TimerBase> timer_;
   };

   int main(int argc, char * argv[])
   {
     setvbuf(stdout, NULL, _IONBF, BUFSIZ);

     rclcpp::init(argc, argv);

     rclcpp::executors::SingleThreadedExecutor exe;

     std::shared_ptr<LifecycleTalker> lc_node =
       std::make_shared<LifecycleTalker>("lc_talker");

     exe.add_node(lc_node->get_node_base_interface());

     exe.spin();

     rclcpp::shutdown();

     return 0;
   }

.. include:: ../../../_internal/Rclcpp-Convenience-Header-Note.rst

2.2 Code Explanation
^^^^^^^^^^^^^^^^^^^^

* **Inheriting from LifecycleNode**:
  Unlike regular nodes that inherit from ``rclcpp::Node``, lifecycle nodes inherit from ``rclcpp_lifecycle::LifecycleNode``.
* **Configuring Resources**:
  In ``on_configure()``, the publisher and wall timer are created.
  The call ``this->create_publisher<std_msgs::msg::String>("lifecycle_chatter", 10)`` on a ``LifecycleNode`` returns an instance of ``rclcpp_lifecycle::LifecyclePublisher``.
* **Activating and Deactivating**:
  Calling ``LifecycleNode::on_activate(state)`` automatically activates all managed entities belonging to the node.
  Alternatively, you can call ``pub_->on_activate()`` directly on specific publishers.
  While deactivated, calling ``pub_->publish()`` does not transmit any message onto the ROS graph, ensuring downstream nodes receive no unready data.
* **Cleaning Up**:
  In ``on_cleanup()``, pointers to timers and publishers are reset.
  This returns the node to a clean unconfigured state, allowing it to be reconfigured later if needed.
* **Spinning the Node**:
  Because ``rclcpp_lifecycle::LifecycleNode`` does not inherit from ``rclcpp::Node``, pass its base interface using ``lc_node->get_node_base_interface()`` to the executor.

3 Writing the Lifecycle Listener Node
-------------------------------------

To observe both the data topic and the node's state transitions, we implement a listener node.
Every lifecycle node automatically publishes its state change notifications to the ``<node_name>/transition_event`` topic as ``lifecycle_msgs/msg/TransitionEvent`` messages.

The code below is based on the `lifecycle_listener.cpp <https://github.com/ros2/demos/blob/{REPOS_FILE_BRANCH}/lifecycle/src/lifecycle_listener.cpp>`__ demo in the ``ros2/demos`` repository.

3.1 Source Code
^^^^^^^^^^^^^^^

Create a file named ``lifecycle_listener.cpp`` inside ``my_lifecycle_pkg/src``:

.. code-block:: c++

   #include <cstdio>
   #include <memory>
   #include <string>

   #include "lifecycle_msgs/msg/transition_event.hpp"

   #include "rclcpp/rclcpp.hpp"

   #include "std_msgs/msg/string.hpp"

   class LifecycleListener : public rclcpp::Node
   {
   public:
     explicit LifecycleListener(const std::string & node_name)
     : Node(node_name)
     {
       // Data topic from the lc_talker node
       sub_data_ = this->create_subscription<std_msgs::msg::String>(
         "lifecycle_chatter", 10,
         [this](std_msgs::msg::String::ConstSharedPtr msg) {
           RCLCPP_INFO(get_logger(), "data_callback: %s", msg->data.c_str());
         });

       // Notification event topic from the lc_talker node
       sub_notification_ = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
         "/lc_talker/transition_event",
         10,
         [this](lifecycle_msgs::msg::TransitionEvent::ConstSharedPtr msg) {
           RCLCPP_INFO(
             get_logger(), "notify callback: Transition from state %s to %s",
             msg->start_state.label.c_str(), msg->goal_state.label.c_str());
         });
     }

   private:
     std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> sub_data_;
     std::shared_ptr<rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>> sub_notification_;
   };

   int main(int argc, char ** argv)
   {
     setvbuf(stdout, NULL, _IONBF, BUFSIZ);

     rclcpp::init(argc, argv);

     auto lc_listener = std::make_shared<LifecycleListener>("lc_listener");
     rclcpp::spin(lc_listener);

     rclcpp::shutdown();

     return 0;
   }

3.2 Code Explanation
^^^^^^^^^^^^^^^^^^^^

* **Listening to Transition Events**:
  Subscribing to ``/lc_talker/transition_event`` receives a ``TransitionEvent`` message containing ``start_state`` and ``goal_state`` whenever the talker node transitions between states.
* **Receiving Data Messages**:
  Subscribing to ``lifecycle_chatter`` receives messages only when the talker's lifecycle publisher is activated.

4 CMakeLists.txt Configuration
------------------------------

Open ``CMakeLists.txt`` in your package.
Declare the executables and link them using modern target-based CMake:

.. code-block:: cmake

   cmake_minimum_required(VERSION 3.20)
   project(my_lifecycle_pkg)

   if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
     add_compile_options(-Wall -Wextra -Wpedantic)
   endif()

   find_package(ament_cmake REQUIRED)
   find_package(rclcpp REQUIRED)
   find_package(rclcpp_lifecycle REQUIRED)
   find_package(lifecycle_msgs REQUIRED)
   find_package(std_msgs REQUIRED)

   add_executable(lifecycle_talker src/lifecycle_talker.cpp)
   target_link_libraries(lifecycle_talker PRIVATE
     rclcpp::rclcpp
     rclcpp_lifecycle::rclcpp_lifecycle
     lifecycle_msgs::lifecycle_msgs
     std_msgs::std_msgs
   )

   add_executable(lifecycle_listener src/lifecycle_listener.cpp)
   target_link_libraries(lifecycle_listener PRIVATE
     rclcpp::rclcpp
     lifecycle_msgs::lifecycle_msgs
     std_msgs::std_msgs
   )

   install(TARGETS
     lifecycle_talker
     lifecycle_listener
     DESTINATION lib/${PROJECT_NAME}
   )

   install(DIRECTORY
     launch
     DESTINATION share/${PROJECT_NAME}
   )

   ament_package()

.. note::

   Modern ROS 2 packages link dependencies directly with ``target_link_libraries`` using namespaced targets (such as ``rclcpp::rclcpp`` and ``lifecycle_msgs::lifecycle_msgs``).
   The legacy macro ``ament_target_dependencies`` has been deprecated.

5 Managing Lifecycle Nodes via CLI
----------------------------------

ROS 2 includes the ``ros2 lifecycle`` command-line tool to query and change node states directly from the terminal.

5.1 Building the Package
^^^^^^^^^^^^^^^^^^^^^^^^

Navigate to your workspace root and build the package:

.. code-block:: console

   cd ~/ros2_ws
   colcon build --packages-select my_lifecycle_pkg
   source install/setup.bash

5.2 Running the Nodes
^^^^^^^^^^^^^^^^^^^^^

In your first terminal, run the lifecycle talker node:

.. code-block:: console

   ros2 run my_lifecycle_pkg lifecycle_talker

In a second terminal, source your workspace and run the listener node:

.. code-block:: console

   source ~/ros2_ws/install/setup.bash
   ros2 run my_lifecycle_pkg lifecycle_listener

5.3 Inspecting Current State
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In a third terminal, query the current lifecycle state of the talker:

.. code-block:: console

   ros2 lifecycle get /lc_talker

Output:

.. code-block:: console

   unconfigured [1]

The node starts in the **unconfigured** state.
Notice that no messages are being published yet.

5.4 Listing Available Transitions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

List the valid transitions available from the current state:

.. code-block:: console

   ros2 lifecycle list /lc_talker

Output:

.. code-block:: console

   - configure [1]
       Start: unconfigured
       Goal: configuring
   - shutdown [5]
       Start: unconfigured
       Goal: shuttingdown

From **unconfigured**, the node can either transition to configuring or shut down.

5.5 Transitioning to Inactive (Configure)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Trigger the ``configure`` transition:

.. code-block:: console

   ros2 lifecycle set /lc_talker configure

Output:

.. code-block:: console

   Transitioning successful

In the talker's console, you will see:

.. code-block:: console

   [INFO] [lc_talker]: on_configure() is called.
   [INFO] [lc_talker]: Lifecycle publisher is currently inactive. Messages are not published.
   [WARN] [LifecyclePublisher]: Trying to publish message on the topic '/lifecycle_chatter', but the publisher is not activated

In the listener's console, you will observe the transition events:

.. code-block:: console

   [INFO] [lc_listener]: notify callback: Transition from state unconfigured to configuring
   [INFO] [lc_listener]: notify callback: Transition from state configuring to inactive

Query the talker state again:

.. code-block:: console

   ros2 lifecycle get /lc_talker

Output:

.. code-block:: console

   inactive [2]

The node is now **inactive**.
The timer callback is executing, but the lifecycle publisher prevents data from being transmitted on ``lifecycle_chatter``.

5.6 Transitioning to Active (Activate)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Trigger the ``activate`` transition:

.. code-block:: console

   ros2 lifecycle set /lc_talker activate

Output:

.. code-block:: console

   Transitioning successful

In the talker's console, you will see the activation log and message transmission:

.. code-block:: console

   [INFO] [lc_talker]: on_activate() is called.
   [INFO] [lc_talker]: Lifecycle publisher is active. Publishing: [Lifecycle HelloWorld #1]
   [INFO] [lc_talker]: Lifecycle publisher is active. Publishing: [Lifecycle HelloWorld #2]

In the listener's console, you will observe both the transition events and the received messages:

.. code-block:: console

   [INFO] [lc_listener]: notify callback: Transition from state inactive to activating
   [INFO] [lc_listener]: notify callback: Transition from state activating to active
   [INFO] [lc_listener]: data_callback: Lifecycle HelloWorld #1
   [INFO] [lc_listener]: data_callback: Lifecycle HelloWorld #2

5.7 Deactivating, Cleaning Up, and Shutting Down
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To pause data transmission while keeping allocated resources, deactivate the node:

.. code-block:: console

   ros2 lifecycle set /lc_talker deactivate

In the listener's console:

.. code-block:: console

   [INFO] [lc_listener]: notify callback: Transition from state active to deactivating
   [INFO] [lc_listener]: notify callback: Transition from state deactivating to inactive

To release all allocated resources and return to **unconfigured**, call cleanup:

.. code-block:: console

   ros2 lifecycle set /lc_talker cleanup

To shut down the node completely:

.. code-block:: console

   ros2 lifecycle set /lc_talker shutdown

6 Automating Transitions with Launch Files
------------------------------------------

In automated or production environments, nodes are typically orchestrated using launch files rather than manual CLI commands.
The ``launch_ros`` package provides dedicated tools to start lifecycle nodes and trigger transitions automatically.

This example is based on the `lifecycle_demo_launch.py <https://github.com/ros2/demos/blob/{REPOS_FILE_BRANCH}/lifecycle/launch/lifecycle_demo_launch.py>`__ demo in ``ros2/demos``.

6.1 Writing the Launch File
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Create a ``launch`` directory inside ``my_lifecycle_pkg`` and create a file named ``lifecycle_launch.py``:

.. code-block:: python

   from launch import LaunchDescription
   from launch.actions import EmitEvent, LogInfo, RegisterEventHandler
   from launch.events import matches_action
   from launch_ros.actions import LifecycleNode, Node
   from launch_ros.event_handlers import OnStateTransition
   from launch_ros.events.lifecycle import ChangeState
   import lifecycle_msgs.msg


   def generate_launch_description():
       talker_node = LifecycleNode(
           package='my_lifecycle_pkg',
           executable='lifecycle_talker',
           name='lc_talker',
           namespace='',
           output='screen',
       )

       listener_node = Node(
           package='my_lifecycle_pkg',
           executable='lifecycle_listener',
           output='screen',
       )

       # Emit an event to trigger the configure transition upon startup
       configure_event = EmitEvent(
           event=ChangeState(
               lifecycle_node_matcher=matches_action(talker_node),
               transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
           )
       )

       # Register an event handler to activate the node once it reaches inactive
       activate_event_handler = RegisterEventHandler(
           OnStateTransition(
               target_lifecycle_node=talker_node,
               goal_state='inactive',
               entities=[
                   LogInfo(msg="lc_talker reached 'inactive'; activating..."),
                   EmitEvent(
                       event=ChangeState(
                           lifecycle_node_matcher=matches_action(talker_node),
                           transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                       )
                   ),
               ],
           )
       )

       return LaunchDescription([
           talker_node,
           listener_node,
           configure_event,
           activate_event_handler,
       ])

6.2 Launch Mechanics Explained
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* **LifecycleNode Action**:
  Instead of using ``launch_ros.actions.Node``, use ``launch_ros.actions.LifecycleNode``.
  This registers the node with the launch lifecycle manager.
* **ChangeState Event**:
  ``launch_ros.events.lifecycle.ChangeState`` requests a state transition using a transition ID from ``lifecycle_msgs.msg.Transition``.
  The matcher ``matches_action(talker_node)`` identifies which node receives the request.
* **OnStateTransition Handler**:
  ``launch_ros.event_handlers.OnStateTransition`` listens for lifecycle transition events.
  When the ``target_lifecycle_node`` reaches the ``goal_state`` of ``inactive``, it emits a new event to trigger ``TRANSITION_ACTIVATE``.
  This ensures deterministic startup without arbitrary sleep delays.

6.3 Running the Launch File
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Rebuild your package to install the launch file and execute it:

.. code-block:: console

   cd ~/ros2_ws
   colcon build --packages-select my_lifecycle_pkg
   source install/setup.bash
   ros2 launch my_lifecycle_pkg lifecycle_launch.py

Output:

.. code-block:: console

   [INFO] [lifecycle_talker-1]: process started with pid [...]
   [INFO] [lifecycle_listener-2]: process started with pid [...]
   [lifecycle_listener-2] [INFO] [lc_listener]: notify callback: Transition from state unconfigured to configuring
   [lifecycle_talker-1] [INFO] [lc_talker]: on_configure() is called.
   [lifecycle_listener-2] [INFO] [lc_listener]: notify callback: Transition from state configuring to inactive
   [INFO] [launch.user]: lc_talker reached 'inactive'; activating...
   [lifecycle_talker-1] [INFO] [lc_talker]: on_activate() is called.
   [lifecycle_listener-2] [INFO] [lc_listener]: notify callback: Transition from state inactive to activating
   [lifecycle_talker-1] [INFO] [lc_talker]: Lifecycle publisher is active. Publishing: [Lifecycle HelloWorld #1]
   [lifecycle_listener-2] [INFO] [lc_listener]: notify callback: Transition from state activating to active
   [lifecycle_listener-2] [INFO] [lc_listener]: data_callback: Lifecycle HelloWorld #1
   [lifecycle_talker-1] [INFO] [lc_talker]: Lifecycle publisher is active. Publishing: [Lifecycle HelloWorld #2]
   [lifecycle_listener-2] [INFO] [lc_listener]: data_callback: Lifecycle HelloWorld #2

Related Content
---------------

* :doc:`Managing node lifecycles - example <Managed-Nodes>`: High-level overview and example of managed nodes in ROS 2.
* :doc:`Using the Node Interfaces Template Class (C++) <Using-Node-Interfaces-Template-Class>`: How to write functions that accept both standard ``Node`` and ``LifecycleNode`` pointers.
* `ROS 2 Design: Node Lifecycle <http://design.ros2.org/articles/node_lifecycle.html>`__: Formal design document defining the ROS 2 lifecycle state machine.
* `rclcpp_lifecycle API documentation <https://docs.ros.org/en/{DISTRO}/p/rclcpp_lifecycle/>`__: Complete C++ API reference for lifecycle nodes and publishers.
* `Lifecycle Demo repository <https://github.com/ros2/demos/tree/{REPOS_FILE_BRANCH}/lifecycle>`__: Complete lifecycle demo package in ``ros2/demos``.
