Invoking Action/Service in Callbacks (C++/Python3)
==================================================

**Goal:** Learn how to invoke services and/or action servers in a callback of another action, service, subscription, or timer.

**Tutorial level:** Intermediate

**Time:** 20 minutes

**Author:** Steve Macenski

.. contents:: Contents
   :depth: 2
   :local:

Prerequisites
-------------

Before continuing in this tutorial, ensure you have a grasp on the basic ROS interfaces (e.g. topics, services, actions), executors, and callback groups.

You can find basic information about ROS interfaces in the Intermediate and Advanced tutorial series.
More information about executors and callback groups can be found below.

- :ref:`Executors`
- :ref:`callback_groups`

Background
----------

There are times that it is necessary to call a server from a callback function.
For example, you wish to call an action server from the callback of another action server (for example, in Nav2's `waypoint follower <https://github.com/ros-planning/navigation2/blob/main/nav2_waypoint_follower/>`_) or you would like to call a service from the callback of a subscription, service, or timer (for example, Nav2's `lifecycle manager <https://github.com/ros-planning/navigation2/blob/main/nav2_lifecycle_manager/src/lifecycle_manager.cpp>`_).

In these situations, it is not possible to use only a ``Node``'s provided Single Threaded Executor.
This is due to the current executor being spun already and having its thread being used to process the current callback.
If a user attempts to ``spin_until_future_complete`` or ``spin_once`` or similar on the executor, it will throw an exception because the current executor is already spinning.
This prevents the ``Node``'s executor from being used to process an action or service request to receive a result.

If using a Multi-Threaded Executor, a similar result will occur since the executor is already spinning.
The benefit of Multi-Threaded Executors is being able to process multiple callbacks in parallel, but still only one instance of ``spin`` can be valid at a given time.

Thus, we need to make use of additional executors.
This guide highlights how to use additional Single Threaded Executors to enable calling services and actions within other callbacks by means of executors and callback groups in C++.
In python, it will show how to use this with multiple ``Node`` objects instead, since ``rclpy`` does not have analogous structures of executors and callback groups. 

.. note::
    Similar processes as described below can be used to leverage executors and callback groups to have multiple parallel processed callbacks within a given program.

    This is not an exhaustive guide for all of the use-cases and applications of executors and callback groups.

rclcpp
------

Lets start with some node containing both an action and a service.
Both the action server and the service server are sharing an implicit executor and callback group using the defaults in the ``Node`` object.

.. code-block:: cpp

    class MyNode : public rclcpp::Node
    {
      MyNode::MyNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
      : Node("my_node", options)
      {
        service_client_ = create_client<SrvT>("service");

        action_server_ = rclcpp_action::create_server<ActionT>(
          this,
          "action",
          std::bind(&MyNode::handle_goal, this, _1, _2),
          std::bind(&MyNode::handle_cancel, this, _1),
          std::bind(&MyNode::handle_accepted, this, _1));
      }

      void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
      {
        // Do some work
        // ...

        auto request = std::make_shared<SrvT::Request>();
        auto result_future = service_client_->async_send_request(request);
        rclcpp::spin_until_future_complete(this, result_future);
        auto result = result_future.get()

        // Do something with the result
        // ...
      }
    };

    int main(int argc, char * argv[])
    {
      rclcpp::init(argc, argv);
      auto node = std::make_shared<MyNode>();
      rclcpp::spin(node->get_node_base_interface());
      rclcpp::shutdown();
      return 0;
    }

As written, this node will throw an exception when ``spin_until_future_complete`` is calls at run-time since the node is already spun in ``main()``, preventing us from getting the result of the service call.

We can modify this node however such that the service client or action server are being processed on another thread via another executor.
For this example, we will process the service client using another callback group that we add to an internal executor.

.. code-block:: cpp

    MyNode::MyNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("my_node", options)
    {
      callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
      service_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
      service_client_ = create_client<SrvT>("service", rmw_qos_profile_services_default, callback_group_);

      action_server_ = rclcpp_action::create_server<ActionT>(
        this,
        "action",
        std::bind(&MyNode::handle_goal, this, _1, _2),
        std::bind(&MyNode::handle_cancel, this, _1),
        std::bind(&MyNode::handle_accepted, this, _1));
    }

Now, instead of using ``rclcpp::spin_until_future_complete()`` to receive our service's response (which uses the internal ``Node`` executor passed by ``this``), we will specifically invoke the ``service_executor_`` to process our callback group containing the service.

.. code-block:: cpp

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
      // Do some work
      // ...

      auto request = std::make_shared<SrvT::Request>();
      auto result_future = service_client_->async_send_request(request);
      service_executor_.spin_until_future_complete(result_future);
      auto result = result_future.get()

      // Do something with the result
      // ...
    }

And thus, our node will now successfully process the service request on another thread, using another executor while the main thread from ``Node``'s executor is blocked while processing the main action server request.

rclpy
-----

In ``rclpy``, there are not the exact same concepts as in ``rclcpp``.
We cannot add our interfaces to callback groups and independent executors in the same fashion.
Instead, we will create additional ``Node`` classes to break apart capabilities into different threads to enable nested interfaces.

The following (**non-functioning**) example is where one might start if having a node with an action service who's callback requires calling a service, the same as in the ``rclcpp`` demo prior. 

.. code-block:: python
    
    class MyNode(Node):

        def __init__(self):
            super().__init__('my_node')
            self.client = self.create_client(SrvT, "service")
            self.action_server = ActionServer(
                self,
                ActionT,
                'action',
                self.execute_callback)

        def execute_callback(self, goal_handle):
            self.get_logger().info('Executing goal...')
            req = SrvT.Request()
            result_future = self.client.call_async(req)
            rclpy.spin_until_future_complete(self, result_future)

            result = ActionT.Result()
            result.response = result_future.result().response
            return result

    def main(args=None):
        rclpy.init(args=args)
        node = MyNode()
        rclpy.spin(node)

    if __name__ == '__main__':
        main()

Just as before, the ``spin_until_future_complete`` line will again throw an exception because the ``Node`` is already spinning in ``main()``.
Instead, we will modify this code to isolate the service client and its execution from the action server.
We will accomplish this by creating a new ``Node`` which will be composed in ``MyNode`` later to provide the service client capability.

The isolated node, ``ClientHandler`` would be as shown below.

.. code-block:: python

    class ClientHandler(Node):
        def __init__(self):
            super().__init__("service_client_handler")
            self.client = self.create_client(SrvT, "service")
            while not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("service not available, waiting again...")

        def send_request(self):
            req = SrvT.Request()
            result_future = self.client.call_async(req)
            rclpy.spin_until_future_complete(self, result_future)
            return result_future.result()

Now, we can use the ``ClientHandler`` as an object within ``MyNode`` to call the service through.
This acts as a node wrapper for the client to have its own node, and therefore executor, to process requests.

.. code-block:: python

    class MyNode(Node):

        def __init__(self):
            super().__init__('my_node')
            self.client = self.ClientHandler()
            self.action_server = ActionServer(
                self,
                ActionT,
                'action',
                self.execute_callback)

        def execute_callback(self, goal_handle):
            self.get_logger().info('Executing goal...')
            service_result = self.client.send_request()

            result = ActionT.Result()
            result.response = service_result.response
            return result
