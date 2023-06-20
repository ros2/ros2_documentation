Using Callback Groups
=====================

When running a node in a Multi-Threaded Executor, ROS 2 offers callback
groups as a tool for controlling the execution of different callbacks.
This page is meant as a guide on how to use callback groups efficiently.
It is assumed that the reader has a basic understanding
about the concept of :doc:`executors <../Concepts/Intermediate/About-Executors>`.

.. contents:: Table of Contents
   :local:

Basics of callback groups
-------------------------

When running a node in a Multi-Threaded Executor,
ROS 2 offers two different types of callback groups for controlling
execution of callbacks:

* Mutually Exclusive Callback Group
* Reentrant Callback Group

These callback groups restrict the execution of their callbacks in
different ways.
In short:

* Mutually Exclusive Callback Group prevents its callbacks from being
  executed in parallel - essentially making it as if the callbacks in the group
  were executed by a SingleThreadedExecutor.
* Reentrant Callback Group allows the executor to schedule and execute
  the group's callbacks in any way it sees fit, without restrictions.
  This means that, in addition to different callbacks being run parallel
  to each other, different instances of the same callback may also be
  executed concurrently.
* Callbacks belonging to different callback groups (of any type) can always
  be executed parallel to each other.

It is also important to keep in mind that different ROS 2 entities relay
their callback group to all callbacks they spawn.
For example, if one assigns a callback group to an action client,
all callbacks created by the client will be assigned to that callback group.

Callback groups can be created by a node's ``create_callback_group``
function in rclcpp and by calling the constructor of the group in rclpy.
The callback group can then be passed as argument/option when creating a subscription, timer, etc.

.. tabs::

  .. group-tab:: C++

    .. code-block:: cpp

      my_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

      rclcpp::SubscriptionOptions options;
      options.callback_group = my_callback_group;

      my_subscription = create_subscription<Int32>("/topic", rclcpp::SensorDataQoS(),
                                                    callback, options);

  .. group-tab:: Python

    .. code-block:: python

      my_callback_group = MutuallyExclusiveCallbackGroup()
      my_subscription = self.create_subscription(Int32, "/topic", self.callback, qos_profile=1,
                                                  callback_group=my_callback_group)

If the user does not specify any callback group when creating a subscription,
timer, etc., this entity will be assigned to the node's default callback group.
The default callback group is a Mutually Exclusive Callback Group and it can be
queried via ``NodeBaseInterface::get_default_callback_group()`` in rclcpp and
via ``Node.default_callback_group`` in rclpy.

About callbacks
^^^^^^^^^^^^^^^

In the context of ROS 2 and executors, a callback means a function whose
scheduling and execution is handled by an executor.
Examples of callbacks in this context are

* subscription callbacks (receiving and handling data from a topic),
* timer callbacks,
* service callbacks (for executing service requests in a server),
* different callbacks in action servers and clients,
* done-callbacks of Futures.

Below are a couple important points about callbacks that should be kept
in mind when working with callback groups.

* Almost everything in ROS 2 is a callback!
  Every function that is run by an executor is, by definition, a callback.
  The non-callback functions in a ROS 2 system are found mainly at
  the edge of the system (user and sensor inputs etc).
* Sometimes the callbacks are hidden and their presence may not be obvious
  from the user/developer API.
  This is the case especially with any kind of “synchronous” call to a
  service or an action (in rclpy).
  For example, the synchronous call ``Client.call(request)`` to a service
  adds a Future's done-callback that needs to be executed during the
  execution of the function call, but this callback is not directly
  visible to the user.


Controlling execution
---------------------

In order to control execution with callback groups, one can consider the
following guidelines.

For the interaction of an individual callback with itself:

* Register it to a Reentrant Callback Group if it should be executed in parallel to itself.
  An example case could be an action/service server that needs to be able to
  process several action calls in parallel to each other.

* Register it to a Mutually Exclusive Callback Group if it should **never** be executed in parallel to itself.
  An example case could be a timer callback that runs a control loop that publishes control commands.

For the interaction of different callbacks with each other:

* Register them to the same Mutually Exclusive Callback Group if they should **never** be executed in parallel.
  An example case could be that the callbacks are accessing shared critical and non-thread-safe resources.

If they should be executed in parallel, you have two options,
depending on whether the individual callbacks should be able to overlap themselves or not:

* Register them to different Mutually Exclusive Callback Groups (no overlap of the individual callbacks)

* Register them to a Reentrant Callback Group (overlap of the individual callbacks)

An example case of running different callbacks in parallel is a Node that has
a synchronous service client and a timer calling this service.
See the detailed example below.

Avoiding deadlocks
------------------

Setting up callback groups of a node incorrectly can lead to deadlocks (or
other unwanted behavior), especially if one desires to use synchronous calls to
services or actions.
Indeed, even the API documentation of ROS 2 mentions that
synchronous calls to actions or services should not be done in callbacks,
because it can lead to deadlocks.
While using asynchronous calls is indeed safer in this regard, synchronous
calls can also be made to work.
On the other hand, synchronous calls also have their advantages, such as
making the code simpler and easier to understand.
Hence, this section provides some guidelines on how to set up a node's
callback groups correctly in order to avoid deadlocks.

First thing to note here is that every node's default callback group is a
Mutually Exclusive Callback Group.
If the user does not specify any other callback group when creating a timer,
subscription, client etc., any callbacks created then or later by these
entities will use the node's default callback group.
Furthermore, if everything in a node uses the same Mutually Exclusive
Callback Group, that node essentially acts as if it was handled
by a Single-Threaded Executor, even if a multi-threaded one is specified!
Thus, whenever one decides to use a Multi-Threaded Executor,
some callback group(s) should always be specified in order for the
executor choice to make sense.

With the above in mind, here are a couple guidelines to help avoid deadlocks:

* If you make a synchronous call in any type of a callback, this callback and
  the client making the call need to belong to

  * different callback groups (of any type), or
  * a Reentrant Callback Group.

* If the above configuration is not possible due to other requirements - such
  as thread-safety and/or blocking of other callbacks while waiting for the
  result (or if you want to make absolutely sure that there is never a
  possibility of a deadlock), use asynchronous calls.

Failing the first point will always cause a deadlock.
An example of such a case would be making a synchronous service call
in a timer callback (see the next section for an example).


Examples
--------

Let us look at some simple examples of different callback group setups.
The following demo code considers calling a service synchronously in a timer
callback.

Demo code
^^^^^^^^^

We have two nodes - one providing a simple service:

.. tabs::

   .. group-tab:: C++

      .. code-block:: cpp

        #include <memory>
        #include "rclcpp/rclcpp.hpp"
        #include "std_srvs/srv/empty.hpp"

        using namespace std::placeholders;

        namespace cb_group_demo
        {
        class ServiceNode : public rclcpp::Node
        {
        public:
            ServiceNode() : Node("service_node")
            {
                service_ptr_ = this->create_service<std_srvs::srv::Empty>(
                        "test_service",
                        std::bind(&ServiceNode::service_callback, this, _1, _2, _3)
                );
            }

        private:
            rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_ptr_;

            void service_callback(
                    const std::shared_ptr<rmw_request_id_t> request_header,
                    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                    const std::shared_ptr<std_srvs::srv::Empty::Response> response)
            {
                (void)request_header;
                (void)request;
                (void)response;
                RCLCPP_INFO(this->get_logger(), "Received request, responding...");
            }
        };  // class ServiceNode
        }   // namespace cb_group_demo

        int main(int argc, char* argv[])
        {
            rclcpp::init(argc, argv);
            auto service_node = std::make_shared<cb_group_demo::ServiceNode>();

            RCLCPP_INFO(service_node->get_logger(), "Starting server node, shut down with CTRL-C");
            rclcpp::spin(service_node);
            RCLCPP_INFO(service_node->get_logger(), "Keyboard interrupt, shutting down.\n");

            rclcpp::shutdown();
            return 0;
        }

   .. group-tab:: Python

      .. code-block:: python

        import rclpy
        from rclpy.node import Node
        from std_srvs.srv import Empty

        class ServiceNode(Node):
            def __init__(self):
                super().__init__('service_node')
                self.srv = self.create_service(Empty, 'test_service', callback=self.service_callback)

            def service_callback(self, request, result):
                self.get_logger().info('Received request, responding...')
                return result


        if __name__ == '__main__':
            rclpy.init()
            node = ServiceNode()
            try:
                node.get_logger().info("Starting server node, shut down with CTRL-C")
                rclpy.spin(node)
            except KeyboardInterrupt:
                node.get_logger().info('Keyboard interrupt, shutting down.\n')
            node.destroy_node()
            rclpy.shutdown()

and another containing a client to the service along with a timer for making
service calls:

.. tabs::

  .. group-tab:: C++

    *Note:* The API of service client in rclcpp does not offer a
    synchronous call method similar to the one in rclpy, so we
    wait on the future object to simulate the effect of a
    synchronous call.

    .. code-block:: cpp

      #include <chrono>
      #include <memory>
      #include "rclcpp/rclcpp.hpp"
      #include "std_srvs/srv/empty.hpp"

      using namespace std::chrono_literals;

      namespace cb_group_demo
      {
      class DemoNode : public rclcpp::Node
      {
      public:
          DemoNode() : Node("client_node")
          {
              client_cb_group_ = nullptr;
              timer_cb_group_ = nullptr;
              client_ptr_ = this->create_client<std_srvs::srv::Empty>("test_service", rmw_qos_profile_services_default,
                                                                      client_cb_group_);
              timer_ptr_ = this->create_wall_timer(1s, std::bind(&DemoNode::timer_callback, this),
                                                  timer_cb_group_);
          }

      private:
          rclcpp::CallbackGroup::SharedPtr client_cb_group_;
          rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
          rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_ptr_;
          rclcpp::TimerBase::SharedPtr timer_ptr_;

          void timer_callback()
          {
              RCLCPP_INFO(this->get_logger(), "Sending request");
              auto request = std::make_shared<std_srvs::srv::Empty::Request>();
              auto result_future = client_ptr_->async_send_request(request);
              std::future_status status = result_future.wait_for(10s);  // timeout to guarantee a graceful finish
              if (status == std::future_status::ready) {
                  RCLCPP_INFO(this->get_logger(), "Received response");
              }
          }
      };  // class DemoNode
      }   // namespace cb_group_demo

      int main(int argc, char* argv[])
      {
          rclcpp::init(argc, argv);
          auto client_node = std::make_shared<cb_group_demo::DemoNode>();
          rclcpp::executors::MultiThreadedExecutor executor;
          executor.add_node(client_node);

          RCLCPP_INFO(client_node->get_logger(), "Starting client node, shut down with CTRL-C");
          executor.spin();
          RCLCPP_INFO(client_node->get_logger(), "Keyboard interrupt, shutting down.\n");

          rclcpp::shutdown();
          return 0;
      }

  .. group-tab:: Python

    .. code-block:: python

      import rclpy
      from rclpy.executors import MultiThreadedExecutor
      from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
      from rclpy.node import Node
      from std_srvs.srv import Empty


      class CallbackGroupDemo(Node):
          def __init__(self):
              super().__init__('client_node')

              client_cb_group = None
              timer_cb_group = None
              self.client = self.create_client(Empty, 'test_service', callback_group=client_cb_group)
              self.call_timer = self.create_timer(1, self._timer_cb, callback_group=timer_cb_group)

          def _timer_cb(self):
              self.get_logger().info('Sending request')
              _ = self.client.call(Empty.Request())
              self.get_logger().info('Received response')


      if __name__ == '__main__':
          rclpy.init()
          node = CallbackGroupDemo()
          executor = MultiThreadedExecutor()
          executor.add_node(node)

          try:
              node.get_logger().info('Beginning client, shut down with CTRL-C')
              executor.spin()
          except KeyboardInterrupt:
              node.get_logger().info('Keyboard interrupt, shutting down.\n')
          node.destroy_node()
          rclpy.shutdown()

The client node's constructor contains options for setting the
callback groups of the service client and the timer.
With the default setting above (both being ``nullptr`` / ``None``),
both the timer and the client will use the node's default
Mutually Exclusive Callback Group.

The problem
^^^^^^^^^^^

Since we are making service calls with a 1 second timer, the
expected outcome is that the service gets called once a second,
the client always gets a response and prints ``Received response``.
If we try running the server and client nodes
in terminals, we get the following outputs.

.. tabs::

  .. group-tab:: Client

    .. code-block:: console

      [INFO] [1653034371.758739131] [client_node]: Starting client node, shut down with CTRL-C
      [INFO] [1653034372.755865649] [client_node]: Sending request
      ^C[INFO] [1653034398.161674869] [client_node]: Keyboard interrupt, shutting down.

  .. group-tab:: Server

    .. code-block:: console

      [INFO] [1653034355.308958238] [service_node]: Starting server node, shut down with CTRL-C
      [INFO] [1653034372.758197320] [service_node]: Received request, responding...
      ^C[INFO] [1653034416.021962246] [service_node]: Keyboard interrupt, shutting down.

So, it turns out that instead of the service being called repeatedly,
the response of the first call is never received, after which the
client node seemingly gets stuck and does not make further calls.
That is, the execution stopped at a deadlock!

The reason for this is that the timer callback and the client are
using the same Mutually Exclusive Callback Group (the node's default).
When the service call is made, the client then passes its callback
group to the Future object (hidden inside the call-method in the
Python version) whose done-callback needs to execute for the result
of the service call to be available.
But because this done-callback and the timer callback are in the
same Mutually Exclusive group and the timer callback is still
executing (waiting for the result of the service call),
the done-callback never gets to execute.
The stuck timer callback also blocks any other executions of itself, so the
timer does not fire for a second time.

Solution
^^^^^^^^

We can fix this easily - for example - by assigning the timer and client
to different callback groups.
Thus, let us change the first two lines of the client node's constructor
to be as follows (everything else shall stay the same):

.. tabs::

  .. group-tab:: C++

    .. code-block:: cpp

      client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  .. group-tab:: Python

    .. code-block:: python

      client_cb_group = MutuallyExclusiveCallbackGroup()
      timer_cb_group = MutuallyExclusiveCallbackGroup()

Now we get the expected result, i.e. the timer fires repeatedly and
each service call gets the result as it should:

.. tabs::

  .. group-tab:: Client

    .. code-block:: console

      [INFO] [1653067523.431731177] [client_node]: Starting client node, shut down with CTRL-C
      [INFO] [1653067524.431912821] [client_node]: Sending request
      [INFO] [1653067524.433230445] [client_node]: Received response
      [INFO] [1653067525.431869330] [client_node]: Sending request
      [INFO] [1653067525.432912803] [client_node]: Received response
      [INFO] [1653067526.431844726] [client_node]: Sending request
      [INFO] [1653067526.432893954] [client_node]: Received response
      [INFO] [1653067527.431828287] [client_node]: Sending request
      [INFO] [1653067527.432848369] [client_node]: Received response
      ^C[INFO] [1653067528.400052749] [client_node]: Keyboard interrupt, shutting down.

  .. group-tab:: Server

    .. code-block:: console

      [INFO] [1653067522.052866001] [service_node]: Starting server node, shut down with CTRL-C
      [INFO] [1653067524.432577720] [service_node]: Received request, responding...
      [INFO] [1653067525.432365009] [service_node]: Received request, responding...
      [INFO] [1653067526.432300261] [service_node]: Received request, responding...
      [INFO] [1653067527.432272441] [service_node]: Received request, responding...
      ^C[INFO] [1653034416.021962246] [service_node]: KeyboardInterrupt, shutting down.

One might consider if just avoiding the node's default callback group
is enough.
This is not the case: replacing the default group by a
different Mutually Exclusive group changes nothing.
Thus, the following configuration also leads to the previously
discovered deadlock.

.. tabs::

  .. group-tab:: C++

    .. code-block:: cpp

      client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      timer_cb_group_ = client_cb_group_;

  .. group-tab:: Python

    .. code-block:: python

      client_cb_group = MutuallyExclusiveCallbackGroup()
      timer_cb_group = client_cb_group

In fact, the exact condition with which everything works in this case
is that the timer and client must not belong to the same
Mutually Exclusive group.
Hence, all of the following configurations (and some others as well)
produce the desired outcome where the timer fires
repeatedly and service calls are completed.

.. tabs::

  .. group-tab:: C++

    .. code-block:: cpp

      client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
      timer_cb_group_ = client_cb_group_;

    or

    .. code-block:: cpp

      client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      timer_cb_group_ = nullptr;

    or

    .. code-block:: cpp

      client_cb_group_ = nullptr;
      timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    or

    .. code-block:: cpp

      client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
      timer_cb_group_ = nullptr;

  .. group-tab:: Python

    .. code-block:: python

      client_cb_group = ReentrantCallbackGroup()
      timer_cb_group = client_cb_group

    or

    .. code-block:: python

      client_cb_group = MutuallyExclusiveCallbackGroup()
      timer_cb_group = None

    or

    .. code-block:: python

      client_cb_group = None
      timer_cb_group = MutuallyExclusiveCallbackGroup()

    or

    .. code-block:: python

      client_cb_group = ReentrantCallbackGroup()
      timer_cb_group = None
