Actions
=======

.. contents:: Table of Contents
   :depth: 2
   :local:

About
-----

Actions are a form of asynchronous communication in ROS.
*Action clients* send goal requests to *action servers*.
Upon receiving a goal, an action server provides feedback messages and, ultimately, a result message when the goal is done.
After a goal request has been accepted by an action server, the action client has the option to cancel the goal before it completes.
For more detailed information about ROS actions, please refer to the `design article <http://design.ros2.org/articles/actions.html>`__.

This document contains tutorials on how to create action servers and action clients as well as using the CLI tool for interaction/introspection.

Creating a new Action
---------------------

In this tutorial we look how to define an action in a ROS package.

Identical to ROS 1, actions are be defined with ``.action`` files of the form:

.. code-block:: bash

    # Request
    ---
    # Result
    ---
    # Feedback

This defines the request, result, and feedback messages for action goals that we intend to send between our action servers and clients.

Say we want to define a new action "Fibonacci" for computing the `Fibonacci sequence <https://en.wikipedia.org/wiki/Fibonacci_number>`__.
Let's start by creating a new package ``actions_tutorial`` in a colcon workspace:

.. code-block:: bash

    mkdir -p action_ws/src
    cd action_ws/src
    ros2 pkg create action_tutorial

Next we add the file ``action/Fibonacci.action`` with the following content:

.. code-block:: bash

    int32 order
    ---
    int32[] sequence
    ---
    int32[] sequence

The goal request is the order of the Fibonacci sequence we want to compute, the result is the final sequence, and the feedback is the sequence computed so far.

Similar to ROS messages and services, we need to generate language specific types for this action by passing our Fibonacci action defintition to the rosidl pipeline.
This is accomplished by adding the following to our ``CMakeLists.txt``:

.. code-block:: cmake

    find_package(action_msgs REQUIRED)
    find_package(rosidl_default_generators REQUIRED)

    rosidl_generate_interfaces(${PROJECT_NAME}
      "action/Fibonacci.action"
      DEPENDENCIES action_msgs
    )

We should also add the required dependencies to our ``package.xml``:

.. code-block:: xml

    <buildtool_depend>rosidl_default_generators</buildtool_depend>

    <depend>action_msgs</depend>

    <!-- We need to use package format 3 for this tag -->
    <member_of_group>rosidl_interface_packages</member_of_group>

Note, we need to depend on ``action_msgs`` since action definitions include additional metadata (e.g. goal IDs).

We're done! We should now be able to build the package containing the "Fibonacci" action definition.

.. code-block:: bash

    # Change to the root of the workspace
    cd ..
    # Build
    colcon build

Writing an Action Server (C++)
------------------------------

Coming soon.


Writing an Action Client (C++)
------------------------------

Coming soon.

Writing an Action Server (Python)
---------------------------------

Here we look at implementing an action server in Python.
Let's walk through the following action server implementation (``fibonacci_action_server.py``):

.. code-block:: python
    :linenos:

    import time

    from action_tutorial.action import Fibonacci

    import rclpy
    from rclpy.action import ActionServer
    from rclpy.node import Node


    class FibonacciActionServer(Node):

        def __init__(self):
            super().__init__('fibonacci_action_server')

            self._action_server = ActionServer(
                self,
                Fibonacci,
                'fibonacci',
                execute_callback=self.execute_callback)

        def destroy(self):
            self._action_server.destroy()
            super().destroy_node()

        def execute_callback(self, goal_handle):
            self.get_logger().info('Executing goal...')

            feedback_msg = Fibonacci.Feedback()
            feedback_msg.sequence = [0, 1]

            for i in range(1, goal_handle.request.order):
                feedback_msg.sequence.append(feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
                self.get_logger().info('Publishing feedback: {0}'.format(feedback_msg.sequence))
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(1)

            goal_handle.succeed()

            result = Fibonacci.Result()
            result.sequence = feedback_msg.sequence
            self.get_logger().info('Returning result: {0}'.format(result.sequence))
            return result


    def main(args=None):
        rclpy.init(args=args)

        fibonacci_action_server = FibonacciActionServer()

        rclpy.spin(fibonacci_action_server)

        fibonacci_action_server.destroy()
        rclpy.shutdown()


    if __name__ == '__main__':
        main()

At line 3 we import our custom action definition from the previous tutorial on `Creating a new Action`_.
Lines 5-7 are importing types from the client library that we need.

Starting at line 10 we define a new class to encapsulate our action server implementation.
It is a subclass of ``Node``.

We initialize the class by calling the ``Node`` constructor, naming our node "fibonacci_action_server":

.. code-block:: python

    super().__init__('fibonacci_action_server')

Also during initialization, we instantiate an ``ActionServer`` with four arguments:

.. code-block:: python

    self._action_server = ActionServer(
        self,
        Fibonacci,
        'fibonacci',
        execute_callback=self.execute_callback)

The first argument is the node to add the action server to (ie. self).
The second argument is the type of the action.
The third argument is the action name.
And the final argument is the function we want called when a new goal is accepted.
Note, all goals are accepted by default.

Line 21-23 defines a ``destroy`` method that is useful for freeing resources used by the node and action server.

Lines 25-42 is the method called whenever we get a new goal request.
It takes one argument that is a handle to the goal.
After logging a message, we create a feedback message:

.. code-block:: python

    feedback_msg = Fibonacci.Feedback()
    feedback_msg.sequence = [0, 1]

Then, we loop up to the requested Fibonacci order (accessed from the goal handle):

.. code-block:: python

    for i in range(1, goal_handle.request.order):

And update the feedback message, publish it, and sleep for dramatic effect:

.. code-block:: python

    feedback_msg.sequence.append(feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
    self.get_logger().info('Publishing feedback: {0}'.format(feedback_msg.sequence))
    goal_handle.publish_feedback(feedback_msg)
    time.sleep(1)

After computing the sequence, we mark the goal as successful:

.. code-block:: python

    goal_handle.succeed()

Finally, we populate the result message and return it:

.. code-block:: python

    result = Fibonacci.Result()
    result.sequence = feedback_msg.sequence
    self.get_logger().info('Returning result: {0}'.format(result.sequence))
    return result

On lines 45-57 We define a main function and call to create an executable.
Because ``FibonacciActionServer`` is a subclass of ``Node`` we can spin on it, which will process our callbacks for any action requests.

Let's run the action server:

.. code-block:: bash

    python3 fibonacci_action_server.py

In another terminal, try sending a goal with the command line tool:

.. code-block:: bash

    ros2 action send_goal -f fibonacci action_tutorial/Fibonacci "{order: 5}"

You should see feedback and the final result sequence printed to both terminals.

Rejecting goals
^^^^^^^^^^^^^^^

Coming soon.

Allowing goals to be canceled
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Coming soon.

Writing an Action Client (Python)
---------------------------------

Coming soon.
