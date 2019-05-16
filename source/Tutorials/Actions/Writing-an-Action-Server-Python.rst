Writing an Action Server (Python)
---------------------------------

In this tutorial, we look at implementing an action server in Python.

We assume you have the `prequisites <../Actions>` and have completed the `Creating an Action tutorial <Creating-an-Action>`.

Let's walk through the following action server implementation (``fibonacci_action_server.py``):

.. code-block:: python
    :linenos:

    import time

    from action_tutorials.action import Fibonacci

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
            feedback_msg.partial_sequence = [0, 1]

            for i in range(1, goal_handle.request.order):
                feedback_msg.partial_sequence.append(
                    feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
                self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(1)

            goal_handle.succeed()

            result = Fibonacci.Result()
            result.sequence = feedback_msg.partial_sequence
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

At line 3 we import our custom action definition from the previous tutorial on `Creating an Action <Creating-an-Action>`.
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

Lines 25-43 is the method called whenever we get a new goal request.
It takes one argument that is a handle to the goal.
After logging a message, we create a feedback message:

.. code-block:: python

    feedback_msg = Fibonacci.Feedback()
    feedback_msg.partial_sequence = [0, 1]

Then, we loop up to the requested Fibonacci order (accessed from the goal handle):

.. code-block:: python

    for i in range(1, goal_handle.request.order):

And update the feedback message, publish it, and sleep for dramatic effect:

.. code-block:: python

    feedback_msg.partial_sequence.append(
        feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
    self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))
    goal_handle.publish_feedback(feedback_msg)
    time.sleep(1)

Now that the goal is done, we need to tell the client the result.
We'll use a method on the ``goal_handle`` to tell the client the goal was successfully reached:

.. code-block:: python

    goal_handle.succeed()

Finally, populate the result message and return it:

.. code-block:: python

    result = Fibonacci.Result()
    result.sequence = feedback_msg.sequence
    self.get_logger().info('Returning result: {0}'.format(result.sequence))
    return result

On lines 46-58 We define a main function and call to create an executable.
Because ``FibonacciActionServer`` is a subclass of ``Node`` we can spin on it, which will process our callbacks for any action requests.

Let's run the action server:

.. code-block:: bash

    python3 fibonacci_action_server.py

In another terminal, try sending a goal with the command line tool:

.. code-block:: bash

    ros2 action send_goal -f fibonacci action_tutorial/Fibonacci "{order: 5}"

You should see feedback and the final result sequence printed to both terminals.
