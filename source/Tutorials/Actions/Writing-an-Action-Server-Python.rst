Writing an Action Server (Python)
=================================

In this tutorial, we look at implementing an action server in Python.

Make sure you have satisfied all `prerequisites <../Actions>`.

Executing Goals
---------------

Let's focus on writing an action server that computes the Fibonacci sequence using the action we created in the `Creating an Action <Creating-an-Action>` tutorial.

To keep things simple, we'll scope this tutorial to a single file.
Open a new file, let's call it ``fibonacci_action_server.py``, and add the following boilerplate code:

.. literalinclude:: server_0.py
    :language: python
    :linenos:
    :lines: 1,3,6-11,21-32

We've defined a class ``FibonacciActionServer`` that is a subclass of ``Node``.
The class is initialized by calling the ``Node`` constructor, naming our node "fibonacci_action_server":

.. literalinclude:: server_0.py
    :language: python
    :lines: 11

After the class defintion, we define a function ``main()`` that initializes ROS, creates an instance of our ``FibonacciActionServer`` node, and calls `rclpy.spin() <https://docs.ros2.org/latest/api/rclpy/api/init_shutdown.html#rclpy.spin>`_ on our node.
The spin will keep our action server alive and responsive to incoming goals.
Finally, we call ``main()`` in the entry point of our Python program.

Next, we'll import our Fibonacci action definition and create an action server:

.. literalinclude:: server_0.py
    :language: python
    :linenos:
    :lines: 1-20
    :emphasize-lines: 2,5,12-20

The action server requires four arguments:

1. a ROS node to add the action client to: ``self``.
2. the type of the action: ``Fibonacci``.
3. the action name: ``'fibonacci'``.
4. a callback function for executing accepted goals: ``self.execute_callback``.
   This callback **must** return a result message for the action type.

Note, all goals are accepted by default.

Let's try running our action server:

.. code-block:: bash

    # Linux/OSX
    python3 fibonacci_action_server.py
    # Windows
    python fibonacci_action_server.py

In another terminal, we can use the command line interface to send a goal:

.. code-block:: bash

    ros2 action send_goal fibonacci action_tutorials/action/Fibonacci "{order: 5}"

You should see our logged message "Executing goal..." followed by a warning that the goal state was not set.
By default, if the goal handle state is not set in the execute callback it assumes the *aborted* state.

We can use the method `succeed() <https://docs.ros2.org/latest/api/rclpy/api/actions.html#rclpy.action.server.ServerGoalHandle.succeeded>`_ on the goal handle to indicate that the goal was successful:

.. literalinclude:: server_1.py
    :language: python
    :linenos:
    :lines: 18-21
    :emphasize-lines: 3

Now if you restart the action server and send another goal, you should see the goal finished with the status ``SUCCEEDED``.

Alright, let's make our goal execution actually compute and return the requested Fibonacci sequence:

.. literalinclude:: server_2.py
    :language: python
    :linenos:
    :lines: 18-30
    :emphasize-lines: 4-7,11-13

After computing the sequence, we assign it to the result message field before returning.

Again, restart the action server and send another goal.
You should see the goal finish with the proper result sequence.

Publishing Feedback
-------------------

One of the nice things about actions is the ability to provide feedback to an action client during goal execution.
We can make our action server publish feedback for action clients by calling the goal handle's `publish_feedback() <https://docs.ros2.org/latest/api/rclpy/api/actions.html#rclpy.action.server.ServerGoalHandle.publish_feedback>`_ method.

We'll replace the ``sequence`` variable, and use a feedback message to store the sequence instead.
After every update of the feedback message in the for-loop, we publish the feedback message and sleep for dramatic effect:

.. literalinclude:: server_3.py
    :language: python
    :linenos:
    :lines: 1-37
    :emphasize-lines: 1,23,24,27-31,36

After restarting the action server, we can confirm that feedback is now published by using the command line tool with the ``--feedback`` option:

.. code-block:: bash

    ros2 action send_goal --feedback fibonacci action_tutorials/action/Fibonacci "{order: 5}"
