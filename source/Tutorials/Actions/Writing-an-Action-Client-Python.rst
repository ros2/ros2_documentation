Writing an Action Client (Python)
=================================

In this tutorial, we look at implementing an action client in Python.

Make sure you have satisfied all `prerequisites <../Actions>`.

Sending a Goal
--------------

Let's get started!

To keep things simple, we'll scope this tutorial to a single file.
Open a new file, let's call it ``fibonacci_action_client.py``, and add the following boilerplate code:

.. literalinclude:: client_0.py
    :language: python
    :linenos:
    :lines: 1,3,6-11,13-22

We've defined a class ``FibonacciActionClient`` that is a subclass of ``Node``.
The class is initialized by calling the ``Node`` constructor, naming our node "fibonacci_action_client":

.. literalinclude:: client_0.py
    :language: python
    :lines: 11

After the class definition, we define a function ``main()`` that initializes ROS and creates an instance of our ``FibonacciActionClient`` node.
Finally, we call ``main()`` in the entry point of our Python program.

You can try running the program:

.. code-block:: bash

    # Linux/OSX
    python3 fibonacci_action_client.py
    # Windows
    python fibonacci_action_client.py

It doesn't do anything interesting...yet.

Let's import and create an action client using the custom action definition from the previous tutorial on `Creating an Action <Creating-an-Action>`.

.. literalinclude:: client_0.py
    :language: python
    :linenos:
    :lines: 1-12
    :emphasize-lines: 2,5,12

At line 12 we create an ``ActionClient`` by passing it three arguments:

1. a ROS node to add the action client to: ``self``.
2. the type of the action: ``Fibonacci``.
3. the action name: ``'fibonacci'``.

Our action client will be able to communicate with action servers of the same action name and type.

Now let's tell the action client to send a goal.
Add a new method ``send_goal()``:

.. literalinclude:: client_1.py
    :language: python
    :linenos:
    :lines: 8-20
    :emphasize-lines: 7-13

We create a new Fibonacci goal message and assign a sequence order.
Before sending the goal message, we must wait for an action server to become available.
Otherwise, a potential action server may miss the goal we're sending.

Finally, call the ``send_goal`` method with a value:

.. literalinclude:: client_1.py
    :language: python
    :linenos:
    :lines: 27-32
    :emphasize-lines: 6

Let's test our action client by first running an action server built in the tutorial on `Writing an Action Server (Python) <Writing-an-Action-Server-Python>`:

.. code-block:: bash

    # Linux/OSX
    python3 fibonacci_action_server.py
    # Windows
    python fibonacci_action_server.py

In another terminal, run the action client:

.. code-block:: bash

    # Linux/OSX
    python3 fibonacci_action_client.py
    # Windows
    python fibonacci_action_client.py

Tada! You should see messages printed by the action server as it successfully executes the goal.

Getting Feedback
----------------

Our action client can send goals.
Nice!
But it would be great if we could get some feedback about the goals we send from the action server.
Easy, let's write a callback function for feedback messages:

.. literalinclude:: client_1.py
    :language: python
    :linenos:
    :lines: 8-24
    :emphasize-lines: 15-17

In the callback we get the feedback portion of the message and print the ``partial_sequence`` field to the screen.

We need to register the callback with the action client.
This is achieved by passing the callback to the action client when we send a goal:

.. literalinclude:: client_2.py
    :language: python
    :linenos:
    :lines: 14-21
    :emphasize-lines: 7

You'll notice at this point that our action client is not printing any feedback.
This is because we're missing a call to `rclpy.spin() <https://docs.ros2.org/latest/api/rclpy/api/init_shutdown.html#rclpy.spin>`_ in order to process callbacks on our node.
Let's add it:

.. literalinclude:: client_2.py
    :language: python
    :linenos:
    :lines: 27-34
    :emphasize-lines: 8

We're all set. If we run our action client, you should see feedback being printed to the screen.

Getting a Result
----------------

So we can send a goal, but how do we know when it is completed?
We can get the result information with a couple steps.
First, we need to get a goal handle for the goal we sent.
Then, we can use the goal handle to request the result.

The `ActionClient.send_goal_async() <https://docs.ros2.org/latest/api/rclpy/api/actions.html#rclpy.action.client.ActionClient.send_goal_async>`_ method returns a future to a goal handle.
Let's register a callback for when the future is complete:

.. literalinclude:: client_3.py
    :language: python
    :linenos:
    :lines: 8-27
    :emphasize-lines: 13-20


Note, the future is completed when an action server accepts or rejects the goal request.
We can actually check to see if the goal was rejected and return early since we know there will be no result:

.. literalinclude:: client_3.py
    :language: python
    :linenos:
    :lines: 26-33
    :emphasize-lines: 4-8

Now that we've got a goal handle, we can use it to request the result with the method `get_result_async() <https://docs.ros2.org/latest/api/rclpy/api/actions.html#rclpy.action.client.ClientGoalHandle.get_result_async>`_.
Similar to sending the goal, we will get a future that will complete when the result is ready.
Let's register a callback just like we did for the goal response:

.. literalinclude:: client_3.py
    :language: python
    :linenos:
    :lines: 26-42
    :emphasize-lines: 10-17

In the callback, we log the result sequence and shutdown ROS for a clean exit.

With an action server running in a separate terminal, go ahead and try running our Fibonacci action client!

.. code-block:: bash

    # Linux/OSX
    python3 fibonacci_action_client.py
    # Windows
    python fibonacci_action_client.py

You should see logged messages for the goal being accepted, feedback, and the final result.
