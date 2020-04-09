.. _ActionsPy:

Writing an action server and client (Python)
============================================

**Goal:** Implement an action server and client in Python.

**Tutorial level:** Beginner

**Time:** 15 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

Actions are a form of asynchronous communication in ROS 2.
*Action clients* send goal requests to *action servers*.
*Action servers* send goal feedback and results to *action clients*.

After completing :ref:`ActionCreate` and this tutorial,
you should expect to have a ROS package that looks like the package
`action_tutorials <https://github.com/ros2/demos/tree/master/action_tutorials>`__.

Prerequisites
-------------

You will need the ``action_tutorials`` package and the ``Fibonacci.action``
interface defined in the previous tutorial, :ref:`ActionCreate`.

Tasks
-----

1 Writing an action server
^^^^^^^^^^^^^^^^^^^^^^^^^^

Let's focus on writing an action server that computes the Fibonacci sequence
using the action we created in the :ref:`ActionCreate` tutorial.

Until now, you've created packages and used ``ros2 run`` to run your nodes.
To keep things simple in this tutorial, however, we’ll scope the action server to a single file.

Open a new file in your home directory, let's call it ``fibonacci_action_server.py``,
and add the following boilerplate code:

.. literalinclude:: server_0.py
    :language: python
    :linenos:
    :lines: 1,3,6-11,21-32

We've defined a class ``FibonacciActionServer`` that is a subclass of ``Node``.
The class is initialized by calling the ``Node`` constructor, naming our node ``fibonacci_action_server``:

.. literalinclude:: server_0.py
    :language: python
    :lines: 11

After the class definition, we define a function ``main()`` that initializes ROS 2,
creates an instance of our ``FibonacciActionServer`` node, and calls
`rclpy.spin() <http://docs.ros2.org/latest/api/rclpy/api/init_shutdown.html#rclpy.spin>`_ on our node.
The spin will keep our action server alive and responsive to incoming goals.
Finally, we call ``main()`` in the entry point of our Python program.

Next, we'll import our Fibonacci action definition and create an action server:

.. literalinclude:: server_0.py
    :language: python
    :linenos:
    :lines: 1-20
    :emphasize-lines: 2,5,12-20

1.1 Executing goals
~~~~~~~~~~~~~~~~~~~

The action server requires four arguments:

1. A ROS 2 node to add the action client to: ``self``
2. The type of the action: ``Fibonacci``
3. The action name: ``'fibonacci'``
4. A callback function for executing accepted goals: ``self.execute_callback``.
   This callback **must** return a result message for the action type.

Note, all goals are accepted by default.

Let's try running our action server:

.. tabs::

  .. group-tab:: Linux/macOS

    .. code-block:: bash

      python3 fibonacci_action_server.py

  .. group-tab:: Windows

    .. code-block:: bash

      python fibonacci_action_server.py

In another terminal, we can use the command line interface to send a goal:

.. code-block:: bash

    ros2 action send_goal fibonacci action_tutorials/action/Fibonacci "{order: 5}"

You should see our logged message "Executing goal..." followed by a warning that the goal state was not set.
By default, if the goal handle state is not set in the execute callback it assumes the *aborted* state.

We can use the method `succeed() <http://docs.ros2.org/latest/api/rclpy/api/actions.html#rclpy.action.server.ServerGoalHandle.succeeded>`_ on the goal handle to indicate that the goal was successful:

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

1.2 Publishing feedback
~~~~~~~~~~~~~~~~~~~~~~~

One of the nice things about actions is the ability to provide feedback to an action client during goal execution.
We can make our action server publish feedback for action clients by calling the goal handle's `publish_feedback() <http://docs.ros2.org/latest/api/rclpy/api/actions.html#rclpy.action.server.ServerGoalHandle.publish_feedback>`_ method.

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

2 Writing an action client
^^^^^^^^^^^^^^^^^^^^^^^^^^

We'll also scope the action client to a single file.
Open a new file, let's call it ``fibonacci_action_client.py``, and add the following boilerplate code:

.. literalinclude:: client_0.py
    :language: python
    :linenos:
    :lines: 1,3,6-11,13-22

We've defined a class ``FibonacciActionClient`` that is a subclass of ``Node``.
The class is initialized by calling the ``Node`` constructor, naming our node ``fibonacci_action_client``:

.. literalinclude:: client_0.py
    :language: python
    :lines: 11

After the class definition, we define a function ``main()`` that initializes ROS 2
and creates an instance of our ``FibonacciActionClient`` node.
Finally, we call ``main()`` in the entry point of our Python program.

You can try running the program:

.. tabs::

  .. group-tab:: Linux/macOS

    .. code-block:: bash

      python3 fibonacci_action_client.py

  .. group-tab:: Windows

    .. code-block:: bash

      python fibonacci_action_client.py

It doesn't do anything interesting...yet.

Let's import and create an action client using the custom action definition from
the previous tutorial on :ref:`ActionCreate`.

.. literalinclude:: client_0.py
    :language: python
    :linenos:
    :lines: 1-12
    :emphasize-lines: 2,5,12

At line 12 we create an ``ActionClient`` by passing it three arguments:

1. A ROS 2 node to add the action client to: ``self``
2. The type of the action: ``Fibonacci``
3. The action name: ``'fibonacci'``

Our action client will be able to communicate with action servers of the same action name and type.

2.1 Sending a goal
~~~~~~~~~~~~~~~~~~

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

Let's test our action client by first running the action server built earlier:

.. tabs::

  .. group-tab:: Linux/macOS

    .. code-block:: bash

      python3 fibonacci_action_server.py

  .. group-tab:: Windows

    .. code-block:: bash

      python fibonacci_action_server.py

In another terminal, run the action client:

.. tabs::

  .. group-tab:: Linux/macOS

    .. code-block:: bash

      python3 fibonacci_action_client.py

  .. group-tab:: Windows

    .. code-block:: bash

      python fibonacci_action_client.py

Tada! You should see messages printed by the action server as it successfully executes the goal.

2.2 Getting feedback
~~~~~~~~~~~~~~~~~~~~

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
This is because we're missing a call to `rclpy.spin() <http://docs.ros2.org/latest/api/rclpy/api/init_shutdown.html#rclpy.spin>`_ in order to process callbacks on our node.
Let's add it:

.. literalinclude:: client_2.py
    :language: python
    :linenos:
    :lines: 27-34
    :emphasize-lines: 8

We're all set. If we run our action client, you should see feedback being printed to the screen.

2.3 Getting a result
~~~~~~~~~~~~~~~~~~~~

So we can send a goal, but how do we know when it is completed?
We can get the result information with a couple steps.
First, we need to get a goal handle for the goal we sent.
Then, we can use the goal handle to request the result.

The `ActionClient.send_goal_async() <http://docs.ros2.org/latest/api/rclpy/api/actions.html#rclpy.action.client.ActionClient.send_goal_async>`_ method returns a future to a goal handle.
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

Now that we've got a goal handle, we can use it to request the result with the method `get_result_async() <http://docs.ros2.org/latest/api/rclpy/api/actions.html#rclpy.action.client.ClientGoalHandle.get_result_async>`_.
Similar to sending the goal, we will get a future that will complete when the result is ready.
Let's register a callback just like we did for the goal response:

.. literalinclude:: client_3.py
    :language: python
    :linenos:
    :lines: 26-42
    :emphasize-lines: 10-17

In the callback, we log the result sequence and shutdown ROS 2 for a clean exit.

With an action server running in a separate terminal, go ahead and try running our Fibonacci action client!

.. tabs::

  .. group-tab:: Linux/macOS

    .. code-block:: bash

      python3 fibonacci_action_client.py

  .. group-tab:: Windows

    .. code-block:: bash

      python fibonacci_action_client.py

You should see logged messages for the goal being accepted, feedback, and the final result.

Summary
-------

In this tutorial, you put together an action server and action client, configured to exchange goals, feedback, and results.

Next steps
----------

Next you will create a simple ROS 2 package with a custom parameter that you will learn to set from a launch file.
This tutorial is available in :ref:`C++ <CppParamNode>`.

Related content
---------------

* There are several ways you could write an action server and client in Python; check out the ``minimal_action_server`` and ``minimal_action_client`` packages in the `ros2/examples <https://github.com/ros2/examples/tree/master/rclpy/actions>`_ repo.

* For more detailed information about ROS actions, please refer to the `design article <http://design.ros2.org/articles/actions.html>`__.
