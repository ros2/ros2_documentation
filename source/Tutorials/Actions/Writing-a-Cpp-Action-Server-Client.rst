.. _ActionsCpp:

Writing an action server and client (C++)
=========================================

**Goal:** Implement an action server and client in C++.

**Tutorial level:** Intermediate

**Time:** 15 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

Actions are a form of asynchronous communication in ROS.
*Action clients* send goal requests to *action servers*.
*Action servers* send goal feedback and results to *action clients*.

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
If you'd like to see what a complete package for the actions tutorials looks like, check out
`action_tutorials <https://github.com/ros2/demos/tree/master/action_tutorials>`__.

Open a new file in your home directory, let's call it ``fibonacci_action_server.cpp``,
and add the following boilerplate code:

1.1 Executing goals
~~~~~~~~~~~~~~~~~~~

1.2 Publishing feedback
~~~~~~~~~~~~~~~~~~~~~~~

2 Writing an action client
^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1 Sending a goal
~~~~~~~~~~~~~~~~~~

2.2 Getting feedback
~~~~~~~~~~~~~~~~~~~~

2.3 Getting a result
~~~~~~~~~~~~~~~~~~~~

Summary
-------

In this tutorial, you put together an action server and action client line by line, and configured them to exchange goals, feedback, and results.

Next steps
----------

Next you will create a simple ROS 2 package with a custom parameter that you will learn to set from a launch file.
This tutorial is available in :ref:`C++ <CppParamNode>`.

Related content
---------------

* There are several ways you could write an action server and client in C++; check out the ``minimal_action_server`` and ``minimal_action_client`` packages in the `ros2/examples <https://github.com/ros2/examples/tree/master/rclcpp>`_ repo.

* For more detailed information about ROS actions, please refer to the `design article <http://design.ros2.org/articles/actions.html>`__.
