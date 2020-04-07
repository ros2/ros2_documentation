.. _ActionsCpp:

Writing an action server and client (C++)
=========================================

**Goal:** Implement an action server and client in C++.

**Tutorial level:** Beginner

**Time:** 15 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

Actions are a form of asynchronous communication in ROS.
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

In this tutorial, you put together an action server and action client, configured to exchange goals, feedback, and results.  

Next steps
----------

Next you will create a simple ROS 2 package with a custom parameter that you will learn to set from a launch file.
This tutorial is available in :ref:`C++ <CppParamNode>`.

Related content
---------------

For reference, after completing this and the previous tutorial, you should expect to have a ROS package that looks like the package `action_tutorials <https://github.com/ros2/demos/tree/master/action_tutorials>`__.

For more detailed information about ROS actions, please refer to the `design article <http://design.ros2.org/articles/actions.html>`__.
