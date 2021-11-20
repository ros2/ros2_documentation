.. _ROS-2-Client-Libraries:

Executors
=========

.. contents:: Table of Contents
   :local:

Overview
--------

Execution management in ROS 2 is explicated by the concept of Executors.
An Executor uses one or more threads of the underlying operating system to invoke the callbacks of subscriptions, timers, service servers, action servers, etc. on incoming messages or events.
The explicit Executor class provides more control over execution management than the spin mechanism in ROS 1, although the basic API is very similar.

