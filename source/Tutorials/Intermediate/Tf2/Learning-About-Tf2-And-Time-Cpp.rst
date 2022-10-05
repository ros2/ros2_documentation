.. redirect-from::

    Tutorials/Tf2/Learning-About-Tf2-And-Time-Cpp

.. _LearningAboutTf2AndTimeCpp:

Using time (C++)
================

**Goal:** Learn how to get a transform at a specific time and wait for a transform to be available on the tf2 tree using ``lookupTransform()`` function.

**Tutorial level:** Intermediate

**Time:** 10 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

In previous tutorials, we recreated the turtle demo by writing a :doc:`tf2 broadcaster <Writing-A-Tf2-Broadcaster-Cpp>` and a :doc:`tf2 listener <Writing-A-Tf2-Listener-Cpp>`.
We also learned how to :doc:`add a new frame to the transformation tree <./Adding-A-Frame-Cpp>` and learned how tf2 keeps track of a tree of coordinate frames.
This tree changes over time, and tf2 stores a time snapshot for every transform (for up to 10 seconds by default).
Until now we used the ``lookupTransform()`` function to get access to the latest available transforms in that tf2 tree, without knowing at what time that transform was recorded.
This tutorial will teach you how to get a transform at a specific time.

Tasks
-----

1 tf2 and time
^^^^^^^^^^^^^^

So let's go back to where we ended in the :doc:`adding a frame tutorial <./Adding-A-Frame-Cpp>`.
Go to ``learning_tf2_cpp`` package.
Open ``turtle_tf2_listener.cpp`` and take a look at the ``lookupTransform()`` call:

.. code-block:: C++

   transformStamped = tf_buffer_->lookupTransform(
      toFrameRel,
      fromFrameRel,
      tf2::TimePointZero);

You can see that we specified a time equal to 0 by calling ``tf2::TimePointZero``.

.. note::

    The ``tf2`` package has it's own time type ``tf2::TimePoint``, which is different from ``rclcpp::Time``.
    Many APIs in the package ``tf2_ros`` automatically convert between ``rclcpp::Time`` and ``tf2::TimePoint``.

    ``rclcpp::Time(0, 0, this->get_clock()->get_clock_type())`` could have been used here, but it would have been converted to ``tf2::TimePointZero`` anyways.

For tf2, time 0 means "the latest available" transform in the buffer.
Now, change this line to get the transform at the current time, ``this->get_clock()->now()``:

.. code-block:: C++

   rclcpp::Time now = this->get_clock()->now();
   transformStamped = tf_buffer_->lookupTransform(
      toFrameRel,
      fromFrameRel,
      now);

Now try to run the launch file.

.. code-block:: console

   ros2 launch learning_tf2_cpp turtle_tf2_demo.launch.py

You will notice that it fails and outputs something similar to this:

.. code-block:: console

   [INFO] [1629873136.345688064] [listener]: Could not transform turtle1 to turtle2: Lookup would
   require extrapolation into the future.  Requested time 1629873136.345539 but the latest data
   is at time 1629873136.338804, when looking up transform from frame [turtle1] to frame [turtle2]

It tells you that the frame does not exist or that the data is in the future.

To understand why is this happening we need to understand how buffers work.
Firstly, each listener has a buffer where it stores all the coordinate transforms coming from the different tf2 broadcasters.
Secondly, when a broadcaster sends out a transform, it takes some time before that transform gets into the buffer (usually a couple of milliseconds).
As a result, when you request a frame transform at time "now", you should wait a few milliseconds for that information to arrive.

2 Wait for transforms
^^^^^^^^^^^^^^^^^^^^^

tf2 provides a nice tool that will wait until a transform becomes available.
You use this by adding a timeout parameter to ``lookupTransform()``.
To fix this, edit your code as shown below (add the last timeout parameter):

.. code-block:: C++

   rclcpp::Time now = this->get_clock()->now();
   transformStamped = tf_buffer_->lookupTransform(
      toFrameRel,
      fromFrameRel,
      now,
      50ms);

The ``lookupTransform()`` can take four arguments, where the last one is an optional timeout.
It will block for up to that duration waiting for it to timeout.

3 Checking the results
^^^^^^^^^^^^^^^^^^^^^^

You can now run the launch file.

.. code-block:: console

   ros2 launch learning_tf2_cpp turtle_tf2_demo.launch.py

You should notice that ``lookupTransform()`` will actually block until the transform between the two turtles becomes available (this will usually take a few milliseconds).
Once the timeout has been reached (fifty milliseconds in this case), an exception will be raised only if the transform is still not available.

Summary
-------

In this tutorial, you learned how to acquire a transform at a specific timestamp and how to wait for a transform to be available on the tf2 tree when using the ``lookupTransform()`` function.
