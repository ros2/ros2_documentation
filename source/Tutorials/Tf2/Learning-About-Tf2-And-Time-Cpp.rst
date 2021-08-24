.. _LearningAboutTf2AndTimeCpp:

Learning about tf2 and time (C++)
=================================

**Goal:** Learn to use the ``timeout`` in ``lookupTransform`` function to wait for a transform to be available on the tf2 tree.

**Tutorial level:** Intermediate

**Time:** 10 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

In previous tutorials, we recreated the turtle demo by writing a :ref:`tf2 broadcaster <WritingATf2BroadcasterCpp>` and a :ref:`tf2 listener <WritingATf2ListenerCpp>`.
We also learned how to :ref:`add a new frame to the transformation tree <AddingAFrameCpp>` and learned how tf2 keeps track of a tree of coordinate frames.
This tree changes over time, and tf2 sutores a time snapshot for every transform (for up to 10 seconds by default).
Until now we used the ``lookpTransform()`` function to get access to the latest available transforms in that tf2 tree, without knowing at what time that transform was recorded.
This tutorial will teach you how to get a transform at a specific time.

Tasks
-----

1 Update the listener node
^^^^^^^^^^^^^^^^^^^^^^^^^^

Edit ``turtle_tf2_listener.cpp`` and remove the last timeout parameter that is passed to the ``lookupTransform()`` call on line 108.
It should look like shown below:

.. code-block:: C++

   transformStamped = tf_buffer_->lookupTransform(
      toFrameRel, fromFrameRel,
      tf2::TimePointZero);

Moreover, import additional exceptions that we will handle in the beggining of the file:

.. code-block:: C++

   from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

Edit the exception handling on line ??? by adding newly imported exceptions and ``raise`` statement to see the exception:

.. code-block:: C++

   except (LookupException, ConnectivityException, ExtrapolationException):
      self.get_logger().info('transform not ready')
      raise
      return

If you now try to run the launch file, you will notice that it is failing:

.. code-block:: console

   ros2 launch learning_tf2_py turtle_tf2_demo.launch.py

2 Fix the listener node
^^^^^^^^^^^^^^^^^^^^^^^

You now should notice that ``lookupTransform()`` is failing. It tells you that the frame does not exist or that the data is in the future.
To fix this, edit your code on line ??? as shown below (return the ``timeout`` parameter):

.. code-block:: C++

   trans = self._tf_buffer.lookupTransform(
      to_frame_rel,
      from_frame_rel,
      now,
      timeout=Duration(seconds=1.0))

The ``lookupTransform`` can take four arguments, where the last one is an optional timeout.
It will block for up to that duration waiting for it to timeout.

.. note::

   Once this change is made, remove the ``raise`` line from the ``except()`` block that we added above or the code will continue to fail.

You can now run the launch file.

.. code-block:: console

   ros2 launch learning_tf2_py turtle_tf2_demo.launch.py

You should notice that ``lookupTransform()`` will actually block until the transform between the two turtles becomes available (this will usually take a few milli-seconds).
Once the timeout has been reached (one second in this case), an exception will be raised only if the transform is still not available.

Summary
-------

This tutorial teaches you to wait for a transform to be available on the tf2 tree when using the ``lookupTransform()`` function.
