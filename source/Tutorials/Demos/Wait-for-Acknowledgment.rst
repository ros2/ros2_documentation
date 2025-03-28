Wait for acknowledgment
=======================

**Goal:** Wait for acknowledgment of messages sent by a publisher.

**Tutorial level:** Advanced

**Time:** 10 minutes

.. contents:: Table of Contents
   :depth: 1
   :local:

Overview
--------

In Publisher-Subscriber architecture, messages are sent from the publisher to the subscribers, and the publisher does not have any built-in mechanism to confirm that the subscriber has received the messages.
This feature enables the publisher to wait for acknowledgment of messages it sent.
This is useful in scenarios where the publisher needs to ensure that the subscriber has received the message before proceeding with further actions, such as sending more messages or performing other operations.

RMW Support
-----------

Wait for acknowledgment requires RMW implementation support.

.. list-table::  Wait-for-Acknowledgment Support Status
   :widths: 25 25

   * - rmw_fastrtps
     - supported
   * - rmw_connextdds
     - supported
   * - rmw_cyclonedds
     - supported

The publisher's :ref:`QoS reliability policy <about_qos_policies>` needs to be ``RELIABLE`` to use the wait for acknowledgment feature, otherwise the publisher will not wait for acknowledgment.

Installing the demo
-------------------

See the :doc:`installation instructions <../../Installation>` for details on installing ROS 2.

If you've installed ROS 2 from packages, ensure that you have ``ros-{DISTRO}-examples-rclcpp-minimal-publisher`` and ``ros-{DISTRO}-examples-rclcpp-minimal-subscriber`` installed.
If you downloaded the archive or built ROS 2 from source, it will already be part of the installation.

Running the demo
----------------

This demo shows how to use the wait for acknowledgment feature in the publisher to ensure that messages sent by the publisher are acknowledged by all subscriptions.

https://github.com/ros2/examples/blob/{REPOS_FILE_BRANCH}/rclcpp/topics/minimal_publisher/member_function_with_wait_for_all_acked.cpp

The publisher can use the ``wait_for_all_acked`` method to wait for message acknowledgments within a specified timeout before shutdown by the signal.

We can start the demo by running the ``publisher_wait_for_all_acked`` and ``subscriber_member_function`` executables from the ``examples_rclcpp_minimal_publisher`` package (don't forget to source the setup file first):

Start the subscriber in one terminal:

.. code-block:: console

    $ ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function
    [INFO] [1743121567.030751270] [minimal_subscriber]: I heard: 'Hello, world! 0'
    [INFO] [1743121567.530981660] [minimal_subscriber]: I heard: 'Hello, world! 1'
    [INFO] [1743121568.031032935] [minimal_subscriber]: I heard: 'Hello, world! 2'
    [INFO] [1743121568.531048458] [minimal_subscriber]: I heard: 'Hello, world! 3'
    [INFO] [1743121569.031049351] [minimal_subscriber]: I heard: 'Hello, world! 4'
    [INFO] [1743121569.530980327] [minimal_subscriber]: I heard: 'Hello, world! 5'
    [INFO] [1743121570.030825871] [minimal_subscriber]: I heard: 'Hello, world! 6'
    ...

Then start the publisher in another terminal:

.. code-block:: console

    $ ros2 run examples_rclcpp_minimal_publisher publisher_wait_for_all_acked
    [INFO] [1743121567.030353553] [minimal_publisher_with_wait_for_all_acked]: Publishing: 'Hello, world! 0'
    [INFO] [1743121567.530420788] [minimal_publisher_with_wait_for_all_acked]: Publishing: 'Hello, world! 1'
    [INFO] [1743121568.030461599] [minimal_publisher_with_wait_for_all_acked]: Publishing: 'Hello, world! 2'
    [INFO] [1743121568.530435646] [minimal_publisher_with_wait_for_all_acked]: Publishing: 'Hello, world! 3'
    [INFO] [1743121569.030431263] [minimal_publisher_with_wait_for_all_acked]: Publishing: 'Hello, world! 4'
    [INFO] [1743121569.530447106] [minimal_publisher_with_wait_for_all_acked]: Publishing: 'Hello, world! 5'
    [INFO] [1743121570.030353934] [minimal_publisher_with_wait_for_all_acked]: Publishing: 'Hello, world! 6'
    ^C[INFO] [1743121570.344981639] [rclcpp]: signal_handler(signum=2)
    [INFO] [1743121570.345398788] [minimal_publisher_with_wait_for_all_acked]: All subscribers acknowledge messages

When the publisher is terminated (e.g., by pressing :kbd:`Ctrl-C`), it will wait for acknowledgment of all messages sent before shutdown.
If all subscribers acknowledge the messages, the publisher will print a message indicating that all subscribers have acknowledged the messages.
If not, it will print a message indicating that not all subscribers acknowledged the messages within the specified timeout.

Related content
---------------

- `Wait-for-Acknowledgment example with rclpy <https://github.com/ros2/examples/blob/{REPOS_FILE_BRANCH}/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function_with_wait_for_all_acked.py>`__.
