.. redirect-from::

    Rosbag-with-ROS1-Bridge
    Tutorials/Rosbag-with-ROS1-Bridge

Recording and playing back data with ``rosbag`` using the ROS 1 bridge
======================================================================

This tutorial is a follow up to the *Bridge communication between ROS 1 and ROS 2* demo as can be found `here <https://github.com/ros2/ros1_bridge/blob/master/README.md>`__, and in the following it is assumed you have completed that tutorial already.

The ros1_bridge can be built from :doc:`source <../../How-To-Guides/Using-ros1_bridge-Jammy-upstream>` for these examples.

What follows is a series of additional examples, like that ones that come at the end of the aforementioned *Bridge communication between ROS 1 and ROS 2* demo.

Recording topic data with rosbag and ROS 1 Bridge
-------------------------------------------------

In this example, we'll be using the ``cam2image`` demo program that comes with ROS 2 and a Python script to emulate a simple turtlebot-like robot's sensor data so that we can bridge it to ROS 1 and use rosbag to record it.

First we'll run a ROS 1 ``roscore`` in a new shell:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

         $ . /opt/ros/kinetic/setup.bash
         $ roscore

   .. group-tab:: macOS

      .. code-block:: console

         $ . ~/ros_catkin_ws/install_isolated/setup.bash
         $ rocore

Then we'll run the ROS 1 <=> ROS 2 ``dynamic_bridge`` with the ``--bridge-all-topics`` option (so we can do ``rostopic list`` and see them) in another shell:

.. note::

   If you installed rosbridge from source, adapt the path to the setup file accordingly:
   ``. <workspace-with-bridge>/install/setup.bash``.

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        $ . /opt/ros/kinetic/setup.bash
        $ . /opt/ros/ardent/setup.bash
        $ export ROS_MASTER_URI=http://localhost:11311
        $ ros2 run ros1_bridge dynamic_bridge --bridge-all-topics

   .. group-tab:: macOS

      .. code-block:: console

        $ . ~/ros_catkin_ws/install_isolated/setup.bash
        $ . /opt/ros/ardent/setup.bash
        $ export ROS_MASTER_URI=http://localhost:11311
        $ ros2 run ros1_bridge dynamic_bridge --bridge-all-topics


----

Now we can start up the ROS 2 programs that will emulate our turtlebot-like robot.
First we'll run the ``cam2image`` program with the ``-b`` option so it doesn't require a camera to work.
In another shell:

.. code-block:: console

   $ . /opt/ros/ardent/setup.bash
   $ ros2 run image_tools cam2image -- -b

TODO: use namespaced topic names

Then we'll run a simple Python script to emulate the ``odom`` and ``imu_data`` topics from a Kobuki base.
I would use the more accurate ``~sensors/imu_data`` topic name for the imu data, but we don't have namespace support just yet in ROS 2 (it's coming!).
Place this script in a file called ``emulate_kobuki_node.py``:

.. code-block:: python

   #!/usr/bin/env python3

   import sys
   import time

   import rclpy

   from nav_msgs.msg import Odometry
   from sensor_msgs.msg import Imu

   def main():
       rclpy.init(args=sys.argv)

       node = rclpy.create_node('emulate_kobuki_node')

       imu_publisher = node.create_publisher(Imu, 'imu_data')
       odom_publisher = node.create_publisher(Odometry, 'odom')

       imu_msg = Imu()
       odom_msg = Odometry()
       counter = 0
       while True:
           counter += 1
           now = time.time()
           if (counter % 50) == 0:
               odom_msg.header.stamp.sec = int(now)
               odom_msg.header.stamp.nanosec = int(now * 1e9) % 1000000000
               odom_publisher.publish(odom_msg)
           if (counter % 100) == 0:
               imu_msg.header.stamp.sec = int(now)
               imu_msg.header.stamp.nanosec = int(now * 1e9) % 1000000000
               imu_publisher.publish(imu_msg)
               counter = 0
           time.sleep(0.001)


   if __name__ == '__main__':
       sys.exit(main())

You can run this python script in a new ROS 2 shell:

.. code-block:: console

   $ . /opt/ros/ardent/setup.bash
   $ python3 emulate_kobuki_node.py

.. note::

   If building ROS 2 from source adapt the path to the setup file accordingly: ``<workspace-with-bridge>/install/setup.bash``.

----

Now that all the data sources and the dynamic bridge are running, we can look at the available topics in a new ROS 1 shell:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

       $ . /opt/ros/kinetic/setup.bash
       $ rostopic list
       /image
       /imu_data
       /odom
       /rosout
       /rosout_agg

   .. group-tab:: macOS

      .. code-block:: console

       $ . ~/ros_catkin_ws/install_isolated/setup.bash
       $ rostopic list
       /image
       /imu_data
       /odom
       /rosout
       /rosout_agg

We can now record this data with ``rosbag record`` in the same shell:

.. code-block:: console

   $ rosbag record /image /imu_data /odom

After a few seconds you can ``Ctrl-c`` the ``rosbag`` command and do an ``ls -lh`` to see how big the file is, you might see something like this:

.. code-block:: console

   $ ls -lh
   total 0
   -rw-rw-r-- 1 william william  12M Feb 23 16:59 2017-02-23-16-59-47.bag

Though the file name will be different for your bag (since it is derived from the date and time).

Playing back topic data with rosbag and ROS 1 Bridge
----------------------------------------------------

Now that we have a bag file you can use any of the ROS 1 tools to introspect the bag file, like ``rosbag info <bag file>``, ``rostopic list -b <bag file>``, or ``rqt_bag <bag file>``.
However, we can also playback bag data into ROS 2 using ``rosbag play`` and the ROS 1 <=> ROS 2 ``dynamic_bridge``.

First close out all the shells you opened for the previous tutorial, stopping any running programs.

Then in a new shell start the ``roscore``:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

       $ . /opt/ros/kinetic/setup.bash
       $ roscore

   .. group-tab:: macOS

      .. code-block:: console

        $ . ~/ros_catkin_ws/install_isolated/setup.bash
        $ roscore

Then run the ``dynamic_bridge`` in another shell:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

       $ . /opt/ros/kinetic/setup.bash
       $ . /opt/ros/ardent/setup.bash
       $ export ROS_MASTER_URI=http://localhost:11311
       $ ros2 run ros1_bridge dynamic_bridge --bridge-all-topics

   .. group-tab:: macOS

      .. code-block:: console

       $ . ~/ros_catkin_ws/install_isolated/setup.bash
       $ . /opt/ros/ardent/setup.bash
       $ export ROS_MASTER_URI=http://localhost:11311
       $ ros2 run ros1_bridge dynamic_bridge --bridge-all-topics

Then play the bag data back with ``rosbag play`` in another new shell, using the ``--loop`` option so that we don't have to keep restarting it for short bags:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        $ . /opt/ros/kinetic/setup.bash
        $ rosbag play --loop path/to/bag_file

   .. group-tab:: macOS

      .. code-block:: console

        $ . ~/ros_catkin_ws/install_isolated/setup.bash
        $ rosbag play --loop path/to/bag_file

.. note::

   Make sure to replace ``path/to/bag_file`` with the path to the bag file you want to play back.

----

Now that the data is being played back and the bridge is running we can see the data coming across in ROS 2.

.. code-block:: console

   $ . /opt/ros/ardent/setup.bash
   $ ros2 topic list
   /clock
   /image
   /imu_data
   /odom
   /parameter_events
   $ ros2 topic echo /odom

You can also see the image being played from the bag by using the ``showimage`` tool:

.. code-block:: console

   $ ros2 run image_tools showimage
