.. redirect-from::

    dummy-robot-demo

Trying the dummy robot demo
===========================

In this demo, we present a simple demo robot with all components from publishing joint states over publishing fake laser data until visualizing the robot model on a map in RViz.

Launching the demo
------------------

We assume your ROS 2 installation dir as ``~/ros2_ws``. Please change the directories according to your platform.

To start the demo, we execute the demo bringup launch file, which we are going to explain in more details in the next section.

.. code-block:: bash

   source ~/ros2_ws/install/setup.bash
   ros2 launch dummy_robot_bringup dummy_robot_bringup.launch.py

You should see some prints inside your terminal along the lines of the following:

.. code-block:: bash

   [INFO] [launch]: process[dummy_map_server-1]: started with pid [25812]
   [INFO] [launch]: process[robot_state_publisher-2]: started with pid [25813]
   [INFO] [launch]: process[dummy_joint_states-3]: started with pid [25814]
   [INFO] [launch]: process[dummy_laser-4]: started with pid [25815]
   Initialize urdf model from file: /home/mikael/work/ros2/bouncy_ws/install_debug_isolated/dummy_robot_bringup/share/dummy_robot_bringup/launch/single_rrbot.urdf
   Parsing robot urdf xml string.
   Link single_rrbot_link1 had 1 children
   Link single_rrbot_link2 had 1 children
   Link single_rrbot_link3 had 2 children
   Link single_rrbot_camera_link had 0 children
   Link single_rrbot_hokuyo_link had 0 children
   got segment single_rrbot_camera_link
   got segment single_rrbot_hokuyo_link
   got segment single_rrbot_link1
   got segment single_rrbot_link2
   got segment single_rrbot_link3
   got segment world
   Adding fixed segment from world to single_rrbot_link1
   Adding moving segment from single_rrbot_link1 to single_rrbot_link2
   [INFO] [dummy_laser]: angle inc:    0.004363
   [INFO] [dummy_laser]: scan size:    1081
   [INFO] [dummy_laser]: scan time increment:  0.000028
   Adding moving segment from single_rrbot_link2 to single_rrbot_link3
   Adding fixed segment from single_rrbot_link3 to single_rrbot_camera_link
   Adding fixed segment from single_rrbot_link3 to single_rrbot_hokuyo_link

If you now open in a next terminal your RViz, you'll see your robot. 🎉

.. code-block:: bash

   $ source <ROS2_INSTALL_FOLDER>/setup.bash
   $ rviz2

This opens RViz2. Assuming you have your dummy_robot_bringup still launched, you can now add the TF display plugin and configure your global frame to ``world``. Once you did that, you should see a similar picture:


.. image:: https://i.imgur.com/pCFDTCv.png
   :target: https://i.imgur.com/pCFDTCv.png
   :alt:


What's happening?
^^^^^^^^^^^^^^^^^

If you have a closer look at the launch file, we start a couple of nodes at the same time.


* dummy_map_server
* dummy_laser
* dummy_joint_states
* robot_state_publisher

The first two packages are relatively simple. The ``dummy_map_server`` constantly publishes an empty map with a periodic update. The ``dummy_laser`` does basically the same; publishing dummy fake laser scans.

The ``dummy_joint_states`` node is publishing fake joint state data. As we are publishing a simple RRbot with only two joints, this node publishes joint states values for these two joints.

The ``robot_state_publisher`` is doing the actual interesting work. It parses the given URDF file, extracts the robot model and listens to the incoming joint states. With this information, it publishes TF values for our robot which we visualize in RViz.

Hooray!
