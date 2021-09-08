.. _DebuggingTf2Problems:

Debugging tf2 Problems
======================

**Goal:** Learn how to use a systematic approach for debugging tf2 related problems.

**Tutorial level:** Advanced

**Time:** 10 minutes

.. contents:: Contents
   :depth: 2
   :local:

1 Setting and starting the example
----------------------------------

This tutorial walks you through the steps to debug a typical tf2 problem.
It will also use many of the tf2 debugging tools, such as ``tf2_echo``, ``tf2_monitor``, ``view_frames``.

For this tutorial we will set up a demo application which has a number of problems.
The goal of this tutorial is to apply a systematic approach to find and tackle these problems.
First, let's create the source file. 

Go to the ``learning_tf2_cpp`` package we created in the previous tutorial.
Inside the ``src`` directory make a copy of the source file ``turtle_tf2_listener.cpp`` and rename it as ``turtle_tf2_listener_debug.cpp``. 

Open the file using your preferred text editor, and change line 67 from:

.. code-block:: C++

   std::string to_frame_rel = "turtle2";

to:

.. code-block:: C++

   std::string to_frame_rel = "turtle3";

and change line 75-79 from:

.. code-block:: C++

   try {
      transformStamped = tf_buffer_->lookupTransform(
        toFrameRel, fromFrameRel, 
        tf2::TimePointZero);
   } catch (tf2::TransformException & ex) {

to:

.. code-block:: C++

   try {
      transformStamped = tf_buffer_->lookupTransform(
        toFrameRel, fromFrameRel,  
        this->now());
   } catch (tf2::TransformException & ex) {

And save changes to the file.
In order to run this demo, we need to create a launch file ``start_tf2_debug_demo.launch.py`` in the ``launch`` subdirectory of package ``learning_tf2_cpp``:

.. code-block:: python

   from launch import LaunchDescription
   from launch.actions import DeclareLaunchArgument
   from launch.substitutions import LaunchConfiguration

   from launch_ros.actions import Node

   def generate_launch_description():
      return LaunchDescription([
         DeclareLaunchArgument(
            'target_frame', default_value='turtle1',
            description='Target frame name.'
         ),
         Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            output='screen'
         ),
         Node(
            package='learning_tf2_cpp',
            executable='turtle_tf2_broadcaster',
            name='broadcaster1',
            parameters=[
                  {'turtlename': 'turtle1'}
            ]
         ),
         Node(
            package='learning_tf2_cpp',
            executable='turtle_tf2_broadcaster',
            name='broadcaster2',
            parameters=[
                  {'turtlename': 'turtle2'}
            ]
         ), 
         Node(
            package='learning_tf2_cpp',
            executable='turtle_tf2_listener_debug',
            name='listener_debug',
            parameters=[
                  {'target_frame': LaunchConfiguration('target_frame')}
            ]
         ),
      ])

Don't forget to add the executable in the ``CMakeLists.txt`` of the package and build the package.

Now let's run it to see what happens:

.. code-block:: console

   ros2 launch learning_tf2_cpp start_tf2_debug_demo.launch.py

You'll see the :ref:`turtlesim <Turtlesim>` come up.
At the same time, if you run the ``turtle_teleop_key`` node of ``turtlesim`` package in another terminal window, you can use the arrow keys to drive the ``turtle1`` robot around.
In the lower left corner there is a second robot.

If the demo would be working correctly, this second robot should be following the robot you can command with the arrow keys.
Obviously, it does not... because we have to solve some problems first. What you do see, is the following message:

.. code-block:: console

   [turtle_tf2_listener_debug-4] [INFO] [1630223454.942322623] [listener_debug]: Could not
   transform turtle3 to turtle1: "turtle3" passed to lookupTransform argument target_frame
   does not exist

2 Finding the tf2 request
-------------------------

So, we first need to find out what exactly we are asking tf2 to do. Therefore we go into the part of the code that is using tf2. Open the ``src/turtle_tf2_listener_debug.cpp`` file, and take a look at line 67:

.. code-block:: C++

   std::string to_frame_rel = "turtle3";

and lines 75-79:

.. code-block:: C++

   try {
      transformStamped = tf_buffer_->lookupTransform(
        toFrameRel, fromFrameRel,  
        this->now());
   } catch (tf2::TransformException & ex) {

Here we do the actual request to tf2. The three arguments tell us directly what we are asking tf2: transform from frame ``"turtle3"`` to frame ``"turtle1"`` at time ``"now"``.

Now, let's take a look at why this request to tf2 is failing.


3 Checking the frames
---------------------

First we want to find out if tf2 knows about our transform between ``turtle3`` and ``turtle1``:

.. code-block:: console

   ros2 run tf2_ros tf2_echo turtle3 turtle1

The output tells us that frame ``"turtle3"`` does not exist:

.. code-block:: console

   [INFO] [1630223557.477636052] [tf2_echo]: Waiting for transform turtle3 ->  turtle1:
   Invalid frame ID "turtle3" passed to canTransform argument target_frame - frame does
   not exist

Then what frames do exist? If you like to get a graphical representation of this, type:

.. code-block:: console

   ros2 run tf2_tools view_frames
   evince frames.pdf

And you'll get the following output:

.. image:: turtlesim_frames.png

So obviously the problem is that we are requesting ``"turtle3"``, which does not exist.
To fix this bug, just replace ``"turtle3"`` with ``"turtle2"`` in line 67.

And now stop the running demo (Ctrl-c), build it, and run it again:

.. code-block:: console

   colcon build --symlink-install --packages-select learning_tf2_cpp
   ros2 launch turtle_tf2 start_debug_demo.launch.py

And right away we run into the next problem:

.. code-block:: console

   [turtle_tf2_listener_debug-4] [INFO] [1630223704.617382464] [listener_debug]: Could not
   transform turtle2 to turtle1: Lookup would require extrapolation into the future. Requested
   time 1630223704.617054 but the latest data is at time 1630223704.616726, when looking up
   transform from frame [turtle1] to frame [turtle2]

4 Checking the timestamp
------------------------

Now that we solved the frame name problem, it is time to look at the timestamps. Remember we are trying to get the transform between ``turtle2`` and ``turtle1`` at time ``"now"``. To get statistics on the timing, run:

.. code-block:: console

   ros2 run tf2_ros tf2_monitor turtle2 turtle1

The result should look something like this:

.. code-block:: console

   RESULTS: for turtle2 to turtle1
   Chain is: turtle1
   Net delay     avg = 0.00287347: max = 0.0167241
   
   Frames:
   Frame: turtle1, published by <no authority available>, Average Delay: 0.000295833, Max Delay: 0.000755072
   
   All Broadcasters:
   Node: <no authority available> 125.246 Hz, Average Delay: 0.000290237 Max Delay: 0.000786781

The key part here is the delay for the chain from ``turtle2`` to ``turtle1``. The output shows there is an average delay of about 3 milliseconds. This means that tf2 can only transform between the turtles after 3 milliseconds are passed. So, if we would be asking tf2 for the transformation between the turtles 3 milliseconds ago instead of ``"now"``, tf2 would be able to give us an answer sometimes. Let's test this quickly by changing lines 75-79 to:

.. code-block:: C++

   try {
      transformStamped = tf_buffer_->lookupTransform(
        toFrameRel, fromFrameRel, 
        this->now() - rclcpp::Duration::from_seconds(0.1));
   } catch (tf2::TransformException & ex) {

So in the new code we are asking for the transform between the turtles 100 milliseconds ago (Why not 3? Just to be safe...). Stop the demo (Ctrl-c), build and run:

.. code-block:: console

   colcon build --symlink-install --packages-select learning_tf2_cpp
   ros2 launch turtle_tf2 start_debug_demo.launch.py

And you should finally see the turtle move!

.. image:: tf2_debug.png

That last fix we made is not really what you want to do, it was just to make sure that was our problem. The real fix would look like this:

.. code-block:: C++

   try {
      transformStamped = tf_buffer_->lookupTransform(
        toFrameRel, fromFrameRel, 
        tf2::TimePointZero);
   } catch (tf2::TransformException & ex) {

or like this:

.. code-block:: C++

   try {
      transformStamped = tf_buffer_->lookupTransform(
        toFrameRel, fromFrameRel, 
        tf2::TimePoint());
   } catch (tf2::TransformException & ex) {

And also can like this:

.. code-block:: C++

   try {
      transformStamped = tf_buffer_->lookupTransform(
        toFrameRel, fromFrameRel, 
        this->now(), rclcpp::Duration::from_seconds(0.1));
   } catch (tf2::TransformException & ex) {

5 Summary
---------

In this tutorial you learned how to use a systematic approach for debugging tf2 related problems.
You also learned how to use tf2 debugging tools, such as ``tf2_echo``, ``tf2_monitor`` and ``view_frames`` to help you debug those tf2 problems.
