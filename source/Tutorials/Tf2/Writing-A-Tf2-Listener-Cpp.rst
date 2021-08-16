.. _WritingATf2ListenerCpp:

Writing a tf2 listener (C++)
============================

**Goal:** Learn how to use tf2 to get access to frame transformations.

**Tutorial level:** Intermediate

**Time:** 10 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

In the previous tutorials we created a tf2 broadcaster to publish the pose of a turtle to tf2. In this tutorial we'll create a tf2 listener to start using tf2.

Prerequisites
-------------

This tutorial assumes you have completed the writing a :ref:`tf2 broadcaster tutorial (C++) <WritingATf2BroadcasterCpp>`.
In previous tutorial, we created a ``learning_tf2_cpp`` package, which is where we will continue working from.

Tasks
-----

1 Write the listener node
^^^^^^^^^^^^^^^^^^^^^^^^^

Let's first create the source files. Go to the ``learning_tf2_cpp`` package we created in the previous tutorial.
Inside the ``src`` directory download the example listener code by entering the following command:

.. tabs::

    .. group-tab:: Linux

        .. code-block:: console

            wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_cpp/src/turtle_tf2_listener.cpp

    .. group-tab:: macOS

        .. code-block:: console

            wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_cpp/src/turtle_tf2_listener.cpp

    .. group-tab:: Windows

        In a Windows command line prompt:

        .. code-block:: console

                curl -sk https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_cpp/src/turtle_tf2_listener.cpp -o turtle_tf2_listener.cpp

        Or in powershell:

        .. code-block:: console

                curl https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_cpp/src/turtle_tf2_listener.cpp -o turtle_tf2_listener.cpp

Open the file using your preferred text editor.

.. code-block:: C++

   #include <geometry_msgs/msg/transform_stamped.hpp>
   #include <geometry_msgs/msg/twist.hpp>

   #include <rclcpp/rclcpp.hpp>
   #include <tf2/exceptions.h>
   #include <tf2_ros/transform_listener.h>
   #include <tf2_ros/buffer.h>
   #include <turtlesim/srv/spawn.hpp>

   #include <chrono>
   #include <memory>
   #include <string>

   using std::placeholders::_1;
   using namespace std::chrono_literals;

   class FrameListener : public rclcpp::Node
   {
   public:
     FrameListener()
     : Node("turtle_tf2_frame_listener")
     {
       // Declare and acquire `target_frame` parameter
       this->declare_parameter<std::string>("target_frame", "turtle1");
       this->get_parameter("target_frame", target_frame_);

       tf_buffer_ =
         std::make_unique<tf2_ros::Buffer>(this->get_clock());
       transform_listener_ =
         std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

       // Create a client to spawn a turtle
       rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawner =
         this->create_client<turtlesim::srv::Spawn>("spawn");

       // Check if the service is available
       while (!spawner->wait_for_service(1s)) {
         if (!rclcpp::ok()) {
           RCLCPP_ERROR(
             this->get_logger(),
             "Interrupted while waiting for the service. Exiting."
           );
           continue;
         }
         RCLCPP_INFO(
           this->get_logger(),
           "Service not available, waiting again..."
         );
       }

       // Initialize request with turtle name and coordinates
       // Note that x, y and theta are defined as floats in turtlesim/srv/Spawn
       auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
       request->x = 4.0;
       request->y = 2.0;
       request->theta = 0.0;
       request->name = "turtle2";
       // Call request
       auto result = spawner->async_send_request(request);

       // Create turtle2 velocity publisher
       publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 1);

       // Call on_timer function every second
       timer_ = this->create_wall_timer(
         1s, std::bind(&FrameListener::on_timer, this));
     }

   private:
     void on_timer()
     {
       // Store frame names in variables that will be used to
       // compute transformations
       std::string fromFrameRel = target_frame_.c_str();
       std::string toFrameRel = "turtle2";

       geometry_msgs::msg::TransformStamped transformStamped;

       // Look up for the transformation between target_frame and turtle2 frames
       // and send velocity commands for turtle2 to reach target_frame
       try {
         transformStamped = tf_buffer_->lookupTransform(
           toFrameRel, fromFrameRel,
           tf2::TimePoint(),
           500ms);
       } catch (tf2::LookupException & ex) {
         RCLCPP_INFO(this->get_logger(), "transform not ready");
         return;
       }

       geometry_msgs::msg::Twist msg;

       static const double scaleRotationRate = 1.0;
       msg.angular.z = scaleRotationRate * atan2(
         transformStamped.transform.translation.y,
         transformStamped.transform.translation.x);

       static const double scaleForwardSpeed = 0.5;
       msg.linear.x = scaleForwardSpeed * sqrt(
         pow(transformStamped.transform.translation.x, 2) +
         pow(transformStamped.transform.translation.y, 2));

       publisher_->publish(msg);
     }
     rclcpp::TimerBase::SharedPtr timer_;
     std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
     std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
     std::string target_frame_;
   };


   int main(int argc, char * argv[])
   {
     rclcpp::init(argc, argv);
     rclcpp::spin(std::make_shared<FrameListener>());
     rclcpp::shutdown();
     return 0;
   }

1.1 Examine the code
~~~~~~~~~~~~~~~~~~~~

Now, let's take a look at the code that is relevant to get access to frame transformations.
The ``tf2_ros`` contains a ``TransformListener`` header file implementation that makes the task of receiving transforms easier.

.. code-block:: C++

    #include <tf2_ros/transform_listener.h>

Here, we create a ``TransformListener`` object. Once the listener is created, it starts receiving tf2 transformations over the wire, and buffers them for up to 10 seconds.

.. code-block:: C++

    transform_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

Finally, we query the listener for a specific transformation. We call ``lookup_transform`` method with following arguments:

#. Target frame

#. Source frame

#. The time at which we want to transform

Providing ``tf2::TimePoint()`` will just get us the latest available transform.
All this is wrapped in a try-except block to catch possible exceptions.

.. code-block:: C++

    transformStamped = tf_buffer_->lookupTransform(
        toFrameRel, fromFrameRel,
        tf2::TimePoint(),
        500ms);

2 Build and run
^^^^^^^^^^^^^^^

With your text editor, open the launch file called ``turtle_tf2_demo.launch.py``, and add the following lines after your first ``turtle1`` broadcaster node:

.. code-block:: python

    from launch import LaunchDescription
    from launch_ros.actions import Node

    def generate_launch_description():
        return LaunchDescription([
            ...,
            DeclareLaunchArgument(
                'target_frame', default_value='turtle1',
                description='Target frame name.'
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
                executable='turtle_tf2_listener',
                name='listener',
                parameters=[
                    {'target_frame': LaunchConfiguration('target_frame')}
                ]
            ),
        ])

This will declare a ``target_frame`` launch argument, start a broadcaster for second turtle that we will spawn and listener that will subscribe to those transformations.
Now you're ready to start your full turtle demo:

.. code-block:: console

    ros2 launch learning_tf2_cpp turtle_tf2_demo.launch.py

You should see the turtle sim with two turtles.
In the second terminal window type the following command:

.. code-block:: console

    ros2 run turtlesim turtle_teleop_key

3 Checking the results
^^^^^^^^^^^^^^^^^^^^^^

To see if things work, simply drive around the first turtle using the arrow keys (make sure your terminal window is active, not your simulator window), and you'll see the second turtle following the first one!

Summary
-------

In this tutorial you learned how to use tf2 to get access to frame transformations.
You also have finished writing your own turtlesim demo that you have tried in the  :ref:`Introduction to tf2 <IntroToTf2>` tutorial.
