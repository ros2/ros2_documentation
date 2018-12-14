
ROS 2 Crystal Clemmys (codename 'crystal'; December 2018)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This is a placeholder page for the upcoming ROS 2 release *Crystal Clemmys* !

Supported Platforms
^^^^^^^^^^^^^^^^^^^

The platforms supported for this release are still pending, see: https://github.com/ros-infrastructure/rep/pull/177

Features
^^^^^^^^

New features in this ROS 2 release
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

TBD

Changes since the Bouncy release
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Changes since the `Bouncy Bolson <Release-Bouncy-Bolson>` release:

* geometry2 - ``tf2_ros::Buffer`` API Change - 
   ``tf2_ros::Buffer`` now uses ``rclcpp::Time``, with the constructor requiring a ``shared_ptr`` to a ``rclcpp::Clock`` instance.
   See https://github.com/ros2/geometry2/pull/67 for details, with example usage::
   
    #include <tf2_ros/transform_listener.h>
    #include <rclcpp/rclcpp.hpp>
    ...
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    tf2_ros::Buffer buffer(clock);
    tf2_ros::TransformListener tf_listener(buffer);



Known Issues
^^^^^^^^^^^^

* first item goes here
