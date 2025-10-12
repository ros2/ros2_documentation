.. Redirect-from::

    Tutorials/Intermediate/URDF/Using-URDF-with-Robot-State-Publisher

.. _URDFPlusRSPCPP:

Using URDF with ``robot_state_publisher`` (C++)
===============================================

**Goal:** Simulate a walking robot modeled in URDF and view it in Rviz.

**Tutorial level:** Intermediate

**Time:** 15 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

This tutorial will show you how to model a walking robot, publish the state as a tf2 message and view the simulation in Rviz.
First, we create the URDF model describing the robot assembly.
Next we write a node which simulates the motion and publishes the JointState and transforms.
We then use ``robot_state_publisher`` to publish the entire robot state to ``/tf2``.

.. image:: images/r2d2_rviz_demo.gif

Prerequisites
-------------

- `rviz2 <https://index.ros.org/p/rviz2/>`__

As always, don't forget to source ROS 2 in :doc:`every new terminal you open <../../Beginner-CLI-Tools/Configuring-ROS2-Environment>`.

Tasks
-----

1 Create a package
^^^^^^^^^^^^^^^^^^

Go to your ROS 2 workplace and create a package names ``urdf_tutorial_cpp``:

.. code-block:: console

    $ cd src
    $ ros2 pkg create --build-type ament_cmake --license Apache-2.0 urdf_tutorial_cpp --dependencies rclcpp
    $ cd urdf_tutorial_cpp

You should now see a ``urdf_tutorial_cpp`` folder.
Next you will make several changes to it.

2 Create the URDF File
^^^^^^^^^^^^^^^^^^^^^^

Create the directory where we will store some assets:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      $ mkdir -p urdf

  .. group-tab:: macOS

    .. code-block:: console

      $ mkdir -p urdf

  .. group-tab:: Windows

    .. code-block:: console

      $ md urdf

Download the :download:`URDF file <documents/r2d2.urdf.xml>` and save it as ``urdf_tutorial_cpp/urdf/r2d2.urdf.xml``.
Download the :download:`Rviz configuration file <documents/r2d2.rviz>` and save it as ``urdf_tutorial_cpp/urdf/r2d2.rviz``.

3 Publish the state
^^^^^^^^^^^^^^^^^^^

Now we need a method for specifying what state the robot is in.

To do this, we must specify all three joints and the overall robot geometry.

Fire up your favorite editor and paste the following code into

``urdf_tutorial_cpp/src/urdf_tutorial.cpp``

.. code-block:: cpp

  #include <rclcpp/rclcpp.hpp>
  #include <geometry_msgs/msg/quaternion.hpp>
  #include <sensor_msgs/msg/joint_state.hpp>
  #include <tf2_ros/transform_broadcaster.h>
  #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
  #include <cmath>
  #include <thread>
  #include <chrono>

  using namespace std::chrono;

  class StatePublisher : public rclcpp::Node{
      public:

      StatePublisher(rclcpp::NodeOptions options=rclcpp::NodeOptions()):
          Node("state_publisher",options){
              joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",10);
              // create a publisher to tell robot_state_publisher the JointState information.
              // robot_state_publisher will deal with this transformation
              broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
              // create a broadcaster to tell the tf2 state information
              // this broadcaster will determine the position of coordinate system 'axis' in coordinate system 'odom'
              RCLCPP_INFO(this->get_logger(),"Starting state publisher");

              loop_rate_=std::make_shared<rclcpp::Rate>(33ms);

              timer_=this->create_wall_timer(33ms,std::bind(&StatePublisher::publish,this));
          }

          void publish();
      private:
      rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
      std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster;
      rclcpp::Rate::SharedPtr loop_rate_;
      rclcpp::TimerBase::SharedPtr timer_;

      //Robot state variables
      // degree means one degree
      const double degree=M_PI/180.0;
      double tilt = 0.;
      double tinc = degree;
      double swivel = 0.;
      double angle = 0.;
      double height = 0.;
      double hinc = 0.005;
  };

  void StatePublisher::publish(){
      // create the necessary messages
      geometry_msgs::msg::TransformStamped t;
      sensor_msgs::msg::JointState joint_state;

      // add time stamp
      joint_state.header.stamp=this->get_clock()->now();
      // Specify joints' name which are defined in the r2d2.urdf.xml and their content
      joint_state.name={"swivel","tilt","periscope"};
      joint_state.position={swivel,tilt,height};

      // add time stamp
      t.header.stamp=this->get_clock()->now();
      // specify the father and child frame

      // odom is the base coordinate system of tf2
      t.header.frame_id="odom";
      // axis is defined in r2d2.urdf.xml file and it is the base coordinate of model
      t.child_frame_id="axis";

      // add translation change
      t.transform.translation.x=cos(angle)*2;
      t.transform.translation.y=sin(angle)*2;
      t.transform.translation.z=0.7;
      tf2::Quaternion q;
      // euler angle into Quanternion and add rotation change
      q.setRPY(0,0,angle+M_PI/2);
      t.transform.rotation.x=q.x();
      t.transform.rotation.y=q.y();
      t.transform.rotation.z=q.z();
      t.transform.rotation.w=q.w();

      // update state for next time
      tilt+=tinc;
      if (tilt<-0.5 || tilt>0.0){
          tinc*=-1;
      }
      height+=hinc;
      if (height>0.2 || height<0.0){
          hinc*=-1;
      }
      swivel+=degree;  // Increment by 1 degree (in radians)
      angle+=degree;    // Change angle at a slower pace

      // send message
      broadcaster->sendTransform(t);
      joint_pub_->publish(joint_state);

      RCLCPP_INFO(this->get_logger(),"Publishing joint state");
  }

  int main(int argc, char * argv[]){
      rclcpp::init(argc,argv);
      rclcpp::spin(std::make_shared<StatePublisher>());
      rclcpp::shutdown();
      return 0;
  }

This file will send ``joint_state`` values  to ``robot_state_publisher`` which in turn will tell tf2 how to place model.

The code file will also tell ``tf2`` how to place the whole model using the  ``transform_broadcaster``

4 Create a launch file
^^^^^^^^^^^^^^^^^^^^^^

Create a new ``urdf_tutorial_cpp/launch`` folder.
Open your editor and paste the following code, saving it as ``urdf_tutorial_cpp/launch/launch.py``

.. literalinclude:: launch/launch.py
  :language: python


5 Edit the CMakeLists.txt file
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You must tell the **colcon** build tool how to install your cpp package.
Edit the ``CMakeLists.txt`` file as follows:

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.8)
  project(urdf_tutorial_cpp)

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  # find dependencies
  find_package(ament_cmake REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  find_package(rclcpp REQUIRED)

  add_executable(urdf_tutorial_cpp src/urdf_tutorial.cpp)

  target_link_libraries(urdf_tutorial_cpp PUBLIC
    ${geometry_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    tf2_ros::tf2_ros
    ${tf2_geometry_msgs_TARGETS}
    rclcpp::rclcpp
  )

  install(TARGETS
    urdf_tutorial_cpp
    DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
  )

  install(DIRECTORY
    urdf
    DESTINATION share/${PROJECT_NAME}
  )

  ament_package()

we use ``install`` command to put the ``r2d2.rviz`` into ``install`` dir

6 Install the package
^^^^^^^^^^^^^^^^^^^^^

To visualize the results you will need to open a new terminal and run RViz using your RViz configuration file.

.. code-block:: console

    $ colcon build --symlink-install --packages-select urdf_tutorial_cpp

Source the setup files:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      $ source install/setup.bash

  .. group-tab:: macOS

    .. code-block:: console

      $ source install/setup.bash

  .. group-tab:: Windows

    .. code-block:: console

      $ call install/setup.bat


7 View the results
^^^^^^^^^^^^^^^^^^

To launch your new package run the following command:

.. code-block:: console

  $ ros2 launch urdf_tutorial_cpp launch.py

To visualize your results you will need to open a new terminal and run Rviz using your rviz configuration file.

.. code-block:: console

  $ rviz2 -d install/urdf_tutorial_cpp/share/urdf_tutorial_cpp/urdf/r2d2.rviz

See the `User Guide <http://wiki.ros.org/rviz/UserGuide>`__ for details on how to use Rviz.

``install/urdf_tutorial_cpp/share/urdf_tutorial_cpp/urdf/r2d2.rviz`` is the dir where the ``r2d2.rviz`` stored.

Summary
-------

Congratulations!
You have created a ``JointState`` publisher node and coupled it with ``robot_state_publisher`` to simulate a walking robot.
