Migrating a C++ Package Example
===============================

.. contents:: Table of Contents
   :depth: 2
   :local:

This example shows how to migrate an example C++ package from ROS 1 to ROS 2.

Prerequisites
-------------

You need a working ROS 2 installation, such as :doc:`ROS {DISTRO} <../../Installation>`.

The ROS 1 code
--------------

Say you have a ROS 1 package called ``talker`` that uses ``roscpp`` in one node, called ``talker``.
This package is in a catkin workspace, located at ``~/ros1_talker``.

Your ROS 1 workspace has the following directory layout:

.. code-block:: bash

   $ cd ~/ros1_talker
   $ find .
   .
   ./src
   ./src/talker
   ./src/talker/package.xml
   ./src/talker/CMakeLists.txt
   ./src/talker/talker.cpp

The files have the following content:

``src/talker/package.xml``:

.. code-block:: xml

   <?xml version="1.0"?>
   <?xml-model href="http://download.ros.org/schema/package_format2.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
   <package format="2">
     <name>talker</name>
     <version>0.0.0</version>
     <description>talker</description>
     <maintainer email="gerkey@example.com">Brian Gerkey</maintainer>
     <license>Apache-2.0</license>
     <buildtool_depend>catkin</buildtool_depend>
     <depend>roscpp</depend>
     <depend>std_msgs</depend>
   </package>

``src/talker/CMakeLists.txt``:

.. code-block:: cmake

   cmake_minimum_required(VERSION 2.8.3)
   project(talker)
   find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
   catkin_package()
   include_directories(${catkin_INCLUDE_DIRS})
   add_executable(talker talker.cpp)
   target_link_libraries(talker ${catkin_LIBRARIES})
   install(TARGETS talker
     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

``src/talker/talker.cpp``:

.. code-block:: cpp

   #include <sstream>
   #include "ros/ros.h"
   #include "std_msgs/String.h"
   int main(int argc, char **argv)
   {
     ros::init(argc, argv, "talker");
     ros::NodeHandle n;
     ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
     ros::Rate loop_rate(10);
     int count = 0;
     std_msgs::String msg;
     while (ros::ok())
     {
       std::stringstream ss;
       ss << "hello world " << count++;
       msg.data = ss.str();
       ROS_INFO("%s", msg.data.c_str());
       chatter_pub.publish(msg);
       ros::spinOnce();
       loop_rate.sleep();
     }
     return 0;
   }

Migrating to ROS 2
------------------

Let's start by creating a new workspace in which to work:

.. code-block:: bash

   mkdir ~/ros2_talker
   cd ~/ros2_talker

We'll copy the source tree from our ROS 1 package into that workspace, where we can modify it:

.. code-block:: bash

   mkdir src
   cp -a ~/ros1_talker/src/talker src

Now we'll modify the C++ code in the node.
The ROS 2 C++ library, called ``rclcpp``, provides a different API from that
provided by ``roscpp``.
The concepts are very similar between the two libraries, which makes the changes
reasonably straightforward to make.

Included headers
~~~~~~~~~~~~~~~~

In place of ``ros/ros.h``, which gave us access to the ``roscpp`` library API, we
need to include ``rclcpp/rclcpp.hpp``, which gives us access to the ``rclcpp``
library API:

.. code-block:: cpp

   //#include "ros/ros.h"
   #include "rclcpp/rclcpp.hpp"

To get the ``std_msgs/String`` message definition, in place of
``std_msgs/String.h``, we need to include ``std_msgs/msg/string.hpp``:

.. code-block:: cpp

   //#include "std_msgs/String.h"
   #include "std_msgs/msg/string.hpp"

Changing C++ library calls
~~~~~~~~~~~~~~~~~~~~~~~~~~

Instead of passing the node's name to the library initialization call, we do
the initialization, then pass the node name to the creation of the node object:

.. code-block:: cpp

   //  ros::init(argc, argv, "talker");
   //  ros::NodeHandle n;
       rclcpp::init(argc, argv);
       auto node = rclcpp::Node::make_shared("talker");

The creation of the publisher and rate objects looks pretty similar, with some
changes to the names of namespace and methods.

.. code-block:: cpp

   //  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
   //  ros::Rate loop_rate(10);
     auto chatter_pub = node->create_publisher<std_msgs::msg::String>("chatter",
       1000);
     rclcpp::Rate loop_rate(10);

To further control how message delivery is handled, a quality of service
(``QoS``) profile could be passed in.
The default profile is ``rmw_qos_profile_default``.
For more details, see the
`design document <https://design.ros2.org/articles/qos.html>`__
and :doc:`concept overview <../../Concepts/Intermediate/About-Quality-of-Service-Settings>`.

The creation of the outgoing message is different in the namespace:

.. code-block:: cpp

   //  std_msgs::String msg;
     std_msgs::msg::String msg;

In place of ``ros::ok()``, we call ``rclcpp::ok()``:

.. code-block:: cpp

   //  while (ros::ok())
     while (rclcpp::ok())

Inside the publishing loop, we access the ``data`` field as before:

.. code-block:: cpp

       msg.data = ss.str();

To print a console message, instead of using ``ROS_INFO()``, we use
``RCLCPP_INFO()`` and its various cousins.
The key difference is that ``RCLCPP_INFO()`` takes a Logger object as the first
argument.

.. code-block:: cpp

   //    ROS_INFO("%s", msg.data.c_str());
       RCLCPP_INFO(node->get_logger(), "%s\n", msg.data.c_str());

Change the publish call to use the ``->`` operator instead of ``.``.

.. code-block:: cpp

   //    chatter_pub.publish(msg);
       chatter_pub->publish(msg);

Spinning (i.e., letting the communications system process any pending
incoming/outgoing messages until no more work is available) is different
in that the call now takes the node and timeout as arguments:

.. code-block:: cpp

   //    ros::spinOnce();
       rclcpp::spin_all(node, 0s);

Sleeping using the rate object is unchanged.

Putting it all together, the new ``talker.cpp`` looks like this:

.. code-block:: cpp

   #include <chrono>
   #include <sstream>
   // #include "ros/ros.h"
   #include "rclcpp/rclcpp.hpp"
   // #include "std_msgs/String.h"
   #include "std_msgs/msg/string.hpp"

   using namespace std::chrono_literals;

   int main(int argc, char **argv)
   {
   //  ros::init(argc, argv, "talker");
   //  ros::NodeHandle n;
     rclcpp::init(argc, argv);
     auto node = rclcpp::Node::make_shared("talker");
   //  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
   //  ros::Rate loop_rate(10);
     auto chatter_pub = node->create_publisher<std_msgs::msg::String>("chatter", 1000);
     rclcpp::Rate loop_rate(10);
     int count = 0;
   //  std_msgs::String msg;
     std_msgs::msg::String msg;
   //  while (ros::ok())
     while (rclcpp::ok())
     {
       std::stringstream ss;
       ss << "hello world " << count++;
       msg.data = ss.str();
   //    ROS_INFO("%s", msg.data.c_str());
       RCLCPP_INFO(node->get_logger(), "%s\n", msg.data.c_str());
   //    chatter_pub.publish(msg);
       chatter_pub->publish(msg);
   //    ros::spinOnce();
       rclcpp::spin_all(node, 0s);
       loop_rate.sleep();
     }
     return 0;
   }

Change the ``package.xml``
~~~~~~~~~~~~~~~~~~~~~~~~~~

ROS 2 packages use CMake functions and macros from ``ament_cmake_ros`` instead of ``catkin``.
Delete the dependency on ``catkin``:

.. code-block::

   <!-- delete this -->
   <buildtool_depend>catkin</buildtool_depend>`

Add a new dependency on ``ament_cmake_ros``:

.. code-block:: xml

     <buildtool_depend>ament_cmake_ros</buildtool_depend>

ROS 2 C++ libraries use `rclcpp <https://index.ros.org/p/roscpp/#noetic>`__ instead of `roscpp <https://index.ros.org/p/roscpp/#noetic>`__.

Delete the dependency on ``roscpp``:

.. code-block::

   <!-- delete this -->
   <depend>roscpp</depend>

Add a dependency on ``rclcpp``:

.. code-block:: xml

     <depend>rclcpp</depend>


Add an ``<export>`` section to tell colcon the package is an ``ament_cmake`` package instead of a ``catkin`` package.

.. code-block:: xml

     <export>
       <build_type>ament_cmake</build_type>
     </export>

Your ``package.xml`` now looks like this:

.. code-block:: xml

   <?xml version="1.0"?>
   <?xml-model href="http://download.ros.org/schema/package_format2.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
   <package format="2">
     <name>talker</name>
     <version>0.0.0</version>
     <description>talker</description>
     <maintainer email="gerkey@example.com">Brian Gerkey</maintainer>
     <license>Apache-2.0</license>
     <buildtool_depend>ament_cmake</buildtool_depend>
     <depend>rclcpp</depend>
     <depend>std_msgs</depend>
     <export>
       <build_type>ament_cmake</build_type>
     </export>
   </package>


Changing the CMake code
~~~~~~~~~~~~~~~~~~~~~~~

Require a newer version of CMake so that ``ament_cmake`` functions work correctly.

.. code-block::

   cmake_minimum_required(VERSION 3.14.4)

Use a newer C++ standard matching the version used by your target ROS distro in `REP 2000 <https://www.ros.org/reps/rep-2000.html>`__.
If you are using C++17, then set that version with the following snippet after the ``project(talker)`` call.
Add extra compiler checks too because it is a good practice.

.. code-block:: cmake

   if(NOT CMAKE_CXX_STANDARD)
     set(CMAKE_CXX_STANDARD 17)
   endif()
   if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
     add_compile_options(-Wall -Wextra -Wpedantic)
   endif()

Replace the ``find_package(catkin ...)`` call with individual calls for each dependency.

.. code-block:: cmake

   find_package(ament_cmake REQUIRED)
   find_package(rclcpp REQUIRED)
   find_package(std_msgs REQUIRED)

Delete the call to ``catkin_package()``.
Add a call to ``ament_package()`` at the bottom of the ``CMakeLists.txt``.

.. code-block:: cmake

   ament_package()

Make the ``target_link_libraries`` call modern CMake targets provided by ``rclcpp`` and ``std_msgs``.

.. code-block:: cmake

   target_link_libraries(talker PUBLIC
     rclcpp::rclcpp
     ${std_msgs_TARGETS})

Delete the call to ``include_directories()``.
Add a call to ``target_include_directories()`` below ``add_executable(talker talker.cpp)``.
Don't pass variables like ``rclcpp_INCLUDE_DIRS`` into ``target_include_directories()``.
The include directories are already handled by calling ``target_link_libraries()`` with modern CMake targets.

.. code-block:: cmake

   target_include_directories(talker PUBLIC
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

Change the call to ``install()`` so that the ``talker`` executable is installed into a project specific directory.

.. code-block:: cmake

   install(TARGETS talker
     DESTINATION lib/${PROJECT_NAME})

The new ``CMakeLists.txt`` looks like this:

.. code-block:: cmake

   cmake_minimum_required(VERSION 3.14.4)
   project(talker)
   if(NOT CMAKE_CXX_STANDARD)
     set(CMAKE_CXX_STANDARD 17)
   endif()
   if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
     add_compile_options(-Wall -Wextra -Wpedantic)
   endif()
   find_package(ament_cmake REQUIRED)
   find_package(rclcpp REQUIRED)
   find_package(std_msgs REQUIRED)
   add_executable(talker talker.cpp)
   target_include_directories(talker PUBLIC
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
   target_link_libraries(talker PUBLIC
     rclcpp::rclcpp
     ${std_msgs_TARGETS})
   install(TARGETS talker
     DESTINATION lib/${PROJECT_NAME})
   ament_package()

Building the ROS 2 code
~~~~~~~~~~~~~~~~~~~~~~~

We source an environment setup file (in this case the one generated by following
the ROS 2 installation tutorial, which builds in ``~/ros2_ws``, then we build our
package using ``colcon build``:

.. code-block:: bash

   . ~/ros2_ws/install/setup.bash
   cd ~/ros2_talker
   colcon build

Running the ROS 2 node
~~~~~~~~~~~~~~~~~~~~~~

Because we installed the ``talker`` executable into the correct directory, after sourcing the
setup file, from our install tree, we can invoke it by running:

.. code-block:: bash

   . ~/ros2_ws/install/setup.bash
   ros2 run talker talker

Conclusion
----------

You have learned how to migrate an example C++ ROS 1 package to ROS 2.
Use the :doc:`Migrating C++ Packages reference page <./Migrating-CPP-Packages>` to help you migrate your own C++ packages from ROS 1 to ROS 2.
