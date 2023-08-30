.. redirect-from::

   Migration-Guide
   Contributing/Migration-Guide
   The-ROS2-Project/Contributing/Migration-Guide

Migrating C++ Packages
======================

.. contents:: Table of Contents
   :depth: 2
   :local:

Build tool
----------

Instead of using ``catkin_make``, ``catkin_make_isolated`` or ``catkin build`` ROS 2 uses the command line tool `colcon <https://design.ros2.org/articles/build_tool.html>`__ to build and install a set of packages.
See the :doc:`beginner tutorial <../../Tutorials/Beginner-Client-Libraries/Colcon-Tutorial>` to get started with ``colcon``.

Build system
------------

The build system in ROS 2 is called `ament <https://design.ros2.org/articles/ament.html>`__.
Ament is built on CMake: ``ament_cmake`` provides CMake functions to make writing ``CMakeLists.txt`` files easier.

Update the *CMakeLists.txt* to use *ament_cmake*
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Apply the following changes to use ``ament_cmake`` instead of ``catkin``:


*
  Set the build type in the ``package.xml`` file export section:

  .. code-block:: xml

     <export>
       <build_type>ament_cmake</build_type>
     </export>

*
  Replace the ``find_package`` invocation with ``catkin`` and the ``COMPONENTS`` with:

  .. code-block:: cmake

     find_package(ament_cmake REQUIRED)
     find_package(component1 REQUIRED)
     # ...
     find_package(componentN REQUIRED)

*
  Move and update the ``catkin_package`` invocation with:


  *
    Invoke ``ament_package`` instead but **after** all targets have been registered.

  *
    The only valid argument for `ament_package <https://github.com/ament/ament_cmake/blob/{REPOS_FILE_BRANCH}/ament_cmake_core/cmake/core/ament_package.cmake>`__ is ``CONFIG_EXTRAS``.
    All other arguments are covered by separate functions which all need to be invoked *before* ``ament_package``:

    * Instead of passing ``CATKIN_DEPENDS ...`` call ``ament_export_dependencies(...)`` before.
    * Instead of passing ``INCLUDE_DIRS ...`` call ``ament_export_include_directories(...)`` before.
    * Instead of passing ``LIBRARIES ...`` call ``ament_export_libraries(...)`` before.

*
  Replace the invocation of ``add_message_files``, ``add_service_files`` and ``generate_messages`` with `rosidl_generate_interfaces <https://github.com/ros2/rosidl/blob/{REPOS_FILE_BRANCH}/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake>`__.


  *
    The first argument is the ``target_name``.
    If you're building just one library it's ``${PROJECT_NAME}``

  *
    Followed by the list of message filenames, relative to the package root.


    * If you will be using the list of filenames multiple times, it is recommended to compose a list of message files and pass the list to the function for clarity.

  *
    The final multi-value-keyword argument fpr ``generate_messages`` is ``DEPENDENCIES`` which requires the list of dependent message packages.

    .. code-block:: cmake

       rosidl_generate_interfaces(${PROJECT_NAME}
         ${msg_files}
         DEPENDENCIES std_msgs
       )

*
  Remove any occurrences of the *devel space*.
  Related CMake variables like ``CATKIN_DEVEL_PREFIX`` do not exist anymore.


  * The ``CATKIN_DEPENDS`` and ``DEPENDS`` arguments are passed to the new function `ament_export_dependencies <https://github.com/ament/ament_cmake/blob/{REPOS_FILE_BRANCH}/ament_cmake_export_dependencies/cmake/ament_export_dependencies.cmake>`__.
  * ``CATKIN_GLOBAL_BIN_DESTINATION``: ``bin``
  * ``CATKIN_GLOBAL_INCLUDE_DESTINATION``: ``include``
  * ``CATKIN_GLOBAL_LIB_DESTINATION``: ``lib``
  * ``CATKIN_GLOBAL_LIBEXEC_DESTINATION``: ``lib``
  * ``CATKIN_GLOBAL_SHARE_DESTINATION``: ``share``
  * ``CATKIN_PACKAGE_BIN_DESTINATION``: ``lib/${PROJECT_NAME}``
  * ``CATKIN_PACKAGE_INCLUDE_DESTINATION``: ``include/${PROJECT_NAME}``
  * ``CATKIN_PACKAGE_LIB_DESTINATION``: ``lib``
  * ``CATKIN_PACKAGE_SHARE_DESTINATION``: ``share/${PROJECT_NAME}``

Unit tests
^^^^^^^^^^

If you are using gtest:

Replace ``CATKIN_ENABLE_TESTING`` with ``BUILD_TESTING``.
Replace ``catkin_add_gtest`` with ``ament_add_gtest``.

.. code-block:: diff

   -   if (CATKIN_ENABLE_TESTING)
   -     find_package(GTest REQUIRED)  # or rostest
   -     include_directories(${GTEST_INCLUDE_DIRS})
   -     catkin_add_gtest(${PROJECT_NAME}-some-test src/test/some_test.cpp)
   -     target_link_libraries(${PROJECT_NAME}-some-test
   -       ${PROJECT_NAME}_some_dependency
   -       ${catkin_LIBRARIES}
   -       ${GTEST_LIBRARIES})
   -   endif()
   +   if (BUILD_TESTING)
   +     find_package(ament_cmake_gtest REQUIRED)
   +     ament_add_gtest(${PROJECT_NAME}-some-test src/test/test_something.cpp)
   +     ament_target_dependencies(${PROJECT_NAME)-some-test
   +       "rclcpp"
   +       "std_msgs")
   +     target_link_libraries(${PROJECT_NAME}-some-test
   +       ${PROJECT_NAME}_some_dependency)
   +   endif()

Add ``<test_depend>ament_cmake_gtest</test_depend>`` to your ``package.xml``.

.. code-block:: diff

   -   <test_depend>rostest</test_depend>
   +   <test_depend>ament_cmake_gtest</test_depend>

Linters
^^^^^^^

In ROS 2 we are working to maintain clean code using linters.
The styles for different languages are defined in our :doc:`Developer Guide <../../The-ROS2-Project/Contributing/Developer-Guide>`.

If you are starting a project from scratch it is recommended to follow the style guide and turn on the automatic linter unit tests by adding these lines just below ``if(BUILD_TESTING)``:

.. code-block:: cmake

   find_package(ament_lint_auto REQUIRED)
   ament_lint_auto_find_test_dependencies()

You will also need to add the following dependencies to your ``package.xml``:

.. code-block:: xml

   <test_depend>ament_lint_auto</test_depend>
   <test_depend>ament_lint_common</test_depend>

Update source code
------------------

Messages, services, and actions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The namespace of ROS 2 messages, services, and actions use a subnamespace (``msg``, ``srv``, or ``action``, respectively) after the package name.
Therefore an include looks like: ``#include <my_interfaces/msg/my_message.hpp>``.
The C++ type is then named: ``my_interfaces::msg::MyMessage``.

Shared pointer types are provided as typedefs within the message structs: ``my_interfaces::msg::MyMessage::SharedPtr`` as well as ``my_interfaces::msg::MyMessage::ConstSharedPtr``.

For more details please see the article about the `generated C++ interfaces <https://design.ros2.org/articles/generated_interfaces_cpp.html>`__.

The migration requires includes to change by:


* inserting the subfolder ``msg`` between the package name and message datatype
* changing the included filename from CamelCase to underscore separation
* changing from ``*.h`` to ``*.hpp``

.. code-block:: cpp

   // ROS 1 style is in comments, ROS 2 follows, uncommented.
   // # include <geometry_msgs/PointStamped.h>
   #include <geometry_msgs/msg/point_stamped.hpp>

   // geometry_msgs::PointStamped point_stamped;
   geometry_msgs::msg::PointStamped point_stamped;

The migration requires code to insert the ``msg`` namespace into all instances.

Use of service objects
^^^^^^^^^^^^^^^^^^^^^^

Service callbacks in ROS 2 do not have boolean return values.
Instead of returning false on failures, throwing exceptions is recommended.

.. code-block:: cpp

   // ROS 1 style is in comments, ROS 2 follows, uncommented.
   // #include "nav_msgs/GetMap.h"
   #include "nav_msgs/srv/get_map.hpp"

   // bool service_callback(
   //   nav_msgs::GetMap::Request & request,
   //   nav_msgs::GetMap::Response & response)
   void service_callback(
     const std::shared_ptr<nav_msgs::srv::GetMap::Request> request,
     std::shared_ptr<nav_msgs::srv::GetMap::Response> response)
   {
     // ...
     // return true;  // or false for failure
   }

Usages of ros::Time
^^^^^^^^^^^^^^^^^^^

For usages of ``ros::Time``:

* Replace all instances of ``ros::Time`` with ``rclcpp::Time``

* If your messages or code makes use of std_msgs::Time:

  * Convert all instances of std_msgs::Time to builtin_interfaces::msg::Time

  * Convert all ``#include "std_msgs/time.h`` to ``#include "builtin_interfaces/msg/time.hpp"``

  * Convert all instances using the std_msgs::Time field ``nsec`` to the builtin_interfaces::msg::Time field ``nanosec``

Usages of ros::Rate
^^^^^^^^^^^^^^^^^^^

There is an equivalent type ``rclcpp::Rate`` object which is basically a drop in replacement for ``ros::Rate``.


Boost
^^^^^

Much of the functionality previously provided by Boost has been integrated into the C++ standard library.
As such we would like to take advantage of the new core features and avoid the dependency on boost where possible.

Shared Pointers
~~~~~~~~~~~~~~~

To switch shared pointers from boost to standard C++ replace instances of:


* ``#include <boost/shared_ptr.hpp>`` with ``#include <memory>``
* ``boost::shared_ptr`` with ``std::shared_ptr``

There may also be variants such as ``weak_ptr`` which you want to convert as well.

Also it is recommended practice to use ``using`` instead of ``typedef``.
``using`` has the ability to work better in templated logic.
For details `see here <https://stackoverflow.com/questions/10747810/what-is-the-difference-between-typedef-and-using-in-c11>`__

Thread/Mutexes
~~~~~~~~~~~~~~

Another common part of boost used in ROS codebases are mutexes in ``boost::thread``.


* Replace ``boost::mutex::scoped_lock`` with ``std::unique_lock<std::mutex>``
* Replace ``boost::mutex`` with ``std::mutex``
* Replace ``#include <boost/thread/mutex.hpp>`` with ``#include <mutex>``

Unordered Map
~~~~~~~~~~~~~

Replace:


* ``#include <boost/unordered_map.hpp>`` with ``#include <unordered_map>``
* ``boost::unordered_map`` with ``std::unordered_map``

function
~~~~~~~~

Replace:


* ``#include <boost/function.hpp>``  with ``#include <functional>``
* ``boost::function`` with ``std::function``

Example: Converting an existing ROS 1 package to ROS 2
------------------------------------------------------

Let's say that we have simple ROS 1 package called ``talker`` that uses ``roscpp`` in one node, called ``talker``.
This package is in a catkin workspace, located at ``~/ros1_talker``.

The ROS 1 code
^^^^^^^^^^^^^^

Here's the directory layout of our catkin workspace:

.. code-block:: bash

   $ cd ~/ros1_talker
   $ find .
   .
   ./src
   ./src/talker
   ./src/talker/package.xml
   ./src/talker/CMakeLists.txt
   ./src/talker/talker.cpp

Here is the content of those three files:

``src/talker/package.xml``:

.. code-block:: xml

   <package>
     <name>talker</name>
     <version>0.0.0</version>
     <description>talker</description>
     <maintainer email="gerkey@osrfoundation.org">Brian Gerkey</maintainer>
     <license>Apache 2.0</license>
     <buildtool_depend>catkin</buildtool_depend>
     <build_depend>roscpp</build_depend>
     <build_depend>std_msgs</build_depend>
     <run_depend>roscpp</run_depend>
     <run_depend>std_msgs</run_depend>
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

Building the ROS 1 code
~~~~~~~~~~~~~~~~~~~~~~~

We source an environment setup file (in this case for Noetic using bash), then we
build our package using ``catkin_make install``:

.. code-block:: bash

   . /opt/ros/noetic/setup.bash
   cd ~/ros1_talker
   catkin_make install

Running the ROS 1 node
~~~~~~~~~~~~~~~~~~~~~~

If there's not already one running, we start a ``roscore``, first sourcing the
setup file from our ``catkin`` install tree (the system setup file at
``/opt/ros/noetic/setup.bash`` would also work here):

.. code-block:: bash

   . ~/ros1_talker/install/setup.bash
   roscore

In another shell, we run the node from the ``catkin`` install space using
``rosrun``, again sourcing the setup file first (in this case it must be the one
from our workspace):

.. code-block:: bash

   . ~/ros1_talker/install/setup.bash
   rosrun talker talker

Migrating to ROS 2
^^^^^^^^^^^^^^^^^^

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

Publishing the message is the same as before:

.. code-block:: cpp

       chatter_pub->publish(msg);

Spinning (i.e., letting the communications system process any pending
incoming/outgoing messages) is different in that the call now takes the node as
an argument:

.. code-block:: cpp

   //    ros::spinOnce();
       rclcpp::spin_some(node);

Sleeping using the rate object is unchanged.

Putting it all together, the new ``talker.cpp`` looks like this:

.. code-block:: cpp

   #include <sstream>
   // #include "ros/ros.h"
   #include "rclcpp/rclcpp.hpp"
   // #include "std_msgs/String.h"
   #include "std_msgs/msg/string.hpp"
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
       chatter_pub->publish(msg);
   //    ros::spinOnce();
       rclcpp::spin_some(node);
       loop_rate.sleep();
     }
     return 0;
   }

Changing the ``package.xml``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

ROS 2 uses a newer version of ``catkin``, called ``ament_cmake``, which we specify in the
``buildtool_depend`` tag:

.. code-block:: xml

   <!--  <buildtool_depend>catkin</buildtool_depend> -->
     <buildtool_depend>ament_cmake</buildtool_depend>

In our build dependencies, instead of ``roscpp`` we use ``rclcpp``, which provides the C++ API that we use.

.. code-block:: xml

   <!--  <build_depend>roscpp</build_depend> -->
     <build_depend>rclcpp</build_depend>

We make the same addition in the run dependencies and also update from the
``run_depend`` tag to the ``exec_depend`` tag (part of the upgrade to version 2 of the package format):

.. code-block:: xml

   <!--  <run_depend>roscpp</run_depend> -->
     <exec_depend>rclcpp</exec_depend>
   <!--  <run_depend>std_msgs</run_depend> -->
     <exec_depend>std_msgs</exec_depend>

In ROS 1, we use ``<depend>`` to simplify specifying dependencies for both
compile-time and runtime.
We can do the same in ROS 2:

.. code-block:: xml

     <depend>rclcpp</depend>
     <depend>std_msgs</depend>

We also need to tell the build tool what *kind* of package we are, so that it knows how
to build us.
Because we're using ``ament`` and CMake, we add the following lines to declare our
build type to be ``ament_cmake``:

.. code-block:: xml

     <export>
       <build_type>ament_cmake</build_type>
     </export>

Putting it all together, our ``package.xml`` now looks like this:

.. code-block:: xml

   <!-- <package> -->
   <package format="2">
     <name>talker</name>
     <version>0.0.0</version>
     <description>talker</description>
     <maintainer email="gerkey@osrfoundation.org">Brian Gerkey</maintainer>
     <license>Apache License 2.0</license>
   <!--  <buildtool_depend>catkin</buildtool_depend> -->
     <buildtool_depend>ament_cmake</buildtool_depend>
   <!--  <build_depend>roscpp</build_depend> -->
   <!--  <run_depend>roscpp</run_depend> -->
   <!--  <run_depend>std_msgs</run_depend> -->
     <depend>rclcpp</depend>
     <depend>std_msgs</depend>
     <export>
       <build_type>ament_cmake</build_type>
     </export>
   </package>


Changing the CMake code
~~~~~~~~~~~~~~~~~~~~~~~

ROS 2 relies on a higher version of CMake:

.. code-block:: bash

   #cmake_minimum_required(VERSION 2.8.3)
   cmake_minimum_required(VERSION 3.5)

ROS 2 relies on the C++17 standard.
Depending on what compiler you're using, support for C++17 might not be enabled by default.
Enable C++17 support explicitly by adding this line near the top of the file:

.. code-block:: cmake

   set(CMAKE_CXX_STANDARD 17)

The preferred way to work on all platforms is this:

.. code-block:: cmake

   if(NOT CMAKE_CXX_STANDARD)
     set(CMAKE_CXX_STANDARD 17)
   endif()
   if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
     add_compile_options(-Wall -Wextra -Wpedantic)
   endif()

Using ``catkin``, we specify the packages we want to build against by passing them
as ``COMPONENTS`` arguments when initially finding ``catkin`` itself.
With ``ament_cmake``, we find each package individually, starting with ``ament_cmake``:

.. code-block:: cmake

   #find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
   find_package(ament_cmake REQUIRED)
   find_package(rclcpp REQUIRED)
   find_package(std_msgs REQUIRED)

System dependencies can be found as before:

.. code-block:: cmake

   find_package(Boost REQUIRED COMPONENTS system filesystem thread)

We call ``catkin_package()`` to auto-generate things like CMake configuration
files for other packages that use our package.
Whereas that call happens *before* specifying targets to build, we now call the
analogous ``ament_package()`` *after* the targets:

.. code-block:: cmake

   # catkin_package()
   # At the bottom of the file:
   ament_package()

The only directories that need to be manually included are local directories
and dependencies that are not ament packages:

.. code-block:: cmake

   #include_directories(${catkin_INCLUDE_DIRS})
   include_directories(include ${Boost_INCLUDE_DIRS})

A better alternative is to specify include directories for each target
individually, rather than including all the directories for all targets:

.. code-block:: cmake

   target_include_directories(target PUBLIC include ${Boost_INCLUDE_DIRS})

Similar to how we found each dependent package separately, we need to link
each one to the build target.
To link with dependent packages that are ament packages, instead of using
``target_link_libraries()``, ``ament_target_dependencies()`` is a more
concise and more thorough way of handling build flags.
It automatically handles both the include directories defined in
``_INCLUDE_DIRS`` and linking libraries defined in ``_LIBRARIES``.

.. code-block:: cmake

   #target_link_libraries(talker ${catkin_LIBRARIES})
   ament_target_dependencies(talker
     rclcpp
     std_msgs)

To link with packages that are not ament packages, such as system dependencies
like ``Boost``, or a library being built in the same ``CMakeLists.txt``, use
``target_link_libraries()``:

.. code-block:: cmake

   target_link_libraries(target ${Boost_LIBRARIES})

For installation, ``catkin`` defines variables like ``CATKIN_PACKAGE_BIN_DESTINATION``.
With ``ament_cmake``, we just give a path relative to the installation root:

.. code-block:: cmake

   #install(TARGETS talker
   #  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
   install(TARGETS talker
     DESTINATION lib/${PROJECT_NAME})

Optionally, we can install and export the included directories for downstream packages:

.. code-block:: cmake

   install(DIRECTORY include/
     DESTINATION include)
   ament_export_include_directories(include)

Optionally, we can export dependencies for downstream packages:

.. code-block:: cmake

   ament_export_dependencies(std_msgs)

Putting it all together, the new ``CMakeLists.txt`` looks like this:

.. code-block:: cmake

   #cmake_minimum_required(VERSION 2.8.3)
   cmake_minimum_required(VERSION 3.5)
   project(talker)
   if(NOT CMAKE_CXX_STANDARD)
     set(CMAKE_CXX_STANDARD 17)
   endif()
   if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
     add_compile_options(-Wall -Wextra -Wpedantic)
   endif()
   #find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
   find_package(ament_cmake REQUIRED)
   find_package(rclcpp REQUIRED)
   find_package(std_msgs REQUIRED)
   #catkin_package()
   #include_directories(${catkin_INCLUDE_DIRS})
   include_directories(include)
   add_executable(talker talker.cpp)
   #target_link_libraries(talker ${catkin_LIBRARIES})
   ament_target_dependencies(talker
     rclcpp
     std_msgs)
   #install(TARGETS talker
   #  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
   install(TARGETS talker
     DESTINATION lib/${PROJECT_NAME})
   install(DIRECTORY include/
     DESTINATION include)
   ament_export_include_directories(include)
   ament_export_dependencies(std_msgs)
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
