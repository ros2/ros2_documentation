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

Update your ``CMakeLists.txt`` to use *ament_cmake*
---------------------------------------------------

ROS 2 C++ packages use `CMake <https://cmake.org/>`__ with convenience functions provided by `ament_cmake <https://index.ros.org/p/ament_cmake/>`__.
Apply the following changes to use ``ament_cmake`` instead of ``catkin``.


Require a newer version of CMake
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

ROS 2 relies on newer versions of CMake than used by ROS 1.
Find the minimum version of CMake used by the ROS distribution you want to support in `REP 2000 <https://www.ros.org/reps/rep-2000.html>`__, and use that version at the top of your ``CMakeLists.txt``.
For example, `3.14.4 is the minimum recommended support for ROS Humble <https://www.ros.org/reps/rep-2000.html#humble-hawksbill-may-2022-may-2027>`__.

.. code-block::

   cmake_minimum_required(VERSION 3.14.4)

Set the build type to ament_cmake
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Remove any dependencies on ``catkin`` from your ``package.xml``

.. code-block::

   # Remove this!
   <buildtool_depend>catkin</buildtool_depend>

Add a new dependency on ``ament_cmake_ros`` (`example <https://github.com/ros2/geometry2/blob/d85102217f692746abea8546c8e41f0abc95c8b8/tf2/package.xml#L25>`__):

.. code-block:: xml

   <buildtool_depend>ament_cmake_ros</buildtool_depend>

Add an ``<export>`` section to your ``package.xml`` if it does not have one already.
Set the ``<build_type>`` to ``ament_cmake`` (`example <https://github.com/ros2/geometry2/blob/d85102217f692746abea8546c8e41f0abc95c8b8/tf2/package.xml#L43-L45>`__)

.. code-block:: xml

   <export>
      <build_type>ament_cmake</build_type>
   </export>

Add a call to ``ament_package()``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Insert a call to ``ament_package()`` at the bottom of your ``CMakeLists.txt`` (`example <https://github.com/ros2/geometry2/blob/d85102217f692746abea8546c8e41f0abc95c8b8/tf2/CMakeLists.txt#L127>`__)

.. code-block:: cmake

   # Add this to the bottom of your CMakeLists.txt
   ament_package()

Update ``find_package()`` calls
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Replace the ``find_package(catkin COMPONENTS ...)``  call with individual ``find_package()`` calls (`example <https://github.com/ros2/geometry2/blob/d85102217f692746abea8546c8e41f0abc95c8b8/tf2/CMakeLists.txt#L14-L18>`_):

For example, change this:

.. code-block::

   find_package(catkin REQUIRED COMPONENTS foo bar std_msgs)
   find_package(baz REQUIRED)

To this:

.. code-block:: cmake

   find_package(ament_cmake_ros REQUIRED)
   find_package(foo REQUIRED)
   find_package(bar REQUIRED)
   find_package(std_msgs REQUIRED)
   find_package(baz REQUIRED)


Use modern CMake targets
^^^^^^^^^^^^^^^^^^^^^^^^

Prefer to use per-target CMake functions so that your package can export modern CMake targets.

If your ``CMakeLists.txt`` uses ``include_directories()``, then delete those calls.

.. code-block::

   # Delete calls to include_directories like this one!
   include_directories(include ${catkin_INCLUDE_DIRS})

Add a call ``target_include_directories()`` for every library in your pacakage (`example <https://github.com/ros2/geometry2/blob/d85102217f692746abea8546c8e41f0abc95c8b8/tf2/CMakeLists.txt#L24-L26>`__).

.. code-block:: cmake

   target_include_directories(my_library PUBLIC
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

Change all ``target_link_libraries()`` calls to use modern CMake targets.
For example, if your package in ROS 1 uses old-style standard CMake variables like this.

.. code-block::

   target_link_libraries(my_library ${catkin_LIBRARIES} ${baz_LIBRARIES})

Then change it to use specific modern CMake targets instead.
Use ``${package_name_TARGETS}`` if the package you're depending on is a message package such as ``std_msgs``.

.. code-block:: cmake

   target_link_libraries(my_library PUBLIC foo::foo bar::bar ${std_msgs_TARGETS} baz::baz)

Choose ``PUBLIC`` or ``PRIVATE`` based on how the dependency is used by your library (`example <https://github.com/ros2/geometry2/blob/d85102217f692746abea8546c8e41f0abc95c8b8/tf2/CMakeLists.txt#L27-L31>`__).

* Use ``PUBLIC`` if the dependency is needed by downstream users, for example, your library's public API uses it.
* Use ``PRIVATE`` if the dependency is only used internally by your library.

Replace ``catkin_package()`` with various ament_cmake calls
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Imagine your ``CMakeLists.txt`` has a call to ``catkin_package`` like this:

.. code-block::

   catkin_package(
       INCLUDE_DIRS include
       LIBRARIES my_library
       CATKIN_DEPENDS foo bar std_msgs
       DEPENDS baz
   )

   install(TARGETS my_library
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
   )


Replacing ``catkin_package(INCLUDE_DIRS ...)``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If you've used modern CMake targets and ``target_include_directories()``, you don't need to do anything further.
Downstream users will get the include directories by depending on your modern CMake targets.

Replacing ``catkin_package(LIBRARIES ...)``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Use ``ament_export_targets()`` and ``install(TARGETS ... EXPORT ...)`` to replace the ``LIBRARIES`` argument.

Use the ``EXPORT`` keyword when installing your ``my_library`` target (`example <https://github.com/ros2/geometry2/blob/d85102217f692746abea8546c8e41f0abc95c8b8/tf2/CMakeLists.txt#L37-L41>`__).

.. code-block:: cmake

   install(TARGETS my_library EXPORT export_my_package
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
   )

The above is a good default for library targets.
If your package used different ``CATKIN_*_DESTINATION`` variables, convert them as follows:

.. list-table::
   :header-rows: 1

   * - **catkin**
     - **ament_cmake**
   * - CATKIN_GLOBAL_BIN_DESTINATION
     - bin
   * - CATKIN_GLOBAL_INCLUDE_DESTINATION
     - include
   * - CATKIN_GLOBAL_LIB_DESTINATION
     - lib
   * - CATKIN_GLOBAL_LIBEXEC_DESTINATION
     - lib
   * - CATKIN_GLOBAL_SHARE_DESTINATION
     - share
   * - CATKIN_PACKAGE_BIN_DESTINATION
     - lib/${PROJECT_NAME}
   * - CATKIN_PACKAGE_INCLUDE_DESTINATION
     - include/${PROJECT_NAME}
   * - CATKIN_PACKAGE_LIB_DESTINATION
     - lib
   * - CATKIN_PACKAGE_SHARE_DESTINATION
     - share/${PROJECT_NAME}

Add a call to ``ament_export_targets()`` with the same name you gave to the ``EXPORT`` keyword (`example <https://github.com/ros2/geometry2/blob/d85102217f692746abea8546c8e41f0abc95c8b8/tf2/CMakeLists.txt#L124-L125>`__).

.. code-block:: cmake

   ament_export_targets(export_my_package)


Replacing ``catkin_package(CATKIN_DEPENDS .. DEPENDS ..)``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Your package's users must ``find_package()`` dependencies used by your package's public API.
In ROS 1 this was done for downstream users with the ``CATKIN_DEPENDS`` and ``DEPENDS`` arguments.
Use `ament_export_dependencies <https://github.com/ament/ament_cmake/blob/{REPOS_FILE_BRANCH}/ament_cmake_export_dependencies/cmake/ament_export_dependencies.cmake>`__ to do this in ROS 2.

.. code-block:: cmake

   ament_export_dependencies(
      foo
      bar
      std_msgs
      baz
   )

Generate messages
^^^^^^^^^^^^^^^^^

If your package contains both C++ code and ROS message, service, or action definitions, then consider splitting it into two packages:

* A package with only the ROS message, service, and/or action definitions
* A package with the C++ code

Add the following dependencies to the ``package.xml`` of the package that contains ROS messages:

1. Add a ``<buildtool_depend>`` on ``rosidl_default_generators`` (`example <https://github.com/ros2/common_interfaces/blob/d685509e9cb9f80bd320a347f2db954a73397ae7/std_msgs/package.xml#L19>`__)

   .. code-block:: xml

      <buildtool_depend>rosidl_default_generators</buildtool_depend>

2. Add an ``<exec_depend>`` on ``rosidl_default_runtime`` (`example <https://github.com/ros2/common_interfaces/blob/d685509e9cb9f80bd320a347f2db954a73397ae7/std_msgs/package.xml#L22>`__)

   .. code-block:: xml

      <exec_depend>rosidl_default_runtime</exec_depend>

3. Add a ``<member_of_group>`` tag with the group name ``rosidl_interface_packages`` (`example <https://github.com/ros2/common_interfaces/blob/d685509e9cb9f80bd320a347f2db954a73397ae7/std_msgs/package.xml#L26>`__)

   .. code-block:: xml

      <member_of_group>rosidl_interface_packages</member_of_group>

In your ``CMakeLists.txt``, replace the invocation of ``add_message_files``, ``add_service_files`` and ``generate_messages`` with `rosidl_generate_interfaces <https://github.com/ros2/rosidl/blob/{REPOS_FILE_BRANCH}/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake>`__.
The first argument must be ``${PROJECT_NAME}`` due to `this bug <https://github.com/ros2/rosidl_typesupport/issues/120>`__.

For example, if your ROS 1 package looks like this:

.. code-block::

   add_message_files(DIRECTORY msg FILES FooBar.msg Baz.msg)
   add_service_files(DIRECTORY srv FILES Ping.srv)

   add_action_files(DIRECTORY action FILES DoPong.action)
   generate_messages(
      DEPENDENCIES actionlib_msgs std_msgs geometry_msgs
   )

Then change it to this (`example <https://github.com/ros2/geometry2/blob/d85102217f692746abea8546c8e41f0abc95c8b8/tf2_msgs/CMakeLists.txt#L18-L25>`__)

.. code-block:: cmake

       rosidl_generate_interfaces(${PROJECT_NAME}
         "msg/FooBar.msg"
         "msg/Baz.msg"
         "srv/Ping.srv"
         "action/DoPong.action"
         DEPENDENCIES actionlib_msgs std_msgs geometry_msgs
       )

Remove references to the devel space
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Remove any references to the *devel space* such as ``CATKIN_DEVEL_PREFIX``.
There is no equivalent to the *devel space* in ROS 2.


Unit tests
^^^^^^^^^^

If your package uses `gtest <https://github.com/google/googletest>`__ then:

* Replace ``CATKIN_ENABLE_TESTING`` with ``BUILD_TESTING``.
* Replace ``catkin_add_gtest`` with ``ament_add_gtest``.
* Add a ``find_package()`` for ``ament_cmake_gtest`` instead of ``GTest``

For example, if your ROS 1 package adds tests like this:

.. code-block::

      if (CATKIN_ENABLE_TESTING)
        find_package(GTest REQUIRED)
        include_directories(${GTEST_INCLUDE_DIRS})
        catkin_add_gtest(my_test src/test/some_test.cpp)
        target_link_libraries(my_test
          # ...
          ${GTEST_LIBRARIES})
      endif()

Then change it to this:

.. code-block:: CMake

      if (BUILD_TESTING)
        find_package(ament_cmake_gtest REQUIRED)
        ament_add_gtest(my_test src/test/test_something.cpp)
        target_link_libraries(my_test
          #...
         )
      endif()

Add ``<test_depend>ament_cmake_gtest</test_depend>`` to your ``package.xml`` (`example <https://github.com/ros2/geometry2/blob/d85102217f692746abea8546c8e41f0abc95c8b8/tf2/package.xml#L35>`__).

.. code-block:: xml

   <test_depend>ament_cmake_gtest</test_depend>

Linters
^^^^^^^

The ROS 2 code :doc:`style guide <../../The-ROS2-Project/Contributing/Developer-Guide>` differs from ROS 1.

If you choose to follow the ROS 2 style guide, then turn on automatic linter tests by adding these lines in a ``if(BUILD_TESTING)`` block:

.. code-block:: cmake

   if(BUILD_TESTING)
      find_package(ament_lint_auto REQUIRED)
      ament_lint_auto_find_test_dependencies()
      # ...
   endif()

Add the following dependencies to your ``package.xml``:

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

Say you have a ROS 1 package called ``talker`` that uses ``roscpp`` in one node, called ``talker``.
This package is in a catkin workspace, located at ``~/ros1_talker``.

The ROS 1 code
^^^^^^^^^^^^^^

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
     <maintainer email="gerkey@osrfoundation.org">Brian Gerkey</maintainer>
     <license>Apache 2.0</license>
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

Change the publish call to use the ``->`` operator instead of ``.``.

.. code-block:: cpp

   //    chatter_pub.publish(msg);
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
   //    chatter_pub.publish(msg);
       chatter_pub->publish(msg);
   //    ros::spinOnce();
       rclcpp::spin_some(node);
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
     <maintainer email="gerkey@osrfoundation.org">Brian Gerkey</maintainer>
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
