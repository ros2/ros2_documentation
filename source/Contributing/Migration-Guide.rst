.. redirect-from::

    Migration-Guide

Migration guide from ROS 1
==========================

There are two different kinds of package migrations:

* Migrating the source code of an existing package from ROS 1 to ROS 2 with the intent that a significant part of the source code will stay the same or at least similar.
  An example for this could be `pluginlib <https://github.com/ros/pluginlib>`_ where the source code is maintained in different branches within the same repository and commonly patches can be ported between those branches when necessary.
* Implementing the same or similar functionality of a ROS 1 package for ROS 2 but with the assumption that the source code will be significantly different.
  An example for this could be `roscpp <https://github.com/ros/ros_comm/tree/melodic-devel/clients/roscpp>`_ in ROS 1 and `rclcpp <https://github.com/ros2/rclcpp/tree/master/rclcpp>`_ in ROS 2 which are separate repositories and don't share any code.

This article focuses on the former case and describes the high-level steps to migrate a ROS 1 package to ROS 2.
It does not aim to be a step-by-step migration instruction and is not considered the *final* "solution".
Future versions will aim to make migration smoother and less effort up to the point of maintaining a single package from the same branch for ROS 1 as well as ROS 2.

Prerequisites
-------------

Before being able to migrate a ROS 1 package to ROS 2 all of its dependencies must be available in ROS 2.

Migration steps
---------------

.. contents::
   :depth: 1
   :local:

Package manifests
^^^^^^^^^^^^^^^^^

ROS 2 doesn't support format 1 of the package specification but only newer format versions (2 and higher).
Therefore the ``package.xml`` file must be updated to at least format 2 if it uses format 1.
Since ROS 1 supports all formats it is safe to perform that conversion in the ROS 1 package.

Some packages might have different names in ROS 2 so the dependencies might need to be updated accordingly.

Metapackages
^^^^^^^^^^^^

ROS 2 doesn't have a special package type for metapackages.
Metapackages can still exist as regular packages that only contain runtime dependencies.
When migrating metapackages from ROS 1, simply remove the ``<metapackage />`` tag in your package manifest.

Message, service, and action definitions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Message files must end in ``.msg`` and must be located in the subfolder ``msg``.
Service files must end in ``.srv`` and must be located in the subfolder ``srv``.
Actions files must end in ``.action`` and must be located in the subfolder ``action``.

These files might need to be updated to comply with the `ROS Interface definition <https://design.ros2.org/articles/interface_definition.html>`__.
Some primitive types have been removed and the types ``duration`` and ``time`` which were builtin types in ROS 1 have been replaced with normal message definitions and must be used from the `builtin_interfaces <https://github.com/ros2/rcl_interfaces/tree/master/builtin_interfaces>`__ package.
Also some naming conventions are stricter than in ROS 1.

In your ``package.xml``:


* Add ``<buildtool_depend>rosidl_default_generators</buildtool_depend>``.
* Add ``<exec_depend>rosidl_default_runtime</exec_depend>``.
* For each dependent message package, add ``<depend>message_package</depend>``.

In your ``CMakeLists.txt``:

* Start by enabling C++14

.. code-block:: cmake

   set(CMAKE_CXX_STANDARD 14)


* Add ``find_package(rosidl_default_generators REQUIRED)``
* For each dependent message package, add ``find_package(message_package REQUIRED)`` and replace the CMake function call to ``generate_messages`` with ``rosidl_generate_interfaces``.

This will replace ``add_message_files`` and ``add_service_files`` listing of all the message and service files, which can be removed.

Build system
^^^^^^^^^^^^

The build system in ROS 2 is called `ament <https://design.ros2.org/articles/ament.html>`__
and the build tool is  `colcon <https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/>`__.
Ament is built on CMake: ``ament_cmake`` provides CMake functions to make writing ``CMakeLists.txt`` files easier.

Build tool
~~~~~~~~~~

Instead of using ``catkin_make``, ``catkin_make_isolated`` or ``catkin build`` ROS 2 uses the command line tool `colcon <https://design.ros2.org/articles/build_tool.html>`__ to build and install a set of packages.

Pure Python package
~~~~~~~~~~~~~~~~~~~

If the ROS 1 package uses CMake only to invoke the ``setup.py`` file and does not contain anything beside Python code (e.g. also no messages, services, etc.) it should be converted into a pure Python package in ROS 2:


*
  Update or add the build type in the ``package.xml`` file:

  .. code-block:: xml

     <export>
       <build_type>ament_python</build_type>
     </export>

*
  Remove the ``CMakeLists.txt`` file

*
  Update the ``setup.py`` file to be a standard Python setup script

ROS 2 supports Python 3 only.
While each package can choose to also support Python 2 it must invoke executables with Python 3 if it uses any API provided by other ROS 2 packages.

Update the *CMakeLists.txt* to use *ament_cmake*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
    The only valid argument for `ament_package <https://github.com/ament/ament_cmake/blob/master/ament_cmake_core/cmake/core/ament_package.cmake>`__ is ``CONFIG_EXTRAS``.
    All other arguments are covered by separate functions which all need to be invoked *before* ``ament_package``:

    * Instead of passing ``CATKIN_DEPENDS ...`` call ``ament_export_dependencies(...)`` before.
    * Instead of passing ``INCLUDE_DIRS ...`` call ``ament_export_include_directories(...)`` before.
    * Instead of passing ``LIBRARIES ...`` call ``ament_export_libraries(...)`` before.

  *
    **TODO document ament_export_targets (``ament_export_interfaces`` in Eloquent and older)?**

*
  Replace the invocation of ``add_message_files``, ``add_service_files`` and ``generate_messages`` with `rosidl_generate_interfaces <https://github.com/ros2/rosidl/blob/master/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake>`__.


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


  * The ``CATKIN_DEPENDS`` and ``DEPENDS`` arguments are passed to the new function `ament_export_dependencies <https://github.com/ament/ament_cmake/blob/master/ament_cmake_export_dependencies/cmake/ament_export_dependencies.cmake>`__.
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
~~~~~~~~~~

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
~~~~~~~

In ROS 2 we are working to maintain clean code using linters.
The styles for different languages are defined in our `Developer Guide <Developer-Guide>`.

If you are starting a project from scratch it is recommended to follow the style guide and turn on the automatic linter unit tests by adding these lines just below ``if(BUILD_TESTING)`` (until alpha 5 this was ``AMENT_ENABLE_TESTING``).

.. code-block:: cmake

   find_package(ament_lint_auto REQUIRED)
   ament_lint_auto_find_test_dependencies()

You will also need to add the following dependencies to your ``package.xml``:

.. code-block:: xml

   <test_depend>ament_lint_auto</test_depend>
   <test_depend>ament_lint_common</test_depend>

Continue to use ``catkin`` in CMake
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

ROS 2 uses ament as the build system but for backward compatibility ROS 2 has a package called ``catkin`` which provides almost the same API as catkin in ROS 1.
In order to use this backward compatibility API the ``CMakeLists.txt`` must only be updated to call the function ``catkin_ament_package()`` *after* all targets.

**NOTE: This has not been implemented yet and is only an idea at the moment.
Due to the number of changes related to dependencies it has not yet been decided if this compatibility API is useful enough to justify the effort.**

Update source code
^^^^^^^^^^^^^^^^^^

Messages, services, and actions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
~~~~~~~~~~~~~~~~~~~~~~

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
~~~~~~~~~~~~~~~~~~~

For usages of ``ros::Time``:

* Replace all instances of ``ros::Time`` with ``rclcpp::Time``

* If your messages or code makes use of std_msgs::Time:

  * Convert all instances of std_msgs::Time to builtin_interfaces::msg::Time

  * Convert all ``#include "std_msgs/time.h`` to ``#include "builtin_interfaces/msg/time.hpp"``

  * Convert all instances using the std_msgs::Time field ``nsec`` to the builtin_interfaces::msg::Time field ``nanosec``

Usages of ros::Rate
~~~~~~~~~~~~~~~~~~~

There is an equivalent type ``rclcpp::Rate`` object which is basically a drop in replacement for ``ros::Rate``.

ROS client library
~~~~~~~~~~~~~~~~~~

.. toctree::
   :titlesonly:

   Migration-Guide-Python


**NOTE: Others to be written**

Boost
~~~~~

Much of the functionality previously provided by Boost has been integrated into the C++ standard library.
As such we would like to take advantage of the new core features and avoid the dependency on boost where possible.

Shared Pointers
"""""""""""""""

To switch shared pointers from boost to standard C++ replace instances of:


* ``#include <boost/shared_ptr.hpp>`` with ``#include <memory>``
* ``boost::shared_ptr`` with ``std::shared_ptr``

There may also be variants such as ``weak_ptr`` which you want to convert as well.

Also it is recommended practice to use ``using`` instead of ``typedef``.
``using`` has the ability to work better in templated logic.
For details `see here <https://stackoverflow.com/questions/10747810/what-is-the-difference-between-typedef-and-using-in-c11>`__

Thread/Mutexes
""""""""""""""

Another common part of boost used in ROS codebases are mutexes in ``boost::thread``.


* Replace ``boost::mutex::scoped_lock`` with ``std::unique_lock<std::mutex>``
* Replace ``boost::mutex`` with ``std::mutex``
* Replace ``#include <boost/thread/mutex.hpp>`` with ``#include <mutex>``

Unordered Map
"""""""""""""

Replace:


* ``#include <boost/unordered_map.hpp>`` with ``#include <unordered_map>``
* ``boost::unordered_map`` with ``std::unordered_map``

function
""""""""

Replace:


* ``#include <boost/function.hpp>``  with ``#include <functional>``
* ``boost::function`` with ``std::function``

Parameters
----------

In ROS 1, parameters are associated with a central server that allowed retrieving parameters at runtime through the use of the network APIs.
In ROS 2, parameters are associated per node and are configurable at runtime with ROS services.

* See `ROS 2 Parameter design document <https://design.ros2.org/articles/ros_parameters.html>`_ for more details about the system model.

* See `ROS 2 CLI usage <https://index.ros.org/doc/ros2/Tutorials/Parameters/Understanding-ROS2-Parameters/>`_ for a better understanding of how the CLI tools work and its differences with ROS 1 tooling.

* See :ref:`yaml-ros1-ros2` to see how YAML parameter files are parsed in ROS 2 and their differences with ROS implementation.

Launch files
------------

While launch files in ROS 1 are always specified using `.xml <https://wiki.ros.org/roslaunch/XML>`__ files, ROS 2 supports Python scripts to enable more flexibility (see `launch package <https://github.com/ros2/launch/tree/master/launch>`__) as well as XML and YAML files.
See `separate tutorial <https://index.ros.org/doc/ros2/Tutorials/Launch-files-migration-guide/>`__ on migrating launch files from ROS 1 to ROS 2.

Example: Converting an existing ROS 1 package to use ROS 2
----------------------------------------------------------

Let's say that we have simple ROS 1 package called ``talker`` that uses ``roscpp``
in one node, called ``talker``.
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

We source an environment setup file (in this case for Jade using bash), then we
build our package using ``catkin_make install``:

.. code-block:: bash

   . /opt/ros/jade/setup.bash
   cd ~/ros1_talker
   catkin_make install

Running the ROS 1 node
~~~~~~~~~~~~~~~~~~~~~~

If there's not already one running, we start a ``roscore``, first sourcing the
setup file from our ``catkin`` install tree (the system setup file at
``/opt/ros/jade/setup.bash`` would also work here):

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
the initialization, then pass the node name to the creation of the node object
(we can use the ``auto`` keyword because now we're requiring a C++14 compiler):

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
and `concept overview <https://index.ros.org/doc/ros2/Concepts/About-Quality-of-Service-Settings>`__.

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

ROS 2 doesn't support format 1 of the package specification but only newer format versions (2 and higher).
We start by specifying the format version in the ``package`` tag:

.. code-block:: xml

   <!-- <package> -->
   <package format="2">

ROS 2 uses a newer version of ``catkin``, called ``ament_cmake``, which we specify in the
``buildtool_depend`` tag:

.. code-block:: xml

   <!--  <buildtool_depend>catkin</buildtool_depend> -->
     <buildtool_depend>ament_cmake</buildtool_depend>

In our build dependencies, instead of ``roscpp`` we use ``rclcpp``, which provides
the C++ API that we use.

.. code-block:: xml

   <!--  <build_depend>roscpp</build_depend> -->
     <build_depend>rclcpp</build_depend>

We make the same addition in the run dependencies and also update from the
``run_depend`` tag to the ``exec_depend`` tag (part of the upgrade to version 2 of
the package format):

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

**TODO: show simpler version of this file just using the ``<depend>`` tag, which is
enabled by version 2 of the package format (also supported in ``catkin`` so,
strictly speaking, orthogonal to ROS 2).**

Changing the CMake code
~~~~~~~~~~~~~~~~~~~~~~~

ROS 2 relies on a higher version of CMake:

.. code-block:: bash

   #cmake_minimum_required(VERSION 2.8.3)
   cmake_minimum_required(VERSION 3.5)

ROS 2 relies on the C++14 standard.
Depending on what compiler you're using, support for C++14 might not be enabled
by default.
Using ``gcc`` 5.3 (which is what is used on Ubuntu Xenial), we need to enable it
explicitly, which we do by adding this line near the top of the file:

.. code-block:: cmake

   set(CMAKE_CXX_STANDARD 14)

The preferred way to work on all platforms is this:

.. code-block:: cmake

   if(NOT CMAKE_CXX_STANDARD)
     set(CMAKE_CXX_STANDARD 14)
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

   target_include_directories(target include ${Boost_INCLUDE_DIRS})

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
With ``ament_cmake``, we just give a path relative to the installation root, like ``bin``
for executables:

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
     set(CMAKE_CXX_STANDARD 14)
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

**TODO: Show what this would look like with ``ament_auto``.**

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

Because we installed the ``talker`` executable into ``bin``, after sourcing the
setup file, from our install tree, we can invoke it by name directly
(also, there is not yet a ROS 2 equivalent for ``rosrun``):

.. code-block:: bash

   . ~/ros2_ws/install/setup.bash
   talker

Update scripts
^^^^^^^^^^^^^^

ROS CLI arguments
~~~~~~~~~~~~~~~~~

Since `ROS Eloquent <../Releases/Release-Eloquent-Elusor>`, ROS arguments should be scoped with ``--ros-args`` and a trailing ``--`` (the trailing double dash may be elided if no arguments follow it).

Remapping names is similar to ROS 1, taking on the form ``from:=to``, except that it must be preceded by a ``--remap`` (or ``-r``) flag.
For example:

.. code-block:: bash

   ros2 run some_package some_ros_executable --ros-args -r foo:=bar

We use a similar syntax for parameters, using the ``--param`` (or ``-p``) flag:


.. code-block:: bash

   ros2 run some_package some_ros_executable --ros-args -p my_param:=value

Note, this is different than using a leading underscore in ROS 1.

To change a node name use ``__node`` (the ROS 1 equivalent is ``__name``):

.. code-block:: bash

   ros2 run some_package some_ros_executable --ros-args -r __node:=new_node_name

Note the use of the ``-r`` flag.
The same remap flag is needed for changing the namespace ``__ns``:

.. code-block:: bash

   ros2 run some_package some_ros_executable --ros-args -r __ns:=/new/namespace

There is no equivalent in ROS 2 for the following ROS 1 keys:

- ``__log`` (but ``--log-config-file`` can be used to provide a logger configuration file)
- ``__ip``
- ``__hostname``
- ``__master``

For more information, see the `design document <https://design.ros2.org/articles/ros_command_line_arguments.html>`_.

Quick reference
"""""""""""""""

+------------+-------------+----------------+
| Feature    | ROS 1       | ROS 2          |
+============+=============+================+
| remapping  | foo:=bar    | -r foo:=bar    |
+------------+-------------+----------------+
| parameters | _foo:=bar   | -p foo:=bar    |
+------------+-------------+----------------+
| node name  | __name:=foo | -r __node:=foo |
+------------+-------------+----------------+
| namespace  | __ns:=foo   | -r __ns:=foo   |
+------------+-------------+----------------+

Licensing
---------

In ROS 2 our recommended license is the `Apache 2.0 License <https://www.apache.org/licenses/LICENSE-2.0>`__.
In ROS 1 our recommended license was the `3-Clause BSD License <https://opensource.org/licenses/BSD-3-Clause>`__.

For any new project we recommend using the Apache 2.0 License, whether ROS 1 or ROS 2.

However, when migrating code from ROS 1 to ROS 2 we cannot simply change the license.
The existing license must be preserved for any preexisting contributions.

To that end if a package is being migrated we recommend keeping the existing license and continuing to contribute to that package under the existing OSI license, which we expect to be the BSD license for core elements.

This will keep things clear and easy to understand.

Changing the License
^^^^^^^^^^^^^^^^^^^^

It is possible to change the license, however you will need to contact all the contributors and get permission.
For most packages this is likely to be a significant effort and not worth considering.
If the package has a small set of contributors then this may be feasible.
