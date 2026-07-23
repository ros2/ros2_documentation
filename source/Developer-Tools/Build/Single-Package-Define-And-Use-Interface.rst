.. redirect-from::

   How-To-Guides/Implementing-custom-interfaces

Implementing custom interfaces - how-to
=======================================

When predefined interface definitions are not enough, you need to create custom interfaces.
In this article, you will learn how to define and build interfaces with different field types.
This will help you implement custom interfaces in ROS to suit your needs.

**Area: Framework | Content-type: how-to | Experience: beginner, intermediate**

.. contents:: Contents
   :depth: 2
   :local:

Summary
-------

Interfaces define how nodes exchange data.
ROS offers three main interface types:

* Topics (``.msg`` files)
* Services (``.srv`` files)
* Actions (``.action`` files)

`Learn more about interfaces <https://docs.ros.org/en/{DISTRO}/Concepts/Basic/Interfaces-Topics-Services-Actions.html>`__

Before creating a custom interface, do the following:

1. Check whether a suitable standard message already exists.
2. If no single standard message fits your use case, consider creating a new message composed of standard messages.
   See standard messages here: https://github.com/ros2/common_interfaces.

Creating a completely custom message should be the last resort.

Creating custom interfaces involves preparing a package, specifying interface definitions, and registering the interfaces in ``package.xml`` and ``CMakeLists.txt``.
Using custom interfaces involves configuring a node to include the interfaces in its source, and configuring the node to build with the interfaces in ``CMakeLists.txt``.

.. tip::

   The best practice is to declare interfaces in dedicated interface packages, but sometimes it may be more convenient for you to declare, create and use an interface all in one package.
   Using a dedicated interface package is preferred because it allows multiple packages to share message definitions without sharing any other code contained in the package.

Prerequisites
-------------

#. :doc:`Install ROS </Get-Started/Installation>`, and create your :doc:`workspace </ROS-Framework/client-libraries/Working-with-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace>`.
#. Make sure you understand how to :doc:`create packages </ROS-Framework/client-libraries/Working-with-Client-Libraries/Creating-Your-First-ROS2-Package>`.

Steps
-----

.. note::

   For our examples, we are using the ``msg`` interface type, but the steps below apply to all interface types.

#. In your workspace ``src`` folder, create a ``more_interfaces`` CMake package with a folder for interface definitions.
   For example:

   .. code-block:: console

      $ ros2 pkg create --build-type ament_cmake more_interfaces
      $ mkdir -p more_interfaces/msg

   .. note::

      In ROS 2, interfaces can only be defined in CMake packages.
      You can also use `ament_cmake_python <https://github.com/ament/ament_cmake/tree/{REPOS_FILE_BRANCH}/ament_cmake_python>`__ to include Python libraries and nodes in a CMake package.

#. In your interface definitions folder, create a file in which you provide the definitions for the interface.
   For example, for a message interface, you can create an ``AddressBook.msg`` file that collects personal data:
   The ``PHONE_TYPE_*`` constants in this example form an enumerated type pattern for ``phone_type`` values.

   .. code-block:: text

      uint8 PHONE_TYPE_HOME=0
      uint8 PHONE_TYPE_WORK=1
      uint8 PHONE_TYPE_MOBILE=2
      string first_name
      string last_name
      string phone_number
      uint8 phone_type
      geometry_msgs/Point location

#. In ``package.xml``, add the following code to register your package as part of interface groups:
   ``rosidl_default_generators``: Needed to generate the code during the build.
   ``rosidl_default_runtime``: Needed only at run time.

   .. code-block:: xml

      <build_depend>rosidl_default_generators</build_depend>
      <exec_depend>rosidl_default_runtime</exec_depend>
      <depend>geometry_msgs</depend>
      <member_of_group>rosidl_interface_packages</member_of_group>

#. In ``CMakeLists.txt``, add the required code to make the runtime libraries available and to generate source files from your interface definition.
   For example:

   .. code-block:: cmake

      find_package(rosidl_default_generators REQUIRED)
      find_package(geometry_msgs REQUIRED)
      set(msg_files "msg/AddressBook.msg")
      rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files}
        DEPENDENCIES geometry_msgs
      )
      ament_export_dependencies(rosidl_default_runtime)

#. In the ``more_interfaces/src`` folder, create a node to interact with your new interface.
   For example, for a message interface, create ``publish_address_book.cpp`` with code to publish the message periodically.

   .. code-block:: c++

      #include <chrono>
      #include <memory>

      #include "rclcpp/rclcpp.hpp"
      #include "more_interfaces/msg/address_book.hpp"

      using namespace std::chrono_literals;

      class AddressBookPublisher : public rclcpp::Node
      {
      public:
        AddressBookPublisher()
        : Node("address_book_publisher")
        {
          address_book_publisher_ =
            this->create_publisher<more_interfaces::msg::AddressBook>("address_book", 10);

          auto publish_msg = [this]() -> void {
              auto message = more_interfaces::msg::AddressBook();

              message.first_name = "John";
              message.last_name = "Doe";
              message.phone_number = "1234567890";
              message.phone_type = message.PHONE_TYPE_MOBILE;
              message.location.x = 37.7749;
              message.location.y = -122.4194;
              message.location.z = 0.0;

              std::cout << "Publishing Contact\nFirst:" << message.first_name <<
                "  Last:" << message.last_name << std::endl;

              this->address_book_publisher_->publish(message);
            };
          timer_ = this->create_wall_timer(1s, publish_msg);
        }

      private:
        rclcpp::Publisher<more_interfaces::msg::AddressBook>::SharedPtr address_book_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
      };


      int main(int argc, char * argv[])
      {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<AddressBookPublisher>());
        rclcpp::shutdown();

        return 0;
      }

#. In ``CMakeLists.txt``, create a new target so the node builds correctly.
   For example:

   .. code-block:: cmake

      find_package(rclcpp REQUIRED)
      add_executable(publish_address_book src/publish_address_book.cpp)
      target_link_libraries(publish_address_book rclcpp::rclcpp)
      install(TARGETS publish_address_book DESTINATION lib/${PROJECT_NAME})

#. In ``CMakeLists.txt``, link the node to your interface.
   For example:

   .. code-block:: cmake

      rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
      target_link_libraries(publish_address_book "${cpp_typesupport_target}")

#. To test your new interface, do the following:

   a) In your workspace root, build the package.

   b) Source the workspace and run the node that uses the interface.

      For example:

      .. tabs::

        .. group-tab:: Linux

          .. code-block:: console

            $ cd ~/ros2_ws
            $ colcon build --packages-up-to more_interfaces
            $ source install/local_setup.bash
            $ ros2 run more_interfaces publish_address_book

        .. group-tab:: macOS

          .. code-block:: console

            $ cd ~/ros2_ws
            $ colcon build --packages-up-to more_interfaces
            $ . install/local_setup.bash
            $ ros2 run more_interfaces publish_address_book

        .. group-tab:: Windows

          .. code-block:: console

            $ cd /ros2_ws
            $ colcon build --merge-install --packages-up-to more_interfaces
            $ call install/local_setup.bat
            $ ros2 run more_interfaces publish_address_book

          Or using Powershell:

          .. code-block:: console

            $ install/local_setup.ps1
            $ ros2 run more_interfaces publish_address_book

   c) Check the interface or interact with it.

      For example, for a message interface, you could open another terminal and use the following code:

      .. tabs::

        .. group-tab:: Linux

          .. code-block:: console

            $ source install/setup.bash
            $ ros2 topic echo /address_book

        .. group-tab:: macOS

          .. code-block:: console

            $ . install/setup.bash
            $ ros2 topic echo /address_book

        .. group-tab:: Windows

          .. code-block:: console

            $ call install/setup.bat
            $ ros2 topic echo /address_book

          Or using Powershell:

          .. code-block:: console

            $ install/setup.ps1
            $ ros2 topic echo /address_book
