
**INCOMPLETE: this is a draft of an upcoming tutorial for creating and using custom ROS interfaces.**

**Disclaimer: The code provided is to support the explanation, it is likely outdated and should not be expected to compile as is**

Introduction to msg and srv
===========================


* msg: msg files are simple text files that describe the fields of a ROS message. They are used to generate source code for messages in different languages.
* srv: an srv file describes a service. It is composed of two parts: a request and a response. The request and response are message declarations.

msgs are just simple text files with a field type and field name per line. The field types you can use are:


* int8, int16, int32, int64 (plus uint*)
* float32, float64
* string
* other msg files
* variable-length array[], fixed-length array[C], bounded-length array[<=C]

Here is an example of a msg that uses a string primitive, and two other msgs:

.. code-block:: bash

     string child_frame_id
     geometry_msgs/PoseWithCovariance pose
     geometry_msgs/TwistWithCovariance twist

srv files are just like msg files, except they contain two parts: a request and a response. The two parts are separated by a '---' line. Here is an example of a srv file:

.. code-block:: bash

   float64 A
   float64 B
   ---
   float64 Sum

In the above example, A and B are the request, and Sum is the response.

msg files are stored in the ``msg`` directory of a package, and srv files are stored in the ``srv`` directory.

These are just simple examples.
For more information about how to create msg and srv files please refer to `About ROS Interfaces <../Concepts/About-ROS-Interfaces>`.

Creating a msg package
======================

**NOTE:** only ament_cmake packages can generate messages currently (not ament_python packages).

For this tutorial we will use the packages stored in the `rosidl_tutorials repository <https://github.com/ros2/tutorials/tree/rosidl_tutorials/rosidl_tutorials>`__

.. code-block:: bash

   cd ~/ros2_overlway_ws/src
   git clone -b rosidl_tutorials https://github.com/ros2/tutorials.git
   cd rosidl_tutorials/rosidl_tutorials_msgs

Creating a msg file
-------------------

Here we will create a message meant to carry information about an individual.

Open ``msg/Contact.msg`` and you will see:

.. code-block:: bash

   bool FEMALE=true
   bool MALE=false

   string first_name
   string last_name
   bool gender
   uint8 age
   string address

This message is composed of 5 fields:


* first_name: of type string
* last_name: of type string
* gender: of type bool, that can be either MALE or FEMALE
* age: of type uint8
* address: of type string

There's one more step, though. We need to make sure that the msg files are turned into source code for C++, Python, and other languages:

Open the ``package.xml``\ , and uncomment these two lines:

.. code-block:: xml

     <buildtool_depend>rosidl_default_generators</buildtool_depend>

     <exec_depend>rosidl_default_runtime</exec_depend>

Note that at build time, we need "rosidl_default_generators", while at runtime, we only need "rosidl_default_runtime".

Open the ``CMakeLists.txt`` and make sure that the following lines are uncommented.

Find the package that generates message code from msg/srv files:

.. code-block:: cmake

   find_package(rosidl_default_generators REQUIRED)

Declare the list of messages you want to generate:

.. code-block:: cmake

   set(msg_files
     "msg/Contact.msg"
   )

By adding the .msg files manually, we make sure that CMake knows when it has to reconfigure the project after you add other .msg files.

Generate the messages:

.. code-block:: cmake

   rosidl_generate_interfaces(${PROJECT_NAME}
     ${msg_files}
   )

Also make sure you export the message runtime dependency:

.. code-block:: cmake

   ament_export_dependencies(rosidl_default_runtime)

Now you're ready to generate source files from your msg definition.

Creating a srv
==============

We will now add a srv declaration to our package.

Open the srv/AddTwoFloats.srv file and paste this srv declaration:

.. code-block:: bash

   float64 a
   float64 b
   ---
   float64 sum

Declare the service in the ``CMakeLists.txt``\ :

.. code-block:: cmake

   set(srv_files
     "srv/AddTwoFloats.srv")

Modify the existing call to rosidl_generate_interfaces to generate the service in addition to the messages:

.. code-block:: cmake

   rosidl_generate_interfaces(${PROJECT_NAME}
     ${msg_files}
     ${srv_files}
   )

Using custom messages
=====================

Using msg/srv from other packages
---------------------------------

Let's write a C++ node using the Contact.msg we just created.

Go to the rosidl_tutorials package and open the src/publish_contact.cpp file.

.. code-block:: c++

   #include <iostream>
   #include <memory>

   #include "rclcpp/rclcpp.hpp"

   #include "rosidl_tutorials_msgs/msg/contact.hpp"


   using namespace std::chrono_literals;

   class ContactPublisher : public rclcpp::Node
   {
   public:
     ContactPublisher()
     : Node("address_book_publisher")
     {
       contact_publisher_ = this->create_publisher<rosidl_tutorials_msgs::msg::Contact>("contact");

       auto publish_msg = [this]() -> void {
           auto msg = std::make_shared<rosidl_tutorials_msgs::msg::Contact>();

           msg->first_name = "John";
           msg->last_name = "Doe";
           msg->age = 30;
           msg->gender = msg->MALE;
           msg->address = "unknown";

           std::cout << "Publishing Contact\nFirst:" << msg->first_name <<
             "  Last:" << msg->last_name << std::endl;

           contact_publisher_->publish(msg);
         };
       timer_ = this->create_wall_timer(1s, publish_msg);
     }

   private:
     rclcpp::Publisher<rosidl_tutorials_msgs::msg::Contact>::SharedPtr contact_publisher_;
     rclcpp::timer::TimerBase::SharedPtr timer_;
   };


   int main(int argc, char * argv[])
   {
     rclcpp::init(argc, argv);

     auto publisher_node = std::make_shared<ContactPublisher>();

     rclcpp::spin(publisher_node);

     return 0;
   }

The code explained
^^^^^^^^^^^^^^^^^^

.. code-block:: c++

   #include "rosidl_tutorials_msgs/msg/contact.hpp"

Here we include the header of the message that we want to use.

.. code-block:: c++

     ContactPublisher()
     : Node("address_book_publisher")
     {

Here we define a node

.. code-block:: c++

   auto publish_msg = [this]() -> void {

A publish_msg function to send our message periodically

.. code-block:: c++

          auto msg = std::make_shared<rosidl_tutorials_msgs::msg::Contact>();

           msg->first_name = "John";
           msg->last_name = "Doe";
           msg->age = 30;
           msg->gender = msg->MALE;
           msg->address = "unknown";

We create a Contact message and populate its fields.

.. code-block:: c++

           std::cout << "Publishing Contact\nFirst:" << msg->first_name <<
             "  Last:" << msg->last_name << std::endl;

           contact_publisher_->publish(msg);

Finally we publish it

.. code-block:: c++

       timer_ = this->create_wall_timer(1s, publish_msg);

Create a 1second timer to call our ``publish_msg`` function every second

Now let's build it!

To use this message we need to declare a dependency on rosidl_tutorials_msgs in the ``package.xml``\ :

.. code-block:: xml

     <build_depend>rosidl_tutorials_msgs</build_depend>

     <exec_depend>rosidl_tutorials_msgs</exec_depend>

And also in the ``CMakeLists.txt``\ :

.. code-block:: cmake

   find_package(rosidl_tutorials_msgs REQUIRED)

And finally we must declare the message package as a target dependency for the executable.

.. code-block:: cmake

   ament_target_dependencies(publish_contact
     "rclcpp"
     "rosidl_tutorials_msgs"
   )

Using msg/srv from the same package
-----------------------------------

While most of the time messages are declared in interface packages, it can be convenient to declare, create and use messages all in the one package.

We will create a message in our rosidl_tutorials package.
Create a msg directory in the rosidl_tutorials package and AddressBook.msg inside that directory.
In that msg paste:

.. code-block:: bash

   rosidl_tutorials_msgs/Contact[] address_book

As you can see we define a message based on the Contact message we created earlier.

To generate this message we need to declare a dependency on this package in the ``package.xml``\ :

.. code-block:: xml

     <build_depend>rosidl_tutorials_msgs</build_depend>

     <exec_depend>rosidl_tutorials_msgs</exec_depend>

And in the ``CMakeLists.txt``\ :

.. code-block:: cmake

   find_package(rosidl_tutorials_msgs REQUIRED)

   set(msg_files
     "msg/AddressBook.msg"
   )

   rosidl_generate_interfaces(${PROJECT_NAME}
     ${msg_files}
     DEPENDENCIES rosidl_tutorials_msgs
   )

Now we can start writing code that uses this message.

Open src/publish_address_book.cpp:

.. code-block:: c++

   #include <iostream>
   #include <memory>

   #include "rclcpp/rclcpp.hpp"

   #include "rosidl_tutorials/msg/address_book.hpp"
   #include "rosidl_tutorials_msgs/msg/contact.hpp"

   using namespace std::chrono_literals;

   class AddressBookPublisher : public rclcpp::Node
   {
   public:
     AddressBookPublisher()
     : Node("address_book_publisher")
     {
       address_book_publisher_ =
         this->create_publisher<rosidl_tutorials::msg::AddressBook>("address_book");

       auto publish_msg = [this]() -> void {
           auto msg = std::make_shared<rosidl_tutorials::msg::AddressBook>();
           {
             rosidl_tutorials_msgs::msg::Contact contact;
             contact.first_name = "John";
             contact.last_name = "Doe";
             contact.age = 30;
             contact.gender = contact.MALE;
             contact.address = "unknown";
             msg->address_book.push_back(contact);
           }
           {
             rosidl_tutorials_msgs::msg::Contact contact;
             contact.first_name = "Jane";
             contact.last_name = "Doe";
             contact.age = 20;
             contact.gender = contact.FEMALE;
             contact.address = "unknown";
             msg->address_book.push_back(contact);
           }

           std::cout << "Publishing address book:" << std::endl;
           for (auto contact : msg->address_book) {
             std::cout << "First:" << contact.first_name << "  Last:" << contact.last_name <<
               std::endl;
           }

           address_book_publisher_->publish(msg);
         };
       timer_ = this->create_wall_timer(1s, publish_msg);
     }

   private:
     rclcpp::Publisher<rosidl_tutorials::msg::AddressBook>::SharedPtr address_book_publisher_;
     rclcpp::timer::TimerBase::SharedPtr timer_;
   };


   int main(int argc, char * argv[])
   {
     rclcpp::init(argc, argv);
     auto publisher_node = std::make_shared<AddressBookPublisher>();

     rclcpp::spin(publisher_node);

     return 0;
   }

The code explained
^^^^^^^^^^^^^^^^^^

.. code-block:: c++

   #include "rosidl_tutorials/msg/address_book.hpp"

We include the header of our newly created AddressBook msg.

.. code-block:: c++

   #include "rosidl_tutorials_msgs/msg/contact.hpp"

Here we include the header of the Contact msg in order to be able to add contacts to our address_book.

.. code-block:: c++

   using namespace std::chrono_literals;

   class AddressBookPublisher : public rclcpp::Node
   {
   public:
     AddressBookPublisher()
     : Node("address_book_publisher")
     {
       address_book_publisher_ =
         this->create_publisher<rosidl_tutorials::msg::AddressBook>("address_book");

We create a node and an AddressBook publisher.

.. code-block:: c++

       auto publish_msg = [this]() -> void {

We create a callback to publish the messages periodically

.. code-block:: c++

           auto msg = std::make_shared<rosidl_tutorials::msg::AddressBook>();

We create an AddressBook message instance that we will later publish.

.. code-block:: c++

     {
     rosidl_tutorials_msgs::msg::Contact contact;
     contact.first_name = "John";
     contact.last_name = "Doe";
     contact.age = 30;
     contact.gender = contact.MALE;
     contact.address = "unknown";
     msg->address_book.push_back(person);
     }
     {
     rosidl_tutorials_msgs::msg::Contact person;
     contact.first_name = "Jane";
     contact.last_name = "Doe";
     contact.age = 20;
     contact.gender = contact.FEMALE;
     contact.address = "unknown";
     msg->address_book.push_back(contact);
     }

We create and populate Contact messages and add them to our address_book message.

.. code-block:: c++

           std::cout << "Publishing address book:" << std::endl;
           for (auto contact : msg->address_book) {
             std::cout << "First:" << contact.first_name << "  Last:" << contact.last_name <<
               std::endl;
           }

           address_book_publisher_->publish(msg);

Finally send the message periodically.

.. code-block:: c++

       timer_ = this->create_wall_timer(1s, publish_msg);

Create a 1second timer to call our ``publish_msg`` function every second

Now let's build it!
We need to create a new target for this node in the ``CMakeLists.txt``\ :

.. code-block:: cmake

   add_executable(publish_address_book 
     src/publish_address_book.cpp
   )

   ament_target_dependencies(publish_address_book
     "rclcpp"
   )

In order to use the messages generated in the same package we need to use the following cmake code:

.. code-block:: cmake

   get_default_rmw_implementation(rmw_implementation)
   find_package("${rmw_implementation}" REQUIRED)
   get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")

   foreach(typesupport_impl ${typesupport_impls})
     rosidl_target_interfaces(publish_address_book
       ${PROJECT_NAME} ${typesupport_impl}
     )
   endforeach()

This finds the relevant generated C++ code from msg/srv and allows your target to link against them.

You may have noticed that this step was not necessary when the interfaces being used were from a package that was built beforehand.
This CMake code is only required when you are trying to use interfaces in the same package as that in which they are built.
