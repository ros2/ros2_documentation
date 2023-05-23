
Reading from a bag file (C++)
=============================

**Goal:** Read data from a bag without having to play the bag.

**Tutorial level:** Advanced

**Time:** 10 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

``rosbag2`` doesn't just provide the ``ros2 bag`` command line tool.
It also provides a C++ API for reading from and writing to a bag from your own source code.
This allows you to read the contents from a bag without having play to the bag, which can be useful for certain use cases.

Prerequisites
-------------

You should have the ``rosbag2`` packages installed as part of your regular ROS 2 setup.

If you've installed from Debian packages on Linux, it may be installed by default.
If it is not, you can install it using this command.

.. code-block:: console

  sudo apt install ros-{DISTRO}-rosbag2

This tutorial discusses using ROS 2 bags, including from the terminal.
You should have already completed the :doc:`basic ROS 2 bag tutorial <../Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data>`, and we will be using the ``subset`` bag you created.

Tasks
-----

1 Create a Package
^^^^^^^^^^^^^^^^^^

Open a new terminal and :doc:`source your ROS 2 installation <../Beginner-CLI-Tools/Configuring-ROS2-Environment>` so that ``ros2`` commands will work.

In a new or existing :ref:`workspace <new-directory>`, navigate to the ``src`` directory and create
a new package:

.. code-block:: console

  ros2 pkg create --build-type ament_cmake bag_reading_cpp --dependencies rclcpp rosbag2_cpp turtlesim

Your terminal will return a message verifying the creation of your package ``bag_recorder_nodes`` and all its necessary files and folders.
The ``--dependencies`` argument will automatically add the necessary dependency lines to ``package.xml`` and ``CMakeLists.txt``.
In this case, the package will use the ``rosbag2_cpp`` package as well as the ``rclcpp`` package.
A dependency on the ``turtlesim`` package is also required for working with the custom turtlesim messages.

1.1 Update ``package.xml``
~~~~~~~~~~~~~~~~~~~~~~~~~~

Because you used the ``--dependencies`` option during package creation, you don't have to manually add dependencies to ``package.xml`` or ``CMakeLists.txt``.
As always, though, make sure to add the description, maintainer email and name, and license information to ``package.xml``.

.. code-block:: xml

  <description>C++ bag writing tutorial</description>
  <maintainer email="you@email.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

2 Write the C++ script
^^^^^^^^^^^^^^^^^^^^^^

Inside your package's ``src`` directory, create a new file called ``simple_bag_reader.cpp`` and paste the following code into it.

.. code-block:: C++

    #include <iostream>

    #include "rclcpp/serialization.hpp"
    #include "rosbag2_cpp/reader.hpp"
    #include "turtlesim/msg/pose.hpp"

    int main() {

        rclcpp::Serialization<turtlesim::msg::Pose> serialization;

        rosbag2_cpp::Reader reader;
        reader.open("/home/mroglan/subset");
        
        while (reader.has_next()) {
            rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();

            if (msg->topic_name != "/turtle1/pose") continue;

            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
            turtlesim::msg::Pose::SharedPtr ros_msg = std::make_shared<turtlesim::msg::Pose>();

            serialization.deserialize_message(&serialized_msg, ros_msg.get());

            std::cout << '(' << ros_msg->x << ", " << ros_msg->y << ")\n";
        }

        reader.close();

        return 0;
    }

2.1 Examine the code
~~~~~~~~~~~~~~~~~~~~

The ``#include`` statements at the top are the package dependencies.
Note the inclusion of headers from the ``rosbag2_cpp`` package for the functions and structures necessary to work with bag files.

First, we need to instantiate an ``rclcpp::Serialization`` object which will handle the deserialization of our specified message, in this case the turtlesim ``Pose`` message.

.. code-block:: C++

    rclcpp::Serialization<turtlesim::msg::Pose> serialization;

Then, we use rosbag2's reader to open the bag. Note that you should specify the path to the bag's directory, not a file within.

.. code-block:: C++

    rosbag2_cpp::Reader reader;
    reader.open("/path/to/subset");

We can now begin reading messages from the bag. To do so we first loop through each serialized message in the bag. 

.. code-block:: C++

    while (reader.has_next()) 
    {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();

The serialized bag message has some metadata which we can access before deserializing the message: the topic name and the timestamp of the message. In this case, we are using the topic name to identify the messages we care about, the turtle's pose, and we are ignoring other messages.

.. code-block:: C++

    if (msg->topic_name != "/turtle1/pose") continue;

We then construct an ``rclcpp::SerializedMessage`` object from the serialized data we just read. Additionally, we need to create a ROS 2 deserialized message which will hold the result of our deserialization. Then, we can pass both these objects to the ``rclcpp::Serialization::deserialize_message`` method.

.. code-block:: C++
    
    rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
    turtlesim::msg::Pose::SharedPtr ros_msg = std::make_shared<turtlesim::msg::Pose>();

    serialization.deserialize_message(&serialized_msg, ros_msg.get());

Our ``Pose`` message is now populated with the recorded pose of the turtle. 
We can then print an (x, y) coordinate to the console.

.. code-block:: C++

    std::cout << '(' << ros_msg->x << ", " << ros_msg->y << ")\n";

Finally, we close the bag reader.

.. code-block:: C++

    reader.close();

2.2 Add executable
~~~~~~~~~~~~~~~~~~

Now open the ``CMakeLists.txt`` file.

Below the dependencies block, which contains ``find_package(rosbag2_cpp REQUIRED)``, add the following lines of code.

.. code-block:: console

    add_executable(simple_bag_reader src/simple_bag_reader.cpp)
    ament_target_dependencies(simple_bag_reader rclcpp rosbag2_cpp turtlesim)

    install(TARGETS
      simple_bag_reader
      DESTINATION lib/${PROJECT_NAME}
    )

3 Build and run
^^^^^^^^^^^^^^^

Navigate back to the root of your workspace and build your new package.

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      colcon build --packages-select bag_reading_cpp

  .. group-tab:: macOS

    .. code-block:: console

      colcon build --packages-select bag_reading_cpp

  .. group-tab:: Windows

    .. code-block:: console

      colcon build --merge-install --packages-select bag_reading_cpp

Next, source the setup files.

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      source install/setup.bash

  .. group-tab:: macOS

    .. code-block:: console

      source install/setup.bash

  .. group-tab:: Windows

    .. code-block:: console

      call install/setup.bat

Now run the script:

.. code-block:: console

    ros2 run bag_reading_cpp simple_bag_reader

You should see the (x, y) coordinates of the turtle printed to the console.