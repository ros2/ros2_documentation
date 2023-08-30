Reading from a bag file (C++)
=============================

**Goal:** Read data from a bag without using the CLI.

**Tutorial level:** Advanced

**Time:** 10 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

``rosbag2`` doesn't just provide the ``ros2 bag`` command line tool.
It also provides a C++ API for reading from and writing to a bag from your own source code.
This allows you to read the contents from a bag without having to play the bag, which can sometimes be useful.

Prerequisites
-------------

You should have the ``rosbag2`` packages installed as part of your regular ROS 2 setup.

If you've installed from Debian packages on Linux, it may be installed by default.
If it is not, you can install it using this command.

.. code-block:: console

  sudo apt install ros-{DISTRO}-rosbag2

This tutorial discusses using ROS 2 bags.
You should have already completed the :doc:`basic ROS 2 bag tutorial <../Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data>`, and we will be using the ``subset`` bag you created there.

Tasks
-----

1 Create a Package
^^^^^^^^^^^^^^^^^^

Open a new terminal and :doc:`source your ROS 2 installation <../Beginner-CLI-Tools/Configuring-ROS2-Environment>` so that ``ros2`` commands will work.

In a new or existing :ref:`workspace <new-directory>`, navigate to the ``src`` directory and create
a new package:

.. code-block:: console

  ros2 pkg create --build-type ament_cmake --license Apache-2.0 bag_reading_cpp --dependencies rclcpp rosbag2_cpp turtlesim

Your terminal will return a message verifying the creation of your package ``bag_reading_cpp`` and all its necessary files and folders.
The ``--dependencies`` argument will automatically add the necessary dependency lines to ``package.xml`` and ``CMakeLists.txt``.
In this case, the package will use the ``rosbag2_cpp`` package as well as the ``rclcpp`` package.
A dependency on the ``turtlesim`` package is also required for working with the custom turtlesim messages.

1.1 Update ``package.xml``
~~~~~~~~~~~~~~~~~~~~~~~~~~

Because you used the ``--dependencies`` option during package creation, you don't have to manually add dependencies to ``package.xml`` or ``CMakeLists.txt``.
As always, though, make sure to add the description, maintainer email and name, and license information to ``package.xml``.

.. code-block:: xml

  <description>C++ bag reading tutorial</description>
  <maintainer email="you@email.com">Your Name</maintainer>
  <license>Apache-2.0</license>

2 Write the C++ Reader
^^^^^^^^^^^^^^^^^^^^^^

Inside your package's ``src`` directory, create a new file called ``simple_bag_reader.cpp`` and paste the following code into it.

.. code-block:: C++

    #include <chrono>
    #include <functional>
    #include <iostream>
    #include <memory>
    #include <string>

    #include "rclcpp/rclcpp.hpp"
    #include "rclcpp/serialization.hpp"
    #include "rosbag2_cpp/reader.hpp"
    #include "turtlesim/msg/pose.hpp"

    using namespace std::chrono_literals;

    class PlaybackNode : public rclcpp::Node
    {
      public:
        PlaybackNode(const std::string & bag_filename)
        : Node("playback_node")
        {
          publisher_ = this->create_publisher<turtlesim::msg::Pose>("/turtle1/pose", 10);
          timer_ = this->create_wall_timer(
              100ms, std::bind(&PlaybackNode::timer_callback, this));

          reader_.open(bag_filename);
        }

      private:
        void timer_callback()
        {
          while (reader_.has_next()) {
            rosbag2_storage::SerializedBagMessageSharedPtr msg = reader_.read_next();

            if (msg->topic_name != "/turtle1/pose") {
              continue;
            }

            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
            turtlesim::msg::Pose::SharedPtr ros_msg = std::make_shared<turtlesim::msg::Pose>();

            serialization_.deserialize_message(&serialized_msg, ros_msg.get());

            publisher_->publish(*ros_msg);
            std::cout << '(' << ros_msg->x << ", " << ros_msg->y << ")\n";

            break;
          }
        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<turtlesim::msg::Pose>::SharedPtr publisher_;

        rclcpp::Serialization<turtlesim::msg::Pose> serialization_;
        rosbag2_cpp::Reader reader_;
    };

    int main(int argc, char ** argv)
    {
      if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <bag>" << std::endl;
        return 1;
      }

      rclcpp::init(argc, argv);
      rclcpp::spin(std::make_shared<PlaybackNode>(argv[1]));
      rclcpp::shutdown();

      return 0;
    }

2.1 Examine the code
~~~~~~~~~~~~~~~~~~~~

The ``#include`` statements at the top are the package dependencies.
Note the inclusion of headers from the ``rosbag2_cpp`` package for the functions and structures necessary to work with bag files.

The next line creates the node which will read from the bag file and play back the data.

.. code-block:: C++

    class PlaybackNode : public rclcpp::Node

Now, we can create a timer callback which will run at 10 hz.
Our goal is to replay one message to the ``/turtle1/pose`` topic each time the callback is run.
Note the constructor takes a path to the bag file as a parameter.

.. code-block:: C++

    public:
      PlaybackNode(const std::string & bag_filename)
      : Node("playback_node")
      {
        publisher_ = this->create_publisher<turtlesim::msg::Pose>("/turtle1/pose", 10);
        timer_ = this->create_wall_timer(
            100ms, std::bind(&PlaybackNode::timer_callback, this));

We also open the bag in the constructor.

.. code-block:: C++

      reader_.open(bag_filename);

Now, inside our timer callback, we loop through messages in the bag until we read a message recorded from our desired topic.
Note that the serialized message has timestamp metadata in addition to the topic name.

.. code-block:: C++

    void timer_callback()
    {
      while (reader_.has_next()) {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader_.read_next();

        if (msg->topic_name != "/turtle1/pose") {
          continue;
        }

We then construct an ``rclcpp::SerializedMessage`` object from the serialized data we just read.
Additionally, we need to create a ROS 2 deserialized message which will hold the result of our deserialization.
Then, we can pass both these objects to the ``rclcpp::Serialization::deserialize_message`` method.

.. code-block:: C++

    rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
    turtlesim::msg::Pose::SharedPtr ros_msg = std::make_shared<turtlesim::msg::Pose>();

    serialization_.deserialize_message(&serialized_msg, ros_msg.get());

Finally, we publish the deserialized message and print out the xy coordinate to the terminal.
We also break out of the loop so that we publish the next message during the next timer calback.

.. code-block:: C++

      publisher_->publish(*ros_msg);
      std::cout << '(' << ros_msg->x << ", " << ros_msg->y << ")\n";

      break;
    }

We must also declare the private variables used throughout the node.

.. code-block:: C++

      rclcpp::TimerBase::SharedPtr timer_;
      rclcpp::Publisher<turtlesim::msg::Pose>::SharedPtr publisher_;

      rclcpp::Serialization<turtlesim::msg::Pose> serialization_;
      rosbag2_cpp::Reader reader_;
    };

Lastly, we create the main function which will check that the user passes an argument for the bag file path and spins our node.

.. code-block:: C++

    int main(int argc, char ** argv)
    {
      if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <bag>" << std::endl;
        return 1;
      }

      rclcpp::init(argc, argv);
      rclcpp::spin(std::make_shared<PlaybackNode>(argv[1]));
      rclcpp::shutdown();

      return 0;
    }

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

Now, run the script.
Make sure to replace ``/path/to/setup`` with the path to your ``setup`` bag.

.. code-block:: console

    ros2 run bag_reading_cpp simple_bag_reader /path/to/setup

You should see the (x, y) coordinates of the turtle printed to the console.

Summary
-------

You created a C++ executable that reads data from a bag.
You then compiled and ran the executable which printed some information from the bag to the console.
