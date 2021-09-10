.. _ROS2BagOwnNode:

Recording a bag from your own node
==================================

**Goal:** Record data from your own C++ node to a bag.

**Tutorial level:** Advanced

**Time:** 20 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

``rosbag2`` doesn't just provide the ``ros2 bag`` command line tool.
It also provides a C++ API for reading from and writing to a bag from your own source code.
This allows you to subscribe to a topic and save the received data to a bag at the same time as performing any other processing of your choice on that data.

Prerequisites
-------------

You should have the ``rosbag2`` packages installed as part of your regular ROS 2 setup.

If you've installed from Debian packages on Linux, it may be installed by default.
If it is not, you can install it using this command.

.. code-block:: console

  sudo apt install ros-{DISTRO}-rosbag2

This tutorial discusses using ROS 2 bags, including from the terminal.
You should have already completed the :ref:`basic ROS 2 bag tutorial <ROS2Bag>`.

Tasks
-----

1 Create a package
^^^^^^^^^^^^^^^^^^

Open a new terminal and :ref:`source your ROS 2 installation <ConfigROS2>` so that ``ros2`` commands will work.

Navigate into the ``dev_ws`` directory created in a :ref:`previous tutorial <new-directory>`.
Navigate into the ``dev_ws/src`` directory and create a new package:

.. code-block:: console

  ros2 pkg create --build-type ament_cmake bag_recorder_nodes --dependencies rclcpp rosbag2_cpp example_interfaces

Your terminal will return a message verifying the creation of your package ``bag_recorder_nodes`` and all its necessary files and folders.
The ``--dependencies`` argument will automatically add the necessary dependency lines to ``package.xml`` and ``CMakeLists.txt``.
In this case, the package will use the ``rosbag2_cpp`` package as well as the ``rclcpp`` package.
A dependency on the ``example_interfaces`` package is also required for later parts of this tutorial.

1.1 Update ``package.xml``
~~~~~~~~~~~~~~~~~~~~~~~~~~

Because you used the ``--dependencies`` option during package creation, you don't have to manually add dependencies to ``package.xml`` or ``CMakeLists.txt``.
As always, though, make sure to add the description, maintainer email and name, and license information to ``package.xml``.

.. code-block:: xml

  <description>C++ bag writing tutorial</description>
  <maintainer email="you@email.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

2 Write the C++ node
^^^^^^^^^^^^^^^^^^^^

Inside the ``dev_ws/src/bag_recorder_nodes/src`` directory, create a new file called ``simple_bag_recorder.cpp`` and paste the following code into it.

.. code-block:: C++


    #include <rclcpp/rclcpp.hpp>
    #include <example_interfaces/msg/string.hpp>

    #include <rosbag2_cpp/writer.hpp>

    using std::placeholders::_1;

    class SimpleBagRecorder : public rclcpp::Node
    {
    public:
      SimpleBagRecorder()
      : Node("simple_bag_recorder")
      {
        writer_ = std::make_unique<rosbag2_cpp::Writer>();

        writer_->open("my_bag");

        subscription_ = create_subscription<example_interfaces::msg::String>(
          "chatter", 10, std::bind(&SimpleBagRecorder::topic_callback, this, _1));
      }

    private:
      void topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
      {
        rclcpp::Time time_stamp = this->now();

        writer_->write(*msg, "chatter", "example_interfaces/msg/String", time_stamp);
      }

      rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr subscription_;
      std::unique_ptr<rosbag2_cpp::Writer> writer_;
    };

    int main(int argc, char * argv[])
    {
      rclcpp::init(argc, argv);
      rclcpp::spin(std::make_shared<SimpleBagRecorder>());
      rclcpp::shutdown();
      return 0;
    }

2.1 Examine the code
~~~~~~~~~~~~~~~~~~~~

The ``#include`` statements at the top are the package dependencies.
Note the inclusion of headers from the ``rosbag2_cpp`` package for the functions and structures necessary to work with bag files.

In the class constructor we begin by creating the writer object we will use to write to the bag.

.. code-block:: C++

        writer_ = std::make_unique<rosbag2_cpp::Writer>();

Now that we have a writer object, we can open the bag using it.
We specify just the URI of the bag to create, leaving other options at their defaults.
The default storage options are used, which means that an ``sqlite3``-format bag will be created.
The default conversion options are used, too, which will perform no conversion, instead storing messages in the serialisation format they are received in.

.. code-block:: C++

        writer_->open("my_bag");

With the writer now set up to record data we pass to it, we create a subscription and specify a callback for it.
We will write data to the bag in the callback.

.. code-block:: C++

        subscription_ = create_subscription<example_interfaces::msg::String>(
          "chatter", 10, std::bind(&SimpleBagRecorder::topic_callback, this, _1));

The callback itself is different from a typical callback.
Rather than receiving an instance of the data type of the topic, we instead receive a ``rclcpp::SerializedMessage``.
We do this for two reasons.

1. The message data will need to be serialised by ``rosbag2`` before being written to the bag, so rather than unserialising it when receiving the data and then re-serialising it, we ask ROS to just give us the serialised message as-is.
2. The writer API can accept a serialised message.

.. code-block:: C++

      void topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
      {

Within the subscription callback, the first thing to do is determine the time stamp to use for the stored message.
This can be anything appropriate to your data, but two common values are the time at which the data was produced, if known, and the time it is received.
The second option, the time of reception, is used here.

.. code-block:: C++

        rclcpp::Time time_stamp = this->now();

We can then write the message into the bag.
Because we have not yet registered any topics with the bag, we must specify the full topic information with the message.
This is why we pass in the topic name and the topic type.

.. code-block:: C++

        writer_->write(*msg, "chatter", "example_interfaces/msg/String", time_stamp);

The class contains two member variables.

1. The subscription object.
   Note that the template parameter is the type of the callback, not the type of the topic.
   In this case the callback receives a ``rclcpp::SerializedMessage`` shared pointer, so this is what the template parameter must be.
2. A managed pointer to the writer object used to write to the bag.
   Note the type of writer used here is the ``rosbag2_cpp::Writer``, the generic writer interface.
   Other writers may be available with different behaviours.

.. code-block:: C++

      rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr subscription_;
      std::unique_ptr<rosbag2_cpp::Writer> writer_;

The file finishes with the ``main`` function used to create an instance of the node and start ROS processing it.

.. code-block:: C++

    int main(int argc, char * argv[])
    {
      rclcpp::init(argc, argv);
      rclcpp::spin(std::make_shared<SimpleBagRecorder>());
      rclcpp::shutdown();
      return 0;
    }

2.2 Add executable
~~~~~~~~~~~~~~~~~~

Now open the ``CMakeLists.txt`` file.

Near the top of the file, change ``CMAKE_CXX_STANDARD`` from ``14`` to ``17``.

.. code-block:: console

    # Default to C++17
    if(NOT CMAKE_CXX_STANDARD)
      set(CMAKE_CXX_STANDARD 17)
    endif()

Below the dependencies block, which contains ``find_package(rosbag2_cpp REQUIRED)``, add the following lines of code.

.. code-block:: console

    add_executable(simple_bag_recorder src/simple_bag_recorder.cpp)
    ament_target_dependencies(simple_bag_recorder rclcpp rosbag2_cpp)

    install(TARGETS
      simple_bag_recorder
      DESTINATION lib/${PROJECT_NAME}
    )

3 Build and run
^^^^^^^^^^^^^^^

Navigate back to the root of your workspace, ``dev_ws``, and build your new package.

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      colcon build --packages-select bag_recorder_nodes

  .. group-tab:: macOS

    .. code-block:: console

      colcon build --packages-select bag_recorder_nodes

  .. group-tab:: Windows

    .. code-block:: console

      colcon build --merge-install --packages-select bag_recorder_nodes

Open a new terminal, navigate to ``dev_ws``, and source the setup files.

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

Now run the node:

.. code-block:: console

    ros2 run bag_recorder_nodes simple_bag_recorder

Open a second terminal and run the ``talker`` example node.

.. code-block:: console

    ros2 run demo_nodes_cpp talker

This will start publishing data on the ``chatter`` topic.
As the bag-writing node receives this data, it will write it to the ``my_bag`` bag.

Terminate both nodes.
Then, in one terminal start the ``listener`` example node.

.. code-block:: console

    ros2 run demo_nodes_cpp listener

In the other terminal, use ``ros2 bag`` to play the bag recorded by your node.

.. code-block:: console

    ros2 bag play my_bag

You will see the messages from the bag being received by the ``listener`` node.

If you wish to run the bag-writing node again, you will first need to delete the ``my_bag`` directory.

4 Record synthetic data from a node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Any data can be recorded into a bag, not just data received over a topic.
A common use case for writing to a bag from your own node is to generate and store synthetic data.
In this section you will learn how to write a node that generates some data and stores it in a bag.
We will demonstrate two approaches for doing this.
The first uses a node with a timer; this is the approach that you would use if your data generation is external to the node, such as reading data directly from hardware (e.g. a camera).
The second approach does not use a node; this is the approach you can use when you do not need to use any functionality from the ROS infrastructure.

4.1 Write a C++ node
~~~~~~~~~~~~~~~~~~~~

Inside the ``dev_ws/src/bag_recorder_nodes/src`` directory, create a new file called ``data_generator_node.cpp`` and paste the following code into it.

.. code-block:: C++

    #include <chrono>

    #include <example_interfaces/msg/int32.hpp>
    #include <rclcpp/rclcpp.hpp>

    #include <rosbag2_cpp/writer.hpp>

    using namespace std::chrono_literals;

    class DataGenerator : public rclcpp::Node
    {
    public:
      DataGenerator()
      : Node("data_generator")
      {
        data.data = 0;
        writer_ = std::make_unique<rosbag2_cpp::Writer>();

        writer_->open("timed_synthetic_bag");

        writer_->create_topic(
          {"synthetic",
           "example_interfaces/msg/Int32",
           rmw_get_serialization_format(),
           ""});

        timer_ = create_wall_timer(1s, std::bind(&DataGenerator::timer_callback, this));
      }

    private:
      void timer_callback()
      {
        writer_->write(data, "synthetic", "example_interfaces/msg/Int32", now());

        ++data.data;
      }

      rclcpp::TimerBase::SharedPtr timer_;
      std::unique_ptr<rosbag2_cpp::Writer> writer_;
      example_interfaces::msg::Int32 data;
    };

    int main(int argc, char * argv[])
    {
      rclcpp::init(argc, argv);
      rclcpp::spin(std::make_shared<DataGenerator>());
      rclcpp::shutdown();
      return 0;
    }

4.2 Examine the code
~~~~~~~~~~~~~~~~~~~~

Much of this code is the same as the first example.
The important differences are described here.

First, the name of the bag is changed.

.. code-block:: C++

        writer_->open("timed_synthetic_bag");

In this example we are registering the topic with the bag in advance.
This is optional in most cases, but it must be done when passing in a serialised message without topic information.

.. code-block:: C++

        writer_->create_topic(
          {"synthetic",
           "example_interfaces/msg/Int32",
           rmw_get_serialization_format(),
           ""});

Rather than a subscription to a topic, this node has a timer.
The timer fires with a one-second period, and calls the given member function when it does.

.. code-block:: C++

        timer_ = create_wall_timer(1s, std::bind(&DataGenerator::timer_callback, this));

Within the timer callback, we generate (or otherwise obtain, e.g. read from a serial port connected to some hardware) the data we wish to store in the bag.
The important difference between this and the previous sample is that the data is not yet serialised.
Instead we are passing to the writer object a ROS message data type, in this case an instance of ``example_interfaces/msg/Int32``.
The writer will serialise the data for us before writing it into the bag.

.. code-block:: C++

        writer_->write(data, "synthetic", now());

4.3 Add executable
~~~~~~~~~~~~~~~~~~

Open the ``CMakeLists.txt`` file and add the following lines after the previously-added lines (specifically, after the ``install(TARGETS ...)`` macro call).

.. code-block:: console

    add_executable(data_generator_node src/data_generator_node.cpp)
    ament_target_dependencies(data_generator_node rclcpp rosbag2_cpp example_interfaces)

    install(TARGETS
      data_generator_node
      DESTINATION lib/${PROJECT_NAME}
    )

4.4 Build and run
~~~~~~~~~~~~~~~~~

Navigate back to the root of your workspace, ``dev_ws``, and build your package.

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      colcon build --packages-select bag_recorder_nodes

  .. group-tab:: macOS

    .. code-block:: console

      colcon build --packages-select bag_recorder_nodes

  .. group-tab:: Windows

    .. code-block:: console

      colcon build --merge-install --packages-select bag_recorder_nodes

Open a new terminal, navigate to ``dev_ws``, and source the setup files.

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

(If the ``timed_synthetic_bag`` directory already exists, you must first delete it before running the node.)

Now run the node:

.. code-block:: console

    ros2 run bag_recorder_nodes data_generator_node

Wait for 30 seconds or so, then terminate the node with ``ctrl-c``.
Next, play back the created bag.

.. code-block:: console

    ros2 bag play timed_synthetic_bag

Open a second terminal and echo the ``/synthetic`` topic.

.. code-block:: console

    ros2 topic echo /synthetic

You will see the data that was generated and stored in the bag printed to the console at a rate of one message per second.

5 Record synthetic data from an executable
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Now that you can create a bag that stores data from a source other than a topic, you will learn how to generate and record synthetic data from a non-node executable.
The advantage of this approach is simpler code and rapid creation of a large quantity of data.

5.1 Write a C++ executable
~~~~~~~~~~~~~~~~~~~~~~~~~~

Inside the ``dev_ws/src/bag_recorder_nodes/src`` directory, create a new file called ``data_generator_executable.cpp`` and paste the following code into it.

.. code-block:: C++

    #include <chrono>

    #include <rclcpp/rclcpp.hpp>  // For rclcpp::Clock, rclcpp::Duration and rclcpp::Time
    #include <example_interfaces/msg/int32.hpp>

    #include <rosbag2_cpp/writer.hpp>
    #include <rosbag2_cpp/writers/sequential_writer.hpp>
    #include <rosbag2_storage/serialized_bag_message.hpp>

    using namespace std::chrono_literals;

    int main(int, char**)
    {
      example_interfaces::msg::Int32 data;
      data.data = 0;
      std::unique_ptr<rosbag2_cpp::Writer> writer_ = std::make_unique<rosbag2_cpp::Writer>();

      writer_->open("big_synthetic_bag");

      writer_->create_topic(
        {"synthetic",
         "example_interfaces/msg/Int32",
         rmw_get_serialization_format(),
         ""});

      rclcpp::Clock clock;
      rclcpp::Time time_stamp = clock.now();
      for (int32_t ii = 0; ii < 100; ++ii) {
        writer_->write(data, "synthetic", time_stamp);
        ++data.data;
        time_stamp += rclcpp::Duration(1s);
      }

      return 0;
    }

5.2 Examine the code
~~~~~~~~~~~~~~~~~~~~

A comparison of this sample and the previous sample will reveal that they are not that different.
The only significant difference is the use of a for loop to drive the data generation rather than a timer.

Notice that we are also now generating time stamps for the data rather than relying on the current system time for each sample.
The time stamp can be any value you need it to be.
The data will be played back at the rate given by these time stamps, so this is a useful way to control the default playback speed of the samples.
Notice also that while the gap between each sample is a full second in time, this executable does not need to wait a second between each sample.
This allows us to generate a lot of data covering a wide span of time in much less time than playback will take.

.. code-block:: C++

      rclcpp::Clock clock;
      rclcpp::Time time_stamp = clock.now();
      for (int32_t ii = 0; ii < 100; ++ii) {
        writer_->write(data, "synthetic", time_stamp);
        ++data.data;
        time_stamp += rclcpp::Duration(1s);
      }

5.3 Add executable
~~~~~~~~~~~~~~~~~~

Open the ``CMakeLists.txt`` file and add the following lines after the previously-added lines.

.. code-block:: console

    add_executable(data_generator_executable src/data_generator_executable.cpp)
    ament_target_dependencies(data_generator_executable rclcpp rosbag2_cpp example_interfaces)

    install(TARGETS
      data_generator_executable
      DESTINATION lib/${PROJECT_NAME}
    )

5.4 Build and run
~~~~~~~~~~~~~~~~~

Navigate back to the root of your workspace, ``dev_ws``, and build your package.

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      colcon build --packages-select bag_recorder_nodes

  .. group-tab:: macOS

    .. code-block:: console

      colcon build --packages-select bag_recorder_nodes

  .. group-tab:: Windows

    .. code-block:: console

      colcon build --merge-install --packages-select bag_recorder_nodes

Open a terminal, navigate to ``dev_ws``, and source the setup files.

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

(If the ``big_synthetic_bag`` directory already exists, you must first delete it before running the executable.)

Now run the executable:

.. code-block:: console

    ros2 run bag_recorder_nodes data_generator_executable

Note that the executable runs and finishes very quickly.

Now play back the created bag.

.. code-block:: console

    ros2 bag play big_synthetic_bag

Open a second terminal and echo the ``/synthetic`` topic.

.. code-block:: console

    ros2 topic echo /synthetic

You will see the data that was generated and stored in the bag printed to the console at a rate of one message per second.
Even though the bag was generated rapidly it is still played back at the rate the time stamps indicate.

Summary
-------

You created a node that records data it receives on a topic into a bag.
You tested recording a bag using the node, and verified the data was recorded by playing back the bag.
You then went on to create a node and an executable to generate synthetic data and store it in a bag.
