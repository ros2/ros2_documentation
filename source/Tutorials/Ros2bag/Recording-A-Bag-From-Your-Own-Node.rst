.. _ROS2BagOwnNode:

Recording a bag from your own node
==================================

**Goal:** Record data from your own node to a bag file.

**Tutorial level:** Advanced

**Time:** 20 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

``rosbag2`` doesn't just provide the ``ros2 bag`` command line tool.
It also provides an API for reading from and writing to a bag file from your own source code.
This allows you to subscribe to a topic and save the received data to a bag file at the same time as performing any other processing of your choice on that data.

Prerequisites
-------------

You should have the ``rosbag2`` packages installed as part of your regular ROS 2 setup.

If you've installed from Debian packages on Linux, it may be installed by default.
If it is not, you can install it using this command.

.. code-block:: console

  sudo apt install ros-foxy-rosbag2

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

  ros2 pkg create --build-type ament_cmake bag_recorder_node --dependencies rclcpp,rosbag2_cpp

Your terminal will return a message verifying the creation of your package ``bag_recorder_node`` and all its necessary files and folders.
The ``--dependencies`` argument will automatically add the necessary dependency lines to ``package.xml`` and ``CMakeLists.txt``.
In this case, the package will use the ``rosbag2_cpp`` package as well as the ``rclcpp`` package.

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

Inside the ``dev_ws/src/bag_recorder_node/src`` directory, create a new file called ``simple_bag_recorder.cpp`` and paste the following code into it.

.. code-block:: C++


    #include <functional>
    #include <memory>

    #include "rclcpp/rclcpp.hpp"
    #include "std_msgs/msg/string.hpp"

    #include <rosbag2_cpp/typesupport_helpers.hpp>
    #include <rosbag2_cpp/writer.hpp>
    #include <rosbag2_cpp/writers/sequential_writer.hpp>
    #include <rosbag2_storage/serialized_bag_message.hpp>

    using std::placeholders::_1;

    class SimpleBagRecorder : public rclcpp::Node
    {
    public:
      SimpleBagRecorder()
      : Node("minimal_subscriber")
      {
        rosbag2_cpp::StorageOptions storage_options({"my_bag", "sqlite3"});
        rosbag2_cpp::ConverterOptions converter_options({rmw_get_serialization_format(), rmw_get_serialization_format()});
        writer_ = std::make_shared<rosbag2_cpp::writers::SequentialWriter>();

        writer_->open(storage_options, converter_options);

        writer_->create_topic({"chatter", "std_msgs/msg/String", rmw_get_serialization_format(), ""});

        subscription_ = create_subscription<std_msgs::msg::String>(
          "chatter", 10, std::bind(&SimpleBagRecorder::topic_callback, this, _1));
      }

    private:
      void topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
      {
        auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();

        bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
          new rcutils_uint8_array_t,
          [this](rcutils_uint8_array_t *msg) {
            auto fini_return = rcutils_uint8_array_fini(msg);
            delete msg;
            if (fini_return != RCUTILS_RET_OK) {
              RCLCPP_ERROR(get_logger(),
                "Failed to destroy serialized message %s", rcutils_get_error_string().str);
            }
          });
        *bag_message->serialized_data = msg->release_rcl_serialized_message();

        bag_message->topic_name = "chatter";
        if (rcutils_system_time_now(&bag_message->time_stamp) != RCUTILS_RET_OK) {
          RCLCPP_ERROR(get_logger(), "Error getting current time: %s",
            rcutils_get_error_string().str);
        }

        writer_->write(bag_message);
      }

      rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr subscription_;
      std::shared_ptr<rosbag2_cpp::writers::SequentialWriter> writer_;
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
We must provide the storage options for the bag.
These specify the name (``my_bag``) and format (``sqlite3``) of the bag.
We must also provide conversion options, which specify how data input into the writer will be serialised and how that same data should be serialised when written to the bag file.
In most cases you can leave these as the same value as no conversion is neccessary.
We use the ``rmw_get_serialization_format()`` function to retrieve the serialisation format used by the underlying middleware, as this is what received data will be serialised as.

Finally, the third line creates the writer object.
We create a ``SequentialWriter``, which is the simplest writer object.
It does not perform advanced operations like compressing the data as it is written to the bag.

.. code-block:: C++

        rosbag2_cpp::StorageOptions storage_options({"my_bag", "sqlite3"});
        rosbag2_cpp::ConverterOptions converter_options({rmw_get_serialization_format(), rmw_get_serialization_format()});
        writer_ = std::make_shared<rosbag2_cpp::writers::SequentialWriter>();

Now that we have a writer object, we can open the bag using it.

.. code-block:: C++

        writer_->open(storage_options, converter_options);

The next step is to inform the writer of each topic that we will write to the bag file.
This is done by calling ``create_topic`` and passing in an instance of the ``rosbag2_storage::TopicMetadata`` structure.
Here we are using modern C++ syntax to construct an instance of this structure in place rather than creating it separately and passing it in.
The arguments stored in the ``rosbag2_storage::TopicMetadata`` structure are:

- The name of the topic.
  Note that this does not need to be the same as the topic the data is received on.
- The type of data in the topic.
  This **must** be the same as the type of data being stored in the bag.
- The serialisation format of the data.
  As before, we simply use the same serialisation format as the underlying middleware.
- Any QoS settings to specify for the topic.
  These must be specified in YAML format.
  Leaving it as an empty string will use the system defaults.

.. code-block:: C++

        writer_->create_topic({"chatter", "std_msgs/msg/String", rmw_get_serialization_format(), ""});

With the writer now set up to record data we pass to it, we create a subscription and specify a callback for it.
We will write data to the bag in the callback.

.. code-block:: C++

        subscription_ = create_subscription<std_msgs::msg::String>(
          "chatter", 10, std::bind(&SimpleBagRecorder::topic_callback, this, _1));

The callback itself is different from a typical callback.
Rather than receiving an instance of the data type of the topic, we instead receive a ``rclcpp::SerializedMessage``.
We do this for two reasons.

1. The message data will need to be serialised before being written to the bag, so rather than unserialising it when receiving the data and then re-serialising it, we ask ROS to just give us the serialised message as-is.
2. The writer API requires a serialised message, so by asking for a serialised message from ROS we save ourselves the effort of serialising the data ourselves.

.. code-block:: C++

      void topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
      {

Within the subscription callback, the first thing to do is to create an instance of ``rosbag2_storage::SerializedBagMessage``.
This is the data type that represents a single data sample in a bag.

.. code-block:: C++

        auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();

The next block of code is somewhat complex due to the memory management requirements of working directly with serialised data from the middleware.
The memory of the serialised data is owned by the ``SerializedMessage`` object, but the ``SerializedBagMessage`` object we will pas to the bag must own the memory.
Otherwise the memory might go out of scope and be deleted before it is written to the bag, causing a memory access error.
To prevent this, we call ``release_rcl_serialized_message()`` on the ``SerializedMessage`` object.
This causes it to release its ownership of the memory, allowing the ``SerializedBagMessage`` to take ownership.

However we also need to ensure that the ``SerializedBagMessage`` object will delete the memory properly when it is cleaned up.
This is achieved by providing a custom deleter function when creating the ``serialized_data`` member of the ``bag_message`` instance.
This is the purpose of the lambda function being passed into the constructor of the ``std::shared_ptr<rcutils_uint8_array_t>`` object (i.e. the ``serialized_data`` member of the ``bag_message`` object).
When the ``shared_ptr`` goes out of scope, the lambda function will be called and will clean up the memory.

.. code-block:: C++

        bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
          new rcutils_uint8_array_t,
          [this](rcutils_uint8_array_t *msg) {
            auto fini_return = rcutils_uint8_array_fini(msg);
            delete msg;
            if (fini_return != RCUTILS_RET_OK) {
              RCLCPP_ERROR(get_logger(),
                "Failed to destroy serialized message %s", rcutils_get_error_string().str);
            }
          });
        *bag_message->serialized_data = msg->release_rcl_serialized_message();

The next line is used to tell the writer what topic this sample is for.
This is necessary as the serialised message contains no type information and could, by error, be written to any topic in the bag.

.. code-block:: C++

        bag_message->topic_name = "chatter";

The time stamp of the message must also be set in the ``time_stamp`` member of the ``bag_message`` objecT.
This can be anything appropriate to your data, but two common values are the time at which the data was produced, if known, and the time it is received.
The second option, the time of reception, is used here.

.. code-block:: C++

        if (rcutils_system_time_now(&bag_message->time_stamp) != RCUTILS_RET_OK) {
          RCLCPP_ERROR(get_logger(), "Error getting current time: %s",
            rcutils_get_error_string().str);
        }

The final step in the callback is to pass the data to the writer object so it can be written to the bag.

.. code-block:: C++

        writer_->write(bag_message);

The class contains two member variables.

1. The subscription object.
   Note that the template parameter is the type of the callback, not the type of the topic.
   In this case the callback receives a ``rclcpp::SerializedMessage`` shared pointer, so this is what the template parameter must be.
2. A shared pointer to the writer object used to write to the bag.
   Note the type of writer used here is the ``rosbag2_cpp::writers::SequentialWriter``.
   Other writers may be available with different behaviours.

.. code-block:: C++

      rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr subscription_;
      std::shared_ptr<rosbag2_cpp::writers::SequentialWriter> writer_;

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
Below the dependency ``find_package(rosbag2_cpp REQUIRED)`` add the following lines of code.

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

      colcon build --packages-select bag_recorder_node

  .. group-tab:: macOS

    .. code-block:: console

      colcon build --packages-select bag_recorder_node

  .. group-tab:: Windows

    .. code-block:: console

      colcon build --merge-install --packages-select bag_recorder_node

Open a new terminal, navigate to ``dev_ws``, and source the setup files.

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      . install/setup.bash

  .. group-tab:: macOS

    .. code-block:: console

      . install/setup.bash

  .. group-tab:: Windows

    .. code-block:: console

      call install/setup.bat

Now run the node:

.. code-block:: console

    ros2 run bag_recorder_node simple_bag_recorder

Open a second terminal and run the ``talker`` example node.

.. code-block:: console

    ros2 run demo_nodes_cpp talker

This will start publishing data on the ``chatter`` topic.
As the bag-writing node receives this data, it will write it to the ``my_bag`` bag.

Terminate both nodes.
Then, in one terminal start the ``listener`` example node.

.. code-block:: console

    ros2 run demo_nodes_cpp listener

In the other terminal, use ``ros2 bag`` to play the bag file recorded by your node.

.. code-block:: console

    ros2 bag play my_bag

You will see the messages from the bag file being received by the ``listener`` node.

Summary
-------

You created a node that records data it receives on a topic into a bag.
You tested recording a bag using the node, and verified the data was recorded by playing back the bag file.
