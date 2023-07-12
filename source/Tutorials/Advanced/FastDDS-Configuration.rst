.. redirect-from::

    FastDDS-Configuration
    Tutorials/FastDDS-Configuration/FastDDS-Configuration

Unlocking the potential of Fast DDS middleware [community-contributed]
======================================================================

**Goal:** This tutorial will show how to use the extended configuration capabilities of Fast DDS in ROS 2.

**Tutorial level:** Advanced

**Time:** 20 minutes

.. contents:: Table of Contents
   :depth: 2
   :local:

Background
----------

The interface between the ROS 2 stack and *Fast DDS* is provided by the ROS 2 middleware implementation `rmw_fastrtps <https://github.com/ros2/rmw_fastrtps>`_.
This implementation is available in all ROS 2 distributions, both from binaries and from sources.

ROS 2 RMW only allows for the configuration of certain middleware QoS
(see :doc:`ROS 2 QoS policies <../../Concepts/Intermediate/About-Quality-of-Service-Settings>`).
However, ``rmw_fastrtps`` offers extended configuration capabilities to take full advantage of the features in *Fast DDS*.
This tutorial will guide you through a series of examples explaining how to use XML files to unlock this extended configuration.

In order to get more information about using *Fast DDS* on ROS 2, please check the `following documentation <https://fast-dds.docs.eprosima.com/en/latest/fastdds/ros2/ros2.html>`__.


Prerequisites
-------------

This tutorial assumes that you know how to :doc:`create a package <../Beginner-Client-Libraries/Creating-Your-First-ROS2-Package>`.
It also assumes you know how to write a :doc:`simple publisher and subscriber<../Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber>` and a :doc:`simple service and client <../Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client>`.
Although the examples are implemented in C++, the same concepts apply to Python packages.


Mixing synchronous and asynchronous publications in the same node
-----------------------------------------------------------------

In this first example, a node with two publishers, one of them with synchronous publication mode and the other one with asynchronous publication mode, will be created.

``rmw_fastrtps`` uses synchronous publication mode by default.

With synchronous publication mode the data is sent directly within the context of the user thread.
This entails that any blocking call occurring during the write operation would block the user thread, thus preventing the application from continuing its operation.
However, this mode typically yields higher throughput rates at lower latencies, since there is no notification nor context switching between threads.

On the other hand, with asynchronous publication mode, each time the publisher invokes the write operation, the data is copied into a queue,
a background thread (asynchronous thread) is notified about the addition to the queue, and control of the thread is returned to the user before the data is actually sent.
The background thread is in charge of consuming the queue and sending the data to every matched reader.

Create the node with the publishers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

First, create a new package named ``sync_async_node_example_cpp`` on a new workspace:

.. tabs::

    .. group-tab:: Linux

       .. code-block:: console

         mkdir -p ~/ros2_ws/src
         cd ~/ros2_ws/src
         ros2 pkg create --build-type ament_cmake --dependencies rclcpp std_msgs -- sync_async_node_example_cpp

    .. group-tab:: macOS

      .. code-block:: console

        mkdir -p ~/ros2_ws/src
        cd ~/ros2_ws/src
        ros2 pkg create --build-type ament_cmake --dependencies rclcpp std_msgs -- sync_async_node_example_cpp

    .. group-tab:: Windows

      .. code-block:: console

        md \ros2_ws\src
        cd \ros2_ws\src
        ros2 pkg create --build-type ament_cmake --dependencies rclcpp std_msgs -- sync_async_node_example_cpp


Then, add a file named ``src/sync_async_writer.cpp`` to the package, with the following content.
Note that the synchronous publisher will be publishing on topic ``sync_topic``, while the asynchronous one will be publishing on topic ``async_topic``.

.. code-block:: C++

    #include <chrono>
    #include <functional>
    #include <memory>
    #include <string>

    #include "rclcpp/rclcpp.hpp"
    #include "std_msgs/msg/string.hpp"

    using namespace std::chrono_literals;

    class SyncAsyncPublisher : public rclcpp::Node
    {
    public:
        SyncAsyncPublisher()
            : Node("sync_async_publisher"), count_(0)
        {
            // Create the synchronous publisher on topic 'sync_topic'
            sync_publisher_ = this->create_publisher<std_msgs::msg::String>("sync_topic", 10);

            // Create the asynchronous publisher on topic 'async_topic'
            async_publisher_ = this->create_publisher<std_msgs::msg::String>("async_topic", 10);

            // This timer will trigger the publication of new data every half a second
            timer_ = this->create_wall_timer(
                    500ms, std::bind(&SyncAsyncPublisher::timer_callback, this));
        }

    private:
        /**
         * Actions to run every time the timer expires
         */
        void timer_callback()
        {
            // Create a new message to be sent
            auto sync_message = std_msgs::msg::String();
            sync_message.data = "SYNC: Hello, world! " + std::to_string(count_);

            // Log the message to the console to show progress
            RCLCPP_INFO(this->get_logger(), "Synchronously publishing: '%s'", sync_message.data.c_str());

            // Publish the message using the synchronous publisher
            sync_publisher_->publish(sync_message);

            // Create a new message to be sent
            auto async_message = std_msgs::msg::String();
            async_message.data = "ASYNC: Hello, world! " + std::to_string(count_);

            // Log the message to the console to show progress
            RCLCPP_INFO(this->get_logger(), "Asynchronously publishing: '%s'", async_message.data.c_str());

            // Publish the message using the asynchronous publisher
            async_publisher_->publish(async_message);

            // Prepare the count for the next message
            count_++;
        }

        // This timer will trigger the publication of new data every half a second
        rclcpp::TimerBase::SharedPtr timer_;

        // A publisher that publishes asynchronously
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr async_publisher_;

        // A publisher that publishes synchronously
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sync_publisher_;

        // Number of messages sent so far
        size_t count_;
    };

    int main(int argc, char * argv[])
    {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<SyncAsyncPublisher>());
        rclcpp::shutdown();
        return 0;
    }

Now open the ``CMakeLists.txt`` file and add a new executable and name it ``SyncAsyncWriter`` so you can run your node using ``ros2 run``:

.. code-block:: cmake

    add_executable(SyncAsyncWriter src/sync_async_writer.cpp)
    ament_target_dependencies(SyncAsyncWriter rclcpp std_msgs)

Finally, add the ``install(TARGETS…)`` section so ``ros2 run`` can find your executable:

.. code-block:: cmake

    install(TARGETS
        SyncAsyncWriter
        DESTINATION lib/${PROJECT_NAME})

You can clean up your ``CMakeLists.txt`` by removing some unnecessary sections and comments, so it looks like this:

.. code-block:: cmake

    cmake_minimum_required(VERSION 3.8)
    project(sync_async_node_example_cpp)

    # Default to C++14
    if(NOT CMAKE_CXX_STANDARD)
      set(CMAKE_CXX_STANDARD 14)
    endif()

    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      add_compile_options(-Wall -Wextra -Wpedantic)
    endif()

    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)

    add_executable(SyncAsyncWriter src/sync_async_writer.cpp)
    ament_target_dependencies(SyncAsyncWriter rclcpp std_msgs)

    install(TARGETS
        SyncAsyncWriter
        DESTINATION lib/${PROJECT_NAME})

    ament_package()

If this node is built and run now, both publishers will behave the same, publishing asynchronously in both topics, because this is the default publication mode.
The default publication mode configuration can be changed in runtime during the node launching, using an XML file.

Create the XML file with the profile configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Create a file with name ``SyncAsync.xml`` and the following content:

.. code-block:: XML

    <?xml version="1.0" encoding="UTF-8" ?>
    <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">

        <!-- default publisher profile -->
        <publisher profile_name="default_publisher" is_default_profile="true">
            <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
        </publisher>

        <!-- default subscriber profile -->
        <subscriber profile_name="default_subscriber" is_default_profile="true">
            <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
        </subscriber>

        <!-- publisher profile for topic sync_topic -->
        <publisher profile_name="/sync_topic">
            <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
            <qos>
                <publishMode>
                    <kind>SYNCHRONOUS</kind>
                </publishMode>
            </qos>
        </publisher>

        <!-- publisher profile for topic async_topic -->
        <publisher profile_name="/async_topic">
            <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
            <qos>
                <publishMode>
                    <kind>ASYNCHRONOUS</kind>
                </publishMode>
            </qos>
        </publisher>

     </profiles>

Note that several profiles for publisher and subscriber are defined.
Two default profiles which are defined setting the ``is_default_profile`` to ``true``, and two profiles with names that coincide with those of the previously defined topics: ``sync_topic`` and another one for ``async_topic``.
These last two profiles set the publication mode to ``SYNCHRONOUS`` or ``ASYNCHRONOUS`` accordingly.
Note also that all profiles specify a ``historyMemoryPolicy`` value, which is needed for the example to work, and the reason will be explained later on this tutorial.

Execute the publisher node
^^^^^^^^^^^^^^^^^^^^^^^^^^

You will need to export the following environment variables for the XML to be loaded:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
      export RMW_FASTRTPS_USE_QOS_FROM_XML=1
      export FASTRTPS_DEFAULT_PROFILES_FILE=path/to/SyncAsync.xml

  .. group-tab:: macOS

    .. code-block:: console

      export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
      export RMW_FASTRTPS_USE_QOS_FROM_XML=1
      export FASTRTPS_DEFAULT_PROFILES_FILE=path/to/SyncAsync.xml

  .. group-tab:: Windows

    .. code-block:: console

      SET RMW_IMPLEMENTATION=rmw_fastrtps_cpp
      SET RMW_FASTRTPS_USE_QOS_FROM_XML=1
      SET FASTRTPS_DEFAULT_PROFILES_FILE=path/to/SyncAsync.xml

Finally, ensure you have sourced your setup files and run the node:

.. code-block:: console

    source install/setup.bash
    ros2 run sync_async_node_example_cpp SyncAsyncWriter

You should see the publishers sending the data from the publishing node, like so:

.. code-block:: console

    [INFO] [1612972049.994630332] [sync_async_publisher]: Synchronously publishing: 'SYNC: Hello, world! 0'
    [INFO] [1612972049.995097767] [sync_async_publisher]: Asynchronously publishing: 'ASYNC: Hello, world! 0'
    [INFO] [1612972050.494478706] [sync_async_publisher]: Synchronously publishing: 'SYNC: Hello, world! 1'
    [INFO] [1612972050.494664334] [sync_async_publisher]: Asynchronously publishing: 'ASYNC: Hello, world! 1'
    [INFO] [1612972050.994368474] [sync_async_publisher]: Synchronously publishing: 'SYNC: Hello, world! 2'
    [INFO] [1612972050.994549851] [sync_async_publisher]: Asynchronously publishing: 'ASYNC: Hello, world! 2'

Now you have a synchronous publisher and an asynchronous publisher running inside the same node.


Create a node with the subscribers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Next, a new node with the subscribers that will listen to the ``sync_topic`` and ``async_topic`` publications is going to be created.
In a new source file named ``src/sync_async_reader.cpp`` write the following content:

.. code-block:: C++

    #include <functional>
    #include <memory>

    #include "rclcpp/rclcpp.hpp"
    #include "std_msgs/msg/string.hpp"

    using std::placeholders::_1;

    class SyncAsyncSubscriber : public rclcpp::Node
    {
    public:

        SyncAsyncSubscriber()
            : Node("sync_async_subscriber")
        {
            // Create the synchronous subscriber on topic 'sync_topic'
            // and tie it to the topic_callback
            sync_subscription_ = this->create_subscription<std_msgs::msg::String>(
                "sync_topic", 10, std::bind(&SyncAsyncSubscriber::topic_callback, this, _1));

            // Create the asynchronous subscriber on topic 'async_topic'
            // and tie it to the topic_callback
            async_subscription_ = this->create_subscription<std_msgs::msg::String>(
                "async_topic", 10, std::bind(&SyncAsyncSubscriber::topic_callback, this, _1));
        }

    private:

        /**
         * Actions to run every time a new message is received
         */
        void topic_callback(const std_msgs::msg::String & msg) const
        {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
        }

        // A subscriber that listens to topic 'sync_topic'
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sync_subscription_;

        // A subscriber that listens to topic 'async_topic'
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr async_subscription_;
    };

    int main(int argc, char * argv[])
    {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<SyncAsyncSubscriber>());
        rclcpp::shutdown();
        return 0;
    }


Open the ``CMakeLists.txt`` file and add a new executable and name it ``SyncAsyncReader`` under the previous ``SyncAsyncWriter``:

.. code-block:: cmake

    add_executable(SyncAsyncReader src/sync_async_reader.cpp)
    ament_target_dependencies(SyncAsyncReader rclcpp std_msgs)

    install(TARGETS
        SyncAsyncReader
        DESTINATION lib/${PROJECT_NAME})


Execute the subscriber node
^^^^^^^^^^^^^^^^^^^^^^^^^^^

With the publisher node running in one terminal, open another one and export the required environment variables for the XML to be loaded:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
      export RMW_FASTRTPS_USE_QOS_FROM_XML=1
      export FASTRTPS_DEFAULT_PROFILES_FILE=path/to/SyncAsync.xml

  .. group-tab:: macOS

    .. code-block:: console

      export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
      export RMW_FASTRTPS_USE_QOS_FROM_XML=1
      export FASTRTPS_DEFAULT_PROFILES_FILE=path/to/SyncAsync.xml

  .. group-tab:: Windows

    .. code-block:: console

      SET RMW_IMPLEMENTATION=rmw_fastrtps_cpp
      SET RMW_FASTRTPS_USE_QOS_FROM_XML=1
      SET FASTRTPS_DEFAULT_PROFILES_FILE=path/to/SyncAsync.xml

Finally, ensure you have sourced your setup files and run the node:

.. code-block:: console

    source install/setup.bash
    ros2 run sync_async_node_example_cpp SyncAsyncReader

You should see the subscribers receiving the data from the publishing node, like so:

.. code-block:: console

    [INFO] [1612972054.495429090] [sync_async_subscriber]: I heard: 'SYNC: Hello, world! 10'
    [INFO] [1612972054.995410057] [sync_async_subscriber]: I heard: 'ASYNC: Hello, world! 10'
    [INFO] [1612972055.495453494] [sync_async_subscriber]: I heard: 'SYNC: Hello, world! 11'
    [INFO] [1612972055.995396561] [sync_async_subscriber]: I heard: 'ASYNC: Hello, world! 11'
    [INFO] [1612972056.495534818] [sync_async_subscriber]: I heard: 'SYNC: Hello, world! 12'
    [INFO] [1612972056.995473953] [sync_async_subscriber]: I heard: 'ASYNC: Hello, world! 12'


Analysis of the example
^^^^^^^^^^^^^^^^^^^^^^^

Configuration profiles XML
~~~~~~~~~~~~~~~~~~~~~~~~~~

The XML file defines several configurations for publishers and subscribers.
You can have a default publisher configuration profile and several topic-specific publisher profiles.
The only requirement is that all publisher profiles have a different name and that there is only a single default profile.
The same goes for subscribers.

In order to define a configuration for a specific topic, just name the profile after the the ROS 2 topic name (like ``/sync_topic`` and ``/async_topic`` in the example),
and ``rmw_fastrtps`` will apply this profile to all publishers and subscribers for that topic.
The default configuration profile is identified by the attribute ``is_default_profile`` set to ``true``, and acts as a fallback profile when there is no other one with a name matching the topic name.

The environment variable ``FASTRTPS_DEFAULT_PROFILES_FILE`` is used to inform *Fast DDS* the path to the XML file with the configuration profiles to load.

RMW_FASTRTPS_USE_QOS_FROM_XML
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Among all the configurable attributes, ``rmw_fastrtps`` treats ``publishMode`` and ``historyMemoryPolicy`` differently.
By default, these values are set to ``ASYNCHRONOUS`` and ``PREALLOCATED_WITH_REALLOC`` within the ``rmw_fastrtps`` implementation, and the values set on the XML file are ignored.
In order to use the values in the XML file, the environment variable ``RMW_FASTRTPS_USE_QOS_FROM_XML`` must be set to ``1``.

However, this entails **another caveat**: If ``RMW_FASTRTPS_USE_QOS_FROM_XML`` is set, but the XML file does not define
``publishMode`` or ``historyMemoryPolicy``, these attributes take the *Fast DDS* default value instead of the ``rmw_fastrtps`` default value.
This is important, especially for ``historyMemoryPolicy``, because the *Fast DDS* deafult value is ``PREALLOCATED`` which does not work with ROS2 topic data types.
Therefore, in the example, a valid value for this policy has been explicitly set (``DYNAMIC``).


Prioritization of rmw_qos_profile_t
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

ROS 2 QoS contained in `rmw_qos_profile_t <http://docs.ros2.org/latest/api/rmw/structrmw__qos__profile__t.html>`_ are always honored, unless set to ``*_SYSTEM_DEFAULT``.
In that case, XML values (or *Fast DDS* default values in the absence of XML ones) are applied.
This means that if any QoS in ``rmw_qos_profile_t`` is set to something other than ``*_SYSTEM_DEFAULT``, the corresponding value in the XML is ignored.


Using other FastDDS capabilities with XML
-----------------------------------------

Although we have created a node with two publishers with different configuration, it is not easy to check that they are behaving differently.
Now that the basics of XML profiles have been covered, let us use them to configure something which has some visual effect on the nodes.
Specifically, a maximum number of matching subscribers on one of the publishers and a partition definition on the other will be set.
Note that these are only very simple examples among all the configuration attributes that can be tuned on ``rmw_fastrtps`` through XML files.
Please refer to `*Fast DDS* documentation <https://fast-dds.docs.eprosima.com/en/latest/fastdds/xml_configuration/xml_configuration.html#xml-profiles>`__ to  see the whole list of attributes that can be configured through XML files.

Limiting the number of matching subscribers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Add a maximum number of matched subscribers to the ``/async_topic`` publisher profile.
It should look like this:

.. code-block:: XML

    <!-- publisher profile for topic async_topic -->
    <publisher profile_name="/async_topic">
        <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
        <qos>
            <publishMode>
                <kind>ASYNCHRONOUS</kind>
            </publishMode>
        </qos>
        <matchedSubscribersAllocation>
            <initial>0</initial>
            <maximum>1</maximum>
            <increment>1</increment>
        </matchedSubscribersAllocation>
    </publisher>

The number of matching subscribers is being limited to one.

Now open three terminals and do not forget to source the setup files and to set the required environment variables.
On the first terminal run the publisher node, and the subscriber node on the other two.
You should see that only the first subscriber node receives the messages from both topics.
The second one could not complete the matching process in the ``/async_topic`` because the publisher prevented it, as it had already reached its maximum of matched publishers.
Consequently, only the messages from the ``/sync_topic`` are going to be received in this third terminal:

.. code-block:: console

    [INFO] [1613127657.088860890] [sync_async_subscriber]: I heard: 'SYNC: Hello, world! 18'
    [INFO] [1613127657.588896594] [sync_async_subscriber]: I heard: 'SYNC: Hello, world! 19'
    [INFO] [1613127658.088849401] [sync_async_subscriber]: I heard: 'SYNC: Hello, world! 20'


Using partitions within the topic
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The partitions feature can be used to control which publishers and subscribers exchange information within the same topic.

Partitions introduce a logical entity isolation level concept inside the physical isolation induced by a Domain ID.
For a publisher to communicate with a subscriber, they have to belong at least to one common partition.
Partitions represent another level to separate publishers and subscribers beyond domain and topic.
Unlike domain and topic, an endpoint can belong to several partitions at the same time.
For certain data to be shared over different domains or topics, there must be a different publisher for each, sharing its own history of changes.
However, a single publisher can share the same data sample over different partitions using a single topic data change, thus reducing network overload.

Let us change the ``/sync_topic`` publisher to partition ``part1`` and create a new ``/sync_topic`` subscriber which uses partition ``part2``.
Their profiles should now look like this:

.. code-block:: XML

    <!-- publisher profile for topic sync_topic -->
    <publisher profile_name="/sync_topic">
        <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
        <qos>
            <publishMode>
                <kind>SYNCHRONOUS</kind>
            </publishMode>
            <partition>
                <names>
                    <name>part1</name>
                </names>
            </partition>
        </qos>
    </publisher>

    <!-- subscriber profile for topic sync_topic -->
    <subscriber profile_name="/sync_topic">
        <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
        <qos>
            <partition>
                <names>
                    <name>part2</name>
                </names>
            </partition>
        </qos>
    </subscriber>

Open two terminals.
Do not forget to source the setup files and to set the required environment variables.
On the first terminal run the publisher node, and the subscriber node on the other one.
You should see that only the ``/async_topic`` messages are reaching the subscriber.
The ``/sync_topic`` subscriber is not receiving the data as it is in a different partition from the corresponding publisher.

.. code-block:: console

    [INFO] [1612972054.995410057] [sync_async_subscriber]: I heard: 'ASYNC: Hello, world! 10'
    [INFO] [1612972055.995396561] [sync_async_subscriber]: I heard: 'ASYNC: Hello, world! 11'
    [INFO] [1612972056.995473953] [sync_async_subscriber]: I heard: 'ASYNC: Hello, world! 12'


Configuring a service and a client
----------------------------------

Services and clients have a publisher and a subscriber each, that communicate through two different topics.
For example, for a service named ``ping`` there is:

* A service subscriber listening to requests on ``/rq/ping``.
* A service publisher sending responses on ``/rr/ping``.
* A client publisher sending requests on ``/rq/ping``.
* A client subscriber listening to responses on ``/rr/ping``.

Although you can use these topic names to set the configuration profiles on the XML, sometimes you may wish to apply the same profile to all services or clients on a node.
Instead of copying the same profile with all topic names generated for all services, you can just create a publisher and subscriber profile pair named ``service``.
The same can be done for clients creating a pair named ``client``.


Create the nodes with the service and client
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Start creating the node with the service.
Add a new source file named ``src/ping_service.cpp`` on your package with the following content:

.. code-block:: C++

    #include <memory>

    #include "rclcpp/rclcpp.hpp"
    #include "example_interfaces/srv/trigger.hpp"

    /**
     * Service action: responds with success=true and prints the request on the console
     */
    void ping(const std::shared_ptr<example_interfaces::srv::Trigger::Request> request,
            std::shared_ptr<example_interfaces::srv::Trigger::Response> response)
    {
        // The request data is unused
        (void) request;

        // Build the response
        response->success = true;

        // Log to the console
        RCLCPP_INFO(rclcpp::get_logger("ping_server"), "Incoming request");
        RCLCPP_INFO(rclcpp::get_logger("ping_server"), "Sending back response");
    }

    int main(int argc, char **argv)
    {
        rclcpp::init(argc, argv);

        // Create the node and the service
        std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("ping_server");
        rclcpp::Service<example_interfaces::srv::Trigger>::SharedPtr service =
            node->create_service<example_interfaces::srv::Trigger>("ping", &ping);

        // Log that the service is ready
        RCLCPP_INFO(rclcpp::get_logger("ping_server"), "Ready to serve.");

        // run the node
        rclcpp::spin(node);
        rclcpp::shutdown();
    }

Create the client in a file named ``src/ping_client.cpp`` with the following content:

.. code-block:: C++

    #include <chrono>
    #include <memory>

    #include "rclcpp/rclcpp.hpp"
    #include "example_interfaces/srv/trigger.hpp"

    using namespace std::chrono_literals;

    int main(int argc, char **argv)
    {
        rclcpp::init(argc, argv);

        // Create the node and the client
        std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("ping_client");
        rclcpp::Client<example_interfaces::srv::Trigger>::SharedPtr client =
            node->create_client<example_interfaces::srv::Trigger>("ping");

        // Create a request
        auto request = std::make_shared<example_interfaces::srv::Trigger::Request>();

        // Wait for the service to be available
        while (!client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("ping_client"), "Interrupted while waiting for the service. Exiting.");
                return 0;
            }
            RCLCPP_INFO(rclcpp::get_logger("ping_client"), "Service not available, waiting again...");
        }

        // Now that the service is available, send the request
        RCLCPP_INFO(rclcpp::get_logger("ping_client"), "Sending request");
        auto result = client->async_send_request(request);

        // Wait for the result and log it to the console
        if (rclcpp::spin_until_future_complete(node, result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(rclcpp::get_logger("ping_client"), "Response received");
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("ping_client"), "Failed to call service ping");
        }

        rclcpp::shutdown();
        return 0;
    }

Open the ``CMakeLists.txt`` file and add two new executables ``ping_service`` and ``ping_client``:

.. code-block:: cmake

    find_package(example_interfaces REQUIRED)

    add_executable(ping_service src/ping_service.cpp)
    ament_target_dependencies(ping_service example_interfaces rclcpp)

    add_executable(ping_client src/ping_client.cpp)
    ament_target_dependencies(ping_client example_interfaces rclcpp)

    install(TARGETS
        ping_service
        DESTINATION lib/${PROJECT_NAME})

    install(TARGETS
        ping_client
        DESTINATION lib/${PROJECT_NAME})

Finally, build the package.


Create the XML profiles for the service and client
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Create a file with name ``ping.xml`` with the following content:

.. code-block:: XML

    <?xml version="1.0" encoding="UTF-8" ?>
    <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">

        <!-- default publisher profile -->
        <publisher profile_name="default_publisher" is_default_profile="true">
            <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
        </publisher>

        <!-- default subscriber profile -->
        <subscriber profile_name="default_subscriber" is_default_profile="true">
            <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
        </subscriber>

        <!-- service publisher is SYNC -->
        <publisher profile_name="service">
            <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
            <qos>
                <publishMode>
                    <kind>SYNCHRONOUS</kind>
                </publishMode>
            </qos>
        </publisher>

        <!-- client publisher is ASYNC -->
        <publisher profile_name="client">
            <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
            <qos>
                <publishMode>
                    <kind>ASYNCHRONOUS</kind>
                </publishMode>
            </qos>
        </publisher>

    </profiles>


This configuration file sets the publication mode to ``SYNCHRONOUS`` on the service and to ``ASYNCHRONOUS`` on the client.
Note that we are only defining the publisher profiles for both the service and the client, but subscriber profiles could be provided too.


Execute the nodes
^^^^^^^^^^^^^^^^^

Open two terminals and source the setup files on each one.
Then set the required environment variables for the XML to be loaded:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
      export RMW_FASTRTPS_USE_QOS_FROM_XML=1
      export FASTRTPS_DEFAULT_PROFILES_FILE=path/to/ping.xml

  .. group-tab:: macOS

    .. code-block:: console

      export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
      export RMW_FASTRTPS_USE_QOS_FROM_XML=1
      export FASTRTPS_DEFAULT_PROFILES_FILE=path/to/ping.xml

  .. group-tab:: Windows

    .. code-block:: console

      SET RMW_IMPLEMENTATION=rmw_fastrtps_cpp
      SET RMW_FASTRTPS_USE_QOS_FROM_XML=1
      SET FASTRTPS_DEFAULT_PROFILES_FILE=path/to/ping.xml


On the first terminal run the service node.

.. code-block:: console

    ros2 run sync_async_node_example_cpp ping_service

You should see the service waiting for requests:

.. code-block:: console

    [INFO] [1612977403.805799037] [ping_server]: Ready to serve

On the second terminal, run the client node.


.. code-block:: console

    ros2 run sync_async_node_example_cpp ping_client

You should see the client sending the request and receiving the response:

.. code-block:: console

    [INFO] [1612977404.805799037] [ping_client]: Sending request
    [INFO] [1612977404.825473835] [ping_client]: Response received

At the same time, the output in the server console has been updated:

.. code-block:: console

    [INFO] [1612977403.805799037] [ping_server]: Ready to serve
    [INFO] [1612977404.807314904] [ping_server]: Incoming request
    [INFO] [1612977404.836405125] [ping_server]: Sending back response
