.. _filtered_topic_keys_tutorial:

Topic Keys Subscription Filtering Tutorial
==========================================

This tutorial aims to demonstrate how to receive data only from certain topic instances by combining the use of topic keys and topic content filtering.

.. contents::
    :depth: 2
    :local:
    :backlinks: none

Background
----------

In ROS 2, topics are a mean for representing the state of an object.
Keyed topics are special topics where each data sample represent an update of the state of a specific object (known as *instance*) among all those objects represented in the topic.
This allows the user to reduce the number of required resources (topics, along with its associated publisher and subscriber) by multiplexing updates of several objects of the same kind into a single resource.

The :doc:`Content Filter Topic <../../Demos/Content-Filtering-Subscription>` facilitates efficient data distribution by allowing the subscription (reader-side) to specify criteria for the types of data they wish to receive.
By defining this criteria, irrelevant data can be filtered out and applications can focus only on the information that is relevant to their needs.
This functionality not only reduces the amount of data transmitted over the network but also minimizes processing overhead on the receiving end, leading to improved system performance and scalability.

When combined with topic instances, the benefits of the Content Filter are further enhanced.
By associating specific filter criteria with each topic instance, it is possible fine-tune the data selection process and tailor it to their precise requirements.
This granular level of filtering enables applications to optimally manage data while ensuring that they exchange only the information that is pertinent to their individual use cases.

.. image:: figures/keyed-topics-cft.gif
    :align: center
    :width: 70%

RMW Support
-----------

Keyed topics require RMW implementation support.

.. list-table::  Keyed Topics Support Status
   :widths: 25 25

   * - rmw_fastrtps
     - supported
   * - rmw_connextdds
     - supported
   * - rmw_cyclonedds
     - not supported

Implementations not supporting the feature will treat keyed topics as standard topics.
The implications are explained throughout the tutorial.
In addition, endpoints using `Cyclone DDS <https://index.ros.org/p/rmw_cyclonedds_cpp/>`_ will not even match with Fast DDS or Connext DDS endpoints for this kind of topics.

Prerequisites
-------------

* An up-to-date ROS 2 installation and setup.
  Either installed in local host, or using Docker images.

Preparing the demo package
--------------------------

Lets start by setting up the ROS 2 environment.
For this, there are two possible options:

#.  Running a ROS 2 Docker image.

    .. code-block:: console

        $ docker run -it --rm osrt/ros:{DISTRO}-desktop

#.  Running the tutorial on the local host.
    Please, follow the :doc:`installation instructions <../../../Installation>` for details on installing ROS 2.

Source the following file to setup the ROS 2 environment:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        $ source /opt/ros/{DISTRO}/setup.bash

      Replace ``.bash`` with your shell if you're not using bash.
      Possible values are: ``setup.bash``, ``setup.sh``, ``setup.zsh``.

   .. group-tab:: macOS

      .. code-block:: console

        $ . ~/ros2_install/ros2-osx/setup.bash

   .. group-tab:: Windows

      .. code-block:: console

        $ call C:\dev\ros2\local_setup.bat


Retrieving the sources
^^^^^^^^^^^^^^^^^^^^^^

Create a new workspace and download the demo package sources as indicated below:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        # Create directory structure
        $ mkdir -p ~/tutorial_ws/src/demo_keys_filtering_cpp
        $ mkdir ~/tutorial_ws/src/demo_keys_filtering_cpp/msg
        $ mkdir ~/tutorial_ws/src/demo_keys_filtering_cpp/src
        $ mkdir ~/tutorial_ws/src/demo_keys_filtering_cpp/launch
        $ cd ~/tutorial_ws/src/demo_keys_filtering_cpp

        # Download demo package source code
        $ wget -O CMakeLists.txt https://raw.githubusercontent.com/ros2/ros2_documentation/{DISTRO}/source/Tutorials/Advanced/Topic-Keys/resources/Filtered/CMakeLists.txt
        $ wget -O package.xml https://raw.githubusercontent.com/ros2/ros2_documentation/{DISTRO}/source/Tutorials/Advanced/Topic-Keys/resources/Filtered/package.xml
        $ wget -O README.md https://raw.githubusercontent.com/ros2/ros2_documentation/{DISTRO}/source/Tutorials/Advanced/Topic-Keys/resources/Filtered/README.md
        $ wget -O msg/KeyedSensorDataMsg.msg https://raw.githubusercontent.com/ros2/ros2_documentation/{DISTRO}/source/Tutorials/Advanced/Topic-Keys/resources/Filtered/msg/KeyedSensorDataMsg.msg
        $ wget -O src/filtered_keyed_sensor.cpp https://raw.githubusercontent.com/ros2/ros2_documentation/{DISTRO}/source/Tutorials/Advanced/Topic-Keys/resources/Filtered/src/filtered_keyed_sensor.cpp
        $ wget -O src/filtered_keyed_controller.cpp https://raw.githubusercontent.com/ros2/ros2_documentation/{DISTRO}/source/Tutorials/Advanced/Topic-Keys/resources/Filtered/src/filtered_keyed_controller.cpp
        $ wget -O launch/keyed_sensors_launch.py https://raw.githubusercontent.com/ros2/ros2_documentation/{DISTRO}/source/Tutorials/Advanced/Topic-Keys/resources/Filtered/launch/keyed_sensors_launch.py

   .. group-tab:: macOS

      .. code-block:: console

        # Create directory structure
        $ mkdir -p ~/tutorial_ws/src/demo_keys_filtering_cpp
        $ mkdir ~/tutorial_ws/src/demo_keys_filtering_cpp/msg
        $ mkdir ~/tutorial_ws/src/demo_keys_filtering_cpp/src
        $ mkdir ~/tutorial_ws/src/demo_keys_filtering_cpp/launch
        $ cd ~/tutorial_ws/src/demo_keys_filtering_cpp

        # Download demo package source code
        $ wget -O CMakeLists.txt https://raw.githubusercontent.com/ros2/ros2_documentation/{DISTRO}/source/Tutorials/Advanced/Topic-Keys/resources/Filtered/CMakeLists.txt
        $ wget -O package.xml https://raw.githubusercontent.com/ros2/ros2_documentation/{DISTRO}/source/Tutorials/Advanced/Topic-Keys/resources/Filtered/package.xml
        $ wget -O README.md https://raw.githubusercontent.com/ros2/ros2_documentation/{DISTRO}/source/Tutorials/Advanced/Topic-Keys/resources/Filtered/README.md
        $ wget -O msg/KeyedSensorDataMsg.msg https://raw.githubusercontent.com/ros2/ros2_documentation/{DISTRO}/source/Tutorials/Advanced/Topic-Keys/resources/Filtered/msg/KeyedSensorDataMsg.msg
        $ wget -O src/filtered_keyed_sensor.cpp https://raw.githubusercontent.com/ros2/ros2_documentation/{DISTRO}/source/Tutorials/Advanced/Topic-Keys/resources/Filtered/src/filtered_keyed_sensor.cpp
        $ wget -O src/filtered_keyed_controller.cpp https://raw.githubusercontent.com/ros2/ros2_documentation/{DISTRO}/source/Tutorials/Advanced/Topic-Keys/resources/Filtered/src/filtered_keyed_controller.cpp
        $ wget -O launch/keyed_sensors_launch.py https://raw.githubusercontent.com/ros2/ros2_documentation/{DISTRO}/source/Tutorials/Advanced/Topic-Keys/resources/Filtered/launch/keyed_sensors_launch.py

   .. group-tab:: Windows

      .. code-block:: console

        # Create directory structure
        $ mkdir C:\tutorial_ws\src\demo_keys_filtering_cpp\msg
        $ mkdir C:\tutorial_ws\src\demo_keys_filtering_cpp\src
        $ mkdir C:\tutorial_ws\src\demo_keys_filtering_cpp\launch
        $ cd C:\tutorial_ws\src\demo_keys_filtering_cpp

        # Download demo package source code
        $ irm -OutFile CMakeLists.txt https://raw.githubusercontent.com/ros2/ros2_documentation/{DISTRO}/source/Tutorials/Advanced/Topic-Keys/resources/Filtered/CMakeLists.txt
        $ irm -OutFile package.xml https://raw.githubusercontent.com/ros2/ros2_documentation/{DISTRO}/source/Tutorials/Advanced/Topic-Keys/resources/Filtered/package.xml
        $ irm -OutFile README.md https://raw.githubusercontent.com/ros2/ros2_documentation/{DISTRO}/source/Tutorials/Advanced/Topic-Keys/resources/Filtered/README.md
        $ irm -OutFile msg/KeyedSensorDataMsg.msg https://raw.githubusercontent.com/ros2/ros2_documentation/{DISTRO}/source/Tutorials/Advanced/Topic-Keys/resources/Filtered/msg/KeyedSensorDataMsg.msg
        $ irm -OutFile src/filtered_keyed_sensor.cpp https://raw.githubusercontent.com/ros2/ros2_documentation/{DISTRO}/source/Tutorials/Advanced/Topic-Keys/resources/Filtered/src/filtered_keyed_sensor.cpp
        $ irm -OutFile src/filtered_keyed_controller.cpp https://raw.githubusercontent.com/ros2/ros2_documentation/{DISTRO}/source/Tutorials/Advanced/Topic-Keys/resources/Filtered/src/filtered_keyed_controller.cpp
        $ irm -OutFile launch/keyed_sensors_launch.py https://raw.githubusercontent.com/ros2/ros2_documentation/{DISTRO}/source/Tutorials/Advanced/Topic-Keys/resources/Filtered/launch/keyed_sensors_launch.py

The resulting directory structure should be:

.. code-block::

    ~/tutorial_ws
     ├──src
        ├── demo_keys_filtering_cpp
            ├── CMakeLists.txt
            ├── README.md
            ├── launch
            │   └── keyed_sensors_launch.py
            ├── msg
            │   └── KeyedSensorDataMsg.msg
            ├── package.xml
            └── src
                ├── filtered_keyed_controller.cpp
                └── filtered_keyed_sensor.cpp

A brief analysis on the provided files is explained below:

* *demo_keys_filtering_cpp* : This directory contains the main source code and configuration files for the demonstration.
* ``CMakeLists.txt``: This file is used with CMake to specify build configurations and dependencies.
* ``README.md``: This is a markdown file providing instructions or information about the demonstration.
* *launch*: This directory contains launch configuration files for launching ROS nodes.

  * ``keyed_sensors_launch.py``: This Python script is used to launch the demonstration nodes.

* *msg*: This directory contains message definition files.

  * KeyedSensorDataMsg.msg: This is the file defining the message structure for keyed sensor data used in the tutorial.

* ``package.xml``: This is an XML file containing metadata about the ROS package.
* *src*: This directory contains the source code files for the demonstration.

  * ``filtered_keyed_controller.cpp``: This is the source code for a controller node that filters keyed sensor data in reception, being the most relevant lines the ones that define the filter expression and Quality of Service settings:

  .. code-block:: cpp

    // Initialize a subscription with a content filter to receive data from sensors 2 to 4
    rclcpp::SubscriptionOptions sub_options;
    sub_options.content_filter_options.filter_expression =
        "sensor_id >= 2 AND sensor_id <= 4 AND measurement > %0";
    sub_options.content_filter_options.expression_parameters = {
    std::to_string(SENSOR_TRIGGER)
    };

    // Create the subscription with the content filter options
    sub_ = create_subscription<demo_keys_filtering_cpp::msg::KeyedSensorDataMsg>("/robot/sensors",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
        callback,
        sub_options);

  * ``filtered_keyed_sensor.cpp``: This is the source code for a sensor node that publishes keyed sensor data.
    The most relevant lines are the ones that create the publication with a particular Quality of Service settings that enables the controller to late join the application but still receiving the latest update for every instance with the use of topic keys.

  .. code-block:: cpp

    pub_ = this->create_publisher<demo_keys_filtering_cpp::msg::KeyedSensorDataMsg>(
        "/robot/sensors",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

Generating the IDL files
^^^^^^^^^^^^^^^^^^^^^^^^

Generate the corresponding IDL definition from the provided ``KeyedSensorDataMsg.msg`` file, using the ``msg2idl.py`` script from the ``rosidl_adapter`` package.

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        $ cd ~/tutorial_ws/src/demo_keys_filtering_cpp/msg
        $ ros2 run rosidl_adapter msg2idl.py KeyedSensorDataMsg.msg
        $ rm KeyedSensorDataMsg.msg

   .. group-tab:: macOS

      .. code-block:: console

        $ cd ~/tutorial_ws/src/demo_keys_filtering_cpp/msg
        $ ros2 run rosidl_adapter msg2idl.py KeyedSensorDataMsg.msg
        $ rm KeyedSensorDataMsg.msg

   .. group-tab:: Windows

      .. code-block:: console

        $ cd C:\tutorial_ws\src\demo_keys_filtering_cpp\msg
        $ ros2 run rosidl_adapter msg2idl.py KeyedSensorDataMsg.msg
        $ del KeyedSensorDataMsg.msg

Next, annotate the ``sensor_id`` field as ``@key`` in the generated ``KeyedSensorDataMsg.idl``.
Its content should look like the following:

.. code-block:: idl

    /* KeyedSensorDataMsg.idl */
    module demo_keys_filtering_cpp {
      module msg {
        struct KeyedSensorDataMsg {
          @key int16 sensor_id;
          double measurement;
          string data;
        };
      };
    };

Building the demo package
^^^^^^^^^^^^^^^^^^^^^^^^^

Once the environment has been setup and the demo package sources are available, the demo package can be built.
Get into the root of the workspace and build it with the following commands:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        $ cd ~/tutorial_ws
        $ colcon build

   .. group-tab:: macOS

      .. code-block:: console

        $ cd ~/tutorial_ws
        $ colcon build

   .. group-tab:: Windows

      .. code-block:: console

        $ cd C:\tutorial_ws
        $ colcon build


Running the demo
----------------

In the demo, different sensors are publishing data to a controller node using a keyed topic as exemplified below:

.. image:: figures/cft_tutorial_diagram.svg
    :align: center
    :width: 80%

Run the demo by executing the following commands in separate terminals:

.. note::

    If a docker deployment was preferred, it would be necessary to attach the other two terminals to the running docker container before executing the above commands.
    This can be done by running ``docker exec -it <container_name> /bin/bash``.

.. tabs::

    .. group-tab:: Linux

        .. tabs::

            .. tab:: Shell 1 (Sensors)

                .. code-block:: console

                    $ source ~/tutorial_ws/install/setup.bash
                    $ ros2 launch demo_keys_filtering_cpp keyed_sensors_launch.py

            .. tab:: Shell 2 (Controller)

                .. code-block:: console

                    $ source ~/tutorial_ws/install/setup.bash
                    $ ros2 run demo_keys_filtering_cpp filtered_keyed_controller

    .. group-tab:: macOS

        .. tabs::

            .. tab:: Shell 1 (Sensors)

                .. code-block:: console

                    $ source ~/tutorial_ws/install/setup.bash
                    $ ros2 launch demo_keys_filtering_cpp keyed_sensors_launch.py

            .. tab:: Shell 2 (Controller)

                .. code-block:: console

                    $ source ~/tutorial_ws/install/setup.bash
                    $ ros2 run demo_keys_filtering_cpp filtered_keyed_controller

    .. group-tab:: Windows

        .. tabs::

            .. tab:: Shell 1 (Sensors)

                .. code-block:: console

                    $ call C:\tutorial_ws\install\setup.bat
                    $ ros2 launch demo_keys_filtering_cpp keyed_sensors_launch.py

            .. tab:: Shell 2 (Controller)

                .. code-block:: console

                    $ call C:\tutorial_ws\install\setup.bat
                    $ ros2 run demo_keys_filtering_cpp filtered_keyed_controller

The resulting output should be similar to the following, in which the controller node is only receiving data from the specified sensors, i.e. sensors which sensor_id is in the range [2, 4].
In addition, only when the measurement is greater than 60, the controller node will receive data.
That is specified in the filter expression ``sensor_id >= 2 AND sensor_id <= 4 AND measurement > %0``:

.. image:: figures/filtered_keyed_topic.gif
    :align: center
    :width: 100%

Even in a late-joining scenario, the controller node will receive the latest data of the sensors that meet the filtering criteria at the moment it joins the application, which could be crucial depending on the type of the real application.

Overall, the combination of topic instances with a content filter topic offers significant benefits in terms of data efficiency, scalability, adaptability and resource optimization.
By leveraging these capabilities, ROS 2 applications can efficiently manage and distribute data in complex distributed environments.
