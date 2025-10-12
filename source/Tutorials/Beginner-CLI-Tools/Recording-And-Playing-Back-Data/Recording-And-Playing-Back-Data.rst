.. redirect-from::

    Tutorials/Ros2bag/Recording-And-Playing-Back-Data

.. _ROS2Bag:

Recording and playing back data
===============================

**Goal:** Record data published on a topic, a service and an action so you can replay and examine it any time.

**Tutorial level:** Beginner

**Time:** 20 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

``ros2 bag`` is a command line tool for recording data published on topics, services and actions in your ROS 2 system.
It accumulates the data passed on any number of topics, services and actions, then saves it in a database.
You can then replay the data to reproduce the results of your tests and experiments.
Recording topics, services and actions is also a great way to share your work and allow others to recreate it.


Prerequisites
-------------

You should have ``ros2 bag`` installed as a part of your regular ROS 2 setup.

If you need to install ROS 2, see the :doc:`Installation instructions <../../../Installation>`.

This tutorial talks about concepts covered in previous tutorials, like :doc:`nodes <../Understanding-ROS2-Nodes/Understanding-ROS2-Nodes>`, :doc:`topics <../Understanding-ROS2-Topics/Understanding-ROS2-Topics>`, :doc:`services <../Understanding-ROS2-Services/Understanding-ROS2-Services>` and :doc:`actions <../Understanding-ROS2-Actions/Understanding-ROS2-Actions>`.
It also uses the :doc:`turtlesim package <../Introducing-Turtlesim/Introducing-Turtlesim>`, :doc:`Service Introspection Demo <../../Demos/Service-Introspection>` and :doc:`Action Introspection Demo <../../Demos/Action-Introspection>`.

As always, don't forget to source ROS 2 in :doc:`every new terminal you open <../Configuring-ROS2-Environment>`.


Managing Topic Data
-------------------

1 Setup
^^^^^^^

You'll be recording your keyboard input in the ``turtlesim`` system to save and replay later on, so begin by starting up the ``/turtlesim`` and ``/teleop_turtle`` nodes.

Open a new terminal and run:

.. code-block:: console

    $ ros2 run turtlesim turtlesim_node

Open another terminal and run:

.. code-block:: console

    $ ros2 run turtlesim turtle_teleop_key

Let's also make a new directory to store our saved recordings, just as good practice:

.. tabs::

    .. group-tab:: Linux

        .. code-block:: console

            $ mkdir bag_files
            $ cd bag_files

    .. group-tab:: macOS

        .. code-block:: console

            $ mkdir bag_files
            $ cd bag_files

    .. group-tab:: Windows

        .. code-block:: console

            $ md bag_files
            $ cd bag_files


2 Choose a topic
^^^^^^^^^^^^^^^^

``ros2 bag`` can record data from messages published to topics.
To see the list of your system's topics, open a new terminal and run the command:

.. code-block:: console

  $ ros2 topic list
  /parameter_events
  /rosout
  /turtle1/cmd_vel
  /turtle1/color_sensor
  /turtle1/pose

In the topics tutorial, you learned that the ``/turtle_teleop`` node publishes commands on the ``/turtle1/cmd_vel`` topic to make the turtle move in turtlesim.

To see the data that ``/turtle1/cmd_vel`` is publishing, run the command:

.. code-block:: console

    $ ros2 topic echo /turtle1/cmd_vel

Nothing will show up at first because no data is being published by the teleop.
Return to the terminal where you ran the teleop and select it so it's active.
Use the arrow keys to move the turtle around, and you will see data being published on the terminal running ``ros2 topic echo``.

.. code-block:: console

  linear:
    x: 2.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0
    ---


3 Record topics
^^^^^^^^^^^^^^^

3.1 Record a single topic
~~~~~~~~~~~~~~~~~~~~~~~~~

To record the data published to a topic use the command syntax:

.. code-block:: console

    $ ros2 bag record --topics <topic_name>

Before running this command on your chosen topic, open a new terminal and move into the ``bag_files`` directory you created earlier, because the rosbag file will save in the directory where you run it.

Run the command:

.. code-block:: console

    $ ros2 bag record --topics /turtle1/cmd_vel
    [INFO] [rosbag2_storage]: Opened database 'rosbag2_2019_10_11-05_18_45'.
    [INFO] [rosbag2_transport]: Listening for topics...
    [INFO] [rosbag2_transport]: Subscribed to topic '/turtle1/cmd_vel'
    [INFO] [rosbag2_transport]: All requested topics are subscribed. Stopping discovery...

Now ``ros2 bag`` is recording the data published on the ``/turtle1/cmd_vel`` topic.
Return to the teleop terminal and move the turtle around again.
The movements don't matter, but try to make a recognizable pattern to see when you replay the data later.

.. image:: images/record.png

Press :kbd:`Ctrl-C` to stop recording.

The data will be accumulated in a new bag directory with a name in the pattern of ``rosbag2_year_month_day-hour_minute_second``.
This directory will contain a ``metadata.yaml`` along with the bag file in the recorded format.

3.2 Record multiple topics
~~~~~~~~~~~~~~~~~~~~~~~~~~

You can also record multiple topics, as well as change the name of the bag directory ``ros2 bag`` saves to.

Run the following command:

.. code-block:: console

  $ ros2 bag record -o subset --topics /turtle1/cmd_vel /turtle1/pose
  [INFO] [rosbag2_storage]: Opened database 'subset'.
  [INFO] [rosbag2_transport]: Listening for topics...
  [INFO] [rosbag2_transport]: Subscribed to topic '/turtle1/cmd_vel'
  [INFO] [rosbag2_transport]: Subscribed to topic '/turtle1/pose'
  [INFO] [rosbag2_transport]: All requested topics are subscribed. Stopping discovery...

The ``-o`` option allows you to choose a unique name for your bag directory.
The following string, in this case ``subset``, is the bag directory name.

To record more than one topic at a time, simply list each topic separated by a space after ``--topics``.
In this case, the command output above confirms that both topics are being recorded.

You can move the turtle around and press :kbd:`Ctrl-C` when you're finished.

.. note::

    There is another option you can add to the command, ``-a``, which records all the topics on your system.

3.3 Split recording into multiple files
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can also split your recording into multiple files, based on either recording duration or file size.
``-d <max_bag_duration>`` ensures that each file only lasts ``<max_bag_duration>`` seconds before it starts writing to a new file, or ``-b <max_bag_size>`` ensures that each file does not exceed ``<max_bag_size>`` bytes in file size.
This prevents large and unwieldy file sizes, and protects against losing all data if the recording operation becomes corrupted at some point.

Run the following for at least 15 seconds, allowing for three 5-second bag files to be written:

.. code-block:: console

    $ ros2 bag record -o subset_split -d 5 --topics /turtle1/cmd_vel /turtle1/pose
    [INFO] [rosbag2_recorder]: Press SPACE for pausing/resuming
    [INFO] [rosbag2_recorder]: Listening for topics...
    [INFO] [rosbag2_recorder]: Event publisher thread: Starting
    [INFO] [rosbag2_recorder]: Recording...
    [INFO] [rosbag2_recorder]: Subscribed to topic '/turtle1/cmd_vel'
    [INFO] [rosbag2_recorder]: Subscribed to topic '/turtle1/pose'
    [INFO] [rosbag2_recorder]: All requested topics are subscribed. Stopping discovery...
    [INFO] [rosbag2_cpp]: Writing remaining messages from cache to the bag. It may take a while
    [INFO] [rosbag2_cpp]: Writing remaining messages from cache to the bag. It may take a while
    [INFO] [rosbag2_cpp]: Writing remaining messages from cache to the bag. It may take a while

Press :kbd:`Ctrl-C` when you're finished.
You should find a ``subset_split`` directory with these files inside: ``subset_split_0.mcap``, ``subset_split_1.mcap``, and so on.

4 Inspect topic data
^^^^^^^^^^^^^^^^^^^^

You can see details about your recording by running:

.. code-block:: console

    $ ros2 bag info <bag_name>

Running this command on the ``subset`` bag recording will return a list of information:

.. code-block:: console

    $ ros2 bag info subset
    Files:             subset_0.mcap
    Bag size:          228.5 KiB
    Storage id:        mcap
    Duration:          48.47s
    Start:             Oct 11 2019 06:09:09.12 (1570799349.12)
    End                Oct 11 2019 06:09:57.60 (1570799397.60)
    Messages:          3013
    Topic information: Topic: /turtle1/cmd_vel | Type: geometry_msgs/msg/Twist | Count: 9 | Serialization Format: cdr
                       Topic: /turtle1/pose | Type: turtlesim_msgs/msg/Pose | Count: 3004 | Serialization Format: cdr
    Services:          0
    Service information:
    Actions:           0
    Action information:

Alternatively, you can also call ``ros2 bag info`` on an individual file, such as ``subset_split/subset_split_0.mcap``, and it will only show information for that portion of the recording; in this case, the first 5 seconds.

5 Play topic data
^^^^^^^^^^^^^^^^^

5.1 Play a single bag
~~~~~~~~~~~~~~~~~~~~~

Before replaying the bag, enter :kbd:`Ctrl-C` in the terminal where the teleop is running.
Then make sure your turtlesim window is visible so you can see the bag file in action.

Enter the command:

.. code-block:: console

    $ ros2 bag play subset
    [INFO] [rosbag2_player]: Set rate to 1
    [INFO] [rosbag2_player]: Adding keyboard callbacks.
    [INFO] [rosbag2_player]: Press SPACE for Pause/Resume
    [INFO] [rosbag2_player]: Press CURSOR_RIGHT for Play Next Message
    [INFO] [rosbag2_player]: Press CURSOR_UP for Increase Rate 10%
    [INFO] [rosbag2_player]: Press CURSOR_DOWN for Decrease Rate 10%
    Progress bar enabled at 3 Hz.
    Progress bar [?]: [R]unning, [P]aused, [B]urst, [D]elayed, [S]topped
    [INFO] [rosbag2_player]: Playback until timestamp: -1


    ====== Playback Progress ======
    [1751923361.427372456] Duration 0.00/48.47 [R]

Your turtle will follow the same path you entered while recording (though not 100% exactly; turtlesim is sensitive to small changes in the system's timing).

.. image:: images/playback.png

Because the ``subset`` file recorded the ``/turtle1/pose`` topic, the ``ros2 bag play`` command won't quit for as long as you had turtlesim running, even if you weren't moving.

This is because as long as the ``/turtlesim`` node is active, it publishes data on the  ``/turtle1/pose`` topic at regular intervals.
You may have noticed in the ``ros2 bag info`` example result above that the  ``/turtle1/cmd_vel`` topic's ``Count`` information was only 9; that's how many times we pressed the arrow keys while recording.

Notice that ``/turtle1/pose`` has a ``Count`` value of over 3000; while we were recording, data was published on that topic 3000 times.

To get an idea of how often position data is published, you can run the command:

.. code-block:: console

    $ ros2 topic hz /turtle1/pose

5.2 Play multiple bags
~~~~~~~~~~~~~~~~~~~~~~

At times, it is relevant to split the desired recorded topics amongst multiple recordings, as a way to distribute the recording workload.
As an example, we can record ``/turtle1/cmd_vel`` and ``/turtle1/pose`` each to their own bag.

Create two terminal instances.
In the first one, run the following:

.. code-block:: console

    $ ros2 bag record -o subset_cmd_vel --topics /turtle1/cmd_vel

In the second terminal, run this:

.. code-block:: console

    $ ros2 bag record -o subset_pose --topics /turtle1/pose

Move the turtle around as you did before, then end both recordings with :kbd:`Ctrl-C` when finished.

To have these two recordings play in parallel with correct timing, call ``ros2 bag play`` with ``-i <bag_name>`` for each bag you want to include.
In this case, run:

.. code-block:: console

    $ ros2 bag play -i subset_cmd_vel -i subset_pose

This will play the ``subset_cmd_vel`` and ``subset_pose`` recordings together, with the playback synced to replicate the original order of messages.
If used, the optional argument ``--message-order {received,sent}`` determines whether the messages are sequenced according to the time they were received or published (defaults to received).
This applies to playing a single bag as well.

Managing Service Data
---------------------

1 Setup
^^^^^^^

You'll be recording service data between ``introspection_client`` and ``introspection_service``, then display and replay that same data later on.
To record service data between service client and server, ``Service Introspection`` must be enabled on the node.

Let's start ``introspection_client`` and ``introspection_service`` nodes and enable ``Service Introspection``.
You can see more details for :doc:`Service Introspection Demo <../../Demos/Service-Introspection>`.

Open a new terminal and run ``introspection_service``, enabling ``Service Introspection``:

.. code-block:: console

    $ ros2 run demo_nodes_cpp introspection_service --ros-args -p service_configure_introspection:=contents

Open another terminal and run ``introspection_client``, enabling ``Service Introspection``:

.. code-block:: console

    $ ros2 run demo_nodes_cpp introspection_client --ros-args -p client_configure_introspection:=contents

2 Check service availability
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``ros2 bag`` can only record data from available services.
To see the list of your system's services, open a new terminal and run the command:

.. code-block:: console

  $ ros2 service list
  /add_two_ints
  /introspection_client/describe_parameters
  /introspection_client/get_parameter_types
  /introspection_client/get_parameters
  /introspection_client/get_type_description
  /introspection_client/list_parameters
  /introspection_client/set_parameters
  /introspection_client/set_parameters_atomically
  /introspection_service/describe_parameters
  /introspection_service/get_parameter_types
  /introspection_service/get_parameters
  /introspection_service/get_type_description
  /introspection_service/list_parameters
  /introspection_service/set_parameters
  /introspection_service/set_parameters_atomically

To check if ``Service Introspection`` is enabled on the client and service, run the command:

.. code-block:: console

  $ ros2 service echo --flow-style /add_two_ints
  info:
    event_type: REQUEST_SENT
    stamp:
      sec: 1713995389
      nanosec: 386809259
    client_gid: [1, 15, 96, 219, 162, 1, 108, 201, 0, 0, 0, 0, 0, 0, 21, 3]
    sequence_number: 133
  request: [{a: 2, b: 3}]
  response: []
  ---

You should see the service communication.

3 Record services
^^^^^^^^^^^^^^^^^

To record service data, the following options are supported.
Service data can be recorded with topics at the same time.

To record specific services:

.. code-block:: console

  $ ros2 bag record --service <service_names>

To record all services:

.. code-block:: console

  $ ros2 bag record --all-services

Run the command:

.. code-block:: console

  $ ros2 bag record --service /add_two_ints
  [INFO] [1713995957.643573503] [rosbag2_recorder]: Press SPACE for pausing/resuming
  [INFO] [1713995957.662067587] [rosbag2_recorder]: Event publisher thread: Starting
  [INFO] [1713995957.662067614] [rosbag2_recorder]: Listening for topics...
  [INFO] [1713995957.666048323] [rosbag2_recorder]: Subscribed to topic '/add_two_ints/_service_event'
  [INFO] [1713995957.666092458] [rosbag2_recorder]: Recording...

Now ``ros2 bag`` is recording the service data published on the ``/add_two_ints`` service.
To stop the recording, enter :kbd:`Ctrl-C` in the terminal.

The data will be accumulated in a new bag directory with a name in the pattern of ``rosbag2_year_month_day-hour_minute_second``.
This directory will contain a ``metadata.yaml`` along with the bag file in the recorded format.

4 Inspect service data
^^^^^^^^^^^^^^^^^^^^^^

You can see details about your recording by running:

.. code-block:: console

  $ ros2 bag info <bag_file_name>
  Files:             rosbag2_2024_04_24-14_59_17_0.mcap
  Bag size:          15.1 KiB
  Storage id:        mcap
  ROS Distro:        rolling
  Duration:          9.211s
  Start:             Apr 24 2024 14:59:17.676 (1713995957.676)
  End:               Apr 24 2024 14:59:26.888 (1713995966.888)
  Messages:          0
  Topic information:
  Service:           1
  Service information: Service: /add_two_ints | Type: example_interfaces/srv/AddTwoInts | Event Count: 78 | Serialization Format: cdr

5 Play service data
^^^^^^^^^^^^^^^^^^^

Before replaying the bag file, enter :kbd:`Ctrl-C` in the terminal where ``introspection_client`` is running.
When ``introspection_client`` stops running, ``introspection_service`` also stops printing the result because there are no incoming requests.

Replaying the service data from the bag file will start sending the requests to ``introspection_service``.

Enter the command:

.. code-block:: console

  $ ros2 bag play --publish-service-requests <bag_file_name>
  [INFO] [1713997477.870856190] [rosbag2_player]: Set rate to 1
  [INFO] [1713997477.877417477] [rosbag2_player]: Adding keyboard callbacks.
  [INFO] [1713997477.877442404] [rosbag2_player]: Press SPACE for Pause/Resume
  [INFO] [1713997477.877447855] [rosbag2_player]: Press CURSOR_RIGHT for Play Next Message
  [INFO] [1713997477.877452655] [rosbag2_player]: Press CURSOR_UP for Increase Rate 10%
  [INFO] [1713997477.877456954] [rosbag2_player]: Press CURSOR_DOWN for Decrease Rate 10%
  [INFO] [1713997477.877573647] [rosbag2_player]: Playback until timestamp: -1

Your ``introspection_service`` terminal will once again start printing the following service messages:

.. code-block:: console

  [INFO] [1713997478.090466075] [introspection_service]: Incoming request
  a: 2 b: 3

This is because ``ros2 bag play`` sends the service request data from the bag file to the ``/add_two_ints`` service.

We can also introspect service communication as ``ros2 bag play`` is playing it back to verify the ``introspection_service``.

Run this command before ``ros2 bag play`` to see the ``introspection_service``:

.. code-block:: console

  $ ros2 service echo --flow-style /add_two_ints

You can see the service request from the bag file and the service response from  ``introspection_service``.

.. code-block:: console

  info:
    event_type: REQUEST_RECEIVED
    stamp:
      sec: 1713998176
      nanosec: 372700698
    client_gid: [1, 15, 96, 219, 80, 2, 158, 123, 0, 0, 0, 0, 0, 0, 20, 4]
    sequence_number: 1
  request: [{a: 2, b: 3}]
  response: []
  ---
  info:
    event_type: RESPONSE_SENT
    stamp:
      sec: 1713998176
      nanosec: 373016882
    client_gid: [1, 15, 96, 219, 80, 2, 158, 123, 0, 0, 0, 0, 0, 0, 20, 4]
    sequence_number: 1
  request: []
  response: [{sum: 5}]

.. _record-play-data-action:

Managing Action Data
--------------------

1 Setup
^^^^^^^

You'll be recording action data between ``fibonacci_action_client`` and ``fibonacci_action_server``, then display and replay that same data later on.
To record action data between action client and server, ``Action Introspection`` must be enabled on the nodes.

Let's start ``fibonacci_action_client`` and ``fibonacci_action_server`` nodes and enable ``Action Introspection``.
You can see more details for :doc:`Action Introspection Demo <../../Demos/Action-Introspection>`.

Open a new terminal and run ``fibonacci_action_server``, enabling ``Action Introspection``:

.. code-block:: console

  $ ros2 run action_tutorials_py fibonacci_action_server --ros-args -p action_server_configure_introspection:=contents

Open another terminal and run ``fibonacci_action_client``, enabling ``Action Introspection``:

.. code-block:: console

  $ ros2 run action_tutorials_cpp fibonacci_action_client --ros-args -p action_client_configure_introspection:=contents

2 Check action availability
^^^^^^^^^^^^^^^^^^^^^^^^^^^

``ros2 bag`` can only record data from available actions.
To see the list of your system's actions, open a new terminal and run the command:

.. code-block:: console

  $ ros2 action list
  /fibonacci

To check if ``Action Introspection`` is enabled on the action, run the command:

.. code-block:: console

  $ ros2 action echo --flow-style /fibonacci
  interface: GOAL_SERVICE
  info:
    event_type: REQUEST_SENT
    stamp:
      sec: 1744917904
      nanosec: 760683446
    client_gid: [1, 15, 165, 231, 234, 109, 65, 202, 0, 0, 0, 0, 0, 0, 19, 4]
    sequence_number: 1
  request: [{goal_id: {uuid: [81, 55, 121, 145, 81, 66, 209, 93, 214, 113, 255, 100, 120, 6, 102, 83]}, goal: {order: 10}}]
  response: []
  ---
  ...

3 Record actions
^^^^^^^^^^^^^^^^

To record action data, the following options are supported.
Action data can be recorded with topics and services at the same time.

To record specific actions:

.. code-block:: console

  $ ros2 bag record --action <action_names>

To record all actions:

.. code-block:: console

  $ ros2 bag record --all-actions

Run the command:

.. code-block:: console

  $ ros2 bag record --action /fibonacci
  [INFO] [1744953225.214114862] [rosbag2_recorder]: Press SPACE for pausing/resuming
  [INFO] [1744953225.218369761] [rosbag2_recorder]: Listening for topics...
  [INFO] [1744953225.218386223] [rosbag2_recorder]: Event publisher thread: Starting
  [INFO] [1744953225.218580294] [rosbag2_recorder]: Recording...
  [INFO] [1744953225.725417634] [rosbag2_recorder]: Subscribed to topic '/fibonacci/_action/cancel_goal/_service_event'
  [INFO] [1744953225.727901848] [rosbag2_recorder]: Subscribed to topic '/fibonacci/_action/feedback'
  [INFO] [1744953225.729655213] [rosbag2_recorder]: Subscribed to topic '/fibonacci/_action/get_result/_service_event'
  [INFO] [1744953225.731315612] [rosbag2_recorder]: Subscribed to topic '/fibonacci/_action/send_goal/_service_event'
  [INFO] [1744953225.735061252] [rosbag2_recorder]: Subscribed to topic '/fibonacci/_action/status'
  ...

Now ``ros2 bag`` is recording the action data for the ``/fibonacci`` action: goal, result, and feedback.
To stop the recording, enter :kbd:`Ctrl-C` in the terminal.

The data will be accumulated in a new bag directory with a name in the pattern of ``rosbag2_year_month_day-hour_minute_second``.
This directory will contain a ``metadata.yaml`` along with the bag file in the recorded format.

4 Inspect action data
^^^^^^^^^^^^^^^^^^^^^

You can see details about your recording by running:

.. code-block:: console

  $ ros2 bag info <bag_file_name>
  Files:             rosbag2_2025_04_17-22_20_40_0.mcap
  Bag size:          20.7 KiB
  Storage id:        mcap
  ROS Distro:        rolling
  Duration:          9.019568080s
  Start:             Apr 17 2025 22:20:47.263125070 (1744953647.263125070)
  End:               Apr 17 2025 22:20:56.282693150 (1744953656.282693150)
  Messages:          0
  Topic information:
  Services:          0
  Service information:
  Actions:           1
  Action information:
    Action: /fibonacci | Type: example_interfaces/action/Fibonacci | Topics: 2 | Service: 3 | Serialization Format: cdr
      Topic: feedback | Count: 9
      Topic: status | Count: 3
      Service: send_goal | Event Count: 4
      Service: cancel_goal | Event Count: 0
      Service: get_result | Event Count: 4

5 Play action data
^^^^^^^^^^^^^^^^^^

Before replaying the bag file, enter :kbd:`Ctrl-C` in the terminal where ``fibonacci_action_client`` is running.
When ``fibonacci_action_client`` stops running, ``fibonacci_action_server`` also stops printing the result because there are no incoming requests.

Replaying the action data from the bag file will start sending the requests to ``fibonacci_action_server``.

Enter the command:

.. code-block:: console

  $ ros2 bag play --send-actions-as-client <bag_file_name>
  [INFO] [1744953720.691068674] [rosbag2_player]: Set rate to 1
  [INFO] [1744953720.702365209] [rosbag2_player]: Adding keyboard callbacks.
  [INFO] [1744953720.702409447] [rosbag2_player]: Press SPACE for Pause/Resume
  [INFO] [1744953720.702423063] [rosbag2_player]: Press CURSOR_RIGHT for Play Next Message
  [INFO] [1744953720.702431404] [rosbag2_player]: Press CURSOR_UP for Increase Rate 10%
  [INFO] [1744953720.702437677] [rosbag2_player]: Press CURSOR_DOWN for Decrease Rate 10%
  Progress bar enabled at 3 Hz.
  Progress bar [?]: [R]unning, [P]aused, [B]urst, [D]elayed, [S]topped
  [INFO] [1744953720.702577680] [rosbag2_player]: Playback until timestamp: -1


  ====== Playback Progress ======
  [1744953656.281683207] Duration 9.02/9.02 [R]

Your ``fibonacci_action_server`` terminal will once again start printing the following service messages:

.. code-block:: console

  [INFO] [1744953720.815577088] [fibonacci_action_server]: Executing goal...
  [INFO] [1744953720.815927050] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1])
  [INFO] [1744953721.816509658] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2])
  [INFO] [1744953722.817220270] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3])
  [INFO] [1744953723.817876426] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3, 5])
  [INFO] [1744953724.818498515] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3, 5, 8])
  [INFO] [1744953725.819182228] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3, 5, 8, 13])
  [INFO] [1744953726.820032562] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3, 5, 8, 13, 21])
  [INFO] [1744953727.820738690] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3, 5, 8, 13, 21, 34])
  [INFO] [1744953728.821449308] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55])

This is because ``ros2 bag play`` sends the action goal request data from the bag file to the ``/fibonacci`` action.

We can also introspect action communication as ``ros2 bag play`` is playing it back to verify the ``fibonacci_action_server``.

Run this command before ``ros2 bag play`` to see the ``fibonacci_action_server``.
You can see the action goal request from the bag file and the service response from  ``fibonacci_action_server``:

.. code-block:: console

  $ ros2 action echo --flow-style /fibonacci
  interface: STATUS_TOPIC
  status_list: [{goal_info: {goal_id: {uuid: [34, 116, 225, 217, 48, 121, 146, 36, 240, 98, 99, 134, 55, 227, 184, 72]}, stamp: {sec: 1744953720, nanosec: 804984321}}, status: 4}]
  ---
  interface: GOAL_SERVICE
  info:
    event_type: REQUEST_RECEIVED
    stamp:
      sec: 1744953927
      nanosec: 957359210
    client_gid: [1, 15, 165, 231, 190, 254, 1, 50, 0, 0, 0, 0, 0, 0, 19, 4]
    sequence_number: 1
  request: [{goal_id: {uuid: [191, 200, 153, 122, 221, 251, 152, 172, 60, 69, 94, 20, 212, 160, 40, 12]}, goal: {order: 10}}]
  response: []
  ---
  interface: GOAL_SERVICE
  info:
    event_type: RESPONSE_SENT
    stamp:
      sec: 1744953927
      nanosec: 957726145
    client_gid: [1, 15, 165, 231, 190, 254, 1, 50, 0, 0, 0, 0, 0, 0, 19, 4]
    sequence_number: 1
  request: []
  response: [{accepted: true, stamp: {sec: 1744953927, nanosec: 957615866}}]
  ---
  interface: STATUS_TOPIC
  status_list: [{goal_info: {goal_id: {uuid: [191, 200, 153, 122, 221, 251, 152, 172, 60, 69, 94, 20, 212, 160, 40, 12]}, stamp: {sec: 1744953927, nanosec: 957663383}}, status: 2}]
  ---
  interface: FEEDBACK_TOPIC
  goal_id:
    uuid: [191, 200, 153, 122, 221, 251, 152, 172, 60, 69, 94, 20, 212, 160, 40, 12]
  feedback:
    sequence: [0, 1, 1]
  ---
  ...

Summary
-------

You can record data passed on topics, services and actions in your ROS 2 system using the ``ros2 bag`` command.
Whether you're sharing your work with others or introspecting your own experiments, it's a great tool to know about.

Next steps
----------

You've completed the "Beginner: CLI Tools" tutorials!
The next step is tackling the "Beginner: Client Libraries" tutorials, starting with :doc:`../../Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace`.

Related content
---------------

A more thorough explanation of ``ros2 bag`` can be found in the README `here <https://github.com/ros2/rosbag2>`__.
For more information on service recording and playback can be found in the design document `here <https://github.com/ros2/rosbag2/blob/{DISTRO}/docs/design/rosbag2_record_replay_service.md>`__.
For more information on action recording and playback can be found in the design document `here <https://github.com/ros2/rosbag2/blob/{DISTRO}/docs/design/rosbag2_record_replay_action.md>`__.
For more information on QoS compatibility and ``ros2 bag``, see :doc:`../../../How-To-Guides/Overriding-QoS-Policies-For-Recording-And-Playback`.
