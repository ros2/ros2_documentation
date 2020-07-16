.. _ROS2Bag-QoS-Override:

Overriding QoS Policies For Recording And Playback
==================================================

**Goal:** Override Ros2Bag QoS profile settings for recording and playback.

**Tutorial level:** Intermediate

**Time:** 5 minutes

.. contents:: Contents
   :depth: 2
   :local:


Background
----------

With the introduction of DDS in ROS2, Quality of Service (QoS) compatibility for publisher/subscriber nodes needs to be considered when recording and playing back data.
More detail on how QoS works can be found `here <https://index.ros.org/doc/ros2/Concepts/About-Quality-of-Service-Settings>`__.
For the purposes of this tutorial, it is sufficient to know that only the reliability and durability policies affect whether publishers/subscribers are compatible and can receive data from one other.
Ros2Bag adapts its requested/offered QoS profile when recording/playing data from a topic to prevent dropped messages.
During playback, Ros2bag also attempts to preserve the policy originally offered by the topic.
Certain situations may require specifying explicit QoS profile settings so Ros2Bag can record/playback topics.
These QoS profile overrides can be specified via the CLI using the ``--qos-profile-overrides-path`` flag.

Using QoS Overrides
-------------------

The YAML schema for the profile overrides is a dictionary of topic names with key/value pairs for each QoS policy:

.. code-block:: yaml

    topic_name: str
      qos_policy_name: str
      ...
      qos_duration: object
        sec: int
        nsec: int

If a policy value is not specified, the value will fallback to the default used by Ros2Bag.
If you specify a Duration based policy such as ``deadline`` or ``lifespan``, you will need to specify both seconds and nanoseconds.
Policy values are determined by the policy’s short keys which can be found using ``ros2topic`` verbs such as ``ros2 topic pub --help``.
All values are replicated below for reference.

.. code-block:: yaml

    history: [keep_all, keep_last]
    depth: int
    reliability: [system_default, reliable, best_effort, unknown]
    durability: [system_default, transient_local, volatile, unknown]
    deadline:
      sec: int
      nsec: int
    lifespan:
      sec: int
      nsec: int
    liveliness: [system_default, automatic, manual_by_node, manual_by_topic, unknown]
    liveliness_lease_duration:
      sec: int
      nsec: int
    avoid_ros_namespace_conventions: [true, false]

Example
-------

Consider a topic ``/talker`` offering a ``transient_local`` Durability policy.
ROS2 publishers by default request ``volatile`` Durability.

.. code-block:: console

    ros2 topic pub /talker std_msgs/String "data: Hello World"

In order for Ros2Bag to record the data, we would want to override the recording policy for that specific topic like so:

.. code-block:: yaml

    # durability_override.yaml
    /talker:
      durability: transient_local
      history: keep_all

And call it from the CLI:

.. code-block:: console

    ros2 bag record -a -o my_bag --qos-profile-overrides-path durability_override.yaml

If we want to playback the bag file but with a different Reliability policy, we can specify one as such;

.. code-block:: yaml

    # reliability_override.yaml
    /talker:
      reliability: best_effort
      history: keep_all

And call it from the CLI:

.. code-block:: console

    ros2 bag play --qos-profile-overrides-path reliability_override.yaml my_bag

We can see the results with ``ros2topic``

.. code-block:: console

    ros2 topic echo --qos-reliability best_effort /talker std_msgs/String
