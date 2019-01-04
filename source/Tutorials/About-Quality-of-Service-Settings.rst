
Overview
--------

ROS 2 offers a rich variety of Quality of Service (QoS) policies that allow you to tune communication between nodes.
With the right set of Quality of Service policies, ROS 2 can be as reliable as TCP or as best-effort as UDP, with many, many possible states in between.
Unlike ROS 1, which primarily only supported TCP, ROS 2 benefits from the flexibility of the underlying DDS transport in environments with lossy wireless networks where a "best effort" policy would be more suitable, or in real-time computing systems where the right Quality of Service profile is needed to meet deadlines.

A set of QoS "policies" combine to form a QoS "profile".
Given the complexity of choosing the correct QoS policies for a given scenario, ROS 2 provides a set of predefined QoS profiles for common usecases (e.g. sensor data).
At the same time, users are given the flexibility to control specific profiles of the QoS policies.

QoS profiles can be specified for publishers, subscribers, service servers and clients.
A QoS profile can be applied independently to each instance of the aforementioned entities, but if different profiles are used it is possible that they will not connect.

QoS policies
------------

The base QoS profile currently includes settings for the following policies:


* History

  * Keep last: only store up to N samples, configurable via the queue depth option.
  * Keep all: store all samples, subject to the configured resource limits of the underlying middleware.

* Depth

  * Size of the queue: only honored if used together with “keep last”.

* Reliability

  * Best effort: attempt to deliver samples, but may lose them if the network is not robust.
  * Reliable: guarantee that samples are delivered, may retry multiple times.

* Durability

  * Transient local: the publisher becomes responsible for persisting samples for "late-joining" subscribers.
  * Volatile: no attempt is made to persist samples.

For each of the policies there is also the option of “system default”, which uses the default of the underlying middleware which may be defined via DDS vendor tools (e.g. XML configuration files).
DDS itself has a wider range of policies that can be configured.
These policies have been exposed because of their similarity to features in ROS 1; it is possible that in the future more policies will be exposed in ROS 2.

Comparison to ROS 1
^^^^^^^^^^^^^^^^^^^

The history and depth policies in ROS 2 combine to provide functionality akin to the queue size in ROS 1.

The reliability policy in ROS 2 is akin to the use of either UDPROS (only in ``roscpp``\ ) for "best effort", or TCPROS (ROS 1 default) for reliable.
Note however that even the reliable policy in ROS 2 is implemented using UDP, which allows for multicasting if appropriate.

The durability policy combined with a depth of 1 provides functionality similar to that of "latching" subscribers.

QoS profiles
------------

Profiles allow developers to focus on their applications without worrying about every QoS setting possible.
A QoS profile defines a set of policies that are expected to go well together for a particular use case.

The currently-defined QoS profiles are:


* Default QoS settings for publishers and subscribers

  In order to make the transition from ROS 1 to ROS 2, exercising a similar network behavior is desirable.
  By default, publishers and subscribers are reliable in ROS 2, have volatile durability, and "keep last" history.

* Services

  In the same vein as publishers and subscribers, services are reliable.
  It is especially important for services to use volatile durability, as otherwise service servers that re-start may receive outdated requests.
  While the client is protected from receiving multiple responses, the server is not protected from side-effects of receiving the outdated requests.

* Sensor data

  For sensor data, in most cases it's more important to receive readings in a timely fashion, rather than ensuring that all of them arrive.
  That is, developers want the latest samples as soon as they are captured, at the expense of maybe losing some.
  For that reason the sensor data profile uses best effort reliability and a smaller queue depth.

* Parameters

  Parameters in ROS 2 are based on services, and as such have a similar profile.
  The difference is that parameters use a much larger queue depth so that requests do not get lost when, for example, the parameter client is unable to reach the parameter service server.

* System default

   This uses the system default for all of the policies.

`Click here <https://github.com/ros2/rmw/blob/release-latest/rmw/include/rmw/qos_profiles.h>`__ for the specific policies in use for the above profiles.
The settings in these profiles are subject to further tweaks, based on the feedback from the community.

While ROS 2 provides some QoS profiles for common use cases, the use of policies that are defined in DDS allows ROS users to take advantage of the vast knowledge base of existing DDS documentation for configuring QoS profiles for their specific use case.

QoS compatibilities
-------------------

**Note:** This section refers to publisher and subscribers but the content applies to service servers and clients in the same manner.

QoS profiles may be configured for publishers and subscribers independently.
A connection between a publisher and a subscriber is only made if the pair has compatible QoS profiles.
QoS profile compatibility is determined based on a "Request vs Offerer" model, wherein connections are only made if the requested policy of the subscriber is not more stringent than the that of the publisher.
The less strict of the two policies will be the one used for the connection.

The QoS policies exposed in ROS 2 that affect compatibility are the durability and reliability policies.
The following tables show the compatibility of the different policy settings and the result:

*Compatibility of QoS durability profiles:*

.. list-table::
   :header-rows: 1

   * - Publisher
     - Subscriber
     - Connection
     - Result
   * - Volatile
     - Volatile
     - Yes
     - Volatile
   * - Volatile
     - Transient local
     - No
     - -
   * - Transient local
     - Volatile
     - Yes
     - Volatile
   * - Transient local
     - Transient local
     - Yes
     - Transient local


*Compatibility of QoS reliability profiles:*

.. list-table::
   :header-rows: 1

   * - Publisher
     - Subscriber
     - Connection
     - Result
   * - Best effort
     - Best effort
     - Yes
     - Best effort
   * - Best effort
     - Reliable
     - No
     - -
   * - Reliable
     - Best effort
     - Yes
     - Best effort
   * - Reliable
     - Reliable
     - Yes
     - Reliable


In order for a connection to be made, all of the policies that affect compatibility must be compatible.
That is, even if a publisher-subscriber pair has compatible reliability QoS profiles, if they have incompatible durability QoS profiles a connection will not be made, and vice-versa.
