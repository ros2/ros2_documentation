.. redirect-from::

    About-ROS-Interfaces
    How-To-Guides/Topics-Services-Actions
    Concepts/Basic/About-Interfaces

.. meta::
   :contentType: about
   :experience: beginner, intermediate, expert
   :area: interfaces, framework
   :distribution: {DISTRO}
   :product: {PRODUCT}

.. _interfaces-topics-services-actions:
.. _TopicsServicesActions:

Interfaces (topics, services, actions)
======================================

.. short-description::
   Interfaces in ROS define the structured data used by nodes to exchange messages, make requests, and coordinate long-running tasks.
   This article explains how message, service, and action interface files are written, organized, and used to generate code across supported programming languages.

.. showmeta::
   :order: area, contentType, experience
   :labels: area=Area, contentType=Content type, experience=Level

.. toctree::
   :maxdepth: 2
   :hidden:

   interfaces/About-Topics
   interfaces/About-Services
   interfaces/About-Actions
   interfaces/Working-with-interfaces


Interfaces in ROS define how nodes exchange data.
This article explains the different types of ROS interface and the differences between them.
With this information, you'll be able to select the right interfaces for your purposes.

Summary
-------
When designing a system there are three primary styles of interfaces.
The specifications for the content is in the :doc:`Interfaces Overview <interfaces/Working-with-interfaces/Interface-specifications>`.
This is written to provide the reader with guidelines about when to use each type of interface.

ROS nodes typically communicate through the following three types of interfaces:

* :doc:`Topics <interfaces/About-Topics>`: For continuous data streams.
* :doc:`Services <interfaces/About-Services>`: For synchronous request/response interactions (short tasks which happen immediately).
* :doc:`Actions <interfaces/About-Actions>`: For long-running tasks with feedback (tasks that may take some time to complete).

For consistent communication, each interface uses definitions provided in ``.msg``, ``.srv``, or ``.action`` files.

:doc:`Learn more about nodes <About-Nodes>`

Topics
------

The topic interface is meant for continuous data streams, for example, streaming sensor data or the status of your robot.
Topic definitions are stored in ``.msg`` files.
Topics implement a publish/subscribe pattern.
A node publishes data to a topic, and other nodes subscribe to receive that data.
This interface type has the following main characteristics:

* Asynchronous, one-way communication
* Multiple publishers and subscribers can share the same topic

.. mermaid::

   flowchart LR
    P[Publisher node] -->|Publishes messages| T[Topic]
    T -->|Delivers messages| S1[Subscriber node]
    T -->|Delivers messages| S2[Subscriber node]

Topic keys identify individual publishers on a topic so nodes and tools can distinguish where messages come from.
Each topic key makes it easier to track data sources when several publishers share the same topic.


* Should be used for continuous data streams (sensor data, robot state, ...)
* Are for continuous data flow.
  Data might be published and subscribed at any time independent of any senders/receivers.
  Many to many connection.
  Callbacks receive data once it is available.
  The publisher decides when data is sent.

Topic statistics
^^^^^^^^^^^^^^^^

Topic statistics are built-in measurements that help you understand how messages behave when a subscription receives them.
When enabled, they automatically track two things:

:Message age: How old a message is when it arrives, based on its timestamp.
:Message period: The time between incoming messages.

For both message age and period, ROS calculates the average, minimum, maximum, standard deviation, and the number of samples, using a moving window that updates every time a new message arrives.
These calculations run in constant time and memory using the dedicated utilities.
When you enable topic statistics for a subscription, ROS publishes the collected data at regular intervals as a ``MetricsMessage`` on a statistics topic.
This gives you a clear view of timing patterns, delays, and irregularities, making it easier to assess system performance or diagnose problems related to the message flow.

.. tip::

   The default interval is 1 second.
   The default statistics topic is ``/statistics``.

:doc:`Learn how to enable topic statistics <../Developer-Tools/Introspection-and-analysis/Topic-Statistics-Tutorial/Topic-Statistics-Tutorial>`
    Concepts/Basic/About-Interfaces

Interface specifications
========================

.. contents:: Table of Contents
   :local:

Background
----------

ROS applications typically communicate through interfaces of one of three types: :doc:`topics <../About-Topics>`, :doc:`services <../About-Services>`, or :doc:`actions <../About-Actions>`.
ROS 2 uses a simplified description language, the interface definition language (IDL), to describe these interfaces.
This description makes it easy for ROS tools to automatically generate source code for the interface type in several target languages.

In this document we will describe the supported types:

* msg: ``.msg`` files are simple text files that describe the fields of a ROS message.
  They are used to generate source code for messages in different languages.
* srv: ``.srv`` files describe a service.
  They are composed of two parts: a request and a response.
  The request and response are message declarations.
* action: ``.action`` files describe actions.
  They are composed of three parts: a goal, a result, and feedback.
  Each part is a message declaration itself.

Messages
--------

Messages are a way for a ROS 2 node to send data on the network to other ROS nodes, with no response expected.
For instance, if a ROS 2 node reads temperature data from a sensor, it can then publish that data on the ROS 2 network using a ``Temperature`` message.
Other nodes on the ROS 2 network can subscribe to that data and receive the ``Temperature`` message.

Messages are described and defined in ``.msg`` files in the ``msg/`` directory of a ROS package.
``.msg`` files are composed of two parts: fields and constants.

Fields
^^^^^^

Each field consists of a type and a name, separated by a space, i.e:

.. code-block:: bash

   fieldtype1 fieldname1
   fieldtype2 fieldname2
   fieldtype3 fieldname3

For example:

.. code-block:: bash

   int32 my_int
   string my_string

Field types
~~~~~~~~~~~

Field types can be:

* a built-in-type
* names of Message descriptions defined on their own, such as "geometry_msgs/PoseStamped"

*Built-in-types currently supported:*

.. list-table::
   :header-rows: 1

   * - Type name
     - `C++ <https://design.ros2.org/articles/generated_interfaces_cpp.html>`__
     - `Python <https://design.ros2.org/articles/generated_interfaces_python.html>`__
     - `DDS type <https://design.ros2.org/articles/mapping_dds_types.html>`__
   * - bool
     - bool
     - builtins.bool
     - boolean
   * - byte
     - uint8_t
     - builtins.bytes*
     - octet
   * - char
     - char
     - builtins.int*
     - char
   * - float32
     - float
     - builtins.float*
     - float
   * - float64
     - double
     - builtins.float*
     - double
   * - int8
     - int8_t
     - builtins.int*
     - octet
   * - uint8
     - uint8_t
     - builtins.int*
     - octet
   * - int16
     - int16_t
     - builtins.int*
     - short
   * - uint16
     - uint16_t
     - builtins.int*
     - unsigned short
   * - int32
     - int32_t
     - builtins.int*
     - long
   * - uint32
     - uint32_t
     - builtins.int*
     - unsigned long
   * - int64
     - int64_t
     - builtins.int*
     - long long
   * - uint64
     - uint64_t
     - builtins.int*
     - unsigned long long
   * - string
     - std::string
     - builtins.str
     - string
   * - wstring
     - std::u16string
     - builtins.str
     - wstring

*Every built-in-type can be used to define arrays:*

.. list-table::
   :header-rows: 1

   * - Type name
     - `C++ <https://design.ros2.org/articles/generated_interfaces_cpp.html>`__
     - `Python <https://design.ros2.org/articles/generated_interfaces_python.html>`__
     - `DDS type <https://design.ros2.org/articles/mapping_dds_types.html>`__
   * - static array
     - std::array<T, N>
     - builtins.list*
     - T[N]
   * - unbounded dynamic array
     - std::vector
     - builtins.list
     - sequence
   * - bounded dynamic array
     - custom_class<T, N>
     - builtins.list*
     - sequence<T, N>
   * - bounded string
     - std::string
     - builtins.str*
     - string

(*) All types that are more permissive than their ROS definition enforce the ROS constraints in range and length by software.

*Example of message definition using arrays and bounded types:*

.. code-block:: bash

   int32[] unbounded_integer_array
   int32[5] five_integers_array
   int32[<=5] up_to_five_integers_array

   string string_of_unbounded_size
   string<=10 up_to_ten_characters_string

   string[<=5] up_to_five_unbounded_strings
   string<=10[] unbounded_array_of_strings_up_to_ten_characters_each
   string<=10[<=5] up_to_five_strings_up_to_ten_characters_each

Field names
~~~~~~~~~~~

Field names must be lowercase alphanumeric characters with underscores for separating words.
They must start with an alphabetic character, and they must not end with an underscore or have two consecutive underscores.

Field default value
~~~~~~~~~~~~~~~~~~~

Default values can be set to any field in the message type.
Currently default values are not supported for string arrays and complex types (i.e. types not present in the built-in-types table above; that applies to all nested messages).

Defining a default value is done by adding a third element to the field definition line, i.e:

.. code-block:: bash

   fieldtype fieldname fielddefaultvalue

For example:

.. code-block:: bash

   uint8 x 42
   int16 y -2000
   string full_name "John Doe"
   int32[] samples [-200, -100, 0, 100, 200]

.. note::

  * string values must be defined in single ``'`` or double ``"`` quotes
  * currently string values are not escaped

Constants
^^^^^^^^^

Each constant definition is like a field description with a default value, except that this value can never be changed programmatically.
This value assignment is indicated by use of an equal '=' sign, e.g.

.. code-block:: bash

   constanttype CONSTANTNAME=constantvalue

For example:

.. code-block:: bash

   int32 X=123
   int32 Y=-123
   string FOO="foo"
   string EXAMPLE='bar'

.. note::

   Constants names have to be UPPERCASE

Services
--------

The service interface is meant for synchronous request/response interactions, for example, when you want to send a query requesting the configuration of a specific robot.
Service definitions are stored in ``.srv`` files.
Services implement a request/response pattern.
A client sends a request, and a server replies with a response.
This interface type has the following main characteristics:

* Synchronous communication
* Ideal for short-lived operations that require confirmation, or provide a result in response to a request

.. mermaid::

   sequenceDiagram
    participant Service client
    participant Service server
    Service client->>Service server: Request
    Service server-->>Service client: Response


* Should be used for remote procedure calls that terminate quickly, e.g. for querying the state of a node or doing a quick calculation such as IK.
  They should never be used for longer running processes, in particular processes that might be required to preempt if exceptional situations occur and they should never change or depend on state to avoid unwanted side effects for other nodes.

Actions
-------

The action interface is meant for long-running tasks with feedback, for example, moving a robot to a specific position, or asking the robot to perform a complex motion.
Action definitions are stored in ``.action`` files.
Actions allow clients to send goals, receive feedback during the execution, cancel if needed, and return a result if available.
This interface type has the following main characteristics:

* Asynchronous, with feedback and result
* Suitable for operations that take time

.. mermaid::

   sequenceDiagram
    participant Action client
    participant Action server
    Client->>Action Server: Sends a goal
    Action server-->>Action client: Provides feedback (periodic)
    Action server-->>Action client: Sends a result

* Should be used for any discrete behavior that moves a robot or that runs for a longer time but provides feedback during execution.
* The most important property of actions is that they can be preempted and preemption should always be implemented cleanly by action servers.
* Actions can keep state for the lifetime of a goal, i.e. if executing two action goals in parallel on the same server, for each client a separate state instance can be kept since the goal is uniquely identified by its id.
* Slow perception routines which take several seconds to terminate or initiating a lower-level control mode are good use cases for actions.
* More complex non-blocking background processing.
  Used for longer tasks like execution of robot actions.
  Semantically for real-world actions.


Key differences between ROS interfaces
--------------------------------------

All three interfaces enable communication between nodes, but each serves a different purpose.
The table below summarizes the differences between ROS interface types:

+--------------+----------------------+-----------------------+-----------------+--------------------+---------------+
|              | Pattern              | Direction             | Provided result | Typical use case   | Cancellation  |
+==============+======================+=======================+=================+====================+===============+
| **Topics**   | Publish/Subscribe    | One-way               | No              | Continuous data    | Not supported |
+--------------+----------------------+-----------------------+-----------------+--------------------+---------------+
| **Services** | Request/Response     | Two-way               | Yes             | Quick queries      | Not supported |
+--------------+----------------------+-----------------------+-----------------+--------------------+---------------+
| **Actions**  | Goal/Feedback/Result | Two-way with feedback | Yes             | Long-running tasks | Supported     |
+--------------+----------------------+-----------------------+-----------------+--------------------+---------------+
