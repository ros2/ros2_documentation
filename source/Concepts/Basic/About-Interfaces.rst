.. redirect-from::

    About-ROS-Interfaces
    Concepts/About-ROS-Interfaces

Interfaces
==========

.. contents:: Table of Contents
   :local:

Background
----------

ROS applications typically communicate through interfaces of one of three types: :doc:`topics <About-Topics>`, :doc:`services <About-Services>`, or :doc:`actions <About-Actions>`.
ROS 2 uses a simplified description language, the interface definition language (IDL), to describe these interfaces.
This description makes it easy for ROS tools to automatically generate source code for the interface type in several target languages.

In this document we will describe the supported types:

* msg: ``.msg`` files are simple text files that describe the fields of a ROS message. They are used to generate source code for messages in different languages.
* srv: ``.srv`` files describe a service. They are composed of two parts: a request and a response. The request and response are message declarations.
* action: ``.action`` files describe actions. They are composed of three parts: a goal, a result, and feedback.
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
     - builtins.str*
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

All types that are more permissive than their ROS definition enforce the ROS constraints in range and length by software.

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

Each constant definition is like a field description with a default value, except that this value can never be changed programatically.
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

Services are a request/response communication, where the client (requester) is waiting for the server (responder) to make a short computation and return a result.

Services are described and defined in ``.srv`` files in the ``srv/`` directory of a ROS package.

A service description file consists of a request and a response msg type, separated by ``---``.
Any two ``.msg`` files concatenated with a ``---`` are a legal service description.

Here is a very simple example of a service that takes in a string and returns a string:

.. code-block:: bash

   string str
   ---
   string str

We can of course get much more complicated (if you want to refer to a message from the same package you must not mention the package name):

.. code-block:: bash

   # request constants
   int8 FOO=1
   int8 BAR=2
   # request fields
   int8 foobar
   another_pkg/AnotherMessage msg
   ---
   # response constants
   uint32 SECRET=123456
   # response fields
   another_pkg/YetAnotherMessage val
   CustomMessageDefinedInThisPackage value
   uint32 an_integer

You cannot embed another service inside of a service.

Actions
-------

Actions are a long-running request/response communication, where the action client (requester) is waiting for the action server (the responder) to take some action and return a result.
In contrast to services, actions can be long-running (many seconds or minutes), provide feedback while they are happening, and can be interrupted.

Action definitions have the following form:

.. code::

   <request_type> <request_fieldname>
   ---
   <response_type> <response_fieldname>
   ---
   <feedback_type> <feedback_fieldname>

Like services, the request fields are before and the response fields are after the first triple-dash (``---``), respectively.
There is also a third set of fields after the second triple-dash, which is the fields to be sent when sending feedback.

There can be arbitrary numbers of request fields (including zero), arbitrary numbers of response fields (including zero), and arbitrary numbers of feedback fields (including zero).

The ``<request_type>``, ``<response_type>``, and ``<feedback_type>`` follow all of the same rules as the ``<type>`` for a message.
The ``<request_fieldname>``, ``<response_fieldname>``, and ``<feedback_fieldname>`` follow all of the same rules as the ``<fieldname>`` for a message.

For instance, the ``Fibonacci`` action definition contains the following:

.. code::

   int32 order
   ---
   int32[] sequence
   ---
   int32[] sequence

This is an action definition where the action client is sending a single ``int32`` field representing the number of Fibonacci steps to take, and expecting the action server to produce an array of ``int32`` containing the complete steps.
Along the way, the action server may also provide an intermediate array of ``int32`` contains the steps accomplished up until a certain point.
