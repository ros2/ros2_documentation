
About ROS 2 Interfaces
======================

.. contents:: Table of Contents
   :local:

1. Background
-------------

ROS applications typically communicate through interfaces of one of two types: messages and services.
ROS uses a simplified description language to describe these interfaces. This description makes it easy for ROS tools to automatically generate source code for the interface type in several target languages.

In this document we will describe the supported types and how to create your own msg/srv files.

2. Message Description Specification
------------------------------------

Messages description are defined in ``.msg`` files in the ``msg/`` directory of a ROS package.
``.msg`` files are composed of two parts: fields and constants.

2.1 Fields
^^^^^^^^^^

Each field consists of a type and a name, separated by a space, i.e:

.. code-block:: bash

   fieldtype1 fieldname1
   fieldtype2 fieldname2
   fieldtype3 fieldname3

For example:

.. code-block:: bash

   int32 my_int
   string my_string

2.1.1 Field Types
~~~~~~~~~~~~~~~~~

Field types can be:


* a built-in-type
* names of Message descriptions defined on their own, such as "geometry_msgs/PoseStamped"

*Built-in-types currently supported:*

.. list-table::
   :header-rows: 1

   * - Type name
     - `C++ <http://design.ros2.org/articles/generated_interfaces_cpp.html>`__
     - `Python <http://design.ros2.org/articles/generated_interfaces_python.html>`__
     - `DDS type <http://design.ros2.org/articles/mapping_dds_types.html>`__
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


*Every built-in-type can be used to define arrays:*

.. list-table::
   :header-rows: 1

   * - Type name
     - `C++ <http://design.ros2.org/articles/generated_interfaces_cpp.html>`__
     - `Python <http://design.ros2.org/articles/generated_interfaces_python.html>`__
     - `DDS type <http://design.ros2.org/articles/mapping_dds_types.html>`__
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


All types that are more permissive than their ROS definition enforce the ROS constraints in range and length by software

*Example of message definition using arrays and bounded types:*

.. code-block:: bash

   int32[] unbounded_integer_array
   int32[5] five_integers_array
   int32[<=5] up_to_five_integers_array

   string string_of_unbounded_size
   string<=10 up_to_ten_characters_string

   string[<=5] up_to_five_unbounded_strings
   string<=10[] unbounded_array_of_string_up_to_ten_characters each
   string<=10[<=5] up_to_five_strings_up_to_ten_characters_each

2.1.2 Field Names
~~~~~~~~~~~~~~~~~

Field names must be lowercase alphanumeric characters with underscores for separating words. They must start with an alphabetic character, they must not end with an underscore and never have two consecutive underscores.

2.1.3 Field Default Value
~~~~~~~~~~~~~~~~~~~~~~~~~

Default values can be set to any field in the message type.
Currently default values are not supported for string arrays and complex types (i.e. types not present in the built-in-types table above, that applies to all nested messages)

Defining a default value is done by adding a third element to the field definition line, i.e:

.. code-block:: bash

   fieldtype fieldname fielddefaultvalue

For example:

.. code-block:: bash

   uint8 x 42
   int16 y -2000
   string full_name "John Doe"
   int32[] samples [-200, -100, 0, 100, 200]

Note:


* string values must be defined in single ``'`` or double quotes ``"``
* currently string values are not escaped

2.2 Constants
^^^^^^^^^^^^^

Each constant definition is like a field description with a default value, except that this value can never be changed programatically. This value assignment is indicated by use of an equal '=' sign, e.g.

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

3. Service Description Specification
------------------------------------

Services description are defined in ``.srv`` files in the ``srv/`` directory of a ROS package.

A service description file consists of a request and a response msg type, separated by '---'. Any two .msg files concatenated together with a '---' are a legal service description.

Here is a very simple example of a service that takes in a string and returns a string:

.. code-block:: bash

   string str
   ---
   string str

We can of course get much more complicated (if you want to refer to a message from the same package you must not mention the package name):

.. code-block:: bash

   #request constants
   int8 FOO=1
   int8 BAR=2
   #request fields
   int8 foobar
   another_pkg/AnotherMessage msg
   ---
   #response constants
   uint32 SECRET=123456
   #response fields
   another_pkg/YetAnotherMessage val
   CustomMessageDefinedInThisPackage value
   uint32 an_integer

You cannot embed another service inside of a service.
