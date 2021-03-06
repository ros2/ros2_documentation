
About ROS 2 middleware implementations
======================================

.. include:: ../../global_substitutions.txt

ROS middleware implementations are sets of |packages| that implement some of the internal ROS interfaces, e.g. the ``rmw``, ``rcl``, and ``rosidl`` |APIs|.

Common Packages for DDS Middleware Packages
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

All of the current ROS middleware implementations are based on full or partial DDS implementations.
For example, there is a middleware implementation that uses RTI's Connext DDS and an implementation which uses eProsima's Fast-RTPS.
Because of this, there are some shared |packages| amongst most DDS based middleware implementations.

In the `ros2/rosidl_dds <https://github.com/ros2/rosidl_dds>`_ repository on |GitHub|_, there is the following |package|:

-  ``rosidl_generator_dds_idl``: provides tools to generate DDS ``.idl`` files from ``rosidl`` files, e.g. ``.msg`` files, ``.srv`` files, etc.

The ``rosidl_generator_dds_idl`` |package| generates a DDS ``.idl`` file for each ``rosidl`` file, e.g. ``.msg`` file, defined by |packages| containing messages.
Currently DDS based ROS middleware implementations make use of this generator's output ``.idl`` files to generate pre-compiled type support that is vendor specific.

Structure of ROS Middleware Implementations
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A ROS middleware implementation is typically made up of a few |packages| in a single repository:

-  ``<implementation_name>_cmake_module``: contains CMake Module for discovering and exposing required dependencies
-  ``rmw_<implementation_name>_<language>``: contains the implementation of the ``rmw`` |API| in a particular language, typically C++
-  ``rosidl_typesupport_<implementation_name>_<language>``: contains tools to generate static type support code for ``rosidl`` files, tailored to the implementation in a particular language, typically C or C++

The ``<implementation_name>_cmake_module`` |package| contains any CMake Modules and functions needed to find the supporting dependencies for the middleware implementation.
In the case of ``connext_cmake_module`` it has CMake Modules for finding the RTI Connext implementation in different places on the system since it does not ship with a CMake Module itself.
Not all repositories will have a package like this, for example eProsima's Fast-RTPS provides a CMake module and so no additional one is required.

The ``rmw_<implementation_name>_<language>`` |package| implements the ``rmw`` C |API| in a particular language.
The implementation itself can be C++, it just must expose the header's symbols as ``extern "C"`` so that C applications can link against it.

The ``rosidl_typesupport_<implementation_name>_<language>`` |package| provides a generator which generates DDS code in a particular language.
This is done using the ``.idl`` files generated by the ``rosidl_generator_dds_idl`` |package| and the DDS IDL code generator provided by the DDS vendor.
It also generates code for converting ROS message structures to and from DDS message structures.
This generator is also responsible for creating a shared library for the message package it is being used in, which is specific to the messages in the message package and to the DDS vendor being used.

As mentioned above, the ``rosidl_typesupport_introspection_<language>`` may be used instead of a vendor specific type support package if an rmw implementation supports runtime interpretation of messages.
This ability to programmatically send and receive types over topics without generating code beforehand is achieved by supporting the `DDS X-Types Dynamic Data standard <http://www.omg.org/spec/DDS-XTypes>`_.
As such, rmw implementations may provide support for the X-Types standard, and/or provide a package for type support generated at compile time specific to their DDS implementation.

As an example of an rmw implementation repository, the ``opensplice`` ROS middleware implementation lives on |GitHub|_ at `ros2/rmw_opensplice <https://github.com/ros2/rmw_opensplice>`_ and has these |packages|:

-  ``opensplice_cmake_module``
-  ``rmw_opensplice_cpp``
-  ``rosidl_typesupport_opensplice_c``
-  ``rosidl_typesupport_opensplice_cpp``

In addition to the ``opensplice`` repository of |packages|, there is the ``connext`` implementation on |GitHub|_ at `ros2/rmw_connext <https://github.com/ros2/rmw_connext>`_.
It contains mostly the same |packages|, but it additionally contains a |package| to support the type support introspection using the DDS X-Types standard.

The rmw implementation using ``FastRTPS`` is on |GitHub|_ at `eProsima/ROS-RMW-Fast-RTPS-cpp <https://github.com/eProsima/ROS-RMW-Fast-RTPS-cpp>`_.
As ``FastRTPS`` currently only supports the type support introspection, there is no vendor specific type support package in this repository.

To learn more about what is required to create a new middleware implementation for ROS see this page:

.. warning::

    TODO: Link to more detailed middleware implementation docs and/or tutorial.
