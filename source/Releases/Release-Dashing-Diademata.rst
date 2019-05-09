
ROS 2 Dashing Diademata (codename 'dashing'; May 31st, 2019)
============================================================

.. contents:: Table of Contents
   :depth: 2
   :local:

*Dashing Diademata* will be the fourth release of ROS 2.

Supported Platforms
-------------------

To be determined.


New features in this ROS 2 release
----------------------------------

During the development the `Dashing meta ticket <https://github.com/ros2/ros2/issues/607>`__ on GitHub contains an up-to-date state of the ongoing high level tasks as well as references specific tickets with more details.


Timeline before the release
---------------------------

A few milestone leading up to the release:

    Mon. Apr 8th (alpha)
        First releases of core packages available.
        Testing can happen from now on (some features might not have landed yet).

    Thu. May 2nd
        API freeze for core packages

    Mon. May 6th (beta)
        Updated releases of core packages available.
        Additional testing of the latest features.

    Thu. May 16th
        Feature freeze.
        Only bug fix releases should be made after this point.
        New packages can be released independently.

    Mon. May 20th (release candidate)
        Updated releases of core packages available.

    Wed. May 29th
        Freeze rosdistro.
        No PRs for Dashing on the `rosdistro` repo will be merged (reopens after the release announcement).


Changes since the Crystal release
---------------------------------

ament_cmake
~~~~~~~~~~~

The CMake function ``ament_index_has_resource`` was returning either ``TRUE`` or ``FALSE``.
As of `this release <https://github.com/ament/ament_cmake/pull/155>`_ it returns either the prefix path in case the resource was found or ``FALSE``.

If you are using the return value in a CMake condition like this:

.. code-block:: cmake

   ament_index_has_resource(var ...)
   if(${var})

you need to update the condition to ensure it considers a string value as ``TRUE``:

.. code-block:: cmake

   if(var)

rclcpp
~~~~~~

The function ``NodeGraph::get_node_names()`` now returns a ``vector`` of fully qualified names and namespaces, instead of just names.

Extended arguments (beyond name and namespace) to the ``rclcpp::Node()`` constructor have been replaced with a ``rclcpp::NodeOptions`` structure.
See `ros2/rclcpp#622 <https://github.com/ros2/rclcpp/pull/622/files>`__ for details about the structure and default values of the options.

If you are using any of the extended arguments to ``rclcpp::Node()`` like this:

.. code-block:: cpp

  auto context = rclcpp::contexts::default_context::get_global_default_context();
  std::vector<std::string> args;
  std::vector<rclcpp::Parameter> params = { rclcpp::Parameter("use_sim_time", true) };
  auto node = std::make_shared<rclcpp::Node>("foo_node", "bar_namespace", context, args, params);

You need to update to use the ``NodeOptions`` structure

.. code-block:: cpp

  std::vector<std::string> args;
  std::vector<rclcpp::Parameter> params = { rclcpp::Parameter("use_sim_time", true) };
  rclcpp::NodeOptions node_options;
  node_options.arguments(args);
  node_options.initial_parameters(params);
  auto node = std::make_shared<rclcpp::Node>("foo_node", "bar_namespace", node_options);

rclcpp_components
~~~~~~~~~~~~~~~~~

The correct way to implement composition in Dashing is by utilizing the ``rclcpp_components`` package.

The following changes must be made to nodes in order to correctly implement runtime composition:

The Node must have a constructor that takes ``rclcpp::NodeOptions``:

.. code-block:: cpp

  class Listener: public rclcpp::Node {
    Listener(const rclcpp::NodeOptions & options)
    : Node("listener", options)
    {
    }
  };

C++ registration macros (if present) need to be updated to use the ``rclcpp_components`` equivalent.
If not present, registration macros must be added in one translation unit.

.. code-block:: cpp

  // Insert at bottom of translation unit, e.g. listener.cpp
  #include "rclcpp_components/register_node_macro.hpp"
  // Use fully-qualifed name in registration
  RCLCPP_COMPONENTS_REGISTER_NODE(composition::Listener);

CMake regsitration macros (if present) need to be updated.
If not present, registration macros must be added to the project's CMake.

.. code-block:: cmake

  add_library(listener src/listener.cpp)
  rclcpp_components_register_nodes(listener "composition::Listener")

For more information on composition, see `the tutorial <https://index.ros.org/doc/ros2/Tutorials/Composition/>`__

rosidl
~~~~~~

Until Crystal each message generator package registered itself using the ``ament_cmake`` extension point ``rosidl_generate_interfaces`` and was passed a set of ``.msg`` / ``.srv`` / ``.action`` files.
As of Dashing the message generation pipeline is based on ``.idl`` files instead.

Any message generator package needs to change and register itself using the new extension point ``rosidl_generate_idl_interfaces`` which passes only ``.idl`` files instead.
The message generators for the commonly supported languages C, C++, and Python as well as the typesupport packages for introspection, FastRTPS, Connext and OpenSplice have already been updated (see `ros2/rosidl#334 <https://github.com/ros2/rosidl/pull/334/files>`__).
The CMake code calling ``rosidl_generate_interfaces()`` can either pass ``.idl`` files directly or pass ``.msg`` / ``.srv`` / ``.action`` which will then internally be converted into ``.idl`` files before being passed to each message generator.

The format of ``.msg`` / ``.srv`` / ``.action`` files is not being evolved in the future.
The mapping between ``.msg`` / ``.srv`` / ``.action`` files and ``.idl`` files is described in `this design article <http://design.ros2.org/articles/legacy_interface_definition.html>`__.
A `second design article <http://design.ros2.org/articles/idl_interface_definition.html>`__ describes the supported features in ``.idl`` files.
In order to leverage any of the new features existing interfaces need to be converted (e.g. using the command line tools  ``msg2idl`` / ``srv2idl`` / ``action2idl``).

Mapping of char in .msg files
"""""""""""""""""""""""""""""

In `ROS 1 <http://wiki.ros.org/msg#Fields>`__ ``char`` has been deprecated for a long time and is being mapped to ``uint8``.
In ROS 2 until Crystal ``char`` was mapped to a single character (``char`` in C / C++, ``str`` with length 1 in Python) in an effort to provide a more natural mapping.
As of Dashing the ROS 1 semantic has been restored and ``char`` maps to ``uint8`` again.

rosidl_generator_cpp
~~~~~~~~~~~~~~~~~~~~

The C++ data structures generated for messages, services and actions provide setter methods for each field.
Until Crystal each setter returned a pointer to the data structure itself to enable the named parameter idiom.
As of Dashing these setters `return a reference <https://github.com/ros2/rosidl/pull/353>`__ instead since that seems to be the more common signature as well as it clarifies that the returned value can't be a ``nullptr``.

rosidl_generator_py
~~~~~~~~~~~~~~~~~~~

Until Crystal an array (fixed size) or sequence (dynamic size, optionally with an upper boundary) field in a message was stored as a ``list`` in Python.
As of Dashing the Python type for arrays / sequences of numeric values has been changed:

* an array of numeric values is stored as a ``numpy.ndarray`` (the ``dtype`` is chosen to match the type of the numeric value)
* a sequence of numeric values is stored as an ``array.array`` (the ``typename`` is chosen to match the type of the numeric value)

As before an array / sequence of non-numeric types is still represented as a ``list`` in Python.

This change brings a number of benefits:

* The new data structures ensure that each item in the array / sequence complies with the value range restrictions of the numeric type.
* The numeric values can be stored more efficiently in memory which avoid the overhead of Python objects for each item.
* The memory layout of both data structures allows to read and write all items of the array / sequence in a single operation which makes the conversion from and to Python significantly faster / more efficient.

launch
~~~~~~

The ``launch_testing`` package caught up with the ``launch`` package redesign done in Bouncy Bolson.
The legacy Python API, already moved into the ``launch.legacy`` submodule, has thus been deprecated and removed.

See ``launch`` `examples <https://github.com/ros2/launch/tree/master/launch/examples>`__ and `documentation <https://github.com/ros2/launch/tree/master/launch/doc>`__ for reference on how to use its new API.

See `demos tests <https://github.com/ros2/demos>`__ for reference on how to use the new ``launch_testing`` API.

rmw
~~~

Changes since the `Crystal Clemmys <Release-Crystal-Clemmys>` release:

* New API in ``rmw``, a fini function for ``rmw_context_t``:

 * `rmw_context_fini <https://github.com/ros2/rmw/blob/c518842f6f82910482470b40c221c268d30691bd/rmw/include/rmw/init.h#L111-L136>`_

* Modification of ``rmw``, now passes ``rmw_context_t`` to ``rmw_create_wait_set``:

 * `rmw_create_wait_set <https://github.com/ros2/rmw/blob/c518842f6f82910482470b40c221c268d30691bd/rmw/include/rmw/rmw.h#L522-L543>`_

* New APIs in ``rmw`` for preallocating space for published and subscribed messages:

 * `rmw_init_publisher_allocation <https://github.com/ros2/rmw/blob/dc7b2f49f1f961d6cf2c173adc54736451be8938/rmw/include/rmw/rmw.h#L262>`_
 * `rmw_fini_publisher_allocation <https://github.com/ros2/rmw/blob/dc7b2f49f1f961d6cf2c173adc54736451be8938/rmw/include/rmw/rmw.h#L279>`_
 * `rmw_init_subscription_allocation <https://github.com/ros2/rmw/blob/dc7b2f49f1f961d6cf2c173adc54736451be8938/rmw/include/rmw/rmw.h#L489>`_
 * `rmw_fini_subscription_allocation <https://github.com/ros2/rmw/blob/dc7b2f49f1f961d6cf2c173adc54736451be8938/rmw/include/rmw/rmw.h#L506>`_
 * `rmw_serialized_message_size <https://github.com/ros2/rmw/blob/dc7b2f49f1f961d6cf2c173adc54736451be8938/rmw/include/rmw/rmw.h#L395>`_

* Modification of ``rmw``, now passes ``rmw_publisher_allocation_t`` or ``rmw_subscription_allocation_t`` to ``rmw_publish`` and ``rmw_take``, respectively.
  Note that this argument can be ``NULL`` or ``nullptr``, which keeps existing Crystal behavior.

 * `rmw_publish <https://github.com/ros2/rmw/blob/dc7b2f49f1f961d6cf2c173adc54736451be8938/rmw/include/rmw/rmw.h#L310>`_
 * `rmw_take<https://github.com/ros2/rmw/blob/dc7b2f49f1f961d6cf2c173adc54736451be8938/rmw/include/rmw/rmw.h#L556>`_

Known Issues
------------

None yet.
