.. redirect-from::

    About-ROS-2-Parameters
    Concepts/About-ROS-2-Parameters

Parameters
==========

.. contents:: Table of Contents
   :local:

Overview
--------

Parameters in ROS 2 are associated with individual nodes.
Parameters are used to configure nodes at startup (and during runtime), without changing the code.
The lifetime of a parameter is tied to the lifetime of the node (though the node could implement some sort of persistence to reload values after restart).

Parameters are addressed by node name, node namespace, parameter name, and parameter namespace.
Providing a parameter namespace is optional.

Each parameter consists of a key, a value, and a descriptor.
The key is a string and the value is one of the following types: ``bool``, ``int64``, ``float64``, ``string``, ``byte[]``, ``bool[]``, ``int64[]``, ``float64[]`` or ``string[]``.
By default all descriptors are empty, but can contain parameter descriptions, value ranges, type information, and additional constraints.

For an hands-on tutorial with ROS parameters see :doc:`../../Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters`.

Parameters background
---------------------

Declaring parameters
^^^^^^^^^^^^^^^^^^^^

By default, a node needs to *declare* all of the parameters that it will accept during its lifetime.
This is so that the type and name of the parameters are well-defined at node startup time, which reduces the chances of misconfiguration later on.
See :doc:`../../Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP` or :doc:`../../Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python` for tutorials on declaring and using parameters from a node.

For some types of nodes, not all of the parameters will be known ahead of time.
In these cases, the node can be instantiated with ``allow_undeclared_parameters`` set to ``true``, which will allow parameters to be get and set on the node even if they haven't been declared.

Parameter types
^^^^^^^^^^^^^^^

Each parameter on a ROS 2 node has one of the pre-defined parameter types as mentioned in the Overview.
By default, attempts to change the type of a declared parameter at runtime will fail.
This prevents common mistakes, such as putting a boolean value into an integer parameter.

If a parameter needs to be multiple different types, and the code using the parameter can handle it, this default behavior can be changed.
When the parameter is declared, it should be declared using a ``ParameterDescriptor`` with the ``dynamic_typing`` member variable set to ``true``.

Parameter callbacks
^^^^^^^^^^^^^^^^^^^

A ROS 2 node can register two different types of callbacks to be informed when changes are happening to parameters.
Both of the callbacks are optional.

The first is known as a "set parameter" callback, and can be set by calling ``add_on_set_parameters_callback`` from the node API.
The callback is passed a list of immutable ``Parameter`` objects, and returns an ``rcl_interfaces/msg/SetParametersResult``.
The main purpose of this callback is to give the user the ability to inspect the upcoming change to the parameter and explicitly reject the change.

.. note::
   It is important that "set parameter" callbacks have no side-effects.
   Since multiple "set parameter" callbacks can be chained, there is no way for an individual callback to know if a later callback will reject the update.
   If the individual callback were to make changes to the class it is in, for instance, it may get out-of-sync with the actual parameter.
   To get a callback *after* a parameter has been successfully changed, see the next type of callback below.

The second type of callback is known as an "on parameter event" callback, and can be set by calling ``on_parameter_event`` from the parameter client APIs.
The callback is passed an ``rcl_interfaces/msg/ParameterEvent`` object, and returns nothing.
This callback will be called after all parameters in the input event have been declared, changed, or deleted.
The main purpose of this callback is to give the user the ability to react to changes from parameters that have successfully been accepted.

Interacting with parameters
---------------------------

ROS 2 nodes can perform parameter operations through node APIs as described in :doc:`../../Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP` or :doc:`../../Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python`.
External processes can perform parameter operations via parameter services that are created by default when a node is instantiated.
The services that are created by default are:

* ``/node_name/describe_parameters``: Uses a service type of ``rcl_interfaces/srv/DescribeParameters``.
  Given a list of parameter names, returns a list of descriptors associated with the parameters.
* ``/node_name/get_parameter_types``: Uses a service type of ``rcl_interfaces/srv/GetParameterTypes``.
  Given a list of parameter names, returns a list of parameter types associated with the parameters.
* ``/node_name/get_parameters``: Uses a service type of ``rcl_interfaces/srv/GetParameters``.
  Given a list of parameter names, returns a list of parameter values associated with the parameters.
* ``/node_name/list_parameters``: Uses a service type of ``rcl_interfaces/srv/ListParameters``.
  Given an optional list of parameter prefixes, returns a list of the available parameters with that prefix.  If the prefixes are empty, returns all parameters.
* ``/node_name/set_parameters``: Uses a service type of ``rcl_interfaces/srv/SetParameters``.
  Given a list of parameter names and values, attempts to set the parameters on the node.  Returns a list of results from trying to set each parameter; some of them may have succeeded and some may have failed.
* ``/node_name/set_parameters_atomically``: Uses a service type of ``rcl_interfaces/srv/SetParametersAtomically``.
  Given a list of parameter names and values, attempts to set the parameters on the node.  Returns a single result from trying to set all parameters, so if one failed, all of them failed.

Setting initial parameter values when running a node
----------------------------------------------------

Initial parameter values can be set when running the node either through individual command-line arguments, or through YAML files.
See :ref:`NodeArgsParameters` for examples on how to set initial parameter values.

Setting initial parameter values when launching nodes
-----------------------------------------------------

Initial parameter values can also be set when running the node through the ROS 2 launch facility.
See :doc:`this document <../../Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects>` for information on how to specify parameters via launch.

Manipulating parameter values at runtime
----------------------------------------

The ``ros2 param`` command is the general way to interact with parameters for nodes that are already running.
``ros2 param`` uses the parameter service API as described above to perform the various operations.
See :doc:`this how-to guide <../../How-To-Guides/Using-ros2-param>` for details on how to use ``ros2 param``.

Migrating from ROS 1
--------------------

The :doc:`Launch file migration guide <../../How-To-Guides/Migrating-from-ROS1/Migrating-Launch-Files>` explains how to migrate ``param`` and ``rosparam`` launch tags from ROS 1 to ROS 2.

The :doc:`YAML parameter file migration guide <../../How-To-Guides/Migrating-from-ROS1/Migrating-Parameters>` explains how to migrate parameter files from ROS 1 to ROS 2.

In ROS 1, the ``roscore`` acted like a global parameter blackboard where all nodes could get and set parameters.
Since there is no central ``roscore`` in ROS 2, that functionality no longer exists.
The recommended approach in ROS 2 is to use per-node parameters that are closely tied to the nodes that use them.
If a global blackboard is still needed, it is possible to create a dedicated node for this purpose.
ROS 2 ships with one in the ``ros-{DISTRO}-demo-nodes-cpp`` package called ``parameter_blackboard``; it can be run with:

.. code-block:: console

   ros2 run demo_nodes_cpp parameter_blackboard

The code for the ``parameter_blackboard`` is `here <https://github.com/ros2/demos/blob/{REPOS_FILE_BRANCH}/demo_nodes_cpp/src/parameters/parameter_blackboard.cpp>`__.
