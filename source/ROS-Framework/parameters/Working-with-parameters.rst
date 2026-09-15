.. meta::
   :contentType: about
   :experience: beginner
   :area: framework
   :distribution: {DISTRO}
   :product: {PRODUCT}

Working with parameters - how-to
================================

.. toctree::
  :maxdepth: 2
  :hidden:

  Working-with-parameters/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters
  Working-with-parameters/Using-ros2-param
  Working-with-parameters/Monitoring-For-Parameter-Changes-Python
  Working-with-parameters/Monitoring-For-Parameter-Changes-CPP

Parameters are configuration values stored by each node in the ROS graph.
This article describes how to interact with ROS parameters.
After reading this article you will be able to understand how to use parameters. 

**[Area: Parameters, Framework | Content-type: Concept | Experience: Beginner]**

.. contents:: Table of Contents
   :depth: 2
   :local:

Background
----------

Before you get started, familiarise yourself with what parameters are in ROS: :doc:`../About-Parameters`

Declaring parameters
--------------------

By default, a node needs to *declare* all of the parameters that it will accept during its lifetime.
This is so that the type and name of the parameters are well-defined at node startup time, which reduces the chances of misconfiguration later on.
See :doc:`../client-libraries/Working-with-Client-Libraries/Using-Parameters-In-A-Class-CPP` or :doc:`../client-libraries/Working-with-Client-Libraries/Using-Parameters-In-A-Class-Python` for tutorials on declaring and using parameters from a node.

For some types of nodes, not all of the parameters will be known ahead of time.
In these cases, the node can be instantiated with ``allow_undeclared_parameters`` set to ``true``, which will allow parameters to be get and set on the node even if they haven't been declared.

Setting parameter types
-----------------------

Supported parameter types are:
  * ``bool`` 
  * ``int64``
  * ``float64``
  * ``string``
  * ``byte[]``
  * ``bool[]``
  * ``int64[]``
  * ``float64[]``
  * ``string[]``

By default, attempts to change the type of a declared parameter at runtime will fail.
This prevents common mistakes, such as putting a boolean value into an integer parameter.

If a parameter needs to be multiple different types, and the code using the parameter can handle it, this default behavior can be changed.
The parameter should be declared using a ``ParameterDescriptor`` with the ``dynamic_typing`` member variable set to ``true``.

Interacting with parameters
---------------------------

ROS nodes can perform parameter operations using client libraries as described in :doc:`../client-libraries/Working-with-Client-Libraries/Using-Parameters-In-A-Class-CPP` or :doc:`../client-libraries/Working-with-Client-Libraries/Using-Parameters-In-A-Class-Python`.

External processes can perform parameter operations using parameter services that are created by default when a node is instantiated.
The parameter services that are created by default are:

+------------------------------------------+------------------------------------------------+------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Service                                  | Service type                                   | Input                              | Returns                                                                                                                 |
+==========================================+================================================+====================================+=========================================================================================================================+
| ``/node_name/describe_parameters``       | ``rcl_interfaces/srv/DescribeParameters``      | List of parameter names            | List of descriptors associated with the parameters                                                                      |
+------------------------------------------+------------------------------------------------+------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| ``/node_name/get_parameter_types``       | ``rcl_interfaces/srv/GetParameterTypes``       | List of parameter names            | List of parameter types associated with the parameters                                                                  |
+------------------------------------------+------------------------------------------------+------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| ``/node_name/get_parameters``            | ``rcl_interfaces/srv/GetParameters``           | List of parameter names            | List of parameter values associated with the parameters                                                                 |
+------------------------------------------+------------------------------------------------+------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| ``/node_name/list_parameters``           | ``rcl_interfaces/srv/ListParameters``          | List of parameter prefixes         | List of the available parameters with that prefix.                                                                      |
|                                          |                                                |                                    |                                                                                                                         |
|                                          |                                                |                                    | If the prefixes are empty, returns all parameters.                                                                      |
+------------------------------------------+------------------------------------------------+------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| ``/node_name/set_parameters``            | ``rcl_interfaces/srv/SetParameters``           | List of parameter names and values | Attempts to set the parameters on the node.                                                                             |
|                                          |                                                |                                    |                                                                                                                         |
|                                          |                                                |                                    | Returns a list of results from trying to set each parameter; some of them may have succeeded and some may have failed.  |                                                                                                                       
+------------------------------------------+------------------------------------------------+------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| ``/node_name/set_parameters_atomically`` | ``rcl_interfaces/srv/SetParametersAtomically`` | List of parameter names and values | Attempts to set the parameters on the node.                                                                             |
|                                          |                                                |                                    |                                                                                                                         |
|                                          |                                                |                                    | Returns a single result from trying to set all parameters, so if one failed, all of them failed.                        |                                                                                                 
+------------------------------------------+------------------------------------------------+------------------------------------+-------------------------------------------------------------------------------------------------------------------------+

Setting initial parameter values when launching nodes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Startup parameter values can be set when running the node through the ROS launch facility.
For information on how to specify parameters via launch, see :doc:`../../Developer-Tools/Launch/Using-ROS2-Launch-For-Large-Projects`.

Setting initial parameter values when running a node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Initial parameter values can be set when running the node either through individual command-line arguments, or through YAML files.
See :ref:`NodeArgsParameters` for examples on how to set initial parameter values.

Manipulating parameter values at runtime
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The ``ros2 param`` command is the primary way to interact with parameters for nodes that are already running.
``ros2 param`` uses the parameter service API to perform the various operations.
For details on how to use ``ros2 param``, see :doc:`Working-with-parameters/Using-ros2-param`.

In addition to the command-line interface, parameters can also be manipulated programmatically at runtime using ROS client libraries.
All client libraries provide APIs to get, set, and react to parameter changes while a node is running.

Setting parameter callbacks
---------------------------

A ROS 2 node can register callbacks to be informed when changes are happening to parameters.

To learn more about parameter callbacks, see :doc:`../About-Parameters`.

For a demo example of these callbacks in use, see `set_parameters_callback.cpp <https://github.com/ros2/demos/blob/{DISTRO}/demo_nodes_cpp/src/parameters/set_parameters_callback.cpp>`__.

Pre-set parameter callbacks
^^^^^^^^^^^^^^^^^^^^^^^^^^^

*Pre-set parameter* callbacks can be set by calling ``add_pre_set_parameters_callback`` from the node API.
This callback is passed a list of the ``Parameter`` objects that are being changed, and returns nothing.
When it is called, it can modify the ``Parameter`` list to change, add, or remove entries.
For example, if ``parameter2`` should change anytime that ``parameter1`` changes, that can be implemented with this callback.

Set parameter callbacks
^^^^^^^^^^^^^^^^^^^^^^^

*Set parameter* callbacks can be set by calling ``add_on_set_parameters_callback`` from the node API.
The callback is passed a list of immutable ``Parameter`` objects, and returns an ``rcl_interfaces/msg/SetParametersResult``.

.. note::
   It is important that *set parameter* callbacks have no side-effects.
   Because multiple *set parameter* callbacks can be chained, there is no way for an individual callback to know if a later callback will reject the update.
   If the individual callback were to make changes to the class it is in, for example, it could get out-of-sync with the actual parameter.
   To get a callback *after* a parameter has been successfully changed, use a post-set parameter callback.

Post-set parameter callbacks
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

*Post-set parameter* callbacks can be set by calling ``add_post_set_parameters_callback`` from the node API.
The callback is passed a list of immutable ``Parameter`` objects, and returns nothing.
