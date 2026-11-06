.. redirect-from::

    About-ROS-2-Parameters
    Concepts/About-ROS-2-Parameters
    Concepts/Basic/About-Parameters

.. meta::
   :contentType: about
   :experience: beginner
   :area: parameters, framework
   :distribution: {DISTRO}
   :product: {PRODUCT}

Parameters
==========

.. short-description::
   Parameters in ROS configure node behavior without requiring changes to the node's source code.
   This article explains how parameters are structured, declared, typed, updated, and monitored, helping you understand how to manage configuration across a ROS system.

.. showmeta::
   :order: area, contentType, experience
   :labels: area=Area, contentType=Content type, experience=Level

.. toctree::
   :maxdepth: 2
   :hidden:

   parameters/Working-with-parameters

.. contents:: Table of Contents
   :depth: 2
   :local:

Summary
-------

Parameters are configuration values stored by each node in the ROS graph.
Parameters are used to configure nodes at startup and during runtime, without changing the code.
You can set and change parameters using launch files, command-line arguments, YAML files, the ``ros2 param`` command, and the client library APIs.

Parameters in ROS
-----------------

Parameters in ROS are associated with individual nodes.
The lifetime of a parameter is tied to the lifetime of the node (though the node could implement some sort of persistence to reload values after restart).

Parameters are addressed by node name, node namespace, parameter name, and parameter namespace.
Providing a parameter namespace is optional.

Each parameter consists of the following:

* Key: A string which is the parameter name.

* Value: The value of the parameter.
  One of the following types: ``bool``, ``int64``, ``float64``, ``string``, ``byte[]``, ``bool[]``, ``int64[]``, ``float64[]`` or ``string[]``.

* Descriptor: An optional ``rcl_interfaces/msg/ParameterDescriptor`` that describes the parameter and its constraints, such as a text description, value ranges, type information, or making it read-only.
  By default, all descriptors are empty.

For example, every node has a ``use_sim_time`` parameter with a boolean value.
It defaults to ``false``, but can be set to ``true`` when running in simulation.

Setting and changing parameters
-------------------------------

In ROS, you can use multiple methods to set and change parameters.

At startup
   Initial parameter values can be set when launching nodes through the ROS launch facility.
   For more information about launch, see :doc:`../Developer-Tools/Launch/Launch-Main`.

   Initial parameter values can also be set when running a node, either through individual command-line arguments or through YAML files.
   See :ref:`NodeArgsParameters`.

At runtime
   For nodes that are already running, the ``ros2 param`` command is the primary way to interact with parameters.

   All client libraries also provide APIs to get, set, and react to parameter changes while a node is running.
   The following client libraries are core to ROS:

   * `rclcpp <https://index.ros.org/p/rclcpp/>`_: The C++ client library for ROS.
     To learn more, see :doc:`client-libraries/Working-with-Client-Libraries/Using-Parameters-In-A-Class-CPP` and :doc:`parameters/Working-with-parameters/Monitoring-For-Parameter-Changes-CPP`.

   * `rclpy <https://index.ros.org/p/rclpy/>`_: The Python client library for ROS.
     To learn more, see :doc:`client-libraries/Working-with-Client-Libraries/Using-Parameters-In-A-Class-Python` and :doc:`parameters/Working-with-parameters/Monitoring-For-Parameter-Changes-Python`.

To learn more about working with parameters, see :doc:`parameters/Working-with-parameters`.
For a hands-on tutorial with ROS parameters see :doc:`parameters/Working-with-parameters/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters`.

Parameter callbacks
^^^^^^^^^^^^^^^^^^^

A ROS node can optionally register three different types of callbacks to be informed when changes are happening to parameters.

* Pre-set parameter callback: When it is called, it can modify the ``Parameter`` list to change, add, or remove entries.
* Set parameter callback: This callback gives you the ability to inspect the upcoming change to the parameter and explicitly reject the change.
* Post-set parameter callback: This callback gives you the ability to react to changes from parameters that have successfully been accepted.

To learn how to register these callbacks, see :doc:`parameters/Working-with-parameters`.

Migrating from ROS 1
--------------------

The :doc:`Launch file migration guide <../Migration-and-Upgrades/Migrating-from-ROS1/Migrating-Launch-Files>` explains how to migrate ``param`` and ``rosparam`` launch tags from ROS 1 to ROS 2.

The :doc:`Migration guide <../Migration-and-Upgrades/Migrating-from-ROS1/Migrating-Parameters>` explains how to migrate parameters from ROS 1 to ROS 2.

Related content
---------------

* :doc:`parameters/Working-with-parameters`
* :doc:`parameters/Working-with-parameters/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters`
* :doc:`../Developer-Tools/Launch/Launch-Main`

FAQs
----

What does a parameter consist of?
   Each parameter has a key, a value, and an optional descriptor.
   The key is the parameter name, the value holds the data, and the descriptor can describe constraints such as making the parameter read-only.

What types can a parameter value have?
   A parameter value must be one of the following types: ``bool``, ``int64``, ``float64``, ``string``, ``byte[]``, ``bool[]``, ``int64[]``, ``float64[]``, or ``string[]``.

How can I set or change parameters?
   You can set initial values at startup through the ROS launch facility, or when you run a node with command-line arguments or YAML files.
   For nodes that are already running, use the ``ros2 param`` command or the client library APIs.

Can I change parameters while a node is running?
   Yes.
   Use ``ros2 param`` from the command line, or use the get, set, and reaction APIs in ``rclcpp`` or ``rclpy``.

What are parameter callbacks for?
   Parameter callbacks let a node respond when parameter changes are requested or accepted.
   For example, the set parameter callback can inspect an upcoming change and reject it.
