.. redirect-from::

    About-ROS-2-Parameters
    Concepts/About-ROS-2-Parameters
    Concepts/Basic/About-Parameters

Parameters
==========

.. toctree::
   :maxdepth: 2
   :hidden:

   parameters/Working-with-parameters

Parameters are configuration values stored by each node in the ROS graph.
This article describes parameters and their role in ROS.

**[Area: Parameters, Framework | Content-type: Concept | Experience: Beginner]**

.. contents:: Table of Contents
   :depth: 2
   :local:

Summary
-------

Parameters are configuration values stored by each node in the ROS graph.
Parameters are used to configure nodes at startup and during runtime, without changing the code.

The ``ros2 param`` command and client libraries allow you to set and change parameters.
The following client libraries are core to ROS:

 * `rclcpp <https://index.ros.org/p/rclcpp/>`_: The C++ client library for ROS.

 * `rclpy <https://index.ros.org/p/rclpy/>`_: The Python client library for ROS.

Parameters in ROS
-----------------

Parameters in ROS are associated with individual nodes.
Parameters are used to configure nodes at startup and during runtime, without changing the code.

Parameters are addressed by node name, node namespace (optional), and parameter name.

Each parameter consists of the following:

* Key: A string which is the parameter name.

* Value: The value of the parameter.
  One of the following types: ``bool``, ``int64``, ``float64``, ``string``, ``byte[]``, ``bool[]``, ``int64[]``, ``float64[]`` or ``string[]``.

* Descriptor: Optional string that allows you to specify a text description of the parameter and its constraints, such as making it read-only, specifying a range, and so on.
  By default, all descriptors are empty.
  Descriptors can be set to contain parameter descriptions, value ranges, type information, and additional constraints.

For example, every node stores a parameter with the key ``use_sim_time`` with a boolean value, but the ``/turtlesim`` node might have the value ``true``.

Setting and changing parameters
-------------------------------

In ROS, you can use multiple methods to set and change parameters:

* Set parameter values at startup
   Initial parameter values can be set when initializing the node through the ROS launch facility.
   For more information about launch, see :doc:`../Developer-Tools/Launch/Launch-Main`.

* Set values at runtime
   * Command line (CLI)
      Initial parameter values can be set when running the node either through individual command-line arguments, or through YAML files.
      For nodes that are already running, the ``ros2 param`` command is the primary way to interact with parameters.

   * Client libraries and APIs
      All client libraries provide APIs to get, set, and react to parameter changes while a node is running.
      The following client libraries are core to ROS:

      * **rclcpp**: C++ client library for ROS.
         To learn more, see :doc:`client-libraries/Working-with-Client-Libraries/Using-Parameters-In-A-Class-CPP` and :doc:`parameters/Working-with-parameters/Monitoring-For-Parameter-Changes-CPP`.

      * **rclpy**: Python client library for ROS.
         To learn more, see :doc:`client-libraries/Working-with-Client-Libraries/Using-Parameters-In-A-Class-Python` and :doc:`parameters/Working-with-parameters/Monitoring-For-Parameter-Changes-Python`.

To learn more about working with parameters, see :doc:`parameters/Working-with-parameters`.
For a hands-on tutorial with ROS parameters see :doc:`parameters/Working-with-parameters/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters`.

Parameter callbacks
^^^^^^^^^^^^^^^^^^^

A ROS node can optionally register three different types of callbacks to be informed when changes are happening to parameters.

* Pre-set parameter callback: When it is called, it can modify the ``Parameter`` list to change, add, or remove entries.
* Set parameter callback: This callback gives you the ability to inspect the upcoming change to the parameter and explicitly reject the change.
* Post-set parameter callback: This callback gives you the ability to react to changes from parameters that have successfully been accepted.

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
