.. redirect-from::

    Managed-Nodes
    Tutorials/Managed-Nodes
    Tutorials/Demos/Managed-Nodes

.. meta::
   :contentType: example
   :experience: expert
   :area: nodes, framework
   :distribution: {DISTRO}
   :product: {PRODUCT}

Managing node lifecycles - example
==================================

.. short-description::
   Managed lifecycles for nodes allow greater control over the state of the ROS system.
   This example uses a simple talker/listener pair of managed nodes to show how a managed lifecycle can be implemented and used.
   You can use the example to understand and experiment with managing nodes in this way.

.. showmeta::
   :order: area, contentType, experience
   :labels: area=Area, contentType=Content type, experience=Level

.. contents:: Contents
   :depth: 2
   :local:

Summary
-------

ROS 2 introduces the concept of managed nodes, also called lifecycle nodes.
These nodes can be used to ensure that resources are correctly initialised, activated, deactivated, and cleaned up as the node moves between lifecycle states.
A common use case is nodes that control hardware, where devices such as cameras, lidars, motor drivers, and other sensors and actuators must be started, configured, and shut down in a controlled order.

Using lifecycle nodes helps ensure hardware is only initialised when it is ready, and is safely released during shutdown or error recovery.
The following packages enable you to implement these managed nodes: `rclcpp_lifecycle <https://index.ros.org/p/rclcpp_lifecycle/>`__ (implementation library) and `lifecycle_msgs <https://index.ros.org/p/lifecycle_msgs/>`__ (interface definitions).
For a step-by-step tutorial on implementing a lifecycle node in C++, see :doc:`Writing a Lifecycle Node (C++) <Writing-a-Lifecycle-Node>`.

Prerequisites
-------------

See the :doc:`installation instructions <../../../Get-Started/Installation>` for details on installing ROS 2.

Example
-------

Access the example
^^^^^^^^^^^^^^^^^^

Information on how to run the example is here: `lifecycle_demo_launch.py <https://github.com/ros2/demos/blob/{REPOS_FILE_BRANCH}/lifecycle_py/launch/lifecycle_demo_launch.py>`__

Commentary
^^^^^^^^^^

For more information about how to run it and what's happening, see: `lifecycle README <https://github.com/ros2/demos/blob/{REPOS_FILE_BRANCH}/lifecycle/README.rst>`__

Related content
---------------

Tutorials
^^^^^^^^^

* :doc:`Writing a Lifecycle Node (C++) <Writing-a-Lifecycle-Node>`: Step-by-step tutorial on implementing a lifecycle node, managing states with the CLI, and automating transitions using launch files.

Packages and references
^^^^^^^^^^^^^^^^^^^^^^^

* `rclcpp_lifecycle <https://index.ros.org/p/rclcpp_lifecycle/>`__: Package containing the lifecycle node implementation.
* `lifecycle_msgs <https://index.ros.org/p/lifecycle_msgs/>`__: Package containing lifecycle message and service definitions.
* `lifecycle <https://docs.ros.org/en/{DISTRO}/p/lifecycle/>`__: Package containing demos for lifecycle implementation.
