.. redirect-from::

    Managed-Nodes
    Tutorials/Managed-Nodes

Managing nodes with managed lifecycles - example
================================================

.. centered:: **Managed lifecycles for nodes allow greater control over the state of the ROS system.
   This example uses a simple talker/listener pair of managed nodes to show how a managed lifecycle can be implemented and used.
   You can use the example to understand and experiment with managing nodes in this way.**

::

   Area: ROS-framework | Content-type: example | Experience: expert

.. contents:: Contents
   :depth: 2
   :local:

Summary
-------

ROS 2 introduces the concept of managed nodes, also called lifecycle nodes.
These nodes can be used to ensure that resources are correctly initialized, activated, deactivated, and cleaned up as the node moves between lifecycle states.
The following packages enable you to implement these managed nodes: `rclcpp_lifecycle <https://index.ros.org/p/rclcpp_lifecycle/>`__ (implementation library) and `lifecycle_msgs <https://index.ros.org/p/lifecycle_msgs/>`__ (interface definitions).

Prerequisites
-------------

Install :doc:`ROS <../../Installation>`.

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

Packages/reference:

* `rclcpp_lifecycle <https://index.ros.org/p/rclcpp_lifecycle/>`_ (implementation library): Package containing a prototype for lifecycle implementation.
* `lifecycle_msgs <https://index.ros.org/p/lifecycle_msgs/>`_ (interface definitions): Package containing some lifecycle related message and service definitions.
* `lifecycle <https://docs.ros.org/en/{DISTRO}/p/lifecycle/>`_: Package containing demos for lifecycle implementation.
