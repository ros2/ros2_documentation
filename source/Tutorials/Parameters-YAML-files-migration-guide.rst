.. _yaml-ros1-ros2:

Migrating YAML parameter files from ROS 1 to ROS 2
==================================================

This tutorial describes how to adapt ROS 1 parameters files for ROS 2.

YAML file example
-----------------

YAML is used to write parameters files in both ROS 1 and ROS 2.
The main difference in ROS 2 is that node names must be used to address parameters.
In addition to the fully qualified node name, we use the key "ros__parameters" to signal the start of parameters for the node.


For example, here is a parameters file in ROS 1:

.. code-block:: yaml

   lidar_name: foo
   lidar_id: 10
   ports: [11312, 11311, 21311]
   debug: true

Let's assume that the first two parameters are for a node named ``/lidar_ns/lidar_node_name``, the second parameter is for a node named ``/imu``, and the last parameter we want to set on both nodes.

We would construct our ROS 2 parameters file as follows:

.. code-block:: yaml

   /lidar_ns:
     lidar_node_name:
       ros__parameters:
         lidar_name: foo
         id: 10
   imu:
     ros__parameters:
       ports: [2438, 2439, 2440]
   /**:
     ros__parameters:
       debug: true

Note the use of wildcards (``/**``) to indicate that the parameter ``debug`` should be set on any node in any namespace.

Feature parity
--------------

Some features of ROS 1 parameters files does not exist in ROS 2:

- Mixed types in a list is not supported yet (`related issue <https://github.com/ros2/rcl/issues/463>`_)
- ``deg`` and ``rad`` substitutions are not supported
