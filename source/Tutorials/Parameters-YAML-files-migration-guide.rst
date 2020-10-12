.. _yaml-ros1-ros2:

Migrating YAML parameter files from ROS 1 to ROS 2
==================================================

The following example shows the same YAML parameter file written for ROS 1 and ROS 2.

YAML file example
-----------------

.. tabs::

  .. group-tab:: ROS 2

    .. code-block:: yaml

      lidar_ns:

        lidar_1:
          ros__parameters:
            id: 10
            name: front_lidar
            ports: [2438, 2439, 2440]
            driver1:
              dx: 4.56
              dy: 2.30
              fr_sensor_specs: [12, 3, 0, 7]
              bk_sensor_specs: [12.1, -2.3, 5.2, 9.0]
              is_front: true
            driver2:
              dx: 1.23
              dy: 0.45

        lidar_2:
          ros__parameters:
            id: 11
            name: back_lidar
            dy1: 0.003
            is_back: false
            driver:
              dz: 7.89

  .. group-tab:: ROS 1

    .. code-block:: yaml



Explanation of differences
--------------------------

-
-

Feature parity
--------------


.. list-table::
   :header-rows: 1

   * - Feature
     - ROS 1
     - ROS 2
   * -
     -
     -
   * -
     -
     -
   * -
     -
     -
   * -
     -
     -
