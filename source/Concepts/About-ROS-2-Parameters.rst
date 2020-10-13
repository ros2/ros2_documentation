.. _AboutParameters:

.. redirect-from::

    About-ROS-2-Parameters

About parameters in ROS 2
=========================

.. contents:: Table of Contents
   :local:

Overview
--------

ROS parameters are associated with ROS nodes.
Parameters are used to externally configure nodes at runtime (and during runtime).
The lifetime of a parameter is tied to the lifetime of the node (though the node could implement some sort of persistence to reload values after restart).

Parameters are addressed by node name, node namespace, parameter name, and parameter namespace.
Providing a parameter namespace is optional.

Each parameter consists of a key and a value, where the key is a string and the value could be one of the following types: bool, int64, float64, string, byte[], bool[], int64[], float64[] or string[]

For an hands-on tutorial with ROS parameters see :ref:`ROS2Params`.

Setting parameters
------------------

Parameters can be set in many different ways.
They can be set from the command-line, from launch files, or programmatically.
They can be set individually or together in a parameters file.

In order to set a parameter on a node, the node must first *declare* that it has that parameter.
See :ref:`CppParamNode` or :ref:`PythonParamNode` for tutorials on declaring and using parameters from a node.
These tutorials also demonstrate how to dynamically set a parameter with the ``ros2 param`` tool and from a launch file.

For examples on setting parameters from the command-line or from a parameters file, see :ref:`NodeArgsParameters`.

Migrating from ROS 1
--------------------

The :ref:`Launch file migration guide <MigratingLaunch>` explains how to migrate ``param`` and ``rosparam`` launch tags from ROS 1 to ROS 2.

The :ref:`YAML parameter file migration guide <yaml-ros1-ros2>` explains how to migrate parameter files from ROS 1 to ROS 2.
