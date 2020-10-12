.. _AboutParameters:

.. redirect-from::

    About-ROS-2-Parameters

About parameters in ROS 2
=========================

.. contents:: Table of Contents
   :local:

Overview
--------

In ROS 2, parameters are hosted on nodes.
Their lifetime is associated with the lifetime of the node where they are hosted (the node could implement some sort of persistence to reload values after restart).

Parameters are addressed by node name and parameter name, including its namespace if required.

Each parameter consists of a key and a value, where key is a string and the value could be one of the following types: bool, int64, float64, string, byte[], bool[], int64[], float64[] or string[]

The structure of a ROS 2 parameter looks like this:

.. code-block:: yaml

  <node_namespace_string>: # optional
    <node1_name>:
      ros__parameters:
        <field_name>: <field_value>
        <parameter_namespace_string>: # optional
          <field1_name>: <field1_value>
          <field2_name>: <field2_value>
    <node2_name>:
      ros__parameters:
        <field_name>: <field_value>
        <parameter_namespace_string>: # optional
          <field1_name>: <field1_value>
          <field2_name>: <field2_value>

Besides including node name and the key:value of the parameter, the structure of a parameter file requires including the keyword “ros__parameters” with a colon in the line after the node name.
An extra namespace for the parameter can be used before the parameter itself.

Setting up parameters
---------------------

Parameters can be set in launch files or in parameter files.
A parameter file can only be written in `YAML <https://yaml.org/spec/>`__.

XML params in a launch file
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: xml

  <launch>

    <param name="string" value="foo-value" />
    <param name="int" value="1" />
    <param name="float" value="1.0" />

    <group ns="g1">
      <param name="string" value="g1-foo-value" />
      <param name="int" value="10" />
      <param name="float" value="10.0" />
    </group>

    <group ns="g2">
      <param name="string" value="g2-foo-value" />
      <param name="int" value="20" />
      <param name="float" value="20.0" />
    </group>

    <test test-name="rosparam_command_line_online" pkg="test_rosparam" type="check_rosparam_command_line_online.py" />
    <test test-name="test_rosparam" pkg="test_rosparam" type="check_rosparam.py" />

    </launch>


YAML file with equivalent information
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: yaml

  string: "foo-value"
  int: 1
  float: 1.0
  g1: {string: “g1-foo-value”, int: 10, float: 10.0}
  g2: {string: “g2-foo-value”, int: 20, float: 20.0}

Migrating parameters
--------------------

The :ref:`Launch file migration guide <MigratingLaunch>` explains how to migrate ``param`` and ``rosparam`` tags from ROS 1 to ROS 2.

The :ref:`YAML parameter file migration guide <yaml-ros1-ros2>` explains how to migrate parameter files from ROS 1 to ROS 2.

Using parameters
----------------

See the following tutorials on parameters:

- :ref:`ROS2Params`
- :ref:`CppParamNode`
- :ref:`PythonParamNode`
