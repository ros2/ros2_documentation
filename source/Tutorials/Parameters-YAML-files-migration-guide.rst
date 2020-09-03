Migrating YAML parameter files from ROS 1 to ROS2
=================================================

For ROS parameter files, there is a 1:1 correspondence with parameters and YAML format.

Example
~~~~~~~

.. code-block:: yaml

  string: 'foo'
  integer: 1234
  float: 1234.5
  boolean: true
  list: [1.0, mixed list]
  dictionary: {a: b, c: d}

This file will load a string named foo to the parameter server, an integer with value 1234 and so on. 
For the case of dictionaries, the elements of the dictionary are unpacked into individual elements of a namespace, that is, in this scenario, “a” will be loaded with value “b” and “c” will be loaded with value “d” for the namespace “dictionary”.

The parameter server uses XMLRPC types for the parameters, supported types include: 32-bit integers, booleans, strings, doubles, iso8601 dates, lists and base64-encoded binary data.

XML params in a launch file
~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: yaml

  string: "foo-value"
  int: 1
  float: 1.0
  g1: {string: “g1-foo-value”, int: 10, float: 10.0}
  g2: {string: “g2-foo-value”, int: 20, float: 20.0}

ROS 2 parameters
===========================
In the case of the ROS 2, parameters are hosted on nodes. Their lifetime is associated with the lifetime of the node where these are hosted (The node could implement some sort of persistence to reload values after restart).

Parameters are addressed by node name and parameter name, including its namespace if required.

Each parameter consists of a key and a value, where key is a string and the value could be one of the following types: bool, int64, float64, string, byte[], bool[], int64[], float64[] or string[]

The structure of a ROS 2 parameter looks as this:

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

Besides including node name and the key:value of the parameter, the structure of a parameter file requires including the keyword “ros__parameters” with a colon in the line after the node name. An extra namespace for the parameter can be used before the parameter itself.

ROS 2 Parameters YAML file example
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
