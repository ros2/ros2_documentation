.. redirect-from::

    Node-arguments
    Guides/Node-arguments
    Tutorials/Node-arguments

Passing ROS arguments to nodes via the command-line
===================================================

.. contents:: Table of Contents
   :depth: 1
   :local:


All ROS nodes take a set of arguments that allow various properties to be reconfigured.
Examples include configuring the name/namespace of the node, topic/service names used, and parameters on the node.
All ROS-specific arguments have to be specified after a ``--ros-args`` flag:


.. code-block:: bash

   ros2 run my_package node_executable --ros-args ...


For more details, see `this design doc <https://design.ros2.org/articles/ros_command_line_arguments.html>`__.

Name remapping
--------------

Names within a node (e.g. topics/services) can be remapped using the syntax ``-r <old name>:=<new name>``.
The name/namespace of the node itself can be remapped using ``-r __node:=<new node name>`` and ``-r __ns:=<new node namespace>``.


Note that these remappings are "static" remappings, in that they apply for the lifetime of the node.
"Dynamic" remapping of names after nodes have been started is not yet supported.

See `this design doc <https://design.ros2.org/articles/static_remapping.html>`__ for more details on remapping arguments (not all functionality is available yet).

Example
^^^^^^^

The following invocation will cause the ``talker`` node to be started under the node name ``my_talker``, publishing on the topic named ``my_topic`` instead of the default of ``chatter``.
The namespace, which must start with a forward slash, is set to ``/demo``, which means that topics are created in that namespace (``/demo/my_topic``), as opposed to globally (``/my_topic``).

.. code-block:: bash

  ros2 run demo_nodes_cpp talker --ros-args -r __ns:=/demo -r __node:=my_talker -r chatter:=my_topic

Passing remapping arguments to specific nodes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If multiple nodes are being run within a single process (e.g. using :doc:`Composition <../Concepts/Intermediate/About-Composition>`), remapping arguments can be passed to a specific node using its name as a prefix.
For example, the following will pass the remapping arguments to the specified nodes:

.. code-block:: bash

  ros2 run composition manual_composition --ros-args -r talker:__node:=my_talker -r listener:__node:=my_listener


The following example will both change the node name and remap a topic (node and namespace changes are always applied *before* topic remapping):

.. code-block:: bash

  ros2 run composition manual_composition --ros-args -r talker:__node:=my_talker -r my_talker:chatter:=my_topic -r listener:__node:=my_listener -r my_listener:chatter:=my_topic


Logger configuration
--------------------

The per-node logging level can be specified using the ``--log-level`` command line argument.
The executable log file name prefix, which includes all nodes in the executable, can be specified using ``--log-file-name`` command line argument.
For more information please see :doc:`the logging page <../Tutorials/Demos/Logging-and-logger-configuration>`.

Parameters
----------

.. _NodeArgsParameters:

Setting parameters directly from the command line
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You can set parameters directly from the command line using the following syntax:

.. code-block:: bash

  ros2 run package_name executable_name --ros-args -p param_name:=param_value

As an example, you can run:

.. code-block:: bash

  ros2 run demo_nodes_cpp parameter_blackboard --ros-args -p some_int:=42 -p "a_string:=Hello world" -p "some_lists.some_integers:=[1, 2, 3, 4]" -p "some_lists.some_doubles:=[3.14, 2.718]"

Other nodes will be able to retrieve the parameter values, e.g.:

.. code-block:: bash

  $ ros2 param list parameter_blackboard
  a_string
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  some_int
  some_lists.some_doubles
  some_lists.some_integers
  use_sim_time

Setting parameters from YAML files
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Parameters can be set from the command-line in the form of yaml files.

`See here <https://github.com/ros2/rcl/tree/{REPOS_FILE_BRANCH}/rcl_yaml_param_parser>`__ for examples of the yaml file syntax.

As an example, save the following as ``demo_params.yaml``:

.. code-block:: yaml

  parameter_blackboard:
      ros__parameters:
          some_int: 42
          a_string: "Hello world"
          some_lists:
              some_integers: [1, 2, 3, 4]
              some_doubles : [3.14, 2.718]

  /**:
    ros__parameters:
      wildcard_full: "Full wildcard for any namespaces and any node names"

  /**/parameter_blackboard:
    ros__parameters:
      wildcard_namespace: "Wildcard for a specific node name under any namespace"

  /*:
    ros__parameters:
      wildcard_nodename_root_namespace: "Wildcard for any node names, but only in root namespace"


.. note::

   Wildcards can be used for node names and namespaces.
   ``*`` matches a single token delimited by slashes (``/``).
   ``**`` matches zero or more tokens delimited by slashes.
   Partial matches are not allowed (e.g. ``foo*``).


Then either declare the parameters within your node with `declare_parameter <http://docs.ros.org/en/{DISTRO}/p/rclcpp/generated/classrclcpp_1_1Node.html#_CPPv4N6rclcpp4Node17declare_parameterERKNSt6stringERKN6rclcpp14ParameterValueERKN14rcl_interfaces3msg19ParameterDescriptorEb>`__  or `declare_parameters <http://docs.ros.org/en/{DISTRO}/p/rclcpp/generated/classrclcpp_1_1Node.html#_CPPv4I0EN6rclcpp4Node18declare_parametersENSt6vectorI10ParameterTEERKNSt6stringERKNSt3mapINSt6stringENSt4pairI10ParameterTN14rcl_interfaces3msg19ParameterDescriptorEEEEEb>`__, or `set the node to automatically declare parameters <http://docs.ros.org/en/{DISTRO}/p/rclcpp/generated/classrclcpp_1_1NodeOptions.html#_CPPv4NK6rclcpp11NodeOptions47automatically_declare_parameters_from_overridesEv>`__ if they were passed in via a command line override.

Then run the following:

.. code-block:: bash

  ros2 run demo_nodes_cpp parameter_blackboard --ros-args --params-file demo_params.yaml


Other nodes will be able to retrieve the parameter values, e.g.:

.. code-block:: bash

  $ ros2 param list parameter_blackboard
  a_string
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  some_int
  some_lists.some_doubles
  some_lists.some_integers
  use_sim_time
  wildcard_full
  wildcard_namespace
  wildcard_nodename_root_namespace
