.. redirect-from::

    Node-arguments

Passing ROS arguments to nodes via the command-line
===================================================

.. contents:: Table of Contents
   :depth: 1
   :local:


All ROS nodes take a set of arguments that allow various properties to be reconfigured.
Examples include configuring the name/namespace of the node, topic/service names used, and parameters on the node.
All ros specific arguments have to be specified after a ``--ros-args`` flag:


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

If multiple nodes are being run within a single process (e.g. using `Composition <Composition>`), remapping arguments can be passed to a specific node using its name as a prefix.
For example, the following will pass the remapping arguments to the specified nodes:

.. code-block:: bash

  ros2 run composition manual_composition --ros-args -r talker:__node:=my_talker -r listener:__node:=my_listener


The following example will both change the node name and remap a topic (node and namespace changes are always applied *before* topic remapping):

.. code-block:: bash

  ros2 run composition manual_composition --ros-args -r talker:__node:=my_talker -r my_talker:chatter:=my_topic -r listener:__node:=my_listener -r my_listener:chatter:=my_topic


Logger configuration
--------------------

See ``--log-level`` argument usage in `the logging page <Logging-and-logger-configuration>`.

Parameters
----------

.. _NodeArgsParameters:

Setting parameters directly in the command line
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
  some_int
  some_lists.some_doubles
  some_lists.some_integers

Setting parameters from YAML files
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Parameters can be set from the command-line in the form of yaml files.

`See here <https://github.com/ros2/rcl/tree/master/rcl_yaml_param_parser>`__ for examples of the yaml file syntax.

As an example, save the following as ``demo_params.yaml``:

.. code-block:: yaml

  parameter_blackboard:
      ros__parameters:
          some_int: 42
          a_string: "Hello world"
          some_lists:
              some_integers: [1, 2, 3, 4]
              some_doubles : [3.14, 2.718]

Then either declare the parameters within your node with ``declare_parameter``  or ``declare_parameters`` (see `documentation <https://docs.ros2.org/foxy/api/rclcpp/classrclcpp_1_1Node.html#a222633623e5c933b7953e5718ec3649a>`__ for function signatures), or `set the node to automatically declare parameters <https://docs.ros2.org/foxy/api/rclcpp/classrclcpp_1_1NodeOptions.html#a094ceb7af7c9b358ec007a4b8e14d40d>`__ if they were passed in via a command line override.

Then run the following:

.. code-block:: bash

  ros2 run demo_nodes_cpp parameter_blackboard --ros-args --params-file demo_params.yaml


Other nodes will be able to retrieve the parameter values, e.g.:

.. code-block:: bash

  $ ros2 param list parameter_blackboard
  a_string
  some_int
  some_lists.some_doubles
  some_lists.some_integers
