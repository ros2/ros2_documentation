
Passing ROS arguments to nodes via the command-line
===================================================

.. contents:: Table of Contents
   :depth: 1
   :local:


All ROS nodes take a set of arguments that allow various properties to be reconfigured.
Examples include configuring the name/namespace of the node, topic/service names used, and parameters on the node.

*Note: all features on this page are only available as of the ROS 2 Bouncy release.*

Name remapping
--------------

Names within a node (e.g. topics/services) can be remapped using the syntax ``<old name>:=<new name>``.
The name/namespace of the node itself can be remapped using ``__node:=<new node name>`` and ``__ns:=<new node namespace>``.

Note that these remappings are "static" remappings, in that they apply for the lifetime of the node.
"Dynamic" remapping of names after nodes have been started is not yet supported.

See `this design doc <http://design.ros2.org/articles/static_remapping.html>`__ for more details on remapping arguments (not all functionality is available yet).

Example
^^^^^^^

The following invocation will cause the ``talker`` node to be started under the node name ``my_talker``, publishing on the topic named ``my_topic`` instead of the default of ``chatter``.
The namespace, which must start with a forward slash, is set to ``/demo``, which means that topics are created in that namespace (``/demo/my_topic``), as opposed to globally (``/my_topic``).

.. code-block:: bash

   ros2 run demo_nodes_cpp talker __ns:=/demo __node:=my_talker chatter:=my_topic

Passing remapping arguments to specific nodes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If multiple nodes are being run within a single process (e.g. using `Composition <Composition>`), remapping arguments can be passed to a specific node using its name as a prefix.
For example, the following will pass the remapping arguments to the specified nodes:

.. code-block:: bash

   ros2 run composition manual_composition talker:__node:=my_talker listener:__node:=my_listener

Logger configuration
--------------------

See ``__log_level`` argument usage in `the logging page <logging-command-line-configuration-of-the-default-severity-level>`.

Parameters
----------

*Parameters support for Python nodes was added in Crystal. In Bouncy only C++ nodes are supported.*

Setting parameters from the command-line is currently supported in the form of yaml files.

`See here <https://github.com/ros2/rcl/tree/master/rcl_yaml_param_parser>`__ for examples of the yaml file syntax. As an example, save the following as ``demo_params.yaml``

.. code-block:: yaml

   talker:
       ros__parameters:
           some_int: 42
           a_string: "Hello world"
           some_lists:
               some_integers: [1, 2, 3, 4]
               some_doubles : [3.14, 2.718]

Then run the following:

.. code-block:: bash

   ros2 run demo_nodes_cpp talker __params:=demo_params.yaml

Other nodes will be able to retrieve the parameter values, e.g.:

.. code-block:: bash

   $ ros2 param list talker
     a_string
     some_int
     some_lists.some_doubles
     some_lists.some_integers
