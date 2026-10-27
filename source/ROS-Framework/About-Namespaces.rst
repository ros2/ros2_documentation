.. _about-namespaces:

ROS 2 Namespaces
================

.. short-description::

   Learn how ROS 2 namespaces organize node and topic names using Turtlesim.

A **namespace** organizes ROS 2 names into a hierarchy. It helps distinguish nodes and topics when running multiple instances of the same application.

In this tutorial, you will use Turtlesim to explore namespaces, compare relative and absolute names, and communicate across namespaces using remapping.

.. contents:: Contents
   :local:
   :depth: 2

Prerequisites
-------------

You will need the ``turtlesim`` package installed. See :doc:`Introducing Turtlesim <../Get-Started/Introducing-Turtlesim/Introducing-Turtlesim>` for installation instructions.

Source your ROS 2 environment in each terminal you use.

Discovering Node Names
----------------------

Open a terminal and run:

.. code-block:: bash

   ros2 run turtlesim turtlesim_node

A Turtlesim window should appear. In another terminal, run:

.. code-block:: bash

   ros2 node list

The output should include:

.. code-block:: text

   /turtlesim

The node runs in the root namespace, ``/``, so its fully qualified name is ``/turtlesim``.

Running a Node in a Namespace
-----------------------------

Stop the simulator with ``Ctrl+C`` and restart it in the ``/robot1`` namespace:

.. code-block:: bash

   ros2 run turtlesim turtlesim_node --ros-args -r __ns:=/robot1

Run ``ros2 node list`` again. The node should now appear as:

.. code-block:: text

   /robot1/turtlesim

The node name is still ``turtlesim``; the ``/robot1`` prefix identifies its namespace.

Namespaces also affect relative topic names. Let's inspect them.

Understanding Topic Names
-------------------------

With Turtlesim running in ``/robot1``, run:

.. code-block:: bash

   ros2 topic list

Look for topics such as:

.. code-block:: text

   /robot1/turtle1/cmd_vel
   /robot1/turtle1/pose

Turtlesim uses ``turtle1/cmd_vel`` to receive velocity commands. Because this is a relative topic name, ROS 2 resolves it using the node's namespace, producing ``/robot1/turtle1/cmd_vel``.

Running Multiple Instances
---------------------------

You can run another Turtlesim instance in a different namespace. Keep the first instance running and open a new terminal:

.. code-block:: bash

   ros2 run turtlesim turtlesim_node --ros-args -r __ns:=/robot2

Run ``ros2 node list`` and ``ros2 topic list`` again. You should see names from both namespaces, including:

.. code-block:: text

   /robot1/turtlesim
   /robot2/turtlesim

   /robot1/turtle1/cmd_vel
   /robot2/turtle1/cmd_vel

Both instances use the same relative topic name, but it resolves to a different fully qualified name in each namespace. This keeps their topics separate.

Relative and Absolute Names
---------------------------

A **relative name** is resolved using the node's namespace. For example, ``turtle1/cmd_vel`` in ``/robot1`` resolves to ``/robot1/turtle1/cmd_vel``.

An **absolute name** starts with ``/`` and is resolved from the root of the ROS 2 name hierarchy. For example, ``/robot2/turtle1/cmd_vel`` remains unchanged, even when used by a node in ``/robot1``.

Use relative names when you want names to follow a node's namespace, and absolute names when you need to refer to a specific name.

Remapping and Cross-Namespace Communication
--------------------------------------------

Namespaces organize names, while **remapping** changes a name to another name. Together, they let nodes communicate across namespaces without changing their source code.

With both Turtlesim instances still running, open a third terminal and start the keyboard controller in ``/robot1``, remapping its velocity topic to the topic used by the turtle in ``/robot2``:

.. code-block:: bash

   ros2 run turtlesim turtle_teleop_key --ros-args -r __ns:=/robot1 -r turtle1/cmd_vel:=/robot2/turtle1/cmd_vel

Click inside the controller terminal and use the arrow keys.

**Observe:** The turtle in ``/robot2`` should move.

The controller's topic would normally resolve to ``/robot1/turtle1/cmd_vel``. Remapping directs its messages to ``/robot2/turtle1/cmd_vel``, which the second Turtlesim instance subscribes to.

This demonstrates that namespaces do not prevent communication between nodes. Publishers and subscribers can communicate across namespaces when they use the same fully qualified topic name and compatible message types.

You can also remap a node's name. For example, stop the ``/robot1`` Turtlesim instance and restart it with a different node name:

.. code-block:: bash

   ros2 run turtlesim turtlesim_node --ros-args -r __ns:=/robot1 -r __node:=simulator

Run ``ros2 node list``. The node should now appear as ``/robot1/simulator``.

Here, ``__ns`` sets the namespace, while ``__node`` changes the node name.

Summary
-------
In this tutorial, we learned how to:

- Place nodes in separate namespaces.
- Distinguish relative names from absolute names.
- Use remapping to change node and topic names.
- Communicate across namespaces by remapping a topic.

Related content
---------------

- :doc:`About ROS 2 Nodes <About-Nodes>` — learn more about nodes and their names.
- :doc:`Understanding ROS 2 Topics <interfaces/topics/Understanding-ROS2-Topics/Understanding-ROS2-Topics>` — learn how nodes exchange messages through topics.
- :doc:`Passing ROS arguments to nodes via the command line <../Developer-Tools/Launch/Node-arguments>` — explore namespace and remapping arguments in more detail.
- `ROS 2 Topic and Service Names <https://design.ros2.org/articles/topic_and_service_names.html>`_ — see the naming rules for topics and services.