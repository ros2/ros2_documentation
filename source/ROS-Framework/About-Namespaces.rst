.. _about-namespaces:

ROS 2 Namespaces
================

.. short-description::
   Learn how ROS 2 namespaces organize node and topic names using Turtlesim.

A **namespace** organizes ROS 2 names into a hierarchy. It helps distinguish nodes and the names they use for communication, especially when running multiple instances of the same application.

For example, imagine running two robots, each with its own controller. Both controllers might use the same topic name to receive velocity commands. Namespaces allow their topic names to be separated.

In this tutorial, you will use Turtlesim to explore how namespaces work. You will first inspect a node without a custom namespace, then introduce namespaces and observe how they affect node and topic names.

Prerequisites
-------------

You will need the ``turtlesim`` package installed. See :doc:`Introducing Turtlesim <../Get-Started/Introducing-Turtlesim/Introducing-Turtlesim>` for installation instructions.

Source your ROS 2 environment in each terminal you use.

Start by running Turtlesim and inspecting its node name.

Discovering Node Names
----------------------

Open a terminal and run:

.. code-block:: bash

   ros2 run turtlesim turtlesim_node

A Turtlesim window should appear. The simulator runs as a ROS 2 node.

Open another terminal, source your ROS 2 environment, and run:

.. code-block:: bash

   ros2 node list

You should see:

.. code-block:: text

   /turtlesim

The output shows the fully qualified name of the node. A fully qualified node name identifies a node within the ROS 2 name hierarchy.

In this example, the node runs in the root namespace, represented by ``/``. Its node name is ``turtlesim``, so its fully qualified name is ``/turtlesim``.

Now, let's run the same node in a different namespace and compare the result.

Running a Node in a Namespace
-----------------------------

Stop the current simulator with ``Ctrl+C``. Then run Turtlesim with the namespace ``/robot1``:

.. code-block:: bash

   ros2 run turtlesim turtlesim_node --ros-args -r __ns:=/robot1

A new Turtlesim window should appear. The argument ``__ns:=/robot1`` sets the node's namespace to ``/robot1``.

Run the following command in another terminal:

.. code-block:: bash

   ros2 node list

You should now see:

.. code-block:: text

   /robot1/turtlesim

Compare this with the previous result, ``/turtlesim``. The node is still named ``turtlesim``, but its fully qualified name now includes the ``/robot1`` namespace.

The name can be broken down into two parts:

- ``/robot1`` is the namespace.
- ``turtlesim`` is the node name.

This demonstrates how a namespace organizes a node within the ROS 2 name hierarchy without requiring a change to the node's name.

Namespaces also affect the relative topic names a node uses. Let's inspect the topics to see this in practice.

Understanding Topic Names
-------------------------

While Turtlesim is running in the ``/robot1`` namespace, run:

.. code-block:: bash

   ros2 topic list

Look for topic names such as:

.. code-block:: text

   /robot1/turtle1/cmd_vel
   /robot1/turtle1/pose

The list may contain additional topics.

Consider ``/robot1/turtle1/cmd_vel``. Turtlesim uses this topic to receive velocity commands for its turtle.

The name contains the namespace prefix ``/robot1``. The topic's relative name is ``turtle1/cmd_vel``. Because the node is running in the ``/robot1`` namespace, ROS 2 resolves that relative name to ``/robot1/turtle1/cmd_vel``.

The same principle applies to other relative topic names used by the node.

So far, we have seen how a namespace changes the names used by one Turtlesim instance. Next, let's see why this is useful when running multiple instances of the same application.

Running Multiple Turtlesim Instances
------------------------------------

Suppose you want to simulate two robots. Each robot should have its own set of nodes and topics.

You can run two Turtlesim instances in different namespaces to keep their names separate.

Keep the first instance running in ``/robot1``. Open another terminal and run:

.. code-block:: bash

   ros2 run turtlesim turtlesim_node --ros-args -r __ns:=/robot2

A second Turtlesim window should appear.

Now run:

.. code-block:: bash

   ros2 node list

The output should include:

.. code-block:: text

   /robot1/turtlesim
   /robot2/turtlesim

Both nodes have the same node name, ``turtlesim``, but they belong to different namespaces. Their fully qualified names are therefore different.

Next, inspect the topic names:

.. code-block:: bash

   ros2 topic list

Look for the following topics:

.. code-block:: text

   /robot1/turtle1/cmd_vel
   /robot1/turtle1/pose
   /robot2/turtle1/cmd_vel
   /robot2/turtle1/pose

Each Turtlesim instance uses its own set of namespaced topics. For example, ``/robot1/turtle1/cmd_vel`` and ``/robot2/turtle1/cmd_vel`` are different topic names.

This separation is useful when multiple robots use similar node and topic names. Each instance can use the same relative names while resolving them within its own namespace.

But why does the namespace prefix appear on some names and not others? To understand this, we need to distinguish relative names from absolute names.

Relative and Absolute Names
---------------------------

ROS 2 resolves names differently depending on whether they are relative or absolute.

A **relative name** does not start with ``/``. ROS 2 resolves it using the node's namespace.

For example, the relative topic name ``turtle1/cmd_vel`` resolves to:

.. code-block:: text

   /robot1/turtle1/cmd_vel

when used by a node in the ``/robot1`` namespace. For a node in ``/robot2``, the same relative name resolves to:

.. code-block:: text

   /robot2/turtle1/cmd_vel

An **absolute name** starts with ``/``. It specifies a name from the root of the name hierarchy and is not prefixed by the node's namespace during ordinary name resolution.

For example, the absolute name ``/shared/cmd_vel`` remains ``/shared/cmd_vel`` even when used by a node in the ``/robot1`` namespace.

This distinction explains the behavior we observed earlier: relative topic names are resolved using the node's namespace, while absolute topic names specify their location directly.

Namespaces help make nodes reusable because their relative topic names can resolve differently depending on where the nodes are placed in the name hierarchy.

Namespaces and Remapping
------------------------

Namespaces organize relative names under a common prefix. **Remapping**, on the other hand, changes a particular name to another name.

We can see the difference by changing the node name used by Turtlesim.

Stop one of the running instances with ``Ctrl+C``. Then start Turtlesim with both a namespace and a node-name remapping:

.. code-block:: bash

   ros2 run turtlesim turtlesim_node --ros-args -r __ns:=/robot1 -r __node:=simulator

Run:

.. code-block:: bash

   ros2 node list

The node should now appear as:

.. code-block:: text

   /robot1/simulator

The two arguments have different purposes:

- ``__ns:=/robot1`` sets the node's namespace.
- ``__node:=simulator`` changes the node's name.

The resulting fully qualified node name combines the namespace and the remapped node name.

This example shows that namespaces and remapping can be used together. Namespaces organize nodes within a hierarchy, while remapping allows individual names to be changed without modifying the node's source code.

Summary
-------

In this tutorial, you explored namespaces using Turtlesim. You learned how to:

- Inspect a node's fully qualified name using ``ros2 node list``.
- Set a node's namespace using ``__ns``.
- Observe how namespaces affect relative topic names.
- Run multiple instances of Turtlesim in separate namespaces.
- Distinguish relative names from absolute names.
- Combine namespaces with node-name remapping.

For more information, see:

- :doc:`About ROS 2 Nodes <About-Nodes>`
- :doc:`Understanding ROS 2 Topics <interfaces/topics/Understanding-ROS2-Topics/Understanding-ROS2-Topics>`
- :doc:`Passing ROS arguments to nodes via the command line <../Developer-Tools/Launch/Node-arguments>`
- :doc:`Using ROS 2 Launch for Large Projects <../Developer-Tools/Launch/Using-ROS2-Launch-For-Large-Projects>`
- `ROS 2 Topic and Service Names <https://design.ros2.org/articles/topic_and_service_names.html>`_