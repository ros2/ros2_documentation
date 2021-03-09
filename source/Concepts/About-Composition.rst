About Composition
=================

.. contents:: Table of Contents
   :depth: 1
   :local:

ROS 1 - Nodes vs. Nodelets
--------------------------

In ROS 1 you can write your code either as a `ROS node <https://wiki.ros.org/Nodes>`__ or as a `ROS nodelet <https://wiki.ros.org/nodelet>`__.
ROS 1 nodes are compiled into executables.
ROS 1 nodelets on the other hand are compiled into a shared library which is then loaded at runtime by a container process.

ROS 2 - Unified API
-------------------

In ROS 2 the recommended way of writing your code is similar to a nodelet - we call it a ``Component``.
This makes it easy to add common concepts to existing code, like a `life cycle <https://design.ros2.org/articles/node_lifecycle.html>`__.
The biggest drawback of different APIs is avoided in ROS 2 since both approaches use the same API in ROS 2.

.. note::

   It is still possible to use the node-like style of "writing your own main" but for the common case it is not recommended.


By making the process layout a deploy-time decision the user can choose between:


* running multiple nodes in separate processes with the benefits of process/fault isolation as well as easier debugging of individual nodes and
* running multiple nodes in a single process with the lower overhead and optionally more efficient communication (see `Intra Process Communication <../Tutorials/Intra-Process-Communication>`).

Additionally ``ros2 launch`` can be used to automate these actions through specialized launch actions.


Writing a Component
-------------------

Since a component is only built into a shared library it doesn't have a ``main`` function (see `Talker source code <https://github.com/ros2/demos/blob/master/composition/src/talker_component.cpp>`__).
A component is commonly a subclass of ``rclcpp::Node``.
Since it is not in control of the thread it shouldn't perform any long running or blocking tasks in its constructor.
Instead it can use timers to get periodic notification.
Additionally it can create publishers, subscribers, servers, and clients.

An important aspect of making such a class a component is that the class registers itself using macros from the package ``rclcpp_components`` (see the last line in the source code).
This makes the component discoverable when its library is being loaded into a running process - it acts as kind of an entry point.

Additionally, once a component is created, it must be registered with the index to be discoverable by the tooling.

.. code-block:: cmake

   add_library(talker_component SHARED
      src/talker_component.cpp)
   rclcpp_components_register_nodes(talker_component "composition::Talker")
   # To register multiple components in the same shared library, use multiple calls
   # rclcpp_components_register_nodes(talker_component "composition::Talker2")

.. note::

   In order for the component_container to be able to find desired components, it must be executed or launched from a shell that has sourced the corresponding workspace.

.. _composition-using-components:

Using Components
----------------

The `composition <https://github.com/ros2/demos/tree/master/composition>`__ package contains a couple of different approaches on how to use components.
The three most common ones are:


#. Start a (`generic container process <https://github.com/ros2/rclcpp/blob/master/rclcpp_components/src/component_container.cpp>`__) and call the ROS service `load_node <https://github.com/ros2/rcl_interfaces/blob/master/composition_interfaces/srv/LoadNode.srv>`__ offered by the container.
   The ROS service will then load the component specified by the passed package name and library name and start executing it within the running process.
   Instead of calling the ROS service programmatically you can also use a `command line tool <https://github.com/ros2/ros2cli/tree/master/ros2component>`__ to invoke the ROS service with the passed command line arguments
#. Create a `custom executable <https://github.com/ros2/demos/blob/master/composition/src/manual_composition.cpp>`__ containing multiple nodes which are known at compile time.
   This approach requires that each component has a header file (which is not strictly needed for the first case).
#. Create a launch file and use ``ros2 launch`` to create a container process with multiple components loaded.

Practical application
---------------------

Try the `Composition demos <../Tutorials/Composition>`.
