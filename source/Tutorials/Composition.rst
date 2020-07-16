.. redirect-from::

    Composition

Composing multiple nodes in a single process
============================================

.. contents:: Table of Contents
   :depth: 2
   :local:

ROS 1 - Nodes vs. Nodelets
--------------------------

In ROS 1 you can write your code either as a `ROS node <https://wiki.ros.org/Nodes>`__ or as a `ROS nodelet <https://wiki.ros.org/nodelet>`__.
ROS 1 nodes are compiled into executables.
ROS 1 nodelets on the other hand are compiled into a shared library which is then loaded at runtime by a container process.

ROS 2 - Unified API
-------------------

In ROS 2 the recommended way of writing your code is similar to a nodelet - we call it a ``Component``.
This makes is easy to add common concepts to existing code, like a `life cycle <https://design.ros2.org/articles/node_lifecycle.html>`__.
The biggest drawback of different APIs is avoided in ROS 2 since both approaches use the same API in ROS 2.

.. note::

   It is still possible to use the node-like style of "writing your own main" but for the common case it is not recommended.


By making the process layout a deploy-time decision the user can choose between:


* running multiple nodes in separate processes with the benefits of process/fault isolation as well as easier debugging of individual nodes and
* running multiple nodes in a single process with the lower overhead and optionally more efficient communication (see `Intra Process Communication <Intra-Process-Communication>`).

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


Run the demos
-------------

The demos use executables from `rclcpp_components <https://github.com/ros2/rclcpp/tree/master/rclcpp_components>`__, `ros2component <https://github.com/ros2/ros2cli/tree/master/ros2component>`__, and  `composition <https://github.com/ros2/demos/tree/master/composition>`__ packages, and can be run with the following commands.


Discover available components
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To see what components are registered and available in the workspace, execute the following in a shell:

.. code-block:: bash

   $ ros2 component types
   composition
     composition::Talker
     composition::Listener
     composition::Server
     composition::Client

Run-time composition using ROS services (1.) with a publisher and subscriber
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In the first shell, start the component container:

.. code-block:: bash

   ros2 run rclcpp_components component_container

Verify that the container is running via ``ros2`` command line tools:

.. code-block:: bash

   $ ros2 component list
   /ComponentManager

In the second shell (see `talker <https://github.com/ros2/demos/blob/master/composition/src/talker_component.cpp>`__ source code).
The command will return the unique ID of the loaded component as well as the node name.

.. code-block:: bash

   $ ros2 component load /ComponentManager composition composition::Talker
   Loaded component 1 into '/ComponentManager' container node as '/talker'


Now the first shell should show a message that the component was loaded as well as repeated message for publishing a message.

Another command in the second shell (see `listener <https://github.com/ros2/demos/blob/master/composition/src/listener_component.cpp>`__ source code):

.. code-block:: bash

   $ ros2 component load /ComponentManager composition composition::Listener
   Loaded component 2 into '/ComponentManager' container node as '/listener'

The ``ros2`` command line utility can now be used to inspect the state of the container:

.. code-block:: bash

   $ ros2 component list
   /ComponentManager
      1  /talker
      2  /listener


Now the first shell should show repeated output for each received message.

Run-time composition using ROS services (1.) with a server and client
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The example with a server and a client is very similar.

In the first shell:

.. code-block:: bash

   ros2 run rclcpp_components component_container


In the second shell (see `server <https://github.com/ros2/demos/blob/master/composition/src/server_component.cpp>`__ and `client <https://github.com/ros2/demos/blob/master/composition/src/client_component.cpp>`__ source code):

.. code-block:: bash

   ros2 component load /ComponentManager composition composition::Server
   ros2 component load /ComponentManager composition composition::Client

In this case the client sends a request to the server, the server processes the request and replies with a response, and the client prints the received response.

Compile-time composition using ROS services (2.)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This demos shows that the same shared libraries can be reused to compile a single executable running multiple components.
The executable contains all four components from above: talker and listener as well as server and client.

In the shell call (see `source code <https://github.com/ros2/demos/blob/master/composition/src/manual_composition.cpp>`__):

.. code-block:: bash

   ros2 run composition manual_composition

This should show repeated messages from both pairs, the talker and the listener as well as the server and the client.

.. note::

   Manually-composed components will not be reflected in the ``ros2 component list`` command line tool output.

Run-time composition using dlopen
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This demo presents an alternative to 1. by creating a generic container process and explicitly passing the libraries to load without using ROS interfaces.
The process will open each library and create one instance of each "rclcpp::Node" class in the library `source code <https://github.com/ros2/demos/blob/master/composition/src/dlopen_composition.cpp>`__).

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

       ros2 run composition dlopen_composition `ros2 pkg prefix composition`/lib/libtalker_component.so `ros2 pkg prefix composition`/lib/liblistener_component.so

  .. group-tab:: macOS

    .. code-block:: bash

       ros2 run composition dlopen_composition `ros2 pkg prefix composition`/lib/libtalker_component.dylib `ros2 pkg prefix composition`/lib/liblistener_component.dylib

  .. group-tab:: Windows

    .. code-block:: bash

       ros2 pkg prefix composition

    to get the path to where composition is installed. Then call

    .. code-block:: bash

       ros2 run composition dlopen_composition <path_to_composition_install>\bin\talker_component.dll <path_to_composition_install>\bin\listener_component.dll

Now the shell should show repeated output for each sent and received message.

.. note::

   dlopen-composed components will not be reflected in the ``ros2 component list`` command line tool output.


Composition using launch actions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

While the command line tools are useful for debugging and diagnosing component configurations, it is frequently more convenient to start a set of components at the same time.
To automate this action, we can use the functionality in ``ros2 launch``.

.. code-block:: bash

   ros2 launch composition composition_demo.launch.py


Advanced Topics
---------------

Now that we have seen the basic operation of components, we can discuss a few more advanced topics.


Unloading components
^^^^^^^^^^^^^^^^^^^^

In the first shell, start the component container:

.. code-block:: bash

   ros2 run rclcpp_components component_container

Verify that the container is running via ``ros2`` command line tools:

.. code-block:: bash

   $ ros2 component list
   /ComponentManager

In the second shell (see `talker <https://github.com/ros2/demos/blob/master/composition/src/talker_component.cpp>`__ source code).
The command will return the unique ID of the loaded component as well as the node name.

.. code-block:: bash

   $ ros2 component load /ComponentManager composition composition::Talker
   Loaded component 1 into '/ComponentManager' container node as '/talker'
   $ ros2 component load /ComponentManager composition composition::Listener
   Loaded component 2 into '/ComponentManager' container node as '/listener'

Use the unique ID to unload the node from the component container.

.. code-block:: bash

   $ ros2 component unload /ComponentManager 1 2
   Unloaded component 1 from '/ComponentManager' container
   Unloaded component 2 from '/ComponentManager' container

In the first shell, verify that the repeated messages from talker and listener have stopped.


Remapping container name and namespace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The component manager name and namespace can be remapped via standard command line arguments:

.. code-block:: bash

   ros2 run rclcpp_components component_container --ros-args -r __node:=MyContainer -r __ns:=/ns

In a second shell, components can be loaded by using the updated container name:

.. code-block:: bash

   ros2 component load /ns/MyContainer composition composition::Listener

.. note::

   Namespace remappings of the container do not affect loaded components.


Remap component names and namespaces
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Component names and namespaces may be adjusted via arguments to the load command.

In the first shell, start the component container:

.. code-block:: bash

   ros2 run rclcpp_components component_container


Some examples of how to remap names and namespaces:

.. code-block:: bash

   # Remap node name
   ros2 component load /ComponentManager composition composition::Talker --node-name talker2
   # Remap namespace
   ros2 component load /ComponentManager composition composition::Talker --node-namespace /ns
   # Remap both
   ros2 component load /ComponentManager composition composition::Talker --node-name talker3 --node-namespace /ns2

The corresponding entries appear in ``ros2 component list``:

.. code-block:: bash

   $ ros2 component list
   /ComponentManager
      1  /talker2
      2  /ns/talker
      3  /ns2/talker3

.. note::

   Namespace remappings of the container do not affect loaded components.
