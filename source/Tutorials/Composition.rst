.. redirect-from::

    Composition

Composing multiple nodes in a single process
============================================

.. contents:: Table of Contents
   :depth: 2
   :local:

Background
----------

See the `conceptual article <../Concepts/About-Composition>`.

Run the demos
-------------

The demos use executables from `rclcpp_components <https://github.com/ros2/rclcpp/tree/master/rclcpp_components>`__, `ros2component <https://github.com/ros2/ros2cli/tree/master/ros2component>`__, and  `composition <https://github.com/ros2/demos/tree/master/composition>`__ packages, and can be run with the following commands.


Discover available components
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To see what components are registered and available in the workspace, execute the following in a shell:

.. code-block:: bash

   $ ros2 component types
   (... components of other packages here)
   composition
     composition::Talker
     composition::Listener
     composition::NodeLikeListener
     composition::Server
     composition::Client
   (... components of other packages here)

Run-time composition using ROS services with a publisher and subscriber
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In the first shell, start the component container:

.. code-block:: bash

   $ ros2 run rclcpp_components component_container

Verify that the container is running via ``ros2`` command line tools:

.. code-block:: bash

   $ ros2 component list
   /ComponentManager

In the second shell load the talker component (see `talker <https://github.com/ros2/demos/blob/master/composition/src/talker_component.cpp>`__ source code):

.. code-block:: bash

   $ ros2 component load /ComponentManager composition composition::Talker
   Loaded component 1 into '/ComponentManager' container node as '/talker'

The command will return the unique ID of the loaded component as well as the node name.

Now the first shell should show a message that the component was loaded as well as repeated message for publishing a message.

Run another command in the second shell to load the listener component (see `listener <https://github.com/ros2/demos/blob/master/composition/src/listener_component.cpp>`__ source code):

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

Run-time composition using ROS services with a server and client
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The example with a server and a client is very similar.

In the first shell:

.. code-block:: bash

   $ ros2 run rclcpp_components component_container

In the second shell (see `server <https://github.com/ros2/demos/blob/master/composition/src/server_component.cpp>`__ and `client <https://github.com/ros2/demos/blob/master/composition/src/client_component.cpp>`__ source code):

.. code-block:: bash

   $ ros2 component load /ComponentManager composition composition::Server
   $ ros2 component load /ComponentManager composition composition::Client

In this case the client sends a request to the server, the server processes the request and replies with a response, and the client prints the received response.

Compile-time composition using ROS services
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This demos shows that the same shared libraries can be reused to compile a single executable running multiple components.
The executable contains all four components from above: talker and listener as well as server and client.

In the shell call (see `source code <https://github.com/ros2/demos/blob/master/composition/src/manual_composition.cpp>`__):

.. code-block:: bash

   $ ros2 run composition manual_composition

This should show repeated messages from both pairs, the talker and the listener as well as the server and the client.

.. note::

   Manually-composed components will not be reflected in the ``ros2 component list`` command line tool output.

Run-time composition using dlopen
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This demo presents an alternative to run-time composition by creating a generic container process and explicitly passing the libraries to load without using ROS interfaces.
The process will open each library and create one instance of each "rclcpp::Node" class in the library `source code <https://github.com/ros2/demos/blob/master/composition/src/dlopen_composition.cpp>`__).

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

       $ ros2 run composition dlopen_composition `ros2 pkg prefix composition`/lib/libtalker_component.so `ros2 pkg prefix composition`/lib/liblistener_component.so

  .. group-tab:: macOS

    .. code-block:: bash

       $ ros2 run composition dlopen_composition `ros2 pkg prefix composition`/lib/libtalker_component.dylib `ros2 pkg prefix composition`/lib/liblistener_component.dylib

  .. group-tab:: Windows

    .. code-block:: bash

       > ros2 pkg prefix composition

    to get the path to where composition is installed. Then call

    .. code-block:: bash

       > ros2 run composition dlopen_composition <path_to_composition_install>\bin\talker_component.dll <path_to_composition_install>\bin\listener_component.dll

Now the shell should show repeated output for each sent and received message.

.. note::

   dlopen-composed components will not be reflected in the ``ros2 component list`` command line tool output.


Composition using launch actions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

While the command line tools are useful for debugging and diagnosing component configurations, it is frequently more convenient to start a set of components at the same time.
To automate this action, we can use the functionality in ``ros2 launch``.

.. code-block:: bash

   $ ros2 launch composition composition_demo.launch.py


Advanced Topics
---------------

Now that we have seen the basic operation of components, we can discuss a few more advanced topics.


Unloading components
^^^^^^^^^^^^^^^^^^^^

In the first shell, start the component container:

.. code-block:: bash

   $ ros2 run rclcpp_components component_container

Verify that the container is running via ``ros2`` command line tools:

.. code-block:: bash

   $ ros2 component list
   /ComponentManager

In the second shell load both the talker and listener as we have before:

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

   $ ros2 run rclcpp_components component_container --ros-args -r __node:=MyContainer -r __ns:=/ns

In a second shell, components can be loaded by using the updated container name:

.. code-block:: bash

   $ ros2 component load /ns/MyContainer composition composition::Listener

.. note::

   Namespace remappings of the container do not affect loaded components.


Remap component names and namespaces
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Component names and namespaces may be adjusted via arguments to the load command.

In the first shell, start the component container:

.. code-block:: bash

   $ ros2 run rclcpp_components component_container


Some examples of how to remap names and namespaces:

.. code-block:: bash

   # Remap node name
   $ ros2 component load /ComponentManager composition composition::Talker --node-name talker2
   # Remap namespace
   $ ros2 component load /ComponentManager composition composition::Talker --node-namespace /ns
   # Remap both
   $ ros2 component load /ComponentManager composition composition::Talker --node-name talker3 --node-namespace /ns2

The corresponding entries appear in ``ros2 component list``:

.. code-block:: bash

   $ ros2 component list
   /ComponentManager
      1  /talker2
      2  /ns/talker
      3  /ns2/talker3

.. note::

   Namespace remappings of the container do not affect loaded components.

Composable nodes as shared libraries
------------------------------------

If you want to export a composable node as a shared library from a package and use that node in another package that does link-time composition, add code to the CMake file which imports the actual targets in downstream packages.

Then install the generated file and export the generated file.

A practical example can be seen here: `ROS Discourse - Ament best practice for sharing libraries <https://discourse.ros.org/t/ament-best-practice-for-sharing-libraries/3602>`__
