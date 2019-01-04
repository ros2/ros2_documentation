
Composing multiple nodes in a single process
============================================

ROS 1 - Nodes vs. Nodelets
--------------------------

In ROS 1 you can write your code either as a `ROS node <http://wiki.ros.org/Nodes>`__ or as a `ROS nodelet <http://wiki.ros.org/nodelet>`__.
ROS 1 nodes are being compiled into executables.
ROS 1 nodelets on the other hand are being compiled into a shared library which is then being loaded at runtime by a container process.

ROS 2 - Unified API
-------------------

In ROS 2 the recommended way of writing your code is that of a nodelet - we call it a ``Component``.
This enables to easily add common concepts to existing code, like a `life cycle <http://design.ros2.org/articles/node_lifecycle.html>`__.
The biggest drawback of different APIs is being avoided in ROS 2 - both approaches can use the same API in ROS 2.

..

   It will still be possible to use the node-like style of "writing your own main" but for the common case it is not recommended.


By making the process layout a deploy-time decision the user can choose between:


* running multiple nodes in separate processes with the benefits of process/fault isolation as well as easier debugging of individual nodes and
* running multiple nodes in a single process with the lower overhead and optionally more efficient communication (see `Intra Process Communication <Intra-Process-Communication>`).

The vision is that a future version of ``ros2 launch`` will support making these different deployments easily configurable.

Writing a Component
-------------------

Since a component is only built into a shared library it doesn't have a ``main`` function (see `Talker source code <https://github.com/ros2/demos/blob/master/composition/src/talker_component.cpp>`__\ ).
A component subclasses from ``rclcpp::Node``.
Since it is not in control of the thread it shouldn't perform any long running or even blocking tasks in its constructor.
Instead it can use timers to get periodic notification.
Additionally it can create publishers, subscribers, servers, and clients.

An important aspect of making such a class a component is that the class registers itself using the package ``class_loader`` (see last line in the source code).
This makes the component discoverable when its library is being loaded into a running process - it acts as kind of an entry point.

.. _composition-using-components:

Using components
----------------

The `composition <https://github.com/ros2/demos/tree/master/composition>`__ package contains a couple of different approaches how to use components.
The two most common ones are:


#. You start a generic container process (\ `1 <https://github.com/ros2/demos/blob/master/composition/src/api_composition.cpp>`__\ ) and call the ROS service `load_node <https://github.com/ros2/demos/blob/master/composition/srv/LoadNode.srv>`__ offered by the container.
   The ROS service will then load the component specified by the passed package name and library name and start executing it within the running process.
   Instead of calling the ROS service programmatically you can also use a `command line tool <https://github.com/ros2/demos/blob/master/composition/src/api_composition_cli.cpp>`__ to invoke the ROS service with the passed command line arguments
#. You create a `custom executable <https://github.com/ros2/demos/blob/master/composition/src/manual_composition.cpp>`__ containing multiple nodes which are known at compile time.
   This approach requires that each component has a header file (which is not strictly needed for the first case).

Run the demos
-------------

The executables from the `composition <https://github.com/ros2/demos/tree/master/composition>`__ packages can be run with the following commands:

Run-time composition using ROS services (1.) with a publisher and subscriber
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In the first shell:

.. code-block:: bash

   ros2 run composition api_composition


In the second shell (see `talker <https://github.com/ros2/demos/blob/master/composition/src/talker_component.cpp>`__ source code):

.. code-block:: bash

   ros2 run composition api_composition_cli composition composition::Talker


Now the first shell should show a message that the component was loaded as well as repeated message for publishing a message.

Another command in the second shell (see `listener <https://github.com/ros2/demos/blob/master/composition/src/listener_component.cpp>`__ source code):

.. code-block:: bash

   ros2 run composition api_composition_cli composition composition::Listener


Now the first shell should show repeated output for each received message.

..

   The demo uses hardcoded topic names and therefore you can't run ``api_composition`` twice.
   But in general it would be possible to run to separate container processes and load the talker and listener into separate ones and they would still communicate with each other.


Run-time composition using ROS services (1.) with a server and client
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The example with a server and a client is very similar.

In the first shell:

.. code-block:: bash

   ros2 run composition api_composition


In the second shell (see `server <https://github.com/ros2/demos/blob/master/composition/src/server_component.cpp>`__ and `client <https://github.com/ros2/demos/blob/master/composition/src/client_component.cpp>`__ source code):

.. code-block:: bash

   ros2 run composition api_composition_cli composition composition::Server
   ros2 run composition api_composition_cli composition composition::Client


In this case the client sends a request to the server, the server processes the request and replies with a response, and the client prints the received response.

Compile-time composition using ROS services (2.)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This demos shows that the same shared libraries can be reused to compile a single executable running multiple components.
The executable contains all four components from above: talker and listener as well as server and client.

In the shell call (see `source code <https://github.com/ros2/demos/blob/master/composition/src/manual_composition.cpp>`__\ ):

.. code-block:: bash

   ros2 run composition manual_composition


This should show repeated messages from both pairs, the talker and the listener as well as the server and the client.

Run-time composition using dlopen
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This demo presents an alternative to 1. by creating a generic container process and pass it explicitly the libraries to load without using ROS interfaces.
The process will open each library and create one instance of each "rclcpp::Node" class in the library `source code <https://github.com/ros2/demos/blob/master/composition/src/dlopen_composition.cpp>`__\ ).

**Linux** In the shell call:

.. code-block:: bash

   ros2 run composition dlopen_composition `ros2 pkg prefix composition`/lib/libtalker_component.so `ros2 pkg prefix composition`/lib/liblistener_component.so


**OSX** In the shell call:

.. code-block:: bash

   ros2 run composition dlopen_composition `ros2 pkg prefix composition`/lib/libtalker_component.dylib `ros2 pkg prefix composition`/lib/liblistener_component.dylib


**Windows** In cmd.exe call

.. code-block:: bash

   ros2 pkg prefix composition


to get the path to where composition is installed. Then call

.. code-block:: bash

   ros2 run composition dlopen_composition <path_to_composition_install>\bin\talker_component.dll <path_to_composition_install>\bin\listener_component.dll


Now the shell should show repeated output for each sent and received message.
