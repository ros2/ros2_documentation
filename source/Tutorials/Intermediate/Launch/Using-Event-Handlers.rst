.. redirect-from::

    Tutorials/Launch-Files/Using-Event-Handlers
    Tutorials/Launch/Using-Event-Handlers

Using event handlers
====================

**Goal:** Learn about event handlers in ROS 2 launch files

**Tutorial level:** Intermediate

**Time:** 15 minutes

.. contents:: Table of Contents
   :depth: 2
   :local:

Background
----------

Launch in ROS 2 is a system that executes and manages user-defined processes.
It is responsible for monitoring the state of processes it launched, as well as reporting and reacting to changes in the state of those processes.
These changes are called events and can be handled by registering an event handler with the launch system.
Event handlers can be registered for specific events and can be useful for monitoring the state of processes.
Additionally, they can be used to define a complex set of rules which can be used to dynamically modify the launch file.

This tutorial shows usage examples of event handlers in ROS 2 launch files.

Prerequisites
-------------

This tutorial uses the :doc:`turtlesim <../../Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim>` package.
This tutorial also assumes you have :doc:`created a new package <../../Beginner-Client-Libraries/Creating-Your-First-ROS2-Package>` of build type ``ament_python`` called ``launch_tutorial``.

This tutorial extends the code shown in the :doc:`Using substitutions in launch files <./Using-Substitutions>` tutorial.

Using event handlers
--------------------

1 Event handlers example launch file
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Create a new file called ``example_event_handlers_launch.py`` file in the ``launch`` folder of the ``launch_tutorial`` package.

.. literalinclude:: launch/example_event_handlers_launch.py
    :language: python

``RegisterEventHandler`` actions for the ``OnProcessStart``, ``OnProcessIO``, ``OnExecutionComplete``, ``OnProcessExit``, and ``OnShutdown`` events were defined in the launch description.

The ``OnProcessStart`` event handler is used to register a callback function that is executed when the turtlesim node starts.
It logs a message to the console and executes the ``spawn_turtle`` action when the turtlesim node starts.

.. literalinclude:: launch/example_event_handlers_launch.py
    :language: python
    :lines: 98-106

The ``OnProcessIO`` event handler is used to register a callback function that is executed when the ``spawn_turtle`` action writes to its standard output.
It logs the result of the spawn request.

.. literalinclude:: launch/example_event_handlers_launch.py
    :language: python
    :lines: 107-115

The ``OnExecutionComplete`` event handler is used to register a callback function that is executed when the ``spawn_turtle`` action completes.
It logs a message to the console and executes the ``change_background_r`` and ``change_background_r_conditioned`` actions when the spawn action completes.

.. literalinclude:: launch/example_event_handlers_launch.py
    :language: python
    :lines: 116-128

The ``OnProcessExit`` event handler is used to register a callback function that is executed when the turtlesim node exits.
It logs a message to the console and executes the ``EmitEvent`` action to emit a ``Shutdown`` event when the turtlesim node exits.
It means that the launch process will shutdown when the turtlesim window is closed.

.. literalinclude:: launch/example_event_handlers_launch.py
    :language: python
    :lines: 129-139

Finally, the ``OnShutdown`` event handler is used to register a callback function that is executed when the launch file is asked to shutdown.
It logs a message to the console why the launch file is asked to shutdown.
It logs the message with a reason for shutdown like the closure of turtlesim window or :kbd:`ctrl-c` signal made by the user.

.. literalinclude:: launch/example_event_handlers_launch.py
    :language: python
    :lines: 140-146

Build the package
-----------------

Go to the root of the workspace, and build the package:

.. code-block:: console

  $ colcon build

Also remember to source the workspace after building.

Launching example
-----------------

Now you can launch the ``example_event_handlers_launch.py`` file using the ``ros2 launch`` command.

.. code-block:: console

    $ ros2 launch launch_tutorial example_event_handlers_launch.py turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200

This will do the following:

#. Start a turtlesim node with a blue background
#. Spawn the second turtle
#. Change the color to purple
#. Change the color to pink after two seconds if the provided ``background_r`` argument is ``200`` and ``use_provided_red`` argument is ``True``
#. Shutdown the launch file when the turtlesim window is closed

Additionally, it will log messages to the console when:

#. The turtlesim node starts
#. The spawn action is executed
#. The ``change_background_r`` action is executed
#. The ``change_background_r_conditioned`` action is executed
#. The turtlesim node exits
#. The launch process is asked to shutdown.

Documentation
-------------

`The launch documentation <https://docs.ros.org/en/{DISTRO}/p/launch/architecture.html>`_ provides detailed information about available event handlers.

Summary
-------

In this tutorial, you learned about using event handlers in launch files.
You learned about their syntax and usage examples to define a complex set of rules to dynamically modify launch files.
