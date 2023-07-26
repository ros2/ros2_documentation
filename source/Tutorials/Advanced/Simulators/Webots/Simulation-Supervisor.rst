The Ros2Supervisor Node
=======================

**Goal:** Extend the interface with a default Supervisor robot, named ``Ros2Supervisor``.

**Tutorial level:** Advanced

**Time:** 10 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

In this tutorial, you will learn how to enable the ``Ros2Supervisor`` node which enhances the interface by creating additional services and topics to interact with the simulation.
You can, for example, record animations or spawn Webots nodes directly from the ROS 2 interface while the simulation is running.
These instructions list in details the current implemented features and how to use them.

Prerequisites
-------------

Before proceeding with this tutorial, make sure you have completed the following:

- Understanding of ROS 2 nodes and topics covered in the beginner :doc:`../../../../Tutorials`.
- Knowledge of Webots and ROS 2 and its interface package.
- Familiarity with :doc:`./Setting-Up-Simulation-Webots-Basic`.

The ``Ros2Supervisor``
----------------------

The ``Ros2Supervisor`` is made of two main parts:

* A Webots Robot node added to the simulation world. Its ``supervisor`` field is set to TRUE.
* A ROS 2 node that connects to the Webots Robot as an extern controller (in a similar way to your own robot plugin).

The ROS 2 node acts as a controller that calls Supervisor API functions to control or interact with the simulation world.
User interactions with the ROS 2 node are mainly performed through services and topics.

These nodes can be automatically created at the Webots launch using the ``ros2_supervisor`` parameter in the ``WebotsLauncher``.

.. code-block:: python

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        mode=mode,
        ros2_supervisor=True
    )

The ``webots._supervisor`` object must also be included in the ``LaunchDescription`` returned by the launch file.

.. code-block:: python

    return LaunchDescription([
        webots,
        webots._supervisor,

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        )
    ])

More information about launch files for ``webots_ros2`` projects can be found in :doc:`./Setting-Up-Simulation-Webots-Basic`.

Clock topic
-----------

The ``Ros2Supervisor`` node is responsible to get the time of the Webots simulation and publish it to the ``/clock`` topic.
This means that it is mandatory to spawn the ``Ros2Supervisor`` if some other nodes have their ``use_sim_time`` parameter set to ``true``.
More information about the ``/clock`` topic can be found in the `ROS wiki <http://wiki.ros.org/Clock>`_.

Import a Webots node
--------------------

The ``Ros2Supervisor`` node also allows you to spawn Webots nodes from strings through a service.

The service is named ``/Ros2Supervisor/spawn_node_from_string`` and is of type ``webots_ros2_msgs/srv/SpawnNodeFromString``.
The ``SpawnNodeFromString`` type expects a ``data`` string as input and returns a ``success`` boolean.

From the given string, the Supervisor node is getting the name of the imported node and adding it to an intern list for potential later removal (see :ref:`Remove a Webots imported node`).

The node is imported using the ``importMFNodeFromString(nodeString)`` `API function <https://cyberbotics.com/doc/reference/supervisor?tab-language=python#wb_supervisor_field_import_mf_node_from_string>`_.

Here is an example to import a simple Robot named ``imported_robot``:

.. code-block:: bash

    ros2 service call /Ros2Supervisor/spawn_node_from_string webots_ros2_msgs/srv/SpawnNodeFromString "data: Robot { name \"imported_robot\" }"

.. note::
    If you try to import some PROTOs in the node string, their respective URLs must be declared in the .wbt world file as EXTERNPROTO or as IMPORTABLE EXTERNPROTO.

.. _Remove a Webots imported node:

Remove a Webots imported node
-----------------------------

Once a node has been imported with the ``/Ros2Supervisor/spawn_node_from_string`` service, it can also be removed.

This can be achieved by sending the name of the node to the topic named ``/Ros2Supervisor/remove_node`` of type ``std_msgs/msg/String``.

If the node is indeed in the imported list, it is removed with the ``remove()`` `API method <https://cyberbotics.com/doc/reference/supervisor?tab-language=python#wb_supervisor_node_remove>`_.

Here is an example on how to remove the ``imported_robot`` Robot:

.. code-block:: bash

    ros2 topic pub --once /Ros2Supervisor/remove_node std_msgs/msg/String "{data: imported_robot}"

Record animations
-----------------

The ``Ros2Supervisor`` node also creates two additional services to record HTML5 animations.

The ``/Ros2Supervisor/animation_start_recording`` service is of type ``webots_ros2_msgs/srv/SetString`` and allows to start the animation.
The ``SetString`` type expects a ``value`` string as input and returns a ``success`` boolean.
The input ``value`` represents the absolute path to the directory where the animations files should be saved.

Here is an example on how to start an animation:

.. code-block:: bash

    ros2 service call /Ros2Supervisor/animation_start_recording webots_ros2_msgs/srv/SetString "{value: "<ABSOLUTE_PATH>/index.html"}"


The ``/Ros2Supervisor/animation_stop_recording`` service is of type ``webots_ros2_msgs/srv/GetBool`` and allows to stop the animation.

.. code-block:: bash

    ros2 service call /Ros2Supervisor/animation_stop_recording webots_ros2_msgs/srv/GetBool "{ask: True}"


Summary
-------

In this tutorial, you learned how to enable the ``Ros2Supervisor`` and how to extend the interface with the Webots simulation.
The node creates multiple services and topics to interact with and modify the simulation.
