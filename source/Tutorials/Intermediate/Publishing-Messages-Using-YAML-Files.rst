Publishing messages using YAML files
====================================

**Goal:** Record and replay topics using YAML files.

**Tutorial level:** Intermediate

**Time:** 5 minutes

.. contents:: Table of Contents
   :depth: 2
   :local:

Overview
--------

Publishing ROS 2 messages via CLI is straightforward for simple types like {interface(std_msgs/msg/Bool)} or {interface(std_msgs/msg/String)}.
However, it becomes tedious for complex message structures.
This tutorial demonstrates how to use ``ros2 echo`` and ``ros2 pub`` with YAML files to record, edit, and replay topic data efficiently.

Prerequisites
-------------

This tutorial uses concepts like ROS 2 topics and CLI tools covered in the following tutorial:

- :doc:`Understanding topics <../Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics>`

Tasks
-----

We assume that an entity is publishing a {interface(geometry_msgs/msg/Twist)} message through a topic named ``cmd_vel`` and we want to capture the message, edit it and publish it to a topic.
We can use the ``echo`` verb to capture the message and save it in a YAML file ``cmd_vel.yaml`` using the output redirection operator ``>``.

.. code-block:: console

    $ ros2 topic echo --once  /cmd_vel > cmd_vel.yaml

This creates a ``cmd_vel.yaml`` file with the following content in the directory the command was executed:

.. code-block:: yaml

    linear:
        x: 1.0
        y: 0.0
        z: 0.0
    angular:
        x: 0.0
        y: 0.0
        z: 0.0
    ---

To publish a message, we utilize the ``--yaml-file`` option available with the ``pub`` verb of the ``ros2 topic`` command.
First, we specify the target topic—in this case, ``/cmd_vel``, followed by the message type {interface(geometry_msgs/msg/Twist)}.
Lastly, we specify the YAML file containing the message data.
The following command will publish the message contained in the ``YAML`` file to the designated ``topic`` once.

.. code-block:: console

    $ ros2 topic pub /cmd_vel geometry_msgs/msg/Twist --yaml-file cmd_vel.yaml
    publisher: beginning loop
    publishing geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=1.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0))

You can also publish more than once by adding more data to the YAML file as demonstrated in the example below.
In this case two messages where added to the YAML file with different values.

.. code-block:: yaml

    linear:
        x: 1.0
        y: 0.0
        z: 0.0
    angular:
        x: 0.0
        y: 0.0
        z: 0.0
    ---
    linear:
        x: 2.0
        y: 0.0
        z: 0.0
    angular:
        x: 0.0
        y: 0.0
        z: 0.0
    ---
    linear:
        x: 3.0
        y: 0.0
        z: 0.0
    angular:
        x: 0.0
        y: 0.0
        z: 0.0
    ---

By executing the same command as before, we publish three different messages to the ``/cmd_vel`` topic.

.. code-block:: console

    $ ros2 topic pub /cmd_vel geometry_msgs/msg/Twist --yaml-file cmd_vel.yaml
    publisher: beginning loop
    publishing #1: geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=1.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0))

    publishing #2: geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=3.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0))

    publishing #3: geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=2.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0))

Next steps
----------

If you are interested in publishing multiple topics using the CLI, you can take a look at the :doc:`Recording and playing back data <../Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data>` tutorial.
