.. redirect-from::

    Tutorials/Launch/CLI-Intro
    Tutorials/Beginner-CLI-Tools/Launching-Multiple-Nodes/Launching-Multiple-Nodes

.. _ROS2Launch:

Launching multiple nodes - how-to
=================================

A launch file lets you start multiple nodes simultaneously with a single command, without opening a separate terminal for each.
This article shows you how to run an existing launch file using ``ros2 launch``.

**Area: ROS-tutorials | Content-type: how-to | Experience: beginner**

.. contents:: Contents
   :depth: 2
   :local:

Summary
-------

Use ``ros2 launch <package> <launch_file>`` to run multiple nodes at once from a single launch file.

Steps
-----

.. note::
   Source ROS in every new terminal you open.
   See :doc:`Configuring environment <../../../../Get-Started/Configuring-ROS2-Environment>`.

1 Run a launch file
^^^^^^^^^^^^^^^^^^^

To run a launch file, use:

.. code-block:: console

   $ ros2 launch <package_name> <launch_file_name>

For example, to run the ``multisim.launch.py`` file from the ``turtlesim`` package, use:

.. code-block:: console

   $ ros2 launch turtlesim multisim.launch.py

This runs the following launch file:

.. literalinclude:: launch/multisim.launch.py
   :language: python

.. note::

   The launch file above is written in Python, but you can also use XML and YAML to create launch files.
   You can see a comparison of these different launch formats in :doc:`../../../../Developer-Tools/Launch/Launch-file-different-formats`.

Two Turtlesim windows open, one for each node defined in the launch file:

.. image:: images/turtlesim_multisim.png

Related content
---------------

* :doc:`ROS launch tutorials <../../../../Developer-Tools/Launch/Launch-Main>`
* :doc:`Launch file formats <../../../../Developer-Tools/Launch/Launch-file-different-formats>`

FAQs
----

Can I write a launch file in a language other than Python?
   Yes. ROS supports launch files written in Python, XML, and YAML.
   For a comparison of these formats, see :doc:`../../../../Developer-Tools/Launch/Launch-file-different-formats`.

How do I write my own launch file?
   For a full guide on writing launch files, see the :doc:`ROS launch tutorials <../../../../Developer-Tools/Launch/Launch-Main>`.

Can I pass arguments to a launch file?
   Yes. Use ``ros2 launch <package> <launch_file> <arg>:=<value>`` to pass arguments at the command line.
   Arguments must be declared within the launch file itself.
