.. redirect-from::

    Launch-system

Launching/monitoring multiple nodes with Launch
===============================================

ROS 2 launch system
-------------------

The launch system in ROS 2 is responsible for helping the user describe the configuration of their system and then execute it as described.
The configuration of the system includes what programs to run, where to run them, what arguments to pass them, and ROS specific conventions which make it easy to reuse components throughout the system by giving them each different configurations.
It is also responsible for monitoring the state of the processes launched, and reporting and/or reacting to changes in the state of those processes.

Launch files written in Python can start and stop different nodes as well as trigger and act on various events.
The package providing this framework is ``launch_ros``, which uses the non-ROS-specific ``launch`` framework underneath.

The `design document <https://design.ros2.org/articles/roslaunch.html>`__ details the goal of the design of ROS 2's launch system (not all functionality is currently available).

Writing a ROS 2 launch file
---------------------------

If you haven't already, make sure you go through the quickstart tutorial on how to create a ROS 2 package.
One way to create launch files in ROS 2 is using a Python file, which are executed by the ROS 2 CLI tool, ``ros2 launch``.
We start by creating a ROS 2 package using ``ros2 pkg create <pkg-name> --dependencies [deps]`` in our workspace and creating a new ``launch`` directory.

Python Packages
^^^^^^^^^^^^^^^

For Python packages, your directory should look like this:

.. code-block:: shell

    src/
        my_package/
            launch/
            setup.py
            setup.cfg
            package.xml

In order for colcon to find the launch files, we need to inform Python's setup tools of our launch files using the ``data_files`` parameter of ``setup``.

Inside our ``setup.py`` file:

.. code-block:: python

    import os
    from glob import glob
    from setuptools import setup

    package_name = 'my_package'

    setup(
        # Other parameters ...
        data_files=[
            # ... Other data files
            # Include all launch files. This is the most important line here!
            (os.path.join('share', package_name), glob('launch/*.launch.py'))
        ]
    )

C++ Packages
^^^^^^^^^^^^

If you are creating a C++ package, we will only be adjusting the ``CMakeLists.txt`` file by adding:

.. code-block:: cmake

    # Install launch files.
    install(DIRECTORY
      launch
      DESTINATION share/${PROJECT_NAME}/
    )

to the end of the file (but before ``ament_package()``).

Writing the launch file
^^^^^^^^^^^^^^^^^^^^^^^

Inside your launch directory, create a new launch file with the ``.launch.py`` suffix.
For example ``my_script.launch.py``.

``.launch.py`` is not specifically required as the file suffix for launch files.
Another popular option is ``_launch.py``, used in the :ref:`beginner level launch files tutorial <ROS2Launch>`.
If you do change the suffix, make sure to adjust the ``glob()`` argument in your ``setup.py`` file accordingly.

Your launch file should define the ``generate_launch_description()`` which returns a ``launch.LaunchDescription()`` to be used by the ``ros2 launch`` verb.

.. code-block:: python

   import launch
   import launch.actions
   import launch.substitutions
   import launch_ros.actions


   def generate_launch_description():
       return launch.LaunchDescription([
           launch.actions.DeclareLaunchArgument(
               'node_prefix',
               default_value=[launch.substitutions.EnvironmentVariable('USER'), '_'],
               description='Prefix for node names'),
           launch_ros.actions.Node(
               package='demo_nodes_cpp', executable='talker', output='screen',
               name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'talker']),
       ])


Usage
^^^^^

While launch files can be written as standalone scripts, the typical usage in ROS is to have launch files invoked by ROS 2 tools.

After running ``colcon build`` and sourcing your workspace, you should be able to launch the launch file as follows:

.. code-block:: bash

   ros2 launch my_package script.launch.py

Example of ROS 2 launch concepts
--------------------------------

The launch file in `this example <https://github.com/ros2/launch_ros/blob/master/launch_ros/examples/lifecycle_pub_sub_launch.py>`__
launches two nodes, one of which is a node with a `managed lifecycle <Managed-Nodes>` (a "lifecycle node").
Lifecycle nodes launched through ``launch_ros`` automatically emit *events* when they transition between states.
The events can then be acted on through the launch framework.
For example, by emitting other events (such as requesting another state transition, which lifecycle nodes launched through ``launch_ros`` automatically have event handlers for) or triggering other *actions* (e.g. starting another node).

In the aforementioned example, various transition requests are requested of the ``talker`` lifecycle node, and its transition events are reacted to by, for example, launching a ``listener`` node when the lifecycle talker reaches the appropriate state.

Documentation
-------------

`The launch documentation <https://github.com/ros2/launch/blob/master/launch/doc/source/architecture.rst>`__ provides more details on concepts that are also used in ``launch_ros``.

Additional documentation/examples of capabilities are forthcoming.
See `the source code <https://github.com/ros2/launch>`__ in the meantime.
