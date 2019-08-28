.. redirect-from::

    Launch-system

Launching/monitoring multiple nodes with Launch
===============================================

ROS 2 launch system
-------------------

The launch system in ROS 2 is responsible for helping the user describe the configuration of their system and then execute it as described.
The configuration of the system includes what programs to run, where to run them, what arguments to pass them, and ROS specific conventions which make it easy to reuse components throughout the system by giving them each different configurations.
It is also responsible for monitoring the state of the processes launched, and reporting and/or reacting to changes in the state of those processes.

The ROS 2 Bouncy release includes a framework in which launch files, written in Python, can start and stop different nodes as well as trigger and act on various events.
The package providing this framework is ``launch_ros``, which uses the non-ROS-specific ``launch`` framework underneath.

The `design document (in review) <https://github.com/ros2/design/pull/163>`__ details the goal of the design of ROS 2's launch system (not all functionality is currently available).

Writing a ROS 2 launch file
---------------------------

ROS 2 uses Python to create launch files which are executed by the ROS 2 CLI verb, ``launch``.
We start by creating a ROS 2 package using ``ros2 pkg create <pkg-name> --dependencies [deps]`` in our workspace and
creating a new ``launch`` directory.

Python Packages
^^^^^^^^^^^^^^^

If you are creating a Python package, remove the ``CMakeLists.txt`` file and replace it with ``setup.py`` and
``setup.cfg`` files.
Your directory should look like this:

.. code-block:: shell

    src/
        my_package/
            launch/
            setup.py
            setup.cfg
            package.xml

In order for colcon to find the launch files, we need to inform Python's setup tools of our launch files using
the ``data_files`` parameter of ``setup``.

Inside our ``setup.py`` file.

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
            (os.path.join('share', package_name, 'launch'), glob('*.launch.py'))
        ]
    )

and ``setup.cfg`` file:

.. code-block:: bash

    [develop]
    script-dir=$base/lib/my_package
    [install]
    install-scripts=$base/lib/my_package

Finally, make sure your ``package.xml`` file is setup to indicate that you are creating a Python package.

.. code-block:: xml

    <?xml version="1.0"?>
    <!-- Make sure to use version 3 -->
    <package format="3">
      <name>my_package</name>
      <version>0.0.0</version>
      <description>My awesome package.</description>
      <license>TODO</license>
      <author email="ros2@ros.com">ROS 2 Developer</author>
      <maintainer email="ros2@ros.com">ROS 2 Developer</maintainer>
      <exec_depend>rclpy</exec_depend>

      <!-- This indicates you have a python package -->
      <export>
        <build_type>ament_python</build_type>
      </export>

    </package>

C++ Packages
^^^^^^^^^^^^

If you are creating a C++ package, we will only be adjusting the ``CMakeLists.txt`` file by adding

.. code-block:: cmake

    # Install launch files.
    install(DIRECTORY
      launch
      DESTINATION share/${PROJECT_NAME}/
    )

to the end of the file (but before ``ament_package()``).


Writing the launch file
^^^^^^^^^^^^^^^^^^^^^^^

Inside your launch directory, create a new launch file with the ``.launch.py`` suffix. For example ``script.launch.py``.
Your launch file should define the ``generate_launch_description`` which returns a ``launch.LaunchDescription()``
to be used by the ``ros2 launch`` verb.

The ``RegisterEventHandler`` action here makes sure that the launch process shuts down when the node exits.

.. code-block:: python

    import launch
    import launch_ros.actions


    def generate_launch_description():
        script = launch_ros.actions.Node(
            package='my_package', node_executable='script', output='screen')
        return launch.LaunchDescription([
            script,
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=client,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )),
        ])

Usage
^^^^^

While launch files can be written as standalone scripts, the typical usage in ROS is to have launch files invoked by ROS 2 tools.

After running ``colcon build`` and sourcing your workspace, you should be able to launch the launch file as follows:

.. code-block:: bash

   ros2 launch my_package script.launch.py


Example of ROS 2 launch concepts
--------------------------------

The launch file in `this example <https://github.com/ros2/launch_ros/blob/master/launch_ros/examples/lifecycle_pub_sub_launch.py>`__ launches two nodes, one of which is a node with a `managed lifecycle <Managed-Nodes>` (a "lifecycle node").
Lifecycle nodes launched through ``launch_ros`` automatically emit *events* when they transition between states.
The events can then be acted on through the launch framework, e.g. by emitting other events (such as requesting another state transition, which lifecycle nodes launched through ``launch_ros`` automatically have event handlers for) or triggering other *actions* (e.g. starting another node).

In the aforementioned example, various transition requests are requested of the ``talker`` lifecycle node, and  its transition events are reacted to by, for example, launching a ``listener`` node when the lifecycle talker reaches the appropriate state.

Documentation
-------------

`The launch documentation <https://github.com/ros2/launch/blob/master/launch/doc/source/architecture.rst>`__ provides more details on concepts that are also used in ``launch_ros``.

Additional documentation/examples of capabilities are forthcoming.
See `the source code <https://github.com/ros2/launch>`__ in the meantime.
