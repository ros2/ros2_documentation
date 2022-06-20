Using ROS 2 launch to launch composable nodes
=============================================

.. contents:: Table of Contents
   :depth: 1
   :local:

In the :doc:`Composition tutorial <../Tutorials/Intermediate/Composition>`, you learned about composable nodes and how to use them from the command-line.
In the :doc:`Launch tutorials <../Tutorials/Intermediate/Launch/Launch-Main>`, you learned about launch files and how to use them to manage multiple nodes.

This guide will combine the above two topics and teach you how to write launch files for composable nodes.

Setup
-----

See the :doc:`installation instructions <../Installation>` for details on installing ROS 2.

If you've installed ROS 2 from packages, ensure that you have ``ros-{DISTRO}-image-tools`` installed.
If you downloaded the archive or built ROS 2 from source, it will already be part of the installation.

Launch file examples
--------------------

Below is a launch file that launches composable nodes in Python, XML, and YAML.
The launch files all do the following:

* Instantiate a cam2image composable node with remappings, custom parameters, and extra arguments
* Instantiate a showimage composable node with remappings, custom parameters, and extra arguments

.. tabs::

  .. group-tab:: Python

    .. code-block:: python

      import launch
      from launch_ros.actions import ComposableNodeContainer
      from launch_ros.descriptions import ComposableNode


      def generate_launch_description():
          """Generate launch description with multiple components."""
          container = ComposableNodeContainer(
                  name='image_container',
                  namespace='',
                  package='rclcpp_components',
                  executable='component_container',
                  composable_node_descriptions=[
                      ComposableNode(
                          package='image_tools',
                          plugin='image_tools::Cam2Image',
                          name='cam2image',
                          remappings=[('/image', '/burgerimage')],
                          parameters=[{'width': 320, 'height': 240, 'burger_mode': True, 'history': 'keep_last'}],
                          extra_arguments=[{'use_intra_process_comms': True}]),
                      ComposableNode(
                          package='image_tools',
                          plugin='image_tools::ShowImage',
                          name='showimage',
                          remappings=[('/image', '/burgerimage')],
                          parameters=[{'history': 'keep_last'}],
                          extra_arguments=[{'use_intra_process_comms': True}])
                  ],
                  output='both',
          )

          return launch.LaunchDescription([container])

Using the Launch files from the command-line
--------------------------------------------

Any of the launch files above can be run with ``ros2 launch``.
Copy the data into a local file, and then run:

.. code-block:: console

  ros2 launch <path_to_launch_file>

Intra-process communications
----------------------------

All of the above examples use an extra argument to setup intra-process communication between the nodes.
For more information on what intra-process communications are, see the :doc:`intra-process comms tutorial <../Tutorials/Demos/Intra-Process-Communication>`.
