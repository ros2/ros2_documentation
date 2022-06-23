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

  .. group-tab:: XML

    .. code-block:: xml

      <launch>
          <node_container pkg="rclcpp_components" exec="component_container" name="image_container" namespace="">
              <composable_node pkg="image_tools" plugin="image_tools::Cam2Image" name="cam2image" namespace="">
                  <remap from="/image" to="/burgerimage" />
                  <param name="width" value="320" />
                  <param name="height" value="240" />
                  <param name="burger_mode" value="true" />
                  <param name="history" value="keep_last" />
                  <extra_arg name="use_intra_process_comms" value="true" />
              </composable_node>
              <composable_node pkg="image_tools" plugin="image_tools::ShowImage" name="showimage" namespace="">
                  <remap from="/image" to="/burgerimage" />
                  <param name="history" value="keep_last" />
                  <extra_arg name="use_intra_process_comms" value="true" />
              </composable_node>
          </node_container>
      </launch>

  .. group-tab:: YAML

    .. code-block:: yaml

      launch:

          - node_container:
              pkg: rclcpp_components
              exec: component_container
              name: image_container
              namespace: ''
              composable_node:
                  -   pkg: image_tools
                      plugin: image_tools::Cam2Image
                      name: cam2image
                      namespace: ''
                      remap:
                          - from: /image
                            to: /burgerimage
                      param:
                          - name: width
                            value: 320
                          - name: height
                            value: 240
                          - name: burger_mode
                            value: true
                          - name: history
                            value: keep_last
                      extra_arg:
                          - name: use_intra_process_comms
                            value: 'true'

                  -   pkg: image_tools
                      plugin: image_tools::ShowImage
                      name: showimage
                      namespace: ''
                      remap:
                          - from: /image
                            to: /burgerimage
                      param:
                          - name: history
                            value: keep_last
                      extra_arg:
                          - name: use_intra_process_comms
                            value: 'true'


Loading composable nodes into an existing container
---------------------------------------------------

Containers can sometimes be launched by other launch files or from a commandline.
In that case, you need to add your components to an existing container.
For this, you may use ``LoadComposableNodes`` to load components into a given container.
The below example launches the same nodes as above.

.. tabs::

  .. group-tab:: Python

    .. code-block:: python

      from launch import LaunchDescription
      from launch_ros.actions import LoadComposableNodes, Node
      from launch_ros.descriptions import ComposableNode

      def generate_launch_description():
          container = Node(
              name='image_container',
              package='rclcpp_components',
              executable='component_container',
              output='both',
          )

          load_composable_nodes = LoadComposableNodes(
              target_container='image_container',
              composable_node_descriptions=[
                  ComposableNode(
                       package='image_tools',
                      plugin='image_tools::Cam2Image',
                      name='cam2image',
                      remappings=[('/image', '/burgerimage')],
                      parameters=[{'width': 320, 'height': 240, 'burger_mode': True, 'history': 'keep_last'}],
                      extra_arguments=[{'use_intra_process_comms': True}],
                  ),
                  ComposableNode(
                      package='image_tools',
                      plugin='image_tools::ShowImage',
                      name='showimage',
                      remappings=[('/image', '/burgerimage')],
                      parameters=[{'history': 'keep_last'}],
                      extra_arguments=[{'use_intra_process_comms': True}]
                  ),
              ],
          )

          return LaunchDescription([container, load_composable_nodes])

  .. group-tab:: XML

    .. code-block:: xml

      <launch>
          <node pkg="rclcpp_components" exec="component_container" name="image_container">
          </node>
          <load_composable_node target="image_container">
              <composable_node pkg="image_tools" plugin="image_tools::Cam2Image" name="cam2image">
                  <remap from="/image" to="/burgerimage" />
                  <param name="width" value="320" />
                  <param name="height" value="240" />
                  <param name="burger_mode" value="true" />
                  <param name="history" value="keep_last" />
                  <extra_arg name="use_intra_process_comms" value="true" />
              </composable_node>
              <composable_node pkg="image_tools" plugin="image_tools::ShowImage" name="showimage" namespace="">
                  <remap from="/image" to="/burgerimage" />
                  <param name="history" value="keep_last" />
                  <extra_arg name="use_intra_process_comms" value="true" />
              </composable_node>
          </load_composable_node>
      </launch>

  .. group-tab:: YAML

    .. code-block:: yaml

      launch:
          - node_container:
              pkg: rclcpp_components
              exec: component_container
              name: image_container
              namespace: ''
              composable_node:
                  -   pkg: image_tools
                      plugin: image_tools::Cam2Image
                      name: cam2image
                      namespace: ''
                      remap:
                          - from: /image
                            to: /burgerimage
                      param:
                          - name: width
                            value: 320
                          - name: height
                            value: 240
                          - name: burger_mode
                            value: true
                          - name: history
                            value: keep_last
                      extra_arg:
                          - name: use_intra_process_comms
                            value: 'true'

                  -   pkg: image_tools
                      plugin: image_tools::ShowImage
                      name: showimage
                      namespace: ''
                      remap:
                          - from: /image
                            to: /burgerimage
                      param:
                          - name: history
                            value: keep_last
                      extra_arg:
                          - name: use_intra_process_comms
                            value: 'true'


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

Python, XML, or YAML: Which should I use?
-----------------------------------------

See the discussion in :doc:`Launch-file-different-formats` for more information.
