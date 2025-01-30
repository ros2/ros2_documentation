RViz User Guide
===============

**Goal:** Understanding RViz

**Tutorial level:** Intermediate

**Time:** 25 Minutes

.. contents:: Contents
   :depth: 2
   :local:


Background
----------
RViz is a 3D visualizer for the Robot Operating System (ROS) framework.

Install or build rviz
---------------------
Follow the :doc:`installation instructions <../../../../Installation>` for your operating system to install RViz.

Startup
-------
Don't forget to source the setup file.

.. code-block:: bash

   source /opt/ros/{DISTRO}/setup.bash

Then start the visualizer

.. code-block:: bash

   ros2 run rviz2 rviz2

When RViz starts for the first time, you will see this window:

.. image:: images/initial_startup.png

The big black window in the middle is the 3D view (empty because there is nothing to see).
On the left is the Displays list, which will show any displays you have loaded.
Right now it just contains the global options and a Grid, which we'll get to later.
On the right are some of the other panels, described below.

Displays
--------
A display is something that draws something in the 3D world, and likely has some options available in the displays list.
An example is a point cloud, the robot state, etc.

Adding a new display
^^^^^^^^^^^^^^^^^^^^
To add a display, click the Add button at the bottom:

.. image:: images/add-button.png

This will pop up the new display dialog:

.. image:: images/add-display-dialog.png

The list at the top contains the display type.
The type details what kind of data this display will visualize.
The text box in the middle gives a description of the selected display type.
Finally, you must give the display a unique name.
If you have, for example, two laser scanners on your robot, you might create two ``Laser Scan`` displays named "Laser Base" and "Laser Head".

Display Properties
^^^^^^^^^^^^^^^^^^
Each display gets its own list of properties.
For example:

.. image:: images/display-properties.png

Display Status
^^^^^^^^^^^^^^
Each display gets its own status to help let you know if everything is OK or not.
The status can be one of: ``OK``, ``Warning``, ``Error``, or ``Disabled``.
The status is indicated in the display's title by the background color,
as well as in the Status category that you can see if the display is expanded:

.. image:: images/display-status.png

The ``Status`` category also expands to show specific status information.
This information is different for different displays, and the messages should be self explanatory.

Built-in Display Types
^^^^^^^^^^^^^^^^^^^^^^
.. list-table::
   :header-rows: 1
   :widths: 10 10 10

   * - Name
     - Description
     - Messages Used
   * - Axes
     - Displays a set of Axes
     -
   * - Effort
     - Shows the effort being put into each revolute joint of a robot
     - `sensor_msgs/msg/JointStates <https://github.com/ros2/common_interfaces/blob/{DISTRO}/sensor_msgs/msg/JointState.msg>`__
   * - Camera
     - Creates a new rendering window from the perspective of a camera, and overlays the image on top of it.
     - `sensor_msgs/msg/Image <https://github.com/ros2/common_interfaces/blob/{DISTRO}/sensor_msgs/msg/Image.msg>`__, `sensor_msgs/msg/CameraInfo <https://github.com/ros2/common_interfaces/blob/{DISTRO}/sensor_msgs/msg/CameraInfo.msg>`__
   * - Grid
     - Displays a 2D or 3D grid along a plane
     -
   * - Grid Cells
     - Draws cells from a grid, usually obstacles from a costmap from the `navigation <https://github.com/ros-planning/navigation2>`__ stack.
     - `nav_msgs/msg/GridCells <https://github.com/ros2/common_interfaces/blob/{DISTRO}/nav_msgs/msg/GridCells.msg>`__
   * - Image
     - Creates a new rendering window with an Image. Unlike the Camera display, this display does not use a CameraInfo
     - `sensor_msgs/msg/Image <https://github.com/ros2/common_interfaces/blob/{DISTRO}/sensor_msgs/msg/Image.msg>`__
   * - InteractiveMarker
     - Displays 3D objects from one or multiple Interactive Marker servers and allows mouse interaction with them
     - `visualization_msgs/msg/InteractiveMarker <https://github.com/ros2/common_interfaces/blob/{DISTRO}/visualization_msgs/msg/InteractiveMarker.msg>`__
   * - Laser Scan
     - Shows data from a laser scan, with different options for rendering modes, accumulation, etc.
     - `sensor_msgs/msg/LaserScan <https://github.com/ros2/common_interfaces/blob/{DISTRO}/sensor_msgs/msg/LaserScan.msg>`__
   * - Map
     - Displays a map on the ground plane.
     - `nav_msgs/msg/OccupancyGrid <https://github.com/ros2/common_interfaces/blob/{DISTRO}/nav_msgs/msg/OccupancyGrid.msg>`__
   * - Markers
     - Allows programmers to display arbitrary primitive shapes through a topic
     - `visualization_msgs/msg/Marker <https://github.com/ros2/common_interfaces/blob/{DISTRO}/visualization_msgs/msg/Marker.msg>`__, `visualization_msgs/msg/MarkerArray <https://github.com/ros2/common_interfaces/blob/{DISTRO}/visualization_msgs/msg/MarkerArray.msg>`__
   * - Path
     - Shows a path from the `navigation <https://github.com/ros-planning/navigation2>`__ stack.
     - `nav_msgs/msg/Path <https://github.com/ros2/common_interfaces/blob/{DISTRO}/nav_msgs/msg/Path.msg>`__
   * - Point
     - Draws a point as a small sphere.
     - `geometry_msgs/msg/PointStamped <https://github.com/ros2/common_interfaces/blob/{DISTRO}/geometry_msgs/msg/PointStamped.msg>`__
   * - Pose
     - Draws a pose as either an arrow or axes.
     - `geometry_msgs/msg/PoseStamped <https://github.com/ros2/common_interfaces/blob/{DISTRO}/geometry_msgs/msg/PoseStamped.msg>`__
   * - Pose Array
     - Draws a "cloud" of arrows, one for each pose in a pose array
     - `geometry_msgs/msg/PoseArray <https://github.com/ros2/common_interfaces/blob/{DISTRO}/geometry_msgs/msg/PoseArray.msg>`__
   * - Point Cloud(2)
     - Shows data from a point cloud, with different options for rendering modes, accumulation, etc.
     - `sensor_msgs/msg/PointCloud <https://github.com/ros2/common_interfaces/blob/{DISTRO}/sensor_msgs/msg/PointCloud.msg>`__, `sensor_msgs/msg/PointCloud2 <https://github.com/ros2/common_interfaces/blob/{DISTRO}/sensor_msgs/msg/PointCloud2.msg>`__
   * - Polygon
     - Draws the outline of a polygon as lines.
     - `geometry_msgs/msg/Polygon <https://github.com/ros2/common_interfaces/blob/{DISTRO}/geometry_msgs/msg/Polygon.msg>`__
   * - Odometry
     - Accumulates odometry poses from over time.
     - `nav_msgs/msg/Odometry <https://github.com/ros2/common_interfaces/blob/{DISTRO}/nav_msgs/msg/Odometry.msg>`__
   * - Range
     - Displays cones representing range measurements from sonar or IR range sensors. Version: Electric+
     - `sensor_msgs/msg/Range <https://github.com/ros2/common_interfaces/blob/{DISTRO}/sensor_msgs/msg/Range.msg>`__
   * - RobotModel
     - Shows a visual representation of a robot in the correct pose (as defined by the current TF transforms).
     -
   * - TF
     - Displays the `tf2 <https://github.com/ros2/geometry2>`__ transform hierarchy.
     -
   * - Wrench
     - Draws a wrench as arrow (force) and arrow + circle (torque)
     - `geometry_msgs/msg/WrenchStamped <https://github.com/ros2/common_interfaces/blob/{DISTRO}/geometry_msgs/msg/WrenchStamped.msg>`__
   * - Twist
     - Draws a twist as arrow (linear) and arrow + circle (angular)
     - `geometry_msgs/msg/TwistStamped <https://github.com/ros2/common_interfaces/blob/{DISTRO}/geometry_msgs/msg/TwistStamped.msg>`__

Configurations
--------------
Different configurations of displays are often useful for different uses of the visualizer.
A configuration useful for a full PR2 is not necessarily useful for a test cart, for example.
To this end, the visualizer lets you load and save different configurations.

A configuration contains:

* Displays + their properties
* Tool properties
* The viewpoint and settings for the 3D visualization

Views Panel
-----------
There are a number of different camera types available in the visualizer.

.. image:: images/camera-types.png

Camera types consist both of different ways of controlling the camera and different types of projection (Orthographic vs. Perspective).

Orbital Camera (default)
^^^^^^^^^^^^^^^^^^^^^^^^
The orbital camera simply rotates around a focal point, while always looking at that point.
The focal point is visualized as a small disc while you're moving the camera:

.. image:: images/focal-point.png

Controls:

* **Left mouse button**: Click and drag to rotate around the focal point.
* **Middle mouse button**: Click and drag to move the focal point in the plane formed by the camera's up and right vectors. The distance moved depends on the focal point -- if there is an object on the focal point, and you click on top of it, it will stay under your mouse.
* **Right mouse button**: Click and drag to zoom in/out of the focal point. Dragging up zooms in, down zooms out.
* **Scrollwheel**: Zoom in/out of the focal point

FPS (first-person) Camera
^^^^^^^^^^^^^^^^^^^^^^^^^
The FPS camera is a first-person camera, so it rotates as if you're looking with your head.

Controls:

* **Left mouse button**: Click and drag to rotate. Control-click to pick the object under the mouse and look directly at it.
* **Middle mouse button**: Click and drag to move along the plane formed by the camera's up and right vectors.
* **Right mouse button**: Click and drag to move along the camera's forward vector. Dragging up moves forward, down moves backward.
* **Scrollwheel**: Move forward/backward.

Top-down Orthographic
^^^^^^^^^^^^^^^^^^^^^
The top-down orthographic camera always looks down along the Z axis (in the robot frame),
and is an orthographic view which means things do not get smaller as they get farther away.

Controls:

* **Left mouse button**: Click and drag to rotate around the Z axis.
* **Middle mouse button**: Click and drag to move the camera along the XY plane.
* **Right mouse button**: Click and drag to zoom the image.
* **Scrollwheel**: Zoom the image.

XY Orbit
^^^^^^^^
Same as the orbital camera, with the focus point restricted to the XY plane.

Controls:

See orbital camera.

Third Person Follower
^^^^^^^^^^^^^^^^^^^^^
The camera maintains a constant viewing angle towards the target frame.
In contrast to XY Orbit the camera turns if the target frame yaws.
This could be handy if you are doing 3D mapping of a hallway with corners for example.

Controls:

See orbital camera.


Custom Views
^^^^^^^^^^^^
The views panel also lets you create different named views, which are saved and can be switched between.
A view consists of a target frame, camera type and camera pose.
You can save a view by clicking the Save button of the views panel.

.. image:: images/views.png

A view consists of:

* View controller type
* View configuration (position, orientation, etc; possibly different for each view controller type.)
* The Target Frame

Views are saved per user, not in the config files.

Coordinate Frames
-----------------
RViz uses the tf transform system for transforming data from the coordinate frame it arrives in into a global reference frame.
There are two coordinate frames that are important to know about in the visualizer, the target frame and the fixed frame.

The Fixed Frame
^^^^^^^^^^^^^^^
The more-important of the two frames is the fixed frame.
The fixed frame is the reference frame used to denote the ``world`` frame.
This is usually the ``map``, or ``world``, or something similar, but can also be, for example, your odometry frame.

If the fixed frame is erroneously set to, say, the base of the robot, then all the objects the robot has ever seen will appear in front of the robot, at the position relative to the robot at which they were detected.
For correct results, the fixed frame should not be moving relative to the world.

If you change the fixed frame, all data currently being shown is cleared rather than re-transformed.

The Target Frame
^^^^^^^^^^^^^^^^
The target frame is the reference frame for the camera view.
For example, if your target frame is the map, you'll see the robot driving around the map.
If your target frame is the base of the robot, the robot will stay in the same place while everything else moves relative to it.

Tools
-----
The visualizer has a number of tools you can use on the toolbar.
The following sections will give a short introduction into these tools.
You can find some more information under Help -> Show Help panel.

.. image:: images/tool.png

Interact
^^^^^^^^
This tool lets you interact with the visualized environment.
You can click on objects and depending on their properties simply select them, move them around and much more.

Keyboard shortcut: ``i``

Move Camera
^^^^^^^^^^^
The Move Camera tool is the default tool.
When this is selected and you click inside the 3D view, the viewpoint changes according to the options and camera type you have selected in the ``Views`` panel. See the previous section ``Views Panel`` for more information.

Keyboard shortcut: ``m``

Select
^^^^^^
The Select tool allows you to select items being displayed in the 3D view.
It supports single-point selection as well as click/drag box selection.
You can add to a selection with the Shift key, and remove from the selection with the Ctrl key.
If you want to move the camera around while selecting without switching back to the Move Camera tool you can hold down the Alt key.
The ``f`` key will focus the camera on the current selection.

.. image:: images/selection_highlight.png

.. image:: images/selection_selected.png

Keyboard shortcut: ``s``

Focus Camera
^^^^^^^^^^^^
Focus camera lets you select a location in the visualizer.
The camera will then focus that point by changing its orientation but not its position.

Keyboard shortcut: ``c``

Measure
^^^^^^^
With the measure tool you can measure the distance between to points in the visualizer.
The first click after activating the tool will set the starting point and the second one the end point of the measurement.
The resulting distance will be displayed at the bottom of the RViz window.
But notice that the measurement tool only works with actually rendered objects in the visualizer, you can not use it in empty space.

.. image:: images/measure.png

Keyboard shortcut: ``n``

2D Pose Estimate
^^^^^^^^^^^^^^^^
This tool lets you set an initial pose to seed the localization system (sent on the ``initialpose`` ROS topic).
Click on a location on the ground plane and drag to select the orientation.
The output topic can be changed in the ``Tool Properties`` panel.

.. image:: images/set_pose.png

This tool works with the `navigation <https://github.com/ros-planning/navigation2>`__ stack.

Keyboard shortcut: ``p``

2D Nav Goal
^^^^^^^^^^^
This tool lets you set a goal sent on the ``goal_pose`` ROS topic.
Click on a location on the ground plane and drag to select the orientation.
The output topic can be changed in the ``Tool Properties`` panel.

This tool works with the `navigation <https://github.com/ros-planning/navigation2>`__ stack.

Keyboard shortcut: ``g``

Publish Point
^^^^^^^^^^^^^
The publish point tool lets you select an object in the visualizer
and the tool will publish the coordinates of that point based on the frame.
The results are shown at the bottom like with the measure tool but are also published on the ``clicked_point`` topic.

Keyboard shortcut: ``u``

Time
----
The Time panel is mostly useful when running in a simulator, since it allows you to see how much ROS Time has passed, vs. how much ``Wall Clock`` (aka real) time has passed.
The time panel also lets you reset the visualizer's internal time state, which resets of all the displays as well as tf's internal cache of data.

.. image:: images/time.png

If you are not running in simulation, the time panel is mostly useless.
In most cases it can be closed and you will probably not even notice (other than having a bit more screen real estate for the rest of rviz).
