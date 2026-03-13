Defining worlds, robots, and sensors
=====================================

**Goal:** Learn the basics of defining MVSim world files, adding vehicles and sensors, and the main features available.

**Tutorial level:** Advanced

**Time:** 30 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

MVSim worlds are defined in XML files (``.world.xml``).
A world file describes the environment (ground, walls, obstacles), the vehicles (dynamics model, shape, sensors),
and simulation parameters (physics timestep, GUI options).

MVSim provides a library of predefined vehicle and sensor definitions that you can reuse in your worlds via XML includes.
You can also define everything from scratch for full control.

Prerequisites
-------------

You should have completed the :doc:`Getting-Started-MVSim` tutorial and have MVSim installed.

Tasks
-----

1 Minimal world file
^^^^^^^^^^^^^^^^^^^^^^

Here is a minimal world file that creates an empty environment with one robot:

.. code-block:: xml

    <mvsim_world version="1.0">
      <!-- Simulation settings -->
      <simul_timestep>5e-3</simul_timestep>

      <!-- GUI options -->
      <gui>
          <cam_distance>15</cam_distance>
      </gui>

      <!-- Ground plane -->
      <element class="ground_grid">
      </element>

      <!-- A differential-drive robot -->
      <vehicle name="robot1">
        <init_pose>0 0 0</init_pose>  <!-- x y yaw(deg) -->

        <!--  Dynamical model -->
        <dynamics class="differential">
            <!-- Params -->
            <l_wheel pos="0.0  0.5" mass="4.0" width="0.20" diameter="0.40" />
            <r_wheel pos="0.0 -0.5" mass="4.0" width="0.20" diameter="0.40" />

            <!-- Visual and physical shape -->
            <chassis mass="15.0" zmin="0.05" zmax="0.6">
            </chassis>

            <!--   Motor controller -->
            <controller class="twist_pid">
                <!-- Params -->
                <KP>4.1543</KP>
                <KI>1.9118</KI>
                <KD>0.0000</KD>
                <max_torque>14.44</max_torque>
                <V>0.0</V><W>0</W>
            </controller>
        </dynamics>

        <!-- Motor controller: accept twist commands, PID controller -->
        <controller class="twist_pid">
          <KP>100</KP> <KI>5</KI> <max_torque>50</max_torque>
        </controller>
      </vehicle>
    </mvsim_world>

Save this as ``my_world.world.xml`` and launch it:

.. code-block:: console

    $ mvsim launch my_world.world.xml

2 Using predefined vehicles and sensors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Instead of defining vehicles from scratch, you can use the predefined definitions that ship with MVSim.
These are XML files in the ``definitions/`` directory of the MVSim package.

**Available vehicles:**

- ``turtlebot3_burger.vehicle.xml`` -- TurtleBot3 Burger (differential drive)
- ``jackal.vehicle.xml`` -- Clearpath Jackal UGV (4-wheel differential)
- ``ackermann.vehicle.xml`` -- Generic Ackermann (car-like) vehicle
- ``pickup.vehicle.xml`` -- Pickup truck (Ackermann)
- ``agricobiot2.vehicle.xml`` -- Agricultural robot (Ackermann drivetrain)

**Available sensors:**

- ``lidar2d.sensor.xml`` -- Generic 2D laser scanner
- ``rplidar-a2.sensor.xml`` -- RPLidar A2
- ``velodyne-vlp16.sensor.xml`` -- Velodyne VLP-16 3D LiDAR
- ``ouster-os1.sensor.xml`` -- Ouster OS1 3D LiDAR
- ``helios-32-FOV-70.sensor.xml`` -- Helios 32-beam 3D LiDAR
- ``camera.sensor.xml`` -- RGB camera
- ``rgbd_camera.sensor.xml`` -- Depth camera (RGBD)
- ``imu.sensor.xml`` -- Inertial measurement unit
- ``gnss.sensor.xml`` -- GPS/GNSS receiver

To use a predefined vehicle with sensors attached, use XML includes:

.. code-block:: xml

    <mvsim_world version="1.0">
      <simul_timestep>5e-3</simul_timestep>

      <gui>
          <cam_distance>15</cam_distance>
      </gui>

      <element class="ground_grid">
      </element>

      <!-- Include the Jackal vehicle definition -->
      <include file="$(ros2 pkg prefix mvsim)/share/mvsim/definitions/jackal.vehicle.xml"
        default_sensors="true"
        />

      <vehicle name="r1" class="jackal">
        <init_pose>0 0 170</init_pose>  <!-- In global coords: x,y, yaw(deg) -->
        <init_vel>0 0 0</init_vel>  <!-- In local coords: vx,vy, omega(deg/s) -->

        <!-- You can also attach sensors to vehicles in separate includes,
            or define them inline within the vehicle block. -->
        <!-- <include file="$(ros2 pkg prefix mvsim)/share/mvsim/definitions/lidar2d.sensor.xml" sensor_x="1.7" sensor_z="1.01" sensor_yaw="0" max_range="70.0" sensor_name="laser1" />  -->
      </vehicle>

      <variable name="WALL_THICKNESS" value="0.2"></variable>

      <!-- Wall with a single door -->
      <element class="vertical_plane">
        <x0>-10</x0> <y0>-10</y0>
        <x1>-10</x1> <y1>10</y1>
        <z>0.0</z> <height>3.0</height>
        <cull_face>NONE</cull_face>
        <texture>https://mrpt.github.io/mvsim-models/textures-cgbookcase/wall-bricks-01.png</texture>
        <texture_size_x>2.5</texture_size_x>
        <texture_size_y>2.5</texture_size_y>
        <thickness>${WALL_THICKNESS}</thickness>  <!-- Wall thickness in meters -->

        <!-- Door at 50% position (middle of wall), 1.2m wide, standard height -->
        <door>
          <position>0.5</position>  <!-- 0.0 = start point, 1.0 = end point -->
          <width>1.2</width>        <!-- meters -->
          <z_min>0.0</z_min>        <!-- bottom of door -->
          <z_max>2.1</z_max>        <!-- top of door -->
          <name>main_entrance</name>
        </door>
      </element>


    </mvsim_world>

3 World environment elements
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

MVSim supports several types of environment elements:

**Occupancy grid maps** load a grayscale image as a 2D obstacle map, commonly used for indoor navigation testing:

.. code-block:: xml

    <element class="occupancy_grid">
      <file>map.png</file>
      <resolution>0.05</resolution>  <!-- meters/pixel -->
    </element>

**Elevation maps** define terrain height from a grayscale heightmap image, useful for outdoor scenarios:

.. code-block:: xml

    <element class="elevation_map">
      <resolution>1.0</resolution>
      <elevation_image>terrain.png</elevation_image>
      <elevation_image_min_z>0.0</elevation_image_min_z>
      <elevation_image_max_z>5.0</elevation_image_max_z>
    </element>

**Textured planes** add visual ground surfaces:

.. code-block:: xml

    <element class="horizontal_plane">
      <cull_face>BACK</cull_face>
      <x_min>-25</x_min> <x_max>25</x_max>
      <y_min>-25</y_min> <y_max>25</y_max>
      <z>0.0</z>
      <texture>asphalt.png</texture>
      <texture_size_x>5.0</texture_size_x>
      <texture_size_y>5.0</texture_size_y>
    </element>

**Blocks** are static or dynamic rigid bodies (boxes, custom shapes) that serve as obstacles or manipulable objects:

.. code-block:: xml

    <block class="obstacle1">
      <shape_from_visual/>
      <visual>
        <model_uri>package://mvsim/models/box.dae</model_uri>
      </visual>
      <init_pose>3.0 2.0 0</init_pose>
      <mass>20</mass>
    </block>

4 Vehicle dynamics models
^^^^^^^^^^^^^^^^^^^^^^^^^

MVSim provides three main dynamics models:

- **Differential drive** (``class="differential"``): Two-wheeled robots like TurtleBot3.
  Controlled via linear and angular velocity.

- **Ackermann** (``class="ackermann"``): Car-like vehicles with front-wheel steering.
  Controlled via linear velocity and steering angle.

- **Ackermann drivetrain** (``class="ackermann_drivetrain"``): Realistic drivetrain model with
  open or Torsen differentials, useful for more accurate vehicle behavior simulation.

Each vehicle can use different motor controllers:

- ``twist_pid``: Accepts ``geometry_msgs/msg/Twist`` commands with PID velocity tracking.
  This is the most common choice for ROS 2 integration.
- ``twist_ideal``: Instantaneous velocity commands (no dynamics delay).
- ``twist_front_steer_pid``: For Ackermann vehicles controlled via linear velocity and steering angle.
- ``raw``: Direct wheel torque control.

5 Sensor noise and configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Sensors in MVSim support configurable noise models.
For example, an IMU sensor with noise parameters:

.. code-block:: xml

    <sensor class="imu" name="imu1">
      <pose>0 0 0.5 0 0 0</pose>  <!-- x y z roll pitch yaw -->
      <rate_hz>100</rate_hz>

      <!-- Gyroscope noise -->
      <gyroscope_noise>
        <noise_std>1e-3</noise_std>           <!-- rad/s -->
        <bias_initial_std>1e-4</bias_initial_std>
        <bias_drift>1e-6</bias_drift>
      </gyroscope_noise>

      <!-- Accelerometer noise -->
      <accelerometer_noise>
        <noise_std>1e-2</noise_std>           <!-- m/s^2 -->
        <bias_initial_std>1e-3</bias_initial_std>
        <bias_drift>1e-5</bias_drift>
      </accelerometer_noise>
    </sensor>

LiDAR sensors support parameters for range, angular resolution, and noise:

.. code-block:: xml

    <sensor class="laser" name="laser1">
      <pose>0.15 0 0.3 0 0 0</pose>
      <rate_hz>10</rate_hz>
      <ray_count>360</ray_count>
      <fov_degrees>360</fov_degrees>
      <range_max>20.0</range_max>
      <range_std_noise>0.01</range_std_noise>  <!-- meters -->
      <raytrace_3d>true</raytrace_3d>  <!-- use 3D collision for 2D scans -->
    </sensor>

6 Additional features
^^^^^^^^^^^^^^^^^^^^^^

**Multi-robot simulation:**
MVSim natively supports multiple vehicles in the same world.
Each vehicle gets its own ROS 2 namespace, TF tree, and set of topics.
Robots can detect each other with their sensors and physically interact through collisions.

**Property regions:**
You can define regions in the world with different physical properties, such as varying friction coefficients
or GPS-denied zones where GNSS sensors stop reporting positions.

**Animated actors:**
MVSim supports skeletal-animated 3D characters (e.g., pedestrians) that follow waypoint paths,
useful for testing perception and planning in dynamic environments.

**Joints and articulated vehicles:**
Vehicles can be connected using distance joints (ropes/cables) or revolute joints (hinges),
enabling simulation of trailers, tow ropes, and articulated systems.

**XML advanced features:**
World files support ``<include>`` directives, variable substitution, mathematical expressions,
``<for>`` loops, and ``<if>`` conditionals, making it possible to procedurally generate complex environments.

**Headless and faster-than-real-time:**
MVSim can run without a GUI and at configurable simulation speed,
which is useful for automated testing and reinforcement learning workflows.

Comparison with other simulators
--------------------------------

MVSim occupies a different niche compared to other simulators:

**Strengths:**

- Very lightweight: low CPU and memory usage, fast startup times.
- Focused vehicle dynamics with multiple friction and drivetrain models.
- Simple XML-based world format, easy to get started.
- Native multi-robot support with per-vehicle ROS 2 namespaces.
- Faster-than-real-time simulation for batch testing.
- Procedural world generation via XML loops and conditionals.

**Limitations:**

- Physics is 2D (Box2D): no full 3D rigid body dynamics.
  Objects do not tip over or fly.
  Elevation maps add terrain height but the physics remains fundamentally 2D.
- Sensor simulation is less detailed than full 3D simulators: camera rendering and LiDAR models
  are functional but not photorealistic.
- Smaller ecosystem of pre-built models and environments compared to Gazebo.
- Focused on wheeled mobile robots.

Further resources
-----------------

- `MVSim documentation <https://mvsimulator.readthedocs.io/>`__
- `MVSim GitHub repository <https://github.com/MRPT/mvsim>`__
- `MVSim paper (SoftwareX) <https://doi.org/10.1016/j.softx.2023.101443>`__
