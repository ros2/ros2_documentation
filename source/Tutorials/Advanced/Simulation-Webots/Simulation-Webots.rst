Simulation [Webots]
===================

Several advanced robot simulators can be used with ROS 2, such as Gazebo, Webots, etc.
Unlike turtlesim, they provide fairly realistic results relying on physics-based models for robots, sensors, actuators and objects.
Hence, what you observe in simulation is very close to what you will get when transferring your ROS 2 controllers to a real robot.

Webots differs from Gazebo in several aspects as described `here <https://en.wikipedia.org/wiki/Robotics_simulator>`_. Comparing to Gazebo:

- Webots comes with a forked version of the ODE physics engine which was tuned for grasping and extended with fluid dynamics for drones and underwater robots.
- Webots provides a custom 3D physically-based rendering (PBR) engine allowing to model camera devices fairly accurately.
- Webots features an easy-to-use web interface allowing to share simulations, animation and models on the web, as seen on `webots.cloud <https://webots.cloud>`_.
- Webots includes a library of models (robots, sensors, actuators, objects, etc.) which are optimized for performance to make simulations run fast.
- Webots runs natively on Linux, Windows and macOS with 3D hardware acceleration for 3D view, simulated cameras, lidars and range-finders.

This set of tutorials will teach you how to configure the Webots simulator with ROS 2.

.. contents:: Contents
   :depth: 2
   :local:

.. toctree::
   :maxdepth: 1

   Installation-Ubuntu
   Installation-Windows
   Installation-MacOS
   Setting-Up-Simulation-Webots
