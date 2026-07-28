.. meta::
   :contentType: learning-path
   :experience: expert
   :area: simulation, capabilities
   :distribution: {DISTRO}
   :product: {PRODUCT}

.. redirect-from::

    Tutorials/Simulators/Simulation-Main
    Tutorials/Advanced/Simulators/Simulation-Main

.. _SimulationMain:

Configuring simulators with ROS 2 - learning path
=================================================

.. short-description::
   Advanced simulators help test robot behavior with physics, sensors, actuators, and realistic environments before hardware deployment.
   This learning path introduces the simulator tutorials for Webots, Gazebo, and MVSim.
   After completing it, you will know where to start when configuring a simulator for a ROS project.

.. showmeta::
   :order: area, contentType, experience
   :labels: area=Area, contentType=Content type, experience=Level

Several advanced robot simulators can be used with ROS 2, such as Gazebo, Webots, etc.
Unlike turtlesim, they provide fairly realistic results relying on physics-based models for robots, sensors, actuators and objects.
Hence, what you observe in simulation is very close to what you will get when transferring your ROS 2 controllers to a real robot.

This set of tutorials will teach you how to configure different simulators with ROS 2.

.. contents:: Contents
   :depth: 2
   :local:

.. toctree::
   :maxdepth: 1

   Webots/Simulation-Webots
   Gazebo/Simulation-Gazebo
   MVSim/Simulation-MVSim
