.. meta::
   :contentType: tutorial
   :experience: expert
   :area: simulation, capabilities
   :distribution: {DISTRO}
   :product: {PRODUCT}

.. redirect-from::

    Tutorials/Advanced/Simulators/MVSim/Simulation-MVSim

Configuring MVSim with ROS 2 - tutorial
=======================================

.. short-description::
   MVSim is useful for fast mobile robot simulation, navigation testing, and multi-robot scenarios.
   This tutorial set introduces installation, demo launch, and world-definition tasks for MVSim.
   After using it, you will know which MVSim tutorial to follow for setup or custom scenario work.

.. showmeta::
   :order: area, contentType, experience
   :labels: area=Area, contentType=Content type, experience=Level

This set of tutorials will teach you how to configure the `MVSim <https://mvsimulator.readthedocs.io/>`__ simulator with ROS 2.

MVSim is a lightweight, open-source, multi-vehicle simulator focused on 2D+3D visualization of mobile robots.
It uses Box2D for 2D rigid body physics and provides realistic vehicle dynamics models (differential drive, Ackermann steering),
sensor simulation (2D/3D LiDARs, cameras, IMUs, GPS), and native ROS 2 integration.
MVSim is particularly well-suited for testing navigation, SLAM, and multi-robot coordination scenarios
with low computational overhead and fast iteration times.

.. contents:: Contents
   :depth: 2
   :local:

.. toctree::
   :maxdepth: 1

   Installation-Ubuntu
   Getting-Started-MVSim
   Defining-Worlds-MVSim
