.. redirect-from::

    Tutorials/Simulators/Ignition/Setting-up-a-Robot-Simulation-Ignition
    Tutorials/Advanced/Simulators/Ignition
    Tutorials/Advanced/Simulators/Gazebo

Setting up a robot simulation (Gazebo)
======================================

**Goal:** Launch a Simulation with Gazebo and ROS 2

**Tutorial level:** Advanced

**Time:** 5 minutes

.. contents:: Contents
   :depth: 2
   :local:


.. note::

   These instructions are about the current `Gazebo <https://gazebosim.org/>`__ (previously known as Ignition), not  `Gazebo Classic <https://classic.gazebosim.org/>`__.

Prerequisites
-------------

You'll need to install both ROS 2 and Gazebo.

ROS 2
^^^^^

For ROS 2 you should follow the :doc:`ROS 2 install instructions <../../../../Installation>`.

Gazebo
^^^^^^

Gazebo and ROS support different combinations of versions.

All supported combinations can be seen `here <https://gazebosim.org/docs/ionic/ros_installation#summary-of-compatible-ros-and-gazebo-combinations>`__.

`ROS REP-2000 <https://www.ros.org/reps/rep-2000.html>`__ standardizes what the default version of Gazebo is for each ROS distribution.

If you haven't installed a version of Gazebo on your system yet, you can install Gazebo by following the `installation instructions <https://gazebosim.org/docs/ionic/ros_installation>`__.

Quick Check
-----------

To verify your Gazebo installation is correct, check you can run it:

.. code-block:: console

   $ gz sim

Further Resources
-----------------

Once Gazebo is installed and is all clear on the last quick test, you can move to the `Gazebo tutorials <https://gazebosim.org/docs/ionic/tutorials>`__ to try out building your own robot!

If you use a different version of Gazebo than the recommended version, make sure to use the dropdown to select the correct version of documentation.

Summary
-------

In this tutorial, you have installed Gazebo and set-up your workspace to start with the `Gazebo tutorials <https://gazebosim.org/docs/ionic/tutorials>`__.
