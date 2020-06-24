.. _Actions:

Actions
=======

.. contents:: Table of Contents
   :depth: 2
   :local:

About
-----

Actions are a form of asynchronous communication in ROS.
*Action clients* send goal requests to *action servers*.
*Action servers* send goal feedback and results to *action clients*.
For more detailed information about ROS actions, please refer to the `design article <https://design.ros2.org/articles/actions.html>`__.

This document contains a list of tutorials related to actions.
For reference, after completing all of the tutorials you should expect to have a ROS package that looks like the package `action_tutorials <https://github.com/ros2/demos/tree/master/action_tutorials>`__.

Prequisites
-----------

- `Install ROS (Dashing or later) <../Installation>`

- `Install colcon <https://colcon.readthedocs.org>`__

- Setup a workspace and create a package named ``action_tutorials``:

  Remember to source your ROS 2 installation.

  Linux / OSX:

  .. code-block:: bash

      mkdir -p action_ws/src
      cd action_ws/src
      ros2 pkg create action_tutorials

  Windows:

  .. code-block:: bash

      mkdir -p action_ws\src
      cd action_ws\src
      ros2 pkg create action_tutorials

Tutorials
---------

.. toctree::
   :maxdepth: 1

   Actions/Creating-an-Action
   Actions/Writing-an-Action-Server-CPP
   Actions/Writing-an-Action-Client-CPP
   Actions/Writing-an-Action-Server-Python
   Actions/Writing-an-Action-Client-Python
