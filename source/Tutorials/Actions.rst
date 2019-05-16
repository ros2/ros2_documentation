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
For more detailed information about ROS actions, please refer to the `design article <http://design.ros2.org/articles/actions.html>`__.

This document contains a list of tutorials related to actions.
Each tutorial builds on the previous, and so it is recommended that they are completed in order.
By the end of all the tutorials, you should expect to have a ROS package that looks the package `action_tutorials <https://github.com/ros2/demos/tree/master/action_tutorials>`__.

Prequisites
-----------

- `Install ROS <../Installation>
- `Install colcon <https://colcon.readthedocs.org>`__
- Setup a workspace create a package named ``action_tutorials``:

.. code-block:: bash

    mkdir -p action_ws/src
    cd action_ws/src
    ros2 pkg create action_tutorials

Tutorials
---------

.. toctree::
   :maxdepth: 1

   Actions/Creating-an-Action
   Actions/Create-an-Action-Server-CPP
   Actions/Create-an-Action-Client-CPP
   Actions/Create-an-Action-Server-Python
   Actions/Create-an-Action-Client-Python
