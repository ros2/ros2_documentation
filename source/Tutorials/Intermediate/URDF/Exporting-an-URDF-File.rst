.. redirect-from::

    Tutorials/URDF/Exporting-an-URDF-file

.. _URDFXacro:

Generating an URDF File
=================================

**Goal:** Learn how to Export and URDF File

**Tutorial level:** Intermediate

**Time:** 5 minutes

.. contents:: Contents
   :depth: 2
   :local:

Most roboticists work in teams, and often those teams include a mechanical engineer who develops a CAD model of robot.
Instead of crafting an URDF by hand it is possible to export an URDF model from many different CAD and modeling programs.
These export tools are often developed by individuals that are familiar with the particular CAD program they use.
Below you will find a list of available URDF exporters for a variety of CAD and 3D modeling software systems.
*The ROS core maintainers do not maintain these packages. As such we make no claims about their performance or ease of use.*
However, we figured it would be helpful to produce a list of available URDF exporters.

**CAD Exporters**

 * `Fusion 360 URDF Exporter <https://github.com/syuntoku14/fusion2urdf>`_
 * `SolidWorkds URDF Exporter <https://github.com/ros/solidworks_urdf_exporter>`_
 * `ROS Workbench for FreeCAD <https://github.com/galou/freecad.cross>`_
 * `OnShape URDF Exporter <https://github.com/Rhoban/onshape-to-robot>`_
 * `CREO Parametric URDF Exporter <https://github.com/icub-tech-iit/creo2urdf>`_


**Other URDF Export Tools**

 * `Copellia Sim URDF Exporter <https://manual.coppeliarobotics.com/en/importExport.htm#urdf>`_
 * `Isaac Sim URDF Exporter <https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_advanced_export_urdf.html>`_
 * `Blender URDF Exporter <https://github.com/dfki-ric/phobos>`_
 * `Gazebo SDFormat to URDF Parser <https://github.com/ros/sdformat_urdf/tree/ros2>`_
 * `SDF to URDF Converter in Python <https://github.com/andreasBihlmaier/pysdf>`_
 * The `Blender Robotics Tools <https://github.com/robotology/blender-robotics-utils/>`_ respository includes a number of useful tools, including a tool to export `URDF files from Blender. <https://github.com/robotology/blender-robotics-utils/tree/master?tab=readme-ov-file#urdftoblender>`_

If you have an URDF tool you like please consider adding it to the list above!
