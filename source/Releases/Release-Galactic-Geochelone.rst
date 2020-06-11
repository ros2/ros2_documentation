.. _upcoming-release:

.. move this directive when next release page is created

ROS 2 Galactic Geochelone (codename 'galactic'; May, 2021)
====================================================

.. contents:: Table of Contents
   :depth: 2
   :local:

*Galactic Geochelone* is the seventh release of ROS 2.

Supported Platforms
-------------------

TBD


New features in this ROS 2 release
----------------------------------

TBD

Changes since the Foxy release
------------------------------

tf2_ros Python split out of tf2_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The Python code that used to live in tf2_ros has been moved into its own package named tf2_ros_py.
Any existing Python code that depends on tf2_ros will continue to work, but the package.xml of those packages should be amended to ``exec_depend`` on tf2_ros_py.


Known Issues
------------

Timeline before the release
---------------------------

TBD
