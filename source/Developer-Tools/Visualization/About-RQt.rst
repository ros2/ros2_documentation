.. redirect-from::

   RQt-Overview-Usage
   Tutorials/RQt-Overview-Usage
   Concepts/About-RQt
   Concepts/Intermediate/About-RQt

.. meta::
   :contentType: about
   :experience: intermediate
   :area: visualization, tools
   :distribution: {DISTRO}
   :product: {PRODUCT}

Overview and using RQt
======================

.. short-description::
   RQt provides a flexible graphical framework for running ROS tools and plugins in a shared, dockable interface.
   This article introduces RQt, explains how to run available plugins, and describes the main components and advantages of using the RQt framework.

.. showmeta::
   :order: area, contentType, experience
   :labels: area=Area, contentType=Content type, experience=Level

.. contents:: Table of Contents
   :depth: 2
   :local:

Overview
--------

RQt is a graphical user interface framework that implements various tools and interfaces in the form of plugins.
One can run all the existing GUI tools as dockable windows within RQt.
The tools can still run in a traditional standalone method, but RQt makes it easier to manage all the various windows in a single screen layout.

You can run any RQt tools/plugins easily by:

.. code-block:: console

   $ rqt

This GUI allows you to choose any available plugins on your system.
You can also run plugins in standalone windows.
For example, RQt Python Console:

.. code-block:: console

   $ ros2 run rqt_py_console rqt_py_console

Users can create their own plugins for RQt with either ``Python`` or ``C++``.
To see what RQt plugins are available for your system, run:

.. code-block:: console

   $ ros2 pkg list

And then look for packages that start with ``rqt_``.

System setup
------------

Installing From debs
^^^^^^^^^^^^^^^^^^^^

.. code-block:: console

   $ sudo apt install ros-{DISTRO}-rqt*


RQt Components Structure
------------------------

RQt consists of two metapackages:

* *rqt* - core infrastructure modules.
* *rqt_common_plugins* - Commonly useful debugging tools.

Advantage of RQt framework
--------------------------

Compared to building your own GUIs from scratch:

* Standardized common procedures for GUI (start-shutdown hook, restore previous states).
* Multiple widgets can be docked in a single window.
* Easily turn your existing Qt widgets into RQt plugins.
* Expect support at `Robotics Stack Exchange <https://robotics.stackexchange.com/>`__ (ROS community website for the questions).

From system architecture's perspective:

* Support multi-platform (basically wherever `QT <http://qt-project.org/>`__ and ROS run) and multi-language (``Python``, ``C++``).
* Manageable lifecycle: RQt plugins using a common API makes maintenance & reuse easier.


Further Reading
---------------

* ROS 2 Discourse `announcement of porting to ROS 2 <https://discourse.openrobotics.org/t/rqt-in-ros2/6428>`__)
* `RQt for ROS 1 documentation <https://wiki.ros.org/rqt>`__
* Brief overview of RQt (from `a Willow Garage intern blog post <http://web.archive.org/web/20130518142837/http://www.willowgarage.com/blog/2012/10/21/ros-gui>`__)
