.. _RQt_Overview_Usage:

.. redirect-from::

   RQt-Overview-Usage

Overview and usage of RQt
=========================

.. contents:: Table of Contents
   :depth: 2
   :local:

Overview
--------

RQt is a graphical user interface framework that implements various tools and interfaces in the form of plugins.
One can run all the existing GUI tools as dockable windows within RQt!
The tools can still run in a traditional standalone method, but RQt makes it easier to manage all the various windows in a single screen layout.

You can run any RQt tools/plugins easily by:

.. code-block:: bash

   rqt

This GUI allows you to choose any available plugins on your system.
You can also run plugins in standalone windows.
For example, RQt Python Console:

.. code-block:: bash

   ros2 run rqt_py_console rqt_py_console

Users can create their own plugins for RQt with either ``Python`` or ``C++``.
`Over 20 plugins <https://wiki.ros.org/rqt/Plugins>`__ were created in ROS 1 and these plugins are currently being ported to ROS 2 (as of Dec 2018, `more info <https://discourse.ros.org/t/rqt-in-ros2/6428>`__).


System setup
------------

Installing From Debian
^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   sudo apt install ros-rolling-rqt*


Building From Source
^^^^^^^^^^^^^^^^^^^^

.. toctree::
   :hidden:

   RQt-Source-Install

See `Building RQt from Source <RQt-Source-Install>`.

RQt Components Structure
------------------------

RQt consists of three metapackages:

* *rqt* - core infrastucture modules.
* *rqt_common_plugins* - Backend tools for building tools.
   TODO: as of Dec 2018 this metapackage isn't available in ROS 2 since not all plugins it contains have been ported yet.
* *rqt_robot_plugins* - Tools for interacting with robots during runtime.
   TODO: as of Dec 2018 this metapackage isn't available in ROS 2 since not all plugins it contains have been ported yet.

Advantage of RQt framework
--------------------------

Compared to building your own GUIs from scratch:

* Standardized common procedures for GUI (start-shutdown hook, restore previous states).
* Multiple widgets can be docked in a single window.
* Easily turn your existing Qt widgets into RQt plugins.
* Expect support at `ROS Answers <https://answers.ros.org>`__ (ROS community website for the questions).

From system architecture's perspective:

* Support multi-platform (basically wherever `QT <http://qt-project.org/>`__ and ROS run) and multi-language (``Python``, ``C++``).
* Manageable lifecycle: RQt plugins using common API makes maintainance & reuse easier.


Further Reading
---------------

* ROS 2 Discourse `announcment of porting to ROS 2 <https://discourse.ros.org/t/rqt-in-ros2/6428>`__).
* `RQt for ROS 1 documentation <https://wiki.ros.org/rqt>`__.
* Brief overview of RQt (from `a Willow Garage intern blog post <http://web.archive.org/web/20130518142837/http://www.willowgarage.com/blog/2012/10/21/ros-gui>`__).

  .. raw:: html

     <iframe width="560" height="315" src="https://www.youtube-nocookie.com/embed/CyP9wHu2PpY" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
