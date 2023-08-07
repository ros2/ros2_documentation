.. redirect-from::

   RQt-Overview-Usage
   Tutorials/RQt-Overview-Usage
   Concepts/About-RQt

Overview and usage of RQt
=========================

.. contents:: Table of Contents
   :local:

Overview
--------

RQt is a graphical user interface framework that implements various tools and interfaces in the form of plugins.
One can run all the existing GUI tools as dockable windows within RQt.
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
To see what RQt plugins are available for your system, run:

.. code-block:: bash

   ros2 pkg list

And then look for packages that start with ``rqt_``.

System setup
------------

Installing From Debian
^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   sudo apt install ros-{DISTRO}-rqt*


Building From Source
^^^^^^^^^^^^^^^^^^^^

See :doc:`Building RQt from Source <../../How-To-Guides/RQt-Source-Install>`.

RQt Components Structure
------------------------

RQt consists of two metapackages:

* *rqt* - core infrastucture modules.
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
* Manageable lifecycle: RQt plugins using a common API makes maintainance & reuse easier.


Further Reading
---------------

* ROS 2 Discourse `announcement of porting to ROS 2 <https://discourse.ros.org/t/rqt-in-ros2/6428>`__)
* `RQt for ROS 1 documentation <https://wiki.ros.org/rqt>`__
* Brief overview of RQt (from `a Willow Garage intern blog post <http://web.archive.org/web/20130518142837/http://www.willowgarage.com/blog/2012/10/21/ros-gui>`__)

  .. raw:: html

     <iframe width="560" height="315" src="https://www.youtube-nocookie.com/embed/CyP9wHu2PpY" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
