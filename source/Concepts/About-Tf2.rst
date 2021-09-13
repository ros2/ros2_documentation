.. _AboutTf2:

About tf2
=========

.. contents:: Table of Contents
   :depth: 2
   :local:

Overview
--------

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

See `Building RQt from Source <../Guides/RQt-Source-Install>`.


Further Reading
---------------

* ROS 2 Discourse `announcment of porting to ROS 2 <https://discourse.ros.org/t/rqt-in-ros2/6428>`__).
* `RQt for ROS 1 documentation <https://wiki.ros.org/rqt>`__.
* Brief overview of RQt (from `a Willow Garage intern blog post <http://web.archive.org/web/20130518142837/http://www.willowgarage.com/blog/2012/10/21/ros-gui>`__).

  .. raw:: html

     <iframe width="560" height="315" src="https://www.youtube-nocookie.com/embed/CyP9wHu2PpY" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
