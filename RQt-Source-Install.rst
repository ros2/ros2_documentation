Building RQt from Source
=========================

System Requirements
-------------------

These instructions are written for the target platforms for Crystal
Clemmeys (see `REP <http://www.ros.org/reps/rep-2000.html>`_).

- Ubuntu Bionic Beaver 18.04 64-bit
- Mac OSX Sierra 10.12.x
- Windows 10 with Visual Studio 2017

Other Requirements
~~~~~~~~~~~~~~~~~~

- In ROS 2 Crystal the minimum Qt version is ``Qt5``

Building From Source
--------------------

In order to build RQt from source, first create a ROS2 workspace at ``~/ros2_ws/``.
This is step is already covered in `building ROS 2 from source <https://index.ros.org/doc/ros2/Installation/>`_. instructions, so we skip it here.

Download RQt Repositories
~~~~~~~~~~~~~~~~~~~~~~~~~

::

   cd ~/ros2_ws
   wget https://raw.githubusercontent.com/PickNikRobotics/rqt2_setup/master/rqt2.repos
   vcs import src --force < rqt2.repos

As an alternative to the hosted ``.repos`` file you can use ``rosinstall_generator`` to generate a custom one:

::

   rosinstall_generator --rosdistro crystal --upstream-development --repos python_qt_binding qt_gui_core rqt <more-repos-with-rqt-plugins> > rqt2.repos

Install Dependencies
~~~~~~~~~~~~~~~~~~~~
For non-Linux platforms, see the `macOS RQt source install page <RQt-Source-Install-MacOS>`_ or the `Windows 10 RQt source install page <RQt-Source-Install-Windows10>`_ before continuing here.

::

   rosdep install --from-paths src --ignore-src --rosdistro bouncy -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 rti-connext-dds-5.3.1 urdfdom_headers"

Build The Workspace
~~~~~~~~~~~~~~~~~~~
Generally building a workspace is as simple as:
::

   colcon build

For Windows, it is recommended to use the ``--merge-install`` option.
::

   colcon build --merge-install

Advanced Colcon usages:

-  Show verbose output on the console:

   ::

     colcon build –event-handlers console_direct+

-  Only build one package and its dependencies:

   ::

     colcon build –packages-up-to rqt_shell

Source your environment
~~~~~~~~~~~~~~~~~~~~~~~

Linux or macOS
::

   . install/local_setup.bash

Windows
::

   call install/local_setup.bat


Using RQt
----------

See `Overview of RQt <RQt-Overview-Usage>`_.
