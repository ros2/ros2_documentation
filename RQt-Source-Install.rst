Building RQt from Source
=========================

System Requirements
-------------------

These instructions are written for the target platforms for Crystal
Clemmeys (see `REP <http://www.ros.org/reps/rep-2000.html>`_).

- Ubuntu Bionic Beaver 18.04 64-bit
- Windows 10 with Visual Studio 2017
- Mac OSX Sierra 10.12.x

Other Requirements
~~~~~~~~~~~~~~~~~~

- In ROS2 Crystal the minimum Qt version is ``Qt5``

Building From Source
--------------------

In order to build RQt from source, first create a ROS2 workspace at ``~/ros2_ws/``.
This is step is already covered in `building ROS2 from source <https://index.ros.org/doc/ros2/Installation/>`_. instructions, so we skip it here.

Download RQt Repositories
~~~~~~~~~~~~~~~~~~~~~~~~~~

::

   cd ~/ros2_ws
   wget https://raw.githubusercontent.com/PickNikRobotics/rqt2_setup/master/rqt2.repos
   vcs import src --force < rqt2.repos

 **TODO:** The wget command should be replaced with an invocation of ``rosinstall_generator`` as soon as the packages have been released.


Install Dependencies
~~~~~~~~~~~~~~~~~~~~

::

   rosdep install --from-paths src --ignore-src --rosdistro bouncy -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 rti-connext-dds-5.3.1 urdfdom_headers"

Build The Workspace
~~~~~~~~~~~~~~~~~~~

::

   colcon build

Advanced Colcon usages:

-  Show verbose output:

   ::

     colcon build –event-handlers console_direct+

-  Only build one package and its dependencies:

   ::

     colcon build –packages-up-to rqt_shell

Source your environment
~~~~~~~~~~~~~~~~~~~~~~~

::

   . install/local_setup.bash


Using RQt
----------

See `Overview of RQt<RQt-Overview-Usage>`.
