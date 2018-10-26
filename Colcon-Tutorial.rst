
Overview
========

This will provide you with a quick summary of how to get up and running using ``colcon`` and a ROS workspace.
It will be a practical tutorial and is not designed to replace the core documentation.

ROS 2 releases before Bouncy were using ``ament_tools`` described in the `ament tutorial <Ament-Tutorial>`.

Background
----------

colcon is an iteration on the ROS build tools catkin_make, catkin_make_isolated, catkin_tools and ament_tools.
For more information on the design of colcon see `this document <http://design.ros2.org/articles/build_tool.html>`__.

The source code can be found in the `colcon GitHub organization <https://github.com/colcon>`__.

Prerequisites
-------------

Development Environment
-----------------------

Make sure that you have setup your development environment according to the building-from-source `instructions <Installation>`.

Basics
------

A ROS workspace is a directory with a particular structure.
Commonly there is a ``src`` subdirectory.
Inside that subdirectory is where the source code of ROS packages will be located.
Typically the directory starts otherwise empty.

colcon does out of source builds.
By default it will create the following directories as peers of the ``src`` directory:


* The ``build`` directory will be where intermediate files are stored.
  For each package a subfolder will be created in which e.g. CMake is being invoked.
* The ``install`` directory is where each package will be installed to.
  By default each package will be installed into a separate subdirectory.
* The ``log`` directory contains various logging information about each colcon invocation.

NB: Compared to catkin there is no ``devel`` directory.

Create directory structure
--------------------------

To make the basic structure in the directory ``~/ros2_ws``\ :

.. code-block:: bash

   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws

This is the directory structure of ``~/ros2_ws`` that you can expect at this point:

.. code-block:: bash

   .
   └── src

   1 directory, 0 files

Add some sources
----------------

To start off we need to setup an underlay workspace without any of ROS 2 installed.

.. code-block:: bash

   wget https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos
   vcs import ~/ros2_ws/src < ros2.repos

This is the directory structure of ``~/ros2_ws`` that you can expect after adding sources (note the exact structure and number of directories/files may change over time):

.. code-block:: bash

   .
   ├── ros2.repos
   └── src
       ├── ament
       │   ├── ament_cmake
       │   ├── ament_index
       |   ...
       │   ├── osrf_pycommon
       │   └── uncrustify
       ├── eProsima
       │   ├── Fast-CDR
       │   └── Fast-RTPS
       ├── ros
       │   ├── class_loader
       │   └── console_bridge
       └── ros2
           ├── ament_cmake_ros
           ├── common_interfaces
           ├── demos
           ...
           ├── urdfdom
           ├── urdfdom_headers
           └── vision_opencv

   51 directories, 1 file

Run the build
-------------

Since build types such as ``ament_cmake`` do not support the concept of the ``devel`` space and require the package to be installed, colcon supports the option ``--symlink-install``.
This allows the installed files to be changed by changing the files in the ``source`` space (e.g. Python files or other not compiled resourced) for faster iteration.

.. code-block:: bash

   colcon build --symlink-install

.. _colcon-run-the-tests:
   
Run the tests
-------------

To run the tests you just built, run the following:

.. code-block:: bash

   colcon test

Source the environment
----------------------

When colcon has completed building successfully the output will be in the ``install`` directory.
To use the executables and libraries you need to e.g. add the ``install/bin`` directory to your path.
colcon will have generated bash/bat files in the ``install`` directory to help setup the environment.
These files will both add the required elements to your path and library paths as well as provide any exported bash or shell commands exported by packages.

.. code-block:: bash

   . install/local_setup.bash

NB: This is slightly different than catkin.
The ``local_setup.*`` file is slightly different than the ``setup.*`` file in that it will only apply settings from the current workspace.
When using more than one workspace you will still source the ``setup.*`` files to get the environment including all parent workspaces.

Try a demo
----------

With the environment sourced you can now run executables built by colcon.

.. code-block:: bash

   ros2 run demo_nodes_cpp listener &
   ros2 run demo_nodes_cpp talker

And you will see the numbers incrementing.

Lets take down the nodes and try creating our own workspace overlay.

.. code-block:: bash

   ^-C
   kill %1

Develop your own package
------------------------

colcon uses the same ``package.xml`` specification as defined for catkin in `REP 149 <http://www.ros.org/reps/rep-0149.html>`__.

You can create your own package inside the ``src`` directory however it is recommended to use an overlay when you are going to iterate only on a few packages.

Create an overlay
-----------------

Let's make a new overlay directory ``~/ros2_overlay_ws``.

.. code-block:: bash

   mkdir -p ~/ros2_overlay_ws/src
   cd ~/ros2_overlay_ws/src

And to get started we'll overlay the `ros2/examples repository <https://github.com/ros2/examples>`__\ :

.. code-block:: bash

   # If you know that you're using the latest branch of all
   # repositories in the underlay, you can also get the latest
   # version of the ros2/examples repository, with this command:
   #   git clone https://github.com/ros2/examples.git
   # Otherwise, clone a copy from the underlay source code:
   git clone ~/ros2_ws/src/ros2/examples

And build the overlay, but let's build with debug so we can make sure to get debug symbols:

.. code-block:: bash

   cd ~/ros2_overlay_ws
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

This overlay has not yet been setup to be on top of the existing underlay so you'll still find that ``which talker`` currently refers to the one from the underlay.

If you source ``~/ros2_overlay_ws/install/local_setup.bash`` it will change to refer to talker in the overlay.

If you are returning with a new terminal to your development and want to pick up developing on your overlay you can simply source ``~/ros2_overlay_ws/install/setup.bash`` which will source all parent workspaces environments automatically.

Create your own package
-----------------------

You can create your own package.
The equivalent of ``catkin_create_package`` is available as ``ros2 pkg create``.

colcon supports multiple build types.
The recommended build types are ``ament_cmake`` and ``ament_python``.
Also supported are pure ``cmake`` packages.

An example of an ``ament_python`` build is the `ament_index_python package <https://github.com/ament/ament_index/tree/master/ament_index_python>`__ , where the setup.py is the primary entry point for building.

A package such as `demo_nodes_cpp <https://github.com/ros2/demos/tree/master/demo_nodes_cpp>`__ uses the ``ament_cmake`` build type, and uses CMake as the build tool.

Tips
----


* 
  If you do not want to build a specific package place an empty file named ``COLCON_IGNORE`` in the directory and it will not be indexed.

* 
  If you want to avoid configuring and building tests in CMake packages you can pass: ``--cmake-args -DBUILD_TESTING=0``.

* 
  If you want to run a single particular test from a package:

  .. code-block:: bash

     colcon test --packages-select YOUR_PKG_NAME --ctest-args -R YOUR_TEST_IN_PKG
