**As of ROS 2 Bouncy the recommended build tool is ``colcon`` described in the `colcon tutorial <Colcon-Tutorial>`.**
The current default branch as well as releases after Bouncy do not include ``ament_tools`` anymore.

Overview
========

This will provide you with a quick summary of how to get up and running using an ament workspace.
It will be a practical tutorial and is not designed to replace the core documentation.

Background
----------

Ament is an iteration on the catkin meta-build tool.
For more information on the design of ament see `this document <http://design.ros2.org/articles/ament.html>`__.

The source can be found in the `ament GitHub organization <https://github.com/ament>`__.

Prerequisites
-------------

Development Environment
-----------------------

Make sure that you have setup your development environment according to the building-from-source `instruction <../Installation>`.

Basics
------

An ament workspace is a directory with a particular structure.
Commonly there is a ``src`` subdirectory.
Inside that subdirectory is where the source code will be located.
Typically the directory starts otherwise empty.

Ament does out of source builds.
By default it will create a ``build`` and ``install`` directory as peers of the ``src`` directory.
The ``build`` directory will be where intermediate files are stored.
For each package a subfolder will be created in which e.g. CMake is being invoked.
The ``install`` directory is where each package will be installed to.

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

To start off we need to setup an underlay without any of ROS2 installed.

Note: if you do not have ``vcs`` installed, instructions are `here <https://github.com/dirk-thomas/vcstool>`__.

.. code-block:: bash

   wget https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos
   vcs import src < ros2.repos

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

Since this is a bootstrap environment we need to call ament.py by its full path.

Note: In the future once ament is either installed on your system or in an underlayed workspace this will no longer be necessary.

Since there is no ``devel`` space in ament and it requires installing each package it supports the option ``--symlink-install``.
This allows the installed files to be changed by changing the files in the ``source`` space (e.g. Python files or other not compiled resourced) for faster iteration.

.. code-block:: bash

   src/ament/ament_tools/scripts/ament.py build --build-tests --symlink-install

Run the tests
-------------

To run the tests you just built, with the ``--build-tests`` option above, run the following:

.. code-block:: bash

   src/ament/ament_tools/scripts/ament.py test

If you have built (and installed) a workspace before including the tests (using ``build --build-tests``\ ) you can skip the build and install step to speed up the process:

.. code-block:: bash

   src/ament/ament_tools/scripts/ament.py test --skip-build --skip-install

Source the environment
----------------------

When ament has completed building successfully the output will be in the ``install`` directory.
To use the executables and libraries you need to e.g. add the ``install/bin`` directory to your path.
Ament will have generated bash files in the ``install`` directory to help setup the environment.
These files will both add the required elements to your path and library paths as well as provide any exported bash or shell commands exported by packages.

.. code-block:: bash

   . install/local_setup.bash

NB: This is slightly different than catkin.
The ``local_setup.*`` file is slightly different than the ``setup.*`` file in that it will only apply settings from the current workspace.
When using more than one workspace you will still source the ``setup.*`` files to get the environment including all parent workspaces.

Try a demo
----------

With the environment sourced you can now run executables built by ament.

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

Ament uses the same ``package.xml`` specification as defined for catkin in `REP 140 <http://www.ros.org/reps/rep-0140.html>`__.

You can create your own package inside the ``src`` directory however it is recommended to use an overlay when you are going to iterate only on a few packages.

Create an overlay
-----------------

Now that you have setup your bootstrap underlay you will also find ``ament`` is on your path.

Lets make a new overlay directory ``~/ros2_overlay_ws``.

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
   ament build --cmake-args -DCMAKE_BUILD_TYPE=Debug

Now this overlay is on top of the existing overlay so you'll find that ``which talker`` currently refers to the one from the underlay.

If you source ``~/ros2_overlay_ws/install/local_setup.bash`` it will change to refer to talker in the overlay.

If you are returning with a new terminal to your development and want to pick up developing on your overlay you can simply source ``~/ros2_overlay_ws/install/setup.bash`` which will source all parent workspaces environments automatically.

Create your own package
-----------------------

You can create your own package.
The equivalent of ``catkin_create_package`` will be ported to ament but is not available yet.

Ament supports multiple build types.
The recommended build types are ``ament_cmake`` and ``ament_python``.
Also supported are pure ``cmake`` packages.
It's expected to add support for more `build types <https://github.com/ament/ament_tools/blob/master/doc/development/build_types.rst>`__.

An example of an ``ament_python`` build is the `ament_tools package <https://github.com/ament/ament_tools>`__, where the setup.py is the primary entry point for building.

A package such as `demo_nodes_cpp <https://github.com/ros2/demos/tree/master/demo_nodes_cpp>`__ uses the ``ament_cmake`` build type, and uses CMake as the build tool.

Tips
----


* If you do not want to build a specific package place an empty file named ``AMENT_IGNORE`` in the directory and it will not be indexed.

    "Catch all" options like --cmake-args should be placed after other options, or delimited with '--':

.. code-block:: bash

   ament build . --force-cmake-configure --cmake-args -DCMAKE_BUILD_TYPE=Debug -- --ament-cmake-args -DCMAKE_BUILD_TYPE=Release


* If you want to run a single particular test from a package:
  .. code-block:: bash

     ament test --only-packages YOUR_PKG_NAME --ctest-args -R YOUR_TEST_IN_PKG
