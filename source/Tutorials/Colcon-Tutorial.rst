.. _Colcon:

.. redirect-from::

    Colcon-Tutorial

Using colcon to build packages
==============================

.. contents:: Table of Contents
   :depth: 2
   :local:

This is a brief tutorial of how to create and build a ROS 2 workspace with ``colcon``.
It is a practical tutorial and not designed to replace the core documentation.

Background
----------

``colcon`` is an iteration on the ROS build tools ``catkin_make``, ``catkin_make_isolated``, ``catkin_tools`` and ``ament_tools``.
For more information on the design of colcon see `this document <https://design.ros2.org/articles/build_tool.html>`__.

The source code can be found in the `colcon GitHub organization <https://github.com/colcon>`__.

Prerequisites
-------------

Install colcon
^^^^^^^^^^^^^^

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

        sudo apt install python3-colcon-common-extensions

  .. group-tab:: macOS

    .. code-block:: bash

        python3 -m pip install colcon-common-extensions

  .. group-tab:: Windows

    .. code-block:: bash

        pip install -U colcon-common-extensions


Install ROS 2
^^^^^^^^^^^^^

To build the samples, you will need to install ROS 2.

Follow the `installation instructions <../Installation>`.

.. attention:: If installing from Debian packages, this tutorial requires the `desktop installation <linux-install-debians-install-ros-2-packages>`.

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

.. note:: Compared to catkin there is no ``devel`` directory.

Create a workspace
^^^^^^^^^^^^^^^^^^

First, create a directory (``ros2_example_ws``) to contain our workspace:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

       mkdir -p ~/ros2_example_ws/src
       cd ~/ros2_example_ws

  .. group-tab:: macOS

    .. code-block:: bash

       mkdir -p ~/ros2_example_ws/src
       cd ~/ros2_example_ws

  .. group-tab:: Windows

    .. code-block:: bash

       md \dev\ros2_example_ws\src
       cd \dev\ros2_example_ws

At this point the workspace contains a single empty directory ``src``:

.. code-block:: bash

    .
    └── src

    1 directory, 0 files

Add some sources
^^^^^^^^^^^^^^^^

Let's clone the `examples <https://github.com/ros2/examples>`__ repository into the ``src`` directory of the workspace:

.. code-block:: bash

    git clone https://github.com/ros2/examples src/examples

.. attention:: It is recommended to checkout a branch that is compatible with the version of ROS that was installed (e.g. ``crystal``).


.. code-block:: bash

    cd ~/ros2_example_ws/src/examples/
    git checkout $ROS_DISTRO
    cd ~/ros2_example_ws

Now the workspace should have the source code to the ROS 2 examples:

.. code-block:: bash

    .
    └── src
        └── examples
            ├── CONTRIBUTING.md
            ├── LICENSE
            ├── rclcpp
            ├── rclpy
            └── README.md

    4 directories, 3 files

Source an underlay
^^^^^^^^^^^^^^^^^^

It is important that we have sourced the environment for an existing ROS 2 installation that will provide our workspace with the necessary build dependencies for the example packages.
This is achieved by sourcing the setup script provided by a binary installation or a source installation, ie. another colcon workspace (see `Installation <../Installation>`).
We call this environment an **underlay**.

Our workspace, ``ros2_examples_ws``, will be an **overlay** on top of the existing ROS 2 installation.
In general, it is recommended to use an overlay when you plan to iterate on a small number of packages, rather than putting all of your packages into the same workspace.

Build the workspace
^^^^^^^^^^^^^^^^^^^

.. attention::

   To build packages on Windows you need to be in a Visual Studio environment, see `Building the ROS 2 Code <windows-dev-build-ros2>` for more details.

In the root of the workspace, run ``colcon build``.
Since build types such as ``ament_cmake`` do not support the concept of the ``devel`` space and require the package to be installed, colcon supports the option ``--symlink-install``.
This allows the installed files to be changed by changing the files in the ``source`` space (e.g. Python files or other not compiled resourced) for faster iteration.

.. code-block:: bash

    colcon build --symlink-install

After the build is finished, we should see the ``build``, ``install``, and ``log`` directories:

.. code-block:: bash

    .
    ├── build
    ├── install
    ├── log
    └── src

    4 directories, 0 files

.. _colcon-run-the-tests:

Run tests
^^^^^^^^^

To run tests for the packages we just built, run the following:

.. code-block:: bash

   colcon test

Source the environment
^^^^^^^^^^^^^^^^^^^^^^

When colcon has completed building successfully, the output will be in the ``install`` directory.
Before you can use any of the installed executables or libraries, you will need to add them to your path and library paths.
colcon will have generated bash/bat files in the ``install`` directory to help setup the environment.
These files will add all of the required elements to your path and library paths as well as provide any bash or shell commands exported by packages.

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

       . install/setup.bash

  .. group-tab:: macOS

    .. code-block:: bash

       . install/setup.bash

  .. group-tab:: Windows

    .. code-block:: bash

       call install\setup.bat

Try a demo
^^^^^^^^^^

With the environment sourced we can run executables built by colcon.
Let's run a subscriber node from the examples:

.. code-block:: bash

    ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function

In another terminal, let's run a publisher node (don't forget to source the setup script):

.. code-block:: bash

    ros2 run examples_rclcpp_minimal_publisher publisher_member_function

You should see messages from the publisher and subscriber with numbers incrementing.

Create your own package
-----------------------

colcon uses the ``package.xml`` specification defined in `REP 149 <https://www.ros.org/reps/rep-0149.html>`__ (`format 2 <https://www.ros.org/reps/rep-0140.html>`__ is also supported).

colcon supports multiple build types.
The recommended build types are ``ament_cmake`` and ``ament_python``.
Also supported are pure ``cmake`` packages.

An example of an ``ament_python`` build is the `ament_index_python package <https://github.com/ament/ament_index/tree/master/ament_index_python>`__ , where the setup.py is the primary entry point for building.

A package such as `demo_nodes_cpp <https://github.com/ros2/demos/tree/master/demo_nodes_cpp>`__ uses the ``ament_cmake`` build type, and uses CMake as the build tool.

For convenience, you can use the tool ``ros2 pkg create`` to create a new package based on a template.

.. note:: For ``catkin`` users, this is the equivalent of ``catkin_create_package``.

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
