.. _linux-latest:

Building ROS 2 on Linux
=======================

.. contents:: Table of Contents
   :depth: 2
   :local:


System requirements
-------------------
The current target platforms for Rolling Ridley are

- Tier 1: Ubuntu Linux - Focal Fossa (20.04) 64-bit

Tier 3 platforms (not actively tested or supported) include:

- Debian Linux - Buster (10)
- Fedora 32, see `alternate instructions <Fedora-Development-Setup>`
- Arch Linux, see `alternate instructions <https://wiki.archlinux.org/index.php/ROS#ROS_2>`__
- OpenEmbedded / webOS OSE, see `alternate instructions <https://github.com/ros/meta-ros/wiki/OpenEmbedded-Build-Instructions>`__

As defined in `REP 2000 <https://www.ros.org/reps/rep-2000.html>`_

System setup
------------

Set locale
^^^^^^^^^^

.. include:: _Linux-Set-Locale.rst

Add the ROS 2 apt repository
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. include:: _Apt-Repositories.rst

Install development tools and ROS tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   sudo apt update && sudo apt install -y \
     build-essential \
     cmake \
     git \
     libbullet-dev \
     python3-colcon-common-extensions \
     python3-flake8 \
     python3-pip \
     python3-pytest-cov \
     python3-rosdep \
     python3-setuptools \
     python3-vcstool \
     wget
   # install some pip packages needed for testing
   python3 -m pip install -U \
     argcomplete \
     flake8-blind-except \
     flake8-builtins \
     flake8-class-newline \
     flake8-comprehensions \
     flake8-deprecated \
     flake8-docstrings \
     flake8-import-order \
     flake8-quotes \
     pytest-repeat \
     pytest-rerunfailures \
     pytest \
     setuptools
   # install Fast-RTPS dependencies
   sudo apt install --no-install-recommends -y \
     libasio-dev \
     libtinyxml2-dev
   # install Cyclone DDS dependencies
   sudo apt install --no-install-recommends -y \
     libcunit1-dev

Ubuntu 18.04 is not an officially supported platform, but may still work.  You'll need at least the following additional dependencies:

.. code-block:: bash

   python3 -m pip install -U importlib-metadata importlib-resources

.. _Rolling_linux-dev-get-ros2-code:

Get ROS 2 code
--------------

Create a workspace and clone all repos:

.. code-block:: bash

   mkdir -p ~/ros2_rolling/src
   cd ~/ros2_rolling
   wget https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos
   vcs import src < ros2.repos

.. _linux-development-setup-install-dependencies-using-rosdep:

Install dependencies using rosdep
---------------------------------

.. code-block:: bash

   sudo rosdep init
   rosdep update
   rosdep install --from-paths src --ignore-src --rosdistro rolling -y --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers"

Install additional DDS implementations (optional)
-------------------------------------------------

If you would like to use another DDS or RTPS vendor besides the default, eProsima's Fast RTPS, you can find instructions `here <DDS-Implementations>`.

Build the code in the workspace
-------------------------------
More info on working with a ROS workspace can be found in `this tutorial </Tutorials/Colcon-Tutorial>`.

.. code-block:: bash

   cd ~/ros2_rolling/
   colcon build --symlink-install

Note: if you are having trouble compiling all examples and this is preventing you from completing a successful build, you can use ``AMENT_IGNORE`` in the same manner as `CATKIN_IGNORE <https://github.com/ros-infrastructure/rep/blob/master/rep-0128.rst>`__ to ignore the subtree or remove the folder from the workspace.
Take for instance: you would like to avoid installing the large OpenCV library.
Well then simply ``$ touch AMENT_IGNORE`` in the ``cam2image`` demo directory to leave it out of the build process.

Optionally install all packages into a combined directory (rather than each package in a separate subdirectory).
On Windows due to limitations of the length of environment variables you should use this option when building workspaces with many (~ >> 100 packages).

Also, if you have already installed ROS 2 from Debian make sure that you run the ``build`` command in a fresh environment.
You may want to make sure that you do not have ``source /opt/ros/${ROS_DISTRO}/setup.bash`` in your ``.bashrc``.
You can make sure that ROS 2 is not sourced with the command ``printenv | grep -i ROS``.
The output should be empty.

.. code-block:: bash

   colcon build --symlink-install --merge-install

Afterwards source the ``local_setup.*`` from the ``install`` folder.

Environment setup
-----------------

Source the setup script
^^^^^^^^^^^^^^^^^^^^^^^

Set up your environment by sourcing the following file.

.. code-block:: bash

   . ~/ros2_rolling/install/setup.bash

.. _talker-listener:

Try some examples
-----------------

In one terminal, source the setup file and then run a C++ ``talker``\ :

.. code-block:: bash

   . ~/ros2_rolling/install/local_setup.bash
   ros2 run demo_nodes_cpp talker

In another terminal source the setup file and then run a Python ``listener``\ :

.. code-block:: bash

   . ~/ros2_rolling/install/local_setup.bash
   ros2 run demo_nodes_py listener

You should see the ``talker`` saying that it's ``Publishing`` messages and the ``listener`` saying ``I heard`` those messages.
This verifies both the C++ and Python APIs are working properly.
Hooray!

Next steps after installing
---------------------------
Continue with the `tutorials and demos </Tutorials>` to configure your environment, create your own workspace and packages, and learn ROS 2 core concepts.

Using the ROS 1 bridge
----------------------
The ROS 1 bridge can connect topics from ROS 1 to ROS 2 and vice-versa. See the dedicated `documentation <https://github.com/ros2/ros1_bridge/blob/master/README.md>`__ on how to build and use the ROS 1 bridge.

Additional RMW implementations (optional)
-----------------------------------------
The default middleware that ROS 2 uses is ``Fast-RTPS``, but the middleware (RMW) can be replaced at runtime.
See the `tutorial </Tutorials/Working-with-multiple-RMW-implementations>` on how to work with multiple RMWs.

Alternate compilers
-------------------

Using a different compiler besides gcc to compile ROS 2 is easy. If you set the environment variables ``CC`` and ``CXX`` to executables for a working C and C++ compiler, respectively, and retrigger CMake configuration (by using ``--force-cmake-config`` or by deleting the packages you want to be affected), CMake will reconfigure and use the different compiler.

Clang
^^^^^

To configure CMake to detect and use Clang:

.. code-block:: bash

   sudo apt install clang
   export CC=clang
   export CXX=clang++
   colcon build --cmake-force-configure

TODO: using ThreadSanitizer, MemorySanitizer

Stay up to date
---------------

See :ref:`MaintainingSource` to periodically refresh your source installation.

Troubleshooting
---------------

Troubleshooting techniques can be found :ref:`here <linux-troubleshooting>`.

Uninstall
---------

1. If you installed your workspace with colcon as instructed above, "uninstalling" could be just a matter of opening a new terminal and not sourcing the workspace's ``setup`` file.
   This way, your environment will behave as though there is no Rolling install on your system.

2. If you're also trying to free up space, you can delete the entire workspace directory with:

   .. code-block:: bash

    rm -rf ~/ros2_rolling
