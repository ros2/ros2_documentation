.. redirect-from::

  Installation/RHEL-Development-Setup

.. _rhel-latest:

RHEL (source)
=============

.. contents:: Table of Contents
   :depth: 2
   :local:


System requirements
-------------------
The current target Red Hat platforms for {DISTRO_TITLE_FULL} are:

- Tier 2: RHEL 8 64-bit

As defined in `REP 2000 <https://www.ros.org/reps/rep-2000.html>`_

System setup
------------

Set locale
^^^^^^^^^^

.. include:: ../_RHEL-Set-Locale.rst

Enable required repositories
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The rosdep database contains packages from the EPEL and PowerTools repositories, which are not enabled by default.
They can be enabled by running:

.. code-block:: bash

   sudo dnf install 'dnf-command(config-manager)' epel-release -y
   sudo dnf config-manager --set-enabled powertools

.. note:: This step may be slightly different depending on the distribution you are using. Check the EPEL documentation: https://docs.fedoraproject.org/en-US/epel/#_quickstart


Install development tools and ROS tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   sudo dnf install -y \
     cmake \
     gcc-c++ \
     git \
     make \
     patch \
     python3-colcon-common-extensions \
     python3-pip \
     python3-pydocstyle \
     python3-pytest \
     python3-pytest-repeat \
     python3-pytest-rerunfailures \
     python3-rosdep \
     python3-setuptools \
     python3-vcstool

   # install some pip packages needed for testing and
   # not available as RPMs
   python3 -m pip install -U --user \
     flake8-blind-except==0.1.1 \
     flake8-builtins \
     flake8-class-newline \
     flake8-comprehensions \
     flake8-deprecated \
     flake8-docstrings \
     flake8-import-order \
     flake8-quotes \
     mypy==0.931

.. _rhel-dev-get-ros2-code:

Get ROS 2 code
--------------

Create a workspace and clone all repos:

.. code-block:: bash

   mkdir -p ~/ros2_{DISTRO}/src
   cd ~/ros2_{DISTRO}
   vcs import --input https://raw.githubusercontent.com/ros2/ros2/{REPOS_FILE_BRANCH}/ros2.repos src

.. _rhel-development-setup-install-dependencies-using-rosdep:

Install dependencies using rosdep
---------------------------------

.. include:: ../_Dnf-Update-Admonition.rst

.. code-block:: bash

   sudo rosdep init
   rosdep update
   rosdep install --from-paths src --ignore-src -y --skip-keys "asio cyclonedds fastcdr fastrtps ignition-cmake2 ignition-math6 python3-babeltrace python3-mypy rti-connext-dds-6.0.1 urdfdom_headers"

Install additional DDS implementations (optional)
-------------------------------------------------

If you would like to use another DDS or RTPS vendor besides the default, you can find instructions :doc:`here <../DDS-Implementations>`.

Build the code in the workspace
-------------------------------

If you have already installed ROS 2 another way (either via RPMs or the binary distribution), make sure that you run the below commands in a fresh environment that does not have those other installations sourced.
Also ensure that you do not have ``source /opt/ros/${ROS_DISTRO}/setup.bash`` in your ``.bashrc``.
You can make sure that ROS 2 is not sourced with the command ``printenv | grep -i ROS``.
The output should be empty.

More info on working with a ROS workspace can be found in :doc:`this tutorial <../../Tutorials/Beginner-Client-Libraries/Colcon-Tutorial>`.

.. code-block:: bash

   cd ~/ros2_{DISTRO}/
   colcon build --symlink-install --cmake-args -DTHIRDPARTY_Asio=ON --no-warn-unused-cli

Note: if you are having trouble compiling all examples and this is preventing you from completing a successful build, you can use ``COLCON_IGNORE`` in the same manner as `CATKIN_IGNORE <https://github.com/ros-infrastructure/rep/blob/master/rep-0128.rst>`__ to ignore the subtree or remove the folder from the workspace.
Take for instance: you would like to avoid installing the large OpenCV library.
Well then simply run ``touch COLCON_IGNORE`` in the ``cam2image`` demo directory to leave it out of the build process.

Environment setup
-----------------

Source the setup script
^^^^^^^^^^^^^^^^^^^^^^^

Set up your environment by sourcing the following file.

.. code-block:: bash

   # Replace ".bash" with your shell if you're not using bash
   # Possible values are: setup.bash, setup.sh, setup.zsh
   . ~/ros2_{DISTRO}/install/local_setup.bash

.. _rhel_talker-listener:

Try some examples
-----------------

In one terminal, source the setup file and then run a C++ ``talker``\ :

.. code-block:: bash

   . ~/ros2_{DISTRO}/install/local_setup.bash
   ros2 run demo_nodes_cpp talker

In another terminal source the setup file and then run a Python ``listener``\ :

.. code-block:: bash

   . ~/ros2_{DISTRO}/install/local_setup.bash
   ros2 run demo_nodes_py listener

You should see the ``talker`` saying that it's ``Publishing`` messages and the ``listener`` saying ``I heard`` those messages.
This verifies both the C++ and Python APIs are working properly.
Hooray!

Next steps after installing
---------------------------
Continue with the :doc:`tutorials and demos <../../Tutorials>` to configure your environment, create your own workspace and packages, and learn ROS 2 core concepts.

Additional RMW implementations (optional)
-----------------------------------------
The default middleware that ROS 2 uses is ``Fast DDS``, but the middleware (RMW) can be replaced at runtime.
See the :doc:`guide <../../How-To-Guides/Working-with-multiple-RMW-implementations>` on how to work with multiple RMWs.

Alternate compilers
-------------------

Using a different compiler besides gcc to compile ROS 2 is easy. If you set the environment variables ``CC`` and ``CXX`` to executables for a working C and C++ compiler, respectively, and retrigger CMake configuration (by using ``--force-cmake-config`` or by deleting the packages you want to be affected), CMake will reconfigure and use the different compiler.

Clang
^^^^^

To configure CMake to detect and use Clang:

.. code-block:: bash

   sudo dnf install clang
   export CC=clang
   export CXX=clang++
   colcon build --cmake-force-configure

Stay up to date
---------------

See :doc:`../Maintaining-a-Source-Checkout` to periodically refresh your source installation.

Troubleshooting
---------------

Troubleshooting techniques can be found :ref:`here <linux-troubleshooting>`.

Uninstall
---------

1. If you installed your workspace with colcon as instructed above, "uninstalling" could be just a matter of opening a new terminal and not sourcing the workspace's ``setup`` file.
   This way, your environment will behave as though there is no {DISTRO_TITLE} install on your system.

2. If you're also trying to free up space, you can delete the entire workspace directory with:

   .. code-block:: bash

    rm -rf ~/ros2_{DISTRO}
