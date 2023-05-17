Installation (Ubuntu)
======================================

**Goal:** Install the ``webots_ros2`` package and run simulation examples on Ubuntu.

**Tutorial level:** Advanced

**Time:** 10 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

The ``webots_ros2`` package provides an interface between ROS 2 and Webots.
It includes several sub-packages, including ``webots_ros2_driver``, which allows you to start Webots and communicate with it.
This interface is used in most of the following tutorials, so it is required to install it beforehand.
Other sub-packages are mainly examples that show multiple possible implementations using the interface.
In this tutorial, you are going to install the package and learn how to run one of these examples.

Prerequisites
-------------

It is recommended to understand basic ROS principles covered in the beginner :doc:`../../../../Tutorials`.
In particular, :doc:`../../../Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace` and :doc:`../../../Beginner-Client-Libraries/Creating-Your-First-ROS2-Package` are useful prerequisites.

The Webots software should be installed in order to use the ``webots_ros2`` interface.
You can follow the `installation procedure <https://cyberbotics.com/doc/guide/installation-procedure>`_ or `build it from sources <https://github.com/cyberbotics/webots/wiki/Linux-installation/>`_.

Alternatively, you can also let ``webots_ros2`` download and install Webots automatically.
This option appears when you launch an example of the package and no Webots installation is found.

Multiple Installations of Webots
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you have installed different versions of Webots on your computer, ``webots_ros2`` will look for Webots at the following locations (in this order):

1. If the ``ROS2_WEBOTS_HOME`` environment variable is set, ROS 2 will use the Webots in this folder, regardless of its version.
2. If the ``WEBOTS_HOME`` environment variable is set, ROS 2 will use the Webots in this folder, regardless of its version.
3. If none of these variables is set, ``webots_ros2`` will look for Webots in the default installation paths for a compatible version: ``/usr/local/webots`` and ``/snap/webots/current/usr/share/webots``.
4. If Webots couldn't be found, ``webots_ros2`` will show a window offering the automatic installation of the latest compatible version of Webots.

Tasks
-----

1 Install ``webots_ros2``
^^^^^^^^^^^^^^^^^^^^^^^^^
You can either install the official released package, or install it from the latest up-to-date sources from `Github <https://github.com/cyberbotics/webots_ros2>`_.

.. tabs::

    .. group-tab:: Install ``webots_ros2`` distributed package

        Run the following command in a terminal.

        .. code-block:: console

            sudo apt-get install ros-{DISTRO}-webots-ros2

    .. group-tab:: Install ``webots_ros2`` from sources

        Create a ROS 2 workspace with its ``src`` directory.

        .. code-block:: console

            mkdir -p ~/ros2_ws/src

        Source the ROS 2 environment.

        .. code-block:: console

            source /opt/ros/{DISTRO}/setup.bash

        Retrieve the sources from Github.

        .. code-block:: console

            cd ~/ros2_ws
            git clone --recurse-submodules https://github.com/cyberbotics/webots_ros2.git src/webots_ros2

        Install the package dependencies.

        .. code-block:: console

            sudo apt install python3-pip python3-rosdep python3-colcon-common-extensions
            sudo rosdep init && rosdep update
            rosdep install --from-paths src --ignore-src --rosdistro {DISTRO}

        Build the package using ``colcon``.

        .. code-block:: console

            colcon build

        Source this workspace.

        .. code-block:: console

            source install/local_setup.bash

2 Launch the ``webots_ros2_universal_robot`` example
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The following instructions explain how to start a provided example.

First source the ROS 2 environment, if not done already.

.. code-block:: console

        source /opt/ros/{DISTRO}/setup.bash

Setting the ``WEBOTS_HOME`` environment variable allows you to start a specific Webots installation.

.. code-block:: console

        export WEBOTS_HOME=/usr/local/webots

If installed from sources, source your ROS 2 workspace, if not done already.

.. code-block:: console

        cd ~/ros2_ws
        source install/local_setup.bash

Use the ROS 2 launch command to start demo packages (e.g. ``webots_ros2_universal_robot``).

.. code-block:: console

        ros2 launch webots_ros2_universal_robot multirobot_launch.py
