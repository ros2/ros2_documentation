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

0 Set up a virtual environment (optional)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you're using Ubuntu, you most likely will want to install Webots and its associated ROS packages natively.
However, you can set up an alternative configuration where the ROS packages run in a Docker container and Webots runs on the host.
This type of setup is similar to macOS and Windows hosts.

* Install Docker Engine on your Ubuntu machine.
  The instructions can be found on the `Docker website <https://docs.docker.com/engine/install/ubuntu/>`_.

* Pull a Docker image for ROS. This tutorial uses the latest `ROS Base <https://hub.docker.com/layers/library/ros/latest/images/sha256-52e27b46c352d7ee113f60b05590bb089628a17ef648fff6992ca363c5e14945?context=explore>`_ image.

The following instructions and commands all are run on the host machine.

* Create a folder to use as a shared folder.
  In this example, the shared folder on Ubuntu is ``/home/username/shared``, where ``username`` is your actual username.

  .. code-block:: console

    mkdir /home/username/shared

* Start a new ROS 2 container with a bind mount for the shared directory you created in the previous step.
  The shared directory will be located at ``/root/shared`` inside the container.

  .. code-block:: console

    docker run -it --mount type=bind,src=/home/username/shared,target=/root/shared --add-host host.docker.internal:host-gateway ros:latest

  .. note::
    Unlike Docker Desktop for Mac and Docker Desktop for Windows, you will need to specify your machine as an extra host.
    This allows the container to resolve Docker's special ``host.docker.internal`` DNS name, allowing the container to communicate with the host.
    See `this <https://stackoverflow.com/questions/24319662/from-inside-of-a-docker-container-how-do-i-connect-to-the-localhost-of-the-mach>`_ StackOverflow post for more information about why this is needed.


* The environment variable ``WEBOTS_SHARED_FOLDER`` must always be set in order for the package to work properly in the Docker container.
  This variable specifies the location of the shared folder that is used to exchange data between the host machine and the container to the ``webots_ros2`` package.
  The value to use for this variable should be in the format of ``<host shared folder>:<container shared folder>``, where ``<host shared folder>`` is the path to the shared folder on the host machine and ``<container shared folder>`` is the path to the same shared folder on the Docker container.

  In this example:

  .. code-block:: console

    export WEBOTS_SHARED_FOLDER=/home/username/shared:/root/shared

If you decide to use a Docker container, you will run steps 1 and 2 in the container unless otherwise specified.
You won't need to use ``sudo`` when you're running the commands because the container's user is ``root``.

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

.. note::
  If you are using a Docker container, you will need to run a local TCP simulation server on the host so the ``webots_ros2`` packages can communicate with the Webots simulator.
  The server can be downloaded here: `local_simulation_server.py <https://github.com/cyberbotics/webots-server/blob/main/local_simulation_server.py>`_.
  Specify (on the host) the Webots installation folder in ``WEBOTS_HOME`` environment variable and run the server using the following commands in a new terminal on the host (not in the container):

  .. code-block:: console

    export WEBOTS_HOME=<path to webots binary>
    python3 local_simulation_server.py

  Be sure to also set ``WEBOTS_SHARED_FOLDER`` on the host.

  .. code-block:: console

    export WEBOTS_SHARED_FOLDER=/home/username/shared:/root/shared

The following instructions explain how to start a provided example.
If you're using a Docker container, run these commands in the container.

First source the ROS 2 environment, if not done already.

.. code-block:: console

        source /opt/ros/{DISTRO}/setup.bash

Setting the ``WEBOTS_HOME`` environment variable allows you to start a specific Webots installation.
Skip this step if you're using Docker container.

.. code-block:: console

        export WEBOTS_HOME=/usr/local/webots

If installed from sources, source your ROS 2 workspace, if not done already.

.. code-block:: console

        cd ~/ros2_ws
        source install/local_setup.bash

If you are using a Docker container, be sure to set the ``WEBOTS_SHARED_FOLDER`` environment variable on the container to the same value you set it to on the host.

Use the ROS 2 launch command to start demo packages (e.g. ``webots_ros2_universal_robot``).

.. code-block:: console

        ros2 launch webots_ros2_universal_robot multirobot_launch.py
