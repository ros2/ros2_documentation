Installation (macOS)
====================

**Goal:** Install the ``webots_ros2`` package and run simulation examples on macOS.

**Tutorial level:** Advanced

**Time:** 10 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

The ``webots_ros2`` package provides an interface between ROS 2 and Webots.
It includes several sub-packages, including ``webots_ros2_driver``, which allows you to start Webots and communicate with it.
Other sub-packages are mainly examples that show multiple possible implementations using the interface.
In this tutorial, you are going to install the package and learn how to run one of these examples.

Prerequisites
-------------

It is recommended to understand basic ROS principles covered in the beginner :doc:`../../../../Tutorials`.
In particular, :doc:`../../../Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace` and :doc:`../../../Beginner-Client-Libraries/Creating-Your-First-ROS2-Package` are useful prerequisites.

It is necessary to install Webots natively on the mac in order to use the ``webots_ros2`` package in the virtual machine or Docker container as explained below.
You can follow the `installation procedure <https://cyberbotics.com/doc/guide/installation-procedure>`_ or `build it from sources <https://github.com/cyberbotics/webots/wiki/macOS-installation/>`_.

Tasks
-----

On macOS, a solution based on UTM virtual machines or Docker containers provides an improved user experience with ROS 2 compared to native macOS installation, as it runs ROS in a Linux environment.
However, Webots should be installed natively on macOS and it will be able to communicate with the ROS nodes running in the virtual environment (VM or container).
This solution allows for native 3D hardware acceleration for Webots.
The virtual environment runs all the ROS parts and connects to the host machine through TCP to start Webots.
A shared folder allows the script to transfer the world and other resource files from the VM or container to macOS where Webots is running.

The following steps explain how to create the VM image or Docker container with the installation of the ``webots_ros2`` released package.
It is also possible to install it from sources.

1 Create the virtual environment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. tabs::

    .. group-tab:: Virtual machine

      Install UTM on your macOS machine.
      The link can be found on the `official UTM website <https://mac.getutm.app/>`_.

      Download the .iso image of `Ubuntu 22.04 <https://cdimage.ubuntu.com/jammy/daily-live/current/>`_ for Humble and Rolling or `Ubuntu 20.04 <https://cdimage.ubuntu.com/focal/daily-live/pending/>`_ for Foxy.
      Be sure to download the image corresponding to your CPU architecture.

      In the UTM software:

      * Create a new image and choose ``Virtualize`` option.
      * Select the ISO image you have downloaded in the ``Boot ISO Image`` field.
      * Leave all hardware settings at default (including hardware acceleration disabled).
      * In the ``Shared Directory`` window, select a folder that will be used by ``webots_ros2`` to transfer all the Webots assets to the host.
        In this example, the selected folder is ``/Users/username/shared``.
      * Leave all the remaining parameters as default.
      * Start the VM.
        Note that you can select another shared folder each time you start the VM.
      * During the first launch of the VM, install Ubuntu and choose a username for your account. In this example, the username is ``ubuntu``.
      * Once Ubuntu is installed, close the VM, remove the iso image from the CD/DVD field and restart the VM.

    .. group-tab:: Docker container

      Install Docker Desktop for Mac on your macOS machine.
      The link can be found on the `Docker website <https://docs.docker.com/desktop/install/mac-install/>`_.

      Pull a Docker image for ROS.
      This tutorial uses the latest `ROS Base <https://hub.docker.com/layers/library/ros/latest/images/sha256-52e27b46c352d7ee113f60b05590bb089628a17ef648fff6992ca363c5e14945?context=explore>`_ image.

      .. code-block:: console

        docker image pull ros:latest

2 Configure the virtual environment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. tabs::

    .. group-tab:: Virtual machine

      In this section, ROS 2 is installed in the VM and the shared folder is configured.
      The following instructions and commands are all run inside the VM.

      * Open a terminal in the started VM and install the ROS 2 distribution you need by following the instructions in :doc:`../../../../Installation/Ubuntu-Install-Debians`:
      * Create a folder in the VM to use as a shared folder.
        In this example, the shared folder in the VM is ``/home/ubuntu/shared``.

        .. code-block:: console

            mkdir /home/ubuntu/shared

      * To mount this folder to the host, execute the following command.
        Don't forget to modify the path to the shared folder, if it is different in your case.

        .. code-block:: console

            sudo mount -t 9p -o trans=virtio share /home/ubuntu/shared -oversion=9p2000.L

      * To automatically mount this folder to the host when starting the VM, add the following line to ``/etc/fstab``.
        Don't forget to modify the path to the shared folder, if it is different in your case.

        .. code-block:: console

            share	/home/ubuntu/shared	9p	trans=virtio,version=9p2000.L,rw,_netdev,nofail	0	0

      * The environment variable ``WEBOTS_SHARED_FOLDER`` must always be set in order for the package to work properly in the VM.
        This variable specifies the location of the shared folder that is used to exchange data between the host machine and the virtual machine (VM) to the ``webots_ros2`` package.
        The value to use for this variable should be in the format of ``<host shared folder>:<VM shared folder>``, where ``<host shared folder>`` is the path to the shared folder on the host machine and ``<VM shared folder>`` is the path to the same shared folder on the VM.

        In this example:

        .. code-block:: console

          export WEBOTS_SHARED_FOLDER=/Users/username/shared:/home/ubuntu/shared

        You can add this command line to the ``~/.bashrc`` file to automatically set this environment variable when starting a new terminal.
        This variable must be set on both the VM and the host machine.

    .. group-tab:: Docker container

        In this section, the ROS 2 image is installed.

        * Create a folder on the host to use as a shared folder.
          In this example, the shared folder on macOS is ``/Users/username/shared``, where ``username`` is your actual username.

          .. code-block:: console

            mkdir /Users/username/shared

        * Start a new ROS 2 container with a bind mount for the shared directory you created in the previous step.
          The shared directory will be located at ``/root/shared`` inside the container.

          .. code-block:: console

            docker run -it --mount type=bind,src=/Users/username/shared,target=/root/shared ros:latest

          .. note::
            The ROS Docker images are minimal environments.
            The ``webots_ros2`` packages rely on some programs and utilities that are not pre-installed, so you will have to install them manually.
            Inside the container, install the ``iproute2`` package, which provides the ``ip`` command:

            .. code-block:: console

              apt update
              apt install iproute2


        * The environment variable ``WEBOTS_SHARED_FOLDER`` must always be set in order for the package to work properly in the Docker container.
          This variable specifies the location of the shared folder that is used to exchange data between the host machine and the container to the ``webots_ros2`` package.
          The value to use for this variable should be in the format of ``<host shared folder>:<container shared folder>``, where ``<host shared folder>`` is the path to the shared folder on the host machine and ``<container shared folder>`` is the path to the same shared folder on the Docker container.

          In this example:

          .. code-block:: console

            export WEBOTS_SHARED_FOLDER=/Users/username/shared:/root/shared

          This variable must be set on both the container and the host machine.

3 Install ``webots_ros2``
^^^^^^^^^^^^^^^^^^^^^^^^^

You can either install ``webots_ros2`` from the official released package, or install it from the latest up-to-date sources from `GitHub <https://github.com/cyberbotics/webots_ros2>`_.

.. tabs::

    .. group-tab:: Install ``webots_ros2`` distributed package

        Run the following command in the VM or container terminal.
        If you're using the container, you don't need to use ``sudo`` because the container's user is ``root``.
        You will, however, need to update APT's package index files since Docker images typically remove them when being built.

        .. code-block:: console

            apt-get update  # Only needed for Docker containers
            sudo apt-get install ros-{DISTRO}-webots-ros2

    .. group-tab:: Install ``webots_ros2`` from sources

        Install git.
        If you're using the container, you don't need to use ``sudo`` because the container's user is ``root``.

        .. code-block:: console

            sudo apt-get install git

        Create a ROS 2 workspace with its ``src`` directory.

        .. code-block:: console

            mkdir -p ~/ros2_ws/src

        Source the ROS 2 environment.

        .. code-block:: console

            source /opt/ros/{DISTRO}/setup.bash

        Retrieve the sources from GitHub.

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

4 Launch the ``webots_ros2_universal_robot`` example
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

As mentioned in previous sections, the package uses the shared folder to communicate with Webots from the VM or container to the host.
In order for Webots to be started on the host from the VM's or container's ROS package, a local TCP simulation server must be run.

If you didn't already, make sure to set ``WEBOTS_SHARED_FOLDER`` on the host before launching the local TCP simulation server.

.. code-block:: console

  export WEBOTS_SHARED_FOLDER=/Users/username/shared:/root/shared

The server can be downloaded here: `local_simulation_server.py <https://github.com/cyberbotics/webots-server/blob/main/local_simulation_server.py>`_.
Specify the Webots installation folder in ``WEBOTS_HOME`` environment variable (e.g. ``/Applications/Webots.app``) and run the server using the following commands in a new terminal on the host (not in the VM or container):

.. code-block:: console

        export WEBOTS_HOME=/Applications/Webots.app
        python3 local_simulation_server.py


Execute the following commands in the virtual environment to start a package.
If you're using a VM, you may need to open a terminal first.

First source the ROS 2 environment, if not done already.

.. code-block:: console

        source /opt/ros/{DISTRO}/setup.bash

If installed from sources, source your ROS 2 workspace, if not done already.

.. code-block:: console

        cd ~/ros2_ws
        source install/local_setup.bash

If not already set in ``~/.bashrc``, set ``WEBOTS_SHARED_FOLDER`` (see previous sections for details).
Be sure to change the paths according to the location of your respective directories.

.. code-block:: console

        export WEBOTS_SHARED_FOLDER=/Users/username/shared:/home/ubuntu/shared

Use the ROS 2 launch command to start demo packages (e.g. ``webots_ros2_universal_robot``).

.. code-block:: console

        ros2 launch webots_ros2_universal_robot multirobot_launch.py

If Webots is closed or the ROS 2 process is interrupted, the local server will automatically wait for a new package launch and the shared folder will be cleaned for the next run.
