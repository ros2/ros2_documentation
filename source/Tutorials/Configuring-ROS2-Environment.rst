.. _ConfigROS2:

Configuring your ROS 2 environment
==================================

**Goal:** This tutorial will show you how to prepare your ROS 2 environment.

**Tutorial level:** Beginner

**Time:** 5 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

ROS 2 relies on the notion of combining workspaces using the shell environment.
"Workspace" is a ROS term for the location on your system where you're developing with ROS 2.
The core ROS 2 workspace is called the underlay.
Subsequent local workspaces are called overlays.
When developing with ROS 2, you will typically have several workspaces active concurrently.

Combining workspaces makes developing against different versions of ROS 2, or against different sets of packages, easier.
It also allows the installation of several ROS 2 distributions (or “distros”, e.g. Dashing and Eloquent) on the same computer and switching between them.

This is accomplished by sourcing setup files every time you open a new shell, or by adding the source command to your shell startup script once.
Without sourcing the setup files, you won’t be able to access ROS 2 commands, or find or use ROS 2 packages.
In other words, you won’t be able to use ROS 2.

Prerequisites
-------------

Before starting these tutorials, install ROS 2 by following the instructions on the ROS 2 :ref:`InstallationGuide` page.

The commands used in this tutorial assume you followed the binary packages installation guide for your operating system (Debian packages for Linux).
You can still follow along if you built from source, but the path to your setup files will likely be different.
You also won't be able to use the ``sudo apt install ros-<distro>-<package>`` command (used frequently in the beginner level tutorials) if you install from source.

If you are using Linux or macOS, but are not already familiar with the shell, `this tutorial <http://www.ee.surrey.ac.uk/Teaching/Unix/>`__ will help.

Tasks
-----

1 Source the setup files
^^^^^^^^^^^^^^^^^^^^^^^^

You will need to run this command on every new shell you open to have access to the ROS 2 commands, like so:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        source /opt/ros/rolling/setup.bash

   .. group-tab:: macOS

      .. code-block:: console

        . ~/ros2_install/ros2-osx/setup.bash

   .. group-tab:: Windows

      .. code-block:: console

        call C:\dev\ros2\local_setup.bat

.. note::
    The exact command depends on where you installed ROS 2.
    If you're having problems, ensure the file path leads to your installation.

2 Add sourcing to your shell startup script
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you don’t want to have to source the setup file every time you open a new shell (skipping task 1), then you can add the command to your shell startup script:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        echo "source /opt/ros/rolling/setup.bash" >> ~/.bashrc

   .. group-tab:: macOS

      .. code-block:: console

        echo "source ~/ros2_install/ros2-osx/setup.bash" >> ~/.bash_profile

   .. group-tab:: Windows

      Only for PowerShell users, create a folder in 'My Documents' called 'WindowsPowerShell'.
      Within 'WindowsPowerShell', create file 'Microsoft.PowerShell_profile.ps1'.
      Inside the file, paste:

      .. code-block:: console

        C:\dev\ros2_rolling\local_setup.ps1

      PowerShell will request permission to run this script everytime a new shell is opened.

To undo this (to change to another distro) in Linux and macOS, locate your system’s shell startup script and remove the appended source command.

3 Add ``colcon_cd`` to your shell startup script
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The command ``colcon_cd`` allows you to quickly change the current working directory of your shell to the directory of a package.
As an example ``colcon_cd some_ros_package`` would quickly bring you to the directory ``~/ros2_install/src/some_ros_package``.

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
        echo "export _colcon_cd_root=~/ros2_install" >> ~/.bashrc

   .. group-tab:: macOS

      .. code-block:: console

        TODO

   .. group-tab:: Windows

      Not yet available

Depending to the way you installed ``colcon_cd`` and where your workspace is, the instructions above may vary, please refer to `the documentation <https://colcon.readthedocs.io/en/released/user/installation.html#quick-directory-changes>`__ for more details.
To undo this in Linux and macOS, locate your system’s shell startup script and remove the appended source and export commands.

4 Check environment variables
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Sourcing ROS 2 setup files will set several environment variables necessary for operating ROS 2.
If you ever have problems finding or using your ROS 2 packages, make sure that your environment is properly setup using the following command:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        printenv | grep -i ROS

   .. group-tab:: macOS

      .. code-block:: console

        printenv | grep -i ROS

   .. group-tab:: Windows

      .. code-block:: console

        set | findstr -i ROS

Check that variables like ``ROS_DISTRO`` and ``ROS_VERSION`` are set.

::

  ROS_VERSION=2
  ROS_PYTHON_VERSION=3
  ROS_DISTRO=rolling

If the environment variables are not set correctly, return to the ROS 2 package installation section of the installation guide you followed.
If you need more specific help (because environment setup files can come from different places), you can `get answers <https://answers.ros.org>`__ from the community.

3.1 The ``ROS_DOMAIN_ID`` variable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If your lab or office has multiple different groups of computers running ROS 2, and you want to avoid cross-talk between the groups, choose a single integer and set it as the environment variable ``ROS_DOMAIN_ID`` on all the computers in a group. Choose a different, unique integer for each subgroup. (For the default RMW on eProsima Fast RTPS, as of ROS 2 Eloquent, this integer must be between 0-232 for the ROS 2 daemon to successfully start.)

The domain ID is used to segment the network in order to avoid interference between different groups of computers running ROS 2 on the same local area network. Machines with different domain IDs will not talk, nor interfere, with each other.

If you run into issues having multiple computers talk to each other check `Troubleshooting </Troubleshooting>`. Additionally, there are multiple past conversations on our `Discourse <https://discourse.ros.org/>`_ and `Answers <https://answers.ros.org/questions/>`_ sites with more in-depth information.

Once you have determined a unique integer for your group of ROS 2 agents, you can set the environment variable with the following command:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        export ROS_DOMAIN_ID=<your_domain_id>

      To maintain this setting between shell sessions, you can add the command to your shell startup script:

      .. code-block:: console

        echo "export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bashrc

   .. group-tab:: macOS

      .. code-block:: console

        export ROS_DOMAIN_ID=<your_domain_id>

      To maintain this setting between shell sessions, you can add the command to your shell startup script:

      .. code-block:: console

        echo "export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bash_profile

   .. group-tab:: Windows

      .. code-block:: console

        set ROS_DOMAIN_ID=<your_domain_id>

      If you want to make this permanant between shell sessions, also run:

      .. code-block:: console

        setx ROS_DOMAIN_ID <your_domain_id>


Summary
-------

The ROS 2 development environment needs to be correctly configured before use.
This can be done in two ways: either sourcing the setup files in every new shell you open, or adding the source command to your startup script.

If you ever face any problems locating or using packages with ROS 2, the first thing you should do is check your environment variables and ensure they are set to the version and distro you intended.

Next steps
----------

Now that you have a working ROS 2 installation and you know how to source its setup files, you can start learning the ins and outs of ROS 2 with the :ref:`turtlesim tool <Turtlesim>`.
