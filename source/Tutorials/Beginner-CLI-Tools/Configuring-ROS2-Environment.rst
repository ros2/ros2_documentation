.. redirect-from::

    Tutorials/Configuring-ROS2-Environment

.. _ConfigROS2:

Configuring environment
=======================

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
Without sourcing the setup files, you won't be able to access ROS 2 commands, or find or use ROS 2 packages.
In other words, you won't be able to use ROS 2.

Prerequisites
-------------

Before starting these tutorials, install ROS 2 by following the instructions on the ROS 2 :doc:`../../Installation` page.

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

      .. code-block:: bash

        # Replace ".bash" with your shell if you're not using bash
        # Possible values are: setup.bash, setup.sh, setup.zsh
        source /opt/ros/{DISTRO}/setup.bash

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

If you don't want to have to source the setup file every time you open a new shell (skipping task 1), then you can add the command to your shell startup script:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        echo "source /opt/ros/{DISTRO}/setup.bash" >> ~/.bashrc

     To undo this, locate your system's shell startup script and remove the appended source command.

   .. group-tab:: macOS

      .. code-block:: console

        echo "source ~/ros2_install/ros2-osx/setup.bash" >> ~/.bash_profile

      To undo this, locate your system's shell startup script and remove the appended source command.

   .. group-tab:: Windows

      Only for PowerShell users, create a folder in 'My Documents' called 'WindowsPowerShell'.
      Within 'WindowsPowerShell', create file 'Microsoft.PowerShell_profile.ps1'.
      Inside the file, paste:

      .. code-block:: console

        C:\dev\ros2_{DISTRO}\local_setup.ps1

      PowerShell will request permission to run this script everytime a new shell is opened.
      To avoid that issue you can run:

      .. code-block:: console

        Unblock-File C:\dev\ros2_{DISTRO}\local_setup.ps1

      To undo this, remove the new 'Microsoft.PowerShell_profile.ps1' file.

3 Check environment variables
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Sourcing ROS 2 setup files will set several environment variables necessary for operating ROS 2.
If you ever have problems finding or using your ROS 2 packages, make sure that your environment is properly set up using the following command:

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
  ROS_DISTRO={DISTRO}

If the environment variables are not set correctly, return to the ROS 2 package installation section of the installation guide you followed.
If you need more specific help (because environment setup files can come from different places), you can `get answers <https://robotics.stackexchange.com/>`__ from the community.

3.1 The ``ROS_DOMAIN_ID`` variable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

See the `domain ID <../../Concepts/Intermediate/About-Domain-ID>` article for details on ROS domain IDs.

Once you have determined a unique integer for your group of ROS 2 nodes, you can set the environment variable with the following command:

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

      If you want to make this permanent between shell sessions, also run:

      .. code-block:: console

        setx ROS_DOMAIN_ID <your_domain_id>

3.2 The ``ROS_LOCALHOST_ONLY`` variable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

By default, ROS 2 communication is not limited to localhost.
``ROS_LOCALHOST_ONLY`` environment variable allows you to limit ROS 2 communication to localhost only.
This means your ROS 2 system, and its topics, services, and actions will not be visible to other computers on the local network.
Using ``ROS_LOCALHOST_ONLY`` is helpful in certain settings, such as classrooms, where multiple robots may publish to the same topic causing strange behaviors.
You can set the environment variable with the following command:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        export ROS_LOCALHOST_ONLY=1

      To maintain this setting between shell sessions, you can add the command to your shell startup script:

      .. code-block:: console

        echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc

   .. group-tab:: macOS

      .. code-block:: console

        export ROS_LOCALHOST_ONLY=1

      To maintain this setting between shell sessions, you can add the command to your shell startup script:

      .. code-block:: console

        echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bash_profile

   .. group-tab:: Windows

      .. code-block:: console

        set ROS_LOCALHOST_ONLY=1

      If you want to make this permanent between shell sessions, also run:

      .. code-block:: console

        setx ROS_LOCALHOST_ONLY 1


Summary
-------

The ROS 2 development environment needs to be correctly configured before use.
This can be done in two ways: either sourcing the setup files in every new shell you open, or adding the source command to your startup script.

If you ever face any problems locating or using packages with ROS 2, the first thing you should do is check your environment variables and ensure they are set to the version and distro you intended.

Next steps
----------

Now that you have a working ROS 2 installation and you know how to source its setup files, you can start learning the ins and outs of ROS 2 with the :doc:`turtlesim tool <./Introducing-Turtlesim/Introducing-Turtlesim>`.
