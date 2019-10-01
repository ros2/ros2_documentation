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

If you are using Linux or macOS, but are not already familiar with the shell, `this tutorial <http://www.ee.surrey.ac.uk/Teaching/Unix/>`__ will help.

Tasks
-----

1 Source the setup files
^^^^^^^^^^^^^^^^^^^^^^^^

You will need to run this command on every new shell you open to have access to the ROS 2 commands, like so:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: bash

        source /opt/ros/<distro>/setup.bash

      For example, if you installed ROS 2 Dashing:

      .. code-block:: bash

        source /opt/ros/dashing/setup.bash

   .. group-tab:: macOS

      .. code-block:: bash

        . ~/ros2_install/ros2-osx/setup.bash

   .. group-tab:: Windows

      .. code-block:: bash

        call C:\dev\ros2\local_setup.bat

.. note::
    The exact command depends on where you installed ROS 2.
    If you're having problems, ensure the file path leads to your installation.

2 Add sourcing to your shell startup script
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you don’t want to have to source the setup file every time you open a new shell (skipping task 1), then you can add the command to your shell startup script:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: bash

        echo "source /opt/ros/<distro>/setup.bash" >> ~/.bashrc

   .. group-tab:: macOS

      .. code-block:: bash

        echo "source ~/ros2_install/ros2-osx/setup.bash" >> ~/.bash_profile

   .. group-tab:: Windows

      Requires registry edits

To undo this (to change to another distro) in Linux and macOS, locate your system’s shell startup script and remove the appended source command.

3 Check environment variables
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Sourcing ROS 2 setup files will set several environment variables necessary for operating ROS 2.
If you ever have problems finding or using your ROS 2 packages, make sure that your environment is properly setup using the following command:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: bash

        printenv | grep -i ROS

   .. group-tab:: macOS

      .. code-block:: bash

        printenv | grep -i ROS

   .. group-tab:: Windows

      .. code-block:: bash

        set | findstr -i ROS

Check that variables like ``ROS_DISTRO`` and ``ROS_VERSION`` are set:

.. code-block:: bash

    ROS_VERSION=2
    ROS_PYTHON_VERSION=3
    ROS_DISTRO=dashing

If the environment variables are not set correctly, return to the ROS 2 package installation section of the installation guide you followed.
If you need more specific help (because environment setup files can come from different places), you can get answers from the community.

.. todo: link to "Getting Answers" once all tutorials are done (no empty references)

3.1 The ``ROS_DOMAIN_ID`` variable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If your lab or office has multiple computers running ROS 2, it is important that each system sets a unique integer for the environment variable ``ROS_DOMAIN_ID``.

The domain ID is used to segment the network in order to avoid interference between ROS 2 programs.
Once you have determined a unique integer for yourself, you can set the environment variable with the following command:


.. tabs::

   .. group-tab:: Linux

      .. code-block:: bash

        export ROS_DOMAIN_ID=<your_domain_id>

   .. group-tab:: macOS

      .. code-block:: bash

        export ROS_DOMAIN_ID=<your_domain_id>

   .. group-tab:: Windows

      .. code-block:: bash

        set ROS_DOMAIN_ID <your_domain_id>

To maintain this setting between sessions on Linux and macOS, you can add the above command to your shell startup script using the syntax from section 2 of this tutorial.

On Windows, you can use the command:

.. code-block:: bash

  setx ROS_DOMAIN_ID <your_domain_id>

Summary
-------

The ROS 2 development environment needs to be correctly configured before use.
This can be done in two ways: either sourcing the setup files in every new shell you open, or adding the source command to your startup script.

If you ever face any problems locating or using packages with ROS 2, the first thing you should do is check your environment variables and ensure they are set to the version and distro you intended.

.. todo: "Next steps section" link to "Introducing turtlesim" once all tutorials are done (no empty references)
