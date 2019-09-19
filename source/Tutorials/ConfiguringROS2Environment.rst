Configuring your ROS 2 environment
==================================

**Goal:** This tutorial will explain how to ensure your ROS 2 environment is ready for use.

**Tutorial level:** Beginner

**Time:** 5 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

ROS 2 relies on the notion of combining spaces using the shell environment.
This makes developing against different versions of ROS 2, or against different sets of packages, easier.
It also allows the installation of several ROS 2 distributions (or “distros”, e.g. Dashing and Eloquent) on the same computer and switching between them.

This is accomplished by sourcing setup files every time you open a new shell, or by adding the source command to your shell startup script once.
Without sourcing the setup files, you won’t be able to access ROS 2 commands, or find or use ROS 2 packages.
In other words, you won’t be able to use ROS 2.

Prerequisites
-------------

Before starting these tutorials, install ROS 2 by following the instructions on the ROS 2 :ref:`InstallationGuide` page.

Tasks
-----

1 Source the setup files
^^^^^^^^^^^^^^^^^^^^^^^^

You will need to run this command on every new shell you open to have access to the ROS 2 commands, like so:

.. code-block:: bash

    $ source /opt/ros/<distro>/setup.bash
    # where <distro> is the short name of you ROS 2 distro, e.g. dashing, eloquent, etc.



2 Add sourcing to your shell startup script
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you don’t want to have to source the setup file every time you open a new shell (skipping task 1), then you can add the command to your shell startup script:

.. code-block:: bash

    $ echo "source /opt/ros/<distro>/setup.bash" >> ~/.bashrc

To undo this (to change to another distro), locate your system’s shell startup script and remove the appended source command.

3 Check environment variables
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Sourcing ROS 2 setup files will set your environment variables.
If you are ever having problems finding or using your ROS 2 packages, make sure that your environment is properly setup using the following command:

.. code-block:: bash

    $ printenv | grep ROS

Check that variables like ``ROS_DISTRO`` and ``ROS_VERSION`` are set:

.. code-block:: bash

    ROS_VERSION=2
    ROS_PYTHON_VERSION=3
    ROS_DISTRO=dashing

If the environment variables are not set correctly, return to the ROS 2 package installation section of the installation guide you followed.
If you need more specific help (because environment setup files can come from different places), you can get answers from the community.
See :ref:`GettingAnswers`

Summary
-------

The ROS 2 development environment needs to be correctly configured before use.
This can be done in two ways: either sourcing the setup files in every new shell you open, or adding the source command to your startup script.

If you ever face any problems locating or using packages with ROS 2, the first thing you should do is check your environment variables and ensure they are set to the version and distro you intended.

Next steps
----------

Now that your environment is ready, you can begin learning the core concepts of ROS 2, starting with :ref:`ROS2Nodes`
