.. redirect-from::

    Tutorials/Configuring-ROS2-Environment

.. _ConfigROS2:

Setting up your environment - how-to
====================================

After you install ROS, you need to configure your shell before commands and packages are available.
In this guide, you will learn how to access ROS commands and packages by sourcing setup files and configuring environment variables.

**Area: ROS-installation | Content-type: how-to | Experience: beginner, intermediate**

.. contents:: Contents
   :depth: 2
   :local:

Summary
-------

You need to configure the ROS development environment before use.
If you ever face any problems locating or using packages with ROS, the first thing you should do is check your environment variables and ensure they are set to the version and distro you intended.

ROS relies on the notion of combining workspaces using the shell environment.
"Workspace" is the location on your system where you are developing with ROS.
The core ROS workspace is called the underlay.
Subsequent local workspaces are called overlays.
When developing with ROS, you will typically have several workspaces active at the same time.

Combining workspaces makes it easier to develop against different versions of ROS 2 or different sets of packages.
It also allows the installation of several ROS 2 distributions (or "distros", such as Dashing or Eloquent) on the same computer and switching between them.

You can do this in one of the two ways:

* Source the setup files in every new shell you open.
* Add the source command to your startup script.

Without sourcing the setup files, you won't be able to access ROS commands or find or use ROS packages.

Prerequisites
-------------

Install ROS by following the instructions on the ROS :doc:`../../Installation` page.

The commands in this guide assume you installed binary packages for your operating system (deb packages on Linux).
If you built ROS from source, you can still follow this guide, but the path to your setup files is probably different.
If you install from source, you cannot use ``sudo apt install ros-<distro>-<package>``, which appears often in the beginner tutorials.

If you use Linux or macOS, but are not already familiar with the shell, follow `this tutorial <https://www.linux.com/training-tutorials/bash-101-working-cli/>`__.

Steps
-----

1 Source the setup files
^^^^^^^^^^^^^^^^^^^^^^^^

To access the ROS commands, you need to run the following command on every new shell you open:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        $ source /opt/ros/{DISTRO}/setup.bash


      Replace ``.bash`` with your shell if you're not using bash.
      Possible values are: ``setup.bash``, ``setup.sh``, ``setup.zsh``.

   .. group-tab:: macOS

      .. code-block:: console

        $ . ~/ros2_install/ros2-osx/setup.bash

   .. group-tab:: Windows

      .. code-block:: console

        $ call C:\dev\ros2\local_setup.bat

.. note::
    The exact command depends on where you installed ROS.
    If you have problems, make sure that the file path leads to your installation.

2 Add sourcing to your shell startup script
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you don't want to source the setup file every time you open a new shell, you can add the following command to your shell startup script:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        $ echo "source /opt/ros/{DISTRO}/setup.bash" >> ~/.bashrc

      To undo this, locate your system's shell startup script and remove the appended source command.

   .. group-tab:: macOS

      .. code-block:: console

        $ echo "source ~/ros2_install/ros2-osx/setup.bash" >> ~/.bash_profile

      To undo this, locate your system's shell startup script and remove the appended source command.

   .. group-tab:: Windows

      If you use PowerShell, create a folder in **My Documents** named **WindowsPowerShell**.
      Inside **WindowsPowerShell**, create the file **Microsoft.PowerShell_profile.ps1**.
      Paste the following line into that file:

      .. code-block:: console

        $ C:\dev\ros2_{DISTRO}\local_setup.ps1

      PowerShell will request permission to run this script every time a new shell is opened.
      To avoid that issue you can run:

      .. code-block:: console

        $ Unblock-File C:\dev\ros2_{DISTRO}\local_setup.ps1

      To undo this step, delete **Microsoft.PowerShell_profile.ps1**.

3 Check environment variables
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Sourcing ROS setup files sets the environment variables that ROS needs.
If you cannot find or use packages, confirm your environment with the following command:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        $ printenv | grep -i ROS

   .. group-tab:: macOS

      .. code-block:: console

        $ printenv | grep -i ROS

   .. group-tab:: Windows

      .. code-block:: console

        $ set | findstr -i ROS

Confirm that variables such as ``ROS_DISTRO`` and ``ROS_VERSION`` are set.

::

  ROS_VERSION=2
  ROS_PYTHON_VERSION=3
  ROS_DISTRO={DISTRO}

If the environment variables are not set correctly, return to the ROS package installation section of the installation guide you followed.
If you need more specific help (because environment setup files can come from different places), you can `get answers <https://robotics.stackexchange.com/>`__ from the community.

3.1 The ``ROS_DOMAIN_ID`` variable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For details on domain IDs, read :doc:`About Domain ID <../../Concepts/Intermediate/About-Domain-ID>`.

After you determine a unique integer for your group of ROS nodes, set the variable with the following command:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        $ export ROS_DOMAIN_ID=<your_domain_id>

      To maintain this setting between shell sessions, you can add the command to your shell startup script:

      .. code-block:: console

        $ echo "export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bashrc

   .. group-tab:: macOS

      .. code-block:: console

        $ export ROS_DOMAIN_ID=<your_domain_id>

      To maintain this setting between shell sessions, you can add the command to your shell startup script:

      .. code-block:: console

        $ echo "export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bash_profile

   .. group-tab:: Windows

      .. code-block:: console

        $ set ROS_DOMAIN_ID=<your_domain_id>

      If you want to make this permanent between shell sessions, also run:

      .. code-block:: console

        $ setx ROS_DOMAIN_ID <your_domain_id>

3.2 The ``ROS_AUTOMATIC_DISCOVERY_RANGE`` variable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

By default, ROS communication is not limited to localhost.
``ROS_AUTOMATIC_DISCOVERY_RANGE`` environment variable allows you to limit ROS discovery range.
Using ``ROS_AUTOMATIC_DISCOVERY_RANGE`` is helpful in certain settings, such as classrooms, where multiple robots may publish to the same topic causing strange behaviors.
See :ref:`Improved Dynamic Discovery <ImprovedDynamicDiscovery>` for more details.

4 Verify your environment configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Use this step to confirm that your shell has the ROS paths and variables from the earlier steps in this guide, including any optional variables you set.

Choose a terminal session that matches how you configured ROS:

* If you only source manually, run the commands below after you run your ``source``, ``.``, or ``call`` command in that terminal.
* If you use a startup script, open a new terminal window first, then run the commands below.

Run:

.. code-block:: console

   $ ros2 --help

If usage text appears, your shell picked up the ROS command paths from the setup files in this guide.

If you set optional ``ROS_DOMAIN_ID`` or ``ROS_AUTOMATIC_DISCOVERY_RANGE`` values earlier in this guide, run the same ROS-related environment check again in this shell (for example ``printenv`` / ``set`` as when you reviewed ROS variables) and confirm you see the values you expect.

Related content
----------------

More articles:

* :doc:`Installing on Ubuntu (deb packages) <../../Installation/Ubuntu-Install-Debs>`
* :doc:`Installing on Windows <../../Installation/Windows-Install-Binary>`
* :doc:`Installing on RHEL (RPM packages) <../../Installation/RHEL-Install-RPMs>`
* :doc:`Creating a workspace <../Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace>`

FAQs
----

Do I need to source the setup files in every new terminal?
   Yes.
   ROS commands and packages are only available in shells where you have sourced the setup files.
   To avoid doing this manually each time, add the source command to your shell startup script.

How do I check that my ROS environment is configured correctly?
   List ROS-related environment variables (for example with ``printenv | grep -i ROS`` on Linux or macOS, or ``set | findstr -i ROS`` on Windows).
   Confirm that ``ROS_DISTRO`` and ``ROS_VERSION`` are set, then run ``ros2 --help``.
   If usage text appears, your shell picked up the ROS command paths from the setup files.

Why is the ``ros2`` command not found after I installed ROS?
   Installing ROS does not configure your shell automatically.
   You must source the setup file for your installation (for example ``/opt/ros/{DISTRO}/setup.bash`` on Linux when using deb packages) in each new shell, or add that command to your startup script.

What is ``ROS_DOMAIN_ID`` and when should I set it?
   ``ROS_DOMAIN_ID`` selects which logical DDS domain your nodes use for discovery.
   Nodes only discover each other when they share the same domain ID.
   Set a unique value when you need to isolate groups of nodes, such as in classrooms where multiple robots would otherwise see the same topics.

Can I install multiple ROS distributions on the same computer?
   Yes.
   Install each distribution separately, then source the setup file for the one you want to use in that shell session.
   Combining workspaces through sourcing also lets you layer local overlays on top of the core underlay.