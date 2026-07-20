.. redirect-from::

    Tutorials/Configuring-ROS2-Environment
    Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment

.. _ConfigROS2:

Setting up your environment - how-to
====================================

After you install ROS, you need to configure your shell to access the relevant commands and packages.
This article describes how to access ROS commands and packages by sourcing setup files and configuring environment variables.

**Area: ROS-installation | Content-type: how-to | Experience: beginner, intermediate**

.. contents:: Contents
   :depth: 2
   :local:

Summary
-------

You always need to ``source`` the setup files to configure your shell environment.

You can do this in one of the two ways:

* Source the setup files in every new shell you open.
* Add the ``source`` command to your startup script.

Without sourcing the setup files, you won't be able to access ROS commands or find and use ROS packages.

Prerequisites
-------------

Install ROS by following the instructions on the ROS :doc:`Installation` page.

This guide assumes you have installed binary packages for your operating system (deb packages on Linux).
If you built ROS from source, you can still follow this guide, but the path to your setup files is probably different.

.. note::

   If you installed from source, you cannot use ``sudo apt install ros-<distro>-<package>``.

If you use Linux or macOS, but are not already familiar with the shell, follow `this tutorial <https://www.linux.com/training-tutorials/bash-101-working-cli/>`__.

Steps
-----

1 Source the setup files
^^^^^^^^^^^^^^^^^^^^^^^^

Use the ``source`` command to import the environment variables which enable your shell to access the ROS commands.
You need to run this on every new shell you open:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        $ source /opt/ros/{DISTRO}/setup.bash


      If you're not using bash, replace ``.bash`` with one of the possible values: ``setup.bash``, ``setup.sh``, or ``setup.zsh``.

   .. group-tab:: macOS

      .. code-block:: console

        $ . ~/ros2_install/ros2-osx/setup.bash

   .. group-tab:: Windows

      .. code-block:: console

        $ call C:\dev\ros2\local_setup.bat

.. note::
    The exact command depends on where you installed ROS.
    If you have problems, make sure that the file path leads to your installation.

2 (Optional) Add sourcing to your shell startup script
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you don't want to source the setup file every time you open a new shell, you can add the following command to your shell startup script:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        $ echo "source /opt/ros/{DISTRO}/setup.bash" >> ~/.bashrc

      To undo this, locate your system's shell startup script and remove the ``source`` command.

   .. group-tab:: macOS

      .. code-block:: console

        $ echo "source ~/ros2_install/ros2-osx/setup.bash" >> ~/.bash_profile

      To undo this, locate your system's shell startup script and remove the appended ``source`` command.

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

After you source the setup files, check that your shell picked up the ROS environment variables.
This confirms which distribution is active and whether the ``source`` command succeeded.

To list ROS-related environment variables, run:

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

.. note::
    If you later can't find or use packages, check these variables again and make sure that they match the version and distribution you intended to use.

If the environment variables are not set correctly, return to the ROS package installation section of the installation guide you followed.
If you need more specific help (because environment setup files can come from different places), :doc:`get answers from the community <../The-ROS2-Project/Contributing>`.

3.1 (Optional) Set the ``ROS_DOMAIN_ID`` variable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``ROS_DOMAIN_ID`` places your nodes in a DDS domain.
Nodes only communicate with other nodes in the same domain, so a unique value prevents your nodes from seeing or being seen by other ROS nodes on the same network.
For more details, see the :doc:`domain ID <../ROS-Framework/nodes/About-Domain-ID>` article.

For the value of ``ROS_DOMAIN_ID``, choose any integer between 0 and 101 that no other ROS user on your network is using.
If you are working alone or on an isolated network, any value in that range is fine.

To set the ``ROS_DOMAIN_ID`` variable, run:

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

3.2 (Optional) Set the ``ROS_AUTOMATIC_DISCOVERY_RANGE`` variable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

By default, ROS nodes try to discover other nodes across the local network, not only on your machine.
The ``ROS_AUTOMATIC_DISCOVERY_RANGE`` environment variable lets you limit how far that discovery reaches.
This is helpful in settings such as classrooms, where multiple robots may publish to the same topic and interfere with one another.

To limit discovery to the local machine, set the variable to ``LOCALHOST``:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        $ export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

      To maintain this setting between shell sessions, you can add the command to your shell startup script:

      .. code-block:: console

        $ echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" >> ~/.bashrc

   .. group-tab:: macOS

      .. code-block:: console

        $ export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

      To maintain this setting between shell sessions, you can add the command to your shell startup script:

      .. code-block:: console

        $ echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" >> ~/.bash_profile

   .. group-tab:: Windows

      .. code-block:: console

        $ set ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

      If you want to make this permanent between shell sessions, also run:

      .. code-block:: console

        $ setx ROS_AUTOMATIC_DISCOVERY_RANGE LOCALHOST

For other values and more advanced discovery options, see :ref:`Improved Dynamic Discovery <ImprovedDynamicDiscovery>`.

4 Verify your environment configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Confirm that your shell has the ROS paths and variables from the earlier steps in this article, including any optional variables you set.

Open a terminal session:

* If you only source manually, open a new terminal window and run your ``source``, ``.``, or ``call`` command.
* If you use a startup script, open a new terminal window.

In your terminal window, run:

.. code-block:: console

   $ ros2 --help

If usage text appears, your shell picked up the ROS command paths from the setup files in this guide.

If you set optional ``ROS_DOMAIN_ID`` or ``ROS_AUTOMATIC_DISCOVERY_RANGE`` values earlier in this guide, run the environment check again in this terminal session to confirm you see the values you expect:

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

Related content
---------------

More articles:

* :doc:`Installing on Ubuntu (deb packages) <Installation/Ubuntu-Install-Debs>`
* :doc:`Installing on Windows <Installation/Windows-Install-Binary>`
* :doc:`Installing on RHEL (RPM packages) <Installation/RHEL-Install-RPMs>`
* :doc:`Creating a workspace <../ROS-Framework/client-libraries/Working-with-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace>`
* :doc:`Introducing turtlesim <Introducing-Turtlesim/Introducing-Turtlesim>`

FAQs
----

Do I need to source the setup files in every new terminal?
   Yes.
   ROS commands and packages are only available in shells where you have sourced the setup files.
   To avoid doing this manually each time, add the ``source`` command to your shell startup script.

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
