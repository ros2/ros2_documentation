.. redirect-from::

   Installation/Linux-Install-Debians
   Installation/Ubuntu-Install-Debians

Installing on Ubuntu - how-to
=============================

ROS is supported on a range of different platforms.
In this article, you will learn how to install ROS deb packages on Ubuntu.
After you follow these steps, ROS is ready for you to work with.

**Area: ROS-installation | Content-type: how-to | Experience: beginner, intermediate**

.. contents:: Contents
   :depth: 3
   :local:

Summary
-------

Deb packages for {DISTRO_TITLE_FULL} are available for Ubuntu Noble (24.04).
We recommend using this current distribution for most situations, the support for this platform is stable.

Resources for this distribution are here:

* Status page: ROS 2 {DISTRO_TITLE} (Ubuntu Noble 24.04): `amd64 <http://repo.ros2.org/status_page/ros_{DISTRO}_default.html>`__\ , `arm64 <http://repo.ros2.org/status_page/ros_{DISTRO}_unv8.html>`__
* `Jenkins Instance <http://build.ros2.org/>`__
* `Repositories <http://repo.ros2.org>`__

The Rolling Ridley development distribution may be supported on different platforms, as new platforms are selected for development.
Future plans for target platforms are defined in `REP 2000 <https://reps.openrobotics.org/rep-2000/>`__.

Prerequisites
-------------

Check the installation requirements: :doc:`Installation <../Installation>`.

Steps
-----

1 Set up your system
^^^^^^^^^^^^^^^^^^^^

#. Set your locale.

   Make sure you have a locale which supports ``UTF-8``.
   If you are in a minimal environment (such as a docker container), the locale may be something minimal like ``POSIX``.
   We test with the following settings.
   However, it should be fine if you're using a different UTF-8 supported locale.

   .. code-block:: console

      $ locale  # check for UTF-8
      $ sudo apt update && sudo apt install locales
      $ sudo locale-gen en_US en_US.UTF-8
      $ sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
      $ export LANG=en_US.UTF-8
      $ locale  # verify settings

#. Enable the required repositories.
   
   You will need to add the ROS 2 apt repository to your system.

   a. First ensure that the `Ubuntu Universe repository <https://help.ubuntu.com/community/Repositories/Ubuntu>`_ is enabled.

      .. code-block:: console

         $ sudo apt install software-properties-common
         $ sudo add-apt-repository universe

   b. Install the `ros-apt-source <https://github.com/ros-infrastructure/ros-apt-source/>`_ packages.
      
      These provide keys and apt source configuration for the various ROS repositories.

      .. code-block:: console

         $ sudo apt update && sudo apt install curl -y
         $ export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
         $ curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
         $ sudo dpkg -i /tmp/ros2-apt-source.deb

      Installing the ros2-apt-source package will configure ROS repositories for your system.
      Updates to repository configuration will occur automatically when new versions of this package are released to the ROS repositories.

#. Install development tools (optional).
   
   If you are going to build ROS packages or otherwise do development, you can also install the development tools:

   .. code-block:: console

      $ sudo apt update && sudo apt install ros-dev-tools

2 Install ROS
^^^^^^^^^^^^^

1. Update your apt repository caches after setting up the repositories.

   .. code-block:: console

      $ sudo apt update

2. Ensure your system is up to date.

   ROS packages are built on frequently updated Ubuntu systems.
   It is always recommended that you ensure your system is up to date before installing new packages.

   .. code-block:: console

      $ sudo apt upgrade

3. Install ROS, with one of the following methods:

   .. _linux-install-debs-install-ros-2-packages:

   Desktop Install (Recommended): ROS, RViz, demos, tutorials.

   .. code-block:: console

      $ sudo apt install ros-{DISTRO}-desktop

   ROS-Base Install (Bare Bones): Communication libraries, message packages, command line tools.
   No GUI tools.

   .. code-block:: console

      $ sudo apt install ros-{DISTRO}-ros-base

4. Install additional RMW implementations (optional).

   You can install optional RMW packages when your project needs a supported DDS or Zenoh vendor other than the default.
   The default middleware that ROS 2 uses is ``Fast DDS``, but the middleware (RMW) can be replaced at runtime.
   See the :doc:`guide <../How-To-Guides/Working-with-multiple-RMW-implementations>` on how to work with multiple RMWs.

3 Set up your environment
^^^^^^^^^^^^^^^^^^^^^^^^^

#. Set up your environment by sourcing the following file.

   .. code-block:: console

      $ source /opt/ros/{DISTRO}/setup.bash

   .. note::

      Replace ``.bash`` with your shell if you're not using bash.
      Possible values are: ``setup.bash``, ``setup.sh``, ``setup.zsh``.

4 Test the installation
^^^^^^^^^^^^^^^^^^^^^^^

If you installed ``ros-{DISTRO}-desktop`` above you can try some examples, to check the installation has been successful.

#. In one terminal, source the setup file and then run a C++ talker:

   .. code-block:: console

      $ source /opt/ros/{DISTRO}/setup.bash
      $ ros2 run demo_nodes_cpp talker

#. In another terminal source the setup file and then run a Python listener:

   .. code-block:: console

      $ source /opt/ros/{DISTRO}/setup.bash
      $ ros2 run demo_nodes_py listener

   You should see the talker saying that it's publishing messages and the listener saying that it hears those messages.
   This verifies both the C++ and Python APIs are working properly.

If there's an issue with the installation, these troubleshooting techniques may help: :doc:`Installation troubleshooting <../How-To-Guides/Installation-Troubleshooting>`.

If you want to use other RMW implementations, you can check the :doc:`guide <./RMW-Implementations>`.

Next steps
----------

After the installation is complete, you can proceed to configure your environment: :doc:`../Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment`.

We recommend that you get familiar with key ROS concepts and check out the tutorials:

* :doc:`../Concepts`
* :doc:`../Tutorials`

Related content
---------------

More articles:

* :doc:`Create a workspace <../Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace>`
* :doc:`First steps with ROS <../First-Steps>`
* :doc:`About ROS <../About-ROS>`

FAQs
----

Which Ubuntu platforms are supported?
   ROS 2 {DISTRO_TITLE_FULL} is available for Ubuntu Noble (24.04).

Why am I seeing <some error or symptom of installation issue>?
   See :doc:`../How-To-Guides/Installation-Troubleshooting`.

Can I switch from binaries to a source-based install?
   Yes.
   See :doc:`Alternatives/Ubuntu-Development-Setup`.

.. _ubuntu-debs-uninstall:

How do I uninstall ROS?
   If you need to uninstall ROS 2 or switch to a source-based install once you have already installed from binaries, run the following command:

   .. code-block:: console

      $ sudo apt remove '~nros-{DISTRO}-*' && sudo apt autoremove

   You may also want to remove the repository:

   .. code-block:: console

      $ sudo apt remove ros2-apt-source
      $ sudo apt update
      $ sudo apt autoremove
      $ sudo apt upgrade # Consider upgrading for packages previously shadowed.