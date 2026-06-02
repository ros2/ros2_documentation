.. redirect-from::

   Installation/Linux-Install-Debians
   Installation/Ubuntu-Install-Debians
   Installation/Linux-Install-Debs

Installing on Ubuntu - how-to
=============================

.. contents:: Contents
   :depth: 3
   :local:

Summary
-------

Deb packages for {DISTRO_TITLE_FULL} are available for {DISTRO_UBUNTU_DEB_PLATFORM}.
We recommend using this current distribution for most situations, the support for this platform is stable.

Resources for this distribution are as follows:

* Status page: ROS 2 {DISTRO_TITLE}, {DISTRO_UBUNTU_DEB_PLATFORM}: `amd64 <http://repo.ros2.org/status_page/ros_{DISTRO}_default.html>`__\ , `arm64 <http://repo.ros2.org/status_page/ros_{DISTRO}_{DISTRO_ARM_STATUS_SUFFIX}.html>`__
* `Jenkins Instance <http://build.ros2.org/>`__
* `Repositories <http://repo.ros2.org>`__

The Rolling Ridley development distribution may be supported on different platforms, as new platforms are selected for development.
Future plans for target platforms are defined in `REP 2000 <https://reps.openrobotics.org/rep-2000/>`__.

Prerequisites
-------------

Check the :doc:`installation requirements <../Installation>`.

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

   a. Ensure that the `Ubuntu Universe repository <https://help.ubuntu.com/community/Repositories/Ubuntu>`_ is enabled.

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

      Installing the ros2-apt-source package configures ROS repositories for your system.
      Updates to repository configuration occur automatically when new versions of this package are released to the ROS repositories.

#. Optional: Install development tools.

   For building packages or other ROS development work, install ``ros-dev-tools``:

   .. code-block:: console

      $ sudo apt update && sudo apt install ros-dev-tools

2 Install ROS
^^^^^^^^^^^^^

1. Update your apt repository caches.

   .. code-block:: console

      $ sudo apt update

2. Ensure your system is up to date.

   ROS packages are built on frequently updated Ubuntu systems.
   Always make sure that your system is up to date before installing new packages.

   .. code-block:: console

      $ sudo apt upgrade

3. Install ROS using one of the following methods:

   .. _linux-install-debs-install-ros-2-packages:

   Desktop install (Recommended): ROS, RViz, demos, tutorials.

   .. code-block:: console

      $ sudo apt install ros-{DISTRO}-desktop

   ROS-Base install (Bare Bones): Communication libraries, message packages, command line tools.
   No GUI tools.

   .. code-block:: console

      $ sudo apt install ros-{DISTRO}-ros-base

4. Optional: Install additional RMW implementations.

   You can install optional RMW packages when your project needs a supported DDS or Zenoh vendor other than the default.
   The default middleware that ROS 2 uses is ``Fast DDS``, but the middleware (RMW) can be replaced at runtime.
   See the :doc:`guide <RMW-Implementations/Working-with-multiple-RMW-implementations>` on how to work with multiple RMWs.

3 Set up your environment
^^^^^^^^^^^^^^^^^^^^^^^^^

#. Set up your environment by sourcing the following file.

   .. code-block:: console

      $ source /opt/ros/{DISTRO}/setup.bash

   .. note::

      If you are not using bash, replace ``.bash`` with your shell.
      Possible values are: ``setup.bash``, ``setup.sh``, ``setup.zsh``.

4 Test the installation
^^^^^^^^^^^^^^^^^^^^^^^

If you installed ``ros-{DISTRO}-desktop``, you can try some examples to check if the installation has been successful.

#. In one terminal, source the setup file, then run a C++ talker:

   .. code-block:: console

      $ source /opt/ros/{DISTRO}/setup.bash
      $ ros2 run demo_nodes_cpp talker

#. In another terminal, source the setup file, then run a Python listener:

   .. code-block:: console

      $ source /opt/ros/{DISTRO}/setup.bash
      $ ros2 run demo_nodes_py listener

   You should see the talker saying that it's publishing messages and the listener saying that it hears those messages.
   This verifies both the C++ and Python APIs are working properly.

If you have issues with the installation, check the :doc:`troubleshooting techniques <Installation-Troubleshooting>`.

If you want to use other RMW implementations, you can check the :doc:`guide <RMW-Implementations>`.

Next steps
----------

After the installation is complete, you can proceed with :doc:`configuring your environment <../Configuring-ROS2-Environment>`.

We recommend that you get familiar with key ROS concepts and check out the tutorials:

* :doc:`First steps with ROS - learning path <../../First-Steps>`

Related content
---------------

More articles:

* :doc:`Create a workspace <../../ROS-Framework/client-libraries/Working-with-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace>`
* :doc:`About ROS <../About-ROS/About-ROS>`

Packages/reference:

* `ros-{DISTRO}-desktop <https://index.ros.org/p/desktop/#{DISTRO}>`__: A package which extends ``ros_base`` and includes high level packages like visualization tools and demos.
* `ros-{DISTRO}-ros-base <https://index.ros.org/p/ros_base/#{DISTRO}>`__: A package which extends ``ros_core`` and includes other basic functionalities like tf2 and urdf.
* `demo_nodes_cpp <https://index.ros.org/p/demo_nodes_cpp/#{DISTRO}>`__: C++ nodes which were previously in the ros2/examples repository but are now just used for demo purposes.
* `demo_nodes_py <https://index.ros.org/p/demo_nodes_py/#{DISTRO}>`__: Python nodes which were previously in the ros2/examples repository but are now just used for demo purposes.
* `ros2-apt-source <https://github.com/ros-infrastructure/ros-apt-source/>`__: Source and key configuration for the ROS 2 apt repository.
* `ros-dev-tools <https://github.com/ros-infrastructure/infra-variants/tree/latest/ros-dev-tools>`__: Variant which includes packages generally useful during ROS development.

FAQs
----

Which Ubuntu platforms are supported?
   ROS 2 {DISTRO_TITLE_FULL} is available for {DISTRO_UBUNTU_DEB_PLATFORM}.

Why am I seeing <some error or symptom of installation issue>?
   See :doc:`Installation-Troubleshooting`.

Can I switch from binaries to a source-based install?
   Yes.
   See :doc:`Alternatives/Ubuntu-Development-Setup`.

.. _ubuntu-debs-uninstall:

How do I uninstall ROS?
   If you need to uninstall ROS or switch to a source-based install once you have already installed from binaries, run the following command:

   .. code-block:: console

      $ sudo apt remove '~nros-{DISTRO}-*' && sudo apt autoremove

   You may also want to remove the repository:

   .. code-block:: console

      $ sudo apt remove ros2-apt-source
      $ sudo apt update
      $ sudo apt autoremove
      $ sudo apt upgrade # Consider upgrading for packages previously shadowed.
