RHEL (RPM packages)
===================

.. contents:: Table of Contents
   :depth: 2
   :local:

RPM packages for ROS 2 {DISTRO_TITLE_FULL} are currently available for RHEL 9.
The Rolling Ridley distribution will change target platforms from time to time as new platforms are selected for development.
The target platforms are defined in `REP 2000 <https://ros.org/reps/rep-2000.html>`__.
Most people will want to use a stable ROS distribution.

Resources
---------

* Status Page:

  * ROS 2 {DISTRO_TITLE} (RHEL 9): `amd64 <http://repo.ros2.org/status_page/ros_{DISTRO}_rhel.html>`__
* `Jenkins Instance <http://build.ros2.org/>`__
* `Repositories <http://repo.ros2.org>`__

System setup
------------

Set locale
^^^^^^^^^^

.. include:: _RHEL-Set-Locale.rst

Enable required repositories
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You will need to enable the EPEL repositories and the PowerTools repository:

.. code-block:: bash

   sudo dnf install 'dnf-command(config-manager)' epel-release -y
   sudo dnf config-manager --set-enabled crb

.. note:: This step may be slightly different depending on the distribution you are using. Check the EPEL documentation: https://docs.fedoraproject.org/en-US/epel/#_quickstart

Next, download the ROS 2 .repo file:

.. code-block:: bash

   sudo dnf install curl
   sudo curl --output /etc/yum.repos.d/ros2.repo http://packages.ros.org/ros2/rhel/ros2.repo

Then, update your metadata cache.
DNF may prompt you to verify the GPG key, which should match the location ``https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc``.

.. code-block:: bash

   sudo dnf makecache

Install development tools (optional)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you are going to build ROS packages or otherwise do development, you can also install the development tools:

.. code-block:: bash

   sudo dnf install -y \
     cmake \
     gcc-c++ \
     git \
     make \
     patch \
     python3-colcon-common-extensions \
     python3-mypy \
     python3-pip \
     python3-pytest \
     python3-pytest-repeat \
     python3-pytest-rerunfailures \
     python3-rosdep \
     python3-setuptools \
     python3-vcstool \
     wget

   # install some pip packages needed for testing and
   # not available as RPMs
   python3 -m pip install -U --user \
     flake8-blind-except==0.1.1 \
     flake8-class-newline \
     flake8-deprecated

Install ROS 2
-------------

.. include:: _Dnf-Update-Admonition.rst

Desktop Install (Recommended): ROS, RViz, demos, tutorials.

.. code-block:: bash

   sudo dnf install ros-{DISTRO}-desktop

ROS-Base Install (Bare Bones): Communication libraries, message packages, command line tools.
No GUI tools.

.. code-block:: bash

   sudo dnf install ros-{DISTRO}-ros-base

Install additional RMW implementations (optional)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The default middleware that ROS 2 uses is ``Fast DDS``, but the middleware (RMW) can be replaced at runtime.
See the :doc:`guide <../How-To-Guides/Working-with-multiple-RMW-implementations>` on how to work with multiple RMWs.

Setup environment
-----------------

Set up your environment by sourcing the following file.

.. code-block:: bash

   # Replace ".bash" with your shell if you're not using bash
   # Possible values are: setup.bash, setup.sh, setup.zsh
   source /opt/ros/{DISTRO}/setup.bash

Try some examples
-----------------

If you installed ``ros-{DISTRO}-desktop`` above you can try some examples.

In one terminal, source the setup file and then run a C++ ``talker``\ :

.. code-block:: bash

   source /opt/ros/{DISTRO}/setup.bash
   ros2 run demo_nodes_cpp talker

In another terminal source the setup file and then run a Python ``listener``\ :

.. code-block:: bash

   source /opt/ros/{DISTRO}/setup.bash
   ros2 run demo_nodes_py listener

You should see the ``talker`` saying that it's ``Publishing`` messages and the ``listener`` saying ``I heard`` those messages.
This verifies both the C++ and Python APIs are working properly.
Hooray!

Next steps
----------

Continue with the :doc:`tutorials and demos <../../Tutorials>` to configure your environment, create your own workspace and packages, and learn ROS 2 core concepts.

Troubleshoot
------------

Troubleshooting techniques can be found :doc:`here <../How-To-Guides/Installation-Troubleshooting>`.

Uninstall
---------

If you need to uninstall ROS 2 or switch to a source-based install once you
have already installed from binaries, run the following command:

.. code-block:: bash

   sudo dnf remove ros-{DISTRO}-*
