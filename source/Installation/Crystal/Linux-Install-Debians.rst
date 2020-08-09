Installing ROS 2 via Debian Packages
====================================

.. contents:: Table of Contents
   :depth: 2
   :local:

Debian packages for ROS 2 Crystal (the latest release) and ROS 2 Bouncy are available for Ubuntu Bionic; packages for ROS 2 Ardent are available for Ubuntu Xenial.

Resources
---------

* Status Pages:

  * ROS 2 Crystal (Ubuntu Bionic): `amd64 <http://repo.ros2.org/status_page/ros_crystal_default.html>`__\ , `arm64 <http://repo.ros2.org/status_page/ros_crystal_ubv8.html>`__
  * ROS 2 Bouncy (Ubuntu Bionic): `amd64 <http://repo.ros2.org/status_page/ros_bouncy_default.html>`__\ , `arm64 <http://repo.ros2.org/status_page/ros_bouncy_ubv8.html>`__
  * ROS 2 Ardent (Ubuntu Xenial): `amd64 <http://repo.ros2.org/status_page/ros_ardent_default.html>`__\ , `arm64 <http://repo.ros2.org/status_page/ros_ardent_uxv8.html>`__
* `Jenkins Instance <http://build.ros2.org/>`__
* `Repositories <http://repo.ros2.org>`__

.. _linux-install-debians-setup-sources:

Setup Locale
------------
Make sure you have a locale which supports ``UTF-8``.
If you are in a minimal environment, such as a docker container, the locale may be something minimal like POSIX.
We test with the following settings.
It should be fine if you're using a different UTF-8 supported locale.

.. code-block:: bash

   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   export LANG=en_US.UTF-8

Setup Sources
-------------

.. include:: ../_Apt-Repositories.rst

Install ROS 2 packages
----------------------

First set an environment variable for the ROS 2 release you want to install so it can be used in other commands.

.. code-block:: bash

   export CHOOSE_ROS_DISTRO=crystal  # or bouncy or ardent
   sudo apt update

Desktop Install (Recommended): ROS, RViz, demos, tutorials.

.. code-block:: bash

   sudo apt install ros-$CHOOSE_ROS_DISTRO-desktop

ROS-Base Install (Bare Bones): Communication libraries, message packages, command line tools. No GUI tools.

.. code-block:: bash

   sudo apt install ros-$CHOOSE_ROS_DISTRO-ros-base

Environment setup
-----------------

Sourcing the setup script
^^^^^^^^^^^^^^^^^^^^^^^^^

Set up your environment by sourcing the following file.

.. code-block:: bash

   source /opt/ros/crystal/setup.bash

Install argcomplete (optional)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

ROS 2 command line tools use argcomplete to autocompletion. So if you want autocompletion, installing argcomplete is necessary.

Ubuntu 18.04
~~~~~~~~~~~~

.. code-block:: bash

   sudo apt install python3-argcomplete

Ubuntu 16.04 (argcomplete >= 0.8.5)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To install ``argcomplete`` on Ubuntu 16.04 (Xenial), you'll need to use pip, because the version available through ``apt`` will not work due to a bug in that version of ``argcomplete``:

.. code-block:: bash

   sudo apt install python3-pip
   sudo pip3 install argcomplete

Try some examples
-----------------

In one terminal, set up the ROS 2 environment as described above and then run a C++ ``talker``:

.. code-block:: bash

   ros2 run demo_nodes_cpp talker

In another terminal source the setup file and then run a Python ``listener``:

.. code-block:: bash

   ros2 run demo_nodes_py listener

You should see the ``talker`` saying that it's ``Publishing`` messages and the ``listener`` saying ``I heard`` those messages.
This verifies both the C++ and Python APIs are working properly.
Hooray!

Next steps after installing
---------------------------
Go on with the `tutorials and demos </Tutorials>` to pursue the configuration of your environment, create your own workspace and packages, and learn ROS 2 core concepts.

Uninstall
---------

If you need to uninstall ROS 2 or switch to a source-based install once you
have already installed from binaries, run the following command:

.. code-block:: bash

  sudo apt remove ros-crystal-* && sudo apt autoremove
