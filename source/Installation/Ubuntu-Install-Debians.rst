.. redirect-from::

   Installation/Linux-Install-Debians

Ubuntu (Debian packages)
========================

.. contents:: Table of Contents
   :depth: 2
   :local:

Debian packages for ROS 2 {DISTRO_TITLE_FULL} are currently available for Ubuntu Noble (24.04).
The target platforms are defined in `REP 2000 <https://ros.org/reps/rep-2000.html>`__.

Resources
---------

* Status Page:

  * ROS 2 {DISTRO_TITLE} (Ubuntu Noble 24.04): `amd64 <http://repo.ros2.org/status_page/ros_{DISTRO}_default.html>`__\ , `arm64 <http://repo.ros2.org/status_page/ros_{DISTRO}_ujv8.html>`__
* `Jenkins Instance <http://build.ros2.org/>`__
* `Repositories <http://repo.ros2.org>`__

System setup
------------

Set locale
^^^^^^^^^^

.. include:: _Ubuntu-Set-Locale.rst

Enable required repositories
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. include:: _Apt-Repositories.rst

.. _linux-install-debians-install-ros-2-packages:

Install development tools (optional)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you are going to build ROS packages or otherwise do development, you can also install the development tools:

.. code-block:: bash

   sudo apt update && sudo apt install ros-dev-tools

Install ROS 2
-------------

Update your apt repository caches after setting up the repositories.

.. code-block:: bash

   sudo apt update

.. include:: _Apt-Upgrade-Admonition.rst

Desktop Install (Recommended): ROS, RViz, demos, tutorials.

.. code-block:: bash

   sudo apt install ros-{DISTRO}-desktop

ROS-Base Install (Bare Bones): Communication libraries, message packages, command line tools.
No GUI tools.

.. code-block:: bash

   sudo apt install ros-{DISTRO}-ros-base

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

   sudo apt remove ~nros-{DISTRO}-* && sudo apt autoremove

You may also want to remove the repository:

.. code-block:: bash

   sudo rm /etc/apt/sources.list.d/ros2.list
   sudo apt update
   sudo apt autoremove
   # Consider upgrading for packages previously shadowed.
   sudo apt upgrade
