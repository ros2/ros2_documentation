Installing ROS 2 on RHEL
========================

.. contents:: Table of Contents
   :depth: 2
   :local:

This page explains how to install ROS 2 on RHEL from a pre-built binary package.

.. note::

    The pre-built binary does not include all ROS 2 packages.
    All packages in the `ROS base variant <https://ros.org/reps/rep-2001.html#ros-base>`_ are included, and only a subset of packages in the `ROS desktop variant <https://ros.org/reps/rep-2001.html#desktop-variants>`_ are included.
    The exact list of packages are described by the repositories listed in `this ros2.repos file <https://github.com/ros2/ros2/blob/{REPOS_FILE_BRANCH}/ros2.repos>`_.

There are also `RPM packages <RHEL-Install-RPMs>` available.

System Requirements
-------------------

We currently support RHEL 8 64-bit.
The Rolling Ridley distribution will change target platforms from time to time as new platforms are selected for development.
Most people will want to use a stable ROS distribution.

Enable required repositories
----------------------------

The rosdep database contains packages from the EPEL and PowerTools repositories, which are not enabled by default.
They can be enabled by running:

.. code-block:: bash

   sudo dnf install 'dnf-command(config-manager)' epel-release -y
   sudo dnf config-manager --set-enabled powertools

Installing prerequisites
------------------------

There are a few packages that must be installed in order to get and unpack the binary release.

.. code-block:: bash

   sudo dnf install tar bzip2 wget -y

Downloading ROS 2
-----------------

Binary releases of Rolling Ridley are not provided.
Instead you may download nightly `prerelease binaries <Prerelease_binaries>`.

* Download the latest package for RHEL; let's assume that it ends up at ``~/Downloads/ros2-package-linux-x86_64.tar.bz2``.

  * Note: there may be more than one binary download option which might cause the file name to differ.

* Unpack it:

  .. code-block:: bash

       mkdir -p ~/ros2_{DISTRO}
       cd ~/ros2_{DISTRO}
       tar xf ~/Downloads/ros2-package-linux-x86_64.tar.bz2

Installing and initializing rosdep
----------------------------------

.. code-block:: bash

       sudo dnf install -y python3-rosdep
       sudo rosdep init
       rosdep update

.. _rhel-install-binary-install-missing-dependencies:

Installing the missing dependencies
-----------------------------------

Set your rosdistro according to the release you downloaded.

.. code-block:: bash

       rosdep install --from-paths ~/ros2_{DISTRO}/ros2-linux/share --ignore-src -y --skip-keys "cyclonedds fastcdr fastrtps ignition-cmake2 ignition-math6 rti-connext-dds-5.3.1 urdfdom_headers pydocstyle python3-mypy python3-babeltrace python3-lttng"

Install additional DDS implementations (optional)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you would like to use another DDS or RTPS vendor besides the default, Eclipse Cyclone DDS, you can find instructions `here <DDS-Implementations>`.

Environment setup
-----------------

Sourcing the setup script
^^^^^^^^^^^^^^^^^^^^^^^^^

Set up your environment by sourcing the following file.

.. code-block:: bash

  . ~/ros2_{DISTRO}/ros2-linux/setup.bash

Try some examples
-----------------

In one terminal, source the setup file and then run a C++ ``talker``:

.. code-block:: bash

   . ~/ros2_{DISTRO}/ros2-linux/setup.bash
   ros2 run demo_nodes_cpp talker

In another terminal source the setup file and then run a Python ``listener``:

.. code-block:: bash

   . ~/ros2_{DISTRO}/ros2-linux/setup.bash
   ros2 run demo_nodes_py listener

You should see the ``talker`` saying that it's ``Publishing`` messages and the ``listener`` saying ``I heard`` those messages.
This verifies both the C++ and Python APIs are working properly.
Hooray!

Next steps after installing
---------------------------
Continue with the `tutorials and demos <../Tutorials>` to configure your environment, create your own workspace and packages, and learn ROS 2 core concepts.

Additional RMW implementations (optional)
-----------------------------------------
The default middleware that ROS 2 uses is ``Cyclone DDS``, but the middleware (RMW) can be replaced at runtime.
See the `guide <../How-To-Guides/Working-with-multiple-RMW-implementations>` on how to work with multiple RMWs.

Troubleshooting
---------------

Troubleshooting techniques can be found `here <../How-To-Guides/Installation-Troubleshooting>`.

Uninstall
---------

1. If you installed your workspace with colcon as instructed above, "uninstalling" could be just a matter of opening a new terminal and not sourcing the workspace's ``setup`` file.
   This way, your environment will behave as though there is no {DISTRO_TITLE} install on your system.

2. If you're also trying to free up space, you can delete the entire workspace directory with:

   .. code-block:: bash

    rm -rf ~/ros2_{DISTRO}
