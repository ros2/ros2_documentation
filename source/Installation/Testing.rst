.. redirect-from::

   Installation/Prerelease-Testing

Alternative Installation Sources for Testing
============================================

Many ROS packages are provided as pre-built binaries.
Usually, you will get the released version of binaries when following :doc:`../Installation`.
There are also pre-released versions of binaries that are useful for testing before making an official release.
This article describes several options if you would like to try out pre-released versions of ROS binaries.

Debian testing repository
-------------------------

When packages are released into a ROS distribution (using bloom), the buildfarm builds them into debian packages which are stored temporarily in the **building** apt repository.
As dependent packages are rebuilt, an automatic process periodically synchronizes the packages in **building** to a secondary repository called **ros-testing**.
**ros-testing** is intended as a soaking area where developers and bleeding-edge users may give the packages extra testing, before they are manually synced into the public ros repository from which users typically install packages.

Approximately every two weeks, the rosdistro's release manager manually synchronizes the contents of **ros-testing** into the **main** ROS repository.

For Debian-based operating systems, you can install binary packages from the **ros-testing** repository.

1. Make sure you have a working ROS 2 installation from Debian packages (see :doc:`../Installation`).

2. Edit (with sudo) the file ``/etc/apt/sources.list.d/ros2-latest.list`` and change ``ros2`` with ``ros2-testing``.
   For example, on Ubuntu Focal the contents should look like the following:

   .. code-block:: sh

      # deb http://packages.ros.org/ros2/ubuntu focal main
      deb http://packages.ros.org/ros2-testing/ubuntu focal main

3. Update the ``apt`` index:

   .. code-block:: sh

      sudo apt update

4. You can now install individual packages from the testing repository, for example:

   .. code-block:: sh

      sudo apt install ros-{DISTRO}-my-just-released-package

5. Alternatively, you can move your entire ROS 2 installation to the testing repository:

   .. code-block:: sh

      sudo apt dist-upgrade

6. Once you are finished testing, you can switch back to the normal repository by changing back the contents of ``/etc/apt/sources.list.d/ros2-latest.list``:

   .. code-block:: sh

      deb http://packages.ros.org/ros2/ubuntu focal main
      # deb http://packages.ros.org/ros2-testing/ubuntu focal main

   and doing an update and upgrade:

   .. code-block:: sh

      sudo apt update
      sudo apt dist-upgrade

.. _Prerelease_binaries:
