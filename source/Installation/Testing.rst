.. redirect-from::

   Installation/Prerelease-Testing

Testing with pre-release binaries
=================================

Many ROS packages are provided as pre-built binaries.
Usually, you will get the released version of binaries when following :doc:`../Installation`.
There are also pre-released versions of binaries that are useful for testing before making an official release.
This article describes several options if you would like to try out pre-released versions of ROS binaries.

When packages are released into a ROS distribution (using bloom), the buildfarm builds them into deb packages which are stored temporarily in the **building** apt repository.
As dependent packages are rebuilt, an automatic process periodically synchronizes the packages in **building** to a secondary repository called **ros-testing**.
**ros-testing** is intended as a soaking area where developers and bleeding-edge users may give the packages extra testing, before they are manually synced into the public ros repository from which users typically install packages.

Approximately every two weeks, the rosdistro's release manager manually synchronizes the contents of **ros-testing** into the **main** ROS repository.

deb testing repository
----------------------

For Debian-based operating systems, you can install binary packages from the **ros-testing** repository.

1. Make sure you have a working ROS 2 installation from deb packages (see :doc:`../Installation`).

2. Install the ros2-testing-apt-source package.
   This will automatically uninstall the ros2-apt-source package since only one repository may be enabled at a time.

   .. code-block:: console

      $ sudo apt install -y ros2-testing-apt-source

3. Update the apt index:

   .. code-block:: console

      $ sudo apt update

4. You can now install individual packages from the testing repository, for example:

   .. code-block:: console

      $ sudo apt install ros-{DISTRO}-my-just-released-package

5. Alternatively, you can move your entire ROS 2 installation to the testing repository:

   .. code-block:: console

      $ sudo apt dist-upgrade

6. Once you are finished testing, you can switch back to the normal repository by re-installing the ros-apt-source package:

   .. code-block:: console

      $ sudo apt install -y ros2-apt-source

   and doing an update and upgrade:

   .. code-block:: console

      $ sudo apt update
      $ sudo apt dist-upgrade


RHEL testing repository
-----------------------

For RHEL you can install binary packages from the **ros-testing** repository, by enabling the testing repository on the source configuration:

1. Make sure you have a working ROS 2 installation for rpm packages (see the :doc:`RHEL installation instructions <RHEL-Install-RPMs>`).

2. Enable testing and disable main repository:

   .. code-block:: console

      $ sudo dnf config-manager setopt ros2-testing.enabled=1
      $ sudo dnf config-manager setopt ros2.enabled=0

3. Update the dnf index:

   .. code-block:: console

      $ sudo dnf update

4.  You can now install individual packages from the testing repository, for example:

   .. code-block:: console

      $ sudo dnf install ros-{DISTRO}-my-just-released-package

5.  Once you are finished testing, you can switch back to the normal repository by re-enabling the main repository:

   .. code-block:: console

      $ sudo dnf config-manager setopt ros2-testing.enabled=0
      $ sudo dnf config-manager setopt ros2.enabled=1

   and doing an update and upgrade:

   .. code-block:: console

      $ sudo dnf update
      $ sudo dnf system-upgrade

.. _Prerelease_binaries:

Binary archives
---------------

For core packages, we run nightly packaging jobs for Ubuntu Linux, RHEL, and Windows.
These packaging jobs produce archives with pre-built binaries that can be downloaded and extracted to your filesystem.

1. Make sure you have all dependencies installed according to the :doc:`latest development setup <Alternatives/Latest-Development-Setup>` for your platform.

2. Go to https://ci.ros2.org/view/packaging/ and select a packaging job from the list corresponding to your platform.

3. Under the heading "Last Successful Artifacts" you should see a download link (e.g. for Windows, ``ros2-package-windows-AMD64.zip``).

4. Download and extract the archive to your file system.

5. To use the binary archive installation, source the ``setup.*`` file that can be found in the root of the archive.

   .. tabs::

     .. group-tab:: Ubuntu Linux and RHEL

       .. code-block:: console

          $ source path/to/extracted/archive/setup.bash

     .. group-tab:: Windows

       .. code-block:: console

          $ call path\to\extracted\archive\setup.bat

Docker
------

For Ubuntu Linux, there is also a nightly Docker image based on the nightly binary archive.

1. Pull the Docker image:

   .. code-block:: console

      $ docker pull osrf/ros2:nightly

2. Start an interactive container:

   .. code-block:: console

      $ docker run -it osrf/ros2:nightly

For support on running GUI applications in Docker, take a look at the tutorial `User GUI's with Docker <https://wiki.ros.org/docker/Tutorials/GUI>`_ or the tool `rocker <https://github.com/osrf/rocker>`_.
