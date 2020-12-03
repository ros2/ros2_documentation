Pre-release Testing
===================

Many ROS packages are provided as pre-built binaries.
Usually, you will get the released version of binaries when following :ref:`InstallationGuide`.
There are also pre-released versions of binaries that are useful for testing before making an official release.
This article describes several options if you would like to try out pre-released versions of ROS binaries.

Debian testing repository
-------------------------

For Debian-based operating systems, you can install binary packages from the **ros-testing** repository.

1. Make sure you have a working ROS 2 installation from Debian packages (see :ref:`InstallationGuide`).

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

      sudo apt install ros-rolling-my-just-released-package

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

Fat binaries
------------

For core packages, we run nightly packaging jobs for Linux, macOS, and Windows.
These packaging jobs produce archives with pre-built binaries that can be downloaded and extracted to your filesystem.

1. Make sure you have all dependencies installed according to the `latest development setup <Latest-Development-Setup>` for your platform.

2. Go to https://ci.ros2.org/view/packaging/ and select a packaging job from the list corresponding to your platform.

3. Under the heading "Last Successful Artifacts" you should see a download link (e.g. for Windows, ``ros2-package-windows-AMD64.zip``).

4. Download and extract the archive to your file system.

5. To use the fat binary installation, source the ``setup.*`` file that can be found in the root of the archive.

   .. tabs::

     .. group-tab:: Linux

       .. code-block:: sh

          source path/to/extracted/archive/setup.bash

     .. group-tab:: macOS

       .. code-block:: sh

          source path/to/extracted/archive/setup.bash

     .. group-tab:: Windows

       .. code-block:: sh

          call path\to\extracted\archive\setup.bat

Docker
------

For Linux, there is also a nightly Docker image based on the nightly fat archive.

1. Pull the Docker image:

   .. code-block:: sh

      docker pull osrf/ros2:nightly

2. Start an interactive container:

   .. code-block:: sh

      docker run -it osrf/ros2:nightly

For support on running GUI applications in Docker, take a look at the tutorial `User GUI's with Docker <https://wiki.ros.org/docker/Tutorials/GUI>`_ or the tool `rocker <https://github.com/osrf/rocker>`_.
