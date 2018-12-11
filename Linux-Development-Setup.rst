
Building ROS 2 on Linux
=======================

System Requirements
-------------------

Target platforms for Bouncy Bolson (see [REP](http://www.ros.org/reps/rep-2000.html)).
- Ubuntu Linux Xenial Xerus 16.04 64-bit
- Ubuntu Linux Bionic Beaver 18.04 64-bit

Recommended Support (not actively tested or supported)
- Debian Stretch
- Fedora 26, see `alternate instructions <Fedora-Development-Setup>`
- Arch Linux, see `alternate instructions <https://wiki.archlinux.org/index.php/Ros#Ros_2>`__.

Make sure that you have a locale set which supports ``UTF-8`` We test with the following settings.
If you are in a minimal environment such as a docker containers the locale may be set to something minimal like POSIX.
To set the locale an example is below. It should be fine if you're using a different UTF-8 supported locale.

.. code-block:: bash

   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   export LANG=en_US.UTF-8

System setup
------------

.. _linux-dev-add-ros2-repo:

Add the ROS 2 apt repository
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

First make sure you have the ROS 2 apt repositories added to your system, if not refer to the Setup Sources section of `this guide <linux-install-debians-setup-sources>`

Install development tools and ROS tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   sudo apt update && sudo apt install -y \
     build-essential \
     cmake \
     git \
     python3-colcon-common-extensions \
     python3-pip \
     python-rosdep \
     python3-vcstool \
     wget
   # install some pip packages needed for testing
   sudo -H python3 -m pip install -U \
     argcomplete \
     flake8 \
     flake8-blind-except \
     flake8-builtins \
     flake8-class-newline \
     flake8-comprehensions \
     flake8-deprecated \
     flake8-docstrings \
     flake8-import-order \
     flake8-quotes \
     git+https://github.com/lark-parser/lark.git@0.7b \
     pytest-repeat \
     pytest-rerunfailures
   # [Ubuntu 16.04] install extra packages not available or recent enough on Xenial
   python3 -m pip install -U \
     pytest \
     pytest-cov \
     pytest-runner \
     setuptools
   # install Fast-RTPS dependencies
   sudo apt install --no-install-recommends -y \
     libasio-dev \
     libtinyxml2-dev

.. _linux-dev-get-ros2-code:

Get ROS 2.0 code
^^^^^^^^^^^^^^^^

Create a workspace and clone all repos:

.. code-block:: bash

   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   wget https://raw.githubusercontent.com/ros2/ros2/release-latest/ros2.repos
   vcs import src < ros2.repos

..

   Note: if you want to get all of the latest bug fixes then you can try the "tip" of development by replacing ``release-latest`` in the URL above with ``master``. The ``release-latest`` is preferred by default because it goes through more rigorous testing on release than changes to master do. See also `Maintaining a Source Checkout <Maintaining-a-Source-Checkout>`.


Install dependencies using rosdep
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   sudo rosdep init
   rosdep update
   rosdep install --from-paths src --ignore-src --rosdistro bouncy -y --skip-keys "console_bridge fastcdr fastrtps libopensplice69 rti-connext-dds-5.3.1 urdfdom_headers"

.. _linux-development-setup-install-more-dds-implementations-optional:

Install more DDS implementations (Optional)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

ROS 2.0 builds on top of DDS.
It is compatible with multiple DDS or RTPS (the DDS wire protocol) vendors.
The repositories you downloaded for ROS 2.0 includes eProsima's Fast RTPS, which is the only bundled vendor.
If you would like to use one of the other vendors you will need to install their software separately before building.
The ROS 2.0 build will automatically build support for vendors that have been installed and sourced correctly.

By default we include eProsima's FastRTPS in the workspace and it is the default middleware. Detailed instructions for installing other DDS vendors are provided below.

PrismTech OpenSplice Debian Packages built by OSRF
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   sudo apt install libopensplice69  # from repo.ros2.org

Add this to your ``~/.bashrc``

.. code-block:: bash

   export OSPL_URI=file:///usr/etc/opensplice/config/ospl.xml


.. raw:: html

   <!--
   ##### Official binary packages from PrismTech

   Install the packages provided by [OpenSplice](https://github.com/ADLINK-IST/opensplice/releases/tag/OSPL_V6_7_180404OSS_RELEASE%2BVS2017%2Bubuntu1804).
   Remember to replace `@@INSTALLDIR@@` with the path where you unpacked the OpenSplice distribution.
   Then, source the ROS `setup.bash` file, and finally, source the `release.com` file in the root of the OpenSplice distribution to set the `OSPL_HOME` environment variable appropriately.
   After that, your shell is ready to run ROS2 binaries with the official OpenSplice distribution.

   You may also need to add the following line to your `.bashrc` file:

   ```
   export PTECH_LICENSE_FILE=path/to/prismtech.lic
   ```

   ##### Building OpenSplice from source

   If you build OpenSplice from source, be sure to remember to following the INSTALL.txt instructions and manually replace the @@INSTALLDIR@@ placeholder in the OpenSplice install/HDE/x86_64.linux/release.com
   -->



RTI Connext (version 5.3.1)
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Debian packages provided in the ROS 2 apt repositories
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can install a Debian package of RTI Connext available on the ROS 2 apt repositories.
You will need to accept a license from RTI.

.. code-block:: bash

   sudo apt install -q -y \
       rti-connext-dds-5.3.1  # from repo.ros2.org

Source the setup file to set the ``NDDSHOME`` environment variable.

.. code-block:: bash

   cd /opt/rti.com/rti_connext_dds-5.3.1/resource/scripts && source ./rtisetenv_x64Linux3gcc5.4.0.bash; cd -

Note: when using ``zsh`` you need to be in the directory of the script when sourcing it to have it work properly

Now you can build as normal and support for RTI will be built as well.

If you want to install the Connext DDS-Security plugins please refer to `this page <Install-Connext-Security-Plugins>`

Official binary packages from RTI
"""""""""""""""""""""""""""""""""

You can install the Connext 5.3.1 package for Linux provided by RTI from their `downloads page <https://www.rti.com/downloads>`__.

To use RTI Connext you will need to have obtained a license from RTI.
Add the following line to your ``.bashrc`` file pointing to your copy of the license.

.. code-block:: bash

   export RTI_LICENSE_FILE=path/to/rti_license.dat

After downloading, use ``chmod +x`` on the ``.run`` executable and then execute it.
Note that if you're installing to a system directory use ``sudo`` as well.

The default location is ``~/rti_connext_dds-5.3.1``

Source the setup file to set the ``NDDSHOME`` environment variable.

.. code-block:: bash

   source ~/rti_connext_dds-5.3.1/resource/scripts/rtisetenv_x64Linux3gcc5.4.0.bash

Now you can build as normal and support for RTI will be built as well.

Build the code in the workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Note: to build the ROS 1 bridge, read the `ros1_bridge instructions <https://github.com/ros2/ros1_bridge/blob/master/README#build-the-bridge-from-source>`__.

More info on working with a ROS workspace can be found in `this tutorial <Colcon-Tutorial>`.

.. code-block:: bash

   cd ~/ros2_ws/
   colcon build --symlink-install

Note: if you are having trouble compiling all examples and this is preventing you from completing a successful build, you can use ``AMENT_IGNORE`` in the same manner as `CATKIN_IGNORE <https://github.com/ros-infrastructure/rep/blob/master/rep-0128.rst>`__ to ignore the subtree or remove the folder from the workspace.
Take for instance: you would like to avoid installing the large OpenCV library.
Well then simply ``$ touch AMENT_IGNORE`` in the ``cam2image`` demo directory to leave it out of the build process.

Optionally install all packages into a combined directory (rather than each package in a separate subdirectory).
On Windows due to limitations of the length of environment variables you should use this option when building workspaces with many (~ >> 100 packages).

Also, if you have already installed ROS2 from Debian make sure that you run the ``build`` command in a fresh environment. You may want to make sure that you do not have ``source /opt/ros/${ROS_DISTRO}/setup.bash`` in your ``.bashrc``.


.. code-block:: bash

   colcon build --symlink-install --merge-install

Afterwards source the ``local_setup.*`` from the ``install`` folder.

Try some examples
^^^^^^^^^^^^^^^^^

In one terminal, source the setup file and then run a ``talker``\ :

.. code-block:: bash

   . ~/ros2_ws/install/local_setup.bash
   ros2 run demo_nodes_cpp talker

In another terminal source the setup file and then run a ``listener``\ :

.. code-block:: bash

   . ~/ros2_ws/install/local_setup.bash
   ros2 run demo_nodes_py listener

You should see the ``talker`` saying that it's ``Publishing`` messages and the ``listener`` saying ``I heard`` those messages.
Hooray!

See the `demos <Tutorials>` for other things to try.

Alternate compilers
-------------------

Using a different compiler besides gcc to compile ROS 2 is easy. If you set the environment variables ``CC`` and ``CXX`` to executables for a working C and C++ compiler, respectively, and retrigger CMake configuration (by using ``--force-cmake-config`` or by deleting the packages you want to be affected), CMake will reconfigure and use the different compiler.

Clang
^^^^^

To configure CMake to detect and use Clang:

.. code-block:: bash

   sudo apt install clang
   export CC=clang
   export CXX=clang++
   colcon build --cmake-force-configure

TODO: using ThreadSanitizer, MemorySanitizer

Troubleshooting
---------------

Internal compiler error
^^^^^^^^^^^^^^^^^^^^^^^

If you experience an ICE when trying to compile on a memory constrained platform like a Raspberry PI you might want to build single threaded (prefix the build invocation with ``MAKEFLAGS=-j1``\ ).

Out of memory
^^^^^^^^^^^^^

The ``ros1_bridge`` in its current form requires 4Gb of free RAM to compile.
If you don't have that amount of RAM available it's suggested to use ``AMENT_IGNORE`` in that folder and skip its compilation.

Multiple Host Interference
^^^^^^^^^^^^^^^^^^^^^^^^^^

If you're running multiple instances on the same network you may get interference.
To avoid this you can set the environment variable ``ROS_DOMAIN_ID`` to a different integer, the default is zero.
This will define the DDS domain id for your system.
Note that if you are using the OpenSplice DDS implementation you will also need to update the OpenSplice configuration file accordingly. The location of the configuration file is referenced in the ``OSPL_URI`` environment variable.
