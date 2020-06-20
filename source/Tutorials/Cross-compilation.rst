Cross-compilation
=================

.. contents:: Table of Contents
   :depth: 2
   :local:

Overview
--------

Open Robotics provides pre-built ROS 2 packages for multiple platforms, but a number of developers still rely on `cross-compilation <https://en.wikipedia.org/wiki/Cross_compiler>`__ for different reasons such as:
 - The development machine does not match the target system.
 - Tuning the build for specific core architecture (e.g. setting -mcpu=cortex-a53 -mfpu=neon-fp-armv8 when building for Raspberry Pi3).
 - Targeting a different file systems other than the ones supported by the pre-built images released by Open Robotics.

This document provides you with details on how to cross-compile the ROS 2 software stack as well as provide examples for cross-compiling to systems based on the Arm cores.

How does it work ?
------------------

Cross-compiling simple software (e.g. no dependencies on external libraries) is relatively simple and only requiring a cross-compiler toolchain to be used instead of the native toolchain.

There are a number of factors which make this process more complex:
 - The software being built must support the target architecture. Architecture specific code must be properly isolated and enabled during the build according to the target architecture. Examples include assembly code.
 - All dependencies (e.g. libraries) must be present, either as pre-built packages or also cross-compiled before the target software using them is cross-compiled.
 - When building software stacks (as opposed to an standalone software) using build tools (e.g. colcon), it is expected from the build tool a mechanism to allow the developer to enable cross-compilation on the underlying build system used by each of software in the stack.

Cross-compiling ROS 2
---------------------

The ROS 2 cross-compile tool is under shared ownership of Open Robotics and ROS Tooling Working Group.

It is a Python script that compiles ROS 2 source files for supported target architectures using an emulator in a docker container.
Detailed design of the tool can be found on `ROS 2 design <https://design.ros2.org/articles/cc_build_tools.html>`__.
Instructions to use the tool are in the `cross_compile package <https://github.com/ros-tooling/cross_compile>`__.

If you are using an older version, please follow the `legacy tool instructions`_.

Legacy tool instructions
------------------------

.. note:: Follow the steps below only if you are using the old version (release `0.0.1 <https://github.com/ros-tooling/cross_compile/releases/tag/0.0.1>`__) of the cross-compile tool. For all other purposes, follow the `cross_compile <https://github.com/ros-tooling/cross_compile>`__ package documentation.

Although ROS 2 is a rich software stack with a number of dependencies, it primarily uses two different types of packages:
 - Python based software, which requires no cross-compilation.
 - CMake based software, which provides a mechanism to do cross-compilation.

Furthermore, the ROS 2 software stack is built with `Colcon <https://github.com/colcon/colcon-core>`__ which provides a mechanism to forward parameters to the CMake instance used for the individual build of each package/library that is part of the ROS 2 distribution.

When building ROS 2 natively, the developer is required to download all the dependencies (e.g. Python and other libraries) before compiling the packages that are part of the ROS 2 distribution. When cross-compiling, the same approach is required. The developer must first have the target system's filesystem with all dependencies already installed.

The next sections of this document explain in detail the use of `cmake-toolchains <https://cmake.org/cmake/help/latest/manual/cmake-toolchains.7.html>`__ and the `CMAKE_SYSROOT <https://cmake.org/cmake/help/latest/variable/CMAKE_SYSROOT.html>`__ feature to cross-compile ROS 2.

CMake toolchain-file
^^^^^^^^^^^^^^^^^^^^

A CMake toolchain-file is a file which defines variables to configure CMake for cross-compilation. The basic entries are:

 - ``CMAKE_SYSTEM_NAME``: the target platform, e.g. ``linux``
 - ``CMAKE_SYSTEM_PROCESSOR``: the target architecture, e.g. ``aarch64`` or ``arm``
 - ``CMAKE_SYSROOT``: the path to the target file-system
 - ``CMAKE_C_COMPILER``: the C cross-compiler, e.g. ``aarch64-linux-gnu-gcc``
 - ``CMAKE_CXX_COMPILER``: the C++ cross-compiler, e.g. ``aarch64-linux-gnu-g++``
 - ``CMAKE_FIND_ROOT_PATH``: an alternative path used by the ``find_*`` command to find the file-system

When cross-compiling ROS 2, the following options are required to be set:

 - ``CMAKE_FIND_ROOT_PATH``: the alternative path used by the ``find_*`` command, use it to specify the path to ROS 2 ``/install`` folder
 - ``CMAKE_FIND_ROOT_PATH_MODE_*``: the search strategy for program,package,library, and include, usually: ``NEVER`` (look on the host-fs), ``ONLY`` (look on sysroot), and ``BOTH`` (look on both sysroot and host-fs)
 - ``PYTHON_SOABI``: the index name of the python libraries generated by ROS 2, e.g. ``cpython-36m-aarch64-linux-gnu``
 - ``THREADS_PTHREAD_ARG "0" CACHE STRING "Result from TRY_RUN" FORCE``: Force the result of the ``TRY_RUN`` cmd to 0 (success) because binaries can not run on the host system.

The toolchain-file is provided to CMake with the ``-DCMAKE_TOOLCHAIN_FILE=path/to/file`` parameter. This will also set the ``CMAKE_CROSSCOMPILING`` variable to ``true`` which can be used by the software being built.

The ``CMAKE_SYSROOT`` is particularly important for ROS 2 as the packages need many dependencies (e.g. python, openssl, opencv, poco, eigen3, ...).
Setting ``CMAKE_SYSROOT`` to a target file-system with all the dependencies installed on it will allow CMake to find them during the cross-compilation.

.. note:: You can find more information on the CMake `documentation <https://cmake.org/cmake/help/latest/manual/cmake-toolchains.7.html>`__ page.

When downloading the ROS 2 source code, a generic toolchain-file is available in the repository `ros-tooling/cross_compile/cmake-toolchains <https://github.com/ros-tooling/cross_compile>`__ which can be downloaded separately. Further examples on using it can be found on the `Cross-compiling examples for Arm`_ section.

Target file-system
^^^^^^^^^^^^^^^^^^

As mentioned previously, ROS 2 requires different libraries which needs to be provided to cross-compile.

There are a number of ways to obtain the file-system:
 - downloading a pre-built image
 - installing the dependencies on the target and exporting the file-system (e.g. with sshfs)
 - using qemu + docker (or chroot) to generate the file-system on the host machine.

.. note:: You can find information on how to use Docker + qemu on the next `Cross-compiling examples for Arm`_ section.

Build process
^^^^^^^^^^^^^

The build process is similar to native compilation. The only difference is an extra argument to ``Colcon`` to specify the ``toolchain-file``:

.. code-block:: bash

    colcon build --merge-install \
        --cmake-force-configure \
        --cmake-args \
            -DCMAKE_TOOLCHAIN_FILE="<path_to_toolchain/toolchainfile.cmake>"

The ``toolchain-file`` provide to CMake the information of the ``cross-compiler`` and the ``target file-system``.
``Colcon`` will call CMake with the given toolchain-file on every package of ROS 2.

Cross-compiling examples for Arm
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
After `downloading the ROS 2 source code <../../Installation/Linux-Development-Setup>`__, you can add cross-compilation assets to the workspace via ``git clone https://github.com/ros-tooling/cross_compile.git -b 0.0.1 src/ros2/cross_compile``. These are working examples on how to cross-compile for Arm cores.

The following targets are supported:
 - Ubuntu-arm64: To be used with any ARMv8-A based system.
 - Ubuntu-armhf: To be used with any modern ARMv7-A based system.

These are the main steps:
 - Installing development tools
 - Downloading ROS 2 source code
 - Downloading the ROS 2 cross-compilation assets
 - Preparing the sysroot
 - Cross-compiling the ROS 2 software stack

The next sections explains in detail each of these steps.
For a quick-setup, have a look at the `Automated Cross-compilation`_.

.. note:: These steps were tested on an Ubuntu 18.04 (Bionic)

1. Install development tools
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This step is similar to when building natively. The difference is that some of the libraries and tools are not required because they will be in the sysroot instead.
The following packages are required

.. code-block:: bash

    sudo apt update && sudo apt install -y \
        cmake \
        git \
        wget \
        python3-pip \
        qemu-user-static \
        g++-aarch64-linux-gnu \
        g++-arm-linux-gnueabihf \
        pkg-config-aarch64-linux-gnu

    python3 -m pip install -U \
        vcstool \
        colcon-common-extensions

.. note:: You can install vcstool and colcon-common-extensions via pip. This
          means you are not required to add extra apt repositories.

Docker is used to build the target environment. Follow the official `documentation <https://docs.docker.com/install/linux/docker-ce/ubuntu/>`__ for the installation.

2. Download ROS 2 source code
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Then create a workspace and download the ROS 2 source code:

.. code-block:: bash

    mkdir -p ~/cc_ws/ros2_ws/src
    cd ~/cc_ws/ros2_ws
    wget https://raw.githubusercontent.com/ros2/ros2/release-latest/ros2.repos
    vcs-import src < ros2.repos
    git clone https://github.com/ros-tooling/cross_compile.git -b 0.0.1 src/ros2/cross_compile
    cd ..

3. Prepare the sysroot
~~~~~~~~~~~~~~~~~~~~~~

Build an arm Ubuntu image with all the ROS 2 dependencies using Docker and qemu:
Copy the ``qemu-static`` binary to the workspace.
It will be used to install the ROS 2 dependencies on the target file-system with docker.

.. code-block:: bash

    mkdir qemu-user-static
    cp /usr/bin/qemu-*-static qemu-user-static

The standard `setup <../../Installation/Linux-Development-Setup>`__ process of ROS 2 is run inside an arm docker. This is possible thanks to ``qemu-static``, which will emulate an arm machine. The base image used is an Ubuntu Bionic from Docker Hub.

.. code-block:: bash

    docker build -t arm_ros2:latest -f ros2_ws/src/ros2/cross_compile/sysroot/Dockerfile_ubuntu_arm .
    docker run --name arm_sysroot arm_ros2:latest

Export the resulting container to a tarball and extract it:

.. code-block:: bash

    docker container export -o sysroot_docker.tar arm_sysroot
    mkdir sysroot_docker
    tar -C sysroot_docker -xf sysroot_docker.tar lib usr opt etc
    docker rm arm_sysroot

This container can be used later as virtual target to run the created file-system and run the demo code.

4. Build
~~~~~~~~

Set the variables used by the generic toolchain-file

.. code-block:: bash

    export TARGET_ARCH=aarch64
    export TARGET_TRIPLE=aarch64-linux-gnu
    export CC=/usr/bin/$TARGET_TRIPLE-gcc
    export CXX=/usr/bin/$TARGET_TRIPLE-g++
    export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-
    export SYSROOT=~/cc_ws/sysroot_docker
    export ROS2_INSTALL_PATH=~/cc_ws/ros2_ws/install
    export PYTHON_SOABI=cpython-36m-$TARGET_TRIPLE

The following packages still cause errors during the cross-compilation (under investigation) and must be disabled for now.

.. code-block:: bash

    touch \
        ros2_ws/src/ros2/rviz/COLCON_IGNORE \
        ros2_ws/src/ros-visualization/COLCON_IGNORE

The ``Poco`` pre-built has a known issue where it is searching for ``libz`` and ``libpcre`` on the host system instead of SYSROOT.
As a workaround for the moment, please link both libraries into the the host's file-system.

.. code-block:: bash

    mkdir -p /usr/lib/$TARGET_TRIPLE
    ln -s `pwd`/sysroot_docker/lib/$TARGET_TRIPLE/libz.so.1 /usr/lib/$TARGET_TRIPLE/libz.so
    ln -s `pwd`/sysroot_docker/lib/$TARGET_TRIPLE/libpcre.so.3 /usr/lib/$TARGET_TRIPLE/libpcre.so

Then, start a build with colcon specifying the toolchain-file:

.. code-block:: bash

    cd ros2_ws

    colcon build --merge-install \
        --cmake-force-configure \
        --cmake-args \
            -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON \
            -DCMAKE_TOOLCHAIN_FILE="$(pwd)/src/ros2/cross_compile/cmake-toolchains/generic_linux.cmake" \
            -DSECURITY=ON

Done! The install and build directories will contain the cross-compiled assets.

Automated Cross-compilation
^^^^^^^^^^^^^^^^^^^^^^^^^^^

All the steps above are also included into a Dockerfile and can be used for automation/CI.

First, download the dockerfile and build the image:

.. code-block:: bash

    wget https://raw.githubusercontent.com/ros-tooling/cross_compile/master/Dockerfile_cc_for_arm
    docker build -t ros2-crosscompiler:latest - < Dockerfile_cc_for_arm

Now run the image with:
(it will take a while !)

.. code-block:: bash

    docker run -it --name ros2_cc \
        -v /var/run/docker.sock:/var/run/docker.sock \
        ros2-crosscompiler:latest

..note:: The -v /var/run/docker.sock allow us to use Docker inside Docker.

The result of the build will be inside the ``ros2_ws`` directory, which can be exported with:

.. code-block:: bash

    docker cp ros2_cc:/root/cc_ws/ros2_ws .

Cross-compiling against a pre-built ROS 2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

It is possible to cross-compile your packages against a pre-built ROS 2. The steps are similar to the previous `Cross-compiling examples for Arm`_ section, with the following modifications:

Instead of downloading the ROS 2 stack, just populate your workspace with your package (ros2 examples on this case) and the cross-compilation assets:

.. code-block:: bash

    mkdir -p ~/cc_ws/ros2_ws/src
    cd ~/cc_ws/ros2_ws/src
    git clone https://github.com/ros2/examples.git
    git clone https://github.com/ros-tooling/cross_compile.git -b 0.0.1
    cd ..

Generate and export the file-system as described in `3. Prepare the sysroot`_, but with the provided ``Dockerfile_ubuntu_arm64_prebuilt``. These ``_prebuilt`` Dockerfile will use the `binary packages <https://index.ros.org/doc/ros2/Linux-Install-Debians/>`__ to install ROS 2 instead of building from source.

Modify the environment variable ``ROS2_INSTALL_PATH`` to point to the installation directory:

.. code-block:: bash

    export ROS2_INSTALL_PATH=~/cc_ws/sysroot_docker/opt/ros/crystal

Source the ``setup.bash`` script on the target file-system:

.. code-block:: bash

    source $ROS2_INSTALL_PATH/setup.bash

Then, start a build with ``Colcon`` specifying the ``toolchain-file``:

.. code-block:: bash

    colcon build \
        --merge-install \
        --cmake-force-configure \
        --cmake-args \
            -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON \
            -DCMAKE_TOOLCHAIN_FILE="$(pwd)/src/cross_compile/cmake-toolchains/generic_linux.cmake"

Run on the target
^^^^^^^^^^^^^^^^^

Copy the file-system on your target or use the previously built docker image:

.. code-block:: bash

    docker run -it --rm -v `pwd`/ros2_ws:/ros2_ws arm_ros2:latest

Source the environment:

.. code-block:: bash

    source /ros2_ws/install/local_setup.bash

Run some of the C++ or python examples:

.. code-block:: bash

    ros2 run demo_nodes_cpp listener &
    ros2 run demo_nodes_py talker
