.. redirect-from::

  Installation/Rolling/OSX-Development-Setup
  Installation/macOS-Development-Setup

macOS (source)
==============

.. contents:: Table of Contents
   :depth: 2
   :local:

System requirements
-------------------

We currently support macOS Sonoma (14.5) with Apple Silicon (e.g. M1) Mac

System setup
------------

Install prerequisites
^^^^^^^^^^^^^^^^^^^^^

You need the following things installed to build ROS 2:


#.
   **Xcode**

   * If you don't already have it installed, install [Xcode](https://apps.apple.com/app/xcode/id497799835).
   * Note: Versions of Xcode later than 11.3.1 can no longer be installed on macOS Mojave, so you will need to install an older version manually, see: https://stackoverflow.com/a/61046761
   * Also, if you don't already have it installed, install the Command Line Tools:

     .. code-block:: bash

        xcode-select --install
        # This command will not succeed if you have not installed Xcode.app
        sudo xcode-select --switch /Applications/Xcode.app/Contents/Developer
        # If you installed Xcode.app manually, you need to either open it or run:
        sudo xcodebuild -license
        # To accept the Xcode.app license

#.
   **brew** *(needed to install dependencies; you probably already have this)*:


   * Follow installation instructions at http://brew.sh/.

   *
     Check that ``brew`` is set not to use ``Rosseta`` (we don't want emulation of x86_64 binaries):

     following code will show configuration of current brew installation. Make sure ``Rosseta 2: false``.

     .. code-block:: bash

        brew config

     If not, remove one that is using Rosseta (it's installed at ``/usr/local``)

     Use the uninstall script by Homebrew (https://github.com/homebrew/install?tab=readme-ov-file#uninstall-homebrew)

     Make sure you add ``--path /usr/local`` at the end of the uninstall command to remove the Rosseta installation.

#.
   Use ``brew`` to install dependencies:

   First, set environment for brew:

   .. code-block:: bash

      echo 'eval "$(/opt/homebrew/bin/brew shellenv)"' >> ~/.zprofile
      eval "$(/opt/homebrew/bin/brew shellenv)"

   .. code-block:: bash

      brew install asio assimp bison bullet cmake console_bridge cppcheck \
         cunit eigen freetype graphviz opencv openssl orocos-kdl pcre poco \
         pyqt@5 python@3.11 qt@5 sip spdlog tinyxml tinyxml2

#.
   Setup some environment variables:

   .. code-block:: bash

      # Add the openssl dir for DDS-Security
      export OPENSSL_ROOT_DIR=$(brew --prefix openssl)"

      # Add the Qt directory to the PATH and CMAKE_PREFIX_PATH
      export CMAKE_PREFIX_PATH=$(brew --prefix qt@5):$(brew --prefix qt@5)/lib:/opt/homebrew/opt:$(brew --prefix)/lib
      export PATH=$PATH:$(brew --prefix qt@5)/bin

      # Disable notification during compile
      export COLCON_EXTENSION_BLOCKLIST=colcon_core.event_handler.desktop_notification

#.
   Check that you have python3.11 installed (we've installed with Brew above):

   .. code-block:: bash

      python3 --version
      # it should print 3.11.x

   Create a virtual environment for Python3.11:

   .. code-block:: bash

      # Generate python virtual environment
      python3 -m venv ~/.ros2_venv
      # Activate python3.11 virtual environment
      source ~/.ros2_venv/bin/activate
      # now you will see (.ros2_venv) in your terminal prompt

   Use ``python3 -m pip`` to install more stuff:

   .. code-block:: bash

      # make sure you see (.ros2_venv) in your terminal prompt
      python3 -m pip install --upgrade pip

      python3 -m pip install -U \
         argcomplete catkin_pkg colcon-common-extensions coverage \
         cryptography empy flake8 flake8-blind-except==0.1.1 flake8-builtins \
         flake8-class-newline flake8-comprehensions flake8-deprecated \
         flake8-docstrings flake8-import-order flake8-quotes \
         importlib-metadata jsonschema lark==1.1.1 lxml matplotlib mock mypy==0.931 netifaces \
         nose pep8 psutil pydocstyle pydot pyparsing==2.4.7 \
         pytest-mock rosdep rosdistro setuptools==59.6.0 vcstool

      python3 -m pip install \
         --config-settings="--global-option=build_ext" \
         --config-settings="--global-option=-I/opt/homebrew/opt/graphviz/include/" \
         --config-settings="--global-option=-L/opt/homebrew/opt/graphviz/lib/" \
         pygraphviz

   Please ensure that the ``$PATH`` environment variable contains the install location of the binaries (``$(brew --prefix)/bin``)


Build ROS 2
-----------

Get ROS 2 code
^^^^^^^^^^^^^^

Create a workspace and clone all repos (cloning the eact release version tag is required for patches):

.. code-block:: bash

   mkdir -p ~/ros2_{DISTRO}/src
   cd ~/ros2_{DISTRO}
   export JAZZY_RELEASE_TAG=release-jazzy-20240523
   vcs import --force --shallow --retry 0 --input https://raw.githubusercontent.com/ros2/ros2/$JAZZY_RELEASE_TAG/ros2.repos src

Patch Files for macOS Installation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

First compile up to ``cyclonedds`` to generate compile structure and apply patches,

.. code-block:: bash

   python3 -m colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF -Wno-dev \
             -Wno-sign-conversion -Wno-infinite-recursion \
             --packages-skip-by-dep python_qt_binding --packages-up-to cyclonedds \
             --event-handlers console_cohesion+

Apply patches,

.. code-block:: bash

   # patch for cyclonedds
   ln -s "../../iceoryx_posh/lib/libiceoryx_posh.dylib" install/iceoryx_binding_c/lib/libiceoryx_posh.dylib
   ln -s "../../iceoryx_hoofs/lib/libiceoryx_hoofs.dylib" install/iceoryx_binding_c/lib/libiceoryx_hoofs.dylib
   ln -s "../../iceoryx_hoofs/lib/libiceoryx_platform.dylib" install/iceoryx_binding_c/lib/libiceoryx_platform.dylib

   # Patch for orocos-kdl(to use ones that of brew)
   curl -sSL https://raw.githubusercontent.com/IOES-Lab/ROS2_Jazzy_MacOS_Native_AppleSilicon/main/patches/geometry2_tf2_eigen_kdl.patch | patch -p1 -Ns
   curl -sSL https://raw.githubusercontent.com/IOES-Lab/ROS2_Jazzy_MacOS_Native_AppleSilicon/main/patches/ros_visualization_interactive_markers.patch | patch -p1 -Ns
   curl -sSL https://raw.githubusercontent.com/IOES-Lab/ROS2_Jazzy_MacOS_Native_AppleSilicon/main/patches/kdl_parser.patch | patch -p1 -Ns

   # patch for rviz_ogre_vendor
   curl -sSL https://raw.githubusercontent.com/IOES-Lab/ROS2_Jazzy_MacOS_Native_AppleSilicon/main/patches/rviz_default_plugins.patch | patch -p1 -Ns
   curl -sSL https://raw.githubusercontent.com/IOES-Lab/ROS2_Jazzy_MacOS_Native_AppleSilicon/main/patches/rviz_ogre_vendor.patch | patch -p1 -Ns
   curl -sSL https://raw.githubusercontent.com/IOES-Lab/ROS2_Jazzy_MacOS_Native_AppleSilicon/main/patches/0001-pragma.patch | patch -p1 -Ns

   # patch for rosbag2_transport
   curl -sSL https://raw.githubusercontent.com/IOES-Lab/ROS2_Jazzy_MacOS_Native_AppleSilicon/main/patches/rosbag2_transport.patch | patch -p1 -Ns

   # Fix brew linking of qt@5 (one from brew installation)
   brew unlink qt && brew link qt@5

   # Revert python_orocos_kdl_vendor back to 0.4.1
   rm -rf src/ros2/orocos_kdl_vendor
   git clone https://github.com/ros2/orocos_kdl_vendor.git src/ros2/orocos_kdl_vendor
   ( cd ./src/ros2/orocos_kdl_vendor/python_orocos_kdl_vendor || exit; git checkout 0.4.1 )

   # Remove eclipse-cyclonedds (this doesn't work)
   rm -rf src/eclipse-cyclonedds


Build the code in the workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Run the ``colcon`` tool to build everything (more on using ``colcon`` in :doc:`this tutorial <../../Tutorials/Beginner-Client-Libraries/Colcon-Tutorial>`):

.. code-block:: bash

   cd ~/ros2_{DISTRO}/
   python3.11 -m colcon build  --symlink-install \
      --packages-skip-by-dep python_qt_binding \
      --cmake-args \
      --no-warn-unused-cli \
      -DBUILD_TESTING=OFF \
      -DINSTALL_EXAMPLES=ON \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_OSX_SYSROOT=/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk \
      -DCMAKE_OSX_ARCHITECTURES="arm64" \
      -DPython3_EXECUTABLE="$HOME/$VIRTUAL_ENV_ROOT/bin/python3" \
      -Wno-dev --event-handlers console_cohesion+

Note: due to an unresolved issue with SIP, Qt@5, and PyQt5, we need to disable ``python_qt_binding`` to have the build succeed.
This will be removed when the issue is resolved, see: https://github.com/ros-visualization/python_qt_binding/issues/103

Setup environment
-----------------

Source the ROS 2 setup file:

.. code-block:: bash

   # one-time activator
   . ~/ros2_{DISTRO}/install/setup.zsh
   # alias creation for future use
   echo "alias ros='source ~/ros2_{DISTRO}/install/setup.zsh'" >> ~/.zprofile
   # if you want it as default (always)
   echo "source ~/ros2_{DISTRO}/install/setup.zsh" >> ~/.zprofile

This will automatically set up the environment for any DDS vendors that support was built for.

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

The Rviz,

.. code-block:: bash

   rviz2


Next steps
----------

Continue with the `tutorials and demos <../../Tutorials>` to configure your environment, create your own workspace and packages, and learn ROS 2 core concepts.

Use the ROS 1 bridge (optional)
-------------------------------

The ROS 1 bridge can connect topics from ROS 1 to ROS 2 and vice-versa.
See the dedicated `documentation <https://github.com/ros2/ros1_bridge/blob/master/README.md>`__ on how to build and use the ROS 1 bridge.

Stay up to date
---------------

See :doc:`../Maintaining-a-Source-Checkout` to periodically refresh your source installation.

Troubleshoot
------------

Troubleshooting techniques can be found :ref:`here <macOS-troubleshooting>`.

Uninstall
---------

1. If you installed your workspace with colcon as instructed above, "uninstalling" could be just a matter of opening a new terminal and not sourcing the workspace's ``setup`` file.
   This way, your environment will behave as though there is no {DISTRO_TITLE} install on your system.

2. If you're also trying to free up space, you can delete the entire workspace directory with:

   .. code-block:: bash

      rm -rf ~/ros2_{DISTRO}
