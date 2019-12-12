
Building ROS 2 on OS X
======================

.. contents:: Table of Contents
   :depth: 2
   :local:

System requirements
-------------------

We support OS X 10.12.x.

However, some new versions like 10.13.x and some older versions like 10.11.x and 10.10.x are known to work as well.

Install prerequisites
---------------------

You need the following things installed to build ROS 2:


#.
   **Xcode**


   *
     If you don't already have it installed, install Xcode and the Command Line Tools:

     .. code-block:: bash

        xcode-select --install

#.
   **brew** *(needed to install more stuff; you probably already have this)*:


   * Follow installation instructions at http://brew.sh/
   *
     *Optional*: Check that ``brew`` is happy with your system configuration by running:

     .. code-block:: bash

        brew doctor

     Fix any problems that it identifies.

#.
   Use ``brew`` to install more stuff:

   .. code-block:: bash

       brew install cmake cppcheck eigen pcre poco python3 tinyxml wget

       # install dependencies for Fast-RTPS if you are using it
       brew install asio tinyxml2

       brew install opencv

       # install console_bridge for rosbag2
       brew install console_bridge

       # install OpenSSL for DDS-Security
       brew install openssl
       # if you are using ZSH, then replace '.bashrc' with '.zshrc'
       echo "export OPENSSL_ROOT_DIR=$(brew --prefix openssl)" >> ~/.bashrc

       # install dependencies for rcl_logging_log4cxx
       brew install log4cxx

       # install CUnit for CycloneDDS
       brew install cunit

#.
   Install rviz dependencies

   .. code-block:: bash

       # install dependencies for Rviz
       brew install qt freetype assimp

       # Add the Qt directory to the PATH and CMAKE_PREFIX_PATH
       export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/usr/local/opt/qt
       export PATH=$PATH:/usr/local/opt/qt/bin

#.
   Use ``python3 -m pip`` (just ``pip`` may install Python3 or Python2) to install more stuff:

   .. code-block:: bash

       python3 -m pip install -U argcomplete catkin_pkg colcon-common-extensions coverage cryptography empy flake8 flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes ifcfg lark-parser lxml mock mypy netifaces nose pep8 pydocstyle pyparsing pytest-mock setuptools vcstool

   Please ensure that the ``$PATH`` environment variable contains the install location of the binaries (default: ``$HOME/Library/Python/<version>/bin``)

#.
   *Optional*: if you want to build the ROS 1<->2 bridge, then you must also install ROS 1:


   * Start with the normal install instructions: http://wiki.ros.org/kinetic/Installation/OSX/Homebrew/Source
   *
     When you get to the step where you call ``rosinstall_generator`` to get the source code, here's an alternate invocation that brings in just the minimum required to produce a useful bridge:

     .. code-block:: bash

          rosinstall_generator catkin common_msgs roscpp rosmsg --rosdistro kinetic --deps --wet-only --tar > kinetic-ros2-bridge-deps.rosinstall
          wstool init -j8 src kinetic-ros2-bridge-deps.rosinstall


     Otherwise, just follow the normal instructions, then source the resulting ``install_isolated/setup.bash`` before proceeding here to build ROS 2.

Disable System Integrity Protection (SIP)
-----------------------------------------

OS X versions >=10.11 have System Integrity Protection enabled by default.
So that SIP doesn't prevent processes from inheriting dynamic linker environment variables, such as ``DYLD_LIBRARY_PATH``, you'll need to disable it `following these instructions <https://developer.apple.com/library/content/documentation/Security/Conceptual/System_Integrity_Protection_Guide/ConfiguringSystemIntegrityProtection/ConfiguringSystemIntegrityProtection.html>`__.

Get the ROS 2 code
------------------

Create a workspace and clone all repos:

.. code-block:: bash

   mkdir -p ~/ros2_eloquent/src
   cd ~/ros2_eloquent
   wget https://raw.githubusercontent.com/ros2/ros2/eloquent/ros2.repos
   vcs import src < ros2.repos

Install additional DDS vendors (optional)
-----------------------------------------

If you would like to use another DDS or RTPS vendor besides the default, eProsima's Fast RTPS, you can find instructions :ref:`here <dds-osx-source>`.

Build the ROS 2 code
--------------------

**Note**\ : if you are trying to build the ROS 1 <-> ROS 2 bridge, follow instead these `modified instructions <https://github.com/ros2/ros1_bridge/blob/master/README#build-the-bridge-from-source>`__.

Run the ``colcon`` tool to build everything (more on using ``colcon`` in `this tutorial </Tutorials/Colcon-Tutorial>`):

.. code-block:: bash

   cd ~/ros2_eloquent/
   colcon build --symlink-install

Environment setup
-----------------

Source the ROS 2 setup file:

.. code-block:: bash

   . ~/ros2_eloquent/install/setup.bash

This will automatically set up the environment for any DDS vendors that support was built for.

Try some examples
-----------------

In one terminal, set up the ROS 2 environment as described above and then run a ``talker``:

.. code-block:: bash

   ros2 run demo_nodes_cpp talker

In another terminal source the setup file and then run a ``listener``:

.. code-block:: bash

   ros2 run demo_nodes_cpp listener

You should see the ``talker`` saying that it's ``Publishing`` messages and the ``listener`` saying ``I heard`` those messages.
Hooray!

.. _Eloquent_osx-development-setup-troubleshooting:

Troubleshooting
---------------

Troubleshooting techniques can also be found `here </Troubleshooting>`.

Segmentation Fault when using ``pyenv``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``pyenv`` seems to default to building Python with ``.a`` files, but that causes issues with ``rclpy``, so it's recommended to build Python with Frameworks enabled on macOS when using ``pyenv``:

https://github.com/pyenv/pyenv/wiki#how-to-build-cpython-with-framework-support-on-os-x

Library not loaded; image not found
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you are seeing library loading issues at runtime (either running tests or running nodes), such as the following:

.. code-block:: bash

   ImportError: dlopen(.../ros2_eloquent/ros2-osx/lib/python3.7/site-packages/rclpy/_rclpy.cpython-37m-darwin.so, 2): Library not loaded: @rpath/librcl_interfaces__rosidl_typesupport_c.dylib
     Referenced from: .../ros2_eloquent/ros2-osx/lib/python3.7/site-packages/rclpy/_rclpy.cpython-37m-darwin.so
     Reason: image not found

then you probably have System Integrity Protection enabled.
See "Disable System Integrity Protection (SIP)" above for how instructions on how to disable it.

Qt build errors e.g. ``unknown type name 'Q_ENUM'``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you see build errors related to Qt, e.g.:

.. code-block:: bash

   In file included from /usr/local/opt/qt/lib/QtGui.framework/Headers/qguiapplication.h:46:
   /usr/local/opt/qt/lib/QtGui.framework/Headers/qinputmethod.h:87:5: error:
         unknown type name 'Q_ENUM'
       Q_ENUM(Action)
       ^

you may be using qt4 instead of qt5: see https://github.com/ros2/ros2/issues/441

Missing symbol when opencv (and therefore libjpeg, libtiff, and libpng) are installed with Homebrew
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you have opencv installed you might get this:

.. code-block:: bash

   dyld: Symbol not found: __cg_jpeg_resync_to_restart
     Referenced from: /System/Library/Frameworks/ImageIO.framework/Versions/A/ImageIO
     Expected in: /usr/local/lib/libJPEG.dylib
    in /System/Library/Frameworks/ImageIO.framework/Versions/A/ImageIO
   /bin/sh: line 1: 25274 Trace/BPT trap: 5       /usr/local/bin/cmake

If so, to build you'll have to do this:

.. code-block:: bash

   $ brew unlink libpng libtiff libjpeg

But this will break opencv, so you'll also need to update it to continue working:

.. code-block:: bash

   $ sudo install_name_tool -change /usr/local/lib/libjpeg.8.dylib /usr/local/opt/jpeg/lib/libjpeg.8.dylib /usr/local/lib/libopencv_highgui.2.4.dylib
   $ sudo install_name_tool -change /usr/local/lib/libpng16.16.dylib /usr/local/opt/libpng/lib/libpng16.16.dylib /usr/local/lib/libopencv_highgui.2.4.dylib
   $ sudo install_name_tool -change /usr/local/lib/libtiff.5.dylib /usr/local/opt/libtiff/lib/libtiff.5.dylib /usr/local/lib/libopencv_highgui.2.4.dylib
   $ sudo install_name_tool -change /usr/local/lib/libjpeg.8.dylib /usr/local/opt/jpeg/lib/libjpeg.8.dylib /usr/local/Cellar/libtiff/4.0.4/lib/libtiff.5.dylib

The first command is necessary to avoid things built against the system libjpeg (etc.) from getting the version in /usr/local/lib.
The others are updating things built by Homebrew so that they can find the version of libjpeg (etc.) without having them in /usr/local/lib.

Xcode-select error: tool ``xcodebuild`` requires Xcode, but active developer directory is a command line instance
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you recently installed Xcode, you may encounter this error:

.. code-block:: bash

   Xcode: xcode-select: error: tool 'xcodebuild' requires Xcode,
   but active developer directory '/Library/Developer/CommandLineTools' is a command line tools instance

To resolve this error, you will need to:

1. Double check that you have the command line tool installed:

.. code-block:: bash

   $ xcode-select --install

2. Accept the terms and conditions of Xcode by typing in terminal:

.. code-block:: bash

   $ sudo xcodebuild -license accept

3. Ensure Xcode app is in the ``/Applications`` directory (NOT ``/Users/{user}/Applications``)

4. Point ``xcode-select`` to the Xcode app Developer directory using the following command:

.. code-block:: bash

   $ sudo xcode-select -s /Applications/Xcode.app/Contents/Developer

qt_gui_cpp error: SIP binding generator NOT available
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

When building qt_gui_cpp there may be errors look like the following:

.. code-block:: bash

   --- stderr: qt_gui_cpp

   CMake Error at src/CMakeLists.txt:10 (message):
     No Python binding generator found.


   ---
   Failed   <<< qt_gui_cpp [ Exited with code 1 ]

To fix this issue, follow `these steps <../../Tutorials/RQt-Source-Install-MacOS>` to install dependencies for RQt.
