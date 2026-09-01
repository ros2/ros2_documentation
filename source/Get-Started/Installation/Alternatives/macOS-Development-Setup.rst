.. redirect-from::

  Installation/Rolling/OSX-Development-Setup
  Installation/macOS-Development-Setup
  Installation/Alternatives/macOS-Development-Setup

macOS (source)
==============

.. contents:: Table of Contents
   :depth: 2
   :local:

This page explains how to setup a development environment for ROS 2 on macOS.

System requirements
-------------------

Disable System Integrity Protection (SIP)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

macOS has System Integrity Protection enabled by default,
which prevents processes from inheriting dynamic linker environment variables, such as ``DYLD_LIBRARY_PATH``.
You can disable it in the `macOS Recovery <https://support.apple.com/en-gb/102518>`__.
After entering macOS Recovery, run the following command in terminal:

.. code-block:: console

   $ csrutil disable

*Optional*: Check the status of SIP:

.. code-block:: console

   $ csrutil status

Install prerequisites
---------------------

Install Xcode
^^^^^^^^^^^^^

In order to compile the ROS 2 code, the Xcode must be installed. You can download it from the `App Store <https://apps.apple.com/app/xcode/id497799835/>`_.

Also, use the following command to install the Command Line Tools:

.. code-block:: console

   $ xcode-select --install
   $ sudo xcode-select --switch /Applications/Xcode.app/Contents/Developer

.. note::

      If you installed Xcode manually, you need to accept the license.
      You can do this by opening Xcode or running:

      .. code-block:: console

         $ sudo xcodebuild -license accept

Install Homebrew
^^^^^^^^^^^^^^^^

Homebrew is a package manager for macOS, and some dependencies are shipped with it. Use the instructions on `http://brew.sh/ <http://brew.sh/>`_ to install it.

*Optional*: Check that ``brew`` is happy with your system configuration by running the command below and fixing any identified problems:

.. code-block:: console

   $ brew doctor

Install Python
^^^^^^^^^^^^^^

Download and install Python 3.14 from the `Python website <https://www.python.org/downloads/latest/python3.14/>`_.

Also, install the certificates:

.. code-block:: console

   $ cd "/Applications/Python 3.14"
   $ "./Install Certificates.command"

Install CMake
^^^^^^^^^^^^^

Since Homebrew no longer ships CMake 3, you can download it from the Legacy Releases section on the `CMake website <https://cmake.org/download/>`_.

Also, run this command to enable CMake from the command line:

.. code-block:: console

   $ sudo "/Applications/CMake.app/Contents/bin/cmake-gui" --install

Install dependencies
^^^^^^^^^^^^^^^^^^^^

Homebrew:

.. code-block:: console

   $ brew install \
     asio \
     assimp \
     bison \
     bullet \
     console_bridge \
     cppcheck \
     cunit \
     eigen@3 \
     freetype \
     graphviz \
     googletest \
     libyaml \
     opencv \
     openssl \
     orocos-kdl \
     pcre \
     pybind11 \
     pyqt@6 \
     qt@6 \
     rust \
     sip \
     spdlog \
     tinyxml2 \
     yaml-cpp

Unlink Python in Homebrew to ensure the correct Python version is used:

.. code-block:: console

   $ brew unlink python

Check the Python version being used:

.. code-block:: console

   $ which python3

PyPI:

.. code-block:: console

   $ python3 -m pip install --upgrade pip
   $ python3 -m pip install -U \
     argcomplete \
     catkin_pkg \
     colcon-common-extensions \
     cryptography \
     flake8 \
     flake8-blind-except \
     flake8-builtins \
     flake8-class-newline \
     flake8-comprehensions \
     flake8-deprecated \
     flake8-docstrings \
     flake8-import-order \
     flake8-quotes \
     lark \
     lxml \
     matplotlib \
     mock \
     mypy \
     psutil \
     PySide6 \
     rosdep \
     rosdistro \
     setuptools \
     vcstool

Build ROS 2
-----------

Get ROS 2 code
^^^^^^^^^^^^^^

.. code-block:: console

   $ mkdir -p ~/ros2_{DISTRO}/src
   $ cd ~/ros2_{DISTRO}
   $ vcs import --input https://raw.githubusercontent.com/ros2/ros2/{REPOS_FILE_BRANCH}/ros2.repos src

Configure system
^^^^^^^^^^^^^^^^

Setup environment variables:

.. code-block:: console

   ~ Add the openssl dir for DDS-Security
   ~ if you are using BASH, then replace '.zshrc' with '.bashrc'
   $ echo "export OPENSSL_ROOT_DIR=$(brew --prefix openssl)" >> ~/.zshrc

   ~ Add the Qt directory to the PATH and CMAKE_PREFIX_PATH
   $ export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:$(brew --prefix qt@6)
   $ export PATH=$PATH:$(brew --prefix qt@6)/bin

Since Homebrew installs Eigen in a different directory, create a symbolic link to the folder expected by ROS 2:

.. code-block:: console

   $ sudo ln -sfn /opt/homebrew/opt/eigen@3/include/eigen3 /opt/homebrew/include/eigen3

If you are using Xcode 26 or later,
apply the following Git patch to ensure the correct flags are set when building ``rviz_ogre_vendor``:

.. code-block:: console

   $ cd ~/ros2_{DISTRO}/src/ros2/rviz/rviz_ogre_vendor
   $ git apply patches/0007-fix-xcodebuild-n-xcode26.patch


Install additional RMW implementations (optional)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The default middleware that ROS 2 uses is ``Fast DDS``, but the middleware (RMW) can be replaced at build or runtime.
See the :doc:`guide <../RMW-Implementations/Working-with-multiple-RMW-implementations>` on how to work with multiple RMWs.

Build the code in the workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: console

   $ cd ~/ros2_{DISTRO}/
   $ python3 -m colcon build --symlink-install --packages-ignore qt_gui_cpp rqt_gui_cpp

.. note::

   Due to an unresolved issue with Qt and PyQt, we need to ignore ``qt_gui_cpp`` and ``rqt_gui_cpp`` to have the build succeed.
   This will be removed when the issue is resolved, see: https://github.com/ros-visualization/python_qt_binding/issues/103

.. note::

   The ``python_orocos_kdl_vendor`` package requires the exact version of ``orocos-kdl`` specified in the package.
   If Homebrew ships a different version, you can ignore this package to allow the build to succeed.

Setup environment
-----------------

Set up your environment by sourcing the following file.:

.. code-block:: console

   $ . ~/ros2_{DISTRO}/install/setup.zsh

Try some examples
-----------------

In one terminal, source the setup file and then run a C++ ``talker``:

.. code-block:: console

   $ . ~/ros2_{DISTRO}/install/setup.zsh
   $ ros2 run demo_nodes_cpp talker

In another terminal source the setup file and then run a Python ``listener``:

.. code-block:: console

   $ . ~/ros2_{DISTRO}/install/setup.zsh
   $ ros2 run demo_nodes_py listener

You should see the ``talker`` saying that it's ``Publishing`` messages and the ``listener`` saying ``I heard`` those messages.
This verifies both the C++ and Python APIs are working properly.
Hooray!

Next steps
----------

Continue with the tutorials to configure your environment, create your own workspace and packages, and learn ROS 2 core concepts.

Stay up to date
---------------

See :doc:`Maintaining-a-Source-Checkout` to periodically refresh your source installation.

Troubleshoot
------------

Troubleshooting techniques can be found :ref:`here <macOS-troubleshooting>`.

Uninstall
---------

1. If you installed your workspace with colcon as instructed above, "uninstalling" could be just a matter of opening a new terminal and not sourcing the workspace's ``setup`` file.
   This way, your environment will behave as though there is no {DISTRO_TITLE} install on your system.

2. If you're also trying to free up space, you can delete the entire workspace directory with:

   .. code-block:: console

      $ rm -rf ~/ros2_{DISTRO}
