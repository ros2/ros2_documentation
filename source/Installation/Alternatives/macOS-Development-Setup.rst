.. _macOS-latest:

macOS (source)
==============

.. contents:: Table of Contents
   :depth: 2
   :local:

System requirements
-------------------

We currently support macOS Mojave (10.14).

Install prerequisites
---------------------

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

       brew install asio assimp bison bullet cmake console_bridge cppcheck \
         cunit eigen freetype graphviz opencv openssl orocos-kdl pcre poco \
         pyqt5 python qt@5 sip spdlog tinyxml tinyxml2

#.
   Setup some environment variables:

   .. code-block:: bash

      # Add the openssl dir for DDS-Security
      # if you are using BASH, then replace '.zshrc' with '.bashrc'
      echo "export OPENSSL_ROOT_DIR=$(brew --prefix openssl)" >> ~/.zshrc

      # Add the Qt directory to the PATH and CMAKE_PREFIX_PATH
      export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:$(brew --prefix qt@5)
      export PATH=$PATH:$(brew --prefix qt@5)/bin

#.
   Use ``python3 -m pip`` (just ``pip`` may install Python3 or Python2) to install more stuff:

   .. code-block:: bash

       python3 -m pip install -U \
        argcomplete catkin_pkg colcon-common-extensions coverage \
        cryptography empy flake8 flake8-blind-except==0.1.1 flake8-builtins \
        flake8-class-newline flake8-comprehensions flake8-deprecated \
        flake8-docstrings flake8-import-order flake8-quotes \
        importlib-metadata lark==1.1.1 lxml matplotlib mock mypy==0.931 netifaces \
        nose pep8 psutil pydocstyle pydot pygraphviz pyparsing==2.4.7 \
        pytest-mock rosdep rosdistro setuptools==59.6.0 vcstool

   Please ensure that the ``$PATH`` environment variable contains the install location of the binaries (``$(brew --prefix)/bin``)

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

macOS/OS X versions >=10.11 have System Integrity Protection enabled by default.
So that SIP doesn't prevent processes from inheriting dynamic linker environment variables, such as ``DYLD_LIBRARY_PATH``, you'll need to disable it `following these instructions <https://developer.apple.com/library/content/documentation/Security/Conceptual/System_Integrity_Protection_Guide/ConfiguringSystemIntegrityProtection/ConfiguringSystemIntegrityProtection.html>`__.

Get the ROS 2 code
------------------

Create a workspace and clone all repos:

.. code-block:: bash

   mkdir -p ~/ros2_{DISTRO}/src
   cd ~/ros2_{DISTRO}
   vcs import --input https://raw.githubusercontent.com/ros2/ros2/{REPOS_FILE_BRANCH}/ros2.repos src

Install additional DDS vendors (optional)
-----------------------------------------

If you would like to use another DDS or RTPS vendor besides the default, you can find instructions :doc:`here <../DDS-Implementations>`.

Build the ROS 2 code
--------------------

Run the ``colcon`` tool to build everything (more on using ``colcon`` in :doc:`this tutorial <../../Tutorials/Beginner-Client-Libraries/Colcon-Tutorial>`):

.. code-block:: bash

   cd ~/ros2_{DISTRO}/
   colcon build --symlink-install --packages-skip-by-dep python_qt_binding

Note: due to an unresolved issue with SIP, Qt@5, and PyQt5, we need to disable ``python_qt_binding`` to have the build succeed.
This will be removed when the issue is resolved, see: https://github.com/ros-visualization/python_qt_binding/issues/103

Environment setup
-----------------

Source the ROS 2 setup file:

.. code-block:: bash

   . ~/ros2_{DISTRO}/install/setup.zsh

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

Next steps after installing
---------------------------
Continue with the `tutorials and demos <../../Tutorials>` to configure your environment, create your own workspace and packages, and learn ROS 2 core concepts.

Using the ROS 1 bridge
----------------------
The ROS 1 bridge can connect topics from ROS 1 to ROS 2 and vice-versa. See the dedicated `documentation <https://github.com/ros2/ros1_bridge/blob/master/README.md>`__ on how to build and use the ROS 1 bridge.

Additional RMW implementations (optional)
-----------------------------------------
The default middleware that ROS 2 uses is ``Fast DDS``, but the middleware (RMW) can be replaced at runtime.
See the :doc:`guide <../../How-To-Guides/Working-with-multiple-RMW-implementations>` on how to work with multiple RMWs.

Stay up to date
---------------

See :doc:`../Maintaining-a-Source-Checkout` to periodically refresh your source installation.

Troubleshooting
---------------

Troubleshooting techniques can be found :ref:`here <macOS-troubleshooting>`.

Uninstall
---------

1. If you installed your workspace with colcon as instructed above, "uninstalling" could be just a matter of opening a new terminal and not sourcing the workspace's ``setup`` file.
   This way, your environment will behave as though there is no {DISTRO_TITLE} install on your system.

2. If you're also trying to free up space, you can delete the entire workspace directory with:

   .. code-block:: bash

    rm -rf ~/ros2_{DISTRO}
