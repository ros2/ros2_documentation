.. redirect-from::

  Installation/Dashing/OSX-Development-Setup

Building ROS 2 on macOS
=======================

.. contents:: Table of Contents
   :depth: 2
   :local:

System requirements
-------------------

We support macOS 10.12.x.

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

       # install CUnit for Cyclone DDS
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
   Install rqt dependencies

  ``brew install graphviz pyqt5 sip``

  Fix some path names when looking for sip stuff during install (see `ROS 1 wiki <https://wiki.ros.org/kinetic/Installation/OSX/Homebrew/Source#Qt_naming_issue>`__):

  ``ln -s /usr/local/share/sip/Qt5 /usr/local/share/sip/PyQt5``

#.
   Use ``python3 -m pip`` (just ``pip`` may install Python3 or Python2) to install more stuff:

   .. code-block:: bash

       python3 -m pip install -U argcomplete catkin_pkg colcon-common-extensions coverage empy flake8 flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes lark-parser mock nose pep8 pydocstyle pydot pygraphviz pyparsing setuptools vcstool

#.
   *Optional*: if you want to build the ROS 1<->2 bridge, then you must also install ROS 1:


   * Start with the normal install instructions: https://wiki.ros.org/kinetic/Installation/OSX/Homebrew/Source
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

   mkdir -p ~/ros2_dashing/src
   cd ~/ros2_dashing
   wget https://raw.githubusercontent.com/ros2/ros2/dashing/ros2.repos
   vcs import src < ros2.repos

Install additional DDS vendors (optional)
-----------------------------------------

If you would like to use another DDS or RTPS vendor besides the default, eProsima's Fast RTPS, you can find instructions `here <DDS-Implementations>`.

Build the ROS 2 code
--------------------
Run the ``colcon`` tool to build everything (more on using ``colcon`` in `this tutorial </Tutorials/Colcon-Tutorial>`):

.. code-block:: bash

   cd ~/ros2_dashing/
   colcon build --symlink-install

Environment setup
-----------------

Source the ROS 2 setup file:

.. code-block:: bash

   . ~/ros2_dashing/install/setup.bash

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
Continue with the `tutorials and demos </Tutorials>` to configure your environment, create your own workspace and packages, and learn ROS 2 core concepts.

Using the ROS 1 bridge
----------------------
The ROS 1 bridge can connect topics from ROS 1 to ROS 2 and vice-versa. See the dedicated `documentation <https://github.com/ros2/ros1_bridge/blob/master/README.md>`__ on how to build and use the ROS 1 bridge.

Additional RMW implementations (optional)
-----------------------------------------
The default middleware that ROS 2 uses is ``Fast-RTPS``, but the middleware (RMW) can be replaced at runtime.
See the `guide <../Guides/Working-with-multiple-RMW-implementations>` on how to work with multiple RMWs.

Stay up to date
---------------

See :ref:`MaintainingSource` to periodically refresh your source installation.

Troubleshooting
---------------

Troubleshooting techniques can be found :ref:`here <macOS-troubleshooting>`.

Uninstall
---------

1. If you installed your workspace with colcon as instructed above, "uninstalling" could be just a matter of opening a new terminal and not sourcing the workspace's ``setup`` file.
   This way, your environment will behave as though there is no Dashing install on your system.

2. If you're also trying to free up space, you can delete the entire workspace directory with:

   .. code-block:: bash

    rm -rf ~/ros2_dashing
