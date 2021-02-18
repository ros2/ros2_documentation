.. redirect-from::

   Installation/Crystal/OSX-Install-Binary

Installing ROS 2 on macOS
=========================

.. contents:: Table of Contents
   :depth: 2
   :local:

This page explains how to install ROS 2 on macOS from a pre-built binary package.

System requirements
-------------------

We support OS X El Capitan and macOS Sierra (10.11.x and 10.12.x).

.. _osx-install-binary-installling-prerequisites:

Installing prerequisites
------------------------

You need the following things installed before installing ROS 2.


*
  **brew** *(needed to install more stuff; you probably already have this)*:


  * Follow installation instructions at http://brew.sh/
  *
    *Optional*: Check that ``brew`` is happy with your system configuration by running:

    .. code-block:: bash

        brew doctor


      Fix any problems that it identifies.

*
  Use ``brew`` to install more stuff:

  .. code-block:: bash

       brew install python3

       # install asio and tinyxml2 for Fast-RTPS
       brew install asio tinyxml2

       # install dependencies for robot state publisher
       brew install tinyxml eigen pcre poco

       # OpenCV isn't a dependency of ROS 2, but it is used by some demos.
       brew install opencv

       # install OpenSSL for DDS-Security
       brew install openssl

       # install Qt for RViz
       brew install qt freetype assimp

       # install dependencies for rcl_logging_log4cxx
       brew install log4cxx

*
  Install rqt dependencies

  ``brew install sip pyqt5``

  Fix some path names when looking for sip stuff during install (see `ROS 1 wiki <https://wiki.ros.org/kinetic/Installation/OSX/Homebrew/Source#Qt_naming_issue>`__):

  ``ln -s /usr/local/share/sip/Qt5 /usr/local/share/sip/PyQt5``

  ``brew install graphviz``

  ``python3 -m pip install pygraphviz pydot``

*
  Install SROS2 dependencies

  ``python3 -m pip install lxml``

*
  Install additional runtime dependencies for command-line tools:

  .. code-block:: bash

       python3 -m pip install catkin_pkg empy lark-parser pyparsing pyyaml setuptools argcomplete

Disable System Integrity Protection (SIP)
-----------------------------------------

macOS/OS X versions >=10.11 have System Integrity Protection enabled by default.
So that SIP doesn't prevent processes from inheriting dynamic linker environment variables, such as ``DYLD_LIBRARY_PATH``, you'll need to disable it `following these instructions <https://developer.apple.com/library/content/documentation/Security/Conceptual/System_Integrity_Protection_Guide/ConfiguringSystemIntegrityProtection/ConfiguringSystemIntegrityProtection.html>`__.

Downloading ROS 2
-----------------


* Go to the releases page: https://github.com/ros2/ros2/releases
* Download the latest package for macOS; let's assume that it ends up at ``~/Downloads/ros2-release-distro-date-macos-amd64.tar.bz2``.

  * Note: there may be more than one binary download option which might cause the file name to differ.

*
  Unpack it:

  .. code-block:: bash

       mkdir -p ~/ros2_crystal
       cd ~/ros2_crystal
       tar xf ~/Downloads/ros2-release-distro-date-macos-amd64.tar.bz2

Install additional DDS implementations (optional)
-------------------------------------------------

If you would like to use another DDS or RTPS vendor besides the default, eProsima's Fast RTPS, you can find instructions `here <DDS-Implementations>`.

Environment setup
-----------------

Source the ROS 2 setup file:

.. code-block:: bash

   . ~/ros2_crystal/ros2-osx/setup.bash

Try some examples
-----------------

In one terminal, set up the ROS 2 environment as described above and then run a C++ ``talker``:

.. code-block:: bash

   ros2 run demo_nodes_cpp talker

In another terminal, set up the ROS 2 environment and then run a Python ``listener``:

.. code-block:: bash

   ros2 run demo_nodes_py listener

You should see the ``talker`` saying that it's ``Publishing`` messages and the ``listener`` saying ``I heard`` those messages.
This verifies both the C++ and Python APIs are working properly.
Hooray!

See the `tutorials and demos </Tutorials>` for other things to try.

Build your own packages
-----------------------

If you would like to build your own packages, refer to the tutorial `"Using Colcon to build packages" </Tutorials/Colcon-Tutorial>`.

Troubleshooting
---------------

Troubleshooting techniques can be found :ref:`here <macOS-troubleshooting>`.

Uninstall
---------

1. If you installed your workspace with colcon as instructed above, "uninstalling" could be just a matter of opening a new terminal and not sourcing the workspace's ``setup`` file.
   This way, your environment will behave as though there is no Crystal install on your system.

2. If you're also trying to free up space, you can delete the entire workspace directory with:

   .. code-block:: bash

    rm -rf ~/ros2_crystal
