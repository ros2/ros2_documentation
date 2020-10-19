Installing ROS 2 on Windows
===========================

.. contents:: Table of Contents
   :depth: 2
   :local:

This page explains how to install ROS 2 on Windows from a pre-built binary package.

.. note::

    The pre-built binary does not include all ROS 2 packages.
    All packages in the `ROS base variant <https://ros.org/reps/rep-2001.html#ros-base>`_ are included, and only a subset of packages in the `ROS desktop variant <https://ros.org/reps/rep-2001.html#desktop-variants>`_ are included.
    The exact list of packages are described by the repositories listed in `this ros2.repos file <https://github.com/ros2/ros2/blob/foxy-release/ros2.repos>`_.

System requirements
-------------------

Only Windows 10 is supported.

.. _Foxy_windows-install-binary-installing-prerequisites:

Installing prerequisites
------------------------

Install Chocolatey
^^^^^^^^^^^^^^^^^^

Chocolatey is a package manager for Windows, install it by following their installation instructions:

https://chocolatey.org/

You'll use Chocolatey to install some other developer tools.

Install Python
^^^^^^^^^^^^^^

Open a Command Prompt and type the following to install Python via Chocolatey:

.. code-block:: bash

   > choco install -y python --version 3.8.3

ROS 2 expects the python installation to be available in directory ``C:\python38``.
Double check that it is installed there.

Install Visual C++ Redistributables
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Open a Command Prompt and type the following to install them via Chocolatey:

.. code-block:: bash

   > choco install -y vcredist2013 vcredist140

Install OpenSSL
^^^^^^^^^^^^^^^

Download the *Win64 OpenSSL v1.1.1h* OpenSSL installer from `this page <https://slproweb.com/products/Win32OpenSSL.html>`__.
Scroll to the bottom of the page and download *Win64 OpenSSL v1.1.1h*.
Don't download the Win32 or Light versions.

Run the installer with default parameters.

The following command sets an environment variable that persists over sessions.
Modify the command to match your installation directory:

* ``setx -m OPENSSL_CONF "C:\Program Files\OpenSSL-Win64\bin\openssl.cfg"``

You will also need to append the OpenSSL-Win64 bin folder to your PATH.
You can do this by clicking the Windows icon, typing "Environment Variables", then clicking on "Edit the system environment variables".
In the resulting dialog, click "Environment Variables", then click "Path" on the bottom pane, finally click "Edit" and add the path below.

* ``C:\Program Files\OpenSSL-Win64\bin\``

Install Visual Studio
^^^^^^^^^^^^^^^^^^^^^

Install Visual Studio 2019.

If you already have a paid version of Visual Studio 2019 (Professional, Enterprise), skip this step.

Microsoft provides a free of charge version of Visual Studio 2019, named Community, which can be used to build applications that use ROS 2:

   https://visualstudio.microsoft.com/downloads/

Make sure that the Visual C++ features are installed.

An easy way to make sure they're installed is to select the ``Desktop development with C++`` workflow during the install.

   .. image:: https://i.imgur.com/2h0IxCk.png

Make sure that no C++ CMake tools are installed by unselecting them in the list of components to be installed.

Install additional DDS implementations (optional)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you would like to use another DDS or RTPS vendor besides the default, eProsima's Fast RTPS, you can find instructions `here <../DDS-Implementations>`.

Install OpenCV
^^^^^^^^^^^^^^

Some of the examples require OpenCV to be installed.

You can download a precompiled version of OpenCV 3.4.6 from https://github.com/ros2/ros2/releases/download/opencv-archives/opencv-3.4.6-vc16.VS2019.zip .

Assuming you unpacked it to ``C:\opencv``\ , type the following on a Command Prompt (requires Admin privileges):

.. code-block:: bash

   setx -m OpenCV_DIR C:\opencv

Since you are using a precompiled ROS version, we have to tell it where to find the OpenCV libraries.
You have to extend the ``PATH`` variable to ``C:\opencv\x64\vc16\bin``.

Install dependencies
^^^^^^^^^^^^^^^^^^^^

There are a few dependencies not available in the Chocolatey package database.
In order to ease the manual installation process, we provide the necessary Chocolatey packages.

As some chocolatey packages rely on it, we start by installing CMake

.. code-block:: bash

   > choco install -y cmake

You will need to append the CMake bin folder ``C:\Program Files\CMake\bin`` to your PATH.

Please download these packages from `this <https://github.com/ros2/choco-packages/releases/latest>`__ GitHub repository.

* asio.1.12.1.nupkg
* bullet.2.89.0.nupkg
* cunit.2.1.3.nupkg
* eigen-3.3.4.nupkg
* tinyxml-usestl.2.6.2.nupkg
* tinyxml2.6.0.0.nupkg
* log4cxx.0.10.0.nupkg

Once these packages are downloaded, open an administrative shell and execute the following command:

.. code-block:: bash

   > choco install -y -s <PATH\TO\DOWNLOADS> asio cunit eigen tinyxml-usestl tinyxml2 log4cxx bullet

Please replace ``<PATH\TO\DOWNLOADS>`` with the folder you downloaded the packages to.

You must also install some python dependencies for command-line tools:

.. code-block:: bash

   python -m pip install -U catkin_pkg cryptography empy ifcfg lark-parser lxml netifaces numpy opencv-python pyparsing pyyaml setuptools

RQt dependencies
~~~~~~~~~~~~~~~~

.. code-block:: bash

   python -m pip install -U pydot PyQt5

.. _Foxy_windows-install-binary-installing-rqt-dependencies:

To run rqt_graph, you'll need `Graphviz <https://graphviz.gitlab.io/>`__.

.. code-block:: bash

   > choco install graphviz

You will need to append the Graphviz bin folder ``C:\Program Files (x86)\GraphvizX.XX\bin`` to your PATH, by navigating to "Edit the system environment variables" as described above.

Downloading ROS 2
-----------------

* Go the releases page: https://github.com/ros2/ros2/releases
* Download the latest package for Windows, e.g., ``ros2-foxy-*-windows-AMD64.zip``.

.. note::

    There may be more than one binary download option which might cause the file name to differ.

.. note::

    To download the ROS 2 debug libraries you'll need to download ``ros2-foxy-*-windows-debug-AMD64.zip``

* Unpack the zip file somewhere (we'll assume ``C:\dev\ros2_foxy``\ ).

Environment setup
-----------------

Start a command shell and source the ROS 2 setup file to set up the workspace:

.. tabs::

  .. group-tab:: Command Prompt

    .. code-block:: bash

       > call C:\dev\ros2_foxy\local_setup.bat

  .. group-tab:: PowerShell

    .. code-block:: bash

       > C:\dev\ros2_foxy\local_setup.ps1

It is normal that the previous command, if nothing else went wrong, outputs "The system cannot find the path specified." exactly once.

Try some examples
-----------------

In a command shell, set up the ROS 2 environment as described above and then run a C++ ``talker``\ :

.. code-block:: bash

   > ros2 run demo_nodes_cpp talker

Start another command shell and run a Python ``listener``\ :

.. code-block:: bash

   > ros2 run demo_nodes_py listener

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
See the `tutorial </Tutorials/Working-with-multiple-RMW-implementations>` on how to work with multiple RMWs.

Troubleshooting
---------------

Troubleshooting techniques can be found :ref:`here <windows-troubleshooting>`.

Uninstall
---------

1. If you installed your workspace with colcon as instructed above, "uninstalling" could be just a matter of opening a new terminal and not sourcing the workspace's ``setup`` file.
   This way, your environment will behave as though there is no Foxy install on your system.

2. If you're also trying to free up space, you can delete the entire workspace directory with:

   .. code-block:: bash

    rmdir /s /q \ros2_foxy

(Alternative) ROS 2 Build Installation from aka.ms/ros
--------------------------------------------------------

https://aka.ms/ros project hosts ROS 2 builds against the release snapshots.
This section explains how to install ROS 2 from this channel.

Install ROS 2 builds
^^^^^^^^^^^^^^^^^^^^

In an administrative command prompt, run the following commands.

.. code-block:: bash

   > mkdir c:\opt\chocolatey
   > set PYTHONNOUSERSITE=1
   > set ChocolateyInstall=c:\opt\chocolatey
   > choco source add -n=ros-win -s="https://aka.ms/ros/public" --priority=1
   > choco upgrade ros-foxy-desktop -y --execution-timeout=0

Environment setup
^^^^^^^^^^^^^^^^^^

Start an administrative command prompt and source the ROS 2 setup file to set up the workspace:

.. code-block:: bash

   > call C:\opt\ros\foxy\x64\local_setup.bat

Stay up-to-date
^^^^^^^^^^^^^^^

To keep up-to-date with the latest builds, run:

.. code-block:: bash

   > set ChocolateyInstall=c:\opt\chocolatey
   > choco upgrade all -y --execution-timeout=0

Uninstall
^^^^^^^^^

If you want to completely remove the environment downloaded above, run this command:

.. code-block:: bash

   > rmdir /s /q C:\opt\
