Installing ROS 2 on Windows
===========================

.. contents:: Table of Contents
   :depth: 2
   :local:

This page explains how to install ROS 2 on Windows from a pre-built binary package.

System requirements
-------------------

Only Windows 10 is supported.

.. _windows-install-binary-installing-prerequisites:

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

   > choco install -y python

Install OpenSSL
^^^^^^^^^^^^^^^

Download an OpenSSL installer from `this page <https://slproweb.com/products/Win32OpenSSL.html>`__. Scroll to the bottom of the page and download *Win64 OpenSSL v1.0.2*. Don't download the Win32 or Light versions.

Run the installer with default parameters. The following commands assume you used the default installation directory:

* ``setx -m OPENSSL_CONF C:\OpenSSL-Win64\bin\openssl.cfg``

You will need to append the OpenSSL-Win64 bin folder to your PATH.
You can do this by clicking the Windows icon, typing "Environment Variables", then clicking on "Edit the system environment variables".
In the resulting dialog, click "Environment Variables", then click "Path" on the bottom pane, finally click "Edit" and add the path below.

* ``C:\OpenSSL-Win64\bin\``

Install Visual Studio
^^^^^^^^^^^^^^^^^^^^^

**A. Install Visual Studio 2015 if using Ardent or earlier**

   If you already have a paid version of Visual Studio 2015 (Professional, Enterprise), skip this step.

   Microsoft provides a free of charge version of Visual Studio 2015, named Community, which can be used to build applications that use ROS 2:

   https://www.visualstudio.com/vs/older-downloads/

   Make sure that the Visual C++ features are installed. First choose 'Custom installation':

   .. image:: https://i.imgur.com/tUcOMOA.png

   Next check Visual C++:

   .. image:: https://i.imgur.com/yWVEUkm.png

   Ensure that the correct features will be installed:

   .. image:: https://i.imgur.com/VxdbA7G.png


**B. Install Visual Studio 2017 if using Bouncy or a nightly**

   If you already have a paid version of Visual Studio 2017 (Professional, Enterprise), skip this step.

.. warning:: Visual Studio 2017 v15.8 seems to have a compiler bug preventing from building some ROS 2 packages. Please try installing an older version of Visual Studio 2017.

   Microsoft provides a free of charge version of Visual Studio 2017, named Community, which can be used to build applications that use ROS 2:

   https://visualstudio.microsoft.com/downloads/

   Make sure that the Visual C++ features are installed.
   An easy way to make sure they're installed is to select the ``Desktop development with C++`` workflow during the install.

   .. image:: https://i.imgur.com/2h0IxCk.png


Install additional DDS implementations (optional)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you would like to use another DDS or RTPS vendor besides the default, eProsima's Fast RTPS, you can find instructions `here <DDS-Implementations>`.

Install OpenCV
^^^^^^^^^^^^^^

Some of the examples require OpenCV to be installed.

You can download a precompiled version of OpenCV 3.4.1 from https://github.com/ros2/ros2/releases/download/opencv-archives/opencv-3.4.1-vc15.VS2017.zip

Assuming you unpacked it to ``C:\opencv``\ , type the following on a Command Prompt (requires Admin privileges):

.. code-block:: bash

   setx -m OpenCV_DIR C:\opencv

Since you are using a precompiled ROS version, we have to tell it where to find the OpenCV libraries. You have to extend the ``PATH`` variable to ``c:\opencv\x64\vc15\bin``

In ardent and earlier
~~~~~~~~~~~~~~~~~~~~~

These releases used OpenCV 2. You can download a precompiled version of OpenCV 2.4.13.2 from https://github.com/ros2/ros2/releases/download/release-beta2/opencv-2.4.13.2-vc14.VS2015.zip

Since you are using a precompiled ROS version, we have to tell it where to find the OpenCV libraries. Assuming you were extracting OpenCV to ``c:\`` you have to extend your ``PATH`` variable to ``c:\opencv-2.4.13.2-vc14.VS2015\x64\vc14\bin``

Install dependencies
^^^^^^^^^^^^^^^^^^^^

There are a few dependencies not available in the Chocolatey package database. In order to ease the manual installation process, we provide the necessary Chocolatey packages.

As some chocolatey packages rely on it, we start by installing CMake

.. code-block:: bash

   > choco install -y cmake

You will need to append the CMake bin folder ``C:\Program Files\CMake\bin`` to your PATH.

Please download these packages from `this <https://github.com/ros2/choco-packages/releases/latest>`__ GitHub repository.


* asio.1.12.1.nupkg
* eigen-3.3.4.nupkg
* tinyxml-usestl.2.6.2.nupkg
* tinyxml2.6.0.0.nupkg
* log4cxx.0.10.0.nupkg

Once these packages are downloaded, open an administrative shell and execute the following command:

.. code-block:: bash

   > choco install -y -s <PATH\TO\DOWNLOADS\> asio eigen tinyxml-usestl tinyxml2 log4cxx

Please replace ``<PATH\TO\DOWNLOADS>`` with the folder you downloaded the packages to.

You must also install some python dependencies for command-line tools:

.. code-block:: bash

   python -m pip install -U catkin_pkg empy lark-parser opencv-python pyparsing pyyaml setuptools

RQt dependencies
~~~~~~~~~~~~~~~~

.. code-block:: bash

   python -m pip install -U pydot PyQt5

SROS2 dependencies
~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   python -m pip install -U lxml

Downloading ROS 2
-----------------


* Go the releases page: https://github.com/ros2/ros2/releases
* Download the latest package for Windows, e.g., ``ros2-package-windows-AMD64.zip``.

  * Notes:

    * there may be more than one binary download option which might cause the file name to differ.
    * [ROS Bouncy only] To download the ROS 2 debug libraries you'll need to download ``ros2-bouncy-windows-Debug-AMD64.zip``

* Unpack the zip file somewhere (we'll assume ``C:\dev\ros2_crystal``\ ).

  * Note (Ardent and earlier): There seems to be an issue where extracting the zip file with 7zip causes RViz to crash on startup. Extract the zip file using the Windows explorer to prevent this.

Environment setup
-----------------

Start a command shell and source the ROS 2 setup file to set up the workspace:

.. code-block:: bash

   > call C:\dev\ros2_crystal\local_setup.bat

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

See the `tutorials and demos </Tutorials>` for other things to try.

Build your own packages
-----------------------

If you would like to build your own packages, refer to the tutorial `"Using Colcon to build packages" </Tutorials/Colcon-Tutorial>`.

Troubleshooting
---------------

Troubleshooting techniques can be found :ref:`here <windows-troubleshooting>`.

Uninstall
---------

1. If you installed your workspace with colcon as instructed above, "uninstalling" could be just a matter of opening a new terminal and not sourcing the workspace's ``setup`` file.
   This way, your environment will behave as though there is no Crystal install on your system.

2. If you're also trying to free up space, you can delete the entire workspace directory with:

   .. code-block:: bash

    rmdir /s /q \ros2_crystal
