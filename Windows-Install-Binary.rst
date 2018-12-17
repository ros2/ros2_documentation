
Installing ROS 2 on Windows
===========================

This page explains how to install ROS 2 on Windows from a pre-built binary package.

System requirements
-------------------

As of beta-2 only Windows 10 is supported.

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

Run the installer with default parameters. Then, define environment variables (the following commands assume you used the default installation directory):


* ``setx -m OPENSSL_CONF C:\OpenSSL-Win64\bin\openssl.cfg``
* Add ``C:\OpenSSL-Win64\bin\`` to your PATH

Install Visual Studio
^^^^^^^^^^^^^^^^^^^^^

**A. Install Visual Studio 2015 if using Ardent or earlier**

   If you already have a paid version of Visual Studio 2015 (Professional, Enterprise), skip this step.

   Microsoft provides a free of charge version of Visual Studio 2015, named Community, which can be used to build applications that use ROS 2:

   https://www.visualstudio.com/vs/older-downloads/

   Make sure that the Visual C++ features are installed. First choose 'Custom installation':

   .. image:: http://i.imgur.com/tUcOMOA.png

   Next check Visual C++:

   .. image:: http://i.imgur.com/yWVEUkm.png

   Ensure that the correct features will be installed:

   .. image:: http://i.imgur.com/VxdbA7G.png


**B. Install Visual Studio 2017 if using Bouncy or a nightly**

   If you already have a paid version of Visual Studio 2017 (Professional, Enterprise), skip this step.

   :warning: Visual Studio 2017 v15.8 seems to have a compiler bug preventing from building some ROS 2 packages. Please try installing an older version of Visual Studio 2017.

   Microsoft provides a free of charge version of Visual Studio 2017, named Community, which can be used to build applications that use ROS 2:

   https://visualstudio.microsoft.com/downloads/
   
   Make sure that the Visual C++ features are installed.
   An easy way to make sure they're installed is to select the `Desktop development with C++` workflow during the install.

   .. image:: https://i.imgur.com/2h0IxCk.png


Install additional DDS implementations (optional)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

ROS 2 builds on top of DDS.
It is compatible with multiple DDS or RTPS (the DDS wire protocol) vendors.

The package you downloaded has been built with optional support for multiple vendors: eProsima FastRTPS, Adlink OpenSplice, and (as of ROS 2 Bouncy) RTI Connext as the middleware options.
Run-time support for eProsima's Fast RTPS is included bundled by default.
If you would like to use one of the other vendors you will need to install their software separately.

Adlink OpenSplice
~~~~~~~~~~~~~~~~~

If you want to use OpenSplice, you will need to download the latest supported version.
* For ROS 2 Crystal version 6.9.181126OSS-HDE-x86_64.win-vs2017 or later is required.
* For ROS 2 Bouncy version 6.7.180404OSS-HDE-x86_64.win-vs2017 or later is required.

Download the `latest supported version <https://github.com/ADLINK-IST/opensplice/releases>`__
For ROS 2 releases up to and including Ardent, extract it but do not do anything else at this point.
For ROS 2 releases later than Ardent, set the ``OSPL_HOME`` environment variable to the unpacked directory that contains the ``release.bat`` script.

RTI Connext
~~~~~~~~~~~

To use RTI Connext (available as of ROS 2 Bouncy) you will need to have obtained a license from RTI.

You can install the Windows package of Connext version 5.3.1 provided by RTI from their `downloads page <https://www.rti.com/downloads>`__.

After installing, run RTI launcher and point it to your license file.

Set the ``NDDSHOME`` environment variable:

.. code-block:: bash

   set "NDDSHOME=C:\Program Files\rti_connext_dds-5.3.1"

If you want to install the Connext DDS-Security plugins please refer to `this page <Install-Connext-Security-Plugins>`

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

Since you are using a precompiled ROS version, we have to tell it where to find the OpenCV libraries. Assuming you were extracting OpenCV to ``c:\`` you have to extend the ``PATH`` variable to ``c:\opencv-2.4.13.2-vc14.VS2015\x64\vc14\bin``

Install dependencies
^^^^^^^^^^^^^^^^^^^^

There are a few dependencies not available in the Chocolatey package database. In order to ease the manual installation process, we provide the necessary Chocolatey packages.

As some chocolatey packages rely on it, we start by installing CMake

.. code-block:: bash

   > choco install -y cmake

You will need to append the CMake bin folder ``C:\Program Files\CMake\bin`` to the PATH (you can do this by clicking the Windows icon, typing "Environment Variables", then clicking on "Edit the system environment variables".
In the resulting dialog, click "Environment Variables", the click "Path" on the bottom pane, then click "Edit" and add the path).

Please download these packages from `this <https://github.com/ros2/choco-packages/releases/latest>`__ GitHub repository. 


* asio.1.12.1.nupkg
* eigen-3.3.4.nupkg
* tinyxml-usestl.2.6.2.nupkg
* tinyxml2.6.0.0.nupkg

Once these packages are downloaded, open an administrative shell and execute the following command:

.. code-block:: bash

   > choco install -y -s <PATH\TO\DOWNLOADS\> asio eigen tinyxml-usestl tinyxml2

Please replace ``<PATH\TO\DOWNLOADS>`` with the folder you downloaded the packages to.

You must also install some python dependencies for command-line tools:

.. code-block:: bash

   python -m pip install -U catkin_pkg empy git+https://github.com/lark-parser/lark.git@0.7b pyparsing pyyaml setuptools

rqt dependencies
~~~~~~~~~~~~~~~~

.. code-block:: bash

   python -m pip install -U pydot PyQt5

Downloading ROS 2
-----------------


* Go the releases page: https://github.com/ros2/ros2/releases
* Download the latest package for Windows, e.g., ``ros2-package-windows-AMD64.zip``.

  * Notes:

    * there may be more than one binary download option which might cause the file name to differ.
    * [ROS Bouncy only] To download the ROS 2 debug libraries you'll need to download ``ros2-bouncy-windows-Debug-AMD64.zip``

* Unpack the zip file somewhere (we'll assume ``C:\dev\ros2``\ ).

  * Note (Ardent and earlier): There seems to be an issue where extracting the zip file with 7zip causes RViz to crash on startup. Extract the zip file using the Windows explorer to prevent this.

Set up the ROS 2 environment
----------------------------

Start a command shell and source the ROS 2 setup file to set up the workspace:

.. code-block:: bash

   > call C:\dev\ros2\local_setup.bat

For ROS 2 releases up to and including Ardent, if you downloaded a release with OpenSplice support you must additionally source the OpenSplice setup file manually (this is done automatically for ROS 2 releases later than Ardent; this step can be skipped).
It is normal that the previous command, if nothing else went wrong, outputs "The system cannot find the path specified." exactly once.
Only do this step **after** you have sourced the ROS 2 setup file:

.. code-block:: bash

   > call "C:\opensplice69\HDE\x86_64.win64\release.bat"

Try some examples
-----------------

In a command shell, set up the ROS 2 environment as described above and then run a ``talker``\ :

.. code-block:: bash

   > ros2 run demo_nodes_cpp talker

Start another command shell and run a ``listener``\ :

.. code-block:: bash

   > ros2 run demo_nodes_py listener

You should see the ``talker`` saying that it's ``Publishing`` messages and the ``listener`` saying ``I heard`` those messages.
Hooray!

If you have installed support for an optional vendor, see `this page <Working-with-multiple-RMW-implementations>` for details on how to use that vendor.

Troubleshooting
^^^^^^^^^^^^^^^


* If at one point your example would not start because of missing dll's, please verify that all libraries from external dependencies such as OpenCV are located inside your ``PATH`` variable.
* If you forget to call the ``local_setup.bat`` file from your terminal, the demo programs will most likely crash immediately.
