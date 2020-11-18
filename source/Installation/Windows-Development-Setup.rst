.. _windows-latest:

Building ROS 2 on Windows
=========================

.. contents:: Table of Contents
   :depth: 2
   :local:

This guide is about how to setup a development environment for ROS 2 on Windows.

Prerequisites
-------------

First follow the steps for `Installing Prerequisites <windows-install-binary-installing-prerequisites>` on the Binary Installation page.

Stop and return here when you reach the "Downloading ROS 2" section.

Additional prerequisites
^^^^^^^^^^^^^^^^^^^^^^^^

When building from source you'll need a few additional prerequisites installed.

Install additional prerequisites from Chocolatey
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

First install git:

.. code-block:: bash

   > choco install -y git

You will need to append the Git cmd folder ``C:\Program Files\Git\cmd`` to the PATH (you can do this by clicking the Windows icon, typing "Environment Variables", then clicking on "Edit the system environment variables".
In the resulting dialog, click "Environment Variables", the click "Path" on the bottom pane, then click "Edit" and add the path).


Install developer tools
-----------------------

Now we are ready to install some our tools that we use to help in developing ROS 2.

Let's start with ``vcstool``:

.. code-block:: bash

   > pip install -U vcstool

You can test it out by just running ``vcs`` (you should be able to do this in the same cmd prompt).

Next, install ``colcon``:

.. code-block:: bash

   > pip install -U colcon-common-extensions

You can test it out by just running ``colcon`` (you should be able to do this in the same cmd prompt).

Also, you should install ``curl``:

.. code-block:: bash

   > choco install -y curl

Install dependencies
--------------------

Next install the latest version of ``setuptools`` and ``pip``:

.. code-block:: bash

   > <PATH_TO_PYTHON_EXECUTABLE> -m pip install -U setuptools pip

Where ``PATH_TO_PYTHON_EXECUTABLE`` looks like: ``c:\python38\python.exe``

Then you can continue installing other Python dependencies:

.. code-block:: bash

   > pip install -U catkin_pkg cryptography EmPy ifcfg importlib-metadata lark-parser lxml numpy pyparsing pyyaml

Next install testing tools like ``pytest`` and others:

.. code-block:: bash

   > pip install -U pytest pytest-mock coverage mock

Next install linters and checkers like ``flake8`` and others:

.. code-block:: bash

   > pip install -U flake8 flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes mypy pep8 pydocstyle

Next install cppcheck:

.. code-block:: bash

   > choco install -y cppcheck

Next install xmllint:

* Download the `64 bit binary archives <https://www.zlatkovic.com/pub/libxml/64bit/>`__ of ``libxml2`` (and its dependencies ``iconv`` and ``zlib``) from https://www.zlatkovic.com/projects/libxml/
* Unpack all archives into e.g. ``C:\xmllint``
* Add ``C:\xmllint\bin`` to the ``PATH``.

Install Qt5
^^^^^^^^^^^

This section is only required if you are building rviz, but it comes with our default set of sources, so if you don't know, then assume you are building it.

First get the installer from Qt's website:

https://www.qt.io/download

Select the Open Source version and then the ``Qt Online Installer for Windows``.

Run the installer and install Qt5.

We recommend you install it to the default location of ``C:\Qt``, but if you choose somewhere else, make sure to update the paths below accordingly.
When selecting components to install, the only thing you absolutely need for Foxy and later is the appropriate MSVC 64-bit component under the ``Qt`` -> ``Qt 5.15.0`` tree.
We're using ``5.15.0`` as of the writing of this document and that's what we recommend since that's all we test on Windows, but later version will probably work too.
For Foxy and later, be sure to select ``MSVC 2019 64-bit``.
After that, the default settings are fine.

Finally, set the ``Qt5_DIR`` environment variable in the ``cmd.exe`` where you intend to build so that CMake can find it:

.. code-block:: bash

   > set Qt5_DIR=C:\Qt\5.15.0\msvc2019_64

You could set it permanently with ``setx -m Qt5_DIR C:\Qt\5.15.0\msvc2019_64`` instead, but that requires Administrator.

.. note::

   This path might change based on which MSVC version you're using or if you installed it to a different directory.

RQt dependencies
~~~~~~~~~~~~~~~~

.. code-block:: bash

   > pip install -U pydot PyQt5

Follow the steps for `Installing Graphviz <Rolling_windows-install-binary-installing-rqt-dependencies>` on the Binary Installation page.

Get the ROS 2 code
------------------

Now that we have the development tools we can get the ROS 2 source code.

First setup a development folder, for example ``C:\dev\ros2_rolling``:

.. code-block:: bash

   > md \dev\ros2_rolling\src
   > cd \dev\ros2_rolling

Get the ``ros2.repos`` file which defines the repositories to clone from:

.. code-block:: bash

   # CMD
   > curl -sk https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos -o ros2.repos

   # PowerShell
   > curl https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos -o ros2.repos

Next you can use ``vcs`` to import the repositories listed in the ``ros2.repos`` file:

.. code-block:: bash

   # CMD
   > vcs import src < ros2.repos

   # PowerShell
   > vcs import --input ros2.repos src

Install additional DDS implementations (optional)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you would like to use another DDS or RTPS vendor besides the default, eProsima's Fast RTPS, you can find instructions `here <DDS-Implementations>`.

Build the ROS 2 code
--------------------

.. _windows-dev-build-ros2:

To build ROS 2 you will need a Visual Studio Command Prompt ("x64 Native Tools Command Prompt for VS 2019") running as Administrator.

Fast RTPS is bundled with the ROS 2 source and will always be built unless you put an ``AMENT_IGNORE`` file in the ``src\eProsima`` folder.

To build the ``\dev\ros2_rolling`` folder tree:

.. code-block:: bash

   > colcon build --merge-install

.. note::

   We're using ``--merge-install`` here to avoid a ``PATH`` variable that is too long at the end of the build.
   If you're adapting these instructions to build a smaller workspace then you might be able to use the default behavior which is isolated install, i.e. where each package is installed to a different folder.

.. note::

   If you are doing a debug build use ``python_d path\to\colcon_executable`` ``colcon``.
   See `Extra stuff for debug mode`_ for more info on running Python code in debug builds on Windows.

Environment setup
-----------------

Start a command shell and source the ROS 2 setup file to set up the workspace:

.. code-block:: bash

   > call C:\dev\ros2_rolling\install\local_setup.bat

This will automatically set up the environment for any DDS vendors that support was built for.

It is normal that the previous command, if nothing else went wrong, outputs "The system cannot find the path specified." exactly once.

Test and run
------------

Note that the first time you run any executable you will have to allow access to the network through a Windows Firewall popup.

You can run the tests using this command:

.. code-block:: bash

   > colcon test --merge-install

.. note::

   ``--merge-install`` should only be used if it was also used in the build step.

Afterwards you can get a summary of the tests using this command:

.. code-block:: bash

   > colcon test-result

To run the examples, first open a clean new ``cmd.exe`` and set up the workspace by sourcing the ``local_setup.bat`` file.
Then, run a C++ ``talker``\ :

.. code-block:: bash

   > call install\local_setup.bat
   > ros2 run demo_nodes_cpp talker

In a separate shell you can do the same, but instead run a Python ``listener``\ :

.. code-block:: bash

   > call install\local_setup.bat
   > ros2 run demo_nodes_py listener

You should see the ``talker`` saying that it's ``Publishing`` messages and the ``listener`` saying ``I heard`` those messages.
This verifies both the C++ and Python APIs are working properly.
Hooray!


.. note::

   It is not recommended to build in the same cmd prompt that you've sourced the ``local_setup.bat``.

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


Extra stuff for Debug mode
--------------------------

If you want to be able to run all the tests in Debug mode, you'll need to install a few more things:


* To be able to extract the Python source tarball, you can use PeaZip:

.. code-block:: bash

   > choco install -y peazip


* You'll also need SVN, since some of the Python source-build dependencies are checked out via SVN:

.. code-block:: bash

   > choco install -y svn hg


* You'll need to quit and restart the command prompt after installing the above.
* Get and extract the Python 3.8.3 source from the ``tgz``:

  * https://www.python.org/ftp/python/3.8.3/Python-3.8.3.tgz
  * To keep these instructions concise, please extract it to ``C:\dev\Python-3.8.3``

* Now, build the Python source in debug mode from a Visual Studio command prompt:

.. code-block:: bash

   > cd C:\dev\Python-3.8.3\PCbuild
   > get_externals.bat
   > build.bat -p x64 -d


* Finally, copy the build products into the Python38 installation directories, next to the Release-mode Python executable and DLL's:

.. code-block:: bash

   > cd C:\dev\Python-3.8.3\PCbuild\amd64
   > copy python_d.exe C:\Python38 /Y
   > copy python38_d.dll C:\Python38 /Y
   > copy python3_d.dll C:\Python38 /Y
   > copy python38_d.lib C:\Python38\libs /Y
   > copy python3_d.lib C:\Python38\libs /Y
   > for %I in (*_d.pyd) do copy %I C:\Python38\DLLs /Y


* Now, from a fresh command prompt, make sure that ``python_d`` works:

.. code-block:: bash

   > python_d
   > import _ctypes

* Once you have verified the operation of ``python_d``, it is necessary to reinstall a few dependencies with the debug-enabled libraries:

.. code-block:: bash

   > python_d -m pip install --force-reinstall https://github.com/ros2/ros2/releases/download/numpy-archives/numpy-1.18.4-cp38-cp38d-win_amd64.whl
   > python_d -m pip install --force-reinstall https://github.com/ros2/ros2/releases/download/lxml-archives/lxml-4.5.1-cp38-cp38d-win_amd64.whl

* To verify the installation of these dependencies:

.. code-block:: bash

   > python_d
   # No import errors should appear when executing the following lines
   > from lxml import etree
   > import numpy

* When you wish to return to building release binaries, it is necessary to uninstall the debug variants and use the release variants:

.. code-block:: bash

   > python -m pip uninstall numpy lxml
   > python -m pip install numpy lxml

* To create executables python scripts(.exe), python_d should be used to invoke colcon

.. code-block:: bash

   > python_d path\to\colcon_executable build

* Hooray, you're done!

Stay up to date
---------------

See :ref:`MaintainingSource` to periodically refresh your source installation.

Troubleshooting
---------------

Troubleshooting techniques can be found :ref:`here <windows-troubleshooting>`.

Uninstall
---------

1. If you installed your workspace with colcon as instructed above, "uninstalling" could be just a matter of opening a new terminal and not sourcing the workspace's ``setup`` file.
   This way, your environment will behave as though there is no Rolling install on your system.

2. If you're also trying to free up space, you can delete the entire workspace directory with:

   .. code-block:: bash

    rmdir /s /q \ros2_rolling
