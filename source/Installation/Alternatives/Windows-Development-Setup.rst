.. redirect-from::

   Installation/Windows-Development-Setup

Windows (source)
================

.. contents:: Table of Contents
   :depth: 2
   :local:

This page explains how to setup a development environment for ROS 2 on Windows.

System requirements
-------------------

Only Windows 10 is supported.

Language support
^^^^^^^^^^^^^^^^

Make sure you have a locale which supports ``UTF-8``.
For example, for a Chinese-language Windows 10 installation, you may need to install an `English language pack <https://support.microsoft.com/en-us/windows/language-packs-for-windows-a5094319-a92d-18de-5b53-1cfc697cfca8>`_.

Create a location for the ROS 2 installation
--------------------------------------------

This location will contain both the installed binary packages, plus the ROS 2 installation itself.

Start a powershell session (usually by clicking on the start menu, then typing ``powershell``).

Then create a directory to store the installation.
Because of Windows path-length limitations, this should be as short as possible.
We'll use ``C:\dev`` for the rest of these instructions.

.. code-block:: console

   $ md C:\dev

Increase the Windows maximum path length
----------------------------------------

By default, Windows is restricted to a maximum path length (MAX_PATH) of 260 characters.
The ROS 2 build will use significantly longer path lengths, so we will increase that.
Using the powershell session you started above, run the following:

.. code-block:: console

   $ New-ItemProperty -Path "HKLM:\SYSTEM\CurrentControlSet\Control\FileSystem" -Name "LongPathsEnabled" -Value 1 -PropertyType DWORD -Force

You can read more about this limitation in `Microsoft's documentation <https://learn.microsoft.com/en-us/windows/win32/fileio/maximum-file-path-limitation?tabs=registry>`__.


Install prerequisites
---------------------

Install MSVC
^^^^^^^^^^^^

In order to compile the ROS 2 code, the MSVC compiler must be installed.
Currently it is recommended to use MSVC 2019.

Continue using the previous powershell session, and run the following command to download it:

.. code-block:: console

   $ irm https://aka.ms/vs/16/release/vs_buildtools.exe -OutFile vs_buildtools_2019.exe

Now install MSVC 2019:

.. code-block:: console

   $ .\vs_buildtools_2019.exe --quiet --wait --norestart --add Microsoft.Component.MSBuild --add Microsoft.Net.Component.4.6.1.TargetingPack --add Microsoft.Net.Component.4.8.SDK --add Microsoft.VisualStudio.Component.CoreBuildTools --add Microsoft.VisualStudio.Component.Roslyn.Compiler --add Microsoft.VisualStudio.Component.TextTemplating --add Microsoft.VisualStudio.Component.VC.CLI.Support --add Microsoft.VisualStudio.Component.VC.CoreBuildTools --add Microsoft.VisualStudio.Component.VC.CoreIde --add Microsoft.VisualStudio.Component.VC.Redist.14.Latest --add Microsoft.VisualStudio.Component.VC.Tools.x86.x64 --add Microsoft.VisualStudio.Component.Windows10SDK --add Microsoft.VisualStudio.Component.Windows10SDK.19041 --add Microsoft.VisualStudio.ComponentGroup.NativeDesktop.Core --add Microsoft.VisualStudio.Workload.MSBuildTools --add Microsoft.VisualStudio.Workload.VCTools

.. note::

   The installation of MSVC can take a long time, and there is no feedback while it is progressing.

Install pixi
^^^^^^^^^^^^

ROS 2 uses `conda-forge <https://conda-forge.org/>`__ as a backend for packages, with `pixi <https://pixi.sh/latest/>`__ as the frontend.

Continue using the previous powershell session, and use the instructions from https://pixi.sh/latest/ to install ``pixi``.
Once ``pixi`` has been installed, close the powershell session and start it again, which will ensure ``pixi`` is on the PATH.

Install dependencies
^^^^^^^^^^^^^^^^^^^^

Download the pixi configuration file in the existing powershell session:

.. code-block:: console

   $ cd C:\dev
   $ irm https://raw.githubusercontent.com/ros2/ros2/refs/heads/{REPOS_FILE_BRANCH}/pixi.toml -OutFile pixi.toml

Install dependencies:

.. code-block:: console

   $ pixi install

You should now close the powershell session, as the rest of the instructions will use the Windows command prompt.

Build ROS 2
-----------

Start a new Windows command prompt, which will be used for the build.

Source the MSVC compiler
^^^^^^^^^^^^^^^^^^^^^^^^

This is required in the command prompt you'll use to compile ROS 2, but it is *not* required when running:

.. code-block:: console

  $ call "C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x86_amd64

Source the pixi environment
^^^^^^^^^^^^^^^^^^^^^^^^^^^

This is required in every command prompt you open to set up paths to the dependencies:

.. code-block:: console

   $ cd C:\dev
   $ pixi shell

Get ROS 2 code
^^^^^^^^^^^^^^

Now that we have the development tools we can get the ROS 2 source code.

Setup a development folder, for example ``C:\dev\{DISTRO}``:

.. code-block:: console

   $ md C:\dev\{DISTRO}\src
   $ cd C:\dev\{DISTRO}

Get the ``ros2.repos`` file which defines the repositories to clone from:

.. code-block:: console

   $ vcs import --input https://raw.githubusercontent.com/ros2/ros2/{REPOS_FILE_BRANCH}/ros2.repos src

Install additional RMW implementations (optional)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The default middleware that ROS 2 uses is ``Fast DDS``, but the middleware (RMW) can be replaced at build or runtime.
See the :doc:`guide <../../How-To-Guides/Working-with-multiple-RMW-implementations>` on how to work with multiple RMWs.

Build the code in the workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. _windows-dev-build-ros2:

To build the ``\{DISTRO}`` folder tree:

.. code-block:: console

   $ colcon build --merge-install

.. note::

   We're using ``--merge-install`` here to avoid a ``PATH`` variable that is too long at the end of the build.
   If you're adapting these instructions to build a smaller workspace then you might be able to use the default behavior which is isolated install, i.e. where each package is installed to a different folder.

.. note::

   Source installation can take a long time given the large number of packages being pulled into the workspace.

Setup environment
-----------------

Start a new Windows command prompt, which will be used in the examples.

Source the pixi environment
^^^^^^^^^^^^^^^^^^^^^^^^^^^

This is required in every command prompt you open to set up paths to the dependencies:

.. code-block:: console

   $ cd C:\dev
   $ pixi shell

Source the ROS 2 environment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This is required in every command prompt you open to setup the ROS 2 workspace:

.. code-block:: console

   $ call C:\dev\{DISTRO}\install\local_setup.bat

This will automatically set up the environment for any DDS vendors that support was built for.

It is normal that the previous command, if nothing else went wrong, outputs ``The system cannot find the path specified.`` exactly once.

Try some examples
-----------------

Note that the first time you run any executable you will have to allow access to the network through a Windows Firewall popup.

You can run the tests using this command:

.. code-block:: console

   $ colcon test --merge-install

.. note::

   ``--merge-install`` should only be used if it was also used in the build step.

Afterwards you can get a summary of the tests using this command:

.. code-block:: console

   $ colcon test-result

To run the examples, first open a clean new ``cmd.exe`` and set up the workspace by sourcing the ``local_setup.bat`` file.
Then, run a C++ ``talker``\ :

.. code-block:: console

   $ call install\local_setup.bat
   $ ros2 run demo_nodes_cpp talker

In a separate command prompt you can do the same, but instead run a Python ``listener``\ :

.. code-block:: console

   $ call install\local_setup.bat
   $ ros2 run demo_nodes_py listener

You should see the ``talker`` saying that it's ``Publishing`` messages and the ``listener`` saying ``I heard`` those messages.
This verifies both the C++ and Python APIs are working properly.
Hooray!

.. note::

   It is not recommended to build in the same cmd prompt that you've sourced the ``local_setup.bat``.

Next steps
----------

Continue with the :doc:`tutorials and demos <../../Tutorials>` to configure your environment, create your own workspace and packages, and learn ROS 2 core concepts.

Stay up to date
---------------

See :doc:`../Maintaining-a-Source-Checkout` to periodically refresh your source installation.

Troubleshoot
------------

Troubleshooting techniques can be found :ref:`here <windows-troubleshooting>`.

Uninstall
---------

1. If you installed your workspace with colcon as instructed above, "uninstalling" could be just a matter of opening a new terminal and not sourcing the workspace's ``setup`` file.
   This way, your environment will behave as though there is no {DISTRO_TITLE} install on your system.

2. If you're also trying to free up space, you can delete the entire workspace directory with:

   .. code-block:: console

      $ rmdir /s /q C:\dev\{DISTRO}
