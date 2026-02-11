Windows (binary)
================

.. contents:: Table of Contents
   :depth: 2
   :local:

This page explains how to install ROS 2 on Windows from a pre-built binary package.

.. note::

    The pre-built binary does not include all ROS 2 packages.
    All packages in the `ROS base variant <https://reps.openrobotics.org/rep-2001/#ros-base>`_ are included, and only a subset of packages in the `ROS desktop variant <https://reps.openrobotics.org/rep-2001/#desktop-variants>`_ are included.
    The exact list of packages are described by the repositories listed in `this ros2.repos file <https://github.com/ros2/ros2/blob/{REPOS_FILE_BRANCH}/ros2.repos>`_.

System requirements
-------------------

Only Windows 10 is supported.

.. _windows-install-binary-installing-prerequisites:

Create a location for the ROS 2 installation
--------------------------------------------

This location will contain both the installed binary packages, plus the ROS 2 installation itself.

Start a Command Prompt session (usually by clicking on the start menu, then typing ``Command Prompt``).

Then create a directory to store the installation.
Because of Windows path-length limitations, this should be as short as possible.
We'll use ``C:\dev`` for the rest of these instructions.

.. code-block:: console

   $ md C:\dev

Install prerequisites
---------------------

ROS 2 uses `conda-forge <https://conda-forge.org/>`__ as a backend for packages, with `pixi <https://pixi.sh/latest/>`__ as the frontend.

.. note::

   The installation of conda-forge may trigger Windows Defender to treat it as a threat, but this can be safely ignored by clicking "More info" and "Run anyway".

Install pixi
^^^^^^^^^^^^

Use use the instructions on https://pixi.sh/latest/ to install ``pixi`` either with the Windows Installer or using command line in your opened Command Prompt terminal.

Once ``pixi`` has been installed, close the Command Prompt session and start it again, which will ensure ``pixi`` is on the PATH.


Install ROS 2
-------------

Binary releases of {DISTRO_TITLE_FULL} are not provided.
Instead you may download nightly :ref:`prerelease binaries <Prerelease_binaries>`.

* Download the latest package for Windows, e.g., ``ros2-package-windows-AMD64.zip``.
* Unpack the zip file somewhere on your system (we'll assume ``C:\dev\``).
* Change the name of the extracted folder to match the distro (we'll assume ``C:\dev\{DISTRO}``)


Install Pixi dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^

Go to the folder where you unzipped the ROS 2 prereleased binaries and install the dependencies

.. code-block:: console

   $ cd C:\dev\{DISTRO}
   $ pixi install

Run preinstall installation script
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Run the preinstall installation setup script to make sure that the zipped file are able to run in the current folder the ROS 2 binaries have been exctrated to:

.. code-block:: console

   $ pixi run python preinstall_setup_windows.py


Install additional RMW implementations (optional)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The default middleware that ROS 2 uses is ``Fast DDS``, but the middleware (RMW) can be replaced at runtime.
See the :doc:`guide <../How-To-Guides/Working-with-multiple-RMW-implementations>` on how to work with multiple RMWs.


Setup environment
-----------------

In either the same Command Prompt terminal or a new one, you can source the ROS 2 environment

Source the pixi environment
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Source the pixi environment to set up dependencies:

.. code-block:: console

   $ cd C:\dev\{DISTRO}

Source the ROS 2 environment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This is required in every Command Prompt you open to setup the ROS 2 workspace:

.. code-block:: console

   $ call C:\dev\{DISTRO}\local_setup.bat

If you do not have RTI Connext DDS installed on your computer, it is normal to receive a warning that it is missing.

Try some examples
-----------------

In a Command Prompt, set up the ROS 2 environment as described above and then run a C++ ``talker``\ :

.. code-block:: console

   $ ros2 run demo_nodes_cpp talker

Start another Command Prompt terminal and run a Python ``listener``\ :

.. code-block:: console

   $ ros2 run demo_nodes_py listener

You should see the ``talker`` saying that it is ``Publishing`` messages and the ``listener`` saying ``I heard`` those messages.
This verifies both the C++ and Python APIs are working properly.
Hooray!


Next steps
----------

Continue with the :doc:`tutorials and demos <../../Tutorials>` to configure your environment, create your own workspace and packages, and learn ROS 2 core concepts.

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
