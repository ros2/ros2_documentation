Ubuntu (source Pixi)
====================

.. contents:: Table of Contents
   :depth: 2
   :local:


System requirements
-------------------
The current Debian-based target platforms for {DISTRO_TITLE_FULL} are:

- Tier 1: Ubuntu Linux - Resolute (26.04) 64-bit
- Tier 3: Ubuntu Linux - Noble (24.04) 64-bit
- Tier 3: Debian Linux - Trixie (13) 64-bit

As defined in `REP 2000 <https://www.ros.org/reps/rep-2000.html>`_.

System setup
------------

Set locale
^^^^^^^^^^

.. include:: ../_Ubuntu-Set-Locale.rst

Create a location for the ROS 2 installation
--------------------------------------------

This location will contain both the installed binary packages, plus the ROS 2 installation itself.

Start a terminal, then create a directory to store the installation.

.. code-block:: console

   $ mkdir ~/ros_ws_pixi

Install prerequisites
---------------------

Install pixi
^^^^^^^^^^^^

ROS 2 uses `conda-forge <https://conda-forge.org/>`__ as a backend for packages, with `pixi <https://pixi.sh/latest/>`__ as the frontend.

Continue using the previous terminal session, and use the instructions from https://pixi.sh/latest/ to install ``pixi``.
Once ``pixi`` has been installed, close the terminal session and start it again, which will ensure ``pixi`` is on the PATH.

Install dependencies
^^^^^^^^^^^^^^^^^^^^

Download the pixi configuration file in the existing terminal session:

.. code-block:: console

   $ cd ~/ros_ws_pixi
   $ wget https://raw.githubusercontent.com/ros2/ros2/{REPOS_FILE_BRANCH}/pixi.toml -O pixi.toml

Install dependencies:

.. code-block:: console

   $ pixi install

Build ROS 2
-----------

Source the pixi environment
^^^^^^^^^^^^^^^^^^^^^^^^^^^

This is required in every command prompt you open to set up paths to the dependencies:

.. code-block:: console

   $ cd ~/ros_ws_pixi
   $ pixi shell

Get ROS 2 code
^^^^^^^^^^^^^^

Now that we have the development tools we can get the ROS 2 source code.

Setup a development folder, for example ``~/ros_ws_pixi/src``:


.. code-block:: console

   $ mkdir ~/ros_ws_pixi/src
   $ cd ~/ros_ws_pixi

Get the ``ros2.repos`` file which defines the repositories to clone from:

.. code-block:: console

   $ vcs import --input https://raw.githubusercontent.com/ros2/ros2/{REPOS_FILE_BRANCH}/ros2.repos src

Build the code in the workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To build the workspace folder tree:

.. code-block:: console

   $ colcon build --merge-install --event-handlers console_direct+ --cmake-args -D LTTNGPY_DISABLED=1 --packages-skip qt_gui_cpp --packages-skip-by-dep qt_gui_cpp --packages-skip-build-finished

.. note::

   Source installation can take a long time given the large number of packages being pulled into the workspace.

Setup environment
-----------------

Start a new terminal, which will be used in the examples.

Source the pixi environment
^^^^^^^^^^^^^^^^^^^^^^^^^^^

This is required in every command prompt you open to set up paths to the dependencies:

.. code-block:: console

   $ cd ~/ros_ws_pixi
   $ pixi shell

Source the ROS 2 environment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This is required in every command prompt you open to setup the ROS 2 workspace:

.. code-block:: console

   $ source ~/ros_ws_pixi/install/local_setup.bash

This will automatically set up the environment for any DDS vendors for which support was built.

Try some examples
-----------------

You can run the tests using this command:

.. code-block:: console

   $ colcon test --merge-install

.. note::

   ``--merge-install`` should only be used if it was also used in the build step.

Afterwards you can get a summary of the tests using this command:

.. code-block:: console

   $ colcon test-result

To run the examples, set up the workspace by sourcing the ``local_setup.bash`` file.
Then, run a C++ ``talker``\ :

.. code-block:: console

   $ source ~/ros_ws_pixi/install/local_setup.bash
   $ ros2 run demo_nodes_cpp talker

In a separate command prompt you can do the same, but instead run a Python ``listener``\ :

.. code-block:: console

   $ source ~/ros_ws_pixi/install/local_setup.bash
   $ ros2 run demo_nodes_py listener

You should see the ``talker`` saying that it's ``Publishing`` messages and the ``listener`` saying ``I heard`` those messages.
This verifies both the C++ and Python APIs are working properly.
Hooray!

.. note::

   It is not recommended to build in the same cmd prompt that you've sourced the ``local_setup.bash``.

Next steps
----------

Continue with the tutorials to configure your environment, create your own workspace and packages, and learn ROS 2 core concepts.

Stay up to date
---------------

See :doc:`Maintaining-a-Source-Checkout` to periodically refresh your source installation.

1. If you installed your workspace with colcon as instructed above, "uninstalling" could be just a matter of opening a new terminal and not sourcing the workspace's ``setup`` file.
   This way, your environment will behave as though there is no {DISTRO_TITLE} install on your system.

2. If you're also trying to free up space, you can delete the entire workspace directory with:

   .. code-block:: console

      $ rm -rf ~/ros_ws_pixi
