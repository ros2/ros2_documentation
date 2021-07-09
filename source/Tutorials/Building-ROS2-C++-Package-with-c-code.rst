Building ROS 2 C++ Package with C code
======================================

.. contents:: Table of Contents
   :depth: 2
   :local:
   
   
1 Source the setup files
^^^^^^^^^^^^^^^^^^^^^^^^

You will need to run this command on every new shell you open to have access to the ROS 2 commands, like so:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        source /opt/ros/{DISTRO}/setup.bash

   .. group-tab:: macOS

      .. code-block:: console

        . ~/ros2_install/ros2-osx/setup.bash

   .. group-tab:: Windows

      .. code-block:: console

        call C:\dev\ros2\local_setup.bat

.. note::
    The exact command depends on where you installed ROS 2.
    If you're having problems, ensure the file path leads to your installation.
    
2 Create a new directory
^^^^^^^^^^^^^^^^^^^^^^^^

Best practice is to create a new directory for every new workspace.
The name doesn’t matter, but it is helpful to have it indicate the purpose of the workspace.
Let’s choose the directory name ``dev_ws``, for “development workspace”:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        mkdir -p ~/dev_ws/src
        cd ~/dev_ws/src

   .. group-tab:: macOS

      .. code-block:: console

        mkdir -p ~/dev_ws/src
        cd ~/dev_ws/src

   .. group-tab:: Windows

     .. code-block:: console

       md \dev_ws\src
       cd \dev_ws\src


Another best practice is to put any packages in your workspace into the ``src`` directory.
The above code creates a ``src`` directory inside ``dev_ws`` and then navigates into it.


  
3 Create a package
^^^^^^^^^^^^^^^^^^

First, :ref:`source your ROS 2 installation <ConfigROS2>`.

Let’s use the workspace you created in the :ref:`previous tutorial <new-directory>`, ``dev_ws``, for your new package.`

Make sure you are in the ``src`` folder before running the package creation command.

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        cd ~/dev_ws/src

   .. group-tab:: macOS

     .. code-block:: console

       cd ~/dev_ws/src

   .. group-tab:: Windows

     .. code-block:: console

       cd \dev_ws\src

The command syntax for creating a new package in ROS 2 is:

.. tabs::

   .. group-tab:: CMake

      .. code-block:: console

        ros2 pkg create --build-type ament_cmake <package_name>

   .. group-tab:: Python

      .. code-block:: console

        ros2 pkg create --build-type ament_python <package_name>
        
   
We start eclipse and select a eclipse-workspace.

.. image:: images/eclipse_work_dir.png
   :target: images/eclipse_work_dir.png
   :alt: eclipse_work_dir

