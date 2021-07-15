Building ROS 2 C++ Package with C code
======================================

.. contents:: Table of Contents
   :depth: 2
   :local:
   
   
Implementing Tensorflow mediapipe (https://github.com/google/mediapipe) and 
using Hands (https://google.github.io/mediapipe/solutions/hands), for tracking multiple hands.
Node publishs custom Hand.msg with x,y,z coordinates of 21 3D hand-knuckle coordinates per hand. Also a 
image stream with the camera-image overlayed with the tracked hands on 
topic /hand_image.

For rviz2 we write a urdf package with floating sphere links to display the 
hand and using the mediapipe node to control it. 
   
  
1 Install some packages
^^^^^^^^^^^^^^^^^^^^^^^

pip3 install opencv-python
pip3 install mediapipe

 
2 Source the setup files
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

        ros2 pkg create --build-type ament_python --node-name mediapipe-hands mediapipe-hands


4 Start eclipse and select a eclipse-workspace.
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. image:: images/eclipse_work_dir.png
   :target: images/eclipse_work_dir.png
   :alt: eclipse_work_dir
   
Open the Git View

.. image:: images/eclipse-open-git-view.png
   :target: images/eclipse-open-git-view.png
   :alt: eclipse-open-git-view

5 Add the my_package git repository you created to the git view.
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. image:: images/add-existing-git-to-eclipse-view.png
   :target: images/add-existing-git-to-eclipse-view.png
   :alt: add-existing-git-to-eclipse-view
   
Select the my_package you just created before.

.. image:: images/eclipse-search-and-select-git-repo.png
   :target: images/eclipse-search-and-select-git-repo.png
   :alt: eclipse-search-and-select-git-repo
