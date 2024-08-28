.. redirect-from::

    Tutorials/Simulators/O3DE/Setting-up-a-Robot-Simulation-O3DE
    Tutorials/Advanced/Simulators/O3DE

Setting up a robot simulation (Basic)
======================================

**Goal:** Setup a robot simulation and control it from ROS 2.

**Tutorial level:** Advanced

**Time:** 30 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

In this tutorial, you will use the Open 3D Engine (O3DE) to set up and run a basic ROS 2 simulation scenario.

The ``o3de-extras`` repository provides additional tools and templates for enhancing your O3DE projects with ROS 2 capabilities. In this tutorial, you will use the ``Ros2ProjectTemplate`` from the ``o3de-extras`` repository to create a project that integrates ROS 2 with O3DE for simulating a robot in a virtual environment. This template includes several example projects, but for this tutorial, you will focus on the ``slam_navigation`` example, which demonstrates how to simulate a robot performing SLAM (Simultaneous Localization and Mapping) and navigation tasks.

For more detailed examples and use cases, you can refer to the `O3DE ROS 2 Examples <https://github.com/o3de/o3de-extras/blob/development/Templates/Ros2ProjectTemplate/Template/Examples/slam_navigation/README.md>`_ page.


Prerequisites
-------------

Before you begin, ensure that you have the following installed:

- O3DE set up on your machine. For instructions, follow the `O3DE installation for Ubuntu <Installation-Ubuntu>` guide.
- ROS 2 (Foxy or later) installed on your system.
- The ``o3de-extras`` repository cloned locally (on the ``stabilization`` branch).


Creating a New ROS 2 Project
----------------------------

1. **Register the ROS 2 Project Template**:

   Navigate to your O3DE directory and register the ROS 2 Project Template from the ``o3de-extras`` repository:

   .. code-block:: bash

      ./scripts/o3de.sh register --all-templates-path <path-to-o3de-extras>/Templates

   This command registers all templates within the ``o3de-extras`` repository, including the ROS 2 Project Template.

2. **Create a New Project**:

   Create a new project using the ROS 2 Project Template:

   .. code-block:: bash

      ./scripts/o3de.sh create-project --project-name <project_name> --template-name Ros2ProjectTemplate --project-path <path-to-project-directory>

   Replace ``<project_name>`` with your desired project name and ``<path-to-project-directory>`` with the directory where you want the project to be created.

3. **Configure and build the Project**:

   After creating the project, you need to cofigure and build it:

   Navigate to your project directory:

   .. code-block:: bash

      cd <project_path>

   .. code-block:: bash

      cmake -B build/ -S . -G "Ninja Multi-Config"

   .. code-block:: bash

      cmake --build <path-to-build-directory> --target <project_name> Editor

   Ensure the build completes without errors.

Setting Up the SLAM Navigation Example
--------------------------------------

The ROS 2 Project Template includes several example projects. In this tutorial, you will use the SLAM navigation example to simulate a robot performing SLAM and navigation tasks.

1. **Navigate to the Example Directory**:

   The SLAM navigation example is located in the following directory:

   .. code-block:: bash

      <project-directory>/Examples/slam_navigation

2. **Run the Example**:

   Launch the example by opening the O3DE Editor:

   .. code-block:: bash

      <path-to-o3de-directory>/build/bin/profile/Editor

   Once in the Editor, open the SLAM navigation level by navigating to the ``Levels`` tab and selecting the SLAM navigation level.

   Press ``Ctrl+G`` to start the simulation.

3. **Launching ROS 2 Nodes**:

   In a new terminal, source your ROS 2 environment and launch the ROS 2 nodes required for SLAM and navigation:

   .. code-block:: bash

      source /opt/ros/foxy/setup.bash
      ros2 launch slam_navigation slam_navigation_launch.py

   This command starts the ROS 2 nodes, enabling the robot in the simulation to perform SLAM and navigation.

Exploring Further
-----------------

Now that your ROS 2 project is up and running in O3DE, you can explore and customize the simulation further. For more detailed examples and documentation, refer to the `O3DE ROS 2 Examples <https://github.com/o3de/o3de-extras/blob/development/Templates/Ros2ProjectTemplate/Template/Examples/slam_navigation/README.md>`_ page.
