Setting up a robot simulation (Advanced)
========================================

**Goal:** Setup a robot simulation and control it from ROS 2.

**Tutorial level:** Advanced

**Time:** 30 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

In this tutorial, you will use the Open 3D Engine (O3DE) to set up and execute a ROS 2 robotic manipulation simulation.

The ``o3de-extras`` repository offers additional tools and templates to enhance O3DE projects with ROS 2 functionality. In this tutorial, you will use the ``Ros2RoboticManipulationTemplate`` from the ``o3de-extras`` repository. This template provides a foundation for integrating ROS 2 with O3DE to simulate robotic manipulation tasks in a virtual environment. It includes predefined configurations and examples to demonstrate how to control and interact with robotic manipulators within O3DE.

For detailed instructions and further examples on using the ROS 2 Robotic Manipulation Template, you can refer to the `O3DE ROS 2 Robotic Manipulation Template <https://github.com/o3de/o3de-extras/tree/development/Templates/Ros2RoboticManipulationTemplate>`_ page.


Prerequisites
-------------
Ensure you have the following before starting:

- O3DE set up on your machine. For instructions, follow the `O3DE installation for Ubuntu <Installation-Ubuntu>` guide.
- ROS 2 (Foxy or later) installed on your system.
- The ``o3de-extras`` repository cloned locally (on the ``stabilization`` branch).
- **MoveIt**, which is used for motion planning in ROS 2. Follow the MoveIt 2 documentation for installation instructions.


Creating a New ROS 2 Project
----------------------------

1. **Register the Template**:

Register the template with O3DE so that it can be used to create a new project.

From the root directory of your O3DE installation, run:

.. code-block:: bash

   ./scripts/o3de.sh register --all-templates-path <path_to_o3de_extras>/Templates

This command registers all templates in the ``o3de-extras`` repository, including the Robotic Manipulation Template.

2. **Create a New Project**:

Create a new project using the ROS 2 Robotic Manipulation Template:

.. code-block:: bash

   ./scripts/o3de.sh create-project --project-name <project_name> --template-name Ros2RoboticManipulationTemplate --project-path <path-to-project-directory>

This will generate a new project directory with the necessary files and configurations.

3. **Install Dependencies**:

Navigate to your project directory:

.. code-block:: bash

   cd <project_path>

Install the required Python packages and dependencies for the project. Typically, you will need to install ROS 2 and MoveIt dependencies. This can often be done with:

.. code-block:: bash

   sudo apt install ros-${ROS_DISTRO}-moveit ros-${ROS_DISTRO}-moveit-resources ros-${ROS_DISTRO}-depth-image-proc

Ensure that you also have any additional dependencies specified in the project's ``requirements.txt`` or equivalent configuration files.

4. **Configure and build the Project**:

After installing dependencies, cofigure and build the project using the following commands:

.. code-block:: bash

   cmake -B build/ -S . -G "Ninja Multi-Config"

.. code-block:: bash

   cmake --build <path-to-build-directory> --target <project_name> Editor

Ensure the build completes without errors.   


Configurations and launch of the project
----------------------------------------

1. **Configure ROS 2 and MoveIt**:

The template may include configuration files for ROS 2 and MoveIt. Ensure these are properly configured to match your simulation setup. Key files include:

- **ROS 2 Launch Files**: Typically found in the ``launch`` directory, configure these files to start the ROS 2 nodes required for your simulation.
- **MoveIt Configuration**: Check the ``moveit_config`` directory for MoveIt configuration files. Ensure these files are correctly set up for your robot and planning requirements.

2. **Launch the Simulation**:

Start the O3DE Editor:

.. code-block:: bash

   <path-to-o3de-directory>/build/bin/profile/Editor

In the O3DE Editor:

1. Open the example level provided by the template. Navigate to the ``File`` menu, select ``Open Level``, and choose the example level from the ``Levels`` directory.
2. Launch the ROS 2 nodes and MoveIt components required for the simulation. Typically, this can be done with:

.. code-block:: bash

   ros2 launch <your_package> <launch_file>.launch.py

3. **Simulate Robotic Manipulation**:

With the simulation running, you can interact with the robotic manipulator in the O3DE Editor. Test different manipulation tasks and adjust configurations as needed. Use the MoveIt interface to plan and execute robotic movements.

For further details on configuring ROS 2 and MoveIt for your specific needs, refer to the `MoveIt 2 Documentation`_ and the `O3DE Robotics Project Configuration`_ guide.

.. _MoveIt 2 Documentation: https://moveit.ros.org/documentation/
.. _O3DE Robotics Project Configuration: https://development--o3deorg.netlify.app/docs/user-guide/interactivity/robotics/project-configuration/
