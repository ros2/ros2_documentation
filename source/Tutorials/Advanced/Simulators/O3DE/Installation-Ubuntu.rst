Installation (Ubuntu)
======================================

**Goal:** Install the package and run simulation examples on Ubuntu.

**Tutorial level:** Advanced

**Time:** 15 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
-------------

This tutorial will guide you through the steps to set up Open 3D Engine (O3DE) directly from the GitHub repository on a Linux system. This guide assumes you are not creating a fork of the repository.

Prerequisites
-------------

It is recommended to understand basic ROS principles covered in the beginner :doc:`../../../../Tutorials`.
In particular, :doc:`../../../Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace` and :doc:`../../../Beginner-Client-Libraries/Creating-Your-First-ROS2-Package` are useful prerequisites.

Before you begin, ensure you have the following installed on your Linux system:

- **Git**: To clone the O3DE repository.
- **CMake (version 3.20 or later)**: For building the project.
- **Ninja or Make**: Build systems supported by O3DE.
- **Python (version 3.7 or later)**: Required for various scripts.
- **Clang (version 12 or later)**: The recommended compiler for O3DE.


Setting up O3DE from GitHub on Linux
------------------------------------
Step 1: Install Required Dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

First, install the necessary dependencies by running the following commands in your terminal:

.. code-block:: bash

   sudo apt-get update
   sudo apt-get install -y build-essential ninja-build python3 python3-pip python3-venv \
                           libglu1-mesa-dev libxcb-xinerama0 libxcb-xinput0 libxcb-xinput-dev \
                           libfontconfig1 libssl-dev uuid-dev clang lld

Step 2: Clone the O3DE Repository
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Next, clone the O3DE repository from GitHub:

.. code-block:: bash

   git clone https://github.com/o3de/o3de.git
   cd o3de

This command downloads the O3DE source code into a directory named ``o3de`` and changes the current working directory to it.

Step 3: Configure the O3DE Project
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Run the ``cmake`` command to configure the project. This will generate the necessary build files in the ``build`` directory.

.. code-block:: bash

   cmake -B build/ -S . -G "Ninja Multi-Config"

Here’s what each argument does:

- ``-B build/``: Specifies the output directory for the build files.
- ``-S .``: Specifies the source directory (current directory).
- ``-G "Ninja Multi-Config"``: Specifies Ninja as the build system with multi-config support.

Step 4: Build O3DE
^^^^^^^^^^^^^^^^^^

Now, build O3DE using the ``cmake`` command:

.. code-block:: bash

   cmake --build build/ --config profile

This command builds O3DE in ``profile`` mode, which is recommended for development. You can replace ``profile`` with ``debug`` or ``release`` depending on your needs.

Step 5: Set Up the Project Environment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Before launching the O3DE Editor, you need to set up the project environment. Run the following script to do so:

.. code-block:: bash

   ./scripts/o3de.sh register --this-engine

This command registers the engine, allowing you to create and manage projects with O3DE.

Step 6: Create or Open a Project
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

With O3DE set up, you can create a new project or open an existing one.
To create a new project:

.. code-block:: bash

   ./scripts/o3de.sh create-project --project-path <path-to-your-project> --template Default

Replace ``<path-to-your-project>`` with the desired directory for your new project.

To open an existing project, navigate to the project directory and use the following command:

.. code-block:: bash

   ./scripts/o3de.sh edit-project --project-path <path-to-existing-project>

Step 7: Launch the O3DE Editor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Finally, launch the O3DE Editor by running:

.. code-block:: bash

   ./build/bin/profile/Editor

This command starts the O3DE Editor in ``profile`` mode.


Install ``o3de-extras``
-----------------------

Clone the Repository
^^^^^^^^^^^^^^^^^^^^
To get started, clone the ``o3de-extras`` repository:

.. code-block:: bash

   git clone https://github.com/o3de/o3de-extras

Setting up o3de-extras
^^^^^^^^^^^^^^^^^^^^^^
The ``o3de-extras`` repository can be cloned to any location on your local machine. Once cloned, you need to inform O3DE about the location of the extra assets in this repository by registering them. From the O3DE repository folder, you can register some or all of the extra assets using the ``o3de register`` command. Since these are optional assets, you may choose to register only those that you need. For example, to register a specific gem, use the following command:

.. code-block:: bash

   scripts\o3de.bat register --gem-path <o3de-extras>/Gems/<gem name>

If you want to register all the gems, you can do so since the repository follows the standard O3DE compound repository structure, with all gems located in the ``<o3de-extras>/Gems`` directory. To register all gems at once, use:

.. code-block:: bash

   scripts\o3de.bat register --all-gems-path <o3de-extras>/Gems

This process can be repeated for any other object types, if they exist:

.. code-block:: bash

   scripts\o3de.bat register --all-engines-path <o3de-extras>/Engines
   scripts\o3de.bat register --all-projects-path <o3de-extras>/Projects
   scripts\o3de.bat register --all-gems-path <o3de-extras>/Gems
   scripts\o3de.bat register --all-templates-path <o3de-extras>/Templates
   scripts\o3de.bat register --all-restricted-path <o3de-extras>/Restricted

If you've registered a gem, which functions like a plugin or component within a project, and you wish to use it in your project, you need to enable it by using the ``o3de enable-gem`` command:

.. code-block:: bash

   scripts\o3de.bat enable-gem --gem-name <gem name> --project-name <project name>