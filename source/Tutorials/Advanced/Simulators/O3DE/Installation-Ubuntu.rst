Installation (Ubuntu)
======================================

**Goal:** Install the package and run simulation examples on Ubuntu.

**Tutorial level:** Advanced

**Time:** 10 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background


Prerequisites
-------------

It is recommended to understand basic ROS principles covered in the beginner :doc:`../../../../Tutorials`.
In particular, :doc:`../../../Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace` and :doc:`../../../Beginner-Client-Libraries/Creating-Your-First-ROS2-Package` are useful prerequisites.

To install the O3DE software, you should follow the `installation procedure <https://www.docs.o3de.org/docs/welcome-guide/setup/installing-linux/>`_ from the official O3DE website.


Install ``o3de-extras``
------------------------

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