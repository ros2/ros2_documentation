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

        
Create your package my_package, and create a git repository from it.
If we create a git repo from it, we can easily use it in eclipse, also
we got directly a history of our changes.

.. image:: images/create-package-add-git.png
   :target: images/create-package-add-git.png
   :alt: create-package-add-git
   
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
   
Now you got your ROS 2 package in eclipse git view.

.. image:: images/eclipse-selected-git-repo-in-view.png
   :target: images/eclipse-selected-git-repo-in-view.png
   :alt: eclipse-selected-git-repo-in-view
   
6 To edit the files in this git repository, we import it.
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. image:: images/eclipse-import-project-from-git-view.png
   :target: images/eclipse-import-project-from-git-view.png
   :alt: eclipse-import-project-from-git-view
   
.. image:: images/eclipse-select-import-git-view-project.png
   :target: images/eclipse-select-import-git-view-project.png
   :alt: eclipse-select-import-git-view-project
   

You can now see, edit all files in project explorer.

.. image:: images/eclipse-git-project-in-project-explorer.png
   :target: images/eclipse-git-project-in-project-explorer.png
   :alt: eclipse-git-project-in-project-explorer
   
7 To get all C++ includes resolved, convert the project to C++ to add C++ nature.
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. image:: images/eclipse-convert-to-c++-project.png
   :target: images/eclipse-convert-to-c++-project.png
   :alt: eclipse-convert-to-c++-project
   
.. image:: images/eclipse-convert-to-c++-select.png
   :target: images/eclipse-convert-to-c++-select.png
   :alt: eclipse-convert-to-c++-select
   
Now you can see the added includes in the project explorer view.

.. image:: images/eclipse-c++-includes.png
   :target: images/eclipse-c++-includes.png
   :alt: eclipse-c++-includes
   
   
8 Adding ROS 2 include path
^^^^^^^^^^^^^^^^^^^^^^^^^^^

The C++ nature also allows you now to set include path. Right-click on your
project in project explorer and select "Properties".

.. image:: images/eclipse_c++_path_and_symbols.png
   :target: images/eclipse_c++_path_and_symbols.png
   :alt: eclipse_c++_path_and_symbols
   
.. image:: images/eclipse_c++_add_directory_path.png
   :target: images/eclipse_c++_add_directory_path.png
   :alt: eclipse_c++_add_directory_path


9 Adding colcon as Builder
^^^^^^^^^^^^^^^^^^^^^^^^^^

To build the project with right-click on  project and select "Build Project", we
setup a builder. Right-click on your project and select "Properties".

.. image:: images/eclipse_c++_properties_builders.png
   :target: images/eclipse_c++_properties_builders.png
   :alt: eclipse_c++_properties_builders
   
Click "Add" and use Program.
  
.. image:: images/eclipse_c++_builder_main.png
   :target: images/eclipse_c++_builder_main.png
   :alt: eclipse_c++_builder_main
   
.. image:: images/eclipse_c++_builder_env.png
   :target: images/eclipse_c++_builder_env.png
   :alt: eclipse_c++_builder_env 
   
   
Now it should look like this.

.. image:: images/eclipse_c++_properties_builders_with_colcon.png
   :target: images/eclipse_c++_properties_builders_with_colcon.png
   :alt: eclipse_c++_properties_builders_with_colcon 

