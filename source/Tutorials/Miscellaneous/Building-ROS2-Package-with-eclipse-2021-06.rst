.. redirect-from::

    Tutorials/Building-ROS2-Package-with-eclipse-2021-06

Building a package with Eclipse 2021-06
=======================================

.. contents:: Table of Contents
   :depth: 2
   :local:

You cannot create a ROS 2 package with eclipse, you need to create it with commandline tools.
Follow the :doc:`Create a package <../Beginner-Client-Libraries/Creating-Your-First-ROS2-Package>` tutorial.

After you created your project, you can edit the source code and build it with eclipse.

We start eclipse and select a eclipse-workspace.

.. image:: images/eclipse_work_dir.png
   :target: images/eclipse_work_dir.png
   :alt: eclipse_work_dir

We create a C++ project

.. image:: images/eclipse_create_c++_project.png
   :target: images/eclipse_create_c++_project.png
   :alt: eclipse_create_c++_project


.. image:: images/eclipse_c++_project_select_type.png
   :target: images/eclipse_c++_project_select_type.png
   :alt: eclipse_c++_project_select_type

We see that we got C++ includes.

.. image:: images/eclipse_c++_project_includes.png
   :target: images/eclipse_c++_project_includes.png
   :alt: eclipse_c++_project_includes


We now import our ROS 2 project. The code is still in the old place.

.. image:: images/eclipse_import_project.png
   :target: images/eclipse_import_project.png
   :alt: eclipse_import_project

.. image:: images/eclipse_import_filesystem.png
   :target: images/eclipse_import_filesystem.png
   :alt: eclipse_import_filesystem


.. image:: images/eclipse_import_select_my_package.png
   :target: images/eclipse_import_select_my_package.png
   :alt: eclipse_import_select_my_package



We see in the source code that the C++ includes got resolved but not the ROS 2 ones.

.. image:: images/eclipse_c++_wo_ros_includes.png
   :target: images/eclipse_c++_wo_ros_includes.png
   :alt: eclipse_c++_wo_ros_includes


.. image:: images/eclipse_c++_path_and_symbols.png
   :target: images/eclipse_c++_path_and_symbols.png
   :alt: eclipse_c++_path_and_symbols


.. image:: images/eclipse_c++_add_directory_path.png
   :target: images/eclipse_c++_add_directory_path.png
   :alt: eclipse_c++_add_directory_path


We now see that the ROS 2 includes got resolved too.

.. image:: images/eclipse_c++_indexer_ok.png
   :target: images/eclipse_c++_indexer_ok.png
   :alt: eclipse_c++_indexer_ok


Adding Builder colcon, so that we can build with right-click on project and "Build project".

.. image:: images/eclipse_c++_properties_builders.png
   :target: images/eclipse_c++_properties_builders.png
   :alt: eclipse_c++_properties_builders


.. image:: images/eclipse_c++_builder_main.png
   :target: images/eclipse_c++_builder_main.png
   :alt: eclipse_c++_builder_main


With PYTHONPATH you can also build python projects.

.. image:: images/eclipse_c++_builder_env.png
   :target: images/eclipse_c++_builder_env.png
   :alt: eclipse_c++_builder_env


.. image:: images/eclipse_c++_properties_builders_with_colcon.png
   :target: images/eclipse_c++_properties_builders_with_colcon.png
   :alt: eclipse_c++_properties_builders_with_colcon


Right-click on the project and select "Build Project".

.. image:: images/eclipse_c++_build_project_with_colcon.png
   :target: images/eclipse_c++_build_project_with_colcon.png
   :alt: eclipse_c++_build_project_with_colcon
