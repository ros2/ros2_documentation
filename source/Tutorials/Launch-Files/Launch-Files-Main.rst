.. _LaunchFilesMain:

Launch File Tutorials
=====================

ROS 2 Launch files allow you to start up and configure a number of executables containing ROS 2 nodes simultaneously.

.. toctree::
   :hidden:

   Creating-Launch-Files
   Launch-system
   Using-Substitutions
   Using-Event-Handlers
   Using-ROS2-Launch-For-Large-Projects

Learning Launch
---------------

#. :doc:`Creating a ROS 2 Launch File <./Creating-Launch-Files>`.

   This tutorial will show you how to create a launch file that will start up nodes and their configurations all at once.

#. :doc:`Launching and Monitor Multiple Nodes with Launch <./Launch-system>`.

   A more advanced overview of how launch files work and of the resources and documentation available for them.

#. :doc:`Using Event Handlers <./Using-Event-Handlers>`.

   Event handlers can be useful for monitoring the state of processes or to define a complex set of rules that can be used to dynamically modify the launch file.

#. :doc:`Using Substitutions <./Using-Substitutions>`.

   Substitutions can be used to provide more flexibility when describing reusable launch files.

#. :doc:`Using ROS 2 Launch For Large Projects <./Launch-system>`.

   This tutorial describes some tips for writing launch files for large projects.
   The focus is on how to structure launch files so they may be reused as much as possible in different situations.
   Additionally, it covers usage examples of different ROS 2 launch tools, like parameters, YAML files, remappings, namespaces, default arguments, and RViz configs.

.. note::

  You can also use XML and YAML to create launch files.
  You can see a comparison of these different ROS 2 launch formats in :doc:`../../How-To-Guides/Launch-file-different-formats`.

.. note::

   If you are coming from ROS 1, you can use the :doc:`ROS Launch Migration guide <../../How-To-Guides/Launch-files-migration-guide>` to help you migrate your launch files to ROS 2.
