.. redirect-from::

    Tutorials/Launch-Files/Launch-Main
    Tutorials/Launch/Launch-Main
    Tutorials/Intermediate/Launch/Launch-Main

.. meta::
   :contentType: learning-path
   :experience: intermediate
   :area: node-management, tools
   :distribution: {DISTRO}
   :product: {PRODUCT}

.. _LaunchFilesMain:

Working with launch files - tutorial
====================================

.. short-description::
   Launch files allow you to start and configure several executables containing ROS nodes at the same time.
   This article introduces the launch file tutorials and the main tools used to build reusable launch systems.
   After reading it, you can choose the right launch tutorial for creating, extending, and organizing launch files.

.. showmeta::
   :order: area, contentType, experience
   :labels: area=Area, contentType=Content type, experience=Level

.. contents:: Table of Contents
   :depth: 2
   :local:

ROS 2 Launch files allow you to start up and configure a number of executables containing ROS 2 nodes simultaneously.

.. toctree::
   :hidden:

   Creating-Launch-Files
   Launch-system
   Using-Substitutions
   Using-Event-Handlers
   Using-ROS2-Launch-For-Large-Projects

#. :doc:`Creating a launch file <Creating-Launch-Files>`.

   Learn how to create a launch file that will start up nodes and their configurations all at once.

#. :doc:`Launching and monitoring multiple nodes <Launch-system>`.

   Get a more advanced overview of how launch files work.

#. :doc:`Using substitutions <Using-Substitutions>`.

   Use substitutions to provide more flexibility when describing reusable launch files.

#. :doc:`Using event handlers <Using-Event-Handlers>`.

   Use event handlers to monitor the state of processes or to define a complex set of rules that can be used to dynamically modify the launch file.

#. :doc:`Managing large projects <Using-ROS2-Launch-For-Large-Projects>`.

   Structure launch files for large projects so they may be reused as much as possible in different situations.
   See usage examples of different launch tools like parameters, YAML files, remappings, namespaces, default arguments, and RViz configs.

.. note::

   If you are coming from ROS 1, you can use the :doc:`ROS Launch Migration guide <../../Migration-and-Upgrades/Migrating-from-ROS1/Migrating-Launch-Files>` to help you migrate your launch files to ROS 2.
