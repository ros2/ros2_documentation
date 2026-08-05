.. meta::
   :contentType: how-to
   :experience: intermediate, expert
   :area: installation
   :distribution: {DISTRO}
   :product: {PRODUCT}

.. redirect-from::

   Working-with-GurumNetworks-GurumDDS
   Installation/RMW-Implementations/DDS-Implementations/Working-with-GurumNetworks-GurumDDS

Working with GurumNetworks GurumDDS - how-to
============================================

.. short-description::
   GurumNetworks GurumDDS is an alternative middleware implementation for ROS systems that need a supported Data Distribution Service (DDS) backend.
   This article describes how to install ``rmw_gurumdds``, select it as your active RMW implementation, and verify that it is working.
   After you follow these steps, you can run ROS nodes using GurumDDS.

``rmw_gurumdds`` is an implementation of the ROS middleware interface using GurumNetworks GurumDDS.
For more information about GurumDDS, visit the `GurumNetworks website <https://gurum.cc/index_eng>`_.

.. showmeta::
   :order: area, contentType, experience
   :labels: area=Area, contentType=Content type, experience=Level

.. contents:: Contents
   :depth: 2
   :local:

Prerequisites
-------------
This guide assumes you have completed the ROS 2 environment setup process, either by :doc:`Installing ROS 2 via Deb Packages <../../Ubuntu-Install-Debs>` or :doc:`Building ROS 2 from source on Ubuntu <../../Alternatives/Ubuntu-Development-Setup>`.

Version Requirements (`see the README for details <https://github.com/ros2/rmw_gurumdds>`_):

================  ================
ROS 2 Distro      GurumDDS Version
================  ================
rolling           ``>= 3.2.0``
lyrical           ``>= 3.2.0``
kilted            ``>= 3.2.0``
jazzy             ``>= 3.2.0``
humble            ``3.1.x``
================  ================

Deb packages of GurumDDS are provided in the ROS 2 apt repositories on Ubuntu.
Windows binary installer of GurumDDS will be available soon.

You can obtain a free trial license from the `GurumDDS Free Trial page <https://gurum.cc/free_trial_eng.html>`_.

After acquiring a license, place it in the following location: ``/etc/gurumnet``

Installation
------------
Option 1: Install from the ROS 2 apt repository (Recommended)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: console

   $ sudo apt install ros-{DISTRO}-rmw-gurumdds-cpp

This installs both ``rmw_gurumdds_cpp`` and ``gurumdds``.

Option 2: Build from source code
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
1. Clone the repository

.. code-block:: console

   $ cd ros2_ws/src
   $ git clone https://github.com/ros2/rmw_gurumdds -b {DISTRO} ros2/rmw_gurumdds

2. Install dependencies:

.. code-block:: console

   $ cd ..
   $ rosdep install --from src -i --rosdistro {DISTRO}

3. Build the workspace using Colcon:

.. code-block:: console

   $ colcon build --symlink-install

Switch to rmw_gurumdds
----------------------
Switch from other RMW implementations to rmw_gurumdds by setting the environment variable:

.. code-block:: console

   $ export RMW_IMPLEMENTATION=rmw_gurumdds_cpp

For more information on working with multiple RMW implementations, see :doc:`Working with multiple RMW implementations <../Working-with-multiple-RMW-implementations>`.

Testing the installation
------------------------
Run the ``talker`` and ``listener`` nodes to verify your installation:

.. code-block:: console

   $ source /opt/ros/{DISTRO}/setup.bash
   $ ros2 run demo_nodes_cpp talker

.. code-block:: console

   $ source /opt/ros/{DISTRO}/setup.bash
   $ ros2 run demo_nodes_cpp listener

If the nodes communicate successfully, your installation is working correctly.

.. note:: Remember to source your ROS 2 setup script before running these commands.
