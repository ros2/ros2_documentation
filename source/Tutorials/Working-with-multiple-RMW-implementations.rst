
Working with multiple ROS 2 middleware implementations
======================================================

.. contents:: Table of Contents
   :depth: 2
   :local:

This page explains the default RMW implementation and how to specify an alternative.

Pre-requisites
--------------

You should have already read the `DDS and ROS middleware implementations page <../Concepts/DDS-and-ROS-middleware-implementations>`.

Multiple RMW implementations
----------------------------

The current ROS 2 binary releases have built-in support for several RMW implementations out of the box (Fast RTPS, RTI Connext Pro, and ADLink OpenSplice at the time of writing), but only Fast RTPS (the default) works without any additional installation steps, because it is the only one we distribute with our binary packages.

Others like OpenSplice or Connext can be enabled by installing additional packages, but without having to rebuild anything or replace any existing packages.

Also, a ROS 2 workspace that has been built from source may build and install multiple RMW implementations simultaneously.
While the core ROS 2 code is being compiled, any RMW implementation that is found will be built if the relevant DDS/RTPS implementation has been installed properly and the relevant environment variables have been configured.
For example, if the code for the `RMW package for RTI Connext <https://github.com/ros2/rmw_connext/tree/master/rmw_connext_cpp>`__ is in the workspace, it will be built if an installation of RTI's Connext Pro can also be found.
For many cases you will find that nodes using different RMW implementations are able to communicate, however this is not true under all circumstances.
A list of supported inter-vendor communication configurations is forthcoming.

Default RMW implementation
--------------------------

If a ROS 2 workspace has multiple RMW implementations, the default RMW implementation is currently selected as Fast RTPS if it's available.
If the Fast RTPS RMW implementation is not installed, the RMW implementation with the first RMW implementation identifier in alphabetical order will be used.
The implementation identifier is the name of the ROS package that provides the RMW implementation, e.g. ``rmw_fastrtps_cpp``.
For example, if both ``rmw_opensplice_cpp`` and ``rmw_connext_cpp`` ROS packages are installed, ``rmw_connext_cpp`` would be the default.
If ``rmw_fastrtps_cpp`` is ever installed, it would be the default.
See below for how to specify which RMW implementation is to be used when running the ROS 2 examples.

Specifying RMW implementations
------------------------------

To have multiple RMW implementations available for use you must have installed our binaries and any additional dependencies for specific RMW implementations, or built ROS 2 from source with multiple RMW implementations in the workspace (they are included by default) and their dependencies are met (for example see `the Linux install instructions <linux-development-setup-install-more-dds-implementations-optional>`).

----

Starting in Beta 2 and above both C++ and Python nodes support an environment variable ``RMW_IMPLEMENTATION``.
To choose a different RMW implemenation you can set the environment variable ``RMW_IMPLEMENTATION`` to a specific implementation identifier.

To run the talker demo using the C++ and listener using python with the RMW implementation for connext:

*Bash*

.. code-block:: bash

   RMW_IMPLEMENTATION=rmw_connext_cpp ros2 run demo_nodes_cpp talker

   # Run in another terminal
   RMW_IMPLEMENTATION=rmw_connext_cpp ros2 run demo_nodes_py listener

*Windows cmd.exe*

.. code-block:: bat

   set RMW_IMPLEMENTATION=rmw_connext_cpp
   ros2 run demo_nodes_cpp talker

   REM run in another terminal
   set RMW_IMPLEMENTATION=rmw_connext_cpp
   ros2 run demo_nodes_py listener

Adding RMW implementations to your workspace
--------------------------------------------

Suppose that you have built your ROS 2 workspace with only Fast RTPS installed and therefore only the Fast RTPS RMW implementation built.
The last time your workspace was built, any other RMW implementation packages, ``rmw_connext_cpp`` for example, were probably unable to find installations of the relevant DDS implementations.
If you then install an additional DDS implementation, Connext for example, you will need to re-trigger the check for a Connext installation that occurs when the Connext RMW implementation is being built.
You can do this by specifying the ``--force-cmake-configure`` flag on your next workspace build, and you should see that the RMW implementation package then gets built for the newly installed DDS implementation.

It is possible to run into a problem when "rebuilding" the workspace with an additional RMW implementation using the ``--force-cmake-configure`` option where the build complains about the default RMW implementation changing.
To resolve this, you can either set the default implementation to what is was before with the ``RMW_IMPLEMENTATION`` CMake argument or you can delete the build folder for packages that complain and continue the build with ``--start-with <package name>``.

Troubleshooting
---------------

Ensuring use of a particular RMW implementation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

ROS 2 Ardent and later
~~~~~~~~~~~~~~~~~~~~~~

If the ``RMW_IMPLEMENTATION`` environment variable is set to an RMW implementation for which support is not installed, you will see an error message similar to the following if you have only one implementation installed:

.. code-block:: bash

   Expected RMW implementation identifier of 'rmw_connext_cpp' but instead found 'rmw_fastrtps_cpp', exiting with 102.

If you have support for multiple RMW implementations installed and you request use of one that is not installed, you will see something similar to:

.. code-block:: bash

   Error getting RMW implementation identifier / RMW implementation not installed (expected identifier of 'rmw_connext_cpp'), exiting with 1.

If this occurs, double check that your ROS 2 installation includes support for the RMW implementation that you have specified in the ``RMW_IMPLEMENTATION`` environment variable.
