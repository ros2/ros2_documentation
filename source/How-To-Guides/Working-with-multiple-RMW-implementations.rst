.. redirect-from::

    Working-with-multiple-RMW-implementations
    Guides/Working-with-multiple-RMW-implementations
    Tutorials/Working-with-multiple-RMW-implementations

Working with multiple ROS 2 middleware implementations
======================================================

.. contents:: Table of Contents
   :depth: 2
   :local:

This page explains the default RMW implementation and how to specify an alternative.

Prerequisites
--------------

You should have already read the :doc:`DDS and ROS middleware implementations page <../Concepts/Intermediate/About-Different-Middleware-Vendors>`.

Specifying RMW implementations
------------------------------

To have multiple RMW implementations available for use you must have installed the ROS 2 binaries and any additional dependencies for specific RMW implementations, or built ROS 2 from source with multiple RMW implementations in the workspace (the RMW implementations are included in the build by default if their compile-time dependencies are met). See :doc:`Install DDS implementations <../Installation/DDS-Implementations>`.

----

Both C++ and Python nodes support an environment variable ``RMW_IMPLEMENTATION`` that allows the user to select the RMW implementation to use when running ROS 2 applications.

The user may set this variable to a specific implementation identifier, such as ``rmw_cyclonedds_cpp``, ``rmw_fastrtps_cpp``, ``rmw_connextdds``, or ``rmw_gurumdds_cpp``.

For example, to run the talker demo using the C++ talker and Python listener with the Connext RMW implementation:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

       RMW_IMPLEMENTATION=rmw_connextdds ros2 run demo_nodes_cpp talker

       # Run in another terminal
       RMW_IMPLEMENTATION=rmw_connextdds ros2 run demo_nodes_py listener

  .. group-tab:: macOS

    .. code-block:: bash

       RMW_IMPLEMENTATION=rmw_connextdds ros2 run demo_nodes_cpp talker

       # Run in another terminal
       RMW_IMPLEMENTATION=rmw_connextdds ros2 run demo_nodes_py listener

  .. group-tab:: Windows

    .. code-block:: bat

       set RMW_IMPLEMENTATION=rmw_connextdds
       ros2 run demo_nodes_cpp talker

       REM run in another terminal
       set RMW_IMPLEMENTATION=rmw_connextdds
       ros2 run demo_nodes_py listener

Adding RMW implementations to your workspace
--------------------------------------------

Suppose that you have built your ROS 2 workspace with only Fast DDS installed and therefore only the Fast DDS RMW implementation built.
The last time your workspace was built, any other RMW implementation packages, ``rmw_connextdds`` for example, were probably unable to find installations of the relevant DDS implementations.
If you then install an additional DDS implementation, Connext for example, you will need to re-trigger the check for a Connext installation that occurs when the Connext RMW implementation is being built.
You can do this by specifying the ``--cmake-clean-cache`` flag on your next workspace build, and you should see that the RMW implementation package then gets built for the newly installed DDS implementation.

It is possible to run into a problem when "rebuilding" the workspace with an additional RMW implementation using the ``--cmake-clean-cache`` option where the build complains about the default RMW implementation changing.
To resolve this, you can either set the default implementation to what is was before with the ``RMW_IMPLEMENTATION`` CMake argument or you can delete the build folder for packages that complain and continue the build with ``--packages-start <package name>``.

Troubleshooting
---------------

Checking the Current RMW
^^^^^^^^^^^^^^^^^^^^^^^^

To check the RMW that is currently in use you simply check the ``RMW_IMPLEMENTATION`` environment variable. On Linux systems ``printenv`` prints the full list of environment variables.
Other operating systems will have other procedures for viewing environment variables.
If ``RMW_IMPLEMENTATION`` is not in the environment it is safe to assume you are using the default for your ROS distro, otherwise the current RMW is the value listed.
The default RMW for each ROS Distro can be found in `REP-2000 <https://www.ros.org/reps/rep-2000.html#platforms-by-distribution>`_.

Ensuring use of a particular RMW implementation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If the ``RMW_IMPLEMENTATION`` environment variable is set to an RMW implementation for which support is not installed, you will see an error message similar to the following if you have only one implementation installed:

.. code-block:: bash

   Expected RMW implementation identifier of 'rmw_connextdds' but instead found 'rmw_fastrtps_cpp', exiting with 102.

If you have support for multiple RMW implementations installed and you request use of one that is not installed, you will see something similar to:

.. code-block:: bash

   Error getting RMW implementation identifier / RMW implementation not installed (expected identifier of 'rmw_connextdds'), exiting with 1.

If this occurs, double check that your ROS 2 installation includes support for the RMW implementation that you have specified in the ``RMW_IMPLEMENTATION`` environment variable.

If you want to switch between RMW implementations, verify that the ROS 2 daemon process is not running with the previous RMW implementation to avoid any issues between nodes and command line tools such as ``ros2 node``.
For example, if you run:

.. code-block:: bash

   RMW_IMPLEMENTATION=rmw_connextdds ros2 run demo_nodes_cpp talker

and

.. code-block:: bash

   ros2 node list

it will generate a daemon with a Fast DDS implementation:

.. code-block:: bash

   21318 22.0  0.6 535896 55044 pts/8    Sl   16:14   0:00 /usr/bin/python3 /opt/ros/{DISTRO}/bin/_ros2_daemon --rmw-implementation rmw_fastrtps_cpp --ros-domain-id 0

Even if you run the command line tool again with the correct RMW implementation, the daemon's RMW implementation will not change and the ROS 2 command line tools will fail.

To solve this, simply stop the daemon process:

.. code-block:: bash

   ros2 daemon stop

and rerun the ROS 2 command line tool with the correct RMW implementation.

RTI Connext on OSX: Failure due to insufficient shared memory kernel settings
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you receive an error message similar to below when running RTI Connext on OSX:

.. code-block:: console

   [D0062|ENABLE]DDS_DomainParticipantPresentation_reserve_participant_index_entryports:!enable reserve participant index
   [D0062|ENABLE]DDS_DomainParticipant_reserve_participant_index_entryports:Unusable shared memory transport. For a more in-   depth explanation of the possible problem and solution, please visit https://community.rti.com/kb/osx510.

This error is caused by an insufficient number or size of shared memory segments allowed by the operating system. As a result, the ``DomainParticipant`` is unable to allocate enough resources and calculate its participant index which causes the error.

You can increase the shared memory resources of your machine either temporarily or permanently.

To increase the settings temporarily, you can run the following commands as user root:

.. code-block:: console

   /usr/sbin/sysctl -w kern.sysv.shmmax=419430400
   /usr/sbin/sysctl -w kern.sysv.shmmin=1
   /usr/sbin/sysctl -w kern.sysv.shmmni=128
   /usr/sbin/sysctl -w kern.sysv.shmseg=1024
   /usr/sbin/sysctl -w kern.sysv.shmall=262144

To increase the settings permanently, you will need to edit or create the file ``/etc/sysctl.conf``. Creating or editing this file will require root permissions. Either add to your existing ``etc/sysctl.conf`` file or create ``/etc/sysctl.conf`` with the following lines:

.. code-block:: console

   kern.sysv.shmmax=419430400
   kern.sysv.shmmin=1
   kern.sysv.shmmni=128
   kern.sysv.shmseg=1024
   kern.sysv.shmall=262144

You will need to reboot the machine after modifying this file to have the changes take effect.

This solution is edited from the RTI Connext community forum.
See the `original post <https://community.rti.com/kb/osx510>`__ for more detailed explanation.
