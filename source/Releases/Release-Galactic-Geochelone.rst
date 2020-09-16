.. _upcoming-release:

.. move this directive when next release page is created

ROS 2 Galactic Geochelone (codename 'galactic'; May, 2021)
==========================================================

.. contents:: Table of Contents
   :depth: 2
   :local:

*Galactic Geochelone* is the seventh release of ROS 2.

Supported Platforms
-------------------

TBD

Installation
------------

TBD

New features in this ROS 2 release
----------------------------------

Ability to specify per-logger log levels
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

It is now possible to specify different logging levels for different loggers on the command line:

.. code-block:: bash

   ros2 run demo_nodes_cpp talker --ros-args --log-level WARN --log-level talker:=DEBUG

The above command sets a global log level of WARN, but sets the log level of the talker node messages to DEBUG.
The ``--log-level`` command-line option can be passed an arbitrary number of times to set different log levels for each logger.


Changes since the Foxy release
------------------------------

tf2_ros Python split out of tf2_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The Python code that used to live in tf2_ros has been moved into its own package named tf2_ros_py.
Any existing Python code that depends on tf2_ros will continue to work, but the package.xml of those packages should be amended to ``exec_depend`` on tf2_ros_py.

rclcpp
^^^^^^

Change in spin_until_future_complete template parameters
""""""""""""""""""""""""""""""""""""""""""""""""""""""""

The first template parameter of ``Executor::spin_until_future_complete`` was the future result type ``ResultT``, and the method only accepted a ``std::shared_future<ResultT>``.
In order to accept other types of futures (e.g.: ``std::future``), that parameter was changed to the future type itself.

In places where a ``spin_until_future_complete`` call was relying on template argument deduction, no change is needed.
If not, this is an example diff:

.. code-block:: dpatch

   std::shared_future<MyResultT> future;
   ...
   -executor.spin_until_future_complete<MyResultT>(future);
   +executor.spin_until_future_complete<std::shared_future<MyResultT>>(future);


For more details, see `ros2/rclcpp#1160 <https://github.com/ros2/rclcpp/pull/1160>`_.
For an example of the needed changes in user code, see `ros-visualization/interactive_markers#72 <https://github.com/ros-visualization/interactive_markers/pull/72>`_.

Known Issues
------------

Timeline before the release
---------------------------

    Mon. March 15, 2021 - Freeze
        API and feature freeze for ``ros_core`` [1]_ packages in Rolling Ridley.
        Note that this includes ``rmw``, which is a recursive dependency of ``ros_core``.
        Only bug fix releases should be made after this point.
        New packages can be released independently.

    Mon. March 29, 2021 - Alpha
        Preliminary testing and stabilization of ``ros_core`` [1]_ packages.
    
    Mon. April 12, 2021 - Branch
        Branch from Rolling Ridley.
        ``rosdistro`` is reopened for Rolling PRs for ``ros_core`` [1]_ packages.
        Galactic development shifts from ``ros-rolling-*`` packages to ``ros-galactic-*`` packages.
         
    Mon. April 19, 2021 - Beta
        Updated releases of ``desktop`` [2]_ packages available.
        Call for general testing.
         
    Mon. May 17, 2021 - RC
        Relese Candidate packages are built.
        Updated releases of ``desktop`` [2]_ packages available.

    Thu. May 20, 2021 - Distro Freeze
        Freeze rosdistro.
        No PRs for Galactic on the `rosdistro` repo will be merged (reopens after the release announcement).

    Sun. May 23, 2021 - General Availability
        Release announcement.
        ``rosdistro`` is reopened for Galactic PRs.

.. [1] The ``ros_core`` variant described in the `variants <https://github.com/ros2/variants>`_ repository.
.. [2] The ``desktop`` variant described in the `variants <https://github.com/ros2/variants>`_ repository.
