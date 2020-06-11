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


New features in this ROS 2 release
----------------------------------

TBD

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
```cpp
std::shared_future<MyResultT> future;
...
-executor.spin_until_future_complete<MyResultT>(future);
+executor.spin_until_future_complete<std::shared_future<MyResultT>>(future);
```

For more details, see `[ros2/rclcpp#1160] <https://github.com/ros2/rclcpp/pull/1160>`_.
For an example of the needed changes in user code, see `[ros-visualization/interactive_markers#72] <https://github.com/ros-visualization/interactive_markers/pull/72>`_.

Known Issues
------------

Timeline before the release
---------------------------

TBD
