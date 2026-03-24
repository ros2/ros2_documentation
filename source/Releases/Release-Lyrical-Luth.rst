.. _upcoming-release:

.. _lyrical-release:

Lyrical Luth (codename 'lyrical'; May, 2026)
============================================

.. contents:: Table of Contents
   :depth: 2
   :local:

*Lyrical Luth* is the twelfth release of ROS 2.
What follows is highlights of the important changes and features in Lyrical Luth since the last release.

Supported Platforms
-------------------

Lyrical Luth supports the following platforms according to `the platform support tiers <../The-ROS2-Project/Platform-Support-Tiers>`:

Tier 1 platforms:

* TODO

Tier 2 platforms:

* TODO

Tier 3 platforms:

* TODO

For more information about RMW implementations, compiler / interpreter versions, and system dependency versions see `REP 2000 <https://reps.openrobotics.org/rep-2000/>`__.

Installation
------------

TODO

New features in this ROS 2 release
----------------------------------

``class_loader``
^^^^^^^^^^^^^^^^

Add support for passing arguments to constructors.
As a result, users can now create plugins that are not default constructible, removing the need for initialize method.

See https://github.com/ros/class_loader/pull/223 for more details.

``plugin_lib``
^^^^^^^^^^^^^^

Add support for passing arguments to constructors.

See https://github.com/ros/pluginlib/pull/291 for more details.

``image_transport``
^^^^^^^^^^^^^^^^^^^

``image_transport`` now supports lifecycle nodes.

See https://github.com/ros-perception/image_common/pull/352 for more details.

``point_cloud_transport``
^^^^^^^^^^^^^^^^^^^^^^^^^

``point_cloud_transport`` now supports lifecycle nodes.

See https://github.com/ros-perception/point_cloud_transport/pull/109 for more details.

``rcl_logging_implementation``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A new ``rcl_logging_implementation`` package has been introduced to allow users to select the logging backend implementation at runtime without rebuilding ``rcl``.
Users can set the ``RCL_LOGGING_IMPLEMENTATION`` environment variable to switch between available logging backends (e.g., ``rcl_logging_spdlog``, ``rcl_logging_noop``, or custom implementations).
If not specified, ``rcl_logging_spdlog`` is used by default.

See https://github.com/ros2/rcl/issues/1178, https://github.com/ros2/rcl/pull/1276, and https://github.com/ros2/rcl_logging/pull/135 for more details.

``rosidl_python``
^^^^^^^^^^^^^^^^^

Passing in Python ``set`` objects into array or sequence fields is now deprecated.
Instead pass in something that implements ``collections.abc.Sequence`` most commonly a ``list``, ``tuple``, or a ``numpy.ndarray``. To be removed in ROS M.

Development progress
--------------------

For progress on the development of Lyrical Luth, see `this project board <https://github.com/orgs/ros2/projects/70>`__.

For the broad process followed by Lyrical Luth, see the :doc:`process description page <Release-Process>`.
