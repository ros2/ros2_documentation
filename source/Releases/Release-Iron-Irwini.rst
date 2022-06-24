.. _upcoming-release:

.. _iron-release:

.. move this directive when next release page is created

ROS 2 Iron Irwini (codename 'iron'; May, 2023)
==============================================

.. contents:: Table of Contents
   :depth: 2
   :local:

*Iron Irwini* is the ninth release of ROS 2.
What follows is highlights of the important changes and features in Iron Irwini since the last release.

Supported Platforms
-------------------

Iron Irwini is primarily supported on the following platforms:

Tier 1 platforms:

TBD

Tier 2 platforms:

TBD

Tier 3 platforms:

TBD

For more information about RMW implementations, compiler / interpreter versions, and system dependency versions see `REP 2000 <https://www.ros.org/reps/rep-2000.html>`__.

Installation
------------

To come.

New features in this ROS 2 release
----------------------------------

To come.

Changes since the Humble release
----------------------------------

Rename ``spin_until_future_complete`` to ``spin_until_complete``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`PR 1874 <https://github.com/ros2/rclcpp/pull/1874>`_ renames ``spin_until_future_complete`` to ``spin_until_complete`` to represent the semantics of being able to spin on values that are not exclusively futures.
The API can now spin until arbitrary conditions.
A deprecation warning will appear for migration.


Known Issues
------------

To come.

Release Timeline
----------------

To come.
