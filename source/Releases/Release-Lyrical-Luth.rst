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

Lyrical Luth is primarily supported on the following platforms:

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

``rosidl_python``
^^^^^^^^^^^^^^^^^

Passing in Python ``set`` objects into array or sequence fields is now deprecated.
Instead pass in something that implements ``collections.abc.Sequence`` most commonly a ``list``, ``tuple``, or a ``numpy.ndarray``. To be removed in ROS M.

Development progress
--------------------

For progress on the development of Lyrical Luth, see `this project board <https://github.com/orgs/ros2/projects/70>`__.

For the broad process followed by Lyrical Luth, see the :doc:`process description page <Release-Process>`.
