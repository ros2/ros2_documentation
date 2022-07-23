Successive Releases for a Catkin Package
========================================

This guide explains how to release a new version of a ROS package that has already been released.

.. contents:: Table of Contents
   :depth: 3
   :local:

Before you start
----------------

.. include:: _Before-you-Start.rst

Install catkin_pkg
------------------

.. include:: _Install-Catkin-Package.rst

Install bloom
-------------

.. include:: _Install-Bloom.rst

Updating Changelog
------------------

For your users and for the developers, it is important to keep the changelog up to date.

.. code-block:: bash

   catkin_generate_changelog

.. include:: _Clean-Up-Changelog.rst

Bump the package version
------------------------

.. include:: _Bump-Package-Version.rst

Releasing Your Package
----------------------

.. note::

   If you have two factor authorization enabled on github, follow :doc:`Github Manual Authorization <Github-Manual-Authorization>` first.

Let bloom generate package artifacts for different platforms with the following command, where you should replace ``my_repo`` with the name of your repository:

.. code-block:: bash

   bloom-release --rosdistro {DISTRO} my_repo

Bloom will automatically create a pull request for you on `rosdistro <https://github.com/ros/rosdistro>`_.

.. tip::

   You don't have to do perform this step in a clone of the repository.

Next Steps
----------

Once your pull request has been submitted, one of the ROS developers will merge your request (this usually happens fairly quickly).
24-48 hours after that, your package should be built by the build farm and released into the building repository.
Packages built are periodically synchronized over to the `shadow-fixed <https://wiki.ros.org/ShadowRepository>`_ and public repositories, so it might take as long as a month before your package is available on the public ROS debian repositories (i.e. available via ``apt-get``).
To get updates on when the next synchronization (sync) is coming, check the `ROS discussion forums <https://discourse.ros.org/>`_.

Individual build details are on the Jenkins build farm `build.ros2.org <http://build.ros2.org/>`__.
Check `ROS {DISTRO} Default Package Status <http://repo.ros2.org/status_page/ros_{DISTRO}_default.html>`__ to see status of released packages.
