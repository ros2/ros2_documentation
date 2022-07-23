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

.. include:: _Next-Steps.rst
