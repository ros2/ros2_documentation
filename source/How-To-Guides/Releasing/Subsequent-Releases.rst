Subsequent Releases
===================

This guide explains how to release a new version of a ROS package that has already been released.

.. contents:: Table of Contents
   :depth: 3
   :local:

Be part of the release team
---------------------------

If you are not part of the release team for the repository, follow :ref:`Join a release team <join-a-release-team>`.

Install dependencies
--------------------

.. include:: _Install-Dependencies.rst

Ensure repositories are up-to-date
----------------------------------

.. include:: _Ensure-Repositories-Are-Up-To-Date.rst

Updating Changelog
------------------

For your users and for the developers, it is important to keep the changelog up to date.

.. code-block:: bash

   catkin_generate_changelog

.. include:: _Clean-Up-Changelog.rst

Bump the package version
------------------------

.. include:: _Bump-Package-Version.rst

Bloom Release
-------------

.. note::

   If you have two factor authorization enabled on GitHub, follow :doc:`GitHub Manual Authorization <GitHub-Manual-Authorization>` first.

Run the following command, replacing ``my_repo`` with the name of your repository:

.. code-block:: bash

   bloom-release --rosdistro {DISTRO} my_repo

Bloom will automatically create a pull request for you against `rosdistro <https://github.com/ros/rosdistro>`_.

Next Steps
----------

.. include:: _Next-Steps.rst
