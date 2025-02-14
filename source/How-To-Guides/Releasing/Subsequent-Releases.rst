Subsequent Releases
===================

This guide explains how to release new versions of ROS packages that have already been released before.

.. contents:: Table of Contents
   :depth: 1
   :local:

Be part of the release team
---------------------------

If you are not part of the release team that has write access to the release repository, follow :ref:`Join a release team <join-a-release-team>`.

Install dependencies
--------------------

.. include:: _Install-Dependencies.rst

Set up a Personal Access Token
------------------------------

.. include:: _Personal-Access-Token.rst

Ensure repositories are up-to-date
----------------------------------

.. include:: _Ensure-Repositories-Are-Up-To-Date.rst

Updating Changelog
------------------

For your users and for the developers, keep the changelog concise and up to date.

.. code-block:: bash

   catkin_generate_changelog

.. include:: _Clean-Up-Changelog.rst

Bump the package version
------------------------

.. include:: _Bump-Package-Version.rst

Bloom Release
-------------

Run the following command, replacing ``my_repo`` with the name of your repository with the packages:

.. code-block:: bash

   bloom-release --rosdistro {DISTRO} my_repo

Bloom will automatically create a pull request for you against `rosdistro <https://github.com/ros/rosdistro>`_.

.. note::

  By default, bloom will release all packages in the source repository.
  To selectively block the release of some packages for a particular ``{DISTRO}``, add ``{DISTRO}.ignored`` files to the ``master``` branch of the release repository.
  In each file, list the name of the package, one per line, to block the release of the package.
  The `rosidl-release <https://github.com/ros2-gbp/rosidl-release>`_ repository may serve as a useful reference for this configuration.

Next Steps
----------

.. include:: _Next-Steps.rst
