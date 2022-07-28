First Time Release
==================

This guide explains how to release ROS 2 packages that you have not released before.
Due to numerous options available when releasing ROS packages, this guide intends to cover the most common scenario and does not cover every corner-case.

.. contents:: Table of Contents
   :depth: 1
   :local:

Be part of a release team
-------------------------

You must be part of a :ref:`release team <what-is-a-release-team>`.
If you are not part of a release team yet, follow either:

* :ref:`Join a release team <join-a-release-team>`
* :ref:`Start a new release team <start-a-new-release-team>`

Create a new release repository
-------------------------------

You need a :ref:`release repository <what-is-a-release-repository>` to release a package.
Follow :ref:`Create a new release repository <create-a-new-release-repository>`.

Install dependencies
--------------------

.. include:: _Install-Dependencies.rst

Set Up a Personal Access Token
------------------------------

.. include:: _Personal-Access-Token.rst

Ensure repositories are up-to-date
----------------------------------

.. include:: _Ensure-Repositories-Are-Up-To-Date.rst

Generate Changelog
------------------

Generate a ``CHANGELOG.rst`` file per package in your repo using the following command:

.. code-block:: bash

   catkin_generate_changelog --all

.. include:: _Clean-Up-Changelog.rst

Bump the package version
------------------------

.. include:: _Bump-Package-Version.rst

Bloom Release
-------------

Run the following command, replacing ``my_repo`` with the name of your repository:

.. code-block:: bash

  bloom-release --new-track --rosdistro {DISTRO} --track {DISTRO} my_repo

.. tip::

   * ``--new-track`` tells bloom to create a new :ref:`track <what-is-a-track>` and configure it.
   * ``--rosdistro {DISTRO}`` indicates that this release is for the ``{DISTRO}`` distro
   * ``--track {DISTRO}`` indicates that you want the track name to be ``{DISTRO}``


You will be prompted to enter information to configure a new track.
In a common scenario such as:

* Your packages are in a repository called ``my_repo``
* You are releasing a branch called ``main``
* The repository is hosted on GitHub at ``https://github.com/my_organization/my_repo.git``
* Your release repository is at ``https://github.com/ros2-gbp/my_repo-release.git``

You should respond to the prompts as following:

.. list-table::
   :header-rows: 1
   :widths: 1 2

   * - Configuration
     - Value
   * - :ref:`Release Repository url <release-repository-url>`
     - ``https://github.com/ros2-gbp/my_repo-release.git``
   * - :ref:`Repository Name <repository-name>`
     - ``my_repo``
   * - :ref:`Upstream Repository URI <upstream-repository-uri>`
     - ``https://github.com/my_organization/my_repo.git``
   * - :ref:`Upstream VCS Type <upstream-vcs-type>`
     -
   * - :ref:`Version <version>`
     -
   * - :ref:`Release Tag <release-tag>`
     -
   * - :ref:`Upstream Devel Branch <upstream-devel-branch>`
     - ``main``
   * - :ref:`ROS Distro <ros-distro>`
     -
   * - :ref:`Patches Directory <patches-directory>`
     -
   * - :ref:`Release Repository Push URL <release-repository-push-url>`
     -

.. note::

  An empty cell in the table indicates that the default value should be used.
  Simply respond to the prompt by pressing Enter.

Bloom will automatically create a pull request for you against `rosdistro <https://github.com/ros/rosdistro>`_.

Next Steps
----------

.. include:: _Next-Steps.rst
