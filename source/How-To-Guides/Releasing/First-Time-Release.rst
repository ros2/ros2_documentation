First Time Release
==================

This guide explains how to release a Catkin package.
If you plan on releasing a ROS package that you developed, this is the guide to follow.
Due to the numerous options available when releasing a ROS package, this guide will not be able to cover every detail.
It intends to cover the most common use case.

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

Generate Changelog
------------------

For your users and for the developers, it is important to keep the changelog up to date.

.. code-block:: bash

   catkin_generate_changelog --all

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

  bloom-release --rosdistro {DISTRO} --track {DISTRO} --new-track my_repo

``bloom-release`` will perform the following:

* Generate a new :ref:`track <track>` and configure it
* Generate platform specific release artifacts
* Push them to the release repository
* Open a Pull Request from your GitHub account to `rosdistro <https://github.com/ros/rosdistro>`_ with changes to add your repository to ``{DISTRO}/distribution.yaml``.

.. tip::

   * ``--rosdistro {DISTRO}`` indicates that this release is for the ``{DISTRO}`` distro
   * ``--track {DISTRO}`` indicates that you want the track name to be ``{DISTRO}``
   * ``--new-track`` tells bloom to create a new :ref:`track <track>` and configure it.

You are trying to release ament library called ``my_repo`` hosted on GitHub at ``https://github.com/my_organization/my_repo.git``.
You want the ``main`` branch from the library to be released it into the ROS ecosystem.
You already have an empty release repository (``https://github.com/ros2-gbp/my_repo-release.git``), from :doc:`Obtain-Access-to-Release-Repository <Obtain-Access-to-Release-Repository>`.

For this scenario, the table below summarises the responses to the questions:

.. list-table::
   :header-rows: 1
   :widths: 1 2

   * - Configuration
     - Value
   * - Release Repository url
     - ``https://github.com/ros2-gbp/my_repo-release.git``
   * - Repository Name
     - ``my_repo``
   * - Upstream Repository URI
     - ``https://github.com/my_organization/my_repo.git``
   * - Upstream VCS Type
     - ``git``
   * - Version
     -
   * - Release Tag
     -
   * - Upstream Devel Branch
     - ``main``
   * - ROS Distro
     - ``{DISTRO}``
   * - Patches Directory
     -
   * - Release Repository Push URL
     -

.. There are many command which come with bloom, even though you will most likely only need
.. to run ``bloom-release``. Many of the bloom commands are prefixed with ``git-``, which indicates
.. that they must be run inside a git repository. If you clone your release repository manually,
.. then you can use ``git-`` prefixed commands to manually manipulate your release repository.
.. One of these commands is called ``git-bloom-config`` and it lets you manage your tracks.
.. Run ``git-bloom-config -h`` to get more information about how to manage your release tracks.

Bloom will automatically create a pull request for you on `rosdistro <https://github.com/ros/rosdistro>`_.

Next Steps
----------

.. include:: _Next-Steps.rst
