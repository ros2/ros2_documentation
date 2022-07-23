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

Generate a ``CHANGELOG.rst`` file per package in your repo with the following command:

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

  bloom-release --new-track --rosdistro {DISTRO} --track {DISTRO} my_repo

.. ``bloom-release`` will perform the following:

.. * Generate a new :ref:`track <track>` and configure it
.. * Generate platform specific release artifacts
.. * Push them to the release repository
.. * Open a Pull Request from your GitHub account to `rosdistro <https://github.com/ros/rosdistro>`_ with changes to add your repository to ``{DISTRO}/distribution.yaml``.

.. tip::

   * ``--new-track`` tells bloom to create a new :ref:`track <what-is-a-track>` and configure it.
   * ``--rosdistro {DISTRO}`` indicates that this release is for the ``{DISTRO}`` distro
   * ``--track {DISTRO}`` indicates that you want the track name to be ``{DISTRO}``


You will be prompted to enter information to configure a new track.
In a common scenario such as:

* You are releasing a repository called ``my_repo``
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

.. There are many command which come with bloom, even though you will most likely only need
.. to run ``bloom-release``. Many of the bloom commands are prefixed with ``git-``, which indicates
.. that they must be run inside a git repository. If you clone your release repository manually,
.. then you can use ``git-`` prefixed commands to manually manipulate your release repository.
.. One of these commands is called ``git-bloom-config`` and it lets you manage your tracks.
.. Run ``git-bloom-config -h`` to get more information about how to manage your release tracks.

Bloom will automatically create a pull request for you against `rosdistro <https://github.com/ros/rosdistro>`_.

Next Steps
----------

.. include:: _Next-Steps.rst
