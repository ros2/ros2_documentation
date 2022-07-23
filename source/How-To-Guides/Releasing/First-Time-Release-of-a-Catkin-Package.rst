First Time Release of a Catkin Package
======================================

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

Releasing Your Packages
-----------------------

.. note::

   If you have two factor authorization enabled on github, follow :doc:`Github Manual Authorization <Github-Manual-Authorization>` first.

The actual releasing of the package should be performed using one of the commands below, where you should replace ``my_repo`` with the name of your repository:

* Releasing a package for the first time, for a new distro, or editing an existing release track:

   .. code-block:: bash

      bloom-release --rosdistro {DISTRO} --track {DISTRO}  --edit my_repo

* Releasing a package update on an existing release track:

   .. code-block:: bash

      bloom-release --rosdistro {DISTRO} my_repo

.. tip::

   * ``--rosdistro {DISTRO}`` indicates that this release is for the ``{DISTRO}`` distro
   * ``--track {DISTRO}`` indicates that you want the track name to be ``{DISTRO}``
   * ``--edit`` tells bloom to create the track if it doesn't exist and configure it.

If you used the ``--edit`` flag, continue with `Configuring the Release Track`_.
If you're releasing a package update on an existing release track without editing it, you can skip the next section.

Configuring the Release Track
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. warning::

   If you're releasing a package update without configuring the track, skip these instructions

The ``bloom-release`` script will prompt you through to perform the following:

* Setup a new track and configure it
* Generate platform specific release artifacts
* Push them to your release repository
* Fork `rosdistro <https://github.com/ros/rosdistro>`_ to your github account and open a Pull Request back upstream with your package to added to ``{DISTRO}/distribution.yaml``.

bloom is designed to allow the release of the same package for different ROS distributions and versions in the same release repository.
To facilitate this, bloom uses release "tracks" to maintain configurations for different release processes.
For normal ament-based ROS packages the default release track is recommended.

In the ``bloom-release`` command you ran above, you specified the ``--track``.
By convention you should create tracks with the same name as the ROS distro you are releasing for, but you could name your track what ever you wanted.

Let's look at a common scenario.

You are trying to release ament library called ``my_repo`` hosted on Github at ``https://github.com/my_organization/my_repo.git``.
You want the ``main`` branch from the library to be released it into the ROS ecosystem.
You already have an empty release repository (``https://github.com/ros2-gbp/my_repo-release.git``), from :doc:`Obtain-Access-to-Release-Repository <Obtain-Access-to-Release-Repository>`.

For this scenario, the table below summarises the responses to the questions:

.. list-table::
   :header-rows: 1

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

Pull Request to rosdistro
-------------------------

.. warning::

  If the automated pull request was opened successfully, then you **do not need to open one manually** as described here.
  You can simply skip this section.

Normally your ``bloom-release`` call should open a pull request for you, but if there is a problem or you do not wish for it to open the pull request on your behalf you can manually open a pull request also.

In the unlikely case that the automated pull request does not open successfully, you will have to manually open a pull request with modifications to rosdistro.

You can open a pull request by simply visiting `{DISTRO}/distribution.yaml <https://github.com/ros/rosdistro/blob/master/{DISTRO}/distribution.yaml>`_ and clicking the edit button (note: you have to be logged into Github for this to work), make your changes and then click "Propose Changes" at the bottom right of the page.

To enter your repository you need to fill out a section like this:

.. code-block:: yaml

   my_repo:
     doc:
       type: git
       url: https://github.com/my_organization/my_repo.git
       version: rolling
     release:
       tags:
         release: release/{DISTRO}/{package}/{version}
       url: https://github.com/ros2-gbp/my_repo-release.git
       version: 0.0.1-1
     source:
       type: git
       url: https://github.com/my_organization/my_repo.git
       version: rolling
     status: developed

You should put the **https://** url of the RELEASE repository here, not the url of your upstream repository.

.. note::

   * put the full version which is the version of your package plus the release increment number separated by a hyphen. (eg. ``0.0.1-1``).
     The release increment number is increased each time you release a package of the same version.
     This can occur when adding patches to the release repository or when changing the release settings.
   * Put your package into the list of packages in ALPHABETICAL order.

.. note::

   If your repository contains multiple packages, their names must be listed in the distro file, too.
   For example if the repository contains two packages ``my_package_1`` and ``my_package_2`` they will be listed as below:

   .. code-block:: yaml

      my_repo:
        doc:
          type: git
          url: https://github.com/my_organization/my_repo.git
          version: rolling
        release:
          packages:
          - my_package_1
          - my_package_2
          tags:
            release: release/{DISTRO}/{package}/{version}
          url: https://github.com/ros2-gbp/my_repo-release.git
          version: 0.0.1-1
        source:
          type: git
          url: https://github.com/my_organization/my_repo.git
          version: rolling
        status: developed

Next Steps
----------

Once your pull request has been submitted, one of the ROS developers will merge your request (this usually happens fairly quickly).
24-48 hours after that, your package should be built by the build farm and released into the building repository.
Packages built are periodically synchronized over to the `shadow-fixed <https://wiki.ros.org/ShadowRepository>`_ and public repositories, so it might take as long as a month before your package is available on the public ROS debian repositories (i.e. available via ``apt-get``).
To get updates on when the next synchronization (sync) is coming, check the `ROS discussion forums <https://discourse.ros.org/>`_.

Individual build details are on the Jenkins build farm `build.ros2.org <http://build.ros2.org/>`__.
Check `ROS {DISTRO} Default Package Status <http://repo.ros2.org/status_page/ros_{DISTRO}_default.html>`__ to see status of released packages.
