.. redirect-from::

    Release-Howto

How to release
==============

This page tries to capture the process we go through to make a new beta release of ROS 2.

We usually don't branch before a release but "freeze" the used branch.
During the testing phase make sure that no unwanted changes are being committed to that branch.
Iteratively test either using the artifacts produced by the packaging jobs or from-source builds and make necessary changes.
Once the current state is ready to be released, follow these steps:


*
  Get a fresh copy of all repositories using the master `ros2.repos file <https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos>`__.


  * ``curl https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos | vcs import ./src``

*
  Update the version number in (most) packages (excluding the ones which have their own numbering scheme). Also update the version numbers in all python packages that have a setup.py.


  * Commit and push these changes: ``vcs custom ./src --args commit -m "beta N" -a``, ``vcs custom ./src --args push``


*
  Create a ``.repos`` file with the exact commit hashes you have checked out locally:


  * ``vcs export --exact ./src > hashes.repos``

*
  Tag (most) repositories using ``vcstool``.


  * For some repositories we are not creating ROS 2 specific tags but use the hashes instead:

    * ``ament/osrf_pycommon``
    * ``eProsima/Fast-CDR``
    * ``eProsima/Fast-RTPS``
    * ``ros/class_loader``
    * ``ros/console_bridge``
    * Remove the above repositories for now: ``rm -fr src/ament/osrf_pycommon src/eProsima src/ros``

  * Note that for this step to work without requiring lots of password typing, you either need a ``~/.netrc`` file with your credentials, or you need to change the GitHub URLs in the ``.repos`` file to use ssh instead of https.
  * Create the release tag:

    * ``vcs custom ./src --args tag release-betaN`` (adjust the tag name appropriately).
    * If we ever have something other than ``git`` repositories we'll need to use the ``--git`` and ``--hg`` (for example) arguments separately.

  * Update the ``release-latest`` tag on all repositories:

    * ``vcs custom ./src --args tag -f release-latest``

  * Push tags (using force to overwrite existing latest tags):

    * ``vcs custom ./src --args push --tags -f``

*
  Create new ``.repos`` file:


  * ``cp hashes.repos tags.repos``
  * Edit ``tags.repos`` and replace the version attribute for all repositories (except the ones skipped before) with ``release-betaN`` (adjust the tag name appropriately).

*
  Repeat the tagging and ``.repos`` file generation for the `turtlebot2_demo.repos file <https://github.com/ros2/turtlebot2_demo/blob/release-latest/turtlebot2_demo.repos>`__.


  * At beta3, only the turtlebot2_demo repo had packages that needed their version bumped, and no repos used a fixed hash instead of the release tag.

*
  Run some packaging job using this new ``.repos`` file.


  * First upload ``tags.repos`` somewhere (e.g. gist.github.com).
  * Then trigger a packaging job for each platform and use the url of the hosted ``.repos`` file in the ``CI_ROS2_REPOS_URL`` field.
  * Rename each artifact file (an archive file) as ``ros2-beta<beta-number>-package-<platform><rmw-impl>-<opt-arch>.<ext>``.

    * E.g. ``ros2-beta2-package-linux-fastrtps-x86_64.tar.bz2``

*
  Create a tag on the ``ros2/ros2`` repository called ``release-betaN`` with the new ``.repos`` file:


  * Clone ``ros2/ros2`` to the master branch.
  * Replace the ``ros2.repos`` file's content with that of the ``tags.repos`` file created above.
  * Commit it with a message like the tag name, e.g. ``release-betaN`` (adjust the tag name appropriately).
  * Tag it with ``git tag release-betaN`` (adjust the tag name appropriately) and ``git tag -f release-latest`` push both with ``git push --tags -f``.

*
  Create a new release in the `Releases <https://github.com/ros2/ros2/releases>`__ section of ``ros2/ros2`` using this new tag:


  * Use the title ``ROS 2 Beta N release`` (matching the style of previous releases).

* Upload the renamed artifacts to the Release on GitHub using the web interface:

  * E.g. https://github.com/ros2/ros2/releases/edit/release-beta2

* Create an overview page for the beta release, e.g. https://github.com/ros2/ros2_documentation/Beta2-Overview
* Update the releases page to point to it: https://github.com/ros2/ros2_documentation/Releases
* Update the `Features page <https://github.com/ros2/ros2_documentation/Features>`__ if appropriate.
* Update the link on the home page: https://github.com/ros2/ros2_documentation/README
* `Run the documentation generation <https://github.com/ros2/docs.ros2.org/tree/doc_gen>`__ and upload and link the results from https://docs.ros2.org/
* Draft and send an announcement to discourse about that release.
