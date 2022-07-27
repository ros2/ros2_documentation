Release Team / Repository
=========================

.. contents:: Table of Contents
   :depth: 3
   :local:

This page explains the recommendation of becoming a part of a release team and hosting a release repository on `ros2-gbp <https://github.com/ros2-gbp>`_.

What is ROS 2 GBP?
------------------

`ros2-gbp <https://github.com/ros2-gbp>`_ is a GitHub organization that hosts the release repositories for ROS packages, and lists release teams and their members that have access to modify the release repositories.
Interactions with ros2-gbp are done through raising GitHub issues.
It is recommended that you get this done early, as it may take some time for the ros2-gbp maintainers to respond to your requests.

.. _what-is-a-release-team:

What is a release team?
-----------------------

A release team is a `GitHub team <https://docs.github.com/en/organizations/organizing-members-into-teams/about-teams>`_ that consists of a group of people who are responsible for the release process of one or more repositories.
Release teams often consist of an organization, a working group, or even an individual, and are named after the team or group that they represent.
The list of release teams and their associated release repositories are maintained at `ros2-gbp-github-org <https://github.com/ros2-gbp/ros2-gbp-github-org>`_.

**To make a release, you must be part of the team that manages the corresponding release repository.**
If you intend to release the repository under an existing team, follow :ref:`Join a release team <join-a-release-team>`.
If you intend to start a new team, follow :ref:`Start a new release team <start-a-new-release-team>`.

.. _join-a-release-team:

Join a release team
^^^^^^^^^^^^^^^^^^^

Fill the `Update Release Team Membership issue <https://github.com/ros2-gbp/ros2-gbp-github-org/issues/new?assignees=&labels=&template=update_release_team_membership.md&title=Update+release+team+membership>`_ issue template
if a release team already exists for your project but you are not part of it.

.. _start-a-new-release-team:

Start a new release team
^^^^^^^^^^^^^^^^^^^^^^^^

Fill the `New Release Team issue <https://github.com/ros2-gbp/ros2-gbp-github-org/issues/new?assignees=&labels=&template=new_release_team.md&title=Add+release+team>`_ issue template
if no release team exists for your project yet, request for a new release team to be created.

.. _what-is-a-release-repository:

What is a release repository?
-----------------------------

A release repository is a repository that

* stores files generated from the release process, for the ROS buildfarm to use
* caches configurations from the release process to simplify subsequent releases of the repository in the future

Having a release repository separate from your source code repository is a requirement for making a release in ROS 2.

.. _create-a-new-release-repository:

Create a new release repository
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Fill the `Add New Release Repositories issue <https://github.com/ros2-gbp/ros2-gbp-github-org/issues/new?assignees=&labels=&template=new_release_repository.md&title=Add+new+release+repositories>`_ issue template
if you don't have a release repo for your project yet.

What if my existing release repo isn't on ros2-gbp?
---------------------------------------------------

Packages released before ros2-gbp existed may have their release repositories hosted elsewhere.
It is now strongly recommended for release repositories to live in this dedicated GitHub organization.
If you are porting a ROS 1 package to ROS 2 and planning on releasing your packages into ROS 2 for the first time, follow standard procedure to request for a new release repository for your ROS 2 releases.
If you have previously released your packages for ROS 2, when raising the `Add New Release Repositories issue <https://github.com/ros2-gbp/ros2-gbp-github-org/issues/new?assignees=&labels=&template=new_release_repository.md&title=Add+new+release+repositories>`_, **specify your current release repository url**, and follow standard procedure for the rest.

.. note::

   Release repositories hosted elsewhere are still supported for stable distributions if you are not planning to release the repository into {DISTRO}.
   Since stable distributions created from Rolling will start with release repositories in the ros2-gbp organization it is recommend that you use the ros2-gbp release repositories for all ROS 2 distributions to avoid fragmenting the release information.

   A ros2-gbp release repository may become a hard requirement in the future and maintaining a single release repository for all ROS 2 distributions simplifies the maintenance of releases for both the Rolling distribution maintainers and package maintainers.
