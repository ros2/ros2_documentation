Request Access to Release Repository
====================================

.. contents:: Table of Contents
   :depth: 3
   :local:

What is a release repository?
-----------------------------

A release repository stores the output of bloom for the buildfarm to use, and configurations for bloom to store your release configuration for successive releases.
The repository is a completely separate repository from where your source code lies.
To release packages in ROS 2, having a release repository is a requirement.

How do I create one?
--------------------

Request the maintainers of `ros2-gbp <https://github.com/ros2-gbp>`_ by filling and submitting templated GitHub issues to `ros2-gbp-github-org <https://github.com/ros2-gbp/ros2-gbp-github-org>`_.

.. note::

   Hosting the release repo on ros2-gbp may sound unfamiliar if you have released packages previously in other distros.
   Release repositories for packages now live in a dedicated GitHub organization, that allows new automation supporting the :doc:`Rolling distribution <../../Releases/Release-Rolling-Ridley>`.

   For more information, see `What if my existing release repo isn't on ros2-gbp?`_.

What is ros2-gbp?
-----------------

`ros2-gbp <https://github.com/ros2-gbp>`_ is a GitHub organization that stores release repositories for packages released into ROS.
The `ros2-gbp-github-org <https://github.com/ros2-gbp/ros2-gbp-github-org>`_ is a repository in the organization that stores a list of release teams and their release repositories.
Each release repository is maintained by one of the release teams.
To request for a release repository to be created, you must either request for a new release team, or get added to an existing release team.

Requesting
----------

Interactions with ros2-gbp are done through raising GitHub issues.
It is recommended that you get this done early, as it may take some time for the ros2-gbp maintainers to respond to your requests.

New release team
^^^^^^^^^^^^^^^^

Fill the `New Release Team issue <https://github.com/ros2-gbp/ros2-gbp-github-org/issues/new?assignees=&labels=&template=new_release_team.md&title=Add+release+team>`_ issue template
if no release team exists for your project yet, request for a new release team to be created.

Getting added to an existing release team
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Fill the `Update Release Team Membership issue <https://github.com/ros2-gbp/ros2-gbp-github-org/issues/new?assignees=&labels=&template=update_release_team_membership.md&title=Update+release+team+membership>`_ issue template
if a release team already exists for your project but you are not part of it.

New release repo
^^^^^^^^^^^^^^^^

Fill the `Add New Release Repositories issue <https://github.com/ros2-gbp/ros2-gbp-github-org/issues/new?assignees=&labels=&template=new_release_repository.md&title=Add+new+release+repositories>`_ issue template
if you don't have a release repo for your project yet.

What if my existing release repo isn't on ros2-gbp?
---------------------------------------------------

Packages released before ros2-gbp existed may have their release repositories hosted elsewhere.
If you are porting a ROS 1 package to ROS 2 and planning on releasing your packages into ROS 2 for the first time, follow standard procedure to request for a new release repository for your ROS 2 releases.
If you have previously released your packages for ROS 2, when raising the `Add New Release Repositories issue <https://github.com/ros2-gbp/ros2-gbp-github-org/issues/new?assignees=&labels=&template=new_release_repository.md&title=Add+new+release+repositories>`_, **specify your current release repository url**, and follow standard procedure for the rest.

.. note::

   Release repositories hosted elsewhere are still supported for stable distributions if you are not planning to release the repository into {DISTRO}.
   Since stable distributions created from Rolling will start with release repositories in the ros2-gbp organization it is recommend that you use the ros2-gbp release repositories for all ROS 2 distributions to avoid fragmenting the release information.

   A ros2-gbp release repository may become a hard requirement in the future and maintaining a single release repository for all ROS 2 distributions simplifies the maintenance of releases for both the Rolling distribution maintainers and package maintainers.
