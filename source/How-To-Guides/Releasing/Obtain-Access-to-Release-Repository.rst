Obtain Access to Release Repository
===================================

.. contents:: Table of Contents
   :depth: 3
   :local:

About ros2-gbp
--------------

`ros2-gbp <https://github.com/ros2-gbp>`_ is a GitHub organization that stores a list of release teams and their release repositories.
gbp is an acronym for `git-buildpackage <http://honk.sigxcpu.org/projects/git-buildpackage/manual-html/man.gbp.buildpackage.html>`_.

To release a package, you need a release repository on ros2-gbp to store the results of the release process.
To set this up, continue on to `Requesting`_.

.. note::

   Hosting the release repo on ros2-gbp may sound unfamiliar if you have released packages previously in other distros.
   Release repositories for packages now live in a dedicated GitHub organization, that allows new automation supporting the :doc:`Rolling distribution <../../Releases/Release-Rolling-Ridley>`.

   For more information, see `What if my release repo isn't on ros2-gbp?`_.

Requesting
----------

Interactions with ros2-gbp are done through raising GitHub issues as instructed below.
It is recommended that you get this done early, as it may take some time for the ros2-gbp maintainers to respond to your requests.

Request for a new release team
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You first must be part of the release team on ros2-gbp you plan to release the project under.
If no release team exists for your project yet, raise a `New Release Team issue <https://github.com/ros2-gbp/ros2-gbp-github-org/issues/new?assignees=&labels=&template=new_release_team.md&title=Add+release+team>`_ to request for a release team to be created.
You must wait for the ros2-gbp maintainers to create a release team for you.

Request to get added to a release team
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If a release team already exists for your project but you are not part of it, request to be added by by raising an `Update Release Team Membership issue <https://github.com/ros2-gbp/ros2-gbp-github-org/issues/new?assignees=&labels=&template=update_release_team_membership.md&title=Update+release+team+membership>`_.
You must wait for the ros2-gbp maintainers to add you to a release team.

Request for a new release repo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you don't have a release repo for your project yet, raise an `Add New Release Repositories issue <https://github.com/ros2-gbp/ros2-gbp-github-org/issues/new?assignees=&labels=&template=new_release_repository.md&title=Add+new+release+repositories>`_.
You must wait for the ros2-gbp maintainers to create an empty release repository for you.

What if my release repo isn't on ros2-gbp?
------------------------------------------

If you or someone else has already released the package you are trying to release in a ROS distro before Humble, there may be a release repository hosted somewhere else.

If porting a ROS 1 package to ROS 2, follow standard procedure to request for a new release repository for your ROS 2 releases.

If you've released for ROS 2 in a distro before Humble, when raising the `Add New Release Repositories issue <https://github.com/ros2-gbp/ros2-gbp-github-org/issues/new?assignees=&labels=&template=new_release_repository.md&title=Add+new+release+repositories>`_, **specify your current release repository url**, and follow standard procedure for the rest.

.. note::

   Release repositories hosted elsewhere are still supported for stable distributions if you are not planning to release the repository into {DISTRO}.
   Since stable distributions created from Rolling will start with release repositories in the ros2-gbp organization it is recommend that you use the ros2-gbp release repositories for all ROS 2 distributions to avoid fragmenting the release information.

   A ros2-gbp release repository may become a hard requirement in the future and maintaining a single release repository for all ROS 2 distributions simplifies the maintenance of releases for both the Rolling distribution maintainers and package maintainers.
