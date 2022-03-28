Obtain Access to Release Repository
===================================

.. contents:: Table of Contents
   :depth: 3
   :local:

About ros2-gbp
--------------

`ros2-gbp <https://github.com/ros2-gbp>`_ is a Github organization that stores a list of release
teams and their release repositories. gbp is an acronym for
`git-buildpackage <http://honk.sigxcpu.org/projects/git-buildpackage/manual-html/man.gbp.buildpackage.html>`_.

To release a package, you need a release repository on
ros2-gbp to store the results of the release process.

.. note::

   Hosting the release repo on ros2-gbp may be new if you have released packages previously in
   other distros. Release repositories for packages now live in a dedicated GitHub organization.
   The dedicated organization allows new automation supporting the
   :doc:`Rolling distribution <../../Releases/Release-Rolling-Ridley>`.

   Release repositories hosted elsewhere are still supported for stable distributions if you are not
   planning to release the repository into {DISTRO}.
   Since stable distributions created from Rolling will start with release repositories in the
   ros2-gbp organization it is recommend that you use the ros2-gbp release repositories for all
   ROS 2 distributions to avoid fragmenting the release information.

   A ros2-gbp release repository may become a hard requirement in the future and
   maintaining a single release repository for all ROS 2 distributions simplifies the maintenance of
   releases for both the Rolling distribution maintainers and package maintainers.

Requesting
----------

Interactions with ros2-gbp are done through filing issues. It is recommended that you get this done
early, as it may take some time for the ros2-gbp maintainers to respond to your requests.

Request for a new release team
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You first must be part of the release team on ros2-gbp you plan to release the project under.
If no release team exists for your project yet, fill out the
`New Release Team <https://github.com/ros2-gbp/ros2-gbp-github-org/issues/new?assignees=&labels=&template=new_release_team.md&title=Add+release+team>`_
issue template to request for a release team to be created.
You must wait for the ros2-gbp maintainers to create a release team for you.

Request to get added to a release team
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If a release team already exists for your project but you are not part of it, request to be added by
by filling out the
`Update Release Team Membership <https://github.com/ros2-gbp/ros2-gbp-github-org/issues/new?assignees=&labels=&template=update_release_team_membership.md&title=Update+release+team+membership>`_
issue template.
You must wait for the ros2-gbp maintainers to add you to a release team.

Request for a new release repo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you don't have a release repo for your project yet, fill out the
`Add New Release Repositories <https://github.com/ros2-gbp/ros2-gbp-github-org/issues/new?assignees=&labels=&template=new_release_repository.md&title=Add+new+release+repositories>`_
issue template.
You must wait for the ros2-gbp maintainers to create an empty release repository for you.


What if I already have a release repo elsewhere?
------------------------------------------------

For packages that have already been released in another ROS distro, you'll have a release repository
hosted elsewhere.

If you've only released the packages in ROS1 before, follow standard procedure to request for
a new release repository for your ROS2 releases.

If you've released packages before in ROS2, when filling out the
`Add New Release Repositories <https://github.com/ros2-gbp/ros2-gbp-github-org/issues/new?assignees=&labels=&template=new_release_repository.md&title=Add+new+release+repositories>`_
issue template, specify your current release repository url.