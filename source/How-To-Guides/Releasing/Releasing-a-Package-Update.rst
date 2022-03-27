:orphan:

Releasing a Package Update
==========================

.. contents:: Table of Contents
   :depth: 3
   :local:

This guide will help you release updates for a package that has already been released.
If you are trying to release a package in a repository that has not been released before,
follow :doc:`Releasing for the First Time <First-Time-Release>`.

Join the Release Team
---------------------

A release team for the package you are trying to release will exist on
`ROS 2 release repositories (ros2-gbp) <https://github.com/ros2-gbp>`_.
Check if you are part of the team.

Only if you are **not** part of the team yet, request to be added by filling out the
`Update Release Team Membership <https://github.com/ros2-gbp/ros2-gbp-github-org/issues/new?assignees=&labels=&template=update_release_team_membership.md&title=Update+release+team+membership>`_
issue template. You must wait for the ros2-gbp maintainers to add you to the release team before proceeding with
the rest of this tutorial.

Install catkin_pkg and bloom
----------------------------

Install `catkin_pkg <https://github.com/ros-infrastructure/catkin_pkg>`_ and
`Bloom <http://ros-infrastructure.github.io/bloom/>`_.
On Ubuntu the recommend method is to use apt:

.. code-block:: bash

   sudo apt install python3-catkin-pkg python3-bloom

.. note::

   On other systems you can install via pypi:

   .. code-block:: bash

      pip3 install -U bloom catkin_pkg

Prerelease Check List
---------------------

Ensure that you've completed all the below:

* your code is up to date
* all tests pass
* you've pushed all changes from your machine to the upstream repository.

Update the Change Log
---------------------

Update ``CHANGELOG.rst`` by running:

.. code-block:: bash

   catkin_generate_changelog

Open ``CHANGELOG.rst`` in an editor. You will see that the previous script
has inserted an entry under the heading ``Forthcoming`` with the messages from all commits since the
last release, like below:

.. code-block:: rst

   ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
   Changelog for package your_package
   ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

   Forthcoming
   -----------
   * you can modify commit message here
   * and here

   0.0.1 (2022-03-12)
   ------------------
   ...

You should clean up the list of commit messages in the changelog to
concisely convey the exact changes that have happened since the last release to your users.

Once you have a nicely formatted changelog, **commit your updated changelog**.

Incorrectly formatted ``CHANGELOG.rst`` can cause problems with your package.
See `rclcpp's CHANGELOG.rst <https://github.com/ros2/rclcpp/blob/master/rclcpp/CHANGELOG.rst>`_ 
for a well formatted example.

.. note::

   You should **not** modify the ``Forthcoming`` heading, as this will be replaced with the
   package version number by ``catkin_prepare_release`` later on.

.. warning::

   If you have any commit messages ending in an underscore, such as member variables (e.g. ``name_``)
   this will throw an error with the RST Changelog format because RST treats those as
   `link targets <http://docutils.sourceforge.net/docs/user/rst/quickstart.html#sections>`_.
   The error will be something like:

   .. code-block::

      <string>:21: (ERROR/3) Unknown target name: "name".

   To fix this, you'll need to escape the variable, for example:

   .. code-block::

      * fix for checking the ``name_``

Increase the package version
----------------------------

Increase the package's version by running:

.. code-block:: bash

   catkin_prepare_release

This script performs the following:

#. replaces the heading ``Forthcoming`` with ``version (date)`` (eg. ``0.0.2 (2022-04-04)``) in ``CHANGELOG.rst``
#. increases the package version in the ``package.xml`` file(s)
#. commits those changes
#. creates a tag (eg. ``0.0.2``)
#. pushes those changes to upstream

Release your Packages
---------------------

The actual releasing of the package should be performed using the command below:

.. code-block:: bash

   bloom-release --rosdistro {DISTRO} <your_repository_name>

.. note::

   ``<your_repository_name>`` is not its url, it is its reference in ``{DISTRO}/distribution.yaml``.

The script will perform the following:

* Generate platform specific release artifacts
* Push them to your release repository
* Fork `rosdistro <https://github.com/ros/rosdistro>`_ to your github account and open a Pull
  Request back upstream with your package to added to ``{DISTRO}/distribution.yaml``.

.. note::

   If you encounter the following error message:

   .. code-block:: bash

      Cannot push to remote release repository.

   you are probably not part of the release team for the package.
   You should request to be added to the release team by filling out the
   `Update Release Team Membership <https://github.com/ros2-gbp/ros2-gbp-github-org/issues/new?assignees=&labels=&template=update_release_team_membership.md&title=Update+release+team+membership>`_
   issue template.

Next Steps
----------

Once your pull request has been submitted, one of the ROS developers will merge your request
(this usually happens fairly quickly). 24-48 hours after that, your package should be built by the
build farm and released into the building repository. Packages built are periodically synchronized
over to the `shadow-fixed <https://wiki.ros.org/ShadowRepository>`_
and public repositories, so it might take as long as a month before your
package is available on the public ROS debian repositories (i.e. available via apt-get).
To get updates on when the next synchronization (sync) is coming, check the
`ROS discussion forums <https://discourse.ros.org/>`_.
