.. redirect-from::

  Guides/Package-maintainer-guide

ROS 2 Package Maintainer Guide
==============================

Each package in the ROS 2 core has one or more maintainers that are responsibile for the general health of the package.
This guide gives some information about the responsibilities of a ROS 2 core package maintainer.

.. contents:: Table of Contents
   :local:

Reviews
-------

All incoming code to ROS 2 core repositories must be reviewed.
The review is looking for:

* Suitability in the package
* Correct code
* Conforms to developer guidelines:

    * :doc:`Developer Guide <../The-ROS2-Project/Contributing/Developer-Guide>`
    * :doc:`Code Style Guide <../The-ROS2-Project/Contributing/Code-Style-Language-Versions>`

* Adds tests for the bug/feature
* Adds documentation for new features
* Clean Continuous Integration run
* Targets default branch (usually "rolling")
* Has at least one approval from a maintainer that is not the author

Continuous Integration
----------------------

All incoming code to ROS 2 core repositories must be run through Continuous Integration.
ROS 2 currently has two separate CI systems, and it is required that PRs pass both of them before merging.

PR builds (https://build.ros2.org/view/Rpr)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

ROS 2 PR (Pull Request) builds run automatically every time a pull request is opened.
These builds run a build and test of this package, and this package only.
This means that it does not build any dependencies, and it also does not build any packages that depend upon this package.
These builds are good for quick feedback to see if the change passes linters, unit tests, etc.
There are two major problems with them:

* These builds do not work across multiple repositories (so won't work for adding or changing an API, etc)
* These tests only run on Linux (they won't run on macOS or Windows)

To address these two problems, there is also the CI builds.

CI builds (https://ci.ros2.org)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

CI builds do not run automatically when a pull request is opened.
One of the maintainers of the package must manually request that a CI build is done by going to https://ci.ros2.org/job/ci_launcher/ .

By default, running a job in this way will build and run tests for all packages (> 300 currently) on all platforms (Linux, macOS, and Windows).
As a full run can take many hours and tie up the CI machines, it is recommended that all runs here restrict the number of packages that are built and tested.
This can be accomplished by using the colcon arguments ``--packages-up-to``, ``--packages-select``, ``--packages-above-and-dependencies``, ``--packages-above``, amongst others.
See the `colcon documentation <https://colcon.readthedocs.io/en/released/user/how-to.html#build-only-a-single-package-or-selected-packages>`__ for more examples on the flags that can be used.
Further documentation on how to use the CI machinery is available at https://github.com/ros2/ci/blob/master/CI_BUILDERS.md.

Merging Pull Requests
---------------------

A pull request can be merged if all of the following are true:

* The DCO bot reports a passing result
* The PR build reports a passing result
* The CI build reports a passing result on all platforms
* The code has been reviewed and approved by at least one maintainer

After a PR is merged, it will automatically get built with the next `nightlies <https://ci.ros2.org/view/nightly>`__.
It is highly recommended to check the nightlies after merging pull requests to ensure no regressions have occurred.

Keeping CI green
----------------

The nightly jobs that run tests are typically much more comprehensive than what is done for individual pull requests.
For this reason, there can be regressions that occur in the nightlies that were not seen in the CI jobs.
It is a package maintainers responsibility to check for regressions in their packages at the following locations:

* https://ci.ros2.org/view/nightly
* https://ci.ros2.org/view/packaging
* https://build.ros2.org/view/Rci
* https://build.ros2.org/view/Rdev

For any problems that are found, new issues and/or pull requests on the relevant repositories should be opened.

Making releases
---------------

In order to get new features and bugfixes out to end users, the package maintainers must periodically do a release of the package (a release may also be requested on-demand from other maintainers).

As outlined in the :ref:`developer guide <semver>`, ROS 2 packages follow semver for version numbers.

A release in ROS terms consists of two distinct steps: making a source release, and then making a binary release.

Source release
^^^^^^^^^^^^^^

A source release creates a changelog and a tag in the relevant repository.

The process starts by generating or updating CHANGELOG.rst files with the following command:

.. code-block:: bash

  $ catkin_generate_changelog

If one or more packages in the repository don't have contain CHANGELOG.rst, add the ``--all`` option to populate all of the previous commits for each package.
The ``catkin_generate_changelog`` command will simply populate the files with the commit logs from the repository.
Since those commit logs aren't always appropriate for a changelog, it is recommended to edit CHANGELOG.rst and edit it to make it more readable.
Once editing is done, it is important to commit the updated CHANGELOG.rst file to the repository.

The next step is to bump the version in the package.xml and the changelog files with the following command:

.. code-block:: bash

  $ catkin_prepare_release

This command will find all of the packages in the repository, check that the changelogs exist, check that there are no uncommitted local changes, increment the version in the package.xml files, and commit/tag the changes with a bloom-compatible tag.
Using this command is the best way to ensure the release versions are consistent and compatible with bloom.
By default, ``catkin_prepare_release`` will bump the patch version of the packages, e.g. 0.1.1 -> 0.1.2 .
However, it can also bump the minor or major number, or even have an exact version set.
See the help output from ``catkin_prepare_release`` for more information.

Assuming the above was successful, a source release has been made.

Binary release
^^^^^^^^^^^^^^

The next step is to use the ``bloom-release`` command to create a binary release.
For full instructions on how to use bloom, please see http://wiki.ros.org/bloom.
To do a binary release of a package, run:

.. code-block:: bash

  $ bloom-release --track <rosdistro> --rosdistro <rosdistro> <repository_name>

For instance, to release the ``rclcpp`` repository to the {DISTRO_TITLE} distribution, the command would be:

.. code-block:: bash

  $ bloom-release --track {DISTRO} --rosdistro {DISTRO} rclcpp

This command will fetch the release repository, make the necessary changes to make the release, push the changes to the release repository, and finally open a pull request to https://github.com/ros/rosdistro .

Backporting to released distributions
-------------------------------------

All incoming changes should first land on the development branch.
Once a change has been merged onto the development branch, it can be considered for backporting to released distributions.
However, any backported code must not break `API <https://en.wikipedia.org/wiki/API>`__ or `ABI <https://en.wikipedia.org/wiki/Application_binary_interface>`__ in a released distribution.
If a change can be backported without breaking API or ABI, then a new pull request targeting the appropriate branch should be created.
The new pull request should be added to the appropriate distributions project board at https://github.com/orgs/ros2/projects.
The new pull request should have all of the steps run as before, but making sure to target the distribution in question for CI, etc.

Responding to issues
--------------------

Package maintainers should also look at incoming issues on the repository and triage the problems that users are having.

For issues that look like questions, the issue should be closed and the user redirected to `Robotics Stack Exchange <https://robotics.stackexchange.com/>`__ .

If an issue looks like a problem, but is not relevant to this particular repository, it should be moved to the appropriate repository with the GitHub "Transfer issue" button.

If the reporter has not provided enough information to determine the cause of the problem, more information should be requested from the reporter.

If this is a new feature, tag the issue with "help-wanted".

Any remaining issues should be reproduced, and determined if they are truly a bug.
If it is a bug, fixes are highly appreciated.

Getting help
------------

While doing maintenance on a package, questions about general procedures or individual issues may come up.

For general questions, please follow the :doc:`contributing guidelines <../The-ROS2-Project/Contributing>`.

For questions on individual issues, please tag the ROS 2 GitHub team (@ros/team), and someone on the team will take a look.
