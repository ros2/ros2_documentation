Release Preparation
===================

.. contents:: Table of Contents
   :depth: 3
   :local:

Prerequisites
-------------

Before releasing the package, here's a list of things that you'd want to check you've completed:

* Your code is up-to-date
* Your package builds correctly
* All your tests are passing
* You've pushed all changes from your machine to the upstream repository.

.. note::

   The **upstream repository** is the repository where you do your development and host the source
   code of your package. This repository can be hosted anywhere (even locally) and can be a git,
   hg, or svn repository or the location of an archive (tar.gz only for now, but there are plans
   for tar.bz and zip).

Install catkin_pkg
------------------

`catkin_pkg <https://github.com/ros-infrastructure/catkin_pkg>`_ is a python library that contains
scripts to simplify the release preparation process.
On Ubuntu the recommended installation method is to use ``apt``:

.. code-block:: bash

   sudo apt install python3-catkin-pkg

.. note::

   On non-debian systems you can install via pypi:

   .. code-block:: bash

      pip3 install -U catkin_pkg

Writing a Change Log
--------------------

To make it easier for users and contributors to see precisely what notable changes have been made
between each release (or version) of the project, you are recommended to have a changelog for your
package.

In the steps below, the package changelogs will be incorporated as part of the source working tree
as recommended in
`REP-132: Incorporation of Changelogs into Package Source Tree <https://www.ros.org/reps/rep-0132.html>`_.

Generate / Update CHANGELOG.rst
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If this is the first time you're releasing the package, you'll likely not have a
``CHANGELOG.rst`` file in your package(s) yet. Rather than creating this file manually, add the
``--all`` flag to let the following script generate the file(s) for you and make an entry for the
upcoming release for you.

If this is not your first release and you already have a ``CHANGELOG.rst`` per package,
omit the ``--all`` flag and let the script make an entry for the upcoming release for you.

.. code-block:: bash

   # Run either:
   catkin_generate_changelog --all  # If CHANGELOG.rst doesn't exist yet
   catkin_generate_changelog        # If CHANGELOG.rst already exists

Clean up the Changelog
^^^^^^^^^^^^^^^^^^^^^^

Open ``CHANGELOG.rst`` in an editor. You will see that the command ``catkin_generate_changelog``
from the previous step has filled in your CHANGELOG.rst with your commit messages, like below:

.. code-block:: rst

   ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
   Changelog for package your_package
   ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

   Forthcoming
   -----------
   * you can modify commit message here
   * and here

You should clean up the list of commit messages to concisely convey the exact changes that have
happened since the last release to your users and maintainers.

Incorrectly formatted ``CHANGELOG.rst`` can cause problems with your package.
See `rclcpp's CHANGELOG.rst <https://github.com/ros2/rclcpp/blob/master/rclcpp/CHANGELOG.rst>`_
for a well formatted example.

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

Commit the Changelog
^^^^^^^^^^^^^^^^^^^^

After cleaning up the changelogs for all packages, commit your changelogs.
**Don't forget this step.** 

Bump package version
--------------------

Every release of the package must have a new package version, higher than the previous release.
Bump the package's version by running:

.. code-block:: bash

   catkin_prepare_release

The script performs the following:

#. increases the package version in the ``package.xml`` file(s)
#. replaces the heading ``Forthcoming`` with ``version (date)`` (eg. ``0.0.1 (2022-01-08)``) in ``CHANGELOG.rst``
#. commits those changes
#. creates a tag (eg. ``0.0.1``)
#. pushes those changes to upstream

.. note::

   By default this command increases the patch version of your package, e.g. ``0.1.1`` -> ``0.1.2``,
   but you can pick minor or major using the ``--bump`` option.

.. note::

   Even if you do not use ``catkin_prepare_release``, you must have one or more valid
   ``package.xml`` (s) with the same version and a matching tag in your upstream repository.
   For example, if you are going to release version 0.1.0 of your
   package, then bloom expects there to be a 0.1.0 tag in your upstream repository.

   If you have a custom version tagging scheme you'd like to use, then bloom can handle while
   configuring a release track using the 'Release Tag' configuration.
