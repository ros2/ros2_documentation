Releasing a Package for the First Time
======================================

.. contents:: Table of Contents
   :depth: 2
   :local:

This guide will help you release a brand new ROS package using the tools **bloom** and **catkin**.
By default, bloom releases packages to the public ROS build farm, which is what this tutorial
teaches you to do. See http://build.ros.org for more information and for policies about released
packages.

Preparing for Release
---------------------

#. `install bloom <http://ros-infrastructure.github.io/bloom/>`_.
   You'll be using it to release your package.
#. Ensure that your code is up to date, all tests pass, and that you've pushed all changes from
   your machine to the upstream repository. The upstream repository is the repository where you
   do your development and host the source code of your package. This repository can be hosted
   anywhere (even locally) and can be a git, hg, or svn repository or the location of an archive
   (tar.gz only for now, but there are plans for tar.bz and zip).

   Bloom has an important requirement for releasing your package.
   If your upstream repository is a vcs (git, hg, or svn), then it must have a tag matching the
   version you intend to release. For example, if you are going to release version 0.1.0 of your
   package, then bloom expects there to be a 0.1.0 tag in your upstream repository.
   **This tagging will be done automatically for you if you follow the rest of the tutorial,
   so there's no need to do it yourself right now.**

   If you have a custom version tagging scheme you'd like to use, then bloom can handle while
   configuring a release track (see below) that using the 'Release Tag' configuration.

Preparing the Upstream Repository
---------------------------------

This page will walk you through setting up your repository for a Bloom release.

.. note::

   This part of the release procedure will not work for ROS2 packages, since ROS2 uses a new build
   system. If using ROS2, you will have to create the changelog and release tags yourself. Please
   see the ROS 2 Bloom page for more information.

Update Changelogs
^^^^^^^^^^^^^^^^^

They are not mandatory but recommended (`REP-132 <https://www.ros.org/reps/rep-0132.html>`_).

Generate a Changelog
~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   catkin_generate_changelog

Run ``catkin_generate_changelog`` to generate or update ``CHANGELOG.rst`` file(s).
If one or more package(s) don't contain ``CHANGELOG.rst``, add ``--all`` option to populate all the
previous commits for each package.

Clean up the Changelog
~~~~~~~~~~~~~~~~~~~~~~

The command ``catkin_generate_changelog`` will simply populate them with the commit logs which are not
always appropriate for changelogs. Open ``CHANGELOG.rst`` and edit to your liking.
Here is an `example of a well formatted CHANGELOG.rst <https://github.com/ros/catkin/blob/groovy-devel/CHANGELOG.rst>`_.

Don't forget to complete the next step!

.. note::

   Incorrectly formatted ``CHANGELOG.rst`` can cause problems with your package.

.. note::

   If you have any commit messages ending in an underscore, such as member variables (e.g. ``name_``)
   this will throw an error with the RST Changelog format because RST treats those as
   `link targets <http://docutils.sourceforge.net/docs/user/rst/quickstart.html#sections>`_.
   The error will be something like:

   .. code-block:: bash

      <string>:21: (ERROR/3) Unknown target name: "name".

   To fix this, you'll need to escape the variable, for example:

   .. code-block:: bash

      * fix for checking the ``name_``

Commit the Changelog
~~~~~~~~~~~~~~~~~~~~

**This step is important, don't forget it!** Commit your new/updated changelog.

.. note::

   Some additional information on ``catkin_generate_changelog``, including some command line flags,
   can be found at the original `discussion thread <https://groups.google.com/forum/?hl=en#!msg/ros-sig-buildsystem/EQ4fzwvwYw0/245SJSFGqPMJ>`_.
   (This reference should eventually be replaced by a more authoritative documentation,
   rather than an email discussion thread).

Update package.xml Version
^^^^^^^^^^^^^^^^^^^^^^^^^^

You must bump the version in your ``package.xml`` file(s) and create a tag matching that version in
your upstream repository. `catkin <https://wiki.ros.org/catkin>`_ provides a tool for doing this,
and it is called ``catkin_prepare_release``:

.. code-block:: bash

   cd /path/to/your/upstream/repository
   catkin_prepare_release

This command will find all of the packages in your upstream repository, check that they have
changelogs (and then they have no uncommitted local changes), increment the version in your
``package.xml``'s, and commit/tag the changes with a bloom compatible flag. Using this command is
the best way to ensure you have a consistent and recommended release of your package.

By default this command increases the patch version of your package, e.g. ``0.1.1`` -> ``0.1.2``,
but you can pick minor or major using the ``--bump`` option.

Even if you do not use ``catkin_prepare_release``, you must have one or more valid
``package.xml`` (s) with the same version and a matching tag in your upstream repository.

Creating a Release Repository
-----------------------------

The next step is to create a release repository. Bloom requires that you have a separate
"release" repository for releasing your package. This repository must be a git repository
and is normally hosted externally by a service such as GitHub. For example, all of the ROS
release repositories are in the `ros-gbp github organization <https://github.com/ros-gbp>`_.
These repositories are the result of running bloom on a repository containing one or more
catkin packages.

We highly recommend that you host your release repository on `GitHub <https://github.com/>`_.
This tutorial uses GitHub, but you can also create your release repository locally and host it
somewhere else.

Creating a Release Repository on github.com
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Create a new repository in the github organization or on the github user of your choice.
By convention you should name it as your package name with the ``-release`` suffix.
So for the ``ros_comm`` repository the corresponding release repository is called
``ros_comm-release``.

.. note::

   When creating your github.com repository, check the box **Initialize this repository with a
   README.md**, this way it starts out as a valid git repo.
   Bloom will later fill this file with information about released versions.

Once you have created this new release repository then you are ready to configure and release
you package. Grab the release repository url from the github page, because you'll need it next.
