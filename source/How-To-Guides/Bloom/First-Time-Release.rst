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

Releasing Your Packages
-----------------------

.. note::

   If you have two factor authorization enabled on github,
   please follow this tutorial first:
   `GithubManualAuthorization <https://wiki.ros.org/bloom/Tutorials/GithubManualAuthorization>`_.

Normally you will make a call like this:

.. code-block:: bash

   # This is an example, do not run this one, run the next one
   bloom-release --rosdistro <ros_distro> --track <ros_distro> repository_name

In order to release the packages from a repository, but on the first release
(and any time you want to configure a new 'track' of settings) you'll want to add the
``--edit`` option:

.. code-block:: bash

   # Replace <ros_distro> with the ROS distribution, e.g. indigo
   $ bloom-release --rosdistro <ros_distro> --track <ros_distro> <your_repository_name> --edit

This option will allow you to edit the track specified before making a release.
This is required on the first release as you do not yet have a track, so bloom will first create
one for you and then allow you to configure it. Please note that ``repository_name`` is not its url,
it is its reference in ``distribution.yaml``.

When you run the above command, it will go out to the ROS distro file for the ROS distro which
you specified and look for your repository's information. Since this is your first release, it
will not find your repository's information, so it will ask you for the release repository url,
like this:

.. code-block:: bash

   No reasonable default release repository url could be determined from previous releases.
   Release repository url [press enter to abort]:

You will only have to provide this information on the first release, but put your RELEASE
repository url here. This is the repository you just created above.

Next bloom may ask you about initializing the new repository.

.. code-block:: bash

   Freshly initialized git repository detected.
   An initial empty commit is going to be made.
   Continue [Y/n]?

Hit enter or type 'y' and then hit enter to continue.

Now bloom will setup a ``master`` branch (this is where the configurations are stored) and
begin prompting you for information about the release.

Configure a Release Track
^^^^^^^^^^^^^^^^^^^^^^^^^

bloom is designed to allow the release of the same package for different ROS distributions and
versions in the same release repository. To facilitate this, bloom uses release "tracks" to
maintain configurations for different release processes. For normal catkin-based ROS packages
the default release track is recommended.

In the ``bloom-release`` command you ran above, you specified the ``--track``.
By convention you should create tracks with the same name as the ROS distro you are releasing for,
but you could name your track what ever you wanted.

The first question provided to you is the repository name:

.. code-block:: bash

   Repository Name:
      upstream
         Default value, leave this as upstream if you are unsure
      <name>
         Name of the repository (used in the archive name)
      ['upstream']:

This name is trivial, but can be used to provide additional tags and to create nicer archive names.
Since our example has a single package called ``foo`` in the repository, it would be appropriate to
put ``foo`` here.

The next configuration is the upstream repository uri:

.. code-block:: bash

   Upstream Repository URI:
      <uri>
         Any valid URI. This variable can be templated, for example an svn url
         can be templated as such: "https://svn.foo.com/foo/tags/foo-:{version}"
         where the :{version} token will be replaced with the version for this release.
      [None]:

This is an important setting; you should put the uri of your repository on which you do development.
This is NOT the place where you intend to host this release repository. In this case,
I will pretend that our code is hosted in the ``bar`` organization on github and put
``https://github.com/bar/foo.git``.

Next, bloom will prompt you for the upstream repository type.

.. code-block:: bash

   Upstream VCS Type:
      svn
         Upstream URI is a svn repository
      git
         Upstream URI is a git repository
      hg
         Upstream URI is a hg repository
      tar
         Upstream URI is a tarball
      ['git']:

In this example our upstream repository is ``git``, but ``svn``, ``hg``, and hosted ``tar`` archives
are also supported.

The next few options (``Version`` and ``Release Tag``) should be okay to leave as the defaults
and are rarely changed unless you are releasing a non-catkin package.
Simply press enter to accept the default.

The next option you need to potentially modify is the upstream development branch:

.. code-block:: bash

   Upstream Devel Branch:
      <vcs reference>
         Branch in upstream repository on which to search for the version.
         This is used only when version is set to ':{auto}'.
      [None]:

This option is the branch of your upstream repository from which you tag releases.
If this is left ``None`` then the default branch is used when guessing the version being released.
If you want to search a branch besides the default branch, choose that.
For example, if you want to use the branch ``indigo-devel`` for this release track, enter
``indigo-devel``.

Next the ROS distro is required:

.. code-block:: bash

   ROS Distro:
      <ROS distro>
         This can be any valid ROS distro, e.g. groovy, hydro
      ['indigo']:

Enter the name of the ROS distro that this track is based on.

The rest of the configurations (``Patches Directory`` and ``Release Repository Push URL``) can be
left as the default in most cases.

Congratulations, you have successfully configured your release track.

There are many command which come with bloom, even though you will most likely only need
to run ``bloom-release``. Many of the bloom commands are prefixed with ``git-``, which indicates
that they must be run inside a git repository. If you clone your release repository manually,
then you can use ``git-`` prefixed commands to manually manipulate your release repository.
One of these commands is called ``git-bloom-config`` and it lets you manage your tracks.
Run ``git-bloom-config -h`` to get more information about how to manage your release tracks.

Finishing the Release
^^^^^^^^^^^^^^^^^^^^^

After your finished configuring your repository, ``bloom-release`` will do many things,
but generally it is cloning your release repository, performing all of the release tasks defined
in the ``actions`` section of your release track, pushing the result, and finally opening a pull
request on your behalf. If you configured your release repository correctly then your bloom
release should eventually succeed, after prompting you for you github credentials.
Once it is done, then it should provide you with a link to the newly created pull request.

Notifying the Build Farm
^^^^^^^^^^^^^^^^^^^^^^^^

Normally your ``bloom-release`` call should open a pull request for you, but if there is a
problem or you do not wish for it to open the pull request on your behalf you can manually open a
pull request also.
**If the automated pull request was opened successfully, then you do not need to open one manually
as described below.**

For each ROS distribution there is a distro file hosted on Github, for hydro it is:

`https://github.com/ros/rosdistro/blob/master/hydro/distribution.yaml <https://github.com/ros/rosdistro/blob/master/hydro/distribution.yaml>`_

You can open a pull request on this file by simply visiting the above URL and clicking the edit
button (note: you have to be logged into Github for this to work), make your changes and then
click "Propose Changes" at the bottom right of the page.

To enter your repository you need to fill out a section like this:

.. code-block:: yaml

   repositories:
      ...
      foo:
         tags:
            release: release/groovy/{package}/{version}
         url: https://github.com/ros-gbp/foo-release.git
         version: 0.1.0-0
      ...

Make sure to use the correct ROS distro name in the release tag (groovy in this case).

Note that you should put the **https://** url of the RELEASE repository here, not the url of your
source repository. Also note that you must put the full version which is the version of your
package plus the release increment number separated by a hyphen. The release increment number
is increased each time you release a package of the same version, this can occur when adding
patches to the release repository or when changing the release settings. Also note that you
should put your package into the list of packages in ALPHABETICAL order. Please.

.. note::

   If your repository contains multiple packages, their names must be listed in the distro file, too:

   .. code-block:: yaml

      repositories:
         ...
         foo:
            packages:
               foo_msgs:
               foo_server:
               foo_utils:
            tags:
               release: release/groovy/{package}/{version}
            url: https://github.com/ros-gbp/foo-release.git
            version: 0.1.0-0
         ...

   Again remember to use the correct ROS distro name for the release tag.

.. note::

   Each item in the list of packages must end with a colon.
   If necessary, a path to that package can be specified after the colon if it is not located
   in the repository root. For example:

   .. code-block:: yaml

      packages:
         foo_msgs: util/foo_msgs
         foo_server: tool/foo_server
