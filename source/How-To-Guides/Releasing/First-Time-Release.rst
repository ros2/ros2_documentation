Releasing for the First Time
============================

.. contents:: Table of Contents
   :depth: 3
   :local:

This guide will help you release a brand new ROS package.
By default, bloom releases packages to the public ROS build farm, which is what this tutorial
teaches you to do. See http://build.ros.org for more information and for policies about released
packages.

Request a Release Repository
----------------------------

To release a package, you must request for a *release* repository to be created in
`ROS 2 release repositories (ros2-gbp) <https://github.com/ros2-gbp>`_ to store the bloom-generated files.

You first must be part of the release team on ros2-gbp you plan to release the project under.
If no release team exists for your project, fill out the
`New Release Team <https://github.com/ros2-gbp/ros2-gbp-github-org/issues/new?assignees=&labels=&template=new_release_team.md&title=Add+release+team>`_
issue template to request for a release team to be created.
If a release team already exists for your project but you are not part of it, request to be added by
by filling out the
`Update Release Team Membership <https://github.com/ros2-gbp/ros2-gbp-github-org/issues/new?assignees=&labels=&template=update_release_team_membership.md&title=Update+release+team+membership>`_
issue template.

If you are already part of the release team for your project, fill out the
`Add New Release Repositories <https://github.com/ros2-gbp/ros2-gbp-github-org/issues/new?assignees=&labels=&template=new_release_repository.md&title=Add+new+release+repositories>`_
issue template.

Now, you must wait for the ros2-gbp maintainers to create an empty release repository for you.
Once the release repository is available, you should have push access to it. 
Note the release repository's url because you'll need it later.

Install Bloom
-------------

`Bloom <http://ros-infrastructure.github.io/bloom/>`_ is a release automation tool,
designed to make generating platform specific release artifacts from source projects easier.

On Ubuntu the recommend method is to use apt:

.. code-block:: bash

   sudo apt-get install python-bloom

On other systems you can install bloom via pypi:

.. code-block:: bash

   sudo pip install -U bloom

Prerelease Check List
---------------------

Ensure that you've completed all the below:

* your code is up to date
* all tests pass
* you've pushed all changes from your machine to the upstream repository.

.. note::

   The upstream repository is the repository where you do your development and host the source
   code of your package. This repository can be hosted anywhere (even locally) and can be a git,
   hg, or svn repository or the location of an archive (tar.gz only for now, but there are plans
   for tar.bz and zip).

Writing a Change Log
--------------------

To make it easier for users and contributors to see precisely what notable changes have been made
between each release (or version) of the project, you are recommended to have a changelog for your
package.
The recommended (in `REP-132 <https://www.ros.org/reps/rep-0132.html>`_) method of incorporating
package changelogs as part of the source working tree rather than maintaining one separately
will be covered here.

Generate CHANGELOG.rst
^^^^^^^^^^^^^^^^^^^^^^

Since this is the first time you're releasing the package, you'll likely not have a
``CHANGELOG.rst`` file in your package yet. Rather than creating this file manually, let the
following command generate the file and make an entry for the upcoming release for you:

.. code-block:: bash

   catkin_generate_changelog --all

.. note::

   If you do have an existing ``CHANGELOG.rst`` file, run:

   .. code-block::

      catkin_generate_changelog

   without the ``--all`` option.

Clean up the Changelog
^^^^^^^^^^^^^^^^^^^^^^

Open ``CHANGELOG.rst`` in an editor. You will see that the command ``catkin_generate_changelog``
from the previous step has filled in your CHANGELOG.rst from previous commit messages, like below:

.. code-block:: rst

   ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
   Changelog for package your_package
   ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

   Forthcoming
   -----------
   * you can modify commit message here
   * and here

You should clean up the list of commit messages in the changelog to
concisely convey the exact changes that have happened since the last release to your users.

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

Commit the Changelog
^^^^^^^^^^^^^^^^^^^^

Don't forget to **commit your new changelog**.

Set package version
^^^^^^^^^^^^^^^^^^^

Set the new package's version by running:

.. code-block:: bash

   catkin_prepare_release

The script performs the following:

#. replaces the heading ``Forthcoming`` with ``version (date)`` (eg. ``0.0.1 (2022-01-08)``) in ``CHANGELOG.rst``
#. increases the package version in the ``package.xml`` file(s)
#. commits those changes
#. creates a tag (eg. ``0.0.1``)
#. pushes those changes to upstream

.. By default this command increases the patch version of your package, e.g. ``0.1.1`` -> ``0.1.2``,
.. but you can pick minor or major using the ``--bump`` option.

.. Even if you do not use ``catkin_prepare_release``, you must have one or more valid
.. ``package.xml`` (s) with the same version and a matching tag in your upstream repository.

.. Bloom has an important requirement for releasing your package.
.. If your upstream repository is a vcs (git, hg, or svn), then it must have a tag matching the
.. version you intend to release. For example, if you are going to release version 0.1.0 of your
.. package, then bloom expects there to be a 0.1.0 tag in your upstream repository.
.. **This tagging will be done automatically for you if you follow the rest of the tutorial,
.. so there's no need to do it yourself right now.**

.. If you have a custom version tagging scheme you'd like to use, then bloom can handle while
.. configuring a release track (see below) that using the 'Release Tag' configuration.

Releasing Your Packages
-----------------------

.. note::

   If you have two factor authorization enabled on github,
   please follow this tutorial first:
   `GithubManualAuthorization <https://wiki.ros.org/bloom/Tutorials/GithubManualAuthorization>`_.

The actual releasing of the package should be performed using the command below:

.. code-block:: bash

   bloom-release --rosdistro {DISTRO} --track {DISTRO} <your_repository_name> --edit

.. note::

   ``<your_repository_name>`` is not its url, it is its reference in ``{DISTRO}/distribution.yaml``.   

The script will prompt you through, to perform the following:

* Setup a new track and configure it
* Generate platform specific release artifacts
* Push them to your release repository
* Fork `rosdistro <https://github.com/ros/rosdistro>`_ to your github account and open a Pull
  Request back upstream with your package to added to ``{DISTRO}/distribution.yaml``.

When you run the above command, it will go out to the ROS distro file for the ROS distro which
you specified and look for your repository's information. Since this is your first release, it
will not find your repository's information, so it will ask you for the release repository url,
like this:

.. code-block:: bash

   No reasonable default release repository url could be determined from previous releases.
   Release repository url [press enter to abort]:

Put your RELEASE repository url here. This is the repository you requested on ros2-gbp.

Next bloom may ask you about initializing the new repository.

.. code-block:: bash

   Freshly initialized git repository detected.
   An initial empty commit is going to be made.
   Continue [Y/n]?

Hit enter or type ``y`` and then hit enter to continue.

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
For example, if you want to use the branch ``ros2`` for this release track, enter
``ros2``.

Next the ROS distro is required:

.. code-block:: bash

   ROS Distro:
      <ROS distro>
         This can be any valid ROS distro, e.g. indigo, kinetic, lunar, melodic
      ['indigo']:

Type ``{DISTRO}`` and press enter.

The rest of the configurations (``Patches Directory`` and ``Release Repository Push URL``) can be
left as the default in most cases.

Congratulations, you have successfully configured your release track.

.. There are many command which come with bloom, even though you will most likely only need
.. to run ``bloom-release``. Many of the bloom commands are prefixed with ``git-``, which indicates
.. that they must be run inside a git repository. If you clone your release repository manually,
.. then you can use ``git-`` prefixed commands to manually manipulate your release repository.
.. One of these commands is called ``git-bloom-config`` and it lets you manage your tracks.
.. Run ``git-bloom-config -h`` to get more information about how to manage your release tracks.

.. Finishing the Release
.. ^^^^^^^^^^^^^^^^^^^^^

.. After your finished configuring your repository, ``bloom-release`` will do many things,
.. but generally it is cloning your release repository, performing all of the release tasks defined
.. in the ``actions`` section of your release track, pushing the result, and finally opening a pull
.. request on your behalf. If you configured your release repository correctly then your bloom
.. release should eventually succeed, after prompting you for you github credentials.
.. Once it is done, then it should provide you with a link to the newly created pull request.

.. Notifying the Build Farm
.. ^^^^^^^^^^^^^^^^^^^^^^^^

.. Normally your ``bloom-release`` call should open a pull request for you, but if there is a
.. problem or you do not wish for it to open the pull request on your behalf you can manually open a
.. pull request also.
.. **If the automated pull request was opened successfully, then you do not need to open one manually
.. as described below.**

.. For each ROS distribution there is a distro file hosted on Github, for hydro it is:

.. `https://github.com/ros/rosdistro/blob/master/hydro/distribution.yaml <https://github.com/ros/rosdistro/blob/master/hydro/distribution.yaml>`_

.. You can open a pull request on this file by simply visiting the above URL and clicking the edit
.. button (note: you have to be logged into Github for this to work), make your changes and then
.. click "Propose Changes" at the bottom right of the page.

.. To enter your repository you need to fill out a section like this:

.. .. code-block:: yaml

..    repositories:
..       ...
..       foo:
..          tags:
..             release: release/groovy/{package}/{version}
..          url: https://github.com/ros-gbp/foo-release.git
..          version: 0.1.0-0
..       ...

.. Make sure to use the correct ROS distro name in the release tag (groovy in this case).

.. Note that you should put the **https://** url of the RELEASE repository here, not the url of your
.. source repository. Also note that you must put the full version which is the version of your
.. package plus the release increment number separated by a hyphen. The release increment number
.. is increased each time you release a package of the same version, this can occur when adding
.. patches to the release repository or when changing the release settings. Also note that you
.. should put your package into the list of packages in ALPHABETICAL order. Please.

.. .. note::

..    If your repository contains multiple packages, their names must be listed in the distro file, too:

..    .. code-block:: yaml

..       repositories:
..          ...
..          foo:
..             packages:
..                foo_msgs:
..                foo_server:
..                foo_utils:
..             tags:
..                release: release/groovy/{package}/{version}
..             url: https://github.com/ros-gbp/foo-release.git
..             version: 0.1.0-0
..          ...

..    Again remember to use the correct ROS distro name for the release tag.

.. .. note::

..    Each item in the list of packages must end with a colon.
..    If necessary, a path to that package can be specified after the colon if it is not located
..    in the repository root. For example:

..    .. code-block:: yaml

..       packages:
..          foo_msgs: util/foo_msgs
..          foo_server: tool/foo_server

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
