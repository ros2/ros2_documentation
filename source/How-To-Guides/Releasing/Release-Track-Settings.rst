Release Track Settings
======================

.. contents:: Table of Contents
   :depth: 3
   :local:


.. _track:

Track
-----

Bloom is designed to allow the release of the same package for different ROS distributions and versions in the same release repository.
To facilitate this, bloom uses release "tracks" to maintain configurations for different release processes.
By convention you should create tracks with the same name as the ROS distro you are releasing for.

.. _release-repository-url:

Release Repository url
----------------------

.. code-block:: bash

   No reasonable default release repository url could be determined from previous releases.
   Release repository url [press enter to abort]:

This is the url of your release repository, from :doc:`Obtain-Access-to-Release-Repository <Obtain-Access-to-Release-Repository>`.

bloom may ask you about initializing the new repository, as following:

.. code-block:: bash

   Freshly initialized git repository detected.
   An initial empty commit is going to be made.
   Continue [Y/n]?

Simply press enter or type ``y`` and then press enter.

.. _repository-name:

Repository Name
---------------

.. code-block:: bash

   Repository Name:
      upstream
         Default value, leave this as upstream if you are unsure
      <name>
         Name of the repository (used in the archive name)
      ['upstream']:

This name is trivial, but can be used to provide additional tags and to create nicer archive names.

.. _upstream-repository-uri:

Upstream Repository URI
-----------------------

.. code-block:: bash

   Upstream Repository URI:
      <uri>
         Any valid URI. This variable can be templated, for example an svn url
         can be templated as such: "https://svn.foo.com/foo/tags/foo-:{version}"
         where the :{version} token will be replaced with the version for this release.
      [None]:

The **upstream repository** is the repository where you do your development and host the source code of your package.
This repository can be hosted anywhere (even locally) and can be a git, hg, or svn repository or the location of an archive (tar.gz only for now, but there are plans for tar.bz and zip).

If you're using GitHub, make sure you **use the https address** (eg. ``https://github.com/bar/foo.git``) and not the ssh address (eg. ``git@github.com:bar/foo.git``)

.. _upstream-vcs-type:

Upstream VCS Type
-----------------

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

This is the `Upstream Repository URI`_'s version control system (VCS) type.
You must specify the type of vcs your repository is using, from  ``svn``, ``git``, ``hg`` or ``tar``.

.. _version:

Version
-------

.. code-block:: bash

   Version:
      :{ask}
         This means that the user will be prompted for the version each release.
         This also means that the upstream devel will be ignored.
      :{auto}
         This means the version will be guessed from the devel branch.
         This means that the devel branch must be set, the devel branch must exist,
         and there must be a valid package.xml in the upstream devel branch.
      <version>
         This will be the version used.
         It must be updated for each new upstream version.
      [':{auto}']:

This is the package release version.

.. _release-tag:

Release Tag
-----------

.. code-block:: bash

   Release Tag:
      :{version}
         This means that the release tag will match the :{version} tag.
         This can be further templated, for example: "foo-:{version}" or "v:{version}"

         This can describe any vcs reference. For git that means {tag, branch, hash},
         for hg that means {tag, branch, hash}, for svn that means a revision number.
         For tar this value doubles as the sub directory (if the repository is
         in foo/ of the tar ball, putting foo here will cause the contents of
         foo/ to be imported to upstream instead of foo itself).
      :{ask}
         This means the user will be prompted for the release tag on each release.
      :{none}
         For svn and tar only you can set the release tag to :{none}, so that
         it is ignored.  For svn this means no revision number is used.
      [':{version}']:

The Release Tag refers to which tag or branch you want to import the code from.
If you always want to pull in the latest ``master`` branch at the time of release from the upstream project, enter ``master``.

Alternatively, if you want to be prompted to enter a different tag every time you do a release, enter ``:{ask}``.
This is useful if the upstream project has frequent tagged releases and you want to refer to the new tag every time you're releasing.

.. _upstream-devel-branch:

Upstream Devel Branch
---------------------

.. code-block:: bash

   Upstream Devel Branch:
      <vcs reference>
         Branch in upstream repository on which to search for the version.
         This is used only when version is set to ':{auto}'.
      [None]:

You need to potentially modify this.
This option is the branch of your upstream repository from which you tag releases.
If this is left ``None`` then the default branch for your repository is used when guessing the version being released.
If you want to use a branch besides the default branch, choose that.
For example, if you want to use the branch ``rolling`` for this release track, enter ``rolling``.

.. _ros-distro:

ROS Distro
----------

.. code-block:: bash

   ROS Distro:
      <ROS distro>
         This can be any valid ROS distro, e.g. indigo, kinetic, lunar, melodic
      ['indigo']:

This is the distribution you're planning on releasing the package into.
If you plan on releasing into ROS {DISTRO}, enter ``{DISTRO}``.

.. _patches-directory:

Patches Directory
-----------------

.. code-block:: bash

   Patches Directory:
      <path in bloom branch>
         This can be any valid relative path in the bloom branch. The contents
         of this folder will be overlaid onto the upstream branch after each
         import-upstream.  Additionally, any package.xml files found in the
         overlay will have the :{version} string replaced with the current
         version being released.
      :{none}
         Use this if you want to disable overlaying of files.
      [None]:

This is only relevant if you're releasing a third party package.
This is the directory where your patches are.

.. _release-repository-push-url:

Release Repository Push URL
---------------------------

.. code-block:: bash

   Release Repository Push URL:
      :{none}
         This indicates that the default release url should be used.
      <url>
         (optional) Used when pushing to remote release repositories. This is only
         needed when the release uri which is in the rosdistro file is not writable.
         This is useful, for example, when a releaser would like to use a ssh url
         to push rather than a https:// url.
      [None]:

Can be left as the default in most cases.
