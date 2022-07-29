Release Track
=============

.. contents:: Table of Contents
   :depth: 2
   :local:

.. _what-is-a-track:

What is a Track?
----------------

Bloom requires the user to enter configuration information when releasing packages for the first time.
It is beneficial to store such configurations in the release repository so we don't have to manually enter configurations that won't change for subsequent releases.

Since some of the configurations will differ when releasing the package for different ROS distributions, bloom uses **release tracks to store the configurations for releasing** per distribution.
By convention you should create tracks with the same name as the ROS distro you are releasing for.

All release track configurations are stored in ``tracks.yaml`` on the master branch of your release repository.

Track Configurations
--------------------

Track configurations are explained in more detail along with the prompts from bloom.

.. _release-repository-url:

Release Repository url
^^^^^^^^^^^^^^^^^^^^^^

This is the url of your release repository, and should be of form ``https://github.com/ros2-gbp/my_repo-release.git`` if your release repository is hosted on ros2-gbp.

.. code-block:: bash

   No reasonable default release repository url could be determined from previous releases.
   Release repository url [press enter to abort]:

Paste your release repository URL and press Enter.

Bloom may additionally ask you about initializing the new repository, as following:

.. code-block:: bash

   Freshly initialized git repository detected.
   An initial empty commit is going to be made.
   Continue [Y/n]?

Simply press Enter to accept the default of yes.

.. _repository-name:

Repository Name
^^^^^^^^^^^^^^^

The repository name is trivial, but it is recommended to set this to the name of your project.

.. code-block:: bash

   Repository Name:
      upstream
         Default value, leave this as upstream if you are unsure
      <name>
         Name of the repository (used in the archive name)
      ['upstream']:

Type the name of your project (eg. ``my_project``) and press Enter.

.. _upstream-repository-uri:

Upstream Repository URI
^^^^^^^^^^^^^^^^^^^^^^^

The **upstream repository** is the repository where your source code is.
This is most likely an https link to your project hosted on a git hosting service such as GitHub or GitLab.

.. code-block:: bash

   Upstream Repository URI:
      <uri>
         Any valid URI. This variable can be templated, for example an svn url
         can be templated as such: "https://svn.foo.com/foo/tags/foo-:{version}"
         where the :{version} token will be replaced with the version for this release.
      [None]:

Make sure you **use the https address** (eg. ``https://github.com/my_organization/my_repo.git``) and not the ssh address.

.. _upstream-vcs-type:

Upstream VCS Type
^^^^^^^^^^^^^^^^^

This is the `Upstream Repository URI`_'s version control system (VCS) type.
You must specify the type of vcs your repository is using, from  ``svn``, ``git``, ``hg`` or ``tar``.

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

Most repositories will be using git, but some legacy repositories might be using hg or svn.

.. _version:

Version
^^^^^^^

This is the version of the package you are releasing. (eg. ``1.0.3``)

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

Setting this to ``:{auto}`` (the default, and recommended setup) will automatically determine the version from the devel branch's package.xml.

Setting this to ``:{ask}`` will bring up a prompt asking for the version every time you run a release with bloom.

.. _release-tag:

Release Tag
^^^^^^^^^^^

The Release Tag refers to which tag or branch you want to import the code from.

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

Setting this to ``:{version}`` (the default, and recommended setup) will make the release tag match the version tag.

A less common setup is to set this to a branch name to always pull in that branch at the time of release from the upstream project.

Alternatively, if you want to be prompted to enter a different tag every time you do a release, enter ``:{ask}``.
``:{ask}`` is useful if the upstream project has frequent tagged releases and you want to refer to the new tag every time you're releasing.

.. _upstream-devel-branch:

Upstream Devel Branch
^^^^^^^^^^^^^^^^^^^^^

The upstream devel branch is the name of the branch in your :ref:`upstream repository <upstream-repository-uri>`.
If you use separate branches for each ROS distribution, this field would be different for each release track.
It is used to determine the version of the package you are releasing when :ref:`Version` is set to ``:{auto}``.

.. code-block:: bash

   Upstream Devel Branch:
      <vcs reference>
         Branch in upstream repository on which to search for the version.
         This is used only when version is set to ':{auto}'.
      [None]:

To release from a branch called ``{DISTRO}``, enter ``{DISTRO}``.
Leaving this as ``None`` would result in the version being determined from the default branch of your repository (this is not recommended).

.. _ros-distro:

ROS Distro
^^^^^^^^^^

This is the distribution you're planning on releasing the package into.

.. code-block:: bash

   ROS Distro:
      <ROS distro>
         This can be any valid ROS distro, e.g. indigo, kinetic, lunar, melodic
      ['indigo']:

If you plan on releasing into ROS {DISTRO}, enter ``{DISTRO}``.

.. _patches-directory:

Patches Directory
^^^^^^^^^^^^^^^^^

This is the directory where any additional patches to the releases are.

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

Adding additional patches to a release is a rarely used feature.
For almost all packages, this should be left as the default ``None``.

.. _release-repository-push-url:

Release Repository Push URL
^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
