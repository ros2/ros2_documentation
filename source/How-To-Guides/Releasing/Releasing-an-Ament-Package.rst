Releasing an Ament Package
==========================

.. contents:: Table of Contents
   :depth: 3
   :local:

Before you start
----------------

Before releasing the package, here's a list of things that you'd want to check you've completed:

* Your code is up-to-date
* Your package builds correctly
* All your tests are passing
* You've pushed all changes from your machine to the upstream repository.

.. note::

   The **upstream repository** is the repository where you do your development and host the source code of your package.
   This repository can be hosted anywhere (even locally) and can be a git, hg, or svn repository or the location of an archive (tar.gz only for now, but there are plans for tar.bz and zip).

Install catkin_pkg and bloom
----------------------------

`catkin_pkg <https://github.com/ros-infrastructure/catkin_pkg>`_ is a python library that contains scripts to be used in the steps `Writing a Change Log`_ and `Bump the package version`_.
Despite the use of the word *catkin* (which is the ROS1 equivalent of ament) in the library name, it works for preparing ament packages too.

`Bloom <http://ros-infrastructure.github.io/bloom/>`_ is a release automation tool, designed to generate platform specific release artifacts from source projects in the `Releasing Your Packages`_ step.

On Ubuntu the recommended installation method is to use ``apt``:

.. code-block:: bash

   sudo apt install python3-catkin-pkg python3-bloom 

.. note::

   On non-debian systems you can install via pypi:

   .. code-block:: bash

      pip3 install -U catkin_pkg bloom

Writing a Change Log
--------------------

To make it easier for users and contributors to see precisely what notable changes have been made between each release, you are recommended to have a changelog for your package.

In the steps below, the package changelogs will be incorporated as part of the source working tree as recommended in `REP-132: Incorporation of Changelogs into Package Source Tree <https://www.ros.org/reps/rep-0132.html>`_.

Generate / Update CHANGELOG.rst
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If this is the first time you're releasing the package, you'll likely not have a ``CHANGELOG.rst`` file in your package yet.
Rather than creating this file manually, let the ``--all`` flag generate the file for you.

If this is not your first release and you already have a ``CHANGELOG.rst`` per package, omit the ``--all`` flag and let the script make an entry for the upcoming release for you.

.. code-block:: bash

   # Run either:
   catkin_generate_changelog
   catkin_generate_changelog --all  # To generate CHANGELOG.rst

Clean up the Changelog
^^^^^^^^^^^^^^^^^^^^^^

Open ``CHANGELOG.rst`` in an editor.
You will see that the command ``catkin_generate_changelog`` from the previous step has simply populated it with commit messages, like below:

.. code-block:: rst

   ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
   Changelog for package your_package
   ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

   Forthcoming
   -----------
   * you can modify commit message here
   * and here

You should clean up the list of commit messages to concisely convey  to your users and maintainers, the notable changes have been made since the last release.

See `rclcpp's CHANGELOG.rst <https://github.com/ros2/rclcpp/blob/master/rclcpp/CHANGELOG.rst>`_ for a well-formatted example.
Incorrectly formatted ``CHANGELOG.rst`` can cause problems with your package.

.. note::

   You should **not** modify the ``Forthcoming`` heading, as this will be replaced with the package version number by ``catkin_prepare_release`` later on.

.. warning::

   If you have any commit messages ending in an underscore, such as member variables (e.g. ``name_``) this will throw an error with the RST Changelog format because RST treats those as `link targets <http://docutils.sourceforge.net/docs/user/rst/quickstart.html#sections>`_.
   The error will be something like:

   .. code-block::

      <string>:21: (ERROR/3) Unknown target name: "name".

   To fix this, you'll need to escape the variable, for example:

   .. code-block::

      * fix for checking the ``name_``

Commit the Changelog
^^^^^^^^^^^^^^^^^^^^

**Don't forget this step!**
Commit the ``CHANGELOG.rst`` files you cleaned up.

Bump the package version
------------------------

Every release of the package must have a unique version number.
Run:

.. code-block:: bash

   catkin_prepare_release

which performs the following:

#. increases the package version in ``package.xml``
#. replaces the heading ``Forthcoming`` with ``version (date)`` (eg. ``0.0.1 (2022-01-08)``) in ``CHANGELOG.rst``
#. commits those changes
#. creates a tag (eg. ``0.0.1``)
#. pushes those changes to upstream

.. note::

   By default this command increases the patch version of your package, e.g. ``0.1.1`` -> ``0.1.2``, but you can pick minor or major using the ``--bump`` option.

.. note::

   Even if you do not use ``catkin_prepare_release``, you must have one or more valid ``package.xml`` with the same version and a matching tag in your upstream repository.
   For example, if you are going to release version 0.1.0 of your package, then bloom expects there to be a 0.1.0 tag in your upstream repository.

   If you have a custom version tagging scheme you'd like to use, then bloom can handle while configuring a release track using the 'Release Tag' configuration.

Releasing Your Packages
-----------------------

.. note::

   If you have two factor authorization enabled on github, follow :doc:`Github Manual Authorization <Github-Manual-Authorization>` first.

The actual releasing of the package should be performed using one of the commands below, where you should replace ``foo`` with the name of your repository:

* Releasing a package for the first time, for a new distro, or editing an existing release track:

   .. code-block:: bash

      bloom-release --rosdistro {DISTRO} --track {DISTRO}  --edit foo

* Releasing a package update on an existing release track:

   .. code-block:: bash

      bloom-release --rosdistro {DISTRO} foo

.. tip::

   * ``--rosdistro {DISTRO}`` indicates that this release is for the ``{DISTRO}`` distro
   * ``--track {DISTRO}`` indicates that you want the track name to be ``{DISTRO}``
   * ``--edit`` tells bloom to create the track if it doesn't exist and configure it.

If you used the ``--edit`` flag, continue with `Configuring the Release Track`_.
If you're releasing a package update on an existing release track without editing it, you can skip the next section.

Configuring the Release Track
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. warning::

   If you're releasing a package update without configuring the track, skip these instructions

The ``bloom-release`` script will prompt you through to perform the following:

* Setup a new track and configure it
* Generate platform specific release artifacts
* Push them to your release repository
* Fork `rosdistro <https://github.com/ros/rosdistro>`_ to your github account and open a Pull Request back upstream with your package to added to ``{DISTRO}/distribution.yaml``.

bloom is designed to allow the release of the same package for different ROS distributions and versions in the same release repository.
To facilitate this, bloom uses release "tracks" to maintain configurations for different release processes.
For normal ament-based ROS packages the default release track is recommended.

In the ``bloom-release`` command you ran above, you specified the ``--track``.
By convention you should create tracks with the same name as the ROS distro you are releasing for, but you could name your track what ever you wanted.

Release Repository url
~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   No reasonable default release repository url could be determined from previous releases.
   Release repository url [press enter to abort]:

Put your release repository on ros2-gbp here (eg. ``https://github.com/ros2-gbp/foo``).
Next bloom may ask you about initializing the new repository:

.. code-block:: bash

   Freshly initialized git repository detected.
   An initial empty commit is going to be made.
   Continue [Y/n]?

Hit enter or type ``y`` and then hit enter to continue.

Repository Name
~~~~~~~~~~~~~~~

.. code-block:: bash

   Repository Name:
      upstream
         Default value, leave this as upstream if you are unsure
      <name>
         Name of the repository (used in the archive name)
      ['upstream']:

This name is trivial, but can be used to provide additional tags and to create nicer archive names.
Since our example has a single package called ``foo`` in the repository, it would be appropriate to put ``foo`` here.

Upstream Repository URI
~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   Upstream Repository URI:
      <uri>
         Any valid URI. This variable can be templated, for example an svn url
         can be templated as such: "https://svn.foo.com/foo/tags/foo-:{version}"
         where the :{version} token will be replaced with the version for this release.
      [None]:

This is an important setting.
You should put the uri of your code repository (eg. ``https://github.com/bar/foo.git``).

Upstream Repository Type
~~~~~~~~~~~~~~~~~~~~~~~~

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

You must specify the type of upstream repository you are using.
Leave this as ``git``, unless your upstream repository is of a different type (``svn``, ``hg``, or hosted ``tar`` archives).

Version
~~~~~~~

Press enter to accept the default unless you are releasing a non-ament package.

Release Tag
~~~~~~~~~~~

Press enter to accept the default unless you are releasing a non-ament package.

Upstream Devel Branch
~~~~~~~~~~~~~~~~~~~~~

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
For example, if you want to use the branch ``ros2`` for this release track, enter ``ros2``.

ROS Distro
~~~~~~~~~~

Next the ROS distro is required:

.. code-block:: bash

   ROS Distro:
      <ROS distro>
         This can be any valid ROS distro, e.g. indigo, kinetic, lunar, melodic
      ['indigo']:

Type ``{DISTRO}`` and press enter.

Patches Directory
~~~~~~~~~~~~~~~~~

Can be left as the default in most cases.

Release Repository Push URL
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Can be left as the default in most cases.

.. There are many command which come with bloom, even though you will most likely only need
.. to run ``bloom-release``. Many of the bloom commands are prefixed with ``git-``, which indicates
.. that they must be run inside a git repository. If you clone your release repository manually,
.. then you can use ``git-`` prefixed commands to manually manipulate your release repository.
.. One of these commands is called ``git-bloom-config`` and it lets you manage your tracks.
.. Run ``git-bloom-config -h`` to get more information about how to manage your release tracks.

Pull Request to rosdistro
-------------------------

.. warning::

  If the automated pull request was opened successfully, then you **do not need to open one manually** as described here.
  You can simply skip this section.

Normally your ``bloom-release`` call should open a pull request for you, but if there is a problem or you do not wish for it to open the pull request on your behalf you can manually open a pull request also.

In the unlikely case that the automated pull request does not open successfully, you will have to manually open a pull request with modifications to rosdistro.

You can open a pull request by simply visiting `{DISTRO}/distribution.yaml <https://github.com/ros/rosdistro/blob/master/{DISTRO}/distribution.yaml>`_ and clicking the edit button (note: you have to be logged into Github for this to work), make your changes and then click "Propose Changes" at the bottom right of the page.

To enter your repository you need to fill out a section like this:

.. code-block:: yaml  

   foo:
     doc:
       type: git
       url: https://github.com/bar/foo.git
       version: ros2
     release:
       tags:
         release: release/{DISTRO}/{package}/{version}
       url: https://github.com/ros2-gbp/foo-release.git
       version: 0.0.1-1
     source:
       type: git
       url: https://github.com/bar/foo.git
       version: ros2
     status: developed

You should put the **https://** url of the RELEASE repository here, not the url of your upstream repository.

.. note::

   * put the full version which is the version of your package plus the release increment number separated by a hyphen. (eg. ``0.0.1-1``).
     The release increment number is increased each time you release a package of the same version.
     This can occur when adding patches to the release repository or when changing the release settings.
   * Put your package into the list of packages in ALPHABETICAL order.

.. note::

   If your repository contains multiple packages, their names must be listed in the distro file, too.
   For example if the repository contains two packages ``baz`` and ``qux`` they will be listed as below: 

   .. code-block:: yaml  

      foo:
        doc:
          type: git
          url: https://github.com/bar/foo.git
          version: ros2
        release:
          packages:
          - baz
          - qux
          tags:
            release: release/{DISTRO}/{package}/{version}
          url: https://github.com/ros2-gbp/foo-release.git
          version: 0.0.1-1
        source:
          type: git
          url: https://github.com/bar/foo.git
          version: ros2
        status: developed

Next Steps
----------

Once your pull request has been submitted, one of the ROS developers will merge your request (this usually happens fairly quickly).
24-48 hours after that, your package should be built by the build farm and released into the building repository.
Packages built are periodically synchronized over to the `shadow-fixed <https://wiki.ros.org/ShadowRepository>`_ and public repositories, so it might take as long as a month before your package is available on the public ROS debian repositories (i.e. available via ``apt-get``).
To get updates on when the next synchronization (sync) is coming, check the `ROS discussion forums <https://discourse.ros.org/>`_.

Individual build details are on the Jenkins build farm `build.ros2.org <http://build.ros2.org/>`__.
Check `ROS {DISTRO} Default Package Status <http://repo.ros2.org/status_page/ros_{DISTRO}_default.html>`__ to see status of released packages.
