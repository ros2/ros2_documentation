Releasing a Third Party Package
===============================

.. contents:: Table of Contents
   :depth: 3
   :local:

In this context a third party package is a software package which exists outside of the ROS ecosystem, is used by packages in the ROS ecosystem, but is not released widely as a system dependency.
These software packages might be designed to be used outside of the ROS ecosystem, but are not big enough or mature enough to be released into operating system package managers.
Instead they are released using the ROS infrastructure along side a ROS distribution as if they were actually ROS packages.

See `REP-0136: Releasing Third Party, Non catkin Packages <http://ros.org/reps/rep-0136.html>`_ for a detailed explanation about releasing third party packages.

The details relating to these requirements are in the REP.

A third party package can accomplish these requirements in two ways: in the upstream repository, or in the release repository.

Install bloom
-------------

`Bloom <http://ros-infrastructure.github.io/bloom/>`_ is a release automation tool, designed to generate platform specific release artifacts from source projects.

On Ubuntu the recommended installation method is to use ``apt``:

.. code-block:: bash

   sudo apt install python3-bloom

.. note::

   On non-debian systems you can install via pypi:

   .. code-block:: bash

      pip3 install -U bloom

Modifying the Upstream Repository
---------------------------------

By adding a ``package.xml`` and a rule to install it in the upstream repository, the package looks just like an ament package to the ROS ecosystem.
This is a clean solution because it doesn't require any modifications to the normal release process and has almost no impact on the upstream repository.
Additionally, if the ``package.xml`` and install rule is in the upstream source code, then the third party package can be built along side ament packages even when checked out from source.
If you choose this method, then simply create a ``package.xml``, and add a rule to your build system to install the ``package.xml``.
Then, when you are ready to release, follow :doc:`Releasing an Ament Package <Releasing-an-Ament-Package>`.

Modifying the Release Repository
--------------------------------

Often times, however, putting a ``package.xml`` in the upstream repository is not an option.
In this case you can "inject" the ``package.xml`` into the release repository using bloom.

Clone Release Repository
^^^^^^^^^^^^^^^^^^^^^^^^

Clone the release repository you obtained access in :doc:`Obtain-Access-to-Release-Repository <Obtain-Access-to-Release-Repository>` to your local machine.

Entering the Release Settings
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

After creating the release repository, you will need to create your release track. You can create a new one from scratch or copy and edit an old configuration.
To create a new track:

.. code-block:: bash

   # In your cloned release repository
   git-bloom-config new {DISTRO}

Alternatively, to copy and edit from an old track (eg. copying from a galactic track):

.. code-block:: bash

   # In your cloned release repository
   git-bloom-config copy galactic {DISTRO}
   git-bloom-config edit {DISTRO}

Where ``{DISTRO}`` is the name of the track you created (and is typically the name of the rosdistro you want to release to).
Follow the instructions for configuring a release track to enter the configuration.

Repository Name
~~~~~~~~~~~~~~~

.. code-block:: bash

   Repository Name:
      <name>
         Name of the repository (used in the archive name)
      upstream
         Default value, leave this as upstream if you are unsure
      ['upstream']:

This name is trivial, but can be used to provide additional tags and to create nicer archive names.
Leave this as ``upstream`` if you are unsure.

Upstream Repository URI
~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   Upstream Repository URI:
      <uri>
         Any valid URI. This variable can be templated, for example an svn url
         can be templated as such: "https://svn.foo.com/foo/tags/foo-:{version}"
         where the :{version} token will be replaced with the version for this release.
      [None]:

You should put the uri of your third party library code. (eg. ``https://github.com/bar/foo.git``)

Upstream VCS Type
~~~~~~~~~~~~~~~~~

.. code-block:: bash

   Upstream VCS Type:
      git
         Upstream URI is a git repository
      hg
         Upstream URI is a hg repository
      svn
         Upstream URI is a svn repository
      tar
         Upstream URI is a tarball
      ['git']:

You must specify the type of upstream repository you are using.
Leave this as ``git``, unless your upstream repository is of a different type (``svn``, ``hg``, or hosted ``tar`` archives).

Version
~~~~~~~

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

Set this to ``:{ask}``, so bloom asks for the package version during the release process.

Release Tag
~~~~~~~~~~~

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
      ['None']:

The Release Tag refers to which tag or branch you want to import the code from.
If you always want to pull in the latest ``master`` branch at the time of release from the upstream project, enter ``master``.

Alternatively, if you want to be prompted to enter a different tag every time you do a release, enter ``:{ask}``.
This is useful if the upstream project has frequent tagged releases and you want to refer to the new tag every time you're releasing.

Upstream Devel Branch
~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   Upstream Devel Branch:
      <vcs reference>
         Branch in upstream repository on which to search for the version.
         This is used only when version is set to ':{auto}'.
      [None]:

Leave this as ``none`` because it is unused when the version is set to ``:{ask}``.

ROS Distro
~~~~~~~~~~

.. code-block:: bash

   ROS Distro:
      <ROS distro>
         This can be any valid ROS distro, e.g. indigo, kinetic, lunar, melodic
      ['{DISTRO}']:

Set this to {DISTRO}.

Patches Directory
~~~~~~~~~~~~~~~~~

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
      ['rolling']:

Set this to {DISTRO} or any name you like.
This will be the folder in the ``master`` branch which contains your ``package.xml``.

Release Repository Push URL
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Can be left as the default in most cases.

Adding a Package.xml to the master branch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Now that you have informed bloom that there will be patches in the master branch under the {DISTRO} folder (or whatever you told it) you need to put a package.xml there for it to overlay onto the upstream have importing.
First change to the master branch and create the patches folder you specified above:

.. code-block:: bash

   git checkout master
   mkdir {DISTRO}

Where {DISTRO} is what you set `Patches Directory`_ to.

Now create ``package.xml`` in the folder you just created, using this as a reference:

.. tabs::

   .. group-tab:: CMake

      .. code-block:: xml

         <?xml version="1.0"?>
         <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
         <package format="3">
           <name>foo</name>
           <version>:{version}</version>
           <description>The foo package</description>
           <maintainer email="user@todo.todo">user</maintainer>
           <license>Apache License V2.0</license>

           <buildtool_depend>ament_cmake</buildtool_depend>

           <export>
             <build_type>ament_cmake</build_type>
           </export>
         </package>

   .. group-tab:: Python

      .. code-block:: xml

         <?xml version="1.0"?>
         <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
         <package format="3">
           <name>foo</name>
           <version>:{version}</version>
           <description>The foo package</description>
           <maintainer email="user@todo.todo">user</maintainer>
           <license>Apache License V2.0</license>

           <export>
             <build_type>ament_python</build_type>
           </export>
         </package>

The ``:{version}`` will be replaced by the version being released each time.

In the case described above, each time you run bloom on the release repository:

* the user will be prompted for the version being released
* an archive of the upstream source code will be fetched based on the "release tag" configuration
* imported into the release repository's upstream branch
* the package.xml is overlaid onto the upstream branch
* and the :{version} token in the package.xml is replaced by the version given by the user.

At this point you need to commit the package.xml template to the master branch:

.. code-block:: bash

   git add {DISTRO}/package.xml
   git commit -m "Added package.xml template"

Adding an Install Rule as a Patch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. note::

   Is this is not your first time releasing of this package, skip this step.

Before adding the install rule as a patch you need to run git-bloom-release once so that there is a release branch to patch:

.. code-block:: bash

   git-bloom-release {DISTRO}

Where ``{DISTRO}`` is the name of the track you created earlier.

You'll be prompted for the following:

Version
~~~~~~~

.. code-block:: bash

   What version are you releasing (version should normally be MAJOR.MINOR.PATCH)?

Enter a version for your package.
Follow the `ROS2 versioning guidelines <https://docs.ros.org/en/{DISTRO}/Contributing/Developer-Guide.html#versioning>`_.


After running once you can add your patch. Start by checking out the release branch:

.. code-block:: bash

   git checkout release/{DISTRO}/foo

Where the release tag is ``release/rosdistro/packagename``.

.. note::

   Note: Notice that the release template is based on the *package* name as opposed to the *repository* name.
   A repository can have multiple packages with in it, therefore there might be multiple **release/rosdistro/*** branches.
   You would need to make a similar install rule patch to each of them.

Now on this branch edit your build system to install the package.xml. In CMake it should look something like this:

.. code-block:: bash

   ...
   # Install package.xml
   install(FILES package.xml DESTINATION share/foo)
   ...

Where ``foo`` is the name of the package (the value in the ``<name>`` tag of the ``package.xml``).

Once you have added this to your build system, commit and push back to the remote:

.. code-block:: bash

   git add .
   git commit -m "Added install rule for package.xml"
   git-bloom-patch export
   git push

Now simply run ``git-bloom-release`` again:

.. code-block:: bash

   git-bloom-release {DISTRO}

Where ``{DISTRO}`` is the name of the track you created and released previously.
Now your release repository has been setup, you will not need to do anything special for future releases.

Adding additional patches to the upstream repository
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Follow the same process as patching in the ``package.xml`` installation from above.
Remember to call ``git-bloom-patch export`` after you've made more commits into ``release/{DISTRO}/foo`` to export the patches.

Porting patches from one rosdistro to another
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you've setup a number of patches to the upstream repo for an older rosdistro release,
and would like to port those patches to a newer rosdistro, then follow the instructions below:

First, perform a release for the newer rosdistro ({DISTRO}) to make sure there is a release branch to patch:

.. code-block:: bash

   git-bloom-release {DISTRO}

Then, checkout the patches from your older rosdistro (eg. foxy), and import them to the newer rosdistro ({DISTRO}):

.. code-block:: bash

   git checkout patches/release/{DISTRO}/foo
   git ls-tree --name-only -r patches/release/foxy/foo | grep '\.patch' | xargs -I {} sh -c 'git show patches/release/foxy/foo:"$1" > "$1"' -- {}
   git add .
   git commit -m "Importing patches from foxy release branch"
   git checkout release/{DISTRO}/foo
   git-bloom-patch import
   git push --all
   git push --tags

Then perform a release as usual:

.. code-block::

   git-bloom-release {DISTRO}

Opening a Pull Request
----------------------

Finally, you have to raise a Pull Request to add / update your repository in `rosdistro/{DISTRO}/distribution.yaml <https://github.com/ros/rosdistro/blob/master/{DISTRO}/distribution.yaml>`_.
Run:

.. code-block:: bash

   bloom-release --rosdistro {DISTRO} --track {DISTRO} foo --pull-request-only

You will be prompted to enter the following.

Release repository url
^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   Release repository url [press enter to abort]:

Enter your release repository here (eg. ``https://github.com/ros2-gbp/foo-release.git``)

Adding Documentation
^^^^^^^^^^^^^^^^^^^^

When bloom asks you for documentation, it is not necessarily asking for a link to any form of documentation.
It is asking for the source repository which could contain documentation (ie: in addition to source code).
In most cases, that will be the repository containing the C++/Python/whatever sources for your package, as that would result in automatic builds of **API documentation** and some other things.

.. code-block:: bash

   Would you like to add documentation information for this repository? [Y/n]?

If you would like API documentation to be generated automatically, simply press enter, or press ``Y`` and then Enter.
If you don't have API documentation, press ``n`` and then Enter.

.. code-block:: bash

   Please enter your repository information for the doc generation job.
   This information should point to the repository from which documentation should be generated.
   VCS Type must be one of git, svn, hg, or bzr.
   VCS Type:

You must specify the type of repository you are using.
Enter ``git``, unless your upstream repository is of a different type (``svn``, ``hg``, or ``bzr``), and press Enter.

.. code-block:: bash

   VCS url:

Enter your release repository here (eg. ``https://github.com/ros2-gbp/foo-release.git``)

.. code-block:: bash

   VCS version must be a branch, tag, or commit, e.g. master or 0.1.0
   VCS version:

Enter ``master`` here.

.. note::

   Even if you don't have documentation in your repository, add this field.
   Pull Requests to `rosdistro <https://github.com/ros/rosdistro>`_ without a documentation field will get disapproved.

Adding Source Repository
^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   Please enter information which points to the active development branch for this repository.
   This information is used to run continuous integration jobs and for developers to checkout from.
   VCS Type must be one of git, svn, hg, or bzr.
   VCS type:

You must specify the type of repository you are using.
Enter ``git``, unless your upstream repository is of a different type (``svn``, ``hg``, or ``bzr``), and press Enter.

.. code-block:: bash

   VCS url:

Enter your release repository here (eg. ``https://github.com/ros2-gbp/foo-release.git``)

.. code-block:: bash

   VCS version must be a branch, tag, or commit, e.g. master or 0.1.0
   VCS version:

Enter ``releases/{DISTRO}/foo`` here.

Pull Request Testing
^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   Would you like to turn on pull request testing? [y/N]? 

Simply press Enter, or type ``N`` and press enter.

Maintenance Status
^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   Would you like to add a maintenance status for this repository? [Y/n]? 

Simply press Enter, or type ``Y`` and press enter.

.. code-block:: bash

   Please enter a maintenance status.
   Valid maintenance statuses:
   - developed: active development is in progress
   - maintained: no new development, but bug fixes and pull requests are addressed
   - unmaintained: looking for new maintainer, bug fixes and pull requests will not be addressed
   - end-of-life: should not be used, will disappear at some point
   Status:

Type ``maintained``, and press Enter.

.. code-block:: bash

   You can also enter a status description.
   This is usually reserved for giving a reason when a status is 'end-of-life'.
   Status Description [press Enter for no change]:

Simply press Enter.

Open a Pull Request?
^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   Open a pull request from 'sample_user/rosdistro:bloom-foo-1' into 'ros/rosdistro:master'?
   Continue [Y/n]?

Simply press Enter, or type ``Y`` and press Enter.
This should open the PR for you.

Future Releases
---------------

.. fill this out

Troubleshooting
---------------

There are a few more details which might be necessary for some releases.

Custom Build Commands
^^^^^^^^^^^^^^^^^^^^^

Some packages require more options than the standard ``cmake && make && make install`` to be built, and some other packages are not even CMake.
In these cases the ``rules`` file in the debian folder needs to be modified.
To do this run the ``git-bloom-release`` command at least once and then checkout to the debian branch:

.. code-block:: bash

   git checkout debian/{DISTRO}/foo

Where foo is the name of the package.

In this branch there should be a ``debian`` folder containing the template files, among them: ``rules.em``.
Edit this file to fit your needs and then commit the changes:

.. code-block:: bash

   git add debian/rules.em
   git commit -m "Customized debian rules file"
   git-bloom-patch export

Then rerun bloom:

.. code-block:: bash

   git-bloom-release {DISTRO}

Where {DISTRO} is the name of the track you wish to run.
