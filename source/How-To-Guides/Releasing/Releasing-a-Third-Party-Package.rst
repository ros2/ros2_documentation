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

.. include:: _Install-Bloom.rst

Modifying the Upstream Repository
---------------------------------

By adding a ``package.xml`` and a rule to install it in the upstream repository, the package looks just like an ament package to the ROS ecosystem.
This is a clean solution because it doesn't require any modifications to the normal release process and has almost no impact on the upstream repository.
Additionally, if the ``package.xml`` and install rule is in the upstream source code, then the third party package can be built along side ament packages even when checked out from source.
If you choose this method, then simply create a ``package.xml``, and add a rule to your build system to install the ``package.xml``.
Then, when you are ready to release, follow :doc:`First Time Release <First-Time-Release>`.

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


Let's look at a common scenario.

You are trying to release a third party library called ``foo`` hosted on GitHub at ``https://github.com/bar/foo.git``.
You want a mirror of the ``main`` branch from the library to be released it into the ROS ecosystem.
You already have an empty release repository (``https://github.com/ros2-gbp/foo-release.git``), from :doc:`Obtain-Access-to-Release-Repository <Obtain-Access-to-Release-Repository>`.

For this scenario, the table below summarises the responses to the questions:

.. list-table::
   :header-rows: 1
   :widths: 1 2

   * - Configuration
     - Value
   * - :ref:`Release Repository url <release-repository-url>`
     - ``https://github.com/ros2-gbp/foo.git``
   * - :ref:`Repository Name <repository-name>`
     - ``foo``
   * - :ref:`Upstream Repository URI <upstream-repository-uri>`
     - ``https://github.com/bar/foo.git``
   * - :ref:`Upstream VCS Type <upstream-vcs-type>`
     - ``git``
   * - :ref:`Version <version>`
     - ``:{ask}``
   * - :ref:`Release Tag <release-tag>`
     - ``main``
   * - :ref:`Upstream Devel Branch <upstream-devel-branch>`
     -
   * - :ref:`ROS Distro <ros-distro>`
   -
   * - :ref:`Patches Directory <patches-directory>`
     - ``patch``
   * - :ref:`Release Repository Push URL <release-repository-push-url>`
     -

Adding a Package.xml to the Patches Directory of the master branch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Change to the master branch and create a directory matching your *Patches Directory* configuration:

.. code-block:: bash

   git checkout master
   mkdir patch

Now create ``package.xml`` in the folder you just created, using the following as a reference:

.. tabs::

   .. group-tab:: CMake

      .. code-block:: xml

         <?xml version="1.0"?>
         <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
         <package format="3">
           <name>foo</name>  <!--Change this-->
           <version>:{version}</version>  <!--DON'T change this-->
           <description>The foo package</description>  <!--Change this-->
           <maintainer email="user@todo.todo">user</maintainer>  <!--Change this-->
           <license>Apache License V2.0</license>  <!--Change this to library's license-->

           <buildtool_depend>ament_cmake</buildtool_depend>

           <!--Add dependencies here using <depend></depend> tag-->

           <export>
             <build_type>ament_cmake</build_type>
           </export>
         </package>

   .. group-tab:: Python

      .. code-block:: xml

         <?xml version="1.0"?>
         <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
         <package format="3">
           <name>foo</name>  <!--Change this-->
           <version>:{version}</version>  <!--DON'T change this-->
           <description>The foo package</description>  <!--Change this-->
           <maintainer email="user@todo.todo">user</maintainer>  <!--Change this-->
           <license>Apache License V2.0</license>  <!--Change this to library's license-->

           <!--Add dependencies here using <depend></depend> tag-->

           <export>
             <build_type>ament_python</build_type>
           </export>
         </package>

Fill out the ``package.xml`` template appropriately with information about your package.
Make sure to delete the comments from the template.

**If your third party library has dependencies, you must add them**, as you would usually with an ament package.

.. tip::

   The ``:{version}`` will be replaced by the version being released each time.

At this point you need to commit the package.xml template to the master branch:

.. code-block:: bash

   git add patch/package.xml
   git commit -m "Added package.xml template"

Below is an explanation of the patching process during the release process:

#. User enters the version to be released
#. The *Release Tag* branch (or tag) of the *Upstream Repository URI* will be copied into the release repository's *upstream* branch.
#. Files in the Release Repository master branch's *Patches Directory* are overlaid onto the *upstream* branch.
#. The ``:{version}`` token in the package.xml is replaced by the package version entered by the user.

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
Follow the `ROS 2 versioning guidelines <https://docs.ros.org/en/{DISTRO}/Contributing/Developer-Guide.html#versioning>`_.


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
