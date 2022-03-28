Release
=======

.. contents:: Table of Contents
   :depth: 3
   :local:

Install Bloom
-------------

`Bloom <http://ros-infrastructure.github.io/bloom/>`_ is a release automation tool,
designed to make generating platform specific release artifacts from source projects easier. 

On Ubuntu the recommended installation method is to use apt:

.. code-block:: bash

   sudo apt install python3-bloom

.. note::

   On non-debian systems you can install via pypi:

   .. code-block:: bash

      pip3 install -U bloom


Releasing Your Packages
-----------------------

.. note::

   If you have two factor authorization enabled on github, follow
   :doc:`Github Manual Authorization <Github-Manual-Authorization>` first.

The actual releasing of the package should be performed using one of the commands below, where
you should replace ``foo`` with the name of your repository:

* Releasing a package for the first time, or when editing an existing release track:

   .. code-block:: bash

      bloom-release --rosdistro {DISTRO} --track {DISTRO}  --edit foo

* Releasing for a new distro:

   .. code-block:: bash

      bloom-release --rosdistro {DISTRO} --track {DISTRO} --new-track foo

* Releasing a package update on an existing release track:

   .. code-block:: bash

      bloom-release --rosdistro {DISTRO} foo


Configuring a Release Track
^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you're releasing a package update without configuring the track, you can skip the instructions
here.
If you ran the script with the ``--edit`` or ``--new-track`` flag, you will be prompted
to configure the track. Follow the instructions here:

The ``bloom-release`` script will prompt you through to perform the following:

* Setup a new track and configure it
* Generate platform specific release artifacts
* Push them to your release repository
* Fork `rosdistro <https://github.com/ros/rosdistro>`_ to your github account and open a Pull
  Request back upstream with your package to added to ``{DISTRO}/distribution.yaml``.

When you run the above command, it will go out to the ROS distro file for the ROS distro which
you specified and look for your repository's information. Since this is your first release, it
will not find your repository's information, so it will ask you for the release repository url,
like this:

bloom is designed to allow the release of the same package for different ROS distributions and
versions in the same release repository. To facilitate this, bloom uses release "tracks" to
maintain configurations for different release processes. For normal ament-based ROS packages
the default release track is recommended.

In the ``bloom-release`` command you ran above, you specified the ``--track``.
By convention you should create tracks with the same name as the ROS distro you are releasing for,
but you could name your track what ever you wanted.

Release Repository url
~~~~~~~~~~~~~~~~~~~~~~

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
Since our example has a single package called ``foo`` in the repository, it would be appropriate to
put ``foo`` here.

Upstream Repository URI
~~~~~~~~~~~~~~~~~~~~~~~

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

In this example our upstream repository is ``git``, but ``svn``, ``hg``, and hosted ``tar`` archives
are also supported.

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
If this is left ``None`` then the default branch for your repository is used when guessing the
version being released. If you want to use a branch besides the default branch, choose that.
For example, if you want to use the branch ``ros2`` for this release track, enter
``ros2``.

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

Opening a Pull Request (Automatic)
----------------------------------

.. warning::

  If the automated pull request was opened successfully, then you **do not need to open one manually**
  as described here. You can simply skip this section.

Normally your ``bloom-release`` call should open a pull request for you, but if there is a
problem or you do not wish for it to open the pull request on your behalf you can manually open a
pull request also.

In the unlikely case that the automated pull request does not open successfully, you will have to
manually open a pull request with modifications to rosdistro.

You can open a pull request by simply visiting `{DISTRO}/distribution.yaml <https://github.com/ros/rosdistro/blob/master/{DISTRO}/distribution.yaml>`_
and clicking the edit button (note: you have to be logged into Github for this to work), make your changes and then
click "Propose Changes" at the bottom right of the page.

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

Note that you should put the **https://** url of the RELEASE repository here, not the url of your
source repository. 

Note that you must:

* put the full version which is the version of your
  package plus the release increment number separated by a hyphen. (eg. ``0.0.1-1``).
  The release increment number is increased each time you release a package of the same version,
  this can occur when adding patches to the release repository or when changing the release settings.
* put your package into the list of packages in ALPHABETICAL order.

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

Once your pull request has been submitted, one of the ROS developers will merge your request
(this usually happens fairly quickly). 24-48 hours after that, your package should be built by the
build farm and released into the building repository. Packages built are periodically synchronized
over to the `shadow-fixed <https://wiki.ros.org/ShadowRepository>`_
and public repositories, so it might take as long as a month before your
package is available on the public ROS debian repositories (i.e. available via apt-get).
To get updates on when the next synchronization (sync) is coming, check the
`ROS discussion forums <https://discourse.ros.org/>`_.
