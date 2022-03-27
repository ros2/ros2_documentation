Releasing for a New Distro
==========================

The context of this tutorial is that you have previously released a package
into a different ROS distro, but now want to release it for {DISTRO}.

Follow instructions on :doc:`Releasing a Package Update <Releasing-a-Package-Update>`, **up to
Release your Packages**.


Release your Packages
---------------------

Perform the release using the command below:

.. code-block:: bash

   bloom-release --rosdistro {DISTRO} --track {DISTRO} --new-track <your_repository_name>


* ``--rosdistro {DISTRO}`` indicates that this release is for the ``{DISTRO}`` distro

* ``--track {DISTRO}`` indicates that you want the track name to be ``{DISTRO}``

* ``--new-track`` tells bloom that it should create the specified track and edit it before trying
  to do the release.

.. note::

   ``<your_repository_name>`` is not its url, it is its reference in ``{DISTRO}/distribution.yaml``.

The script will prompt you through to setup a new track and create a release.

Configuring your new Track
--------------------------

The option you need to potentially modify is the ``Upstream Development Branch``:

.. code-block:: bash

   Upstream Devel Branch:
      <vcs reference>
         Branch in upstream repository on which to search for the version.
         This is used only when version is set to ':{auto}'.
      [None]:

The upstream development branch is the branch on which bloom searches for the appropriate version
to release. Your development branch may likely be different from the one used for a previous release
in a different ROS distro.
