.. _SnapshotRepository:

Installing from the snapshot repository
=======================================

The main ROS apt repository only holds the most recent version of each package, so a sync can move your system onto newer packages than the ones you last tested against.
This article describes how to install ROS from a snapshot of the ROS apt repository, which lets you control exactly when your system moves to a newer sync.

.. note::

   Snapshots are only available for released (non-Rolling) ROS 2 distributions on Ubuntu.

.. contents:: Contents
   :depth: 2
   :local:

Summary
-------

Packages are synced from the ``ros-testing`` :doc:`apt repository </Developer-Tools/Debugging/Testing/Testing>` into the main ROS apt repository periodically for each active distribution.
When a sync happens, the older versions of the updated packages are removed from the main repository and can no longer be installed.
Also, while maintainers and ROS Bosses strive to avoid regressions, packages failing to build are removed when syncing.
This is disruptive if your build or deployment depends on a specific set of package versions.

To improve overall reproducibility and to provide more control over package updates, the snapshot repository at `snapshots.ros.org <http://snapshots.ros.org/>`__ keeps a copy of the main ROS apt repository as it was just after a sync.
Installing from a snapshot gives you a fixed set of package versions that does not change until you choose a different snapshot.

Most users should install from the main repository by following the :doc:`installation instructions <Ubuntu-Install-Debs>`.
The snapshot repository is useful if you bundle ROS packages into an artifact such as a container image, or if you want to move to a new sync deliberately rather than whenever you next run ``apt upgrade``.

Snapshots are taken for a ROS distribution's Ubuntu target platform and architectures.
See :ref:`the list of platforms binaries are built for <binary-package-platforms>`.

.. warning::

   Snapshots do not receive bug fixes or security updates.
   Keep following ROS releases and upgrade periodically.

Usage
-----

1 Add the snapshot repository key
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The snapshot repository is signed with its own key, which is *not* the key that the ``ros2-apt-source`` package installs for the main repository.
Download it into a keyring of its own:

.. code-block:: console

   $ sudo mkdir -p /etc/apt/keyrings
   $ curl -fsSL "https://keyserver.ubuntu.com/pks/lookup?op=get&search=0x4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA" | sudo gpg --dearmor -o /etc/apt/keyrings/ros-snapshots-archive-keyring.gpg

Check that you downloaded the expected key before using it:

.. code-block:: console

   $ gpg --show-keys --with-fingerprint /etc/apt/keyrings/ros-snapshots-archive-keyring.gpg

Confirm that the fingerprint it prints is ``4B63 CF8F DE49 746E 98FA  01DD AD19 BAB3 CBF1 25EA``.

2 Choose a snapshot
^^^^^^^^^^^^^^^^^^^

Browse `snapshots.ros.org <http://snapshots.ros.org/>`__, and choose the distribution and then snapshot you want to install.

Snapshots are named after the date of the sync they were taken from, such as ``2026-08-07``.
When a distribution reaches end of life, a last snapshot named ``final`` is taken, which you can use like any other snapshot.

Record your choice in the shell you will use for the remaining steps, for example, ``humble`` and ``2026-08-07``:

.. code-block:: console

   $ export ROS_DISTRO=humble
   $ export SNAPSHOT=2026-08-07

3 Add the snapshot repository to apt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Write an apt source file pointing at the snapshot you chose:

.. code-block:: bash

   sudo tee /etc/apt/sources.list.d/ros2-snapshots.sources > /dev/null <<EOF
   Types: deb
   URIs: http://snapshots.ros.org/${ROS_DISTRO}/${SNAPSHOT}/ubuntu
   Suites: $(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})
   Components: main
   Signed-By: /etc/apt/keyrings/ros-snapshots-archive-keyring.gpg
   EOF

If you have already installed ROS, remove the ``ros2-apt-source`` package:

.. code-block:: console

   $ sudo apt remove ros2-apt-source

In case you are switching to an older snapshot, give the snapshot repository a pin priority above 1000 so that apt prefers it, otherwise apt will not downgrade to the snapshot's older versions:

.. code-block:: bash

   sudo tee /etc/apt/preferences.d/ros2-snapshots > /dev/null <<EOF
   Package: *
   Pin: origin snapshots.ros.org
   Pin-Priority: 1001
   EOF

Then update the apt caches:

.. code-block:: console

   $ sudo apt update

4 Use a matching rosdistro index
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The rosdistro index is versioned separately from the apt repositories, so ``rosdep`` may resolve keys against a newer state of `rosdistro <https://github.com/ros/rosdistro>`__ than the snapshot you installed.
Syncs are tagged in ``rosdistro`` using the same ``<distribution>/<date>`` naming as snapshots, so you can :doc:`pin the index </Migration-and-Upgrades/Using-Custom-Rosdistro>` to the tag matching your snapshot.

.. code-block:: console

   $ sed -i "s|ros\/rosdistro\/master|ros\/rosdistro\/${ROS_DISTRO}\/${SNAPSHOT}|" /etc/ros/rosdep/sources.list.d/20-default.list
   $ export ROSDISTRO_INDEX_URL=https://raw.githubusercontent.com/ros/rosdistro/${ROS_DISTRO}/${SNAPSHOT}/index-v4.yaml
   $ rosdep update

5 Install ROS
^^^^^^^^^^^^^

Install packages as you normally would.
The versions you get are the ones that were in the main repository at the time of the snapshot:

.. code-block:: console

   $ sudo apt install ros-{DISTRO}-desktop

If you already had ROS installed from the main repository, move the existing installation onto the snapshot:

.. code-block:: console

   $ sudo apt dist-upgrade

Note that this may downgrade packages, since the snapshot can be older than the main repository.

Return to the main repository
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To go back to the main repository, remove the snapshot source and the pin:

.. code-block:: console

   $ sudo rm /etc/apt/sources.list.d/ros2-snapshots.sources
   $ sudo rm -f /etc/apt/preferences.d/ros2-snapshots

.. include:: _Apt-Repositories.rst

Then update the apt index and upgrade:

.. code-block:: console

   $ sudo apt update
   $ sudo apt dist-upgrade

Reset the rosdistro index:

.. code-block:: console

   $ unset ROSDISTRO_INDEX_URL
   $ rm /etc/ros/rosdep/sources.list.d/20-default.list
   $ sudo rosdep init
   $ rosdep update

To move to a different snapshot instead, start from step 2 again.

FAQs
----

How often are snapshots published?
   A snapshot is usually taken after a sync, but the interval between snapshots varies.
   Browse `snapshots.ros.org <http://snapshots.ros.org/>`__ to see what is available for a distribution.

How long are snapshots kept?
   Snapshots for released distributions are available for at least six months from their sync or end of life date.

Is there a snapshot repository for Rolling?
   No.
   Snapshots are only published for released distributions.

How is this different from the ``ros-testing`` repository?
   The two point in opposite directions.
   The ``ros-testing`` repository holds packages that have *not yet* been synced into the main repository, so you can test them before a release.
   A snapshot holds packages exactly as they were at a *past* sync, so you can keep installing the versions you already tested.
   See :doc:`Testing with pre-release binaries </Developer-Tools/Debugging/Testing/Testing>`.

Can I use a snapshot on a production robot?
   Snapshots receive no bug fixes and no security updates.
   If you use one, treat moving to a newer snapshot or distro as part of your regular maintenance.
   Note that ABI compatibility between snapshots is not tested or guaranteed, so when switching a system to a new snapshot, updating all ROS packages is recommended.

How can I get a snapshot of Debian packages from Ubuntu, e.g., ROS dependencies?
   `Ubuntu has its own snapshot service <https://snapshot.ubuntu.com/>`__.
