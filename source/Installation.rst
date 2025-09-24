.. _InstallationGuide:
.. _RollingInstall:

Installation
============

Options for installing ROS 2 {DISTRO_TITLE_FULL}:

.. toctree::
   :hidden:
   :glob:

   Installation/Ubuntu-Install-Debs
   Installation/Windows-Install-Binary
   Installation/RHEL-Install-RPMs
   Installation/Alternatives
   Installation/Maintaining-a-Source-Checkout
   Installation/Testing
   Installation/RMW-Implementations

.. _binary-package-platforms:

Binary packages
---------------

Binaries are only created for the Tier 1 operating systems listed in `REP-2000 <https://www.ros.org/reps/rep-2000.html#rolling-ridley-june-2020-ongoing>`__.
Given the nature of Rolling, this list may be updated at any time.
If you are not running any of the following operating systems you may need to build from source or use a :doc:`container solution <How-To-Guides/Run-2-nodes-in-single-or-separate-docker-containers>` to run ROS 2 on your platform.

We provide ROS 2 binary packages for the following platforms:

* Ubuntu Linux (amd64 / aarch64) - Noble Numbat (24.04)

  * :doc:`deb packages <Installation/Ubuntu-Install-Debs>` (recommended)
  * :doc:`binary archive <Installation/Alternatives/Ubuntu-Install-Binary>`

* Red Hat Enterprise Linux 9 (amd64)

  * :doc:`RPM packages <Installation/RHEL-Install-RPMs>` (recommended)
  * :doc:`binary archive <Installation/Alternatives/RHEL-Install-Binary>`

* Windows 10 (amd64)

  * :doc:`Windows Binary (VS 2019) <Installation/Windows-Install-Binary>`

.. _building-from-source:

Building from source
--------------------

We support building ROS 2 from source on the following platforms:

* :doc:`Ubuntu Linux 24.04 <Installation/Alternatives/Ubuntu-Development-Setup>`
* :doc:`Windows 10 <Installation/Alternatives/Windows-Development-Setup>`
* :doc:`RHEL-9/Fedora <Installation/Alternatives/RHEL-Development-Setup>`
* :doc:`macOS <Installation/Alternatives/macOS-Development-Setup>`

Which install should you choose?
--------------------------------

Installing from binary packages or from source will both result in a fully-functional and usable ROS 2 install.
Differences between the options depend on what you plan to do with ROS 2.

**Binary packages** are for general use and provide an already-built install of ROS 2.
This is great for people who want to dive in and start using ROS 2 as-is, right away.

Linux users have two options for installing binary packages:

- Packages (debs or RPMS, depending on the platform)
- binary archive

Installing from packages is the recommended method, as it installs necessary dependencies automatically and also updates alongside regular system updates.
However, you need root access in order to install deb packages.
If you don't have root access, the binary archive is the next best choice.

Windows users who choose to install from binary packages only have the binary archive option
(deb packages are exclusive to Ubuntu/Debian).

**Building from source** is meant for developers looking to alter or explicitly omit parts of ROS 2's base.
It is also recommended for platforms that don't support binaries.
Building from source also gives you the option to install the absolute latest version of ROS 2.

Contributing to ROS 2 core?
^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you plan to contribute directly to ROS 2 core packages, you can install the :doc:`latest development from source <Installation/Alternatives/Latest-Development-Setup>` which shares installation instructions with the :ref:`Rolling distribution <rolling_distribution>`.
