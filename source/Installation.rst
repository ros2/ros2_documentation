.. _InstallationGuide:
.. _RollingInstall:

Installation
============

Options for installing ROS 2 {DISTRO_TITLE_FULL}:

.. toctree::
   :hidden:
   :glob:

   Installation/Ubuntu-Install-Debians
   Installation/Windows-Install-Binary
   Installation/RHEL-Install-RPMs
   Installation/Alternatives
   Installation/Maintaining-a-Source-Checkout
   Installation/Testing
   Installation/DDS-Implementations

Binary packages
---------------

Binaries are only created for the Tier 1 operating systems listed in `REP-2000 <https://www.ros.org/reps/rep-2000.html#rolling-ridley-june-2020-ongoing>`__.
Given the nature of Rolling, this list may be updated at any time.
If you are not running any of the following operating systems you may need to build from source or use a :doc:`container solution <How-To-Guides/Run-2-nodes-in-single-or-separate-docker-containers>` to run ROS 2 on your platform.

We provide ROS 2 binary packages for the following platforms:

* Ubuntu Linux - Jammy Jellyfish (22.04)

  * :doc:`Debian packages <Installation/Ubuntu-Install-Debians>` (recommended)
  * :doc:`"fat" archive <Installation/Alternatives/Ubuntu-Install-Binary>`

* RHEL 8

  * :doc:`RPM packages <Installation/RHEL-Install-RPMs>` (recommended)
  * :doc:`"fat" archive <Installation/Alternatives/RHEL-Install-Binary>`

* :doc:`Windows (VS 2019) <Installation/Windows-Install-Binary>`


.. _building-from-source:

Building from source
--------------------

We support building ROS 2 from source on the following platforms:


* :doc:`Ubuntu Linux <Installation/Alternatives/Ubuntu-Development-Setup>`
* :doc:`Windows <Installation/Alternatives/Windows-Development-Setup>`
* :doc:`RHEL <Installation/Alternatives/RHEL-Development-Setup>`
* :doc:`macOS <Installation/Alternatives/macOS-Development-Setup>`


Which install should you choose?
--------------------------------

Installing from binary packages or from source will both result in a fully-functional and usable ROS 2 install.
Differences between the options depend on what you plan to do with ROS 2.

**Binary packages** are for general use and provide an already-built install of ROS 2.
This is great for people who want to dive in and start using ROS 2 as-is, right away.

Linux users have two options for installing binary packages:

- Debian packages
- "fat" archive

Installing from Debian packages is the recommended method.
It's more convenient because it installs its necessary dependencies automatically.
It also updates alongside regular system updates.

However, you need root access in order to install Debian packages.
If you don't have root access, the "fat" archive is the next best choice.

macOS and Windows users who choose to install from binary packages only have the "fat" archive option
(Debian packages are exclusive to Ubuntu/Debian).

**Building from source** is meant for developers looking to alter or explicitly omit parts of ROS 2's base.
It is also recommended for platforms that don't support binaries.
Building from source also gives you the option to install the absolute latest version of ROS 2.

Contributing to ROS 2 core?
^^^^^^^^^^^^^^^^^^^^^^^^^^^
If you plan to contribute directly to ROS 2 core packages, you can install the :doc:`latest development from source <Installation/Alternatives/Latest-Development-Setup>` which shares installation instructions with the :ref:`Rolling distribution <rolling_distribution>`.
