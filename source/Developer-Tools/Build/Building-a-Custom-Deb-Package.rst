.. redirect-from::

  Guides/Building-a-Custom-Debian-Package
  How-To-Guides/Building-a-Custom-Debian-Package
  How-To-Guides/Building-a-Custom-Deb-Package

.. meta::
   :contentType: how-to
   :experience:
   :area: builds, tools
   :distribution: {DISTRO}
   :product: {PRODUCT}

Building a custom deb package - how-to
======================================

.. short-description::
   Many Ubuntu users install ROS by using pre-built Debian packages.
   This article gives a short workflow for building a local custom deb package from a ROS package source directory.
   After you follow it, you can generate Debian packaging files and build a package for local installation.

.. showmeta::
   :order: area, contentType, experience
   :labels: area=Area, contentType=Content type, experience=Level

.. contents:: Table of Contents
   :depth: 2
   :local:

Summary
-------

Many Ubuntu users install ROS 2 on their system by installing :doc:`deb packages <../../Get-Started/Installation/Ubuntu-Install-Debs>`.
This guide gives a short set of instructions to build local, custom deb packages.

Prerequisites
-------------

To successfully build a custom package, all of the dependencies of the package to be built must be available locally or in rosdep.
Additionally, all of the dependencies of the package should be properly declared in the ``package.xml`` file of the package.

Install dependencies
--------------------

Run the following command to install utilities needed for the build:

.. code:: console

  $ sudo apt install python3-bloom python3-rosdep fakeroot debhelper dh-python

Initialize rosdep
-----------------

Initialize the rosdep database by calling:

.. code:: console

  $ sudo rosdep init
  $ rosdep update

Note that the ``rosdep init`` command may fail if it has already been initialized in the past; this can safely be ignored.

Build the deb from the package
------------------------------

Run the following commands to build the deb:

.. code:: console

  $ cd /path/to/pkg_source  # this should be the directory that contains the package.xml
  $ bloom-generate rosdebian
  $ fakeroot debian/rules binary

Assuming that all required dependencies are available and that compilation succeeds, the new package will be available in the parent directory of this directory.
