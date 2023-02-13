.. redirect-from::

  Guides/Building-a-Custom-Debian-Package

Building a custom Debian package
================================

Many Ubuntu users install ROS 2 on their system by installing :doc:`debian packages <../Installation/Ubuntu-Install-Debians>`.
This guide gives a short set of instructions to build local, custom Debian packages.

.. contents:: Table of Contents
   :local:

Prerequisites
-------------

To successfully build a custom package, all of the dependencies of the package to be built must be available locally or in rosdep.
Additionally, all of the dependencies of the package should be properly declared in the ``package.xml`` file of the package.

Install dependencies
--------------------

Run the following command to install utilities needed for the build:

.. code:: bash

  $ sudo apt install python3-bloom python3-rosdep fakeroot debhelper dh-python

Initialize rosdep
-----------------

Initialize the rosdep database by calling:

.. code:: bash

  $ sudo rosdep init
  $ rosdep update

Note that the ``rosdep init`` command may fail if it has already been initialized in the past; this can safely be ignored.

Build the debian from the package
---------------------------------

Run the following commands to build the debian:

.. code:: bash

  $ cd /path/to/pkg_source  # this should be the directory that contains the package.xml
  $ bloom-generate rosdebian
  $ fakeroot debian/rules binary

Assuming that all required dependencies are available and that compilation succeeds, the new package will be available in the parent directory of this directory.
