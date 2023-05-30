.. redirect-from::

    Tutorials/Rosdep

.. _rosdep:

Managing Dependencies with rosdep
=================================

**Goal:** Manage external dependencies using ``rosdep``.

**Tutorial level:** Intermediate

**Time:** 5 minutes

.. contents:: Contents
   :depth: 2
   :local:

Author: Steve Macenski

This tutorial will explain how to manage external dependencies using ``rosdep``.

What is rosdep?
---------------

``rosdep`` is ROS's dependency management utility that can work with ROS packages and external libraries.
``rosdep`` is a command-line utility for identifying and installing dependencies to build or install a package.
It can be or is invoked when:

- Building a workspace and needing appropriate dependencies to build the packages within
- Install packages (e.g. ``sudo apt install ros-{DISTRO}-demo-nodes-cpp``) to check the dependencies needed for it to execute
- and more!

It has the ability to work over a single package or over a directory of packages (e.g. workspace).

A little about package.xml files
--------------------------------

A package's ``package.xml`` file contains a set of dependencies.
The dependencies in this file are generally referred to as "rosdep keys".
These are represented in the tags ``<depend>``, ``<test_depend>``, ``<exec_depend>``, ``<build_depend>``, and ``<build_export_depend>``.
They specify in what situation each of the dependencies are required in.

- For dependencies only used in testing the code (e.g. ``gtest``), use ``test_depend``.
- For dependencies only used in building the code, use ``build_depend``.
- For dependencies needed by headers the code exports, use ``build_export_depend``.
- For dependencies only used when running the code, use ``exec_depend``.
- For mixed purposes, use ``depend``, which covers build, export, and execution time dependencies.

These dependencies are manually populated in the ``package.xml`` file by the package's creators and should be an exhaustive list of any non-builtin libraries and packages it requires.

How does rosdep work?
---------------------

``rosdep`` will check for ``package.xml`` files in its path or for a specific package and find the rosdep keys stored within.
These keys are then cross-referenced against a central index to find the appropriate ROS package or software library in various package managers.
Finally, once the packages are found, they are installed and ready to go!

The central index is known as ``rosdistro``, which `may be found here <https://github.com/ros/rosdistro>`_.
We'll explore that more in the next section.

How do I know what keys to put in my package.xml?
-------------------------------------------------

Great question, I'm glad you asked!

For ROS packages (e.g. ``nav2_bt_navigator``), you may simply place the name of the package.
You can find a list of all released ROS packages in ``rosdistro`` at ``<distro>/distribution.yaml`` for your given ROS distribution.

For non-ROS package system dependencies, we will need to find the keys for a particular library.
In general, there are two files of interest: ``rosdep/base.yaml`` and ``rosdep/python.yaml``.
``base.yaml`` in general contains the ``apt`` system dependencies.
``python.yaml`` in general contains the ``pip`` python dependencies.

To find a key, search for your library in this file and find the name in ``yaml`` that contains it.
This is the key to put in a ``package.xml`` file.

For example, imagine a package had a dependency on ``doxygen`` because it is a great piece of software that cares about quality documentation (hint hint).
We would search ``base.yaml`` for ``doxygen`` and come across:

.. code-block:: yaml

  doxygen:
    arch: [doxygen]
    debian: [doxygen]
    fedora: [doxygen]
    freebsd: [doxygen]
    gentoo: [app-doc/doxygen]
    macports: [doxygen]
    nixos: [doxygen]
    openembedded: [doxygen@meta-oe]
    opensuse: [doxygen]
    rhel: [doxygen]
    ubuntu: [doxygen]

That means our rosdep key is ``doxygen``, which would resolve to those various names in different operating system's package managers for installation.

What if my library isn't in rosdistro?
--------------------------------------

If your library isn't in ``rosdistro``, you can experience the greatness that is open-source software development: you can add it yourself!
Pull requests for rosdistro are typically merged well within a week.

`Detailed instructions may be found here <https://github.com/ros/rosdistro/blob/master/CONTRIBUTING.md#rosdep-rules-contributions>`_ for how to contribute new rosdep keys.
If for some reason these may not be contributed openly, it is possible to fork rosdistro and maintain a alternate index for use.


How do I use the rosdep tool?
-----------------------------

Now that we have some understanding of ``rosdep``, ``package.xml``, and ``rosdistro``, we're ready to use the utility itself!
Firstly, if this is the first time using ``rosdep``, it must be initialized via:

.. code-block:: bash

    sudo rosdep init
    rosdep update

This will initialize rosdep and ``update`` will update the locally cached rosdistro index.
It is a good idea to ``update`` rosdep on occasion to get the latest index.

Finally, we can run ``rosdep install`` to install dependencies.
Typically, this is run over a workspace with many packages in a single call to install all dependencies.
A call for that would appear as the following, if in the root of the workspace with directory ``src`` containing source code.

.. code-block:: bash

    rosdep install --from-paths src -y --ignore-src

Breaking that down:

- ``--from-paths src`` specifies the path to check for ``package.xml`` files to resolve keys for
- ``-y`` means to default yes to all prompts from the package manager to install without prompts
- ``--ignore-src`` means to ignore installing dependencies, even if a rosdep key exists, if the package itself is also in the workspace.

There are additional arguments and options available.
Use ``rosdep -h`` to see them.
