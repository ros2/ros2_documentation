Porting packages from ROS 1
===========================

Migrating from ROS 1 to ROS 2 gives you access to `ROS 2's architectual improvements <https://doi.org/10.1126/scirobotics.abm6074>`__, but it can be a substantial effort.
This guide is meant to walk through the process of porting your ROS 1 packages to ROS 2.

.. contents:: Table of Contents
   :depth: 2
   :local:

Identify what ROS 2 distributions you're targeting
--------------------------------------------------

Pick the ROS 2 distributions you want to support.
For example, maybe you want to use a particular operating system.

No matter what stable distributions you pick (if any), it's always recommended to support ROS Rolling.
This makes sure you're packages are up to date with the latest improvements to ROS 2.
Releasing into Rolling `can allow it to be automatically included in future ROS 2 stable distributions <../Releasing/Release-Team-Repository.rst>`__.

Identify what you're porting
----------------------------

The first step is to pick which of your packages you're going to port.
You must port your packages from the bottom up, meaning all of a package's dependencies are ported before beginning to port it.
You can use ``colcon graph`` in a ROS 1 workspace to visually determine what packages are at the bottom of the depedency graph.
Packages at the bottom of the dependency graph will be at the bottom of the generated image.

.. code-block::

   colcon graph --dot | dot -Tpng > dependency_graph.png


You'll also need to look at the package's ``package.xml`` to determine if all of it's third party dependencies are ported.
Look up each package name on `index.ros.org <https://index.ros.org>`__.
For example, if you depend on a third party package called ``name_of_package``, then you can determine if it's ported to ROS 2 by going to\ the following URL.

.. code-block::

   https://index.ros.org/p/name_of_package

Note that some ROS 1 packages have been replaced instead of ported.
If you don't see a package in ROS 2, then check if it has `a ROS 2 equivalent <./ROS-1-Package-Equivalents>`__.

Port the build system first
---------------------------

Port your package's build system before making any other changes.
It won't build at first, but that's ok.
Porting the build system first allows you to use the compiler to discover what code still needs to be ported.

- `Porting the build system for C++ packages <./Porting-From-Catkin-To-Ament-C++>`__

- `Porting build system for packages that define messages, services, or actions <./Porting-From-Catkin-To-Ament-Interface>`__

- `Porting the build system Python packages <./Porting-From-Catkin-To-Ament-Python>`__


Release your Package
--------------------

Congratulations, you've ported your package to ROS 2!
Now consider releasing it so that others who depend on your package can begin porting theirs to ROS 2 too.