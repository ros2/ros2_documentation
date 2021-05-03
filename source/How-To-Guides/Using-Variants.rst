Using variants
==============

Variants, refered to as "metapackages" in ROS 1 are packages which provide no functionality directly but depend on a specified set of ROS packages in order to provide a convenient installation mechansim for the entire group of packages at once.

The different variants in ROS 2 are specified in [REP-2001](https://ros.org/reps/rep-2001.html).

In addition to the official variants, there may be metapackages for specific institutions or robots as described in [REP-108](https://www.ros.org/reps/rep-0108.html#institution-specific).

Adding variants
---------------

Additional variants that are of general use to the ROS community can be proposed by contributing an update to [REP-2001 via pull request](https://github.com/ros-infrastructure/rep/blob/master/rep-2001.rst) describing the packages included in the new variant.
Institution and robot specific variants can be published directly by their respective maintainers and no update to REP-2001 is required.

Creating project-specific variants
----------------------------------

If you are creating ROS packages to use privately in your own projects, you can create variants specific to your projects using the official variants as examples.
A minimal variant package is created as a package with the ``ament_cmake`` build type, a ``buildtool_depend`` on ``ament_cmake`` and ``exec_depend`` entries for each package you want to include in the variant.


You can then build and install your variant package alongside your other private packages.

Creating custom variants with platform-specific tools
*****************************************************

Some platforms have tools for creating basic packages that do not require a full ROS build farm environment or equivalent infrastructure.
It is possible to use these tools to create platform-dependent variants.
This approach does not include support for ROS packaging tools and is platform dependent but requires much less infrastructure to produce if you are creating collections of existing packages rather than a mix of public and private ROS packages.
For example, on Debian or Ubuntu systems you can use the ``equivs`` utilities.
The Debian Administrator's handbook has a `Section on meta-packages <https://www.debian.org/doc/manuals/debian-handbook/sect.building-first-package.en.html#id-1.18.5.2>`_.


