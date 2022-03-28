Releasing a Third Party Package
===============================

.. warning::

   Instructions here have not been ported yet. Please see
   `Release Third Party on the ROS wiki <http://wiki.ros.org/bloom/Tutorials/ReleaseThirdParty>`_.

A third party package is a software package which exists outside of the ROS
ecosystem, is used by packages in the ROS ecosystem, but is not released widely as a system
dependency.
These software packages might be designed to be used outside of the ROS ecosystem,
but are not big enough or mature enough to be released into operating system package managers.
Instead they are released using the ROS infrastructure along side a ROS distribution as if
they were actually ROS packages.

In this context a third party package is a non-ament package, which you want to release into the
ROS ecosystem. The requirements for a package to be released into the ROS ecosystem is detailed in
`REP-0136: Releasing Third Party, Non catkin Packages <http://ros.org/reps/rep-0136.html>`_.
Basically the package must do these things:

* Have a package.xml

   * Which run_depend's on catkin
   * And which has a <build_type> tag in the <export> tag
* Install a package.xml

The details relating to these requirements are in the REP.

A third party package can accomplish these requirements in two ways: in the upstream repository,
or in the release repository.