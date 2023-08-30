Migrating Packages
==================

.. contents:: Table of Contents
   :depth: 2
   :local:

There are two different kinds of package migrations:

* Migrating the source code of an existing package from ROS 1 to ROS 2 with the intent that a significant part of the source code will stay the same or at least similar.
  An example for this is `pluginlib <https://github.com/ros/pluginlib>`_ where the source code is maintained in different branches within the same repository and common patches can be ported between those branches when necessary.
* Implementing the same or similar functionality of a ROS 1 package for ROS 2 but with the assumption that the source code will be significantly different.
  An example for this is `roscpp <https://github.com/ros/ros_comm/tree/melodic-devel/clients/roscpp>`_ in ROS 1 and `rclcpp <https://github.com/ros2/rclcpp/tree/rolling/rclcpp>`_ in ROS 2 which are separate repositories and don't share any code.

Prerequisites
-------------

Before being able to migrate a ROS 1 package to ROS 2 all of its dependencies must be available in ROS 2.

Package format version
----------------------

ROS 2 doesn't support format 1 of the package specification but only newer format versions (2 and higher).
Therefore the ``package.xml`` file must be updated to at least format 2 if it uses format 1.
Since ROS 1 supports all formats it is safe to perform that conversion in the ROS 1 package.

Migrating from package format 1 to 2+
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The differences between format 1 and format 2 only affect the package.xml with its dependencies.
`REP-0140 <https://www.ros.org/reps/rep-0140.html>`__ defines these differences and provides their rationale.

See :doc:`the rosdep documentation <../../Tutorials/Intermediate/Rosdep>` for more information about the various tags.

**<package>**
~~~~~~~~~~~~~

The <package> tag determines which format to use, change it like this:

.. code:: xml

  <package format="2">

**<depend>**
~~~~~~~~~~~~~

This is a new tag, intended to reduce unnecessary repetition.
If your format 1 package contained:

.. code:: xml

  <build_depend>foo</build_depend>
  <run_depend>foo</run_depend>

It should be replaced with:

.. code:: xml

  <depend>foo</depend>

In format 2, that is equivalent to:

.. code:: xml

  <build_depend>foo</build_depend>
  <build_export_depend>foo</build_export_depend>
  <exec_depend>foo</exec_depend>


**<run_depend>**
~~~~~~~~~~~~~~~~

This tag is no longer allowed.
Wherever found, it must be replaced:

.. code:: xml

  <run_depend>foo</run_depend>

In format 2, that is equivalent to these two new tags:

.. code:: xml

  <build_export_depend>foo</build_export_depend>
  <exec_depend>foo</exec_depend>

If the dependency is only used at run-time, only the ``<exec_depend>`` is needed.
If it is only exported to satisfy the build dependencies of other packages, use ``<build_export_depend>``.
If both are needed, this may be a better choice:

.. code:: xml

  <depend>foo</depend>


**<test_depend>**
~~~~~~~~~~~~~~~~~

In format 2, this tag can satisfy build dependencies, not just those needed for executing your tests.
Unlike format 1, ``<test_depend>`` may now refer to a package also declared as some other type of dependency.

Some test-only dependencies that formerly required a ``<build_depend>``, should now be expressed using ``<test_depend>``.
For example:

.. code:: xml

  <build_depend>testfoo</build_depend>

becomes:

.. code:: xml

  <test_depend>testfoo</test_depend>

In your CMakeLists.txt make sure your test dependencies are only referenced within the conditional test block:

.. code:: cmake

  if (BUILD_TESTING)
    find_package(testfoo REQUIRED)
  endif()


**<doc_depend>**
~~~~~~~~~~~~~~~~

This tag defines dependencies needed for building your documentation:

.. code:: xml

  <doc_depend>doxygen</doc_depend>
  <doc_depend>python3-sphinx</doc_depend>

This does not create binary package dependencies, unless they were also declared using some other dependency tag.

Dependency names
----------------

Dependency names that come from :doc:`rosdep <../../Tutorials/Intermediate/Rosdep>` should not need to change, as those are shared across ROS 1 and ROS 2.

Some packages released into ROS might have different names in ROS 2 so the dependencies might need to be updated accordingly.

Metapackages
------------

ROS 2 doesn't have a special package type for metapackages.
Metapackages can still exist as regular packages that only contain runtime dependencies.
When migrating metapackages from ROS 1, simply remove the ``<metapackage />`` tag in your package manifest.
See :doc:`Using variants <../Using-Variants>` for more information on metapackages/variants.

Licensing
---------

In ROS 1 our recommended license was the `3-Clause BSD License <https://opensource.org/licenses/BSD-3-Clause>`__.
In ROS 2 our recommended license is the `Apache 2.0 License <https://www.apache.org/licenses/LICENSE-2.0>`__.

For any new project we recommend using the Apache 2.0 License, whether ROS 1 or ROS 2.

However, when migrating code from ROS 1 to ROS 2 we cannot simply change the license.
The existing license must be preserved for any preexisting contributions.

To that end if a package is being migrated we recommend keeping the existing license and continuing to contribute to that package under the existing OSI license, which we expect to be the BSD license for core elements.

This will keep things clear and easy to understand.

Changing the License
^^^^^^^^^^^^^^^^^^^^

It is possible to change the license, however you will need to contact all the contributors and get permission.
For most packages this is likely to be a significant effort and not worth considering.
If the package has a small set of contributors then this may be feasible.
