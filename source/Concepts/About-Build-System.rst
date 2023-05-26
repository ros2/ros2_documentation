About the build system
======================

.. include:: ../../global_substitutions.txt

Under everything is the build system.
Iterating on ``catkin`` from ROS 1, we have created a set of |packages| under the moniker ``ament``.
Some of the reasons for changing the name to ``ament`` are that we wanted it to not collide with ``catkin`` (in case we want to mix them at some point) and to prevent confusion with existing ``catkin`` documentation.
``ament``'s primary responsibility is to make it easier to develop and maintain ROS 2 core |packages|.
However, this responsibility extends to any user who is willing to make use of our build system conventions and tools.
Additionally it should make |packages| conventional, such that developers should be able to pick up any ``ament`` based |package| and make some assumptions about how it works, how to introspect it, and how to build or use it.

``ament`` consists of a few important repositories which are all in the ``ament`` `GitHub organization <https://github.com/ament>`_:

.. contents::
   :depth: 1
   :local:

The ``ament_package`` Package
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Located on |GitHub|_ at `ament/ament_package <https://github.com/ament/ament_package>`_, this repository contains a single :term:`ament Python package` that provides various utilities for |ament packages|, e.g. templates for environment hooks.

All |ament packages| must contain a single :term:`package.xml` file at the root of the package regardless of their underlying build system.
The :term:`package.xml` "manifest" file contains information that is required in order to process and operate on a |package|.
This |package| information includes things like the |package|'s name, which is globally unique, and the package's dependencies.
The :term:`package.xml` file also serves as the marker file which indicates the location of the |package| on the file system.

Parsing of the :term:`package.xml` files is provided by ``catkin_pkg`` (as in ROS 1), while functionality to locate |packages| by searching the file system for these :term:`package.xml` files is provided by build tools such as ``colcon``.

.. glossary::

   package.xml
       Package manifest file which marks the root of a :term:`package` and contains meta information about the :term:`package` including its name, version, description, maintainer, license, dependencies, and more.
       The contents of the manifest are in machine readable XML format and the contents are described in the |REPs| `127 <http://www.ros.org/reps/rep-0127.html>`_ and `140 <http://www.ros.org/reps/rep-0140.html>`_, with the possibility of further modifications in future |REPs|.

So anytime some |package| is referred to as an :term:`ament package`, it means that it is a single unit of software (source code, build files, tests, documentation, and other resources) which is described using a :term:`package.xml` manifest file.

.. glossary::

   ament package
       Any |package| which contains a :term:`package.xml` and follows the packaging guidelines of ``ament``, regardless of the underlying build system.

Since the term :term:`ament package` is build system agnostic, there can be different kinds of |ament packages|, e.g. :term:`ament CMake package`, :term:`ament Python package`, etc.

Here is a list of common package types that you might run into in this software stack:

.. glossary::

    CMake package
        Any |package| containing a plain CMake project and a :term:`package.xml` manifest file.

    ament CMake package
        A :term:`CMake package` that also follows the ``ament`` packaging guidelines.

    Python package
        Any |package| containing a `setuptools <https://pypi.org/project/setuptools/>`_ based Python project and a :term:`package.xml` manifest file.

    ament Python package
        A :term:`Python package` that also follows the ``ament`` packaging guidelines.

The ``ament_cmake`` Repository
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Located on |GitHub|_ at `ament/ament_cmake <https://github.com/ament/ament_cmake>`_, this repository contains many "ament CMake" and pure CMake packages which provide the infrastructure in CMake that is required to create "ament CMake" packages.
In this context "ament CMake" packages means: ``ament`` packages that are built using CMake.
So the |packages| in this repository provide the necessary CMake functions/macros and CMake Modules to facilitate creating more "ament CMake" (or ``ament_cmake``) packages.
Packages of this type are identified with the ``<build_type>ament_cmake</build_type>`` tag in the ``<export>`` tag of the :term:`package.xml` file.

The |packages| in this repository are extremely modular, but there is a single "bottleneck" |package| called ``ament_cmake``.
Anyone can depend on the ``ament_cmake`` |package| to get all of the aggregate functionality of the |packages| in this repository.
Here a list of the |packages| in the repository along with a short description:

-  ``ament_cmake``

   - aggregates all other |packages| in this repository, users need only to depend on this

-  ``ament_cmake_auto``

   - provides convenience CMake functions which automatically handle a lot of the tedious parts of writing a |package|'s ``CMakeLists.txt`` file

-  ``ament_cmake_core``

   - provides all built-in core concepts for ``ament``, e.g. environment hooks, resource indexing, symbolic linking install and others

-  ``ament_cmake_gmock``

   - adds convenience functions for making gmock based unit tests

-  ``ament_cmake_gtest``

   - adds convenience functions for making gtest based automated tests

-  ``ament_cmake_nose``

   - adds convenience functions for making nosetests based python automated tests

-  ``ament_cmake_python``

   - provides CMake functions for |packages| that contain Python code
   - see the :doc:`ament_cmake_python user documentation <../How-To-Guides/Ament-CMake-Python-Documentation>`

-  ``ament_cmake_test``

   - aggregates different kinds of tests, e.g. gtest and nosetests, under a single target using `CTest <https://cmake.org/Wiki/CMake/Testing_With_CTest>`_

The ``ament_cmake_core`` |package| contains a lot of the CMake infrastructure that makes it possible to cleanly pass information between |packages| using conventional interfaces.
This makes the |packages| have more decoupled build interfaces with other |packages|, promoting their reuse and encouraging conventions in the build systems of different |packages|.
For instance, it provides a standard way to pass include directories, libraries, definitions, and dependencies between |packages| so that consumers of this information can access this information in a conventional way.

The ``ament_cmake_core`` |package| also provides features of the ``ament`` build system like symbolic link installation, which allows you to symbolically link files from either the source space or the build space into the install space rather than copying them.
This allows you to install once and then edit non-generated resources like Python code and configuration files without having to rerun the install step for them to take effect.
This feature essentially replaces the "devel space" from ``catkin`` because it has most of the advantages with few of the complications or drawbacks.

Another feature provided by ``ament_cmake_core`` is the |package| resource indexing which is a way for |packages| to indicate that they contain a resource of some type.
The design of this feature makes it much more efficient to answer simple questions like what |packages| are in this prefix (e.g. ``/usr/local``) because it only requires that you list the files in a single possible location under that prefix.
You can read more about this feature in the `design docs <https://github.com/ament/ament_cmake/blob/{REPOS_FILE_BRANCH}/ament_cmake_core/doc/resource_index.md>`_ for the resource index.

Like ``catkin``, ``ament_cmake_core`` also provides environment setup files and |package| specific environment hooks.
The environment setup files, often named something like ``setup.bash``, are a place for |package| developers to define changes to the environment that are needed to utilize their |package|.
The developers are able to do this using an "environment hook" which is basically an arbitrary bit of shell code that can set or modify environment variables, define shell functions, setup auto-completion rules, etc...
This feature is how, for example, ROS 1 set the ``ROS_DISTRO`` environment variable without ``catkin`` knowing anything about the ROS distribution.

The ``ament_lint`` Repository
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Located on |GitHub|_ at `ament/ament_lint <https://github.com/ament/ament_lint>`_, this repository provides several |packages| which provide linting and testing services in a convenient and consistent manner.
Currently there are |packages| to support C++ style linting using ``uncrustify``, static C++ code checks using ``cppcheck``, checking for copyright in source code, Python style linting using ``pep8``, and other things.
The list of helper packages will likely grow in the future.

Build tools
~~~~~~~~~~~

A build tool performs the task of building a workspace of packages together at once with a single invocation.
For ROS 2 releases up to Ardent, the build tool providing this functionality is called ``ament_tools``.
As of ROS 2 Bouncy, ``ament_tools`` has been superseded by ``colcon``, as described in `the universal build tool article <http://design.ros2.org/articles/build_tool.html>`_.
