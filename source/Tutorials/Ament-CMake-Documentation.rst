ament_cmake user documentation
==============================

ament_cmake is the build system for CMake based packages in ROS 2 (in particular, it will be used for most if not all C/C++ projects).
It is a set of scripts enhancing CMake and adding convenience functionality for package authors.
Knowing the basics of `CMake <https://cmake.org/cmake/help/v3.5/>`__ will be very helpful, an official tutorial can be found `here <https://cmake.org/cmake-tutorial/>`__.

.. contents:: Table of Contents
   :depth: 2
   :local:

Basics
------

A basic CMake outline can be produced using ``ros2 pkg create <package_name>`` on the command line.
The basic build information is then gathered in two files: the ``package.xml`` and the ``CMakeLists.txt``.
The ``package.xml`` must contain all dependencies and a bit of metadata to allow colcon to find the correct build order for your packages, to install the required dependencies in CI as well as provide the information for a release with ``bloom``.
The ``CMakeLists.txt`` contains the commands to build and package executables and libraries and will be the main focus of this document.

Basic project outline
^^^^^^^^^^^^^^^^^^^^^

The basic outline of the ``CMakeLists.txt`` of an ament package contains:

.. code-block:: cmake

   cmake_minimum_required(VERSION 3.5)
   project(my_project)

   ament_package()

The argument to ``project`` will be the package name and must be identical to the package name in the ``package.xml``.

The project setup is done by ``ament_package()`` and this call must occur exactly once per package.
``ament_package()`` installs the ``package.xml``, registers the package with the ament index, and installs config (and possibly target) files for CMake so that it can be found by other packages using ``find_package``.
Since ``ament_package()`` gathers a lot of information from the ``CMakeLists.txt`` it should be the last call in your ``CMakeLists.txt``.
Although it is possible to follow calls to ``ament_package()`` by calls to ``install`` functions copying files and directories, it is simpler to just keep ``ament_package()`` the last call.

``ament_package`` can be given additional arguments:

- ``CONFIG_EXTRAS``: a list of CMake files (``.cmake`` or ``.cmake.in`` templates expanded by ``configure_file()``) which should be available to clients of the package.
  For an example of when to use these arguments, see the discussion in `Adding resources`_.
  For more information on how to use template files, see `the official documentation <https://cmake.org/cmake/help/v3.5/command/configure_file.html>`__.

- ``CONFIG_EXTRAS_POST``: same as ``CONFIG_EXTRAS``, but the order in which the files are added differs.
  While ``CONFIG_EXTRAS`` files are included before the files generated for the ``ament_export_*`` calls the files from ``CONFIG_EXTRAS_POST`` are included afterwards.

Instead of adding to ``ament_package``, you can also add to the variable ``${PROJECT_NAME}_CONFIG_EXTRAS`` and ``${PROJECT_NAME}_CONFIG_EXTRAS_POST`` with the same effect.
The only difference is again the order in which the files are added with the following total order:

- files added by ``CONFIG_EXTRAS``

- files added by appending to ``${PROJECT_NAME}_CONFIG_EXTRAS``

- files added by appending to ``${PROJECT_NAME}_CONFIG_EXTRAS_POST``

- files added by ``CONFIG_EXTRAS_POST``

Adding files and headers
^^^^^^^^^^^^^^^^^^^^^^^^

There are two main targets to build: libraries and executables which are built by ``add_library`` and ``add_executable`` respectively.

With the separation of header files and implementation in C/C++, it is not always necessary to add both files as argument to ``add_library``/ ``add_executable``.

The following best practice is proposed:

- if you are building a library, put all headers which should be usable by clients and therefore must be installed into a subdirectory of the ``include`` folder named like the package, while all other files (``.c/.cpp`` and header files which should not be exported) are inside the ``src`` folder.

- only cpp files are explicitly referenced in the call to ``add_library`` or ``add_executable``

- allow to find headers via

.. code-block:: cmake

    target_include_directories(my_target
      PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

This adds all files in the folder ``${CMAKE_CURRENT_SOURCE_DIR}/include`` to the public interface during build time and all files in the include folder (relative to ``${CMAKE_INSTALL_DIR}``) when being installed.

In principle, using generator expressions here is not necessary if both folders are called ``include`` and top-level with respect to ``${CMAKE_CURRENT_SOURCE_DIR}`` and ``${CMAKE_INSTALL_DIR}``, but it is very common.

Adding Dependencies
^^^^^^^^^^^^^^^^^^^

There are two ways to link your packages against a new dependency.

The first and recommended way is to use the ament macro ``ament_target_dependencies``.
As an example, suppose we want to link ``my_target`` against the linear algebra library Eigen3.

.. code-block:: cmake

    find_package(Eigen3 REQUIRED)
    ament_target_dependencies(my_target Eigen3)

It includes the necessary headers and libraries and their dependencies to be correctly found by the project.
It will also ensure that the include directories of all dependencies are ordered correctly when using overlay workspaces.

The second way is to use ``target_link_libraries``.

The recommended way in modern CMake is to only use targets, exporting and linking against them.
CMake targets are namespaced, similar to C++.
For instance, ``Eigen3`` defines the target ``Eigen3::Eigen``.

At least until ``Crystal Clemmys`` target names are not supported in the ``ament_target_dependencies`` macro.
Sometimes it will be necessary to call the ``target_link_libaries`` CMake function.
In the example of Eigen3, the call should then look like

.. code-block:: cmake

    find_package(Eigen3 REQUIRED)
    target_link_libraries(my_target Eigen3::Eigen)

This will also include necessary headers, libraries and their dependencies, but in contrast to ``ament_target_dependencies`` it might not correctly order the dependencies when using overlay workspaces.

.. note::

   It should never be necessary to ``find_package`` a library that is not explicitly needed but is a dependency of another dependency that is explicitly needed.
   If that is the case, file a bug against the corresponding package.

Building a Library
^^^^^^^^^^^^^^^^^^

When building a reusable library, some information needs to be exported for downstream packages to easily use it.

.. code-block:: cmake

    ament_export_targets(export_my_library HAS_LIBRARY_TARGET)
    ament_export_dependencies(some_dependency)

    install(
      DIRECTORY include/
      DESTINATION include
    )

    install(
      TARGETS my_library
      EXPORT export_my_library
      LIBRARY DESTINATION lib
      ARCHIVE DESTINATION lib
      RUNTIME DESTINATION bin
      INCLUDES DESTINATION include
    )


Here, we assume that the folder ``include`` contains the headers which need to be exported.
Note that it is not necessary to put all headers into a separate folder, only those that should be included by clients.

Here is what's happening in the snippet above:

- The ``ament_export_targets`` macro exports the targets for CMake.
  This is necessary to allow your library's clients to use the ``target_link_libraries(client my_library::my_library)`` syntax.
  ``ament_export_targets`` can take an arbitrary list of targets named as ``EXPORT`` in an install call and an additional option ``HAS_LIBRARY_TARGET``, which adds potential libraries to environment variables.

- The ``ament_export_dependencies`` exports dependencies to downstream packages.
  This is necessary so that the user of the library does not have to call ``find_package`` for those dependencies, too.

- The first ``install`` commands installs the header files which should be available to clients.

.. warning::

   Calling ``ament_export_targets``, ``ament_export_dependencies``, or other ament commands from a CMake subdirectory will not work as expected.
   This is because the CMake subdirectory has no way of setting necessary variables in the parent scope where ``ament_package`` is called.

- The last large install command installs the library.
  Archives and library files will be exported to the lib folder, runtime binaries will be installed to the bin folder and the path to installed headers is ``include``.

.. note::

   Windows dlls are treated as runtime artifacts and installed into the ``RUNTIME DESTINATION`` folder.
   It is therefore advised to not leave out the ``RUNTIME`` install even when developing libraries on Unix based systems.

- Regarding the ``include directory``, the install command only adds information to CMake, it does not actually install the includes folder.
  This is done by copying the headers via ``install(DIRECTORY <dir> DESTINATION <dest>)`` as described above.

- The ``EXPORT`` notation of the install call requires additional attention:
  It installs the CMake files for the ``my_library`` target.
  It is named exactly like the argument in ``ament_export_targets`` and could be named like the library.
  However, this will then prohibit using the ``ament_target_dependencies`` way of including your library.
  To allow for full flexibility, it is advised to prepend the export target with something like ``export_<target>``.

- All install paths are relative to ``CMAKE_INSTALL_PREFIX``, which is already set correctly by colcon/ament

There are two additional functions which can be used but are superfluous for target based installs:

.. code-block:: cmake

    ament_export_include_directories(include)
    ament_export_libraries(my_library)

The first macro marks the directory of the exported include directories (this is achieved by ``INCLUDES DESTINATION`` in the target ``install`` call).
The second macro marks the location of the installed library (this is done by the ``HAS_LIBRARY_TARGET`` argument in the call to ``ament_export_targets``).

Some of the macros can take different types of arguments for non-target exports, but since the recommended way for modern Make is to use targets, we will not cover them here.
Documentation of these options can be found in the source code itself.

Compiler and linker options
^^^^^^^^^^^^^^^^^^^^^^^^^^^

ROS 2 targets compilers which comply with the C++14 and C99 standard until at least ``Crystal Clemmys``.
Newer versions might be targeted in the future and are referenced `here <https://www.ros.org/reps/rep-2000.html>`__.
Therefore it is customary to set the corresponding CMake flags:

.. code-block:: cmake

    if(NOT CMAKE_C_STANDARD)
      set(CMAKE_C_STANDARD 99)
    endif()
    if(NOT CMAKE_CXX_STANDARD)
      set(CMAKE_CXX_STANDARD 14)
    endif()

To keep the code clean, compilers should throw warnings for questionable code and these warnings should be fixed.

It is recommended to at least cover the following warning levels:

- For Visual Studio, the default ``W1`` warnings are kept

- For GCC and Clang: ``-Wall -Wextra -Wpedantic`` are required and ``-Wshadow -Werror`` are advisable (the latter makes warnings errors).

Although modern CMake advises to add compiler flags on a target basis, i.e. call

.. code-block:: cmake

    target_compile_options(my_target PRIVATE -Wall)

it is at the moment recommended to use the directory level function ``add_compile_options(-Wall)`` to not clutter the code with target-based compile options for all executables and tests.

Building libraries on Windows
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Since Linux, Mac and Windows are all officially supported platforms, to have maximum impact any package should also build on Windows.
The Windows library format enforces symbol visibility:
Every symbol which should be used from a client has to be explicitly exported by the library (and data symbols need to be implicitly imported).

To keep this compatible with Clang and GCC builds, it is advised to use the logic in `the GCC wiki <https://gcc.gnu.org/wiki/Visibility>`__.
To use it for a package called ``my_library``:

- Copy the logic in the link into a header file called ``visibility_control.hpp``.

- Replace ``DLL`` by ``MY_LIBRARY`` (for an example, see visibility control of `rviz_rendering <https://github.com/ros2/rviz/blob/ros2/rviz_rendering/include/rviz_rendering/visibility_control.hpp>`__).

- Use the macros "MY_LIBRARY_PUBLIC" for all symbols you need to export (i.e. classes or functions).

- In the project ``CMakeLists.txt`` use:

.. code-block:: cmake

    target_compile_definitions(my_library PRIVATE "MY_LIBRARY_BUILDING_LIBRARY")

Testing and Linting
-------------------

In order to separate testing from building the library with colcon, wrap all calls to linters and tests in a conditional:

.. code-block:: cmake

    if(BUILD_TESTING)
      find_package(ament_gtest)
      ament_add_gtest(<tests>)
    endif()

Linting
^^^^^^^

It's advised to use the combined call from `ament_lint_auto <https://github.com/ament/ament_lint/blob/master/ament_lint_auto/doc/index.rst#ament_lint_auto>`_:

.. code-block:: cmake

    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

This will run linters as defined in the ``package.xml``.
It is recommended to use the set of linters defined by the package ``ament_lint_common``.
The individual linters included there, as well as their functions, can be seen in the `ament_lint_common docs <https://github.com/ament/ament_lint/blob/master/ament_lint_common/doc/index.rst>`_.

Linters provided by ament can also be added separately, instead of running ``ament_lint_auto``.
One example of how to do so can be found in the `ament_cmake_lint_cmake documentation <https://github.com/ament/ament_lint/blob/master/ament_cmake_lint_cmake/doc/index.rst>`_.

Testing
^^^^^^^

Ament contains CMake macros to simplify setting up GTests. Call:

.. code-block:: cmake

    find_package(ament_gtest)
    ament_add_gtest(some_test <test_sources>)

to add a GTest.
This is then a regular target which can be linked against other libraries (such as the project library).
The macros have additional parameters:

- ``APPEND_ENV``: append environment variables.
  For instance you can add to the ament prefix path by calling:

.. code-block:: cmake

    find_package(ament_gtest REQUIRED)
    ament_add_gtest(some_test <test_sources>
      APPEND_ENV PATH=some/addtional/path/for/testing/resources)

- ``APPEND_LIBRARY_DIRS``: append libraries so that they can be found by the linker at runtime.
  This can be achieved by setting environment variables like ``PATH`` on Windows and ``LD_LIBRARY_PATH`` on Linux, but this makes the call platform specific.

- ``ENV``: set environment variables (same syntax as ``APPEND_ENV``).

- ``TIMEOUT``: set a test timeout in second. The default for GTests is 60 seconds.  For example:

.. code-block:: cmake

    ament_add_gtest(some_test <test_sources> TIMEOUT 120)

- ``SKIP_TEST``: skip this test (will be shown as "passed" in the console output).

- ``SKIP_LINKING_MAIN_LIBRARIES``: Don't link against GTest.

- ``WORKING_DIRECTORY``: set the working directory for the test.

The default working directory otherwise is the ``CMAKE_SOURCE_DIR``, which will be evaluated to the directory of the top-level ``CMakeLists.txt``.

Similarly, there is a CMake macro to set up GTest including GMock:

.. code-block:: cmake

    find_package(ament_gmock REQUIRED)
    ament_add_gmock(some_test <test_sources>)

It has the same additional parameters as ``ament_add_gtest``.

Extending ament
---------------

It is possible to register additional macros/functions with ``ament_cmake`` and extend it in several ways.

Adding a function/macro to ament
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Extending ament will often times mean that you want to have some functions available to other packages.
The best way to provide the macro to client packages is to register it with ament.

This can be done by appending the ``${PROJECT_NAME}_CONFIG_EXTRAS`` variable, which is used by ``ament_package()`` via

.. code-block:: cmake

    list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS
      path/to/file.cmake"
      other/pathto/file.cmake"
    )

Alternatively, you can directly add the files to the ``ament_package()`` call:

.. code-block:: cmake

    ament_package(CONFIG_EXTRAS
      path/to/file.cmake
      other/pathto/file.cmake
    )

Adding to extension points
^^^^^^^^^^^^^^^^^^^^^^^^^^

In addition to simple files with functions that can be used in other packages, you can also add extensions to ament.
Those extensions are scripts which are executed with the function which defines the extension point.
The most common use-case for ament extensions is probably registering rosidl message generators:
When writing a generator, you normally want to generate all messages and services with your generator also without modifying the code for the message/service definition packages.
This is possible by registering the generator as an extension to ``rosidl_generate_interfaces``.

As an example, see

.. code-block:: cmake

    ament_register_extension(
      "rosidl_generate_interfaces"
      "rosidl_generator_cpp"
      "rosidl_generator_cpp_generate_interfaces.cmake")

which registers the macro ``rosidl_generator_cpp_generate_interfaces.cmake`` for the package ``rosidl_generator_cpp`` to the extension point ``rosidl_generate_interfaces``.
When the extension point gets executed, this will trigger the execution of the script ``rosidl_generator_cpp_generate_interfaces.cmake`` here.
In particular, this will call the generator whenever the function ``rosidl_generate_interfaces`` gets executed.

The most important extension point aside from ``rosidl_generate_interfaces`` for generators is ``ament_package``, which will simply execute scripts with the ``ament_package()`` call.
This extension point is useful when registering resources (see below).

``ament_register_extension`` is a function which takes exactly three arguments:

- ``extension_point``: The name of the extension point (most of the time this will be one of ``ament_package`` or ``rosidl_generate_interfaces``)

- ``package_name``: The name of the package containing the CMake file (i.e. the project name of the project where the file is written to)

- ``cmake_filename``: The CMake file executed when the extension point is run

.. note::

   It is possible to define custom extension points in a similar manner to ``ament_package`` and ``rosidl_generate_interfaces``, but this should hardly be necessary.

Adding extension points
^^^^^^^^^^^^^^^^^^^^^^^

Very rarely, it might be interesting to define a new extension point to ament.

Extension points can be registered within a macro so that all extensions will be executed when the corresponding macro is called.
To do so:

- Define and document a name for your extension (e.g. ``my_extension_point``), which is the name passed to the ``ament_register_extension`` macro when using the extension point.

- In the macro/function which should execute the extensions call:

.. code-block:: cmake

   ament_execute_extensions(my_extension_point)

Ament extensions work by defining a variable containing the name of the extension point and filling it with the macros to be executed.
Upon calling ``ament_execute_extensions``, the scripts defined in the variable are then executed one after another.

Adding resources
----------------

Especially when developing plugins or packages which allow plugins it is often essential to add resources to one ROS package from another (e.g. a plugin).
Examples can be plugins for tools using the pluginlib.

This can be achieved using the ament index (also called "resource index").

The ament index explained
^^^^^^^^^^^^^^^^^^^^^^^^^

For details on the design and intentions, see `here <https://github.com/ament/ament_cmake/blob/master/ament_cmake_core/doc/resource_index.md>`__

In principle, the ament index is contained in a folder within the install/share folder of your package.
It contains shallow subfolders named after different types of resources.
Within the subfolder, each package providing said resource is referenced by name with a "marker file".
The file may contain whatever content necessary to obtain the resources, e.g. relative paths to the installation directories of the resource, it may also be simply empty.

To give an example, consider providing display plugins for RViz:
When providing RViz plugins in a project named ``my_rviz_displays`` which will be read by the pluginlib, you will provide a ``plugin_description.xml`` file, which will be installed and used by the pluginlib to load the plugins.
To achieve this, the plugin_description.xml is registered as a resource in the resource_index via

.. code-block:: cmake

    pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

When running ``colcon build``, this installs a file ``my_rviz_displays`` into a subfolder ``rviz_common__pluginlib__plugin`` into the resource_index.
Pluginlib factories within rviz_common will know to gather information from all folders named ``rviz_common__pluginlib__plugin`` for packages that export plugins.
The marker file for pluginlib factories contains an install-folder relative path to the ``plugins_description.xml`` file (and the name of the library as marker file name).
With this information, the pluginlib can load the library and know which plugins to load from the ``plugin_description.xml`` file.

As a second example, consider the possibility to let your own RViz plugins use your own custom meshes.
Meshes get loaded at startup time so that the plugin owner does not have to deal with it, but this implies RViz has to know about the meshes.
To achieve this, RViz provides a function:

.. code-block:: cmake

    register_rviz_ogre_media_exports(DIRECTORIES <my_dirs>)

This registers the directories as an ogre_media resource in the ament index.
In short, it installs a file named after the project which calls the function into a subfolder called ``rviz_ogre_media_exports``.
The file contains the install folder relative paths to the directories listed in the macros.
On startup time, RViz can now search for all folders called ``rviz_ogre_media_exports`` and load resources in all folders provided.
These searches are done using ``ament_index_cpp`` (or ``ament_index_py`` for Python packages).

In the following sections we will explore how to add your own resources to the ament index and provide best practices for doing so.

Querying the ament index
^^^^^^^^^^^^^^^^^^^^^^^^

If necessary, it is possible to query the ament index for resources via CMake.
To do so, there are three functions:

``ament_index_has_resource``: obtain a prefix path to the resource if it exists with the following parameters:

- ``var``: the output parameter: fill this variable with FALSE if the resource does not exist or the prefix path to the resource otherwise

- ``resource_type``: The type of the resource (e.g. ``rviz_common__pluginlib__plugin``)

- ``resource_name``: The name of the resource which usually amounts to the name of the package having added the resource of type resource_type (e.g. ``rviz_default_plugins``)

``ament_index_get_resource``: Obtain the content of a specific resource, i.e. the contents of the marker file in the ament index.

- ``var``: the output parameter: filled with the content of the resource marker file if it exists.

- ``resource_type``: The type of the resource (e.g. ``rviz_common__pluginlib__plugin``)

- ``resource_name``: The name of the resource which usually amounts to the name of the package having added the resource of type resource_type (e.g. ``rviz_default_plugins``)

- ``PREFIX_PATH``: The prefix path to search for (usually, the default ``ament_index_get_prefix_path()`` will be enough).

Note that ``ament_index_get_resource`` will throw an error if the resource does not exist, so it might be necessary to check using ``ament_index_has_resource``.

``ament_index_get_resources``: Get all packages which registered resources of a specific type from the index

- ``var``: Output parameter: filled with a list of names of all packages which registered a resource of resource_type

- ``resource_type``: The type of the resource (e.g. ``rviz_common__pluginlib__plugin``)

- ``PREFIX_PATH``: The prefix path to search for (usually, the default ``ament_index_get_prefix_path()`` will be enough).

Adding to the ament index
^^^^^^^^^^^^^^^^^^^^^^^^^

Defining a resource requires two bits of information:

- a name for the resource which must be unique,

- a layout of the marker file, which can be anything and could also be empty (this is true for instance for the "package" resource marking a ROS 2 package)

For the RViz mesh resource, the corresponding choices were:

- ``rviz_ogre_media_exports`` as name of the resource,

- install path relative paths to all folders containing resources. This will already enable you to write the logic for using the corresponding resource in your package.

To allow users to easily register resources for your package, you should furthermore provide macros or functions such as the pluginlib function or ``rviz_ogre_media_exports`` function.

To register a resource, use the ament function ``ament_index_register_resource``.
This will create and install the marker files in the resource_index.
As an example, the corresponding call for ``rviz_ogre_media_exports`` is the following:

.. code-block:: cmake

    ament_index_register_resource(rviz_ogre_media_exports CONTENT ${OGRE_MEDIA_RESOURCE_FILE})

This installs a file named like ``${PROJECT_NAME}`` into a folder ``rviz_ogre_media_exports`` into the resource_index with content given by variable ``${OGRE_MEDIA_RESOURCE_FILE}``.
The macro has a number of parameters that can be useful:

- the first (unnamed) parameter is the name of the resource, which amounts to the name of the folder in the resource_index

- ``CONTENT``: The content of the marker file as string. This could be a list of relative paths, etc. ``CONTENT`` cannot be used together with ``CONTENT_FILE``.

- ``CONTENT_FILE``: The path to a file which will be use to create the marker file. The file can be a plain file or a template file expanded with ``configure_file()``.
  ``CONTENT_FILE`` cannot be used together with ``CONTENT``.

- ``PACKAGE_NAME``: The name of the package/library exporting the resource, which amounts to the name of the marker file. Defaults to ``${PROJECT_NAME}``.

- ``AMENT_INDEX_BINARY_DIR``: The base path of the generated ament index. Unless really necessary, always use the default ``${CMAKE_BINARY_DIR}/ament_cmake_index``.

- ``SKIP_INSTALL``: Skip installing the marker file.

Since only one marker file exists per package, it is usually a problem if the CMake function/macro gets called twice by the same project.
However, for large projects it might be best to split up calls registering resources.

Therefore, it is best practice to let a macro registering a resource such as ``register_rviz_ogre_media_exports.cmake`` only fill some variables.
The real call to ``ament_index_register_resource`` can then be added within an ament extension to ``ament_package``.
Since there must only ever be one call to ``ament_package`` per project, there will always only be one place where the resource gets registered.
In the case of ``rviz_ogre_media_exports`` this amounts to the following strategy:

- The macro ``register_rviz_ogre_media_exports`` takes a list of folders and appends them to a variable called ``OGRE_MEDIA_RESOURCE_FILE``.

- Another macro called ``register_rviz_ogre_media_exports_hook`` calls ``ament_index_register_resource`` if ``${OGRE_MEDIA_RESOURCE_FILE}`` is non-empty.

- The ``register_rviz_ogre_media_exports_hook.cmake`` file is registered as an ament extension in a third file ``register_rviz_ogre_media_exports_hook-extras.cmake`` via calling

.. code-block:: cmake

    ament_register_extension("ament_package" "rviz_rendering"
      "register_rviz_ogre_media_exports_hook.cmake")

- The files ``register_rviz_ogre_media_exports.cmake`` and ``register_rviz_ogre_media_exports_hook-extra.cmake`` are registered as ``CONFIG_EXTRA`` with ``ament_package()``.
