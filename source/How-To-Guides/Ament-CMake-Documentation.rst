.. redirect-from::

  Guides/Ament-CMake-Documentation
  Tutorials/Ament-CMake-Documentation

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

   cmake_minimum_required(VERSION 3.14)
   project(my_project)

   ament_package()

The argument to ``project`` will be the package name and must be identical to the package name in the ``package.xml``.

The project setup is done by ``ament_package()`` and this call must occur exactly once per package.
``ament_package()`` installs the ``package.xml``, registers the package with the ament index, and installs CMake files so that it can be found by other packages using ``find_package``.
``ament_package()`` should be the last call in your ``CMakeLists.txt``.

``ament_package`` can be given additional arguments:

- ``CONFIG_EXTRAS``: a list of CMake files (``.cmake`` or ``.cmake.in`` templates expanded by ``configure_file()``) which should be available to clients of the package.
  For an example of when to use these arguments, see the discussion in `Adding resources`_.
  For more information on how to use template files, see `the official documentation <https://cmake.org/cmake/help/v3.5/command/configure_file.html>`__.

- ``CONFIG_EXTRAS_POST``: same as ``CONFIG_EXTRAS``, but the order in which the files are added differs.
  While ``CONFIG_EXTRAS`` files are included before the files generated for the ``ament_export_*`` calls the files from ``CONFIG_EXTRAS_POST`` are included afterwards.

Adding files and headers
^^^^^^^^^^^^^^^^^^^^^^^^

There are two main targets to build: libraries and executables.
These are built by `add_library() <https://cmake.org/cmake/help/v3.14/command/add_library.html>`__ and `add_executable() <https://cmake.org/cmake/help/v3.14/command/add_executable.html>`__ respectively.

It's recommended to put all headers which should be usable by clients into a subdirectory of the ``include`` folder (such as ``include/my_project/foo.hpp``, while all other files (``.c/.cpp`` and header files which should not be installed) should be inside a ``src`` folder.

Use `target_include_directories() <https://cmake.org/cmake/help/v3.14/command/target_include_directories.html>`__ to tell libraries and executables where the headers are.

.. code-block:: cmake

    target_include_directories(my_target
      PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

Adding Dependencies
^^^^^^^^^^^^^^^^^^^

The recommended way is to use `target_link_libraries() <https://cmake.org/cmake/help/v3.14/command/target_link_libraries.html>`__ and modern CMake targets.
Here's how to link the target ``my_target`` against the ROS C++ client library ``rclcpp``.

.. code-block:: cmake

    find_package(rclcpp REQUIRED)
    target_link_libraries(my_target PUBLIC rclcpp::rclcpp)

That will make sure your target can include ``rclcpp``'s headers and link against its libraries.

.. note::

   It should never be necessary to ``find_package`` a package you do not directly use.
   If you find that's the case, then one of your dependencies did not properly ``find_package`` it for you.
   File a bug against the offending package.

Building a Library
^^^^^^^^^^^^^^^^^^

When building a reusable library, some information needs to be exported for downstream packages to easily use it.

.. code-block:: cmake

    ament_export_targets(my_libraryTargets HAS_LIBRARY_TARGET)
    ament_export_dependencies(some_dependency)

    install(
      DIRECTORY include/
      DESTINATION include/${PROJECT_NAME}
    )

    install(
      TARGETS my_library
      EXPORT my_libraryTargets
      LIBRARY DESTINATION lib
      ARCHIVE DESTINATION lib
      RUNTIME DESTINATION bin
    )


Here, we assume that the folder ``include`` contains the headers which need to be exported.

Here is what the above snippet does.

- The ``ament_export_targets`` makes sure packages that depend on yours can use your library with ``target_link_libraries(downstream_target my_package::my_library)``.
  ``ament_export_targets`` takes a name for the export file. You can use any name as long as it matches the name you give to ``EXPORT`` in the the `install() <https://cmake.org/cmake/help/v3.14/command/install.html#installing-targets>`__ command.
  The option ``HAS_LIBRARY_TARGET`` makes sure environment variables like ``LD_LIBRARY_PATH`` are set when a workspace containing your package is sourced.
- The ``ament_export_dependencies`` exports dependencies to downstream packages.

.. warning::

   The macros ``ament_export_targets``, ``ament_export_dependencies`` must be called in the top level CMakeLists.txt, not a subdirectory.
   This is necessary so that the user of the library does not have to call ``find_package`` for those dependencies, too.

- The first ``install`` commands installs the package's public header files.

- The last large install command installs the library.
  Archive and library files will be exported to the lib folder, runtime binaries will be installed to the bin folder.

.. note::

   Windows DLLs are treated as runtime artifacts and installed into the ``RUNTIME DESTINATION`` folder.
   It is therefore advised to not leave out the ``RUNTIME DESTINATION`` option even when developing libraries on Unix based systems.

- The ``EXPORT`` notation installs an export file defined earlier by ``ament_export_targets``.

- All install paths are relative to `CMAKE_INSTALL_PREFIX <https://cmake.org/cmake/help/v3.14/variable/CMAKE_INSTALL_PREFIX.html>`__, which you do not need to set when using `colcon <https://colcon.readthedocs.io/en/released/>`__.

Compiler and linker options
^^^^^^^^^^^^^^^^^^^^^^^^^^^

ROS 2 releases have minimum C and C++ standards, which cna be found in `REP-2000 <https://www.ros.org/reps/rep-2000.html>`__.
It's recommended to set them using `target_compile_features() <https://cmake.org/cmake/help/v3.14/command/target_compile_features.html>`__

.. code-block:: cmake

    # If using C
    target_compile_features(my_library PUBLIC c_std_99)
    # If using C++
    target_compile_features(my_library PUBLIC cxx_std_17)

To keep the code clean, compilers should throw warnings for questionable code and these warnings should be fixed.

It is recommended to at least cover the following warning levels:

- For Visual Studio, the default ``W1`` warnings are kept

- For GCC and Clang: ``-Wall -Wextra -Wpedantic`` are required and ``-Wshadow -Werror`` are advisable (the latter makes warnings errors).

Modern CMake advises to add compiler flags on a target basis with `target_compile_options() <https://cmake.org/cmake/help/v3.14/command/target_compile_options.html>`__

.. code-block:: cmake

    target_compile_options(my_target PRIVATE -Wall)

Building libraries on Windows
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Windows is an officially supported platform, so we recommend making sure your packages can build on it.

The Windows library format enforces symbol visibility:
Every symbol which should be used from a client has to be explicitly exported by the library (and data symbols need to be implicitly imported).

Use the logic in `the GCC wiki <https://gcc.gnu.org/wiki/Visibility>`__ to keep this compatible with Clang and GCC builds.
Say your package creates a library named ``my_library``:

- Copy the logic in the link into a header file called ``visibility_control.hpp``.

- Replace ``DLL`` by ``MY_LIBRARY`` (for an example, see visibility control of `rviz_rendering <https://github.com/ros2/rviz/blob/ros2/rviz_rendering/include/rviz_rendering/visibility_control.hpp>`__).

- Use the macros "MY_LIBRARY_PUBLIC" for all symbols you need to export (i.e. classes or functions).

- In the project ``CMakeLists.txt`` use:

.. code-block:: cmake

    target_compile_definitions(my_library PRIVATE "MY_LIBRARY_BUILDING_LIBRARY")

For more details, see :ref:`Windows Symbol Visibility in the Windows Tips and Tricks document <Windows_Symbol_Visibility>`.

Testing and Linting
-------------------

In order to separate testing from building the library with colcon, wrap all calls to linters and tests in a conditional:

.. code-block:: cmake

    if(BUILD_TESTING)
      find_package(ament_cmake_gtest REQUIRED)
      ament_add_gtest(<tests>)
    endif()

Linting
^^^^^^^

It's advised to use the combined call from `ament_lint_auto <https://github.com/ament/ament_lint/blob/{REPOS_FILE_BRANCH}/ament_lint_auto/doc/index.rst#ament_lint_auto>`_:

.. code-block:: cmake

    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

This will run linters as defined in the ``package.xml``.
It is recommended to use the set of linters defined by the package ``ament_lint_common``.
The individual linters included there, as well as their functions, can be seen in the `ament_lint_common docs <https://github.com/ament/ament_lint/blob/{REPOS_FILE_BRANCH}/ament_lint_common/doc/index.rst>`_.

Linters provided by ament can also be added separately, instead of running ``ament_lint_auto``.
One example of how to do so can be found in the `ament_cmake_lint_cmake documentation <https://github.com/ament/ament_lint/blob/{REPOS_FILE_BRANCH}/ament_cmake_lint_cmake/doc/index.rst>`_.

Testing
^^^^^^^

Ament contains CMake macros to simplify setting up GTests. Call:

.. code-block:: cmake

    find_package(ament_cmake_gtest)
    ament_add_gtest(some_test <test_sources>)

to add a GTest.
This is then a regular target which can be linked against other libraries (such as the project library).
The macros have additional parameters:

- ``APPEND_ENV``: append environment variables.
  For instance you can add to the ament prefix path by calling:

.. code-block:: cmake

    find_package(ament_cmake_gtest REQUIRED)
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

    find_package(ament_cmake_gmock REQUIRED)
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

The most important extension point for generators, aside from ``rosidl_generate_interfaces``, is ``ament_package``, which will simply execute scripts with the ``ament_package()`` call.
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

For details on the design and intentions, see `here <https://github.com/ament/ament_cmake/blob/{REPOS_FILE_BRANCH}/ament_cmake_core/doc/resource_index.md>`__

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
