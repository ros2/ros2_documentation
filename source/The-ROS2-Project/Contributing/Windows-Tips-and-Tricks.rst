.. redirect-from::

    Contributing/Windows-Tips-and-Tricks

Windows Tips and Tricks
=======================

.. contents:: Table of Contents
   :depth: 1
   :local:

ROS 2 supports Windows 10 as a Tier 1 platform, which means that all code that goes into the ROS 2 core must support Windows.
For those used to traditional development on Linux or other Unix-like systems, developing on Windows can be a bit of a challenge.
This document aims to lay out some of those differences.

Maximum Path Length
-------------------
By default, Windows has a `maximum path length <https://docs.microsoft.com/en-us/windows/win32/fileio/maximum-file-path-limitation>`__ of 260 characters.
Practically speaking, 4 of those characters are always used by the drive letter, colon, initial backslash, and final NULL character.
That means that only 256 characters are available for the *sum* of all parts of the path.
This has two practical consequences for ROS 2:

* Some of the ROS 2 internal path names are fairly long. Because of this, we always recommend using a short path name for the root of your ROS 2 directory, like ``C:\dev``.
* When building ROS 2 from source, the default isolated build mode of colcon can generate very long path names. To avoid these very long path names, use ``--merge-install`` when building on Windows.

**Note**: It is possible to change Windows to have much longer maximum path lengths.  See `this article <https://docs.microsoft.com/en-us/windows/win32/fileio/maximum-file-path-limitation?tabs=cmd#enable-long-paths-in-windows-10-version-1607-and-later>`__ for more information.

Exporting symbols
-----------------
All symbols are private by default when building a library on Windows.
The consequence of this is that without special handling, another library or executable cannot call any symbols in the library.
This special handling is called `dllexport <https://docs.microsoft.com/en-us/cpp/build/exporting-from-a-dll-using-declspec-dllexport?view=msvc-160>`__ and `dllimport <https://docs.microsoft.com/en-us/cpp/build/importing-into-an-application-using-declspec-dllimport?view=msvc-160>`__.
The ``dllexport`` is used in the library that wants to make a symbol publically callable, and ``dllimport`` is used by the library or application that wants to call the symbol.
In the ROS 2 core code, packages often have a header called ``visibility_macros.h`` that use macros to define the proper ``dllexport``/``dllimport`` statements.
With these macros in place, any symbol in header files that needs to be public needs to be decorated with the ``PUBLIC`` version of the macro.
The CMakeLists.txt of the package should also have a stanza like:

.. code-block:: cmake

  target_compile_definitions(${PROJECT_NAME}
    PRIVATE "XXX_BUILDING_LIBRARY")

where "XXX" is replaced with the name of the macro in ``visibility_macros.h``.

Finally, it is important that the header file that exports the symbols be included into at least one of the ``.cpp`` files in the package so that the macros will get expanded and placed into the resulting binary.
Otherwise the symbols will still not be callable.

Debug builds
------------
When building in Debug mode on Windows, several very important things change.
The first is that all DLLs get ``_d`` automatically appended to the library name.
So if the library is called ``libfoo.dll``, in Debug mode it will be ``libfoo_d.dll``.
The dynamic linker on Windows also knows to look for libraries of that form, so it will not find libraries without the ``_d`` prefix.
Additionally, Windows turns on a whole set of compile-time and run-time checks in Debug mode that is far more strict than Release builds.
For these reasons, it is a good idea to run a Windows Debug build and test on many pull requests.

Forward-slash vs. back-slash
----------------------------
In Windows the default path separator is a backslash (``\``), which differs from the forward-slash (``/``) used in Linux and macOS.
Most of the Windows APIs can deal with either as a path separator, but this is not universally true.
For instance, the ``cmd.exe`` shell can only do tab-completion when using the backslash character, not the forward-slash.
For maximum compatibility on Windows, a backslash should always be used as the path separator on Windows.

Patching vendored packages
--------------------------
When vendoring a package in ROS 2, it is often necessary to apply a patch to fix a bug, add a feature, etc.
The typical way to do this is to modify the ``ExternalProject_add`` call to add a ``PATCH`` command, using the ``patch`` executable.
Unfortunately, the ``patch`` executable as delivered by chocolatey requires Administrator access to run.
The workaround is to use ``git apply-patch`` when applying patches to external projects.

``git apply-patch`` has its own issues in that it only works properly when applied to a git repository.
For that reason, external projects should always use the ``GIT`` method to obtain the project and then use the ``PATCH_COMMAND`` to invoke ``git apply-patch``.

An example usage of all of the above looks something like:

.. code-block:: cmake

  ExternalProject_Add(mylibrary-${version}
    GIT_REPOSITORY https://github.com/lib/mylibrary.git
    GIT_TAG ${version}
    GIT_CONFIG advice.detachedHead=false
    # Suppress git update due to https://gitlab.kitware.com/cmake/cmake/-/issues/16419
    # See https://github.com/ament/uncrustify_vendor/pull/22 for details
    UPDATE_COMMAND ""
    TIMEOUT 600
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install
      ${extra_cmake_args}
      -Wno-dev
    PATCH_COMMAND
      ${CMAKE_COMMAND} -E chdir <SOURCE_DIR> git apply -p1 --ignore-space-change --whitespace=nowarn ${CMAKE_CURRENT_SOURCE_DIR}/install-patch.diff
  )

Windows slow timers (slowness in general)
-----------------------------------------
Software running on Windows is, in general, much slower than that running on Linux.
This is due to a number of factors, from the default time slice (every 20 ms, according to the `documentation <https://docs.microsoft.com/en-us/windows/win32/procthread/multitasking>`__), to the number of anti-virus and anti-malware processes running, to the number of background processes running.
Because of all of this, tests should *never* expect tight timing on Windows.
All tests should have generous timeouts, and only expect events to happen eventually (this will also prevent tests from being flakey on Linux).

Shells
------
There are two main command-line shells on Windows: the venerable ``cmd.exe``, and PowerShell.

``cmd.exe`` is the command shell that most closely emulates the old DOS shell, though with greatly enhanced capabilities.
It is completely text based, and only understands DOS/Windows ``batch`` files.

PowerShell is the newer, object-based shell that Microsoft recommends for most new applications.
It understands ``ps1`` files for configuration.

ROS 2 supports both ``cmd.exe`` and PowerShell, so any changes (especially to things like ``ament`` or ``colcon``) should be tested on both.
