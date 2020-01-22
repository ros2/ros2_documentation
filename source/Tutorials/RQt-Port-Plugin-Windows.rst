.. redirect-from::

    RQt-Port-Plugin-Windows

Porting RQt plugins to Windows
==============================

.. contents:: Table of Contents
   :depth: 2
   :local:

RQt has not been historically supported on Windows, but compatibility is happening, slowly.

RQt Porting examples
--------------------

Microsoft pushed an effort to port much of ROS to Windows, their pull request is a good resource for necessary changes.
For example: https://github.com/ros-visualization/qt_gui_core/pull/188

Here is the ROS 2 port of `qt_gui_core <https://github.com/ros-visualization/qt_gui_core/commit/6fb9624033a849f56d1bc1aad0e40d252bf99c2b>`_.

Here is the ROS 2 port of `python_qt_binding <https://github.com/ros-visualization/python_qt_binding/pull/56>`__.

Considerations for Windows 10
-----------------------------

Troubles with TinyXML version 1
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

I could not successfully use TinyXML.
I upgraded to TinyXML-2 where needed.
It’s a pretty straight forward change.

Checkout `this PR <https://github.com/ros-visualization/qt_gui_core/pull/147>`__ for an example of porting to TinyXML-2.

Code that uses ``__cplusplus`` and code that requires pluginlib
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In some places, notably in the ROS 2 port of pluginlib, there is use of the ``__cplusplus`` flag.
Unfortunately on Windows Visual Studio does not set this flag correctly regardless of the C++ standard that is actually being used.
See `this page <https://docs.microsoft.com/en-us/cpp/build/reference/zc-cplusplus?view=vs-2017>`__ for more information.

To set it, you need to add the compile option ``/Zc:__cplusplus``.

For example, in CMake you could do something like this:

.. code-block:: cmake

   target_compile_options(${PROJECT_NAME} PUBLIC "/Zc:__cplusplus")

Locations of build artifacts (before install)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This only came up during when building ``qt_gui_cpp``.
In that package, a custom command depends on a target library in another part of the package.
However, that library isn’t installed until build is complete. Windows builds in a ${configuration} directory.
For example:

On Linux, ``qt_gui_cpp.a`` would be built in:
<ros2_ws>/build/qt_gui_cpp/src/qt_gui_cpp/

But on Windows ``qt_gui_cpp.lib`` is built in
<ros2_ws>/build/qt_gui_cpp/src/qt_gui_cpp/Release

For compatibility across platforms in this situation, use `CMake generator expressions <https://cmake.org/cmake/help/v3.5/manual/cmake-generator-expressions.7.html>`__.
However, when you need a library to link against be sure to use ``$<TARGET_LINKER_FILE:_target>`` instead of ``$<TARGET_FILE:_target>``.
The latter will find ``.dll`` files, which cannot be linked against on Windows.
See an `example here <https://github.com/ros-visualization/qt_gui_core/pull/162/files>`__.

Compiler and linker flags
^^^^^^^^^^^^^^^^^^^^^^^^^

In general when porting to Windows, many packages might make use of additional compiler flags.
You can find the Windows compiler flags on `Microsoft's documentation <https://docs.microsoft.com/en-us/cpp/build/reference/compiler-options-listed-by-category?view=vs-2017>`__.
The C++ compiler is called ``cl.exe``.

For linker flags see `Microsoft's documentation <https://docs.microsoft.com/en-us/cpp/build/reference/linker-options?view=vs-2017>`__.
The linker program is called ``link.exe``.

However, CMake actually provides many of these options in variables.
This `StackOverflow page <https://stackoverflow.com/questions/9298278/cmake-print-out-all-accessible-variables-in-a-script>`__ contains a good example of how to see all the CMake variables available in a script.
