Vendoring a library as a ROS 2 package
======================================

When should you consider vendoring a package?
---------------------------------------------

.. warning::

    Vendoring is discouraged and should only be considered as a last resort.

Vendoring a package means that you include the source code of a library in
a ros package and build it using colcon. This is useful if you want
publish a package that needs a library that is not available as dependency
from other sources.
Other sources can be:

* **OS package manager** - If the library is available as a package from 
    the OS package manager, you should use that package instead of vendoring.
    You can add it to rosdistro via base.yml.
* **Python package manager** - If the library is available as a python package
    from pip, you should use that package instead of vendoring.
    You can add it to rosdistro.

Only consider vendoring a package if it is not available from any of these
sources.


Vendoring a libary using ament_cmake_vendor_package
----------------------------------------------------

.. note::

    This is the recommended way to vendor a CMake based package.

The easiest way to vendor a library is to use the
``ament_cmake_vendor_package`` function in your CMakeLists.txt file.
This function can be used to vendor a third party library that uses
CMake as build system.

The first step is to create a new ament_cmake package in your workspace.

.. code:: bash

    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_cmake <package_name>

Once you have created the package, delete all folders in the package,
you do not need them. In case you need to apply patches to the code,
create a patches folder in the package and add your patches there.
The folder structure should look like this:

.. code:: bash

    <package_name>
    ├── CMakeLists.txt
    ├── package.xml
    └── patches
        └── 0001-<patch_name>.patch

Now you can modify the CMakeLists.txt file to build your vendor package
using the ``ament_cmake_vendor_package`` module. Add a find_package() call
to find the ament_cmake_vendor_package module. Then call the ament_vendor()
function. It takes a variety of arguments, the most important ones are
``VCS_URL``, ``VCS_TYPE`` and ``VCS_VERSION``. The ``VCS_URL`` is the
url to the source code. The ``VCS_TYPE`` is the type of version control
system used by the source code. The ``VCS_VERSION`` is the version of the
source code you want to use. You can also use a branch name or a tag name
here. The ``ament_vendor()`` function will download the source code and
build it as part of your package.

.. code:: cmake

    cmake_minimum_required(VERSION 3.10)
    project(<project_name>)

    find_package(ament_cmake REQUIRED)
    find_package(ament_cmake_vendor_package REQUIRED)

    ament_vendor(<project_name>
    VCS_URL <url>
    VCS_VERSION <version>
    VCS_TYPE git
    )

    ament_package()

For more possible arguments to the ``ament_vendor()`` function, check the
in source documentation of the function on [github](https://github.com/ament/ament_cmake/blob/rolling/ament_cmake_vendor_package/cmake/ament_vendor.cmake).


Vendoring a package using AddExternalProject manually
-----------------------------------------------------

.. note::

    This is the way to vendor a library that is not using CMake as build system.


Our example here will focus on autotools-based libraries, but the concepts are
applicable to other build systems as well.

First you need to create an ament cmake package in your workspace.

.. code:: bash

    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_cmake <package_name>

Once you have created the package, delete all folders in the package,
we do not need them. In case you need to apply patches to the code,
create a patches folder in the package and add your patches there.
The folder structure should look like this:

.. code:: bash

    <package_name>
    ├── CMakeLists.txt
    ├── package.xml
    └── patches
        └── 0001-<patch_name>.patch

Then open the CMakeLists.txt file and add ``include(ExternalProject)`` and the
call to the ``ExternalProject_Add()`` function.

.. code::

    include(ExternalProject)
    ExternalProject_Add(<package_name>_upstream
      SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/upstream
      INSTALL_DIR "${CMAKE_INSTALL_PREFIX}"
      BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/build
      GIT_REPOSITORY <git-url>
      GIT_TAG <tag>>
      TIMEOUT 60
      UPDATE_COMMAND
      COMMAND git reset --hard
      COMMAND git apply --whitespace=fix --reject ${CMAKE_CURRENT_SOURCE_DIR}/patches/<patch-name>.patch
      CONFIGURE_COMMAND autoreconf -i <SOURCE_DIR>
      COMMAND <SOURCE_DIR>/configure --prefix=<INSTALL_DIR> --disable-xxx
      BUILD_COMMAND $(MAKE) -C ${CMAKE_CURRENT_BINARY_DIR}/build
      INSTALL_COMMAND ""
    )

Lets inspect a few details of the ``ExternalProject_Add()`` call.
You'll first need to configure your external project. You can find all available
options in the `documentation <https://cmake.org/cmake/help/latest/module/ExternalProject.html>`_.
In our case we need to set the source directory, the binary directory (build directory) and the install directory
to configure the paths used during build and installation.
We also need to define where the source is that we want to build, for us it is a git repository
at a specific commit. We set the timeout to fetch the sources to 60 seconds.

.. code::

    SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/upstream
    INSTALL_DIR "${CMAKE_INSTALL_PREFIX}"
    BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/build
    GIT_REPOSITORY <git-url>
    GIT_TAG <tag>>
    TIMEOUT 60

Next you can define an update or patch step.

.. code::

    UPDATE_COMMAND
    COMMAND git reset --hard
    COMMAND git apply --whitespace=fix --reject ${CMAKE_CURRENT_SOURCE_DIR}/patches/<patch-name>.patch

The next step is the configure step. For autotools projects you need to call
autoreconf and then the configure script.
You need to add the ``--prefix=<INSTALL_DIR>`` option to the configure script
to make sure the project is installed into the workspaces install directory.

.. code::

      CONFIGURE_COMMAND autoreconf -i <SOURCE_DIR>
      COMMAND <SOURCE_DIR>/configure --prefix=<INSTALL_DIR> --disable-xxx

The build step is the easiest one, just call make in the build directory.

.. code::

    BUILD_COMMAND $(MAKE) -C ${CMAKE_CURRENT_BINARY_DIR}/build

The last step is the install step. In our case we do not want to install the
package from ``ExternalProject_Add()`` as this will break debian and rpm
builds. We need to call it from the main flow in the CMakeLists.txt file.

To install the library in the ros install folder we need to add the
``make install`` command of the external project as an install command.

.. code:: cmake

    install(CODE "execute_process(COMMAND make install WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/build)")

This will run the ``make install`` command of the external project during
the install step of the main CMakeLists.txt file.

Now you will need to update the package.xml file with the dependencies of
the external project. In our case we need python3-empy.

.. code:: xml

    <depend>python3-empy</depend>

Also make sure that versioning and licensing information are correct.
You can now build the package using colcon and check if the external project
builds.

.. code:: bash

    colcon build --packages-select <package_name>

If successful, you can check if the external project was correctly installed.
Check in the install folder of your workspace if the you find all headers and
libraries and other files in the include and lib folders.

You can also test if it is possible to build the package as a debian
using bloom. To do so run the following commands in the packages root folder.

.. code:: bash

    bloom-generate rosdebian --os-name ubuntu --os-version bionic --ros-distro dashing
    fakeroot debian/rules binary

If the debian builds successfully, you can be reasonably sure
that a bloom release will build on the build farm.

For more checks also check prerelease tests.
