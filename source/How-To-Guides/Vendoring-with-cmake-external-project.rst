Vendoring a non CMAKE package using AddExternalProject
======================================================

Sometimes you need to vendor a package that doesn't use CMAKE, because it is not in any distribution.
This How-To-Guide gives you some hints on how to do that, using the example of the lely_core_libraries package.

The lely_core_libraries package is a C++ library that is used by the ros2_canopen stack. It uses autotools as
build system and is only available via ppa for Ubuntu. To make it available in rosdistro, we need to vendor it.

Step 1: Create a new ament_cmake package
----------------------------------------

First you need to create an ament cmake package in your workspace.

.. code:: bash

    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_cmake <package_name>

Once you have created the package, delete all folders in the package, we do not need them.

Step 2: Add the external project to CMakeLists.txt
--------------------------------------------------
Open the CMakeLists.txt file and add the following code after the find_package() calls.

.. code::

    include(ExternalProject)
    ExternalProject_Add(upstr_lely_core_libraries
      SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/upstream
      INSTALL_DIR "${CMAKE_INSTALL_PREFIX}"
      BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/build
      GIT_REPOSITORY https://gitlab.com/lely_industries/lely-core.git
      GIT_TAG 7824cbb2ac08d091c4fa2fb397669b938de9e3f5
      TIMEOUT 60
      UPDATE_COMMAND
      COMMAND git reset --hard
      COMMAND git apply --whitespace=fix --reject ${CMAKE_CURRENT_SOURCE_DIR}/patches/0001-Fix-dcf-tools.patch
      CONFIGURE_COMMAND autoreconf -i <SOURCE_DIR>
      COMMAND <SOURCE_DIR>/configure --prefix=<INSTALL_DIR> --disable-cython --disable-doc --disable-tests --disable-static --disable-diag
      BUILD_COMMAND $(MAKE) -C ${CMAKE_CURRENT_BINARY_DIR}/build
      INSTALL_COMMAND ""
    )

Lets inspect a few details of the ExternalProject_Add() call.
You'll first need to configure your external project. You can find all available
options in the `documentation <https://cmake.org/cmake/help/latest/module/ExternalProject.html>`_.
In our case we need to set the source directory, the binary directory (build directory) and the install directory
to configure the paths used during build and installation.
We also need to define where the source is that we want to build, for us it is a git repository
at a specific commit. We set the timeout to fetch the sources to 60 seconds.

.. code::

    # Configuration
    SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/upstream
    INSTALL_DIR "${CMAKE_INSTALL_PREFIX}"
    BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/build
    GIT_REPOSITORY https://gitlab.com/lely_industries/lely-core.git
    GIT_TAG 7824cbb2ac08d091c4fa2fb397669b938de9e3f5
    TIMEOUT 60

Next you can define an update or patch step. In the case of lely_core_libraries
we need to apply a patch to fix the installation of a component.

.. code::

    # UPDATE step apply patch to fix dcf-tools install
    UPDATE_COMMAND
    COMMAND git reset --hard
    COMMAND git apply --whitespace=fix --reject ${CMAKE_CURRENT_SOURCE_DIR}/patches/0001-Fix-dcf-tools.patch

The next step is the configure step. For autotools projects you need to call
autoreconf and then the configure script.
You need to add the ``--prefix=<INSTALL_DIR>`` option to the configure script to
make sure the project is installed into the workspaces install directory.

.. code::

    # CONFIGURE step execute autoreconf and configure
    CONFIGURE_COMMAND autoreconf -i <SOURCE_DIR>
    COMMAND <SOURCE_DIR>/configure --prefix=<INSTALL_DIR> --disable-cython --disable-doc --disable-tests --disable-static --disable-diag

The build step is the easiest one, just call make in the build directory.

.. code::

    # BUILD STEP execute make
    BUILD_COMMAND $(MAKE) -C ${CMAKE_CURRENT_BINARY_DIR}/build

The last step is the install step. In our case we do not want to install the
package from external project as this will break debian and rpm builds.
We need to call it from the main flow in the CMakeLists.txt file.

Step 3: Add install in CMakeLists.txt
-------------------------------------

To install the package we need to add the ``make install`` command of the external
project as an install command.

.. code:: cmake

    install(CODE "execute_process(COMMAND make install WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/build)")

This will run the ``make install`` command of the external project during
the install step of the main CMakeLists.txt file.

Step 4: Update package.xml with dependencies
--------------------------------------------

Now you will need to update the package.xml file with the dependencies of
the external project. In our case we need python3-empy.

.. code:: xml

    <depend>python3-empy</depend>

Also make sure that versioning and licensing information are correct.

Step 5: Build the package
-------------------------

You can now build the package using colcon and check if the external project
builds.

.. code:: bash

    colcon build --packages-select <package_name>

If successful, you can check if the external project was correctly installed.
Check in the install folder of your workspace if the you find all headers and
libraries and other files in the include and lib folders.

You can also test if it is possible to build the package as a debian using bloom.
To do so run the following commands in the packages root folder.

.. code:: bash

    bloom-generate rosdebian --os-name ubuntu --os-version bionic --ros-distro dashing
    fakeroot debian/rules binary

If the debian builds successfully, you can be reasonably sure that a bloom release will
build on the build farm.

For more checks also check prerelease tests.
