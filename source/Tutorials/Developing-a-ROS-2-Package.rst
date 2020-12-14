.. redirect-from::

    Developing-a-ROS-2-Package

Developing a ROS 2 package
##########################

.. contents:: Table of Contents
   :depth: 2
   :local:

This tutorial will teach you how to create your first ROS 2 application.
It is intended for developers who want to learn how to create custom packages in ROS 2, not for people who want to use ROS 2 with its existing packages.

Prerequisites
-------------

- `Install ROS <../../Installation>`__

- `Install colcon <https://colcon.readthedocs.io/en/released/user/installation.html>`__

- Setup your workspace by sourcing your ROS 2 installation.

Creating a package
------------------

All ROS 2 packages begin by running the command

.. code-block:: bash

   ros2 pkg create <pkg-name> --dependencies [deps]

in your workspace (usually ``~/ros2_ws/src``).

To create a package for a specific client library:

.. tabs::

  .. group-tab:: C++

    .. code-block:: bash

       ros2 pkg create <pkg-name> --dependencies [deps] --build-type ament_cmake

  .. group-tab:: Python

    .. code-block:: bash

       ros2 pkg create <pkg-name> --dependencies [deps] --build-type ament_python

You can then update the ``package.xml`` with your package info such as dependencies, descriptions, and authorship.

C++ Packages
^^^^^^^^^^^^

You will mostly use the ``add_executable()`` CMake macro along with

.. code-block:: cmake

   ament_target_dependencies(<executable-name> [dependencies])

to create executable nodes and link dependencies.

To install your launch files and nodes, you can use the ``install()`` macro placed towards the end of the file but before the ``ament_package()`` macro.

An example for launch files and nodes:

.. code-block:: cmake

   # Install launch files
   install(
     DIRECTORY launch
     DESTINATION share/${PROJECT_NAME}
   )

   # Install nodes
   install(
     TARGETS [node-names]
     DESTINATION lib/${PROJECT_NAME}
   )

Python Packages
^^^^^^^^^^^^^^^

ROS 2 follows Python's standard module distribution process that uses ``setuptools``.
For Python packages, the ``setup.py`` file complements a C++ package's ``CMakeLists.txt``.
More details on distribution can be found in the `official documentation <https://docs.python.org/3/distributing/index.html#distributing-index>`_.

In your ROS 2 package, you should have a ``setup.cfg`` file which looks like:

.. code-block:: bash

   [develop]
   script-dir=$base/lib/<package-name>
   [install]
   install-scripts=$base/lib/<package-name>

and a ``setup.py`` file that looks like:

.. code-block:: python

   import os
   from glob import glob
   from setuptools import setup

   package_name = 'my_package'

   setup(
       name=package_name,
       version='0.0.0',
       # Packages to export
       packages=[package_name],
       # Files we want to install, specifically launch files
       data_files=[
           # Install marker file in the package index
           ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
           # Include our package.xml file
           (os.path.join('share', package_name), ['package.xml']),
           # Include all launch files.
           (os.path.join('share', package_name, 'launch'), glob('*.launch.py'))
       ],
       # This is important as well
       install_requires=['setuptools'],
       zip_safe=True,
       author='ROS 2 Developer',
       author_email='ros2@ros.com',
       maintainer='ROS 2 Developer',
       maintainer_email='ros2@ros.com',
       keywords=['foo', 'bar'],
       classifiers=[
           'Intended Audience :: Developers',
           'License :: TODO',
           'Programming Language :: Python',
           'Topic :: Software Development',
       ],
       description='My awesome package.',
       license='TODO',
       # Like the CMakeLists add_executable macro, you can add your python
       # scripts here.
       entry_points={
           'console_scripts': [
               'my_script = my_package.my_script:main'
           ],
       },
   )
