.. redirect-from::

    Developing-a-ROS-2-Package

Developing a ROS 2 Package
##########################

This tutorial goes through how to create your first ROS 2 application. It is
suggested you follow this guide as part of the quickstart tutorials.

All ROS 2 packages begin by running the command

.. code-block:: bash

   ros2 pkg create <pkg-name> --dependencies [deps]

in your workspace (usually ``~/ros2_ws/src``).

To explicitly create a C++ package

.. code-block:: bash

   ros2 pkg create <pkg-name> --dependencies [deps] --build-type ament_cmake

To explicitly create a Python package

.. code-block:: bash

   ros2 pkg create <pkg-name> --dependencies [deps] --build-type ament_python

You can then update the ``package.xml`` with your package info such as
dependencies, descriptions, and authorship.

C++ Packages
************

You will mostly use the

.. code-block:: cmake

   add_executable(<executable-name> <executable-file>)

macro along with

.. code-block:: cmake

   ament_target_dependencies(<executable-name> [dependencies])

to create executable nodes and link dependencies.

To install your launch files and nodes, you can use the ``install`` macro
placed towards the end of the file but before the ``ament_package()`` macro.

.. code-block:: cmake

   # Install launch files.
   install(
     DIRECTORY launch
     DESTINATION share/${PROJECT_NAME}/
   )

   # Install nodes
   install(
     TARGETS [node-names]
     DESTINATION lib/${PROJECT_NAME}
   )

Python Packages
***************

Your ``setup.cfg`` file should look like

.. code-block:: bash

   [develop]
   script-dir=$base/lib/<package-name>
   [install]
   install-scripts=$base/lib/<package-name>

and your ``setup.py`` file should look like

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
           ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
           # Include our package.xml file
           (os.path.join('share', package_name), ['package.xml']),
           # Include all launch files. This is the most important line here!
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
               'my_script = my_package.script:main'
           ],
       },
   )

Follow the next tutorial on how to create your first node.
