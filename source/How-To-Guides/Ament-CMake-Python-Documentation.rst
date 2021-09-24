.. redirect-from::

  Guides/Ament-CMake-Python-Documentation

ament_cmake_python user documentation
=====================================

``ament_cmake_python`` is a package that provides CMake functions for packages of the ``ament_cmake`` build type that contain Python code.
See the `ament_cmake user documentation <Ament-CMake-Documentation>`__ for more information.

.. note::

   Pure Python packages should use the ``ament_python`` build type in most cases.
   To create an ``ament_python`` package, see `Creating your first ROS 2 package <../Tutorials/Creating-Your-First-ROS2-Package>`__.
   ``ament_cmake_python`` should only be used in cases where that is not possible, like when mixing C/C++ and Python code.

.. contents:: Table of Contents
   :depth: 2
   :local:

Basics
------

Basic project outline
^^^^^^^^^^^^^^^^^^^^^

The outline of a package called "my_project" with the ``ament_cmake`` build type that uses ``ament_cmake_python`` looks like:

.. code-block::

   .
   └── my_project
       ├── CMakeLists.txt
       ├── package.xml
       └── my_project
           ├── __init__.py
           └── my_script.py

The ``__init__.py`` file can be empty, but it is needed to `make Python treat the directory containing it as a package <https://docs.python.org/3/tutorial/modules.html#packages>`__.
There can also be a ``src`` or ``include`` directory alongside the ``CMakeLists.txt`` which holds C/C++ code.

Using ament_cmake_python
^^^^^^^^^^^^^^^^^^^^^^^^

The package must declare a dependency on ``ament_cmake_python`` in its ``package.xml``.

.. code-block:: xml

   <buildtool_depend>ament_cmake_python</buildtool_depend>

The ``CMakeLists.txt`` should contain:

.. code-block:: cmake

   find_package(ament_cmake_python REQUIRED)
   # ...
   ament_python_install_package(${PROJECT_NAME})

The argument to ``ament_python_install_package()`` is the name of the directory alongside the ``CMakeLists.txt`` that contains the Python file.
In this case, it is ``my_project``, or ``${PROJECT_NAME}``.

Then, another Python package that correctly depends on ``my_project`` can use it as a normal Python module:

.. code-block:: python

   from my_project.my_script import my_function

Assuming ``my_script.py`` contains a function called ``my_function()``.
