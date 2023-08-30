Migrating Interfaces
====================

.. contents:: Table of Contents
   :depth: 2
   :local:

Messages, services, and actions are collectively called ``interfaces`` in ROS 2.

Interface definitions
---------------------

Message files must end in ``.msg`` and must be located in the subfolder ``msg``.
Service files must end in ``.srv`` and must be located in the subfolder ``srv``.
Actions files must end in ``.action`` and must be located in the subfolder ``action``.

These files might need to be updated to comply with the `ROS Interface definition <http://design.ros2.org/articles/legacy_interface_definition.html>`__.
Some primitive types have been removed and the types ``duration`` and ``time`` which were builtin types in ROS 1 have been replaced with normal message definitions and must be used from the `builtin_interfaces <https://github.com/ros2/rcl_interfaces/tree/{REPOS_FILE_BRANCH}/builtin_interfaces>`__ package.
Also some naming conventions are stricter than in ROS 1.
There is additional information in the :doc:`conceptual article <../../Concepts/Basic/About-Interfaces>`.

Building interfaces
-------------------

The way in which interfaces are built in ROS 2 differs substantially from ROS 1.
Interfaces can only be built from packages containing a ``CMakeLists.txt``.
If you are developing a pure Python package, then the interfaces should be placed in a different package containing only the interfaces (which is best practice anyway).
See the :doc:`custom interfaces tutorial<../../Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces>` for more information.

Migrating interface package to ROS 2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In your ``package.xml``:

* Add ``<buildtool_depend>rosidl_default_generators</buildtool_depend>``.
* Add ``<exec_depend>rosidl_default_runtime</exec_depend>``.
* Add ``<member_of_group>rosidl_interface_packages</member_of_group>``
* For each dependent message package, add ``<depend>message_package</depend>``.

In your ``CMakeLists.txt``:

* Enable C++17

.. code-block:: cmake

   set(CMAKE_CXX_STANDARD 17)

* Add ``find_package(rosidl_default_generators REQUIRED)``
* For each dependent message package, add ``find_package(message_package REQUIRED)`` and replace the CMake function call to ``generate_messages`` with ``rosidl_generate_interfaces``.

This will replace ``add_message_files`` and ``add_service_files`` listing of all the message and service files, which can be removed.

