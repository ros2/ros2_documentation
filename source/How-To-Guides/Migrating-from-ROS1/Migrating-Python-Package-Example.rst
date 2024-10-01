Migrating a Python Package Example
==================================

.. contents:: Table of Contents
   :depth: 2
   :local:

This guide shows how to migrate an example Python package from ROS 1 to ROS 2.

Prerequisites
-------------

You need a working ROS 2 installation, such as :doc:`ROS {DISTRO} <../../Installation>`.

The ROS 1 code
--------------

Say you have a ROS 1 package called ``talker_py`` that uses ``rospy`` in one node, called ``talker_py_node``.
This package is in a catkin workspace, located at ``~/ros1_talker``.

Your ROS 1 workspace has the following directory layout:

.. code-block:: bash

   $ cd ~/ros1_talker
   $ find .
   .
   src
   src/talker_py
   src/talker_py/package.xml
   src/talker_py/CMakeLists.txt
   src/talker_py/src
   src/talker_py/src/talker_py
   src/talker_py/src/talker_py/__init__.py
   src/talker_py/scripts
   src/talker_py/scripts/talker_py_node
   src/talker_py/setup.py

The files have the following content:

``src/talker_py/package.xml``:

.. code-block:: xml

    <?xml version="1.0"?>
    <?xml-model href="http://download.ros.org/schema/package_format2.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
    <package format="2">
        <name>talker_py</name>
        <version>1.0.0</version>
        <description>The talker_py package</description>
        <maintainer email="gerkey@osrfoundation.org">Brian Gerkey</maintainer>
        <license>BSD</license>

        <buildtool_depend>catkin</buildtool_depend>

        <depend>rospy</depend>
        <depend>std_msgs</depend>
    </package>

``src/talker_py/CMakeLists.txt``:

.. code-block:: cmake

    cmake_minimum_required(VERSION 3.0.2)
    project(talker_py)

    find_package(catkin REQUIRED)

    catkin_python_setup()

    catkin_package()

    catkin_install_python(PROGRAMS
        scripts/talker_py_node
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

``src/talker/src/talker_py/__init__.py``:

.. code-block:: python

    import rospy
    from std_msgs.msg import String

    def main():
        pub = rospy.Publisher('chatter', String, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            hello_str = "hello world %s" % rospy.get_time()
            rospy.loginfo(hello_str)
            pub.publish(hello_str)
            rate.sleep()

``src/talker_py/scripts/talker_py_node``:

.. code-block:: python

    #!/usr/bin/env python

    import talker_py

    if __name__ == '__main__':
        talker_py.main()

``src/talker_py/setup.py``:

.. code-block:: python

    from setuptools import setup
    from catkin_pkg.python_setup import generate_distutils_setup

    setup_args = generate_distutils_setup(
        packages=['talker_py'],
        package_dir={'': 'src'}
    )

    setup(**setup_args)

Migrating to ROS 2
------------------

TODO

Conclusion
----------

You have learned how to migrate an example Python ROS 1 package to ROS 2.
Use the :doc:`Migrating Python Packages reference page <./Migrating-Python-Packages>` to help you migrate your own Python packages from ROS 1 to ROS 2.
