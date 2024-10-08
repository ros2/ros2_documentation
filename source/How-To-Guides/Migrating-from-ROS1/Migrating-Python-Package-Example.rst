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

Create a colcon workspace at ``~/ros2_tallker_py``.

.. code-block:: bash

    mkdir -p ~/ros2_talker_py/src

Next, create a ROS 1 package in the new workspace.
The ROS 1 package is called ``talker_py``, and it has one node called ``talker_py_node``.

.. code-block:: bash

    cd ~/ros2_talker_py
    mkdir -p src/talker_py/src/talker_py
    mkdir -p src/talker_py/scripts
    touch src/talker_py/package.xml
    touch src/talker_py/CMakeLists.txt
    touch src/talker_py/src/talker_py/__init__.py
    touch src/talker_py/scripts/talker_py_node
    touch src/talker_py/setup.py

Put the following content into each file

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

You now have a ROS 1 package in a new workspace.

Migrating to ROS 2
------------------

When migrating large packages to ROS 2, it is helpful to build and run tests as you go.
Migrate the build system files first so that you can do this.
Always start with the ``package.xml``.

Migrating the ``package.xml``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

ROS 2 does not use ``catkin``.
Delete the ``<buildtool_depend>`` on it.

.. code-block::

    <!-- delete this -->
    <buildtool_depend>catkin</buildtool_depend>


ROS 2 uses ``rclpy`` instead of ``rospy``.
Delete the dependency on ``rospy``.

.. code-block::

    <!-- Delete this -->
    <depend>rospy</depend>


Add a new dependency on ``rclpy``.

.. code-block:: xml

    <depend>rclpy</depend>

Add an ``<export>`` section to tell colcon the package is an ``ament_python`` package instead of a ``catkin`` package.

.. code-block:: xml

     <export>
       <build_type>ament_python</build_type>
     </export>


Your ``package.xml`` is fully migrated.
It now looks like this:

.. code-block:: xml

    <?xml version="1.0"?>
    <?xml-model href="http://download.ros.org/schema/package_format2.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
    <package format="2">
        <name>talker_py</name>
        <version>1.0.0</version>
        <description>The talker_py package</description>
        <maintainer email="gerkey@osrfoundation.org">Brian Gerkey</maintainer>
        <license>BSD</license>

        <depend>rclpy</depend>
        <depend>std_msgs</depend>

        <export>
            <build_type>ament_python</build_type>
        </export>
    </package>

Migrating to ``setup.py`` and ``setup.cfg``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Python packages in ROS 2 do not use CMake, so delete the ``CMakeLists.txt``.

Create a new file called ``setup.cfg`` next to the ``package.xml``.
The ``setup.cfg`` needs two pieces of information with the package's name, ``talker_py``, in it.
Put the following content into ``setup.cfg``

.. code-block::

    [develop]
    script_dir=$base/lib/talker_py
    [install]
    install_scripts=$base/lib/talker_py

The ``setup.py`` can no longer be automatically generated with ``catkin_pkg``.
Start by deleting the import from ``catkin_pkg``.

.. code-block::

    # Delete this
    from catkin_pkg.python_setup import generate_distutils_setup

Move all arguments given to ``generate_distutils_setup()`` directly into the call to ``setup()``.

.. code-block:: python

    setup(
        packages=['talker_py'],
        package_dir={'': 'src'},
    )

Delete the call to ``generate_distutils_setup()``.

.. code-block::

    # Delete this
    setup_args = generate_distutils_setup(
        packages=['talker_py'],
        package_dir={'': 'src'}
    )

The call to ``setup()`` needs some `additional metadata <https://docs.python.org/3.11/distutils/setupscript.html#additional-meta-data>`__ copied from the ``package.xml``:

* package name via the ``name`` argument
* package version via the ``version`` argument
* maintainer via the ``maintainer`` and ``maintainer_email`` arguments
* description via the ``description`` argument
* license via the ``license`` argument

The package name will be used multiple times.
Create a variable called ``package_name`` in the ``setup.py``.

.. code-block:: python

    package_name = 'talker_py'

Copy all of the remaining information into the arguments of ``setup()`` in ``setup.py``.
Your call to ``setup()`` should look like this:

.. code-block:: python

    setup(
        name=package_name,
        version='1.0.0',
        packages=['talker_py'],
        package_dir={'': 'src'},
        maintainer='Brian Gerkey',
        maintainer_email='gerkey@osrfoundation.org',
        description='The talker_py package',
        license='BSD',
    )

A ROS 2 must install two additional data files so that command line tools like ``ros2 run`` can find the package:

* a ``package.xml``
* a package marker file

You already have a ``package.xml``, but you do not yet have a package marker file.
Create the marker file by creating a directory next to your ``package.xml`` called ``resource``.
Create an empty file in that directory with the same name as your package.

.. code-block:: bash

    mkdir resource
    touch resource/talker_py

You must tell ``setuptools`` how to install these files.
Add the following ``data_files`` argument to the call to ``setup()`` to do so.

.. code-block:: python

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],


TODO

.. code-block:: python

    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'talker_py_node = talker_py:main',
        ],
    },

Conclusion
----------

You have learned how to migrate an example Python ROS 1 package to ROS 2.
Use the :doc:`Migrating Python Packages reference page <./Migrating-Python-Packages>` to help you migrate your own Python packages from ROS 1 to ROS 2.
