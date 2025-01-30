Migrating a Python Package Example
==================================

This guide shows how to migrate an example Python package from ROS 1 to ROS 2.

.. contents:: Table of Contents
   :depth: 2
   :local:

Prerequisites
-------------

You need a working ROS 2 installation, such as :doc:`ROS {DISTRO} <../../Installation>`.

The ROS 1 code
--------------

You won't be using `catkin <https://index.ros.org/p/catkin/>`__ in this guide, so you don't need a working ROS 1 installation.
You are going to use ROS 2's build tool `Colcon <https://colcon.readthedocs.io/>`__ instead.

This section gives you the code for a ROS 1 Python package.
The package is called ``talker_py``, and it has one node called ``talker_py_node``.
To make it easier to run Colcon later, these instructions make you create the package inside a `Colcon workspace <https://colcon.readthedocs.io/en/released/user/what-is-a-workspace.html>`__,

First, create a folder at ``~/ros2_talker_py`` to be the root of the Colcon workspace.

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

        mkdir -p ~/ros2_talker_py/src

  .. group-tab:: macOS

    .. code-block:: bash

        mkdir -p ~/ros2_talker_py/src

  .. group-tab:: Windows

    .. code-block:: bash

        md \ros2_talker_py\src

Next, create the files for the ROS 1 package.

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

        cd ~/ros2_talker_py
        mkdir -p src/talker_py/src/talker_py
        mkdir -p src/talker_py/scripts
        touch src/talker_py/package.xml
        touch src/talker_py/CMakeLists.txt
        touch src/talker_py/src/talker_py/__init__.py
        touch src/talker_py/scripts/talker_py_node
        touch src/talker_py/setup.py

  .. group-tab:: macOS

    .. code-block:: bash

        cd ~/ros2_talker_py
        mkdir -p src/talker_py/src/talker_py
        mkdir -p src/talker_py/scripts
        touch src/talker_py/package.xml
        touch src/talker_py/CMakeLists.txt
        touch src/talker_py/src/talker_py/__init__.py
        touch src/talker_py/scripts/talker_py_node
        touch src/talker_py/setup.py

  .. group-tab:: Windows

    .. code-block:: bash

        cd \ros2_talker_py
        md src\talker_py\src\talker_py
        md src\talker_py\scripts
        type nul > src\talker_py\package.xml
        type nul > src\talker_py\CMakeLists.txt
        type nul > src\talker_py\src\talker_py\__init__.py
        type nul > src\talker_py\scripts/talker_py_node
        type nul > src\talker_py\setup.py

Put the following content into each file.

``src/talker_py/package.xml``:

.. code-block:: xml

    <?xml version="1.0"?>
    <?xml-model href="http://download.ros.org/schema/package_format2.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
    <package format="2">
        <name>talker_py</name>
        <version>1.0.0</version>
        <description>The talker_py package</description>
        <maintainer email="gerkey@example.com">Brian Gerkey</maintainer>
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

.. code-block:: Python

    import rospy
    from std_msgs.msg import String

    def main():
        rospy.init_node('talker')
        pub = rospy.Publisher('chatter', String, queue_size=10)
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            hello_str = "hello world %s" % rospy.get_time()
            rospy.loginfo(hello_str)
            pub.publish(hello_str)
            rate.sleep()

``src/talker_py/scripts/talker_py_node``:

.. code-block:: Python

    #!/usr/bin/env python

    import talker_py

    if __name__ == '__main__':
        talker_py.main()

``src/talker_py/setup.py``:

.. code-block:: Python

    from setuptools import setup
    from catkin_pkg.python_setup import generate_distutils_setup

    setup_args = generate_distutils_setup(
        packages=['talker_py'],
        package_dir={'': 'src'}
    )

    setup(**setup_args)

This is the complete ROS 1 Python package.

Migrate the ``package.xml``
---------------------------

When migrating packages to ROS 2, migrate the build system files first so that you can check your work by building and running code as you go.
Always start by migrating your ``package.xml``.

First, ROS 2 does not use ``catkin``.
Delete the ``<buildtool_depend>`` on it.

.. code-block::

    <!-- delete this -->
    <buildtool_depend>catkin</buildtool_depend>


Next, ROS 2 uses ``rclpy`` instead of ``rospy``.
Delete the dependency on ``rospy``.

.. code-block::

    <!-- Delete this -->
    <depend>rospy</depend>


Replace it with a new dependency on ``rclpy``.

.. code-block:: xml

    <depend>rclpy</depend>

Add an ``<export>`` section to tell ROS 2's build tool `Colcon <https://colcon.readthedocs.io/>`__ that this is an ``ament_python`` package instead of a ``catkin`` package.

.. code-block:: xml

     <export>
       <build_type>ament_python</build_type>
     </export>


Your ``package.xml`` is fully migrated.
It should now look like this:

.. code-block:: xml

    <?xml version="1.0"?>
    <?xml-model href="http://download.ros.org/schema/package_format2.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
    <package format="2">
        <name>talker_py</name>
        <version>1.0.0</version>
        <description>The talker_py package</description>
        <maintainer email="gerkey@example.com">Brian Gerkey</maintainer>
        <license>BSD</license>

        <depend>rclpy</depend>
        <depend>std_msgs</depend>

        <export>
            <build_type>ament_python</build_type>
        </export>
    </package>

Delete the ``CMakeLists.txt``
-----------------------------

Python packages in ROS 2 do not use CMake, so delete the ``CMakeLists.txt``.

Migrate the ``setup.py``
------------------------

The arguments to ``setup()`` in the ``setup.py`` can no longer be automatically generated with ``catkin_pkg``.
You must pass these arguments manually, which means there will be some duplication with your ``package.xml``.

Start by deleting the import from ``catkin_pkg``.

.. code-block::

    # Delete this
    from catkin_pkg.python_setup import generate_distutils_setup

Move all arguments given to ``generate_distutils_setup()`` to the call to ``setup()``, and then add the ``install_requires`` and ``zip_safe`` arguments.
Your call to ``setup()`` should  look like this:

.. code-block:: Python

    setup(
        packages=['talker_py'],
        package_dir={'': 'src'},
        install_requires=['setuptools'],
        zip_safe=True,
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
Create a variable called ``package_name`` above the call to ``setup()``.

.. code-block:: Python

    package_name = 'talker_py'

Copy all of the remaining information into the arguments of ``setup()`` in ``setup.py``.
Your call to ``setup()`` should look like this:

.. code-block:: Python

    setup(
        name=package_name,
        version='1.0.0',
        install_requires=['setuptools'],
        zip_safe=True,
        packages=['talker_py'],
        package_dir={'': 'src'},
        maintainer='Brian Gerkey',
        maintainer_email='gerkey@example.com',
        description='The talker_py package',
        license='BSD',
    )


ROS 2 packages must install two data files:

* a ``package.xml``
* a package marker file

Your package already has a ``package.xml``.
It describes your package's dependencies.
A package marker file tells tools like ``ros2 run`` where to find your package.

Create a directory next to the ``package.xml`` called ``resource``.
Create an empty file in the ``resource`` directory with the same name as the package.

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

        mkdir resource
        touch resource/talker_py

  .. group-tab:: macOS

    .. code-block:: bash

        mkdir resource
        touch resource/talker_py

  .. group-tab:: Windows

    .. code-block:: bash

        md resource
        type nul > resource\talker_py

The ``setup()`` call in ``setup.py`` must tell ``setuptools`` how to install these files.
Add the following ``data_files`` argument to the call to ``setup()``.

.. code-block:: Python

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],

Your ``setup.py`` is almost complete.

Migrate Python scripts and create ``setup.cfg``
-----------------------------------------------

ROS 2 Python packages uses ``console_scripts`` `entry points <https://python-packaging.readthedocs.io/en/latest/command-line-scripts.html#the-console-scripts-entry-point>`__ to install Python scripts as executables.
The `configuration file <https://setuptools.pypa.io/en/latest/userguide/declarative_config.html>`__ ``setup.cfg`` tells ``setuptools`` to install those executables in a package specific directory so that tools like ``ros2 run`` can find them.
Create a ``setup.cfg`` file next to the ``package.xml``.

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

        touch setup.cfg

  .. group-tab:: macOS

    .. code-block:: bash

        touch setup.cfg

  .. group-tab:: Windows

    .. code-block:: bash

        type nul > touch setup.cfg

Put the following content into it:

.. code-block::

    [develop]
    script_dir=$base/lib/talker_py
    [install]
    install_scripts=$base/lib/talker_py

You'll need to use the ``console_scripts`` entry point to define the executables to be installed.
Each entry has the format ``executable_name = some.module:function``.
The first part specifies the name of the executable to create.
The second part specifies the function that should be run when the executable starts.
This package needs to create an executable called ``talker_py_node``, and the executable needs to call the function ``main`` in the ``talker_py`` module.
Add the following entry point specification as another argument to ``setup()`` in your ``setup.py``.

.. code-block:: Python

    entry_points={
        'console_scripts': [
            'talker_py_node = talker_py:main',
        ],
    },

The ``talker_py_node`` file is no longer necessary.
Delete the file ``talker_py_node`` and delete the ``scripts/`` directory.

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

        rm scripts/talker_py_node
        rmdir scripts

  .. group-tab:: macOS

    .. code-block:: bash

        rm scripts/talker_py_node
        rmdir scripts

  .. group-tab:: Windows

    .. code-block:: bash

        del scripts/talker_py_node
        rd scripts

The addition of ``console_scripts`` is the last change to your ``setup.py``.
Your final ``setup.py`` should look like this:

.. code-block:: Python

    from setuptools import setup

    package_name = 'talker_py'

    setup(
        name=package_name,
        version='1.0.0',
        packages=['talker_py'],
        package_dir={'': 'src'},
        install_requires=['setuptools'],
        zip_safe=True,
        data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
        ],
        maintainer='Brian Gerkey',
        maintainer_email='gerkey@example.com',
        description='The talker_py package',
        license='BSD',
        entry_points={
            'console_scripts': [
                'talker_py_node = talker_py:main',
            ],
        },
    )

Migrate Python code in ``src/talker_py/__init__.py``
----------------------------------------------------

ROS 2 changed a lot of the best practices for Python code.
Start by migrating the code as-is.
It will be easier to refactor code later after you have something working.

Use ``rclpy`` instead of ``rospy``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

ROS 2 packages use `rclpy <https://index.ros.org/p/rclpy>`__ instead of ``rospy``.
You must do two things to use ``rclpy``:

    1. Import ``rclpy``
    2. Initialize ``rclpy``

Remove the statement that imports ``rospy``.

.. code-block:: Python

    # Remove this
    import rospy

Replace it with a statement that imports ``rclpy``.

.. code-block:: Python

    import rclpy

Add a call to ``rclpy.init()`` as the very first statement in the ``main()`` function.

.. code-block:: Python

    def main():
        # Add this line
        rclpy.init()

Execute callbacks in the background
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Both ROS 1 and ROS 2 use `callbacks <https://en.wikipedia.org/wiki/Callback_(computer_programming)>`__.
In ROS 1, callbacks are always executed in background threads, and users are free to block the main thread with calls like ``rate.sleep()``.
In ROS 2, ``rclpy`` uses :doc:`Executors <../../Concepts/Intermediate/About-Executors>` to give users more control over where callbacks are called.
When porting code that uses blocking calls like ``rate.sleep()``, you must make sure that those calls won't interfere with the executor.
One way to do this is to create a dedicated thread for the executor.

First, add these two import statements.

.. code-block:: Python

    import threading

    from rclpy.executors import ExternalShutdownException

Next, add top-level function called ``spin_in_background()``.
This function asks the default executor to execute callbacks until something shuts it down.

.. code-block:: Python

    def spin_in_background():
        executor = rclpy.get_global_executor()
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

Add the following code in the ``main()`` function just after the call to ``rclpy.init()`` to start a thread that calls ``spin_in_background()``.

.. code-block:: Python

        # In rospy callbacks are always called in background threads.
        # Spin the executor in another thread for similar behavior in ROS 2.
        t = threading.Thread(target=spin_in_background)
        t.start()


Finally, join the thread when the program ends by putting this statement at the bottom of the ``main()`` function.

.. code-block:: Python

        t.join()


Create a node
~~~~~~~~~~~~~

In ROS 1, Python scripts can only create a single node per process, and the API ``init_node()`` creates it.
In ROS 2, a single Python script may create multiple nodes, and the API to create a node is named ``create_node``.

Remove the call to ``rospy.init_node()``:

.. code-block::

    rospy.init_node('talker')

Add a new call to ``rclpy.create_node()`` and store the result in a variable named ``node``:

.. code-block:: Python

    node = rclpy.create_node('talker')

We must tell the executor about this node.
Add the following line just below the creation of the node:

.. code-block:: Python

    rclpy.get_global_executor().add_node(node)

Create a publisher
~~~~~~~~~~~~~~~~~~

In ROS 1, users create publishers by instantiating the ``Publisher`` class.
In ROS 2, users create publishers through a node's ``create_publisher()`` API.
The ``create_publisher()`` API has an unfortunate difference with ROS 1: the topic name and topic type arguments are swapped.

Remove the creation of the ``rospy.Publisher`` instance.

.. code-block::

    pub = rospy.Publisher('chatter', String, queue_size=10)

Replace it with a call to ``node.create_publisher()``.

.. code-block:: Python

    pub = node.create_publisher(String, 'chatter', 10)


Create a rate
~~~~~~~~~~~~~

In ROS 1, users create ``Rate`` instances directly, while in ROS 2 users create them through a node's ``create_rate()`` API.

Remove the creation of the ``rospy.Rate`` instance.

.. code-block::

    rate = rospy.Rate(10)  # 10hz

Replace it with a call to ``node.create_rate()``.

.. code-block:: Python

    rate = node.create_rate(10)  # 10hz

Loop on ``rclpy.ok()``
~~~~~~~~~~~~~~~~~~~~~~

In ROS 1, the ``rospy.is_shutdown()`` API indicates if the process has been asked to shutdown.
In ROS 2, the ``rclpy.ok()`` API does this.

Remove the statement ``not rospy.is_shutdown()``

.. code-block::

    while not rospy.is_shutdown():

Replace it with a call to ``rclpy.ok()``.

.. code-block:: Python

    while rclpy.ok():


Create a ``String`` message with the current time
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You must make a few changes to this line

.. code-block::

    hello_str = "hello world %s" % rospy.get_time()

In ROS 2 you:

* Must get the time from a ``Clock`` instance
* Should format the ``str`` data using `f-strings <https://docs.python.org/3/reference/lexical_analysis.html#f-strings>`__ since  `% is discouraged in active Python versions <https://docs.python.org/3/library/stdtypes.html#printf-style-string-formatting>`__
* Must instantiate a ``std_msgs.msg.String`` instance

Start with getting the time.
ROS 2 nodes have a ``Clock`` instance.
Replace the call to ``rospy.get_time()`` with ``node.get_clock().now()`` to get the current time from the node's clock.

Next, replace the use of ``%`` with an f-string: ``f'hello world {node.get_clock().now()}'``.

Finally, instantiate a ``std_msgs.msg.String()`` instance and assign the above to the ``data`` attribute of that instance.
Your final code should look like this:

.. code-block:: Python

    hello_str = String()
    hello_str.data = f'hello world {node.get_clock().now()}'

Log an informational message
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In ROS 2, you must send log messages through a ``Logger`` instance, and the node has one.

Remove the call to ``rospy.loginfo()``.

.. code-block::

    rospy.loginfo(hello_str)

Replace it with a call to ``info()`` on the node's ``Logger`` instance.

.. code-block:: Python

    node.get_logger().info(hello_str.data)

This is the last change to ``src/talker_py/__init__.py``.
Your file should look like the following:

.. code-block:: Python

    import threading

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from std_msgs.msg import String


    def spin_in_background():
        executor = rclpy.get_global_executor()
        try:
            executor.spin()
        except ExternalShutdownException:
            pass


    def main():
        rclpy.init()
        # In rospy callbacks are always called in background threads.
        # Spin the executor in another thread for similar behavior in ROS 2.
        t = threading.Thread(target=spin_in_background)
        t.start()

        node = rclpy.create_node('talker')
        rclpy.get_global_executor().add_node(node)
        pub = node.create_publisher(String, 'chatter', 10)
        rate = node.create_rate(10)  # 10hz

        while rclpy.ok():
            hello_str = String()
            hello_str.data = f'hello world {node.get_clock().now()}'
            node.get_logger().info(hello_str.data)
            pub.publish(hello_str)
            rate.sleep()

        t.join()


Build and run ``talker_py_node``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Create three terminals:

1. One to build ``talker_py``
2. One to run ``talker_py_node``
3. One to echo the message published by ``talker_py_node``

Build the workspace in the first terminal.

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

        cd ~/ros2_talker_py
        . /opt/ros/{DISTRO}/setup.bash
        colcon build

  .. group-tab:: macOS

    .. code-block:: bash

        cd ~/ros2_talker_py
        . /opt/ros/{DISTRO}/setup.bash
        colcon build

  .. group-tab:: Windows

    .. code-block:: bash

        cd \ros2_talker_py
        call C:\dev\ros2\local_setup.bat
        colcon build

Source your workspace in the second terminal, and run the ``talker_py_node``.

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

        cd ~/ros2_talker_py
        . install/setup.bash
        ros2 run talker_py talker_py_node

  .. group-tab:: macOS

    .. code-block:: bash

        cd ~/ros2_talker_py
        . install/setup.bash
        ros2 run talker_py talker_py_node

  .. group-tab:: Windows

    .. code-block:: bash

        cd \ros2_talker_py
        call install\setup.bat
        ros2 run talker_py talker_py_node

Echo the message published by the node in the third terminal:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

        . /opt/ros/{DISTRO}/setup.bash
        ros2 topic echo /chatter

  .. group-tab:: macOS

    .. code-block:: bash

        . /opt/ros/{DISTRO}/setup.bash
        ros2 topic echo /chatter

  .. group-tab:: Windows

    .. code-block:: bash

        call C:\dev\ros2\local_setup.bat
        ros2 topic echo /chatter


You should see messages with the current time being published in the second terminal, and those same messages received in the third.

Refactor code to use ROS 2 conventions
--------------------------------------

You have successfully migrated a ROS 1 Python package to ROS 2!
Now that you have something working, consider refactoring it to align better with ROS 2's Python APIs.
Follow these two principles.

* Create a class that inherits from ``Node``.
* Do all work in callbacks, and never block those callbacks.

For example, create a ``Talker`` class that inherits from ``Node``.
As for doing work in callbacks, use a ``Timer`` with a callback instead of ``rate.sleep()``.
Make the timer callback publish the message and return.
Make ``main()`` create a ``Talker`` instance rather than using ``rclpy.create_node()``, and give the executor the main thread to execute in.

Your refactored code might look like this:

.. code-block:: Python

    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from std_msgs.msg import String


    class Talker(Node):

        def __init__(self, **kwargs):
            super().__init__('talker', **kwargs)

            self._pub = self.create_publisher(String, 'chatter', 10)
            self._timer = self.create_timer(1 / 10, self.do_publish)

        def do_publish(self):
            hello_str = String()
            hello_str.data = f'hello world {self.get_clock().now()}'
            self.get_logger().info(hello_str.data)
            self._pub.publish(hello_str)


    def main():
        try:
            with rclpy.init():
                rclpy.spin(Talker())
        except (ExternalShutdownException, KeyboardInterrupt):
            pass

Conclusion
----------

You have learned how to migrate an example Python ROS 1 package to ROS 2.
From now on, refer to the :doc:`Migrating Python Packages reference page <./Migrating-Python-Packages>` as you migrate your own Python packages.
