.. _CreatePkg:

Creating your first ROS 2 package
=================================

**Goal:** Create a new package using either CMake or Python, and run its executable.

**Tutorial level:** Beginner

**Time:** 15 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

1 What is a ROS 2 package?
^^^^^^^^^^^^^^^^^^^^^^^^^^

A package can be considered a container for your ROS 2 code.
If you want to be able to install your code or share it with others, then you’ll need it organized in a package.
With packages, you can release your ROS 2 work and allow others to build and use it easily.

Package creation in ROS 2 uses ament as its build system and colcon as its build tool.
You can create a package using either CMake or Python, which are officially supported, though other build types do exist.

2 What makes up a ROS 2 package?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

ROS 2 Python and CMake packages each have their own minimum required contents:

.. tabs::

   .. group-tab:: CMake

      * ``package.xml`` file containing meta information about the package
      * ``CMakeLists.txt`` file that describes how to build the code within the package

   .. group-tab:: Python

      * ``package.xml`` file containing meta information about the package
      * ``setup.py`` containing instructions for how to install the package
      * ``setup.cfg`` is required when a package has executables, so ``ros2 run`` can find them
      * ``/<package_name>`` - a directory with the same name as your package, used by ROS 2 tools to find your package, contains ``__init__.py``

The simplest possible package may have a file structure that looks like:

.. tabs::

   .. group-tab:: CMake

      .. code-block:: console

        my_package/
             CMakeLists.txt
             package.xml

   .. group-tab:: Python

      .. code-block:: console

        my_package/
              setup.py
              package.xml
              resource/my_package


3 Packages in a workspace
^^^^^^^^^^^^^^^^^^^^^^^^^

A single workspace can contain as many packages as you want, each in their own folder.
You can also have packages of different build types in one workspace (CMake, Python, etc.).
You cannot have nested packages.

Best practice is to have a ``src`` folder within your workspace, and to create your packages in there.
This keeps the top level of the workspace “clean”.

A trivial workspace might look like:

.. code-block:: console

  workspace_folder/
      src/
        package_1/
            CMakeLists.txt
            package.xml

        package_2/
            setup.py
            package.xml
            resource/package_2
        ...
        package_n/
            CMakeLists.txt
            package.xml


Prerequisites
-------------

You should have a ROS 2 workspace after following the instructions in the :ref:`previous tutorial <ROS2Workspace>`.
You will create your package in this workspace.


Tasks
-----

1 Create a package
^^^^^^^^^^^^^^^^^^

First, :ref:`source your ROS 2 installation <ConfigROS2>`.

Let’s use the workspace you created in the previous tutorial, ``dev_ws``, for your new package.`

Make sure you are in the ``src`` folder before running the package creation command.

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        cd ~/dev_ws/src

   .. group-tab:: macOS

     .. code-block:: console

       cd ~/dev_ws/src

   .. group-tab:: Windows

     .. code-block:: console

       cd \dev_ws\src

The command syntax for creating a new package in ROS 2 is:

.. tabs::

   .. group-tab:: CMake

      .. code-block:: console

        ros2 pkg create --build-type ament_cmake <package_name>

   .. group-tab:: Python

      .. code-block:: console

        ros2 pkg create --build-type ament_python <package_name>

For this tutorial, you will use the optional argument ``--node-name`` which creates a simple Hello World type executable in the package.

Enter the following command in your terminal:

.. tabs::

   .. group-tab:: CMake

      .. code-block:: console

        ros2 pkg create --build-type ament_cmake --node-name my_node my_package

   .. group-tab:: Python

      .. code-block:: console

        ros2 pkg create --build-type ament_python --node-name my_node my_package

You will now have a new folder within your workspace’s ``src`` directory called ``my_package``.

After running the command, your terminal will return the message:

.. tabs::

   .. group-tab:: CMake

      .. code-block:: console

        going to create a new package
        package name: my_package
        destination directory: /home/user/dev_ws/src
        package format: 3
        version: 0.0.0
        description: TODO: Package description
        maintainer: ['<name> <email>']
        licenses: ['TODO: License declaration']
        build type: ament_cmake
        dependencies: []
        node_name: my_node
        creating folder ./my_package
        creating ./my_package/package.xml
        creating source and include folder
        creating folder ./my_package/src
        creating folder ./my_package/include/my_package
        creating ./my_package/CMakeLists.txt
        creating ./my_package/src/my_node.cpp

   .. group-tab:: Python

      .. code-block:: console

        going to create a new package
        package name: my_package
        destination directory: /home/user/dev_ws/src
        package format: 3
        version: 0.0.0
        description: TODO: Package description
        maintainer: ['<name> <email>']
        licenses: ['TODO: License declaration']
        build type: ament_python
        dependencies: []
        node_name: my_node
        creating folder ./my_package
        creating ./my_package/package.xml
        creating source folder
        creating folder ./my_package/my_package
        creating ./my_package/setup.py
        creating ./my_package/setup.cfg
        creating folder ./my_package/resource
        creating ./my_package/resource/my_package
        creating ./my_package/my_package/__init__.py
        creating folder ./my_package/test
        creating ./my_package/test/test_copyright.py
        creating ./my_package/test/test_flake8.py
        creating ./my_package/test/test_pep257.py
        creating ./my_package/my_package/my_node.py

You can see the automatically generated files for the new package.

2 Build a package
^^^^^^^^^^^^^^^^^

Putting packages in a workspace is especially valuable because you can build many packages at once by running ``colcon build`` in the workspace root.
Otherwise, you would have to build each package individually.

Return to the root of your workspace:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        cd ~/dev_ws

   .. group-tab:: macOS

      .. code-block:: console

        cd ~/dev_ws

   .. group-tab:: Windows

     .. code-block:: console

       cd \dev_ws

Now you can build your packages:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      colcon build

  .. group-tab:: macOS

    .. code-block:: console

      colcon build

  .. group-tab:: Windows

    .. code-block:: console

      colcon build --merge-install

    Windows doesn’t allow long paths, so ``merge-install`` will combine all the paths into the ``install`` directory.

Recall from the last tutorial that you also have the ``ros_tutorials`` packages in your ``dev_ws``.
You might’ve noticed that running ``colcon build`` also built the ``turtlesim`` package.
That’s fine when you only have a few packages in your workspace, but when there are many packages, ``colcon build`` can take a long time.

To build only the ``my_package`` package next time, you can run:

.. code-block:: console

    colcon build --packages-select my_package

3 Source the setup file
^^^^^^^^^^^^^^^^^^^^^^^

To use your new package and executable, first open a new terminal and source your main ROS 2 installation.

Then, from inside the ``dev_ws`` directory, run the following command to source your workspace:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      . install/setup.bash

  .. group-tab:: macOS

    .. code-block:: console

      . install/setup.bash

  .. group-tab:: Windows

    .. code-block:: console

      call install/local_setup.bat

Now that your workspace has been added to your path, you will be able to use your new package’s executables.

4 Use the package
^^^^^^^^^^^^^^^^^

To run the executable you created using the ``--node-name`` argument during package creation, enter the command:

.. code-block:: console

  ros2 run my_package my_node

Which will return a message to your terminal:

.. tabs::

   .. group-tab:: CMake

      .. code-block:: console

        hello world my_package package

   .. group-tab:: Python

      .. code-block:: console

        Hi from my_package.

5 Examine package contents
^^^^^^^^^^^^^^^^^^^^^^^^^^

Inside ``dev_ws/src/my_package``, you will see the files and folders that ``ros2 pkg create`` automatically generated:

.. tabs::

   .. group-tab:: CMake

      .. code-block:: console

        CMakeLists.txt  include  package.xml  src

      ``my_node.cpp`` is inside the ``src`` directory.
      This is where all your custom C++ nodes will go in the future.

   .. group-tab:: Python

      .. code-block:: console

        my_package  package.xml  resource  setup.cfg  setup.py  test

      ``my_node.py`` is inside the ``my_package`` directory.
      This is where all your custom Python nodes will go in the future.

6 Customize package.xml
^^^^^^^^^^^^^^^^^^^^^^^

You may have noticed in the return message after creating your package that the fields ``description`` and ``license`` contain ``TODO`` notes.
That’s because the package description and license declaration are not automatically set, but are required if you ever want to release your package.
The ``maintainer`` field may also need to be filled in.

From ``dev_ws/src/my_package``, open ``package.xml`` using your preferred text editor:

.. tabs::

   .. group-tab:: CMake

    .. code-block:: xml
     :linenos:

     <?xml version="1.0"?>
     <?xml-model
        href="http://download.ros.org/schema/package_format3.xsd"
        schematypens="http://www.w3.org/2001/XMLSchema"?>
     <package format="3">
      <name>my_package</name>
      <version>0.0.0</version>
      <description>TODO: Package description</description>
      <maintainer email="user@todo.todo">user</maintainer>
      <license>TODO: License declaration</license>

      <buildtool_depend>ament_cmake</buildtool_depend>

      <test_depend>ament_lint_auto</test_depend>
      <test_depend>ament_lint_common</test_depend>

      <export>
        <build_type>ament_cmake</build_type>
      </export>
     </package>

   .. group-tab:: Python

    .. code-block:: xml
     :linenos:

     <?xml version="1.0"?>
     <?xml-model
        href="http://download.ros.org/schema/package_format3.xsd"
        schematypens="http://www.w3.org/2001/XMLSchema"?>
     <package format="3">
      <name>my_package</name>
      <version>0.0.0</version>
      <description>TODO: Package description</description>
      <maintainer email="user@todo.todo">user</maintainer>
      <license>TODO: License declaration</license>

      <buildtool_depend>ament_python</buildtool_depend>

      <test_depend>ament_copyright</test_depend>
      <test_depend>ament_flake8</test_depend>
      <test_depend>ament_pep257</test_depend>
      <test_depend>python3-pytest</test_depend>

      <export>
        <build_type>ament_python</build_type>
      </export>
     </package>

Input your name and email on line 7 if it hasn't been automatically populated for you.
Then, edit the description on line 6 to summarize the package:

.. code-block:: xml

  <description>Beginner client libraries tutorials practice package</description>

Then, update the license on line 8.
You can read more about open source licenses `here <https://opensource.org/licenses/alphabetical>`__.

Since this package is only for practice, it’s safe to use any license. We use ``Apache License 2.0``:

.. code-block:: xml

  <license>Apache License 2.0</license>

Don’t forget to save once you’re done editing.

Below the license tag, you will see some tag names ending with ``_depend``.
This is where your ``package.xml`` would list its dependencies on other packages, for colcon to search for.
``my_package`` is simple and doesn’t have any dependencies, but you will see this space being utilized in upcoming tutorials.

.. tabs::

   .. group-tab:: CMake

      You’re all done for now!

   .. group-tab:: Python

      The ``setup.py`` file contains the same description, maintainer and license fields as ``package.xml``, so you need to set those as well.
      They need to match exactly in both files.
      The version and name (``package_name``) also need to match exactly, and should be automatically populated in both files.

      Open ``setup.py`` with your preferred text editor.

      .. code-block:: python
       :linenos:

       from setuptools import setup

       package_name = 'my_py_pkg'

       setup(
        name=package_name,
        version='0.0.0',
        packages=[package_name],
        data_files=[
            ('share/ament_index/resource_index/packages',
                    ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
          ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='TODO',
        maintainer_email='TODO',
        description='TODO: Package description',
        license='TODO: License declaration',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                    'my_node = my_py_pkg.my_node:main'
            ],
          },
       )

      Edit lines 16-19 to match ``package.xml``.

      Don’t forget to save the file.


Summary
-------

You’ve created a package to organize your code and make it easy to use for others.

Your package was automatically populated with the necessary files, and then you used colcon to build it so you can use its executables in your local environment.

Next steps
----------

Next, let's add something meaningful to a package.
You'll start with a simple publisher/subscriber system, which you can choose to write in either :ref:`C++ <CppPubSub>` or :ref:`Python <PyPubSub>`.
