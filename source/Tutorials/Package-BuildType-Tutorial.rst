
ROS 2 Package Build-Type Tutorial
=================================

This tutorial demonstrates how to create a custom ROS 2 package build-type plugin. You may already be familiar with package build-types as the ROS 2 distributions have supported creation and building of packages with the build-types: ``ament_cmake``, ``ament_python`` and ``cmake``. Beginning with the ROS 2 Foxy Fitzroy release, ROS 2 can be extended to support additional build-types developed by the community for popular programming languages such as Java, JavaScript, and Go.  

Before we begin creating our custom ROS 2 package build-type, let's learn about the role and responsibilities of a build-type.  A ROS 2 package build-type indicates to the ROS 2 tooling the type of programming model and build technology (e.g., C++ and Python) required by a package's implementation. The package build-type is specified during the package creation process as shown below. Thereafter it can be found in the package.xml file of the package root directory.

.. code-block:: console

   ros2 pkg create --build-type <build-type> newPkgName

A build-type plugin is responsible for creating the standard ROS 2 package directory and package.xml file as well as additional directories and files specific to the build-type. The plugin must implement the small interface defined by ``ros2pkg.build_type.BuildTypeExtension``.

.. code-block:: python

   class BuildTypeExtension():
       """The interface for build-type extensions (plugins)."""

       def create_package(self, args):
           """
           Create the package structure and content.

           :param args: Command Line args wrapped in a package_create_args object.
           """
           raise NotImplementedError

Build-types are "plugged in" to the ros2cli via the ``ros2pkg.build_type`` entry-point. Below is the code for the standard ROS 2 build-type plugins.

.. code-block:: python

   entry_points={
     'ros2pkg.build_type': [
       'ament_cmake = ros2pkg.build_type.ament_cmake:AmentCMakeBuildType',
       'ament_python = ros2pkg.build_type.ament_python:AmentPythonBuildType',
       'cmake = ros2pkg.build_type.cmake:CMakeBuildType',
     ],
   },...

Creating a Custom Build-Type
----------------------------

Let's walk through the steps for creating a build-type plugin. We will name our build-type ``ament_x`` and it will create a ROS 2 package structured for a fictitious programming language known as ``X``.

The source code for this tutorial is available `here <https://github.com/wayneparrott/ament_x_buildtype>`_.

1. Create the ROS 2 python package, ament_x
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

We will implement the ``ament_x`` build-type as a ROS 2 package and code it in Python. The first step is to create the ``ament_x`` package.

From a command shell enter the following commands:

.. code-block:: console

   ros2 pkg create --build-type ament_python --destination-directory ament_x_buildtype ament_x
   cd ament_x_buildtype

2. Implement ament_x Build-Type
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

We have two options for how we create the ``ament_x`` build-type. 

1. This option involves subclassing ``ros2pkg.api.build_type.BuildTypeExtension``, an interface, and implementing its ``create_package()`` method. The benefit of this approach is that it gives us fine control over how the package files' structure and content are created. But it can be tedious to implement as it requires us to manage every aspect of the package creation process.
2. This option involves extending the ``ros2pkg.api.build_type.BaseBuildType`` class and customizing the parts of its implementation needed for our ``ament_x`` build-type. ``BaseBuildType`` is a base class that implements a simple package creation workflow and common package creation functionality. This approach trades off maximum flexibility for implementation simplicity and convenience.

While both approaches have their merits, this tutorial will pursue option 2 and subclass our ``ament_x`` build-type from ``BaseBuildType``.

In the ``ament_x_buildtype/ament_x/ament_x/`` directory, create a file named ``ament_x.py`` with the following content. Note that the imported ``create_file()`` and ``create_folder()`` functions will be used at a later step in this tutorial.

.. code-block:: python

   from ros2pkg.api.build_type import BaseBuildType
   from ros2pkg.api.create import create_file
   from ros2pkg.api.create import create_folder

   class AmentXBuildType(BaseBuildType):

       def __init__(self):
           super(AmentXBuildType, self).__init__()

This initial version of the ``ament_x`` build-type uses ``BaseBuildType``'s default ``create_package()`` functionality to create a package directory and a configured package.xml file. But before we can test it out we have a bit more work.

3. Register ament_x Built-Type
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Next we need to contribute our ``AmentXBuildType`` plugin. Open the ``setup.py`` file in your editor and change the entry_point as follows:

.. code-block:: python

   entry_points={
     'ros2pkg.build_type': [
         'ament_x = ament_x.ament_x:AmentXBuildType',
     ],
   },

4. Build ament_x Package and Update ROS 2 Environment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

With the plugin created and registered, we need to build it using the ROS 2 ``colcon`` build tool. From your command shell enter the following:

.. code-block:: console

   cd <ament_x_buildtype directory>
   colcon build

Now let's make the ``ament_x`` build-type visible to the ros2 command line tool. Execute the command that best applies to your shell environment.

.. tags::

  .. group-tag:: Linux & Mac OSX

    .. code-block:: bash

      source install/local_setup.bash
      source install/local_setup.sh
      source install/local_setup.zsh

  .. group-tag:: Windows

    .. code-block:: bash

      install\local_setup.ps1

5. Test Initial Version of ament_x Build-Type
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Let's confirm that the ros2 cli recognizes our ``ament_x`` build-type by running the following command:

.. code-block:: console

   ros2 pkg create -h

Notice in the console output below that the ``--build-type`` description includes ``ament_x`` in the list of available build-types.

.. code-block:: console

   ros2 pkg create -h

   usage: ros2 pkg create [-h] [--package-format {2,3}]
                          [--description DESCRIPTION] [--license LICENSE]
                          [--destination-directory DESTINATION_DIRECTORY]
                    >>>>  [--build-type {ament_cmake,ament_python,ament_x,cmake}]
                          [--dependencies DEPENDENCIES [DEPENDENCIES ...]]
                          [--maintainer-email MAINTAINER_EMAIL]
                          [--maintainer-name MAINTAINER_NAME]
                          [--node-name NODE_NAME] [--library-name LIBRARY_NAME]
                          package_name

Now, let's create a ROS 2 package with the ``ament_x`` build-type. Note: the current ``ament_x`` build-type implementation doesn't do much yet. It is limited to creation of a new package directory and it's package.xml file.

From the command line, enter the ``ros2 pkg create ...`` command shown below and observe the output.

.. code-block:: console

   ros2 pkg create --build-type ament_x my_x_project

Output from the package creation process appears below.

.. code-block:: console

   ros2 pkg create --build-type ament_x my_x_project
   create package
     package name: my_x_package
     destination directory: /dev
     package format: 3
     version: 0.0.0
     description: TODO: Package description
     maintainer: ['']
     licenses: ['TODO: License declaration']
     build type: ament_x
     dependencies: []
   creating folder ./my_x_package
   creating ./my_x_package/package.xml

The ``my_x_project/package.xml`` will look similar to this:

.. code-block:: xml

   <?xml version="1.0"?>
   <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/200
   1/XMLSchema"?>
   <package format="3">
     <name>my_x_package</name>
     <version>0.0.0</version>
     <description>TODO: Package description</description>
     <maintainer email="nobody@nowhere.com">nobody</maintainer>
     <license>TODO: License declaration</license>

     <buildtool_depend>ament_x</buildtool_depend>

     <export>
       <build_type>ament_x</build_type>
     </export>
   </package>

YOU'RE DOING MARVELOUS!

6. Populating the ament_x Package
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A ROS 2 package implemented using our fictitious programming language ``X`` requires all code reside in a folder named ``src``. Additionally, the initial entry point for execution of the package must be a file named ``main.x``. So let's look at how we can create a ``src`` directory and file.

While you could use general Python programming api for this task, we will use utilities from the ``ros2pkg.create`` Python module.

7. Create src Directory
^^^^^^^^^^^^^^^^^^^^^^^

Returning to the ``ament_x.py`` file, implement the ``create_source_folders()`` method inside the ``AmentXBuildType`` class as shown below. The ``create_folder()`` utility function simplifies directory creation.

.. code-block:: python

   def create_source_folders(self):
     super(AmentXBuildType, self).create_source_folders()

     print('creating source folder')
     self.source_directory = create_folder('src', self.package_directory)
     if not self.source_directory:
       return 'unable to create source folder in ' + self.package_directory

8. Create main.x Using an EmPy Template File
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

We will use the `EmPy <http://www.alcyone.com/software/empy/>`_ template engine to generate the ``main.x`` file. To do this we need to create a template file for ``main.x`` and provide it to the EmPy interpreter along with a set of values for replacing template parameters and the file path where intepreter output should be written.

Create the file ``main.x.em`` as shown below:

.. tags::

  .. group-tag:: Linux & Mac OSX

    .. code-block:: console

      cd ament_x_buildtype/ament_x/ament_x
      mkdir -p resource/ament_x
      touch resource/ament_x/main.x.em

  .. group-tag:: Windows

    .. code-block:: console

      cd ament_x_buildtype\ament_x\ament_x
      mkdir resource\ament_x
      type nul > resource\ament_x\main.x.em

Add the following content to ``main.x.em`` and save the file. Note that template variables are preceded with a ``@`` character.

.. code-block:: javascript

   main() {
     print('Hi from @package_name.')
   }

*Hang in there we are almost done!*

Add the ``populate()`` method to the ``AmentXBuildType`` class in the ``ament_x.py`` file. This code translates the ``main.x.em`` template file into a new package's ``src/main.x`` file.

.. code-block:: python

   def populate(self):
     super(AmentXBuildType, self).populate()

     params = {
       'package_name': self.package.name
     }
     create_file(
       'ament_x',
       'ament_x/main.x.em',
       self.source_directory,
       'main.x',
       params)

Our final step is to inform the Python `setuptools <https://pythonhosted.org/an_example_pypi_project/setuptools.html>`_ installation system to copy the ``ament_x_buildtype/ament_x/ament_x/resource/main.x.em`` file.

Add the ``package_data`` snippet to ``setup.py`` just below the entry_points.

.. code-block::

   entry_points={
     'console_scripts': [
     ],
     'ros2pkg.build_type': [
       'ament_x = ament_x.ament_x:AmentXBuildType',
     ],
   },
   package_data={
     'ament_x': [
       'resource/**/*',
     ],
   },

9. Rebuild ament_x package
^^^^^^^^^^^^^^^^^^^^^^^^^^

We need to rebuild the ``ament_x`` package to pick up our recent changes. From the ``ament_x_buildtype`` directory enter the following on the command line:

.. code-block:: console

   colcon build

10. Test ament_x Build-type
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Our last step is to test our latest changes by recreating the ``my_x_project`` from step `7. Create src directory`_. If you haven't done so already remove the former ``my_x_project`` directory and its content.

.. tags::

  .. group-tag:: Linux & Mac OSX

    .. code-block:: console

      cd <path>/my_x_project/..
      rm -rf my_x_project

  .. group-tag:: Windows

    .. code-block:: console

      cd <path>\my_x_project/..
      rmdir /S my_x_project

Now using ros2 cli tool, create a new version of ``my_x_project``.

.. code-block:: console

   ros2 pkg create --build-type ament_x my_x_project

The ``my_x_project`` directory should appear as follows:

.. code-block:: console

   my_x_project/
     package.xml
     src/
       main.x

Congrats! 
You've successfully created a ROS 2 package build-type. The ament_x build-type can serve as a working example for creating more sophisticated custom build-types. 
The next step is to create a build-type plugin for the colcon build-tool. 
You can start that journey by getting familiar with the colcon `TaskExtensionPoint <https://github.com/colcon/colcon-core/blob/master/colcon_core/task/__init__.py>`_ and explore implementations such as the Python build-type found in the same directory.
