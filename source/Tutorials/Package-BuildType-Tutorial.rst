
ROS 2 Package Build-Type Tutorial
=================================

This tutorial demonstrates how to create a custom ROS 2 package build-type plugin. A ROS 2 package build-type indicates to the ROS 2 tooling the type of programming and build technology (e.g., C++ and Python) required by the package's implementation. The package build-type is specified during the package creation process. Thereafter it can be found in the package.xml file of the package root directory. Here's an example package creation command line:

.. code-block::

   ros2 pkg create --build-type <build-type> newPkgName

Beginning with the ROS 2 Foxy Fitzroy release, build-types are implemented as plugins. This design enhancement enables ROS 2 to support build-types beyond the classic ament_cmake, cmake and ament_python build-types.

Build-type plugins are responsible for creating the package directory, the package.xml file and other directories and files specific to each build-type. 
All build-type plugins must implement the small interface defined by ros2pkg.build_type.BuildTypeExtension.

.. code-block:: python

   class BuildTypeExtension():
       """The interface for build-type extensions (plugins)."""

       def create_package(self, args):
           """
           Create the package structure and content.

           :param args: Command Line args wrapped in a package_create_args object.
           """
           raise NotImplementedError

Build-types are "plugged in" to the ros2cli via the ``ros2pkg.build_type`` entry-point. See how the ament_cmake, cmake and ament_python build-types are defined below.

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

Let's walk through the steps for creating a build-type plugin. We will name our build-type **ament_x** and it will create ROS 2 packages structured for a fictitious programming language known as X.

The source code for this tutorial is available `here <https://github.com/wayneparrott/ament_x_buildtype>`_.

1. Create the ROS 2 python package, ament_x
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

We will implement the ament_x build-type as a ROS 2 package and code it in Python. The first step is to create the ament_x package.

From a command shell enter the following commands:

.. code-block::

   ros2 pkg create --build-type ament_python ament_x
   cd ament_x

2. Implement ament_x Build-Type
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

We have two options for how we create the ament_x build-type. 
The first is we can subclass the BuildTypeExtension, an interface, and implement its ``create_package()`` method.
Doing this gives us fine grained control over how the package files' structure and content are created. 
The second option is to extend the BaseBuildType class and customize the parts of its implementation needed for the ament_x build-type. 
BaseBuildType implements a workflow and common package creation functionality.

Both approaches have their merits but for simplicity we will pursue option-2 and subclass our ament_x build-type from BaseBuildType.

In the ament_x subdirectory, create a file named **ament_x.py** with the following content.

.. code-block:: python

   from ros2pkg.api.build_type import BaseBuildType

   class AmentXBuildType(BaseBuildType):

       def __init__(self):
           super(AmentXBuildType, self).__init__()

This initial version of the ament_x build-type uses BaseBuildType's default create_package() functionality to create a package directory and a configured package.xml file. But before we can test it out we have a bit more work.

3. Register ament_x Built-Type
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Next we need to contribute our AmentXBuildType plugin. Open the setup.py file in your editor and change the entry_point as follows:

.. code-block:: python

   entry_points={
     'ros2pkg.build_type': [
         'ament_x = ament_x.ament_x:AmentXBuildType',
     ],
   },

4. Build ament_x Package and Update ROS 2 Environment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

With the plugin created and registered, we need to build it using the ``colcon`` build tool. From your command shell enter the following:

.. code-block:: sh

   cd <ament_x package root directory>
   colcon build

Now let's make the ament_x build-type visible to the ros2 command line tool. Execute the command that best applies to your shell environment. For Linux and Mac users choose one of the following commands:

.. code-block:: sh

   source install/local_setup.bash
   source install/local_setup.sh
   source install/local_setup.zsh

For Windows users use this command:

.. code-block:: sh

   install/local_setup.ps1

5. Test Initial Version of ament_x Build-Type
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

We can confirm that the ros2 cli should now know about our ament_x build type by inspecting the ros package creation command line flags:

.. code-block::

   ros2 pkg create -h

Notice that the --build-type description includes 'ament_x' in the list of available build-types.

.. code-block::

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

Now, let's create a ROS 2 package with the ament_x build-type. Note: the current ament_x build-type implementation doesn't do much yet. It is limited to creation of a new package directory and it's package.xml file.

From the command line, enter the ``ros2 pkg create ...`` command shown below and observe the output.

.. code-block:: sh

   ros2 pkg create --build-type ament_x my_x_project

Output from the package creation process appears below.

.. code-block:: sh

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

Populating The ament_x Package
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A package implemented using our fictitious programming language X requires all code reside in a folder named ``src`` and the initial entry point must be a file named ``main.x``. So let's look at how we can create this src directory and file.

While you could use general Python programming api for this task, we will use utilities from the ros2pkg.create Python module.

5 - Create 'src' Directory
^^^^^^^^^^^^^^^^^^^^^^^^^^

Returning to the ament_x.py file, implement the ``create_source_folders()`` method inside `AmentXBuildType` class as shown below.

.. code-block:: python

   def create_source_folders(self):
     super(AmentXBuildType, self).create_source_folders()

     print('creating source folder')
     self.source_directory = create_folder('src', self.package_directory)
     if not self.source_directory:
       return 'unable to create source folder in ' + self.package_directory

The create_folder() utility function simplifies directory creation. You will also need to add an import for the create_folder() function at the top of the file.

.. code-block:: python

   from ros2pkg.api.create import create_folder

6 - Create main.x Using An EmPy Template File
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

We will use the `EmPy <http://www.alcyone.com/software/empy/>`_ template engine to generate the ``main.x`` file. To do this we need to create a template file for main.x and provide it to the EmPy interpreter along with a set of values for replacing template parameters and the location to output translated content.

Create the file ``main.x.em`` at the path shown below:

.. code-block:: sh

   cd <package_root_dir>/ament_x/
   mkdir -p resource/ament_x
   touch resource/ament_x/main.x.em

Add the following content to main.x.em and save the file. Note that template variables are preceded with a @ character.

.. code-block:: javascript

   main() {
     print('Hi from @package_name.')
   }

*Hang in there we are almost done!*

Add the ``populate()`` method to the `AmentXBuildType` class in `ament_x.py` file. This code translates the main.x.em template file into a new package's 'src/main.x' file.

.. code-block:: python

   def populate(self):
     super(AmentXBuildType, self).populate()

     params = {
       'package_name': self.package.name
     }
     create_template_file(
       'ament_x',
       'ament_x/main.x.em',
       self.source_directory,
       'main.x',
       params)

Our final step is to inform the Python `setuptools <https://pythonhosted.org/an_example_pypi_project/setuptools.html>`_ installation system to copy the ``ament_x/resource/main.x.em`` file.

Add the ``package_data`` snippet to setup.py just below the entry_points.

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

Rebuild ament_x package
^^^^^^^^^^^^^^^^^^^^^^^

We need to rebuild the ament_x package to pick up our recent changes. From the root folder of the package enter the following on the commandline:

.. code-block:: sh

   colcon build

Test ament_x Build-type
^^^^^^^^^^^^^^^^^^^^^^^

To test this last change to the ament_x build-type will recreate the my_x_project from `step-5 <5-create-src-directory>`_. If you haven't done so already remove the former my_x_project directory and its content.

.. code-block::

   cd <path>/my_x_project/..
   rm -rf my_x_project

Now using ros2 cli tools, create a new version of my_x_project.

.. code-block:: sh

   ros2 pkg create --build-type ament_x my_x_project
   cd my_x_project
   ls src

The my_x_project directory should appear as follows:

.. code-block:: sh

   my_x_project/
     package.xml
     src/
       main.x

Congrats! You've successfully created a ROS 2 package build-type that can serve as a working example for more sophisticated custom build-types. The next step is to create a build-type plugin for the colcon build-tool. You can start that journey by getting familiar with the colcon `TaskExtensionPoint <https://github.com/colcon/colcon-core/blob/master/colcon_core/task/__init__.py>`_ and explore implementations such as the python build-type found in the same directory.
