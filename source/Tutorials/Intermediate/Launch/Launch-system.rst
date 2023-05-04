.. redirect-from::

  Tutorials/Launch-system
  Tutorials/Launch-Files/Launch-system
  Tutorials/Launch/Launch-system

Integrating launch files into ROS 2 packages
============================================

**Goal:** Add a launch file to a ROS 2 package

**Tutorial level:** Intermediate

**Time:** 10 minutes

.. contents:: Contents
   :depth: 2
   :local:

Prerequisites
-------------

You should have gone through the tutorial on how to :doc:`create a ROS 2 package <../../Beginner-Client-Libraries/Creating-Your-First-ROS2-Package>`.

As always, don’t forget to source ROS 2 in :doc:`every new terminal you open <../../Beginner-CLI-Tools/Configuring-ROS2-Environment>`.

Background
----------

In the :doc:`previous tutorial <Creating-Launch-Files>`, we saw how to write a standalone launch file.
This tutorial will show how to add a launch file to an existing package, and the conventions typically used.

Tasks
-----

1 Create a package
^^^^^^^^^^^^^^^^^^

Create a workspace for the package to live in:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

      mkdir -p launch_ws/src
      cd launch_ws/src

  .. group-tab:: macOS

    .. code-block:: bash

      mkdir -p launch_ws/src
      cd launch_ws/src

  .. group-tab:: Windows

    .. code-block:: bash

      md launch_ws\src
      cd launch_ws\src

.. tabs::

  .. group-tab:: Python package

    .. code-block:: console

      ros2 pkg create py_launch_example --build-type ament_python

  .. group-tab:: C++ package

    .. code-block:: console

      ros2 pkg create cpp_launch_example --build-type ament_cmake

2 Creating the structure to hold launch files
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

By convention, all launch files for a package are stored in the ``launch`` directory inside of the package.
Make sure to create a ``launch`` directory at the top-level of the package you created above.

.. tabs::

  .. group-tab:: Python package

    For Python packages, the directory containing your package should look like this:

    .. code-block:: console

      src/
        py_launch_example/
          launch/
          package.xml
          py_launch_example/
          resource/
          setup.cfg
          setup.py
          test/

    In order for colcon to find the launch files, we need to inform Python's setup tools of our launch files using the ``data_files`` parameter of ``setup``.

    Inside our ``setup.py`` file:

    .. code-block:: python

      import os
      from glob import glob
      from setuptools import setup

      package_name = 'py_launch_example'

      setup(
          # Other parameters ...
          data_files=[
              # ... Other data files
              # Include all launch files.
              (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
          ]
      )

  .. group-tab:: C++ package

    For C++ packages, we will only be adjusting the ``CMakeLists.txt`` file by adding:

    .. code-block:: cmake

      # Install launch files.
      install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/
      )

    to the end of the file (but before ``ament_package()``).


3 Writing the launch file
^^^^^^^^^^^^^^^^^^^^^^^^^

.. tabs::

  .. group-tab:: Python launch file

    Inside your ``launch`` directory, create a new launch file called ``my_script_launch.py``.
    ``_launch.py`` is recommended, but not required, as the file suffix for Python launch files.
    However, the launch file name needs to end with ``launch.py`` to be recognized and autocompleted by ``ros2 launch``.

    Your launch file should define the ``generate_launch_description()`` function which returns a ``launch.LaunchDescription()`` to be used by the ``ros2 launch`` verb.

    .. code-block:: python

      import launch
      import launch_ros.actions

      def generate_launch_description():
          return launch.LaunchDescription([
              launch_ros.actions.Node(
                  package='demo_nodes_cpp',
                  executable='talker',
                  name='talker'),
        ])

  .. group-tab:: XML launch file

    Inside your ``launch`` directory, create a new launch file called ``my_script_launch.xml``.
    ``_launch.xml`` is recommended, but not required, as the file suffix for XML launch files.

    .. code-block:: xml

      <launch>
        <node pkg="demo_nodes_cpp" exec="talker" name="talker"/>
      </launch>

  .. group-tab:: YAML launch file

    Inside your ``launch`` directory, create a new launch file called ``my_script_launch.yaml``.
    ``_launch.yaml`` is recommended, but not required, as the file suffix for YAML launch files.

    .. code-block:: yaml

      launch:

      - node:
          pkg: "demo_nodes_cpp"
          exec: "talker"
          name: "talker"


4 Building and running the launch file
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Go to the top-level of the workspace, and build it:

.. code-block:: console

  colcon build

After the ``colcon build`` has been successful and you've sourced the workspace, you should be able to run the launch file as follows:

.. tabs::

  .. group-tab:: Python package

    .. tabs::

      .. group-tab:: Python launch file

        .. code-block:: console

          ros2 launch py_launch_example my_script_launch.py

      .. group-tab:: XML launch file

        .. code-block:: console

          ros2 launch py_launch_example my_script_launch.xml

      .. group-tab:: YAML launch file

        .. code-block:: console

          ros2 launch py_launch_example my_script_launch.yaml

  .. group-tab:: C++ package

    .. tabs::

      .. group-tab:: Python launch file

        .. code-block:: console

          ros2 launch cpp_launch_example my_script_launch.py

      .. group-tab:: XML launch file

        .. code-block:: console

          ros2 launch cpp_launch_example my_script_launch.xml

      .. group-tab:: YAML launch file

        .. code-block:: console

          ros2 launch cpp_launch_example my_script_launch.yaml


Documentation
-------------

`The launch documentation <https://github.com/ros2/launch/blob/{REPOS_FILE_BRANCH}/launch/doc/source/architecture.rst>`__ provides more details on concepts that are also used in ``launch_ros``.

Additional documentation/examples of launch capabilities are forthcoming.
See the source code (https://github.com/ros2/launch and https://github.com/ros2/launch_ros) in the meantime.
