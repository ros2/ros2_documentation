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

As always, don't forget to source ROS 2 in :doc:`every new terminal you open <../../Beginner-CLI-Tools/Configuring-ROS2-Environment>`.

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

    .. code-block:: console

      $ mkdir -p launch_ws/src
      $ cd launch_ws/src

  .. group-tab:: macOS

    .. code-block:: console

      $ mkdir -p launch_ws/src
      $ cd launch_ws/src

  .. group-tab:: Windows

    .. code-block:: console

      $ md launch_ws\src
      $ cd launch_ws\src

.. tabs::

  .. group-tab:: Python package

    .. code-block:: console

      $ ros2 pkg create --build-type ament_python --license Apache-2.0 py_launch_example

  .. group-tab:: C++ package

    .. code-block:: console

      $ ros2 pkg create --build-type ament_cmake --license Apache-2.0 cpp_launch_example

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

    To enable colcon to locate and utilize our launch files, we need to inform Python's setup tools of their presence.
    To achieve this, open the ``setup.py`` file, add the necessary ``import`` statements at the top, and include the launch files into the ``data_files`` parameter of ``setup``:

    .. code-block:: python

      import os
      from glob import glob
      # Other imports ...

      package_name = 'py_launch_example'

      setup(
          # Other parameters ...
          data_files=[
              # ... Other data files
              # Include all launch files.
              (os.path.join('share', package_name, 'launch'), glob('launch/*'))
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

  .. group-tab:: XML launch file

    Inside your ``launch`` directory, create a new launch file called ``my_script_launch.xml``.
    ``_launch.xml`` is recommended, but not required, as the file suffix for XML launch files.

    .. literalinclude:: launch/my_script_launch.xml
      :language: xml

  .. group-tab:: YAML launch file

    Inside your ``launch`` directory, create a new launch file called ``my_script_launch.yaml``.
    ``_launch.yaml`` is recommended, but not required, as the file suffix for YAML launch files.

    .. literalinclude:: launch/my_script_launch.yaml
      :language: yaml

  .. group-tab:: Python launch file

    Inside your ``launch`` directory, create a new launch file called ``my_script_launch.py``.
    ``_launch.py`` is recommended, but not required, as the file suffix for Python launch files.
    However, the launch file name needs to end with ``launch.py`` to be recognized and autocompleted by ``ros2 launch``.

    Your launch file should define the ``generate_launch_description()`` function which returns a ``launch.LaunchDescription()`` to be used by the ``ros2 launch`` verb.

    .. literalinclude:: launch/my_script_launch.py
      :language: python


4 Building and running the launch file
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Go to the top-level of the workspace, and build it:

.. code-block:: console

  $ colcon build

After the ``colcon build`` has been successful and you've sourced the workspace, you should be able to run the launch file as follows:

.. tabs::

  .. group-tab:: Python package

    .. tabs::

      .. group-tab:: XML launch file

        .. code-block:: console

          $ ros2 launch py_launch_example my_script_launch.xml

      .. group-tab:: YAML launch file

        .. code-block:: console

          $ ros2 launch py_launch_example my_script_launch.yaml

      .. group-tab:: Python launch file

        .. code-block:: console

          $ ros2 launch py_launch_example my_script_launch.py

  .. group-tab:: C++ package

    .. tabs::

      .. group-tab:: XML launch file

        .. code-block:: console

          $ ros2 launch cpp_launch_example my_script_launch.xml

      .. group-tab:: YAML launch file

        .. code-block:: console

          $ ros2 launch cpp_launch_example my_script_launch.yaml

      .. group-tab:: Python launch file

        .. code-block:: console

          $ ros2 launch cpp_launch_example my_script_launch.py


Documentation
-------------

`The launch documentation <https://docs.ros.org/en/{DISTRO}/p/launch/architecture.html>`__ provides more details on concepts that are also used in ``launch_ros``.

Additional documentation/examples of launch capabilities are forthcoming.
See the source code (https://github.com/ros2/launch and https://github.com/ros2/launch_ros) in the meantime.
