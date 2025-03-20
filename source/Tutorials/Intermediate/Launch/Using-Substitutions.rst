.. redirect-from::

    Tutorials/Launch-Files/Using-Substitutions
    Tutorials/Launch/Using-Substitutions

Using substitutions
===================

**Goal:** Learn about substitutions in ROS 2 launch files.

**Tutorial level:** Intermediate

**Time:** 15 minutes

.. contents:: Table of Contents
   :depth: 2
   :local:

Background
----------

Launch files are used to start nodes, services and execute processes.
This set of actions may have arguments, which affect their behavior.
Substitutions can be used in arguments to provide more flexibility when describing reusable launch files.
Substitutions are variables that are only evaluated during execution of the launch description and can be used to acquire specific information like a launch configuration, an environment variable, or to evaluate an arbitrary Python expression.

This tutorial shows usage examples of substitutions in ROS 2 launch files.

Prerequisites
-------------

This tutorial uses the :doc:`turtlesim <../../Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim>` package.
This tutorial also assumes you are familiar with :doc:`creating packages <../../Beginner-Client-Libraries/Creating-Your-First-ROS2-Package>`.

As always, don't forget to source ROS 2 in :doc:`every new terminal you open <../../Beginner-CLI-Tools/Configuring-ROS2-Environment>`.

Using substitutions
-------------------

1 Create and setup the package
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

First, create a new package with the name ``launch_tutorial``:

.. tabs::

  .. group-tab:: Python package

    Create a new package of build_type ``ament_python``:

    .. code-block:: console

      ros2 pkg create --build-type ament_python --license Apache-2.0 launch_tutorial

  .. group-tab:: C++ package

    Create a new package of build_type ``ament_cmake``:

    .. code-block:: console

      ros2 pkg create --build-type ament_cmake --license Apache-2.0 launch_tutorial

Inside of that package, create a directory called ``launch``:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

      mkdir launch_tutorial/launch

  .. group-tab:: macOS

    .. code-block:: bash

      mkdir launch_tutorial/launch

  .. group-tab:: Windows

    .. code-block:: bash

      md launch_tutorial/launch

Finally, make sure to install the launch files:

.. tabs::

  .. group-tab:: Python package

    Add in following changes to the ``setup.py`` of the package:

    .. code-block:: python

      import os
      from glob import glob
      from setuptools import find_packages, setup

      package_name = 'launch_tutorial'

      setup(
          # Other parameters ...
          data_files=[
              # ... Other data files
              # Include all launch files.
              (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
          ]
      )

  .. group-tab:: C++ package

    Append following code to the ``CMakeLists.txt`` just before ``ament_package()``:

    .. code-block:: cmake

      install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
      )



2 Parent launch file
^^^^^^^^^^^^^^^^^^^^

Let's create a launch file that will call and pass arguments to another launch file.
This launch file can either be in YAML, XML, or in Python.

To do this, create following file in the ``launch`` folder of the ``launch_tutorial`` package.

Copy and paste the complete code into the ``launch/example_main.launch`` file:

.. tabs::

  .. group-tab:: YAML

    .. literalinclude:: code/example_main_yaml.launch
      :language: yaml

  .. group-tab:: XML

    .. literalinclude:: code/example_main_xml.launch
      :language: xml

  .. group-tab:: Python

    .. literalinclude:: code/example_main_py.launch
      :language: python

Step-by-step explanation:

.. tabs::

  .. group-tab:: YAML

    The ``$(find-pkg-share launch_tutorial)`` substitution is used to find the path to the ``launch_tutorial`` package.
    The path substitution is then joined with the ``example_substitutions.launch`` file name.

    .. literalinclude:: code/example_main_yaml.launch
      :language: yaml
      :lines: 9

    The ``background_r`` variable with ``turtlesim_ns`` and ``use_provided_red`` arguments is passed to the ``include`` action.
    The ``$(var background_r)`` substitution is used to define the ``new_background_r`` argument with the value of the ``background_r`` variable.

    .. literalinclude:: code/example_main_yaml.launch
      :lines: 10-16

  .. group-tab:: XML

    The ``$(find-pkg-share launch_tutorial)`` substitution is used to find the path to the ``launch_tutorial`` package.
    The path substitution is then joined with the ``example_substitutions.launch`` file name.

    .. literalinclude:: code/example_main_xml.launch
      :language: xml
      :lines: 4

    The ``background_r`` variable with ``turtlesim_ns`` and ``use_provided_red`` arguments is passed to the ``include`` action.
    The ``$(var background_r)`` substitution is used to define the ``new_background_r`` argument with the value of the ``background_r`` variable.

    .. literalinclude:: code/example_main_xml.launch
      :language: xml
      :lines: 5-7

  .. group-tab:: Python

    The ``FindPackageShare`` substitution is used to find the path to the ``launch_tutorial`` package.
    The ``PathJoinSubstitution`` substitution is then used to join the path to that package path with the ``example_substitutions.launch`` file name.

    .. literalinclude:: code/example_main_py.launch
      :language: python
      :lines: 18-22

    The ``launch_arguments`` dictionary with ``turtlesim_ns`` and ``use_provided_red`` arguments is passed to the ``IncludeLaunchDescription`` action.
    The ``TextSubstitution`` substitution is used to define the ``new_background_r`` argument with the value of the ``background_r`` key in the ``colors`` dictionary.

    .. literalinclude:: code/example_main_py.launch
      :language: python
      :lines: 24-28

3 Substitutions example launch file
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Now create the substitution launch file ``launch/example_substitutions.launch`` in the same folder:

.. tabs::

  .. group-tab:: YAML

    .. literalinclude:: code/example_substitutions_yaml.launch
      :language: yaml

  .. group-tab:: XML

    .. literalinclude:: code/example_substitutions_xml.launch
      :language: xml

  .. group-tab:: Python

    .. literalinclude:: code/example_substitutions_py.launch
      :language: python

Step-by-step explanation:

.. tabs::

  .. group-tab:: YAML

    The ``turtlesim_ns``, ``use_provided_red``, and ``new_background_r`` launch configurations are defined.
    They are used to store values of launch arguments in the above variables and to pass them to required actions.
    The launch configuration arguments can later be used with the ``$(var <name>)`` substitution to acquire the value of the launch argument in any part of the launch description.

    The ``arg`` tag is used to define the launch argument that can be passed from the above launch file or from the console.

    .. literalinclude:: code/example_substitutions_yaml.launch
      :language: yaml
      :lines: 4-12

    The ``turtlesim_node`` node with the ``namespace`` set to the ``turtlesim_ns`` launch configuration value using the ``$(var <name>)`` substitution is defined.

    .. literalinclude:: code/example_substitutions_yaml.launch
      :language: yaml
      :lines: 14-18

    Afterwards, an ``executable`` action is defined with the corresponding ``cmd`` tag.
    This command makes a call to the spawn service of the turtlesim node.

    Additionally, the ``$(var <name>)`` substitution is used to get the value of the ``turtlesim_ns`` launch argument to construct a command string.

    .. literalinclude:: code/example_substitutions_yaml.launch
      :language: yaml
      :lines: 19-20

    The same approach is used for the ``ros2 param`` ``executable`` actions that change the turtlesim background's red color parameter.
    The difference is that the second action inside of the timer is only executed if the provided ``new_background_r`` argument equals ``200`` and the ``use_provided_red`` launch argument is set to ``True``.
    The evaluation of the ``if`` predicate is done using the ``$(eval <python-expression>)`` substitution.

    .. literalinclude:: code/example_substitutions_yaml.launch
      :language: yaml
      :lines: 21-

  .. group-tab:: XML

    The ``turtlesim_ns``, ``use_provided_red``, and ``new_background_r`` launch configurations are defined.
    They are used to store values of launch arguments in the above variables and to pass them to required actions.
    The launch configuration arguments can later be used with the ``$(var <name>)`` substitution to acquire the value of the launch argument in any part of the launch description.

    The ``arg`` tag is used to define the launch argument that can be passed from the above launch file or from the console.

    .. literalinclude:: code/example_substitutions_xml.launch
      :language: xml
      :lines: 3-5

    The ``turtlesim_node`` node with the ``namespace`` set to the ``turtlesim_ns`` launch configuration value using the ``$(var <name>)`` substitution is defined.

    .. literalinclude:: code/example_substitutions_xml.launch
      :language: xml
      :lines: 7

    Afterwards, an ``executable`` action is defined with the corresponding ``cmd`` tag.
    This command makes a call to the spawn service of the turtlesim node.

    Additionally, the ``$(var <name>)`` substitution is used to get the value of the ``turtlesim_ns`` launch argument to construct a command string.

    .. literalinclude:: code/example_substitutions_xml.launch
      :language: xml
      :lines: 8

    The same approach is used for the ``ros2 param`` ``executable`` actions that change the turtlesim background's red color parameter.
    The difference is that the second action inside of the timer is only executed if the provided ``new_background_r`` argument equals ``200`` and the ``use_provided_red`` launch argument is set to ``True``.
    The evaluation of the ``if`` predicate is done using the ``$(eval <python-expression>)`` substitution.

    .. literalinclude:: code/example_substitutions_xml.launch
      :language: xml
      :lines: 9-15

  .. group-tab:: Python

    First we provide variables with our argument names so that we reduce room for typo error.
    They will be used to declare arguments, and by ``LaunchConfiguration`` substitutions to allow us to acquire the value of the launch argument.

    .. literalinclude:: code/example_substitutions_py.launch
      :language: python
      :lines: 11-13

    ``DeclareLaunchArgument`` is used to define the launch argument that can be passed from the above launch file or from the console.

    .. literalinclude:: code/example_substitutions_py.launch
      :language: python
      :lines: 16-18

    The ``turtlesim_node`` node named ``sim`` with ``namespace`` set to ``turtlesim_ns`` is created.

    .. literalinclude:: code/example_substitutions_py.launch
      :language: python
      :lines: 19-24

    Afterwards, an ``ExecuteProcess`` action is defined with the corresponding ``cmd`` argument.
    This command makes a call to the spawn service of the turtlesim node.

    Additionally, the ``LaunchConfiguration`` substitution is used to get the value of the ``turtlesim_ns`` launch argument to construct a command string.

    .. literalinclude:: code/example_substitutions_py.launch
      :language: python
      :lines: 25-34

    The same approach is used for the ``ExecuteProcess`` actions that change the turtlesim background's red color parameter.
    The difference is that the action in the timer is only executed if the provided ``new_background_r`` argument equals ``200`` and the ``use_provided_red`` launch argument is set to ``True``.
    The evaluation inside the ``IfCondition`` is done using the ``PythonExpression`` substitution.

    .. literalinclude:: code/example_substitutions_py.launch
      :language: python
      :lines: 35-65

4 Build the package
^^^^^^^^^^^^^^^^^^^

Go to the root of the workspace, and build the package:

.. code-block:: console

  colcon build

Also remember to source the workspace after building.

Launching example
-----------------

Now you can launch using the ``ros2 launch`` command.

.. code-block:: console

  ros2 launch launch_tutorial example_main.launch

This will do the following:

#. Start a turtlesim node with a blue background
#. Spawn the second turtle
#. Change the color to purple
#. Change the color to pink after two seconds if the provided ``background_r`` argument is ``200`` and ``use_provided_red`` argument is ``True``

Modifying launch arguments
--------------------------

If you want to change the provided launch arguments, you can either update the ``background_r`` variable in the ``example_main.launch`` or launch the ``example_substitutions.launch`` with preferred arguments.
To see arguments that may be given to the launch file, run the following command:

.. code-block:: console

    ros2 launch launch_tutorial example_substitutions.launch --show-args

This will show the arguments that may be given to the launch file and their default values.

.. code-block:: console

    Arguments (pass arguments as '<name>:=<value>'):

        'turtlesim_ns':
            no description given
            (default: 'turtlesim1')

        'use_provided_red':
            no description given
            (default: 'False')

        'new_background_r':
            no description given
            (default: '200')

Now you can pass the desired arguments to the launch file as follows:

.. code-block:: console

    ros2 launch launch_tutorial example_substitutions.launch turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200


Documentation
-------------

`The launch documentation <https://docs.ros.org/en/{DISTRO}/p/launch/architecture.html>`_ provides detailed information about available substitutions.

Summary
-------

In this tutorial, you learned about using substitutions in launch files.
You learned about their possibilities and capabilities to create reusable launch files.

You can now learn more about :doc:`using event handlers in launch files <./Using-Event-Handlers>` which are used to define a complex set of rules which can be used to dynamically modify the launch file.
