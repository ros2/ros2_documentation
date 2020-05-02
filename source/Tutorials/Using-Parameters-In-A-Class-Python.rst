.. _PythonParamNode:

Using parameters in a class (Python)
=================================

**Goal:** Create and run a class with ROS parameters using Python (rclpy).

**Tutorial level:** Beginner

**Time:** 20 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

When making your own :ref:`nodes <ROS2Nodes>` you will sometimes need to add parameters that can be set from the launch file.

This tutorial will show you how to create those parameters in a Python class, and how to set them in a launch file.

Prerequisites
-------------

In previous tutorials, you learned how to :ref:`create a workspace <ROS2Workspace>` and :ref:`create a package <CreatePkg>`.
You have also learned about :ref:`parameters <ROS2Params>` and their function in a ROS 2 system.

Tasks
-----
1 Create a package
^^^^^^^^^^^^^^^^^^

Open a new terminal and :ref:`source your ROS 2 installation <ConfigROS2>` so that ``ros2`` commands will work.

Navigate into the ``dev_ws`` directory created in a previous tutorial.

Recall that packages should be created in the ``src`` directory, not the root of the workspace.
Navigate into ``dev_ws/src`` and create a new package:

.. code-block:: console

  ros2 pkg create --build-type ament_python python_parameters --dependencies rclpy

Your terminal will return a message verifying the creation of your package ``python_parameters`` and all its necessary files and folders.

The ``--dependencies`` argument will automatically add the necessary dependency lines to ``package.xml`` and ``CMakeLists.txt``.

1.1 Update ``package.xml``
~~~~~~~~~~~~~~~~~~~~~~~~~~

Because you used the ``--dependencies`` option during package creation, you don’t have to manually add dependencies to ``package.xml`` or ``CMakeLists.txt``.

As always, though, make sure to add the description, maintainer email and name, and license information to ``package.xml``.

.. code-block:: xml

  <description>Python parameter tutorial</description>
  <maintainer email="you@email.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

2 Write the Python node
^^^^^^^^^^^^^^^^^^^^

Inside the ``dev_ws/src/python_parameters/src`` directory, create a new file called ``python_parameters_node.py`` and paste the following code within:

.. code-block:: Python




2.1 Examine the code
~~~~~~~~~~~~~~~~~~~~


3 Build and run
^^^^^^^^^^^^^^^

It's good practice to run ``rosdep`` in the root of your workspace (``dev_ws``) to check for missing dependencies before building:

.. code-block:: console

  sudo rosdep install -i --from-path src --rosdistro <distro> -y

Navigate back to the root of your workspace, ``dev_ws``, and build your new package:

.. code-block:: console

    colcon build --packages-select python_parameters

Open a new terminal, navigate to ``dev_ws``, and source the setup files:

.. code-block:: console

    . install/setup.bash

Now run the node:

.. code-block:: console

     ros2 run python_parameters parameter_node

The terminal should return the following message every second:

.. code-block:: console

    [INFO] [parameter_node]: Hello world

Now you can see the default value of your parameter, but you want to be able to set it yourself.
There are two ways to accomplish this.

3.1 Change via the console
~~~~~~~~~~~~~~~~~~~~~~~~~~

This part will use the knowledge you have gained from the :ref:`tutoral about parameters <ROS2Params>` and apply it to the node you have just created.

Make sure the node is running:

.. code-block:: console

     ros2 run python_parameters parameter_node

Open another terminal, source the setup files from inside ``dev_ws`` again, and enter the following line:

.. code-block:: console

    ros2 param list

There you will see the custom parameter ``my_parameter``.
To change it simply run the following line in the console:

.. code-block:: console

    ros2 param set /parameter_node my_parameter earth

You know it went well if you get the output ``Set parameter successful``.
If you look at the other terminal, you should see the output change to ``[INFO] [parameter_node]: Hello earth``

3.2 Change via a launch file
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Refer to the C++ equivalent tutorial.

Summary
-------

You created a node with a custom parameter, that can be set either from the launch file or the command line.
You added the dependencies, executable, and launch file to the package configuration files so that you could build and run them, and see the parameter in action.

Next steps
----------

Now that you have some packages and ROS 2 systems of your own, the :ref:`next tutorial <Ros2Doctor>` will show you how to examine issues in your environment and systems in case you have problems.
