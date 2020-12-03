.. _CppParamNode:

Using parameters in a class (C++)
=================================

**Goal:** Create and run a class with ROS parameters using C++.

**Tutorial level:** Beginner

**Time:** 20 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

When making your own :ref:`nodes <ROS2Nodes>` you will sometimes need to add parameters that can be set from the launch file.

This tutorial will show you how to create those parameters in a C++ class, and how to set them in a launch file.

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

  ros2 pkg create --build-type ament_cmake cpp_parameters --dependencies rclcpp

Your terminal will return a message verifying the creation of your package ``cpp_parameters`` and all its necessary files and folders.

The ``--dependencies`` argument will automatically add the necessary dependency lines to ``package.xml`` and ``CMakeLists.txt``.

1.1 Update ``package.xml``
~~~~~~~~~~~~~~~~~~~~~~~~~~

Because you used the ``--dependencies`` option during package creation, you don’t have to manually add dependencies to ``package.xml`` or ``CMakeLists.txt``.

As always, though, make sure to add the description, maintainer email and name, and license information to ``package.xml``.

.. code-block:: xml

  <description>C++ parameter tutorial</description>
  <maintainer email="you@email.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

2 Write the C++ node
^^^^^^^^^^^^^^^^^^^^

Inside the ``dev_ws/src/cpp_parameters/src`` directory, create a new file called ``cpp_parameters_node.cpp`` and paste the following code within:

.. code-block:: C++

    #include <rclcpp/rclcpp.hpp>
    #include <chrono>
    #include <string>
    #include <functional>

    using namespace std::chrono_literals;

    class ParametersClass: public rclcpp::Node
    {
      public:
        ParametersClass()
          : Node("parameter_node")
        {
          this->declare_parameter<std::string>("my_parameter", "world");
          timer_ = this->create_wall_timer(
          1000ms, std::bind(&ParametersClass::respond, this));
        }
        void respond()
        {
          this->get_parameter("my_parameter", parameter_string_);
          RCLCPP_INFO(this->get_logger(), "Hello %s", parameter_string_.c_str());
        }
      private:
        std::string parameter_string_;
        rclcpp::TimerBase::SharedPtr timer_;
    };

    int main(int argc, char** argv)
    {
      rclcpp::init(argc, argv);
      rclcpp::spin(std::make_shared<ParametersClass>());
      rclcpp::shutdown();
      return 0;
    }

2.1 Examine the code
~~~~~~~~~~~~~~~~~~~~
The ``#include`` statements at the top are the package dependencies.

The next piece of code creates the class and the constructor.
The first line of this constructor creates our parameter.
Our parameter has the name ``my_parameter`` and is assigned the default value ``world``.
Next, ``timer_`` is initialized, which causes the ``respond`` function to be executed once a second.

.. code-block:: C++

    class ParametersClass: public rclcpp::Node
    {
      public:
        ParametersClass()
          : Node("parameter_node")
        {
          this->declare_parameter<std::string>("my_parameter", "world");
          timer_ = this->create_wall_timer(
          1000ms, std::bind(&ParametersClass::respond, this));
        }

The first line of our ``respond`` function gets the parameter ``my_parameter`` from the node, and stores it in ``parameter_string_``.
The ``RCLCPP_INFO`` function ensures the message is logged.

.. code-block:: C++

    void respond()
    {
      this->get_parameter("my_parameter", parameter_string_);
      RCLCPP_INFO(this->get_logger(), "Hello %s", parameter_string_.c_str());
    }

Last is the declaration of ``timer_`` and ``parameter_string_``

.. code-block:: C++

    private:
      std::string parameter_string_;
      rclcpp::TimerBase::SharedPtr timer_;

Following our ``ParametersClass`` is our ``main``.
Here ROS 2 is initialized, and ``rclcpp::spin`` starts processing data from the node.

.. code-block:: C++

    int main(int argc, char** argv)
    {
      rclcpp::init(argc, argv);
      rclcpp::spin(std::make_shared<ParametersClass>());
      rclcpp::shutdown();
      return 0;
    }


2.2 Add executable
~~~~~~~~~~~~~~~~~~

Now open the ``CMakeLists.txt`` file. Below the dependency ``find_package(rclcpp REQUIRED)`` add the following lines of code.

.. code-block:: console

    add_executable(parameter_node src/cpp_parameters_node.cpp)
    ament_target_dependencies(parameter_node rclcpp)

    install(TARGETS
      parameter_node
      DESTINATION lib/${PROJECT_NAME}
    )


3 Build and run
^^^^^^^^^^^^^^^

It's good practice to run ``rosdep`` in the root of your workspace (``dev_ws``) to check for missing dependencies before building:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        rosdep install -i --from-path src --rosdistro rolling -y

   .. group-tab:: macOS

      rosdep only runs on Linux, so you can skip ahead to next step.

   .. group-tab:: Windows

      rosdep only runs on Linux, so you can skip ahead to next step.

Navigate back to the root of your workspace, ``dev_ws``, and build your new package:

.. code-block:: console

    colcon build --packages-select cpp_parameters

Open a new terminal, navigate to ``dev_ws``, and source the setup files:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      . install/setup.bash

  .. group-tab:: macOS

    .. code-block:: console

      . install/setup.bash

  .. group-tab:: Windows

    .. code-block:: console

      call install/setup.bat

Now run the node:

.. code-block:: console

     ros2 run cpp_parameters parameter_node

The terminal should return the following message every second:

.. code-block:: console

    [INFO] [parameter_node]: Hello world

Now you can see the default value of your parameter, but you want to be able to set it yourself.
There are two ways to accomplish this.

3.1 Change via the console
~~~~~~~~~~~~~~~~~~~~~~~~~~

This part will use the knowledge you have gained from the :ref:`tutorial about parameters <ROS2Params>` and apply it to the node you have just created.

Make sure the node is running:

.. code-block:: console

     ros2 run cpp_parameters parameter_node

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
You can also set the parameter in a launch file, but first you will need to add the launch directory.
Inside the ``dev_ws/src/cpp_parameters/`` directory, create a new directory called ``launch``.
In there, create a new file called ``cpp_parameters_launch.py``


.. code-block:: Python

  from launch import LaunchDescription
  from launch_ros.actions import Node

  def generate_launch_description():
      return LaunchDescription([
          Node(
              package="cpp_parameters",
              executable="parameter_node",
              name="custom_parameter_node",
              output="screen",
              emulate_tty=True,
              parameters=[
                  {"my_parameter": "earth"}
              ]
          )
      ])

Here you can see that we set ``my_parameter`` to ``earth`` when we launch our node ``parameter_node``.
By adding the two lines below, we ensure our output is printed in our console.

.. code-block:: console

          output="screen",
          emulate_tty=True,

Now open the ``CMakeLists.txt`` file.
Below the lines you added earlier, add the following lines of code.

.. code-block:: console

    install(
      DIRECTORY launch
      DESTINATION share/${PROJECT_NAME}
    )

Open a console and navigate to the root of your workspace, ``dev_ws``, and build your new package:

.. code-block:: console

    colcon build --packages-select cpp_parameters

Then source the setup files in a new terminal:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      . install/setup.bash

  .. group-tab:: macOS

    .. code-block:: console

      . install/setup.bash

  .. group-tab:: Windows

    .. code-block:: console

      call install/setup.bat

Now run the node using the launch file we have just created:

.. code-block:: console

     ros2 launch cpp_parameters cpp_parameters_launch.py

The terminal should return the following message every second:

.. code-block:: console

    [parameter_node-1] [INFO] [custom_parameter_node]: Hello earth

Summary
-------

You created a node with a custom parameter, that can be set either from a launch file or the command line.
You added the dependencies, executables, and a launch file to the package configuration files so that you could build and run them, and see the parameter in action.

Next steps
----------

Now that you have some packages and ROS 2 systems of your own, the :ref:`next tutorial <Ros2Doctor>` will show you how to examine issues in your environment and systems in case you have problems.
