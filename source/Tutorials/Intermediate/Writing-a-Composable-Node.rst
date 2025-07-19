Writing a Composable Node (C++)
===============================

.. contents:: Table of Contents
   :depth: 2
   :local:

Starting Place
--------------

Let's assume that you have a regular ``rclcpp::Node`` executable that you want to run in the same process as other nodes to enable more efficient communication.

We'll start from having a class that directly inherits from ``Node``, and that also has a main method defined.

.. code-block:: c++

    namespace palomino
    {
        class VincentDriver : public rclcpp::Node
        {
            // ...
        };
    }

    int main(int argc, char * argv[])
    {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<palomino::VincentDriver>());
        rclcpp::shutdown();
        return 0;
    }

This will typically be compiled as an executable in your Cmake.

.. code-block:: cmake

    # ...
    add_executable(vincent_driver src/vincent_driver.cpp)
    # ...
    install(TARGETS vincent_driver
        DESTINATION lib/${PROJECT_NAME}
    )

Code Updates
------------

Add the Package Dependency
^^^^^^^^^^^^^^^^^^^^^^^^^^

Your `package.xml <https://github.com/ros2/demos/tree/{REPOS_FILE_BRANCH}/composition/package.xml>`__ should have a dependency on ``rclcpp_components``, a la

.. code-block:: xml

    <depend>rclcpp_components</depend>

Alternatively, you can independently add a ``build_depend/exec_depend``.

Class Definition
^^^^^^^^^^^^^^^^

The only change to your class definition that you may have to do is ensure that `the constructor for the class <https://github.com/ros2/demos/tree/{REPOS_FILE_BRANCH}/composition/src/talker_component.cpp>`__ takes a ``NodeOptions`` argument.

.. code-block:: c++

    VincentDriver(const rclcpp::NodeOptions & options) : Node("vincent_driver", options)
    {
      // ...
    }

No More Main Method
^^^^^^^^^^^^^^^^^^^

Replace your main method with a ``pluginlib``-style macro invocation.

.. code-block:: c++

    #include <rclcpp_components/register_node_macro.hpp>
    RCLCPP_COMPONENTS_REGISTER_NODE(palomino::VincentDriver)

.. caution::
    If the main method you are replacing contains a ``MultiThreadedExecutor``, be sure to make note and ensure that your container node is multithreaded.
    See section below.

CMake Changes
^^^^^^^^^^^^^
First, add ``rclcpp_components`` as a dependency in your CMakeLists.txt with:

.. code-block:: cmake

    find_package(rclcpp_components REQUIRED)

Second, we're going to replace our ``add_executable`` with a ``add_library`` with a new target name.

.. code-block:: cmake

    add_library(vincent_driver_component SHARED src/vincent_driver.cpp)

Third, replace other build commands that used the old target to act on the new target.
Don't forget to add ``rclcpp_components::component`` in ``target_link_libraries``.
i.e. ``target_link_libraries(vincent_driver ...)`` becomes ``target_link_libraries(vincent_driver_component rclcpp_components::component ...)``

Fourth, add a new command to declare your component.

.. code-block:: cmake

    rclcpp_components_register_node(
        vincent_driver_component
        PLUGIN "palomino::VincentDriver"
        EXECUTABLE vincent_driver
    )

Fifth and finally, change any installation commands in the CMake that operated on the old target to install the library version instead.
For instance, do not install either target into ``lib/${PROJECT_NAME}``.
Replace with the library installation.

.. code-block:: cmake

    ament_export_targets(export_vincent_driver_component)
    install(TARGETS vincent_driver_component
            EXPORT export_vincent_driver_component
            ARCHIVE DESTINATION lib
            LIBRARY DESTINATION lib
            RUNTIME DESTINATION bin
    )


Running Your Node
-----------------

See the :doc:`Composition tutorial <Composition>` for an in-depth look at composing nodes.
The quick and dirty version is that if you had the following in your Python launch file,

.. code-block:: python

    from launch_ros.actions import Node

    # ..

    ld.add_action(Node(
        package='palomino',
        executable='vincent_driver',
        # ..
    ))

you can replace it with

.. code-block:: python

    from launch_ros.actions import ComposableNodeContainer
    from launch_ros.descriptions import ComposableNode

    # ..
    ld.add_action(ComposableNodeContainer(
        name='a_buncha_nodes',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='palomino',
                plugin='palomino::VincentDriver',
                name='vincent_driver',
                # ..
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ]
    ))

.. caution::

    If you need multi-threading, instead of setting your executable to ``component_container``, set it to ``component_container_mt``
