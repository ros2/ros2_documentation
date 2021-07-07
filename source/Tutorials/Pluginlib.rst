Creating and Using Plugins (C++)
======================================

**Goal:** Learn to create and load a simple plugin using pluginlib

**Tutorial level:** Beginner

.. **Time:** 20 minutes

**Minimum Platform:** Ardent

.. contents:: Contents
   :depth: 3
   :local:

Background
----------

This tutorial is derived from `<http://wiki.ros.org/pluginlib>`_ and `Writing and Using a Simple Plugin Tutorial <http://wiki.ros.org/pluginlib/Tutorials/Writing%20and%20Using%20a%20Simple%20Plugin>`_.

pluginlib is a C++ library for loading and unloading plugins from within a ROS package. Plugins are dynamically loadable classes that are loaded from a runtime library (i.e. shared object, dynamically linked library). With pluginlib, one does not have to explicitly link their application against the library containing the classes -- instead pluginlib can open a library containing exported classes at any point without the application having any prior awareness of the library or the header file containing the class definition. Plugins are useful for extending/modifying application behavior without needing the application source code.

Prerequisites
-------------

This tutorial assumes basic C++ knowledge and that you have ``pluginlib`` installed.

.. code-block:: console

  sudo apt-get install ros-{DISTRO}-pluginlib


Tasks
-----

In this tutorial, you will create a two new packages, one that defines the base class, and the other that provides the plugins. The base class will define a generic polygon class, and then our plugins will define specific shapes.


1 Create the Base Class Package
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Create a new empty package in your ``dev_ws/src`` folder with the following terminal command.

.. code-block:: console

  ros2 pkg create --build-type ament_cmake polygon_base --dependencies pluginlib --node-name area_node


Open your favorite editor, edit ``dev_ws/src/polygon_base/include/polygon_base/regular_polygon.hpp``, and paste the following inside of it:

.. code-block:: C++

    #ifndef POLYGON_BASE_REGULAR_POLYGON_HPP
    #define POLYGON_BASE_REGULAR_POLYGON_HPP

    namespace polygon_base
    {
      class RegularPolygon
      {
        public:
          virtual void initialize(double side_length) = 0;
          virtual double area() = 0;
          virtual ~RegularPolygon(){}

        protected:
          RegularPolygon(){}
      };
    }  // namespace polygon_base

    #endif  // POLYGON_BASE_REGULAR_POLYGON_HPP

This code above should be pretty self explanatory... we're creating an abstract class called ``RegularPolygon``. One thing to notice is the presence of the initialize method. With ``pluginlib``, a constructor without parameters is required for classes so, if any parameters are required, we use the initialize method to initialize the object.

We need to make this header available to other classes, so open ``dev_ws/src/polygon_base/CMakeLists.txt`` for editing. Add the following lines after the ``ament_target_dependencies`` command.

.. code-block:: console

    install(
      DIRECTORY include/
      DESTINATION include
    )

And add this command before the ``ament_package`` command

.. code-block:: console

    ament_export_include_directories(
      include
    )

We will return to this package later to write our test node.

2 Create the Plugin Package
^^^^^^^^^^^^^^^^^^^^^^^^^^^
Now we're going to write two non-virtual implementations of our abstract class. Create a second empty package in your ``dev_ws/src`` folder with the following terminal command.

.. code-block:: console

  ros2 pkg create --build-type ament_cmake polygon_plugins --dependencies polygon_base pluginlib --library-name polygon_plugins

2.1 Source code for the plugins
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Open ``dev_ws/src/polygon_plugins/src/polygon_plugins.cpp`` for editing, and paste the following inside of it:

.. code-block:: C++

    #include <polygon_base/regular_polygon.hpp>
    #include <cmath>

    namespace polygon_plugins
    {
      class Square : public polygon_base::RegularPolygon
      {
        public:
          void initialize(double side_length) override
          {
            side_length_ = side_length;
          }

          double area() override
          {
            return side_length_ * side_length_;
          }

        protected:
          double side_length_;
      };

      class Triangle : public polygon_base::RegularPolygon
      {
        public:
          void initialize(double side_length) override
          {
            side_length_ = side_length;
          }

          double area() override
          {
            return 0.5 * side_length_ * getHeight();
          }

          double getHeight()
          {
            return sqrt((side_length_ * side_length_) - ((side_length_ / 2) * (side_length_ / 2)));
          }

        protected:
          double side_length_;
      };
    }

    #include <pluginlib/class_list_macros.hpp>

    PLUGINLIB_EXPORT_CLASS(polygon_plugins::Square, polygon_base::RegularPolygon)
    PLUGINLIB_EXPORT_CLASS(polygon_plugins::Triangle, polygon_base::RegularPolygon)

The implementation of the Square and Triangle classes should be fairly straightforward: save the side length, and use it to calculate the area. The only piece that is pluginlib specific is the last three lines, which invokes some magical macros that register the classes as actual plugins. Let's go through the arguments to the ``PLUGINLIB_EXPORT_CLASS`` macro:

1. The fully-qualified type of the plugin class, in this case, ``polygon_plugins::Square``.
2. The fully-qualified type of the base class, in this case, ``polygon_base::RegularPolygon``.

2.2 Plugin Declaration XML
~~~~~~~~~~~~~~~~~~~~~~~~~~
The steps above make it so that instances of our plugins can be created once the library they exist in is loaded, but the plugin loader still needs a way to find that library and to know what to reference within that library. To this end, we'll also create an XML file that, along with a special export line in the package manifest, makes all the necessary information about our plugins available to the ROS toolchain.

Create ``dev_ws/src/polygon_plugins/plugins.xml`` with the following code:

.. code-block:: XML

    <library path="polygon_plugins">
      <class type="polygon_plugins::Square" base_class_type="polygon_base::RegularPolygon">
        <description>This is a square plugin.</description>
      </class>
      <class type="polygon_plugins::Triangle" base_class_type="polygon_base::RegularPolygon">
        <description>This is a triangle plugin.</description>
      </class>
    </library>

A couple things to note:

1. The ``library`` tag gives the relative path to a library that contains the plugins that we want to export. In ROS 2, that is just the name of the library. In ROS 1 it contained the prefix ``lib`` or sometimes ``lib/lib`` (i.e. ``lib/libpolygon_plugins``) but here it is simpler.
2. The ``class`` tag declares a plugin that we want to export from our library. Let's go through its parameters:

  * ``type``: The fully qualified type of the plugin. For us, that's ``polygon_plugins::Square``.
  * ``base_class``: The fully qualified base class type for the plugin. For us, that's ``polygon_base::RegularPolygon``.
  * ``description``: A description of the plugin and what it does.
  * ``name``: There used to be a name attribute, but it is no longer required.

2.3 CMake Plugin Declaration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The last step is to export your plugins via ``CMakeLists.txt``. This is a change from ROS 1, where the exporting was done via ``package.xml``. Add the following block to your ``dev_ws/src/polygon_plugins/CMakeLists.txt`` after the line reading ``find_package(pluginlib REQUIRED)``

.. code-block:: console

    add_library(polygon_plugins src/polygon_plugins.cpp)
    target_include_directories(polygon_plugins PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
    ament_target_dependencies(
      polygon_plugins
      polygon_base
      pluginlib
    )

    pluginlib_export_plugin_description_file(polygon_base plugins.xml)

    install(
      TARGETS polygon_plugins
      EXPORT export_${PROJECT_NAME}
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )

And before the ``ament_package`` command, add

.. code-block:: console

    ament_export_libraries(
      polygon_plugins
    )
    ament_export_targets(
      export_${PROJECT_NAME}
    )


The arguments to this CMake command are

1. The package for the base class, i.e. ``polygon_base``
2. The relative path to the Plugin Declaration xml, i.e. ``plugins.xml``


3 Use the Plugins
^^^^^^^^^^^^^^^^^
Now its time to use the plugins. This can be done in any package, but here we're going to do it in the base package. Edit ``dev_ws/src/polygon_base/src/area_node.cpp`` to contain the following:

.. code-block:: C++

    #include <pluginlib/class_loader.hpp>
    #include <polygon_base/regular_polygon.hpp>

    int main(int argc, char** argv)
    {
      // To avoid unused parameter warnings
      (void) argc;
      (void) argv;

      pluginlib::ClassLoader<polygon_base::RegularPolygon> poly_loader("polygon_base", "polygon_base::RegularPolygon");

      try
      {
        std::shared_ptr<polygon_base::RegularPolygon> triangle = poly_loader.createSharedInstance("polygon_plugins::Triangle");
        triangle->initialize(10.0);

        std::shared_ptr<polygon_base::RegularPolygon> square = poly_loader.createSharedInstance("polygon_plugins::Square");
        square->initialize(10.0);

        printf("Triangle area: %.2f\n", triangle->area());
        printf("Square area: %.2f\n", square->area());
      }
      catch(pluginlib::PluginlibException& ex)
      {
        printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
      }

      return 0;
    }

The ``ClassLoader`` is the key class to understand, defined in the ``class_loader.hpp`` `header <https://github.com/ros/pluginlib/blob/ros2/pluginlib/include/pluginlib/class_loader.hpp>`_.

 * It is templated with the base class, i.e. ``polygon_base::RegularPolygon``
 * The first argument is a string for the package name of the base class, i.e. ``polygon_base``
 * The second argument is a string with the fully qualified base class type for the plugin, i.e. ``polygon_base::RegularPolygon``

There are a number of ways to instantiate an instance of the class. In this example, we're using shared pointers. We just need to call ``createSharedInstance`` with the fully-qualified type of the plugin class, in this case, ``polygon_plugins::Square``.

Important note: the ``polygon_base`` package in which this node is defined does NOT depend on the ``polygon_plugins`` class. The plugins will be loaded dynamically without any dependency needing to be declared. Furthermore, we're instantiating the classes with hardcoded plugin names, but you can also do so dynamically with parameters, etc.

4 Build and run
^^^^^^^^^^^^^^^

Navigate back to the root of your workspace, ``dev_ws``, and build your new packages:

.. code-block:: console

    colcon build --packages-select polygon_base polygon_plugins

From ``dev_ws``, be sure to source the setup files:

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

     ros2 run polygon_base area_node

It should print

.. code-block:: console

    Triangle area: 43.30
    Square area: 100.00



Summary
-------

Congratulations! You've just written and used your first plugins.
