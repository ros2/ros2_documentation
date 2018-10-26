
Defining custom interfaces (msg/srv)
====================================

**INCOMPLETE**

While we encourage reuse of existing "standard" message and service definitions wherever possible, there are plenty of cases in which you'll need to define your own custom messages and/or services for a particular application.
The first step in defining a custom message or service is to write the ``.msg`` or ``.srv`` file, which you do using the `ROS interface definition language <About-ROS-Interfaces>`.
By convention, ``.msg`` files go into a package subdirectory called ``msg`` and ``.srv`` files go into a package subdirectory called ``srv`` (you can pick different locations, but we recommend following the convention).

Having written your ``.msg`` and/or ``.srv`` files, you need to add some code to your package's ``CMakelists.txt`` file to make the code generators run over your definitions. In lieu of a more complete tutorial on this topic, consult the `pendulum_msgs package <https://github.com/ros2/demos/tree/master/pendulum_msgs>`__ as an example. You can see the relevant CMake calls in that packages's `CMakeLists.txt file <https://github.com/ros2/demos/blob/master/pendulum_msgs/CMakeLists.txt>`__.

Note that the ``package.xml`` format must equal 3 for this to build, this is because the ``member_of_group`` command requires format 3. ROS2's create package generates the ``package.xml`` with a default format of 2.
