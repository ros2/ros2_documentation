Package documentation
=====================

.. toctree::
   :maxdepth: 1

   Package-documentation/Documenting-a-ROS-2-Package

Package documentation is essential for others to understand and use the package you've developed.
This article summarizes the tools and guidance available to help with creating documentation for ROS packages.

**Area: package documentation, tools | Content-type: about | Experience: beginner, intermediate, expert**

.. contents:: Table of Contents
   :local:

Summary
--------

When creating a new package, clear documentation is essential for others who may use it.
Package documentation tools in ROS make it easy to write, maintain, and generate web-browsable documentation from your source code.

Core ROS packages:

* `rosdoc2 <https://docs.ros.org/en/rolling/How-To-Guides/Documenting-a-ROS-2-Package.html>`_: Tool for documenting a ROS package.

.. Community-contributed packages:

Package documentation locations
-------------------------------

ROS package documentation can be found in multiple places.
Here is a brief list of where to look for specific ROS package documentation.

* Most ROS packages have their package level documentation `included in this index page <https://docs.ros.org/en/lyrical/p/>`_.

* All ROS package documentation is hosted alongside its information on the `ROS Index <https://index.ros.org/>`_.

Searching for packages on ROS Index will yield their information such as released distributions, README.md files, URLs, and other important metadata.

Larger packages
---------------

Larger packages like MoveIt, Nav2, and microROS, are given their own domain or subdomain on ros.org.
Here is a short list.

* `MoveIt <https://moveit.ros.org/>`_

* `Navigation2 <https://nav2.org/>`_

* `Control <https://control.ros.org/rolling/index.html>`_

* `microROS (embedded systems) <https://micro.vulcanexus.org/>`_

API documentation
-----------------

You can find the API level documentation for the ROS client libraries in the Lyrical distribution using the links below:

* `rclcpp - C++ client library <https://docs.ros.org/en/lyrical/p/rclcpp/generated/index.html>`_

* `rclcpp_lifecycle - C++ lifecycle library <https://docs.ros.org/en/lyrical/p/rclcpp_lifecycle/generated/index.html>`_

* `rclcpp_components - C++ components library <https://docs.ros.org/en/lyrical/p/rclcpp_components/generated/index.html>`_

* `rclcpp_action - C++ actions library <https://docs.ros.org/en/lyrical/p/rclcpp_action/generated/index.html>`_

Guidelines for adding your package documentation
------------------------------------------------

All released ROS packages are automatically added to docs.ros.org and `ROS Index <https://index.ros.org/>`_.
If you would like to enable or configure your own package, see `Documenting a ROS package <https://docs.ros.org/en/lyrical/How-To-Guides/
Documenting-a-ROS-2-Package.html>`_.

.. Related content (placeholder)
   -----------------------------

   More articles about package documentation:

   * Example

   * Example

   FAQs (placeholder)
   ------------------

   * Example

   * Example
