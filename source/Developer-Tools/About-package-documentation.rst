.. meta::
   :contentType: about
   :experience: beginner
   :area: package-documentation, tools
   :distribution: {DISTRO}
   :product: {PRODUCT}

Package documentation
=====================

.. short-description::
   Package documentation is essential for others to understand and use the package you've developed.
   This article summarizes the tools and guidance available to help with creating documentation for ROS packages.

.. showmeta::
   :order: area, contentType, experience
   :labels: area=Area, contentType=Content type, experience=Level

.. toctree::
   :hidden:

   Package-documentation/Documenting-a-ROS-2-Package

.. contents:: Contents
   :depth: 2
   :local:

Summary
-------

When creating a new package, clear documentation is essential for others who may use it.
Package documentation tools in ROS make it easy to write, maintain, and generate web-browsable documentation from your source code.

Core ROS packages
-----------------

* :doc:`rosdoc2 <Package-documentation/Documenting-a-ROS-2-Package>`: Tool for documenting a ROS package.

Package documentation locations
-------------------------------

ROS package documentation can be found in multiple places.
Here is a brief list of where to look for specific ROS package documentation.

* Most ROS packages have their package level documentation `included in this index page <https://docs.ros.org/en/{DISTRO}/p/>`__.

* All ROS package documentation is hosted alongside its information on the `ROS Index <https://index.ros.org/>`__.

Searching for packages on ROS Index will yield details such as released distributions, README.md files, URLs, and other important metadata.

Larger packages
^^^^^^^^^^^^^^^

Larger projects like MoveIt, Nav2, ros2_control, and micro-ROS host their documentation on their own domain or subdomain:

* `MoveIt <https://moveit.ros.org/>`__

* `Navigation2 <https://nav2.org/>`__

* `ros2_control <https://control.ros.org/{DISTRO}/index.html>`__

* `micro-ROS (embedded systems) <https://micro.ros.org/>`__

API documentation
^^^^^^^^^^^^^^^^^

You can find the API level documentation for the ROS client libraries in the {DISTRO_TITLE} distribution using the links below:

* `rclcpp - C++ client library <{package_link(rclcpp)}generated/index.html>`__

* `rclpy - Python client library <{package_link(rclpy)}>`__

* `rclcpp_lifecycle - C++ lifecycle library <{package_link(rclcpp_lifecycle)}generated/index.html>`__

* `rclcpp_components - C++ components library <{package_link(rclcpp_components)}generated/index.html>`__

* `rclcpp_action - C++ actions library <{package_link(rclcpp_action)}generated/index.html>`__

Guidelines for adding your package documentation
------------------------------------------------

All released ROS packages are automatically added to `docs.ros.org <https://docs.ros.org>`__ and `ROS Index <https://index.ros.org/>`__.

If you would like to enable or configure your own package, see :doc:`Package-documentation/Documenting-a-ROS-2-Package`.
