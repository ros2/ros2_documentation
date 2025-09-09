.. redirect-from::

Package Docs
============

ROS package documentation, that is to say documentation for specific packages you install via apt or some other tool, can be found in multiple places.
Here is a brief list of where to look for specific ROS package documentation.


* Most ROS 2 packages have their package level documentation `included in this index page <https://docs.ros.org/en/{DISTRO}/p/>`__.
* All ROS 2 package's documentation is hosted alongside its information on the `ROS Index <https://index.ros.org/>`_.
  Searching for packages on ROS Index will yield their information such as released distributions, ``README.md`` files, URLs, and other important metadata.

Larger Packages
---------------

Larger packages like MoveIt, Nav2, and microROS, are given their own domain or subdomain on ros.org.
Here is a short list.

* `MoveIt <https://moveit.ai/>`__
* `Navigation2 <https://nav2.org/>`__
* `Control <https://control.ros.org/master/index.html>`__
* `microROS (embedded systems) <https://micro.ros.org/>`__

API Documentation
-----------------

You can find the API level documentation for the ROS client libraries in the {DISTRO_TITLE} distribution using the links below:

* `rclcpp - C++ client library <https://docs.ros.org/en/{DISTRO}/p/rclcpp/generated/index.html>`_
* `rclcpp_lifecycle - C++ lifecycle library <https://docs.ros.org/en/{DISTRO}/p/rclcpp_lifecycle/generated/index.html>`_
* `rclcpp_components - C++ components library <https://docs.ros.org/en/{DISTRO}/p/rclcpp_components/generated/index.html>`_
* `rclcpp_action - C++ actions library <https://docs.ros.org/en/{DISTRO}/p/rclcpp_action/generated/index.html>`_

Adding Your Package to docs.ros.org
-----------------------------------

All released ROS 2 packages are automatically added to docs.ros.org and `ROS Index <https://index.ros.org/>`_.
If you would like to enable or configure your own package please see: :doc:`./How-To-Guides/Documenting-a-ROS-2-Package`.
