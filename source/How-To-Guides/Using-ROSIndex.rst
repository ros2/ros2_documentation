Searching the ROS 2 Packages
############################

.. contents:: Table of Contents
   :depth: 2
   :local:


This guide introduces the standard way of searching for ROS 2 package-level documentation.
There are thousands of ROS packages that encompass all aspects of robotics; everything from hardware drivers to high-level planning and navigation.
Some of these packages exist solely as open source code, while others are distributed as binary packages that can be easily installed on a ROS system using tools like `apt`.
A complete and searchable list of these ROS binary packages is available at `index.ros.org <http://index.ros.org>`__, which is similar to `Python's Package Index (PyPI) <https://pypi.org/>`__.
A good way to find a solution to many common robotics problems is to use ROS Index to find an appropriate ROS 2 binary package from the ROS ecosystem.
Using ROS Index you can find binary ROS packages, the source code behind them, and their associated documentation  and  tutorials.

Prerequisites
-------------

- `Open ROS Index <https://index.ros.org/>`_


Searching for ROS Packages
----------------------------

Locate the search bar in the middle of the screen on `index.ros.org <https://index.ros.org/>`__:

.. image:: ros-index-images/search_bar.png
  :width: 500 px
  :alt: Search Bar on ROS Index

You can use this search bar to find specific packages or search for specific package features.
ROS Index has the capability to search through the available package documentation and find keywords that match the given search query.

For example if one wanted to do color processing on images they could either search `image_proc` or `image processing`.
After clicking the top package in the list, it will bring one to a page similar to `this <https://index.ros.org/p/image_proc/github-ros-perception-image_pipeline/#humble>`_:

.. image:: ros-index-images/package_info.png
  :width: 500 px
  :alt: Package Info on ROS Index

Breaking it Down
~~~~~~~~~~~~~~~~

1. `Package Description` The general description of the package, this should be similar to what was searched if it wasn't an exact package name.
2. `Tutorials` If there are any tutorials available in the documentation they will show up here. Those looking to get started with a package should look here as the tutorials will often be up to date and well maintained by those who use the package often.
3. `Documentation` The original documentation for the ROS package and the repository where source code and documentation can be found.```
4. `Distribution` ROS 2 has `various distributions <https://docs.ros.org/en/rolling/Releases.html>`_ which may have differences in design. Click on the ROS distribution (version) that matches the ROS version installed on your system.
5. `Contributing` This software is open source, so it needs the community's help to actively be maintained. One should utilize this section if they wish to help solve issues, review pull requests, or are looking to see if an active fix is being deployed for a problem they encountered.
6. `Maintainers` This tab lists the community members who are actively responsible for the development of the package. While the maintainers have a vast amount of information one should use the `Robotics Stack Exchange <https://robotics.stackexchange.com/search?q=>`_ with the [tag] of the package, if they want questions answered.
