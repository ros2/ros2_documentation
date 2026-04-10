.. _DocsToolingAndWorkflow:

Documentation tooling and workflow
==================================

.. contents:: Table of Contents
   :depth: 1
   :local:

The site is built using `Sphinx <https://www.sphinx-doc.org/en/master/>`_, and more particularly using `Sphinx multiversion <https://sphinx-contrib.github.io/multiversion/main/index.html>`_.

Branch structure
----------------

The source code of documentation is located in the `ROS 2 Documentation GitHub repository <https://github.com/ros2/ros2_documentation>`_.
This repository is set up with one branch per ROS 2 distribution to handle differences between the distributions.
If a change is common to all ROS 2 distributions, it should be made to the ``rolling`` branch (and then will be backported as appropriate).
If a change is specific to a particular ROS 2 distribution, it should be made to the respective branch.

Source structure
----------------

The source files for the site are all located under the ``source`` subdirectory.
Templates for various sphinx plugins are located under ``source/_templates``.
The root directory contains configuration and files required to locally build the site for testing.
