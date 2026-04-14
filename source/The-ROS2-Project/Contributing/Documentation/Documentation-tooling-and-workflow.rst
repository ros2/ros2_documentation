.. _DocsToolingAndWorkflow:

Documentation tooling and workflow
==================================

.. centered:: TBC

.. parsed-literal::

    Area: ROS-community | Content-type: about | Experience: beginner, intermediate, expert

.. contents:: Table of Contents
   :depth: 1
   :local:

Summary
-------

TBC

The following sections relate to updating the ROS 2 user documentation.

For more information about making updates to the documentation, and building the site, see :doc:`./Creating-or-updating-documentation`.

Tools
-----

ROS 2 documentation is written in `reStructuredText (RST) <https://www.sphinx-doc.org/en/master/usage/restructuredtext/>`_, with additional roles and directives which contributors use to create a well-structured documentation site.
The site is built using `Sphinx <https://www.sphinx-doc.org/en/master/>`_, and more particularly using `Sphinx multiversion <https://sphinx-contrib.github.io/multiversion/main/index.html>`_.

Branch structure
----------------

The source code of the documentation is located in the `ROS 2 Documentation GitHub repository <https://github.com/ros2/ros2_documentation>`_.
This repository is set up with one branch per ROS 2 distribution to handle differences between the distributions:

* If a change is common to all ROS 2 distributions, it should be made to the ``rolling`` branch (and then will be backported as appropriate).
* If a change is specific to a particular ROS 2 distribution, it should be made to the respective branch.

Source structure
----------------

* The source files for the site are all located under the ``source`` subdirectory.
* Templates for various sphinx plugins are located under ``source/_templates``.
* The root directory contains configuration and files required to locally build the site for testing.

Workflow
--------

Contributors to the ROS 2 documentation on GitHub use the standard Fork and Pull Request (PR) workflow.

#. After cloning the ``ros2_documentation`` repository, create your own fork, then take a branch on that fork for your changes.
#. When your change to the docs source is ready on your branch, create a Pull Request to the upstream repository.
   Your PR will attract review comments and suggestions from the ROS 2 community, which you can then action and agree on.
#. When your PR is accepted, it is merged to the targeted branch and published to the respective version(s) of the site.
