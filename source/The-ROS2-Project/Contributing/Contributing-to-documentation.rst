.. redirect-from::

    Contributing/Contributing-To-ROS-2-Documentation

.. _ContributingToDocumentation:

Contributing to documentation
=============================

ROS documentation is maintained by the community and helps users learn, build, and contribute effectively.
This article describes where the documentation source lives, how it is structured, and how contributors submit changes.

**Area: contributing, community | Content-type: about | Experience: beginner, intermediate, expert**

.. toctree::
   :hidden:
   :maxdepth: 2

   Documentation/Creating-or-updating-documentation
   Documentation/Documentation-guidelines

.. contents:: Table of Contents
   :depth: 2
   :local:

Summary
-------

Contributions to the ROS documentation are very welcome.
The ROS documentation is written in reStructuredText (RST), and built using Sphinx.
You can find the documentation source code on `GitHub <https://github.com/ros2/ros2_documentation>`__.
Use the standard GitHub fork and pull request (PR) workflow when making your docs contributions.

This article relates to contributing to the ROS documentation site.
For more information about creating or updating package documentation, see :doc:`/Developer-Tools/Package-documentation/Documenting-a-ROS-2-Package`.

Tools
-----

ROS documentation is written in `reStructuredText (RST) <https://www.sphinx-doc.org/en/master/usage/restructuredtext/>`__, with additional roles and directives.
The site is built using `Sphinx <https://www.sphinx-doc.org/en/master/>`__, and more particularly using `Sphinx multiversion <https://sphinx-contrib.github.io/multiversion/main/index.html>`__.

For more information about using these tools to make updates to the documentation and build the site, see:

* :doc:`Documentation/Creating-or-updating-documentation`
* :doc:`Documentation/Documentation-guidelines`

Branch structure
----------------

The source code of the documentation is located in the `ROS Documentation GitHub repository <https://github.com/ros2/ros2_documentation>`__.
This repository is set up with one branch per ROS distribution to handle differences between the distributions:

* If a change is common to all ROS distributions, it should be made to the ``rolling`` branch (and then will be backported as appropriate).
* If a change is specific to a particular ROS distribution, it should be made to the respective branch.

Source structure
----------------

* The source files for the site are all located under the ``source`` subdirectory.
* Templates for various Sphinx plugins are located under ``source/_templates``.
* The root directory contains configuration and files required to locally build the site for testing.

Workflow
--------

Contributors to the ROS documentation on GitHub use the standard fork and pull request (PR) workflow.

#. After creating your own fork of the ``ros2_documentation`` repository, clone it locally, and then create a branch from the existing ``rolling`` branch for your changes.
#. When your change to the documentation source is ready on your branch, create a pull request to the upstream repository.
   Your PR will attract review comments and suggestions from the ROS community, which you can then action and agree on.
#. When your PR is accepted, it is merged to the target branch and published to the respective versions of the site.

Related content
---------------

More articles and information about the ROS docs:

* :doc:`Documentation/Creating-or-updating-documentation`
* :doc:`Documentation/Documentation-guidelines`
* :doc:`/The-ROS2-Project/Contributing`
* :doc:`/Developer-Tools/Package-documentation/Documenting-a-ROS-2-Package`
* `Documentation issue list <https://github.com/ros2/ros2_documentation/issues>`__
