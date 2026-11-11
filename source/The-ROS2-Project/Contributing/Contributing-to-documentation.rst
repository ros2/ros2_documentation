.. redirect-from::

    Contributing/Contributing-To-ROS-2-Documentation

.. meta::
   :contentType: about
   :experience: beginner, intermediate, expert
   :area: contributing, community
   :distribution: {DISTRO}
   :product: {PRODUCT}

.. _ContributingToDocumentation:

Contributing to documentation
=============================

.. short-description::
   ROS documentation is maintained by the community and helps users learn concepts, complete tasks, and contribute improvements to the wider ecosystem.
   This article describes where the documentation source is hosted, how the content is structured, and how contributors create, check, and submit changes.
.. showmeta::
   :order: area, contentType, experience
   :labels: area=Area, contentType=Content type, experience=Level

.. toctree::
   :hidden:
   :maxdepth: 2

   Documentation/Creating-or-updating-documentation
   Documentation/Documentation-guidelines

.. contents:: Contents
   :depth: 2
   :local:

Summary
-------

Contributions to the ROS documentation are very welcome.
The ROS documentation is written in reStructuredText (RST), and built using Sphinx.
You can find the documentation source code on `GitHub <https://github.com/ros2/ros2_documentation>`__.
Use the standard GitHub fork and pull request (PR) workflow when making your docs contributions.

This article relates to contributing to the ROS documentation site.
For more information about creating or updating package documentation, see :doc:`../../Developer-Tools/Package-documentation/Documenting-a-ROS-2-Package`.

Tools
-----

ROS documentation is written in `reStructuredText (RST) <https://www.sphinx-doc.org/en/master/usage/restructuredtext/>`__, with additional roles and directives.
The site is built using `Sphinx <https://www.sphinx-doc.org/en/master/>`__, and more particularly using `Sphinx multiversion <https://sphinx-contrib.github.io/multiversion/main/index.html>`__.

For more information about using these tools to make updates to the documentation and build the site, see:

* :doc:`Documentation/Creating-or-updating-documentation`
* :doc:`Documentation/Documentation-guidelines`

Metadata
--------

Every article carries metadata, such as its topic ``area`` and content type, in a ``.. meta::`` block at the top of the file.
The metadata is summarized near the top of the page, and it powers the search and related content features.
A check on each pull request flags any article that is missing metadata or that uses a value that is not allowed.

For the metadata values to use, and how to add them to an article, see :doc:`Documentation/Documentation-guidelines`.

Search
------

The site search is built from the metadata on each article.
When the site is built, the pages are indexed for search.
The metadata fields become filters, so that readers can narrow results by distribution, area, content type, and experience level, among others.
Results start filtered to the distribution the reader is viewing, which they can change or clear.

Related articles
----------------

A *Related articles* list links to other articles that share the same ``area`` as the page.
The list is generated when the site is built, so the links are part of the published HTML.
Entries are ordered by content type and then title, and a long list is collapsed behind a *Show more* control.

To learn how to add a related articles list to an article, see :doc:`Documentation/Documentation-guidelines`.

Related packages
----------------

A *Related packages* list shows the ROS packages relevant to the page.
Unlike related articles, which are built into the page when the site is built, these lists are built in the reader's browser.
When a reader opens the page, the browser downloads the latest snapshot of the ROS package index, keeps only the packages whose own metadata matches the page's ``area``, and fills in the list.
If it cannot reach the latest snapshot, it falls back to one saved when the site was built.
This keeps the lists current between documentation builds.
A package opts in to these lists through metadata in its ``package.xml``.

To learn how to add a related packages list to an article, see :doc:`Documentation/Documentation-guidelines`.

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

Related articles:

* :doc:`Documentation/Creating-or-updating-documentation`
* :doc:`Documentation/Documentation-guidelines`
* :doc:`../Contributing`
* :doc:`../../Developer-Tools/Package-documentation/Documenting-a-ROS-2-Package`
* `Documentation issue list <https://github.com/ros2/ros2_documentation/issues>`__

.. ros-related-articles::
