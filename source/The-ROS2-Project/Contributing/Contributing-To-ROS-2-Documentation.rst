.. redirect-from::

    Contributing/Contributing-To-ROS-2-Documentation

Contributing to ROS 2 Documentation
===================================

.. contents:: Table of Contents
   :depth: 2
   :local:

Contributions to this site are most welcome.
This page explains how to contribute to ROS 2 Documentation.
Please be sure to read the below sections carefully before contributing.

The site is built using `Sphinx <https://www.sphinx-doc.org/en/master/>`_, and more particularly using `Sphinx multiversion <https://holzhaus.github.io/sphinx-multiversion/master/index.html>`_.

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

Building the site locally
-------------------------

Start by installing requirements located in the ``requirements.txt`` file:

.. tabs::

  .. group-tab:: Linux

    The next command does a user-specific install, which requires ``~/.local/bin/`` to be added to ``$PATH``:

    .. code-block:: console

       pip3 install --user --upgrade -r requirements.txt

  .. group-tab:: macOS

    .. code-block:: console

       pip3 install --user --upgrade -r requirements.txt

  .. group-tab:: Windows

    .. code-block:: console

      python -m pip install --user --upgrade -r requirements.txt

In order for Sphinx to be able to generate diagrams, the ``dot`` command must be available.

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

       sudo apt update ; sudo apt install graphviz

  .. group-tab:: macOS

    .. code-block:: console

      brew install graphviz

  .. group-tab:: Windows

      Download an installer from `the Graphviz Download page <https://graphviz.gitlab.io/_pages/Download/Download_windows.html>`__ and install it.
      Make sure to allow the installer to add it to the Windows ``%PATH%``, otherwise Sphinx will not be able to find it.

Building the site for one branch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To build the site for just this branch, type ``make html`` at the top-level of the repository.
This is the recommended way to test out local changes.

.. code-block:: console

   make html

The build process can take some time.
To see the output, open ``build/html/index.html`` in your browser.

You can also run the documentation tests locally (using `doc8 <https://github.com/PyCQA/doc8>`_) with the following command:

.. code-block:: console

   make test

Building the site for all branches
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To build the site for all branches, type ``make multiversion`` from the ``rolling`` branch.
This has two drawbacks:

#. The multiversion plugin doesn't understand how to do incremental builds, so it always rebuilds everything.
   This can be slow.

#. When typing ``make multiversion``, it will always check out exactly the branches listed in the ``conf.py`` file.
   That means that local changes will not be shown.

To show local changes in the multiversion output, you must first commit the changes to a local branch.
Then you must edit the `conf.py <https://github.com/ros2/ros2_documentation/blob/rolling/conf.py>`_ file and change the ``smv_branch_whitelist`` variable to point to your branch.

Checking for broken links
^^^^^^^^^^^^^^^^^^^^^^^^^

To check for broken links on the site, run:

.. code-block:: console

   make linkcheck

This will check the entire site for broken links, and output the results to the screen and ``build/linkcheck``.

Writing pages
-------------

The ROS 2 documentation website uses the ``reStructuredText`` format, which is the default plaintext markup language used by Sphinx.
This section is a brief introduction to ``reStructuredText`` concepts, syntax, and best practices.

You can refer to `reStructuredText User Documentation <https://docutils.sourceforge.io/rst.html>`_ for a detailed technical specification.

Table of Contents
^^^^^^^^^^^^^^^^^

There are two types of directives used for the generation of a table of contents, ``.. toctree::`` and ``.. contents::``.
The ``.. toctree::`` is used in top-level pages like ``Tutorials.rst`` to set ordering and visibility of its child pages.
This directive creates both left navigation panel and in-page navigation links to the child pages listed.
It helps readers to understand the structure of separate documentation sections and navigate between pages.

.. code-block:: rst

   .. toctree::
      :maxdepth: 1

The ``.. contents::`` directive is used for the generation of a table of contents for that particular page.
It parses all present headings in a page and builds an in-page nested table of contents.
It helps readers to see an overview of the content and navigate inside a page.

The ``.. contents::`` directive supports the definition of maximum depth of nested sections.
Using ``:depth: 2`` will only show Sections and Subsections in the table of contents.

.. code-block:: rst

   .. contents:: Table of Contents
      :depth: 2
      :local:

Headings
^^^^^^^^

There are four main Heading types used in the documentation.
Note that the number of symbols has to match the length of the title.

.. code-block:: rst

   Page Title Header
   =================

   Section Header
   --------------

   2 Subsection Header
   ^^^^^^^^^^^^^^^^^^^

   2.4 Subsubsection Header
   ~~~~~~~~~~~~~~~~~~~~~~~~

We usually use one digit for numbering subsections and two digits (dot separated) for numbering subsubsections in Tutorials and How-To-Guides.

Lists
^^^^^

Stars ``*`` are used for listing unordered items with bullet points and number sign ``#.``  is used for listing numbered items.
Both of them support nested definitions and will render accordingly.

.. code-block:: rst

   * bullet point

     * bullet point nested
     * bullet point nested

   * bullet point

.. code-block:: rst

  #. first listed item
  #. second lited item

Code Formatting
^^^^^^^^^^^^^^^

In-text code can be formatted using ``backticks`` for showing ``highlighted`` code.

.. code-block:: rst

   In-text code can be formatted using ``backticks`` for showing ``highlighted`` code.

Code blocks inside a page need to be captured using ``.. code-block::`` directive.
``.. code-block::`` supports code highlighting for syntaxes like ``C++``, ``YAML``, ``console``, ``bash``, and more.
Code inside the directive needs to be indented.

.. code-block:: rst

   .. code-block:: C++

      int main(int argc, char** argv)
      {
         rclcpp::init(argc, argv);
         rclcpp::spin(std::make_shared<ParametersClass>());
         rclcpp::shutdown();
         return 0;
      }

Images
^^^^^^

Images can be inserted using the ``.. image::`` directive.

.. code-block:: rst

   .. image:: images/turtlesim_follow1.png

References and Links
^^^^^^^^^^^^^^^^^^^^

External links
~~~~~~~~~~~~~~

The syntax of creating links to external web pages is shown below.

.. code-block:: rst

   `ROS Docs <https://docs.ros.org>`_

The above link will appear as `ROS Docs <https://docs.ros.org>`_.
Note the underscore after the final single quote.

Internal links
~~~~~~~~~~~~~~

The ``:doc:`` directive is used to create in-text links to other pages.

.. code-block:: rst

   :doc:`Quality of Service <../Tutorials/Quality-of-Service>`

Note that the relative path to the file is used.

The ``ref`` directive is used to make links to specific parts of a page.
These could be headings, images or code sections inside the current or different page.

Definition of explicit target right before the desired object is required.
In the example below, the target is defined as ``_talker-listener`` one line before the heading ``Try some examples``.

.. code-block:: rst

   .. _talker-listener:

   Try some examples
   -----------------

Now the link from any page in the documentation to that header can be created.

.. code-block:: rst

   :ref:`talker-listener demo <talker-listener>`

This link will navigate a reader to the target page with an HTML anchor link ``#talker-listener``.

Macros
~~~~~~

Macros can be used to simplify writing documentation that targets multiple distributions.

Use a macro by including the macro name in curly braces.
For example, when generating the docs for Rolling on the ``rolling`` branch:


=====================  =====================  ==================================
Use                    Becomes (for Rolling)  Example
=====================  =====================  ==================================
\{DISTRO\}             rolling                ros-\{DISTRO\}-pkg
\{DISTRO_TITLE\}       Rolling                ROS 2 \{DISTRO_TITLE\}
\{DISTRO_TITLE_FULL\}  Rolling Ridley         ROS 2 \{DISTRO_TITLE_FULL\}
\{REPOS_FILE_BRANCH\}  rolling                git checkout \{REPOS_FILE_BRANCH\}
=====================  =====================  ==================================

The same file can be used on multiple branches (i.e., for multiple distros) and the generated content will be distro-specific.
