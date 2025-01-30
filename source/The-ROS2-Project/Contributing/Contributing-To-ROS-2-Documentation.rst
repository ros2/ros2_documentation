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

Start by creating `venv <https://docs.python.org/3/library/venv.html>`__ to build the documentation:

.. code-block:: console

   # activate the venv
   python3 -m venv ros2doc

   # activate venv
   source ros2doc/bin/activate

And install requirements located in the ``requirements.txt`` file:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

       pip install -r requirements.txt -c constraints.txt

  .. group-tab:: macOS

    .. code-block:: console

       pip install -r requirements.txt -c constraints.txt

  .. group-tab:: Windows

    .. code-block:: console

      python -m pip install -r requirements.txt -c constraints.txt

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

View Site Through Github CI
^^^^^^^^^^^^^^^^^^^^^^^^^^^

For small changes to the ROS 2 Docs you can view your changes as rendered HTML using artifacts generated in our Github Actions.
The "build" action produces the entire ROS Docs as a downloadable Zip file that contains all HTML for `docs.ros.org <https://docs.ros.org/>`_
This build action is triggered after passing the test action and lint action.

To download and view your changes first go to your pull request and under the title click the "Checks" tab.
On the left hand side of the checks page, click on the "Test" section under the "tests" section  click on "build" dialog.
This will open a menu on the right, where you can click on "Upload document artifacts" and scroll to the bottom to see the download link for the Zipped' HTML files under the heading "Artifact download URL".

.. image:: ./images/github_action.png
  :width: 100%
  :alt: Steps to find rendered HTML files on ROS Github action

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

Migrating Pages from the ROS Wiki
---------------------------------

The first step in migrating a page from the `ROS Wiki <https://wiki.ros.org>`_ to the ROS 2 documentation is to determine if the page needs to be migrated.
Check if the content, or something similar, is available on https://docs.ros.org/en/rolling by searching for related terms.
If it has already been migrated, congratulations!
You are done.
If it hasn't been migrated, then consider whether it is worth keeping.
Pages that you or others find useful, and refer to regularly, are good candidates assuming they have not been superseded by other documentation.
Pages for ROS projects and features that are no longer supported by a current distribution should not be migrated.

The next step for migrating a ROS Wiki page is to determine the correct location for the migrated page.
Only ROS Wiki pages that cover core ROS concepts belong in the ROS Documentation, these pages should be migrated to a logical location within the ROS documentation.
Package specific documentation should be migrated to the package-level documentation generated in the package's source repository.
Once the package level documentation has been updated it will be visible `as part of the package-level documentation <https://docs.ros.org/en/rolling/p/>`__.
If you are unsure whether and where to migrate a page, please get in touch via an issue on https://github.com/ros2/ros2_documentation or on https://discourse.ros.org.

Once you've determined that a ROS Wiki page is worth migrating, and found an appropriate landing spot in the ROS documentation, the next step in the migration process is to set up the conversion tools necessary to migrate the page.
In most cases the only tools necessary to migrate a single ROS Wiki page to the ROS Docs are the `PanDoc <https://pandoc.org/>`_ command line tool and a text editor.
PanDoc is supported by most modern operating systems using the installation instruction found on their website.
It is worth noting that the ROS Wiki uses an older wiki technology (MoinMoin), so the markup language used is an obscure dialect of the `MediaWiki <https://www.mediawiki.org/wiki/Help:Formatting>`__ format.
We've found that the easiest way to migrate a page from the ROS Wiki is to convert it from HTML into reStructured text using PanDoc.


Migrating a Wiki File
^^^^^^^^^^^^^^^^^^^^^

#. Clone the appropriate repository.  If you are migrating a page to the official documentation hosted here, then you should clone https://github.com/ros2/ros2_documentation.

#. Create a new Github branch for your migrated page. We suggest something like ``pagename-migration``.

#. Download the appropriate ROS Wiki page to an html file using wget or a similar tool (e.g. ``wget -O urdf.html https://wiki.ros.org/urdf``).
   Alternatively you can use your web browser to save the page's HTML.

#. Next you need to remove the extraneous HTML in the file you downloaded
   Using your browser's developer mode, find the name of the first useful HTML element in the Wiki page.
   In most cases all of the HTML between the third line of the file, starting with the ``<head>`` tag, through the start of the first ``<h1>`` tag can be safely removed.
   In the case where there is a table of contents, the first useful tag may be an ``<h2>`` tag.  Similarly, the ROS wiki contains some footer text that starts with ``<div id="pagebottom"></div>`` and ends just above ``</body></html>`` that can also be removed.

#. Convert your html file by running a PanDoc conversion between HTML and restructured text.
   The following command converts an HTML file to the equivalent reStructured text files: ``pandoc -f html -t rst urdf.html > URDF.rst``.

#. Attempt to build your new documentation using the ``make html`` command.
   There may be errors and warnings that you will need to address.

#. **CAREFULLY** read through the entire page making sure the material is up to date for ROS 2.
   Check every single link to make sure it points to the appropriate location on docs.ros.org.
   Internal document references must be updated to point to the equivalent ROS 2 material.
   Your updated document should not point to the ROS Wiki unless it is absolutely necessary.
   This process may require you alter the document considerably, and you may need to pull multiple wiki files.
   You should verify that every code sample in the document is working correctly under ROS 2.

#. Find and download any images that may be in the old document. The easiest way to do this is to right click in the browser and download all of the images. Alternatively you can find images by searching for ``<img src>`` tags in the HTML file.

#. For each image files downloaded update the image file links to point to the correct image directory for the ROS Docs.
   If any of the images require updating, or could be replaced with a `Mermaid <https://mermaid.js.org/intro/>`__ chart, please make this change.
   Be aware that Mermaid.js is only supported in the core ROS 2 documentation currently.

#. Once your document is complete add a table of contents to the top of your new rst document using the appropriate Sphinx commands.
   This block should replace any existing table of contents from the old ROS Wiki.

#. Issue your pull request.
   Make sure to point to the original ROS Wiki file for reference.

#. Once your pull request has been accepted please add a note to the top of the page on the original ROS Wiki article pointing to the new documentation page.

For a real-world example of this process in action, please refer to the ROS 2 Image Processing Pipeline in both `the ROS 2 Docs <https://github.com/ros-perception/image_pipeline/blob/rolling/image_pipeline/doc/tutorials.rst>`__ and in the original `ROS Wiki <https://wiki.ros.org/image_pipeline>`__.
The completed documentation page can be found in the `ROS 2 package documentation for image_pipeline <https://docs.ros.org/en/rolling/p/image_pipeline/>`__.

Building the Site with GitHub Codespaces
----------------------------------------
First, you need to have a GitHub account (if you don't have one, you can create one for free).
Then, you need to go to the `ROS 2 Documentation GitHub repository <https://github.com/ros2/ros2_documentation>`__.
After that, you can open the repository in Codespaces, it can be done just by clicking on the "Code" button on the repository page, then choose "Open with Codespaces" from the dropdown menu.

.. image:: images/codespaces.png
   :width: 100%
   :alt: Codespaces creation

After that, you will be redirected to your Codespaces page, where you can see the progress of the Codespaces creation.
Once it is done, a Visual Studio Code tab will be opened in your browser. You can open the terminal by clicking on the "Terminal" tab in the top panel or by pressing :kbd:`Ctrl-J`.

In this terminal, you can run any command you want, for example, you can run the following command to build the site for just this branch:

.. code-block:: console

   make html

Finally, to view the site, you can click on the "Go Live" button in the right bottom panel and then, it will open the site in a new tab in your browser (you will need to browse to the ``build/html`` folder).

.. image:: images/live_server.png
   :width: 100%
   :alt: Live Server

Building the Site with Devcontainer
-----------------------------------

`ROS 2 Documentation GitHub repository <https://github.com/ros2/ros2_documentation>`__ also supports ``Devcontainer`` development environment with Visual Studio Code.
This will enable you to build the documentation much easier without changing your operating system.

See :doc:`../../How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container` to install VS Code and Docker before the following procedure.

Clone repository and start VS Code:

.. code-block:: console

   git clone https://github.com/ros2/ros2_documentation
   cd ./ros2_documentation
   code .

To use ``Devcontainer``, you need to install "Remote Development" Extension within VS Code search in Extensions (CTRL+SHIFT+X) for it.

And then, use ``View->Command Palette...`` or ``Ctrl+Shift+P`` to open the command palette.
Search for the command ``Dev Containers: Reopen in Container`` and execute it.
This will build your development docker container for you automatically.

To build the documentation, open a terminal using ``View->Terminal`` or ``Ctrl+Shift+``` and ``New Terminal`` in VS Code.
Inside the terminal, you can build the documentation:

.. code-block:: console

   make html

.. image:: images/vscode_devcontainer.png
   :width: 100%
   :alt: VS Code Devcontainer

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
