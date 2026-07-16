.. _DocsGuidelines:

Documentation guidelines
========================

The ROS documentation uses reStructuredText (RST) to create consistent, reviewable articles for the documentation site.
This article describes the main RST formatting patterns, directives, and roles used in the ROS documentation.

**Area: ROS-community | Content-type: reference | Experience: beginner, intermediate, expert**

.. contents:: Table of Contents
   :depth: 2
   :local:

Summary
-------

When creating content for the ROS documentation, write in reStructuredText (RST) and ensure that you follow good practice guidelines, or your pull request may not be accepted.

General formatting guidelines
-----------------------------

The ROS documentation website uses the ``reStructuredText`` format, which is the default plaintext markup language used by Sphinx.
This section is a brief introduction to ``reStructuredText`` concepts, syntax, and best practices.
When formatting your ``reStructuredText`` file **make sure to write only one sentence per line as it makes reviewing and modifying your file much easier.**
Also, be mindful of the use of white space in your file!
The ROS documentation linter will not accept pull requests with trailing white space.
We recommend that you enable automatic white space highlighting and cleanup if your editor supports it.

You can refer to `reStructuredText User Documentation <https://docutils.sourceforge.io/rst.html>`_ for a detailed technical specification.

This article relates to contributing to the ROS documentation site.
For more information about creating or updating package documentation, see :doc:`/Developer-Tools/Package-documentation/Documenting-a-ROS-2-Package`.

Table of contents
-----------------

There are two types of directives used for the generation of a table of contents: ``.. toctree::`` and ``.. contents::``.

The ``.. toctree::`` directive is used in top-level pages like ``Tutorials.rst`` to set ordering and visibility of its child pages.
This directive creates both left navigation panel and in-page navigation links to the child pages listed.
It helps readers to understand the structure of separate documentation sections and navigate between pages.

.. code-block:: rst

   .. toctree::
      :maxdepth: 1

The ``.. contents::`` directive is used for the generation of a table of contents for that particular page.
It parses all present headings in a page and builds an in-page nested table of contents.
It helps readers to see an overview of the content and navigate inside a page.

The ``.. contents::`` directive supports the definition of maximum depth of nested sections.
Using ``:depth: 2`` will only show sections and subsections in the table of contents.

.. code-block:: rst

   .. contents:: Table of Contents
      :depth: 2
      :local:

Headings
--------

There are four main heading types used in the documentation.
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

We usually use one digit for numbering subsections and two digits (dot separated) for numbering subsubsections in tutorials and how-to guides.

Lists
-----

Stars ``*`` are used for listing unordered items with bullet points, and number sign ``#.``  is used for listing numbered items.
Both of them support nested definitions and will render accordingly.

.. code-block:: rst

   * bullet point

     * bullet point nested
     * bullet point nested

   * bullet point

.. code-block:: rst

  #. first listed item
  #. second lited item

Code formatting
---------------

In-text code can be formatted using ``backticks`` for showing ``highlighted`` code.

.. code-block:: rst

   In-text code can be formatted using ``backticks`` for showing ``highlighted`` code.

Code blocks inside a page need to be captured using ``.. code-block::`` `directives <https://www.sphinx-doc.org/en/master/usage/restructuredtext/directives.html#directive-code-block>`_.
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

Code blocks: ``bash`` vs. ``console``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``bash`` and ``console`` are similar, but they serve two different purposes.
Choosing the right one is important to ensure that the content is formatted correctly and that the copy button copies the right content.
Below is an explanation of each one; skip to the end of this section for a list of use-cases and corresponding examples.

``bash`` is meant for scripts, e.g., for bash commands from a script file.
Example result:

.. code-block:: bash

   export ROS_DOMAIN_ID=42
   ros2 run turtlesim turtlesim_node

``console`` is meant for commands to be run in a terminal, optionally including their output.
This makes it clear that the given commands need to be run in a terminal.
It also allows separating command lines from output lines using prompt symbols such as ``$`` or ``#``.
Command lines are formatted as bash commands while output lines are formatted as normal text.
The prompt symbol is not selectable, and clicking on the copy button in the upper right-hand corner copies *only* the commands, not the outputs nor the prompt symbols.
This means that if a ``console`` code block is used without any ``$``, the copy button will not copy any lines.
Example result:

.. code-block:: console

   $ export ROS_DOMAIN_ID=42
   $ ros2 run turtlesim turtlesim_node --ros-args --remap "__node:=my_turtle"
   [INFO] [1742150439.022947971] [my_turtle]: Starting turtlesim with node name /my_turtle
   [INFO] [1742150439.026043867] [my_turtle]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]

Compare the above with a ``bash`` code-block:

.. code-block:: bash

   $ export ROS_DOMAIN_ID=42
   $ ros2 run turtlesim turtlesim_node --ros-args --remap "__node:=my_turtle"
   [INFO] [1742150439.022947971] [my_turtle]: Starting turtlesim with node name /my_turtle
   [INFO] [1742150439.026043867] [my_turtle]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]

To simplify code blocks, ``bash`` can still be used without ``$`` for commands meant to be run in a terminal if the code block does not include any output lines.
To help choose between ``bash`` and ``console``, see the following list of use-cases and corresponding examples:

* Commands meant to be copied into a script file.
  Use ``.. code-block:: bash`` without ``$``:

  .. code-block:: bash

     export ROS_DOMAIN_ID=42
     ros2 run turtlesim turtlesim_node

* Commands meant to be run in a terminal.
  It is highly recommended to use ``.. code-block:: console`` with ``$`` on all command lines for consistency and clarity.

  If there is output that needs to be displayed, include it in the same block:

  .. code-block:: console

     $ source /opt/ros/{DISTRO}/setup.bash
     $ ros2 run turtlesim turtlesim_node
     [INFO] [1743878028.269334696] [turtlesim]: Starting turtlesim with node name /turtlesim
     [INFO] [1743878028.275096618] [turtlesim]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]

  .. note::

     If some output lines start with ``#``, it is crucial to separate commands from their output because the ``#`` symbol is used to denote a command.
     Therefore, place the output in a separate ``.. code-block:: text``.

Images
------

Images can be inserted using the ``.. image::`` directive.

.. code-block:: rst

   .. image:: images/turtlesim_follow1.png

In this case, the image file (``turtlesim_follow1.png``) is located in the ``images/`` directory relative to the ``.rst`` file that uses the image.

However, all image files end up in an ``_images/`` directory relative to the root of the docs.
Therefore, when using ``:target:`` to add a hyperlink to the image file, use a relative link going up to the root directory and then down to the ``_images/`` directory.

.. code-block:: rst

   .. image:: images/turtlesim_follow1.png
      :target: ../../_images/turtlesim_follow1.png

Charts, graphs, and diagrams
----------------------------

ROS documentation now supports charts, graphs, and diagrams written using `Mermaid Charts. <https://mermaid.js.org/intro/>`__
We prefer that charts, graphs, and diagrams use Mermaid instead of static image files as it allows us to programmatically update and edit these resources as the project evolves.
Full documentation of the Mermaid graph language syntax can be found `on their website. <https://mermaid.js.org/intro/syntax-reference.html>`__

References and links
--------------------

External links
^^^^^^^^^^^^^^

The syntax of creating links to external web pages is shown below.

.. code-block:: rst

   `ROS Docs <https://docs.ros.org>`_

The above link will appear as `ROS Docs <https://docs.ros.org>`_.
Note the underscore after the final single quote.

Internal links
^^^^^^^^^^^^^^

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
------

Macros can be used to simplify writing documentation that targets multiple distributions.

Use a macro by including the macro name in curly braces.
For example, when generating the docs for Rolling on the ``rolling`` branch:

.. list-table::
   :header-rows: 1

   * - Macro
     - Example
     - Becomes (for {DISTRO_TITLE})
   * - \{DISTRO\}
     - ros-\{DISTRO\}-pkg
     - ros-{DISTRO}-pkg
   * - \{DISTRO_TITLE\}
     - ROS 2 \{DISTRO_TITLE\}
     - ROS 2 {DISTRO_TITLE}
   * - \{DISTRO_TITLE_FULL\}
     - ROS 2 \{DISTRO_TITLE_FULL\}
     - ROS 2 {DISTRO_TITLE_FULL}
   * - \{REPOS_FILE_BRANCH\}
     - git checkout \{REPOS_FILE_BRANCH\}
     - git checkout {REPOS_FILE_BRANCH}
   * - \{interface_link(std_msgs/msg/String)\}
     - See: \{interface_link(std_msgs/msg/String)\}.
     - See: {interface_link(std_msgs/msg/String)}.
   * - \{interface(std_msgs/msg/String)\}
     - Publish a \{interface(std_msgs/msg/String)\}.
     - Publish a {interface(std_msgs/msg/String)}.
   * - \{package_link(rclcpp)\}
     - See: \{package_link(rclcpp)\}.
     - See: {package_link(rclcpp)}.
   * - \{package(rclcpp)\}
     - Use \{package(rclcpp)\}.
     - Use {package(rclcpp)}.

The same file can be used on multiple branches (i.e., for multiple distros) and the generated content will be distro-specific.
