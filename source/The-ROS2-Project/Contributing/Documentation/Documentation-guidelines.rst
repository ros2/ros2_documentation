.. meta::
   :contentType: reference
   :experience: beginner, intermediate, expert
   :area: contributing, community
   :distribution: {DISTRO}
   :product: {PRODUCT}

.. _DocsGuidelines:

Documentation guidelines
========================

.. short-description::
   Consistent documentation helps contributors create ROS articles that are easy to review, maintain, and navigate.
   This article describes the RST formatting conventions, metadata, directives, content types, and other guidelines used when creating ROS documentation.

.. showmeta::
   :order: area, contentType, experience
   :labels: area=Area, contentType=Content type, experience=Level

.. contents:: Table of Contents
   :depth: 2
   :local:

Summary
-------

When creating content for the ROS documentation, write in reStructuredText (RST) and ensure that you follow good practice guidelines, or your pull request may not be accepted.

Formatting
----------

The guidance in this section will help you to make sure your ROS documentation is properly formatted.

General formatting guidelines
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The ROS documentation website uses the ``reStructuredText`` format, which is the default plaintext markup language used by Sphinx.
This section is a brief introduction to ``reStructuredText`` concepts, syntax, and best practices.
When formatting your ``reStructuredText`` file, write only one sentence per line.
That makes reviewing and modifying the file much easier.
Also, be mindful of the use of white space in your file!
The ROS documentation linter will not accept pull requests with trailing white space.
We recommend that you enable automatic white space highlighting and cleanup if your editor supports it.

You can refer to `reStructuredText User Documentation <https://docutils.sourceforge.io/rst.html>`_ for a detailed technical specification.

This article relates to contributing to the ROS documentation site.
For more information about creating or updating package documentation, see :doc:`/Developer-Tools/Package-documentation/Documenting-a-ROS-2-Package`.

Metadata and related directives
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Article metadata provides information about the content of an article, such as its context and purpose, and related packages or articles.
Adding metadata to an article also improves search engine optimization (SEO) and web indexing.
ROS articles also include related directives to manage visible page metadata and article summaries.

Every ROS article should include specific metadata and supporting directives so that the documentation renders correctly and related content can be generated.

Required metadata
~~~~~~~~~~~~~~~~~
Each ROS article must include the ``.. meta::`` directive, and within that, the ``:area:`` field.

``:area:`` indicates the area of ROS documentation the article belongs to and its position in the documentation hierarchy.
This is essential for enabling search filtering and for building lists of related packages or articles.

List one or more values, ordered **most specific first**.
For example, ``:area: contributing, community`` means the article belongs to the ``contributing`` subsection inside the ``community`` area.
The first value (``contributing``) is the primary value used when building lists of related articles and packages.

.. note::

   If the ``:area:`` field is missing, empty, or uses a value outside the allowed vocabulary, the Enhance check on the pull request fails.

To add a value for the ``:area:`` field, use one or more of the following values.
Broader parent areas (such as ``community``, ``framework``, ``tools``, and ``capabilities``) may be listed after more specific subsection values:

.. literalinclude:: ../../../../tools/enhance.yaml
   :language: yaml
   :start-after: allow_multiple: true
   :end-before: experience:
   :dedent:

Recommended related directives
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Each ROS article should include the ``.. showmeta::`` and ``.. short-description::`` related directives to manage visible page metadata and article summaries.
Omitting them triggers a warning in the Enhance check rather than failing the pull request.

``.. showmeta::`` renders the specified metadata at the top of a published article.

Add the following related directive to your article:

 .. code-block:: rst

    .. showmeta::
       :order: area, contentType, experience
       :labels: area=Area, contentType=Content type, experience=Level

``.. short-description::``: Provides a two or three sentence explanation about the article's content.

  Use the following structure to create a short description, making sure your sentences answer each of the questions:

  * Sentence 1 - Context: Why would a user be interested in reading this article?

  * Sentence 2 - Purpose: What does the article describe or what will a user get out of reading it?

  * Sentence 3 - Outcome: What will the user be able to do after reading the article or following the guidance?
    For ``about`` type articles, this sentence is optional.

You can use the following examples to guide you.

Short description for :doc:`About ROS<../../../About-ROS>`:

.. code-block:: rst

  .. short-description::

   ROS (Robot Operating System) is an open-source ecosystem that provides the framework, tools, and libraries for building, deploying, running, and maintaining robotic applications.
   This article introduces the main areas of the ecosystem and outlines their intended use.

Short description for :doc:`Using the Node Interfaces Template Class (C++) — tutorial<../../../ROS-Framework/nodes/Working-with-nodes/Using-Node-Interfaces-Template-Class>`:

  .. code-block:: rst

    .. short-description::

    Different ROS node types can expose the same capabilities through different C++ classes.
    This tutorial explains how to use ``rclcpp::node_interfaces::NodeInterfaces<>`` to write functions that accept both standard and lifecycle nodes.
    After completing it, you can pass node interfaces compactly and retrieve node information reliably.

Optional metadata
~~~~~~~~~~~~~~~~~

Some metadata is optional, meaning that omitting it only triggers a warning in the Enhance check rather than failing the pull request.
It is best practice to include the optional metadata.

In the ``.. meta::`` directive, the following fields and corresponding values are optional.

* ``:contentType:``: The type of content in the article.
  The value for this field helps structure search results and allows readers to filter content to find the information they need.

Add one of the following values:

   * about
   * how-to
   * tutorial
   * reference
   * example
   * learning-path
   * process-overview
   * release-note

For more information on content types, see :ref:`Content types <ContentTypes>`.

* ``:experience:``: The level of experience required to understand the article.

Add one or more of the following values:

   * beginner
   * intermediate
   * expert

* ``:distribution:``: Enables the documentation to work across multiple ROS distributions.

Add the following value:

  * ``{DISTRO}``

* ``:product:``: Identifies which ROS product the article belongs to.

Add the following value:

 * ``{PRODUCT}``

Related content
^^^^^^^^^^^^^^^

Related content is the packages or articles linked from an article based on the primary (first) value in the ``:area:`` field.

To generate related articles automatically at build time, add the ``.. ros-related-articles::`` directive.
Write a short intro line such as ``Related articles:`` above it.
Matching uses the primary ``:area:`` value: other pages that include that value anywhere in their own ``:area:`` list are listed.

.. code-block:: rst

  Related articles:

  .. ros-related-articles::

You can also add related content manually.
Use the following formatting and naming conventions.
You can check :doc:`About client libraries <../../../ROS-Framework/About-Client-Libraries>` for an example.

.. code-block:: rst

  Related content
  ---------------

  Related articles
  ~~~~~~~~~~~~~~~~

  * Related article here

  Related packages
  ~~~~~~~~~~~~~~~~

  Core ROS packages
  ^^^^^^^^^^^^^^^^^

  * Package here

  Community packages
  ^^^^^^^^^^^^^^^^^^

  * Package here

Table of contents
^^^^^^^^^^^^^^^^^

There are two types of directives used for generating a table of contents: ``.. toctree::`` and ``.. contents::``.

The ``.. toctree::`` directive is used in top-level pages, for example, :doc:`Community <../../../The-ROS2-Project>`, to organize and display the list of subarticles.
This directive creates both the left-hand navigation pane and the in-page navigation links to the subarticles.
It helps readers understand the structure of the documentation sections and navigate between articles.

.. code-block:: rst

   .. toctree::
      :maxdepth: 1

The ``.. contents::`` directive is used for generating the table of contents for a particular article by parsing all headings in the article.
The table of contents shows readers the structural overview of the content and helps them navigate it.

The ``.. contents::`` directive defines the maximum depth of sections displayed in the article's table of contents.
The recommended value is ``:depth: 2`` to ensure only section and subsection headings are displayed.
This is particularly important for large articles with many nested sections, for example, :doc:`Quality Guide <../Quality-Guide>`.

.. code-block:: rst

   .. contents:: Table of Contents
      :depth: 2
      :local:

Headings
^^^^^^^^

There are four main heading types used in the documentation.
Note that the number of symbols has to match the length of the title.

.. code-block:: rst

   Page title heading
   ==================

   Section heading
   ---------------

   2 Subsection heading
   ^^^^^^^^^^^^^^^^^^^^

   2.4 Subsubsection heading
   ~~~~~~~~~~~~~~~~~~~~~~~~~

We usually use one digit for numbering subsections and two digits (dot separated) for numbering subsubsections in tutorials and how-to guides.

Lists
^^^^^

Stars ``*`` are used for listing unordered items using bullet points, and the number sign ``#.`` is used for listing numbered items.
Both of them support nested definitions and will render accordingly.

.. code-block:: rst

   * Bullet point

     * Bullet point nested
     * Bullet point nested

   * Bullet point

.. code-block:: rst

  #. First listed item
  #. Second listed item

Code formatting
^^^^^^^^^^^^^^^

In-text code can be formatted using ``backticks`` for showing ``highlighted`` code.

.. code-block:: rst

   In-text code can be formatted using ``backticks`` for showing ``highlighted`` code.

Code blocks inside a page need to be captured using the ``.. code-block::`` `directive <https://www.sphinx-doc.org/en/master/usage/restructuredtext/directives.html#directive-code-block>`_.
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
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``bash`` and ``console`` are similar, but they serve two different purposes.
Choosing the right one ensures that the content is formatted correctly and that the copy button copies the right content.
The following section explains each one.
You can skip to the end of this section for a list of use cases and corresponding examples.

``bash`` is meant for scripts, for example for bash commands from a script file.
Example result:

.. code-block:: bash

   export ROS_DOMAIN_ID=42
   ros2 run turtlesim turtlesim_node

``console`` is meant for commands to be run in a terminal, optionally including their output.
This makes it clear that the given commands need to be run in a terminal.
It also allows you to separate command lines from output lines using prompt symbols such as ``$`` or ``#``.
Command lines are formatted as bash commands, while output lines are formatted as normal text.
The prompt symbol is not selectable.
Clicking the copy button in the upper right-hand corner copies *only* the commands, not the outputs or the prompt symbols.
If a ``console`` code block is used without any ``$``, the copy button will not copy any lines.
Example result:

.. code-block:: console

   $ export ROS_DOMAIN_ID=42
   $ ros2 run turtlesim turtlesim_node --ros-args --remap "__node:=my_turtle"
   [INFO] [1742150439.022947971] [my_turtle]: Starting turtlesim with node name /my_turtle
   [INFO] [1742150439.026043867] [my_turtle]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]

Compare the previous example result with a ``bash`` code-block:

.. code-block:: bash

   $ export ROS_DOMAIN_ID=42
   $ ros2 run turtlesim turtlesim_node --ros-args --remap "__node:=my_turtle"
   [INFO] [1742150439.022947971] [my_turtle]: Starting turtlesim with node name /my_turtle
   [INFO] [1742150439.026043867] [my_turtle]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]

To simplify code blocks, ``bash`` can still be used without ``$`` for commands meant to be run in a terminal if the code block does not include any output lines.
To help choose between ``bash`` and ``console``, see the following list of use cases and corresponding examples:

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
^^^^^^

Images can be inserted in an article by using the ``.. image::`` directive.

.. code-block:: rst

   .. image:: images/turtlesim_follow1.png

In this case, the image file (``turtlesim_follow1.png``) is located in the ``images/`` directory relative to the ``.rst`` file that uses the image.

However, all image files end up in an ``_images/`` directory relative to the root of the docs.
Therefore, when using ``:target:`` to add a hyperlink to the image file, use a relative link going up to the root directory and then down to the ``_images/`` directory.

.. code-block:: rst

   .. image:: images/turtlesim_follow1.png
      :target: ../../_images/turtlesim_follow1.png

Charts, graphs, and diagrams
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

ROS documentation now supports charts, graphs, and diagrams written using `Mermaid Charts. <https://mermaid.js.org/intro/>`__
Prefer Mermaid instead of static image files, so these resources can be updated programmatically as the project evolves.
Full documentation of the Mermaid graph language syntax can be found `on their website. <https://mermaid.js.org/intro/syntax-reference.html>`__

References and links
^^^^^^^^^^^^^^^^^^^^

External links
~~~~~~~~~~~~~~

The syntax of creating links to external web pages is shown in the following example.

.. code-block:: rst

   `ROS Docs <https://docs.ros.org>`_

The above link will appear as `ROS Docs <https://docs.ros.org>`_.
Note the underscore after the final single quote.

Internal links
~~~~~~~~~~~~~~

The ``:doc:`` directive is used to create links to other articles in the ROS documentation.

.. code-block:: rst

   :doc:`Quality of Service <../../../ROS-Framework/interfaces/topics/Working-with-topics/Quality-of-Service>`

Note that the relative path to the file is used.

The ``:ref:`` directive is used for linking to a specific target, for example, a heading, image, or code section, either in the same article, or in another article.

To create a targeted link, place a definition directly above the target you want to link to.
In the example below, the target is defined as ``_talker-listener`` one line before the ``Try some examples`` heading.

.. code-block:: rst

   .. _talker-listener:

   Try some examples
   -----------------

Now the link from any article in the documentation to that header can be created.

.. code-block:: rst

   :ref:`talker-listener demo <talker-listener>`

This link will navigate a reader to the target page with an HTML anchor link ``#talker-listener``.

Macros
^^^^^^

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

.. _ContentTypes:

Content types
-------------

Content types define patterns in the content of different types of articles, designed to meet a range of information needs.
The main purpose of content types is to set expectations for humans and AI about the expected content range and scope.
The patterns defined by content types are also valuable for enabling efficient content creation and maintenance.
To best support information retrieval by both humans and machines, each article contains a single content type.

Most ROS articles should be one of the following content types.
Use the examples in the following table for guidance about the type of article you are creating.

.. list-table::
   :header-rows: 1

   * - Content type
     - Description
     - Suitable for
     - Examples
   * - About
     - Explanation of tools or capability areas, or of a technical concept.
     - Explaining capability and technical concepts.
     - :doc:`About ROS <../../../About-ROS>`

       :doc:`Debugging <../../../Developer-Tools/About-Debugging>`

       :doc:`Interfaces (topics, services, actions)<../../../ROS-Framework/Interfaces-Topics-Services-Actions>`
   * - Learning path
     - Define learning path (curated syllabus based on other articles in the site or in other sites).

       Links to other articles for the detail of each learning step.
     - Use for defining a list of reading and learning activities or a particular functionality area or stage in learning.

       Suitable for all levels of experience but more likely to be used by beginners.
     - :doc:`First steps with ROS <../../../First-Steps>`
   * - Process overview
     - Define overall steps in a complex process to help understand the process and find detailed guidance for each step (links to separate articles).
     - Use for setting out guidance for complex processes.

       Useful for all levels of experience.
     - No example currently
   * - How-to
     - Procedure for how to do something.

       Not tied to a particular scenario or use case, though may advise on how to handle it within the detail of the steps.
     - Describe steps and options for a task but without tying explanations to a specific target goal.

       Best suited to explaining what to do and where to do it rather than exactly how to do it and why to do it that way.
     - :doc:`Implementing custom interfaces - how-to <../../../ROS-Framework/client-libraries/Working-with-Client-Libraries/Single-Package-Define-And-Use-Interface>`

       :doc:`Installing on Ubuntu - how-to <../../../Get-Started/Installation/Ubuntu-Install-Debs>`
   * - Tutorial
     - Detailed actions for achieving a particular scenario or use case, including an explanation of what is going on within each step, or why to do it.
     - Best suited to explaining exactly how to do something and why to do it that way.

       Good for complete beginners but also useful for more expert users, particularly for explaining how and why to follow best practice (experts may be more likely to follow the detail of substeps rather than the full tutorial).

       High effort to create and maintain.

       Before making a tutorial, consider if a how-to or example type article might do the job.
     - :doc:`Learning about topics - tutorial <../../../ROS-Framework/interfaces/topics/Understanding-ROS2-Topics/Understanding-ROS2-Topics>`
   * - Example
     - Simple article to make a demo example available to search and browse (detail is mainly in the example code in a separate location).
     - Suited to helping people find a demo example that they can use to understand and adapt for their needs.

       Expects a set of commented code in a separate location.

       If you want to provide step-by-step instructions for implementing and adapting the example, a tutorial is probably better.
     - :doc:`Managing node lifecycles - example <../../../ROS-Framework/nodes/Working-with-nodes/Managed-Nodes>`
   * - Reference
     - List and explain the detail of specifications.

       These will mostly be separate API docs but even for these it may be useful to have an article to help with findability.
     - Suitable for listing any reference information.

       The article normally shouldn't include instructional or conceptual information.

       The main reference docs will be API docs for packages.
     - This article

Related articles
----------------

* :doc:`Contributing to documentation<../../../The-ROS2-Project/Contributing/Contributing-to-documentation>`

* :doc:`Creating or updating documentation<../../../The-ROS2-Project/Contributing/Documentation/Creating-or-updating-documentation>`

FAQs
----

What metadata and directives are required in a ROS documentation article?
  Every ROS article must include a ``.. meta::`` directive with an ``:area:`` field.
  List ``:area:`` values most specific first; the first value is the primary value used for related content.
  Each article should also include the ``.. showmeta::`` and ``.. short-description::`` directives.
  A missing, empty, or disallowed ``:area:`` value causes the Enhance check on the pull request to fail.

How should I format sentences and white space in a ROS reStructuredText file?
  Write only one sentence per line to make the file easier to review and modify.
  Do not leave trailing white space, because the ROS documentation linter will not accept pull requests that contain it.

When should I use bash instead of console for a code block?
  Use bash for commands or content intended to be copied into a script file.
  Use console for commands that readers should run in a terminal, especially when you also need to show terminal output.
  In console blocks, prefix command lines with $ or # so the copy button copies commands rather than output.

When should I use ``:doc:`` and ``:ref:`` for internal links?
  Use ``:doc:`` to link to another article in the ROS documentation, using its relative path.
  Use ``:ref:`` to link to a defined target such as a heading, image, or code section in the same article or another article.
