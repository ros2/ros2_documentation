Contributing to ROS 2 Documentation
===================================

.. contents:: Table of Contents
   :depth: 2
   :local:

This page explains how to contribute to ROS 2 Documentation.

Building documentation locally
------------------------------

The source code of documentation is located in the `ROS 2 Documentation Github page <https://github.com/ros2/ros2_documentation>`_.
Fork the repository, clone and navigate to the root folder.
In order to build the documentation locally and test it install requirements located in the ``requirements.txt`` file.

.. code-block:: console
   
   pip3 install --user --upgrade -r requirements.txt

Now documentation can be built locally 

.. code-block:: console

   make html

The build process can take some time.
In order to see the output, open ``build/html/index.html`` in your browser.

..
   Depending 

   Installation

   Tutorials

   How-To-Guides

   Concepts

rst Guide
---------

Table of Contents
^^^^^^^^^^^^^^^^^

There are two types of directives used for generation of tables of contents, ``.. toctree::`` and ``.. contents::``.
The ``.. toctree::`` is used in a top level pages like ``Tutorials.rst`` to set ordering and visibility of its child pages.
This directive creates both left navigation panel and in-page navigation links to the child pages listed.

.. code-block:: rst

   .. toctree::
      :maxdepth: 1

The ``.. contents::`` directive is used for generation of table of contents for each individual page.
It parses the all existing headings in a page and build an in-page nested table of contents.
It helps readers to navigate in a page.

The ``.. contents::`` directive supports definition of maximum depth of nested sections.
Using ``:depth: 2`` will only show Sections and Subsections.

.. code-block:: rst

   .. contents:: Table of Contents
      :depth: 2
      :local:

Headings
^^^^^^^^^^^^^^^^^

There are four main Heading types used in the documentation.
Number of symbols has to match the length of the title.

.. code-block:: rst

   Page Title Header
   =================

   Section Header
   --------------

   Subsection Header
   ^^^^^^^^^^^^^^^^^

   Subsubsection Header
   ~~~~~~~~~~~~~~~~~~~~

Lists
^^^^^

Use ``*`` for unordered lists with bullet points and ``#.`` for listing numbered items.

.. code-block:: rst

   * bullet point
   * bullet point
  
     * bullet point nested
     * bullet point nested

   * bullet point

.. code-block:: rst

  #. first listed item
  #. second lited item

Code Formatting
^^^^^^^^^^^^^^^

.. code-block:: rst

   In-text code can be formatted using ``backticks`` for showing ``highlighted`` code.

Code blocks inside a page needs to be captured using ``.. code-block::`` directive.
``.. code-block::`` supports highlighting code for various syntaxes like ``C++``, ``YAML``, ``console``, ``bash``, and more.
Code that is used inside directive needs to be indented.

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

Images can inserted using the ``.. image::`` directive.

.. code-block:: rst

   .. image:: images/turtlesim_follow1.png

References and Links
^^^^^^^^^^^^^^^^^^^^

External links
~~~~~~~~~~~~~~

The syntax is for creating links to external web-pages is 

.. code-block:: rst

   `ROS Docs <https://docs.ros.org>`_

which appear as `ROS Docs <https://docs.ros.org>`_.
Note the underscore after the final single quote.


Internal links
~~~~~~~~~~~~~~

The ``:doc:`` directive is used to create links to other pages.
The syntax is shown below.
Note that the relative path to the file is used.

.. code-block:: rst

   :doc:`Quality of Service <../Tutorials/Quality-of-Service>`

The ``ref`` directive is used to make links to specific parts of current or other pages.
This could be headings, images or code sections.


For this you will need to define an explicit target before a desired object.

.. code-block:: rst

   .. _talker-listener:

   Try some examples
   -----------------

Now you can create links to that header as shown below.

.. code-block:: rst

   :ref:`talker-listener demo <talker-listener>`