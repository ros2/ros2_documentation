.. _CodeStyle:

Code style and language versions
================================

.. contents:: Table of Contents
   :depth: 2
   :local:

In order to achieve a consistent looking product we will all follow externally (if possible) defined style guidelines for each language.
For other things like package layout or documentation layout we will need to come up with our own guidelines, drawing on current, popular styles in use now.

Additionally, wherever possible, developers should use integrated tools to allow them to check that these guidelines are followed in their editors.
For example, everyone should have a PEP8 checker built into their editor to cut down on review iterations related to style.

Also where possible, packages should check style as part of their unit tests to help with the automated detection of style issues (see `ament_lint_auto <https://github.com/ament/ament_lint/blob/master/ament_lint_auto/doc/index.rst>`__).

C
-

Standard
^^^^^^^^

We will target C99.

Style
^^^^^

We will use `Python's PEP7 <https://www.python.org/dev/peps/pep-0007/>`__ for our C style guide, with some modifications and additions:

* We will target C99, as we do not need to support C89 (as PEP7 recommends)

  * rationale: among other things it allows us to use both ``//`` and ``/* */`` style comments
  * rationale: C99 is pretty much ubiquitous now

* C++ style ``//`` comments are allowed
* Always place literals on the left-hand side of comparison operators, e.g. ``0 == ret`` instead of ``ret == 0``

  * rationale: ``ret == 0`` too easily turns into ``ret = 0`` by accident

All of the following modifications only apply if we are not writing Python modules:

* Do not use ``Py_`` as a prefix for everything

  * instead use a CamelCase version of the package name or other appropriate prefix

* The stuff about documentation strings doesn't apply

We can use the `pep7 <https://github.com/mike-perdide/pep7>`__ python module for style checking. The editor integration seems slim, we may need to look into automated checking for C in more detail.

C++
---

Standard
^^^^^^^^

We will target C++14, using new built-in C++14 features over Boost equivalents wherever possible.

Style
^^^^^


We will use the `Google C++ Style Guide <https://google.github.io/styleguide/cppguide.html>`__, with some modifications:

Line Length
~~~~~~~~~~~

* Our maximum line length is 100 characters.

File Extensions
~~~~~~~~~~~~~~~

* Header files should use the .hpp extension.

  * rationale: Allow tools to determine content of files, C++ or C.

* Implementation files should use the .cpp extension.

  * rationale: Allow tools to determine content of files, C++ or C.

Variable Naming
~~~~~~~~~~~~~~~

* For global variables use lowercase with underscores prefixed with ``g_``

  * rationale: keep variable naming case consistent across the project
  * rationale: easy to tell the scope of a variable at a glance
  * consistency across languages

Function and Method Naming
~~~~~~~~~~~~~~~~~~~~~~~~~~

* Google style guide says ``CamelCase``, but the C++ std library's style of ``snake_case`` is also allowed

  * rationale: ROS 2 core packages currently use ``snake_case``

    * reason: either an historical oversight or a personal preference that didn't get checked by the linter
    * reason for not changing: retroactively changing would be too disruptive
  * other considerations:

    * ``cpplint.py`` does not check this case (hard to enforce other than with review)
    * ``snake_case`` can result in more consistency across languages
  * specific guidance:

    * for existing projects, prefer the existing style
    * for new projects, either is acceptable, but a preference for matching related existing projects is advised
    * final decision is always developer discretion

      * special cases like function pointers, callable types, etc. may require bending the rules
    * Note that classes should still use ``CamelCase`` by default

Access Control
~~~~~~~~~~~~~~

* Drop requirement for all class members to be private and therefore require accessors

  * rationale: this is overly constraining for user API design
  * we should prefer private members, only making them public when they are needed
  * we should consider using accessors before choosing to allow direct member access
  * we should have a good reason for allowing direct member access, other than because it is convenient for us

Exceptions
~~~~~~~~~~

* Exceptions are allowed

  * rationale: this is a new code base, so the legacy argument doesn't apply to us
  * rationale: for user-facing API's it is more idiomatic C++ to have exceptions
  * Exceptions in destructors should be explicitly avoided

* We should consider avoiding Exceptions if we intend to wrap the resulting API in C

  * rationale: it will make it easier to wrap in C
  * rationale: most of our dependencies in code we intend to wrap in C do not use exceptions anyways

Function-like Objects
~~~~~~~~~~~~~~~~~~~~~

* No restrictions on Lambda's or ``std::function`` or ``std::bind``

Boost
~~~~~

* Boost should be avoided until absolutely required

Comments and Doc Comments
~~~~~~~~~~~~~~~~~~~~~~~~~

* Use ``///`` and ``/** */`` comments for *documentation* purposes and ``//`` style comments for notes and general comments

  * Class and Function comments should use ``///`` and ``/** */`` style comments
  * rationale: these are recommended for Doxygen and Sphinx in C/C++
  * rationale: mixing ``/* */`` and ``//`` is convenient for block commenting out code which contains comments
  * Descriptions of how the code works or notes within classes and functions should use ``//`` style comments

Pointer Syntax Alignment
~~~~~~~~~~~~~~~~~~~~~~~~

* Use ``char * c;`` instead of ``char* c;`` or ``char *c;`` because of this scenario ``char* c, *d, *e;``

Class Privacy Keywords
~~~~~~~~~~~~~~~~~~~~~~

* Do not put 1 space before ``public:``, ``private:``, or ``protected:``, it is more consistent for all indentions to be a multiple of 2

  * rationale: most editors don't like indentions which are not a multiple of the (soft) tab size
  * Use zero spaces before ``public:``, ``private:``, or ``protected:``, or 2 spaces
  * If you use 2 spaces before, indent other class statements by 2 additional spaces
  * Prefer zero spaces, i.e. ``public:``, ``private:``, or ``protected:`` in the same column as the class

Nested Templates
~~~~~~~~~~~~~~~~

* Never add whitespace to nested templates

  * Prefer ``set<list<string>>`` (C++11 feature) to ``set<list<string> >`` or ``set< list<string> >``

Always Use Braces
~~~~~~~~~~~~~~~~~

* Always use braces following ``if``, ``else``, ``do``, ``while``, and ``for``, even when the body is a single line.

  * rationale: less opportunity for visual ambiguity and for complications due to use of macros in the body

Open Versus Cuddled Braces
~~~~~~~~~~~~~~~~~~~~~~~~~~

* Use open braces for ``function``, ``class``, and ``struct`` definitions, but cuddle braces on ``if``, ``else``, ``while``, ``for``, etc...

  * Exception: when an ``if`` (or ``while``, etc.) condition is long enough to require line-wrapping, then use an open brace (i.e., don't cuddle).

* When a function call cannot fit on one line, wrap at the open parenthesis (not in between arguments) and start them on the next line with a 2-space indent.  Continue with the 2-space indent on subsequent lines for more arguments.  (Note that the `Google style guide <https://google.github.io/styleguide/cppguide.html#Function_Calls>`__ is internally contradictory on this point.)

  * Same goes for ``if`` (and ``while``, etc.) conditions that are too long to fit on one line.

Examples
~~~~~~~~

This is OK:

.. code-block:: c++

   int main(int argc, char **argv)
   {
     if (condition) {
       return 0;
     } else {
       return 1;
     }
   }

   if (this && that || both) {
     ...
   }

   // Long condition; open brace
   if (
     this && that || both && this && that || both && this && that || both && this && that)
   {
     ...
   }

   // Short function call
   call_func(foo, bar);

   // Long function call; wrap at the open parenthesis
   call_func(
     foo, bar, foo, bar, foo, bar, foo, bar, foo, bar, foo, bar, foo, bar, foo, bar, foo, bar,
     foo, bar, foo, bar, foo, bar, foo, bar, foo, bar, foo, bar, foo, bar, foo, bar, foo, bar);

   // Very long function argument; separate it for readability
   call_func(
     bang,
     fooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo,
     bar, bat);

This is **not** OK:

.. code-block:: c++

   int main(int argc, char **argv) {
     return 0;
   }

   if (this &&
       that ||
       both) {
     ...
   }


Use open braces rather than excessive indention, e.g. for distinguishing constructor code from constructor initializer lists

This is OK:

.. code-block:: c++

   ReturnType LongClassName::ReallyReallyReallyLongFunctionName(
     Type par_name1,  // 2 space indent
     Type par_name2,
     Type par_name3)
   {
     DoSomething();  // 2 space indent
     ...
   }

   MyClass::MyClass(int var)
   : some_var_(var),
     some_other_var_(var + 1)
   {
     ...
     DoSomething();
     ...
   }

This is **not** OK, even weird (the google way?):

.. code-block:: c++

   ReturnType LongClassName::ReallyReallyReallyLongFunctionName(
       Type par_name1,  // 4 space indent
       Type par_name2,
       Type par_name3) {
     DoSomething();  // 2 space indent
     ...
   }

   MyClass::MyClass(int var)
       : some_var_(var),             // 4 space indent
         some_other_var_(var + 1) {  // lined up
     ...
     DoSomething();
     ...
   }

Linters
~~~~~~~

Most of these styles and restrictions can be checked with a combination of Google's `cpplint.py <http://google-styleguide.googlecode.com/svn/trunk/cpplint/>`__ and `uncrustify <https://github.com/uncrustify/uncrustify>`__, though we may need to modify them slightly for our above changes.

We provide command line tools with custom configurations:

* `ament_cpplint <https://github.com/ament/ament_lint/blob/master/ament_cpplint/doc/index.rst>`__
* `ament_uncrustify <https://github.com/ament/ament_lint/blob/master/ament_uncrustify/doc/index.rst>`__: `configuration <https://github.com/ament/ament_lint/blob/master/ament_uncrustify/ament_uncrustify/configuration/ament_code_style.cfg>`__

We also run other tools to detect and eliminate as many warnings as possible.
Here's a non-exhaustive list of additional things we try to do on all of our packages:

* use compiler flags like ``-Wall -Wextra -Wpedantic``
* run static code analysis like ``cppcheck``, which we have integrated in `ament_cppcheck <https://github.com/ament/ament_lint/blob/master/ament_cppcheck/doc/index.rst>`__.

Python
------

Version
^^^^^^^

We will target Python 3 for our development.

Style
^^^^^

We will use the `PEP8 guidelines <https://www.python.org/dev/peps/pep-0008/>`_ for code format.

We chose the following more precise rule where PEP 8 leaves some freedom:

* `We allow up to 100 characters per line (fifth paragraph) <https://www.python.org/dev/peps/pep-0008/#maximum-line-length>`_.
* `We pick single quotes over double quotes as long as no escaping is necessary <https://www.python.org/dev/peps/pep-0008/#string-quotes>`_.
* `We prefer hanging indents for continuation lines <https://www.python.org/dev/peps/pep-0008/#indentation>`_.

Tools like the ``(ament_)pycodestyle`` Python package should be used in unit-test and/or editor integration for checking Python code style.

The pycodestyle configuration used in the linter is `here <https://github.com/ament/ament_lint/blob/master/ament_pycodestyle/ament_pycodestyle/configuration/ament_pycodestyle.ini>`__.

Integration with editors:

* atom: https://atom.io/packages/linter-pycodestyle
* emacs: http://kwbeam.com/emacs-for-python-i.html
* Sublime Text: https://sublime.wbond.net/packages/SublimeLinter-flake8
* vim: https://github.com/nvie/vim-flake8

CMake
-----

Version
^^^^^^^

We will target CMake 3.5.

Style
^^^^^

Since there is not an existing CMake style guide we will define our own:

* Use lowercase command names (``find_package``, not ``FIND_PACKAGE``).
* Use ``snake_case`` identifiers (variables, functions, macros).
* Use empty ``else()`` and ``end...()`` commands.
* No whitespace before ``(``\ 's.
* Use two spaces of indention, do not use tabs.
* Do not use aligned indentation for parameters of multi-line macro invocations. Use two spaces only.
* Prefer functions with ``set(PARENT_SCOPE)`` to macros.
* When using macros prefix local variables with ``_`` or a reasonable prefix.

Markdown / reStructured Text / docblocks
----------------------------------------

Style
^^^^^

The following rules to format text is intended to increase readability as well as versioning.

* *[.md, .rst only]* Each section title should be preceded by one empty line and succeeded by one empty line.

  * Rationale: It expedites to get an overview about the structure when screening the document.

* *[.rst only]* In reStructured Text the headings should follow the hierarchy described in the `Sphinx style guide <https://documentation-style-guide-sphinx.readthedocs.io/en/latest/style-guide.html#headings>`__:

  * ``#`` with overline (only once, used for the document title)
  * ``*`` with overline
  * ``=``
  * ``-``
  * ``^``
  * ``"``
  * Rationale: A consistent hierarchy expedites getting an idea about the nesting level when screening the document.

* *[.md only]* In Markdown the headings should follow the ATX-style described in the `Markdown syntax documentation <https://daringfireball.net/projects/markdown/syntax#header>`__

  * ATX-style headers use 1-6 hash characters (``#``) at the start of the line to denote header levels 1-6.
  * A space between the hashes and the header title should be used (such as ``# Heading 1``) to make it easier to visually separate them.
  * Justification for the ATX-style preference comes from the `Google Markdown style guide <https://github.com/google/styleguide/blob/gh-pages/docguide/style.md#atx-style-headings>`__
  * Rationale: ATX-style headers are easier to search and maintain, and make the first two header levels consistent with the other levels.

* *[any]* Each sentence must start on a new line.

  * Rationale: For longer paragraphs a single change in the beginning makes the diff unreadable since it carries forward through the whole paragraph.

* *[any]* Each sentence can optionally be wrapped to keep each line short.
* *[any]* The lines should not have any trailing white spaces.
* *[.md, .rst only]* A code block must be preceded and succeeded by an empty line.

  * Rationale: Whitespace is significant only directly before and directly after fenced code blocks.
    Following these instructions will ensure that highlighting works properly and consistently.

* *[.md, .rst only]* A code block should specify a syntax (e.g. ``bash``).

Javascript
----------

*(Speculative, not yet used)*

Version
^^^^^^^

We will target Javascript 1.5, which seems to provide the best balance of support in browsers and languages (node.js) and new features.

Style
^^^^^

We will use the `airbnb Javascript Style guide <https://github.com/airbnb/javascript>`__.

The repository referred to above comes with a ``jshintrc`` file which allows the style to be enforced using ``jshint``.

Editor integration for ``jshint`` include ``vim``, ``emacs``, ``Sublime Text``, and others can be found `here <http://www.jshint.com/install/>`__.
