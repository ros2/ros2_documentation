.. redirect-from::

    Developer-Guide

ROS 2 Developer Guide
=====================

.. contents:: Table of Contents
   :depth: 2
   :local:

This page defines the practices and policies we employ when developing ROS 2.

General Principles
------------------

Some principles are common to all ROS 2 development:


* **Shared ownership**: Everybody working on ROS2 should feel ownership over all parts of the system.
  The original author of a chunk of code does not have any special permission or obligation to control or maintain that chunk of code.
  Everyone is free to propose changes anywhere, to handle any type of ticket, and to review any pull request.
* **Be willing to work on anything**: As a corollary to shared ownership, everybody should be willing to take on any available task and contribute to any aspect of the system.
* **Ask for help**: If you run into trouble on something, ask your fellow developers for help, via tickets, comments, or email, as appropriate.

General Practices
-----------------

Some practices are common to all ROS 2 development:

Issues
^^^^^^

When filing an issue please make sure to:

- Include enough information for another person to understand the issue.
  In ROS 2, the following points are needed for narrowing down the cause of an issue. Testing with as many alternatives in each category as feasible will be especially helpful.
  - **The operating system and version.** Reasoning: ROS 2 supports multiple platforms, and some bugs are specific to particular versions of operating systems/compilers.
  - **The installation method.** Reasoning: Some issues only manifest if ROS 2 has been installed from "fat archives" or from Debians. This can help us determine if the issue is with the packaging process.
  - **The specific version of ROS 2.** Reasoning: Some bugs may be present in a particular ROS 2 release and later fixed. It is important to know if your installation includes these fixes.
  - **The DDS/RMW implementation being used** (see `this page <../Tutorials/Working-with-multiple-RMW-implementations>` for how to determine which one). Reasoning: Communication issues may be specific to the underlying ROS middleware being used.
  - **The ROS 2 client library being used.** Reasoning: This helps us narrow down the layer in the stack at which the issue might be.

- Include a list of steps to reproduce the issue.
- In case of a bug consider to provide a `short, self contained, correct (compilable), example <http://sscce.org/>`__. Issues are much more likely to be resolved if others can reproduce them easily.
- Mention troubleshooting steps that have been tried already, including:
  - Upgrading to the latest version of the code, which may include bug fixes that have not been released yet. See `this section <building-from-source>` and follow the instructions to get the "master" branches.
  - Trying with a different RMW implementation. See `this page <../Tutorials/Working-with-multiple-RMW-implementations>` for how to do that.

Pull requests
^^^^^^^^^^^^^

* A pull request should only focus on one change.
  Separate changes should go into separate pull requests.
  See `GitHub's guide to writing the perfect pull request <https://github.com/blog/1943-how-to-write-the-perfect-pull-request>`__.

  * For some ROS 2 repositories the `Developer Certificate of Origin (DCO) <https://developercertificate.org/>`_ is enforced on pull requests.
    It requires all commit messages to contain the ``Signed-off-by`` line with an email address that matches the commit author.
    You can pass ``-s`` / ``--signoff`` to the ``git commit`` invocation or write the expected message manually (e.g. ``Signed-off-by: Your Name Developer <your.name@example.com>``).

* A patch should be minimal in size and avoid any kind of unnecessary changes.
* Always run CI jobs for all platforms for every pull request and include links to jobs in the pull request.
  (If you don't have access to the Jenkins job someone will trigger the jobs for you.)

* A pull request must contain minimum number of meaningful commits.

  * You can create new commits while the pull request is under review.

* Before merging a pull request all changes should be squashed into a small number of semantic commits to keep the history clear.

  * But avoid squashing commits while a pull request is under review.
    Your reviewers might not notice that you made the change, thereby introducing potential for confusion.
    Plus, you're going to squash before merging anyway; there's no benefit to doing it early.

* A minimum of 1 ``+1`` from a fellow developer is required to consider a pull request to be approved, which is required before merging.
* Any developer is welcome to review and approve a pull request (see `General Principles`_).
* When you start reviewing a pull request, comment on the pull request so that other developers know that you're reviewing it.
* Pull-request review is not read-only, with the reviewer making comments and then waiting for the author to address them.
  As a reviewer, feel free to make minor improvements (typos, style issues, etc.) in-place.
  As the opener of a pull-request, if you are working in a fork, checking the box to `allow edits from upstream contributors <https://github.com/blog/2247-improving-collaboration-with-forks>`__ will assist with the aforementioned.
  As a reviewer, also feel free to make more substantial improvements, but consider putting them in a separate branch (either mention the new branch in a comment, or open another pull request from the new branch to the original branch).
* Any developer (the author, the reviewer, or somebody else) can merge any approved pull request.

Development Process
^^^^^^^^^^^^^^^^^^^

* The default branch (in most cases the master branch) must always build, pass all tests and compile without warnings.
  If at any time there is a regression it is the top priority to restore at least the previous state.
* Always build with tests enabled.
* Always run tests locally after changes and before proposing them in a pull request.
  Besides using automated tests, also run the modified code path manually to ensure that the patch works as intended.
* Always run CI jobs for all platforms for every pull request and include links to the jobs in the pull request.

For more details on recommended software development workflow, see `Software Development Lifecycle`_ section.

Changes to RMW API
^^^^^^^^^^^^^^^^^^

When updating `RMW API <https://github.com/ros2/rmw>`__, it is required that RMW implementations for the Tier 1 middleware libraries are updated as well.
For example, a new function ``rmw_foo()`` introduced to the RMW API must be implemented in the following packages (as of ROS Crystal):

* `rmw_fastrtps <https://github.com/ros2/rmw_fastrtps/tree/master/rmw_fastrtps_cpp>`__
* `rmw_connext <https://github.com/ros2/rmw_connext>`__

Updates for non-Tier 1 middleware libraries should also be considered if feasible (e.g. depending on the size of the change).
See `REP-2000 <http://www.ros.org/reps/rep-2000.html#crystal-clemmys-december-2018-december-2019>`__ for the list of middleware libraries and their tiers.

kanban board (waffle.io)
^^^^^^^^^^^^^^^^^^^^^^^^

To help organize the work, the core ROS 2 development team is using a kanban system hosted at waffle.io: `ROS 2 kanban <https://waffle.io/ros2/ros2>`__.
This board augments the capabilities of GitHub by using labels to give a custom view into issues and pull requests across multiple repositories.
The data produced and edited via waffle.io are stored in the underlying GitHub objects, so there's no requirement to use waffle.io (or for the core team to be tied to it); it just provides a useful perspective on things.

Here's how we're using the columns in the board:

* **Backlog**: cards (issues) that nobody is yet working on.
  Their order in the backlog is an approximate indicator of priority, with cards higher in the column having higher priority.
* **Ready**: cards on which work will be started very soon.
  Cards in this column should have an owner assigned.
  Cards should not sit in this column for more than a few days.
* **In Progress**: cards on which work is currently in progress.
  Cards in this column must have an owner assigned.
  Cards should not sit in this column for more than a week.
  When it is determined that a card will take longer, break it up into multiple cards and put the extras in the backlog.
* **In Review**: cards for which the work is done and the relevant pull request/s is/are ready for review.
  Cards remain in this column during review, but if review uncovers significant extra work to be done, move the card into an earlier column as appropriate.
* **Done**: cards for which the work is done, meaning that the relevant pull request/s has/have been merged.
  This column shows recently completed cards, for informational purposes only.

Tips for working with the kanban board:

* Requesting permission to make changes. Simply comment on specific tickets that you want to work on it. Depending on the complexity it might be useful to describe how you want to address it. We will update the status (if you don't have the permission) and you can start working on a pull request. If you contribute regularly we will likely just grant you permission to manage the labels etc. yourself.
* Using markup to connect issues and pull requests (see the `waffle.io FAQ <https://github.com/waffleio/waffle.io/wiki/FAQs#prs-connect-keywords>`__).
* Doing equivalent things outside waffle.io, directly via GitHub. The column a card is in is determined by the label. The first and last column do not require a specific label. For the other column a label with the same name can be assigned.

Programming conventions
^^^^^^^^^^^^^^^^^^^^^^^

* Defensive programming: ensure that assumptions are held as early as possible.
  E.g. check every return code and make sure to at least throw an exception until the case is handled more gracefully.
* All error messages must be directed to ``stderr``.
* Declare variables in the narrowest scope possible.
* Keep group of items (dependencies, imports, includes, etc.) ordered alphabetically.

C++ specific
~~~~~~~~~~~~

* Avoid using direct streaming (``<<``) to ``stdout`` / ``stderr`` to prevent interleaving between multiple threads.
* Avoid using references for ``std::shared_ptr`` since that subverts the reference counting. If the original instance goes out of scope and the reference is being used it accesses freed memory.

Software Development Lifecycle
------------------------------

This section describes step-by-step how to plan, design, and implement a new feature:

1. Task Creation
2. Creating the Design Document
3. Design Review
4. Implementation
5. Code Review

Task creation
^^^^^^^^^^^^^

Tasks requiring changes to critical parts of ROS 2 should have design reviews during early stages of the release cycle.
If a design review is happening in the later stages, the changes will be part of a future release.

* An issue should be created in the appropriate `ros2 repository <https://github.com/ros2/>`__, clearly describing the task being worked on.

  * It should have a clear success criteria and highlight the concrete improvements expected from it.
  * If the feature is targeting a ROS release, ensure this is tracked in the ROS release ticket (`example <https://github.com/ros2/ros2/issues/607>`__).

Writing the design document
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Design docs must never include confidential information.
Whether or not a design document is required for your change depends on how big the task is.

1. You are making a small change or fixing a bug:

  * A design document is not required, but an issue should be opened in the appropriate repository to track the work and avoid duplication of efforts.

2. You are implementing a new feature or would like to contribute to OSRF-owned infrastructure (like Jenkins CI):

  * Design doc is required and should be contributed to `ros2/design <https://github.com/ros2/design/>`__ to be made accessible on http://design.ros2.org/.
  * You should fork the repository and submit a pull request detailing the design.

  Mention the related ros2 issue (for example, ``Design doc for task ros2/ros2#<issue id>``) in the pull request or the commit message.
  Detailed instructions are on the `ROS2 Contribute <http://design.ros2.org/contribute.html>`__ page.
  Design comments will made directly on the pull request.

If the task is planned to be released with a specific version of ROS, this information should be included in the pull request.

Design document review
^^^^^^^^^^^^^^^^^^^^^^

Once the design is ready for review, a pull request should be opened and appropriate reviewers should be assigned.
It is recommended to include project owner(s) -
maintainers of all impacted packages (as defined by ``package.xml`` maintainer field, see `REP-140 <http://www.ros.org/reps/rep-0140.html#maintainer-multiple-but-at-least-one>`__) - as reviewers.

* If the design doc is complex or reviewers have conflicting schedules, an optional design review meeting can be setup. In this case,

  **Before the meeting**

  * Send a meeting invite at least one week in advance
  * Meeting duration of one hour is recommended
  * Meeting invite should list all decisions to be made during the review (decisions requiring package maintainer approval)
  * Meeting required attendees: design pull request reviewers
      Meeting optional attendees: all OSRF engineers, if applicable

  **During the meeting**

  * The task owner drives the meeting, presents their ideas and manages discussions to ensure an agreement is reached on time

  **After the meeting**

  * The task owner should send back meeting notes to all attendees
  * If minor issues have been raised about the design:

    * The task owner should update the design doc pull request based on the feedback
    * Additional review is not required

  * If major issues have been raised about the design:

    * It is acceptable to remove sections for which there is no clear agreement
    * The debatable parts of the design can be resubmitted as a separate task in the future
    * If removing the debatable parts is not an option, work directly with package owners to reach an agreement

* Once consensus is reached:

  * Ensure the `ros2/design <https://github.com/ros2/design/>`__ pull request has been merged, if applicable
  * Update and close the github issue associated with this design task

Implementation
^^^^^^^^^^^^^^

Before starting, go through the `Pull requests`_ section for best practices.

* For each repo to be modified:

  * Modify the code, go to the next step if finished or at regular interval to backup your work.
  * `Self review <https://git-scm.com/book/en/v2/Git-Tools-Interactive-Staging>`__ your changes using ``git add -i``.
  * Create a new signed commit using ``git commit -s``.

    * A pull request should contain minimal semantically meaningful commits (for instance, a large number of 1-line commits is not acceptable).
      Create new fixup commits while iterating on feedback, or optionally, amend existing commits using ``git commit --amend`` if you don't want to create a new commit every time.
    * Each commit must have a properly written, meaningful, commit message.
      More instructions `here <https://chris.beams.io/posts/git-commit/>`__.
    * Moving files must be done in a separate commit, otherwise git may fail to accurately track the file history.
    * Either the pull request description or the commit message must contain a reference to the related ros2 issue, so it gets automatically closed when the pull request is merged.
      See this `doc <https://help.github.com/articles/closing-issues-using-keywords/>`__ for more details.
    * Push the new commits.

Code Review
^^^^^^^^^^^

Once the change is ready for code review:

* Open a pull request for each modified repository.

  * Remember to follow `Pull requests`_ best practices.
  * `hub <https://hub.github.com/>`__ can be used to create pull requests from the command line.
  * If the task is planned to be released with a specific version of ROS, this information should be included in each pull request.

* Package owners who reviewed the design document should be mentioned in the pull request.
* Code review SLO: although reviewing pull requests is best-effort,
  it is helpful to have reviewers comment on pull requests within a week and
  code authors to reply back to comments within a week, so there is no loss of context.
* Iterate on feedback as usual, amend and update the development branch as needed.
* Once the PR is approved, package maintainers will merge the changes in.

Language Versions and Code Format
---------------------------------

In order to achieve a consistent looking product we will all follow externally (if possible) defined style guidelines for each language.
For other things like package layout or documentation layout we will need to come up with our own guidelines, drawing on current, popular styles in use now.

Additionally, where ever possible, developers should use integrated tools to allow them to check that these guidelines are followed in their editors.
For example, everyone should have a PEP8 checker built into their editor to cut down on review iterations related to style.

Also where possible, packages should check style as part of their unit tests to help with the automated detection of style issues (see `ament_lint_auto <https://github.com/ament/ament_lint/blob/master/ament_lint_auto/doc/index.rst>`__).

C
^

Standard
~~~~~~~~


We will target C99.

Style
~~~~~


We will use `Python's PEP7 <https://www.python.org/dev/peps/pep-0007/>`__ for our C style guide, with some modifications and additions:

* We will target C99, as we do not need to support C89 (as PEP7 recommends)

  * rationale: among other things it allows us to use both ``//`` and ``/* */`` style comments
  * rationale: C99 is pretty much ubiquitous now

* C++ style ``//`` comments are allowed
* Always place literals on the left hand side of comparison operators, e.g. ``0 == ret`` instead of ``ret == 0``

  * rationale: ``ret == 0`` too easily turns into ``ret = 0`` by accident

All of the following modifications only apply if we are not writing Python modules:

* Do not use ``Py_`` as a prefix for everything

  * instead use a CamelCase version of the package name or other appropriate prefix

* The stuff about documentation strings doesn't apply

We can use the `pep7 <https://github.com/mike-perdide/pep7>`__ python module for style checking. The editor integration seems slim, we may need to look into automated checking for C in more detail.

C++
^^^


Standard
~~~~~~~~


We will target C++14, using new built-in C++14 features over Boost equivalents where ever possible.

Style
~~~~~


We will use the `Google C++ Style Guide <https://google.github.io/styleguide/cppguide.html>`__, with some modifications:

Line Length
"""""""""""


* Our maximum line length is 100 characters.

Variable Naming
"""""""""""""""

* For global variables use lowercase with underscores prefixed with ``g_``

  * rationale: keep variable naming case consistent across the project
  * rationale: easy to tell the scope of a variable at a glance
  * consistency across languages

Function and Method Naming
""""""""""""""""""""""""""


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
""""""""""""""


* Drop requirement for all class members to be private and therefore require accessors

  * rationale: this is overly constraining for user API design
  * we should prefer private members, only making them public when they are needed
  * we should consider using accessors before choosing to allow direct member access
  * we should have a good reason for allowing direct member access, other than because it is convenient for us

Exceptions
""""""""""


* Exceptions are allowed

  * rationale: this is a new code base, so the legacy argument doesn't apply to us
  * rationale: for user facing API's it is more idiomatic C++ to have exceptions
  * Exceptions in destructors should be explicitly avoided

* We should consider avoiding Exceptions if we intend to wrap the resulting API in C

  * rationale: it will make it easier to wrap in C
  * rationale: most of our dependencies in code we intend to wrap in C do not use exceptions anyways

Function-like Objects
"""""""""""""""""""""


* No restrictions on Lambda's or ``std::function`` or ``std::bind``

Boost
"""""


* Boost should be avoided until absolutely required

Comments and Doc Comments
"""""""""""""""""""""""""


* Use ``///`` and ``/** */`` comments for *documentation* purposes and ``//`` style comments for notes and general comments

  * Class and Function comments should use ``///`` and ``/** */`` style comments
  * rationale: these are recommended for Doxygen and Sphinx in C/C++
  * rationale: mixing ``/* */`` and ``//`` is convenient for block commenting out code which contains comments
  * Descriptions of how the code works or notes within classes and functions should use ``//`` style comments

Pointer Syntax Alignment
""""""""""""""""""""""""


* Use ``char * c;`` instead of ``char* c;`` or ``char *c;`` because of this scenario ``char* c, *d, *e;``

Class Privacy Keywords
""""""""""""""""""""""


* Do not put 1 space before ``public:``, ``private:``, or ``protected:``, it is more consistent for all indentions to be a multiple of 2

  * rationale: most editors don't like indentions which are not a multiple of the (soft) tab size
  * Use zero spaces before ``public:``, ``private:``, or ``protected:``, or 2 spaces
  * If you use 2 spaces before, indent other class statements by 2 additional spaces
  * Prefer zero spaces, i.e. ``public:``, ``private:``, or ``protected:`` in the same column as the class

Nested Templates
""""""""""""""""


* Never add whitespace to nested templates

  * Prefer ``set<list<string>>`` (C++11 feature) to ``set<list<string> >`` or ``set< list<string> >``

Always Use Braces
"""""""""""""""""


* Always use braces following ``if``, ``else``, ``do``, ``while``, and ``for``, even when the body is a single line.

  * rationale: less opportunity for visual ambiguity and for complications due to use of macros in the body

Open Versus Cuddled Braces
""""""""""""""""""""""""""


* Use open braces for ``function``, ``class``, and ``struct`` definitions, but cuddle braces on ``if``, ``else``, ``while``, ``for``, etc...

  * Exception: when an ``if`` (or ``while``, etc.) condition is long enough to require line-wrapping, then use an open brace (i.e., don't cuddle).

* When a function call cannot fit on one line, wrap at the open parenthesis (not in between arguments) and start them on the next line with a 2-space indent.  Continue with the 2-space indent on subsequent lines for more arguments.  (Note that the `Google style guide <https://google.github.io/styleguide/cppguide.html#Function_Calls>`__ is internally contradictory on this point.)

  * Same goes for ``if`` (and ``while``, etc.) conditions that are too long to fit on one line.

Examples
""""""""

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
"""""""

Most of these styles and restrictions can be checked with a combination of Google's `cpplint.py <http://google-styleguide.googlecode.com/svn/trunk/cpplint/>`__ and `uncrustify <https://github.com/uncrustify/uncrustify>`__, though we may need to modify them slightly for our above changes.

We provide command line tools with custom configurations:

* `ament_cpplint <https://github.com/ament/ament_lint/blob/master/ament_cpplint/doc/index.rst>`__
* `ament_uncrustify <https://github.com/ament/ament_lint/blob/master/ament_uncrustify/doc/index.rst>`__: `configuration <https://github.com/ament/ament_lint/blob/master/ament_uncrustify/ament_uncrustify/configuration/ament_code_style.cfg>`__

We also run other tools to detect and eliminate as many warnings as possible.
Here's a non-exhaustive list of additional things we try to do on all of our packages:

* use compiler flags like ``-Wall -Wextra -Wpedantic``
* run static code analysis like ``cppcheck``, which we have integrated in `ament_cppcheck <https://github.com/ament/ament_lint/blob/master/ament_cppcheck/doc/index.rst>`__.

Python
^^^^^^

Version
~~~~~~~

We will target Python 3 for our development.

Style
~~~~~

We will use the `PEP8 guidelines <https://www.python.org/dev/peps/pep-0008/>`_ for code format.

We chose the following more precise rule where PEP 8 leaves some freedom:

* `We allow up to 100 character per line (fifth paragraph) <https://www.python.org/dev/peps/pep-0008/#maximum-line-length>`_.
* `We pick single quotes over double quotes as long as no escaping is necessary <https://www.python.org/dev/peps/pep-0008/#string-quotes>`_.
* `We prefer hanging indents for continuation lines <https://www.python.org/dev/peps/pep-0008/#indentation>`_.

Tools like the ``(ament_)pep8`` Python package should be used in unit-test and/or editor integration for checking Python code style.

The pep8 configuration used in the linter is `here <https://github.com/ament/ament_lint/blob/master/ament_pep8/ament_pep8/configuration/ament_pep8.ini>`__.

Integration with editors:

* atom: https://atom.io/packages/linter-pep8
* emacs: http://kwbeam.com/emacs-for-python-i.html
* Sublime Text: https://sublime.wbond.net/packages/SublimeLinter-flake8
* vim: https://github.com/nvie/vim-flake8

CMake
^^^^^

Version
~~~~~~~

We will target CMake 3.5.

Style
~~~~~

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
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Style
~~~~~

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

* *[.md only]* In Markdown the headings should follow the atx-style described in the `Markdown syntax documentation <https://daringfireball.net/projects/markdown/syntax#header>`__

  * Atx-style headers use 1-6 hash characters (``#``) at the start of the line to denote header levels 1-6.
  * A space between the hashes and the header title should be used (such as ``# Heading 1``) to make it easier to visually separate them.
  * Justification for the ATX-style preference comes from the `Google Markdown style guide <https://github.com/google/styleguide/blob/gh-pages/docguide/style.md#atx-style-headings>`__
  * Rationale: Atx-style headers are easier to search and maintain, and make the first two header levels consistent with the other levels.

* *[any]* Each sentence must start on a new line.

  * Rationale: For longer paragraphs a single change in the beginning makes the diff unreadable since it carries forward through the whole paragraph.

* *[any]* Each sentence can optionally be wrapped to keep each line short.
* *[any]* The lines should not have any trailing white spaces.
* *[.md, .rst only]* A code block must be preceded and succeeded by an empty line.

  * Rationale: Whitespace is significant only directly before and directly after fenced code blocks.
    Following these instructions will ensure that highlighting works properly and consistently.

* *[.md, .rst only]* A code block should specify a syntax (e.g. ``bash``).

Javascript
^^^^^^^^^^

*(Speculative, not yet used)*

Version
~~~~~~~

We will target Javascript 1.5, which seems to provide the best balance of support in browsers and languages (node.js) and new features.

Style
~~~~~

We will use the `airbnb Javascript Style guide <https://github.com/airbnb/javascript>`__.

The repository referred to above comes with a ``jshintrc`` file which allows the style to be enforced using ``jshint``.

Editor integration for ``jshint`` include ``vim``, ``emacs``, ``Sublime Text``, and others can be found `here <http://www.jshint.com/install/>`__.

Testing
-------

All packages should have some level of tests.
Tests can be broken down into three main categories: System tests, Integration tests, and Unit tests.

Unit tests should always be in the package which is being tested and should make use of tools like ``Mock`` to try and test narrow parts of the code base in constructed scenarios.
Unit tests should not bring in test dependencies that are not testing tools, e.g. gtest, nosetest, pytest, mock, etc...

Integration tests can test interactions between parts of the code or between parts of the code and the system.
They often test software interfaces in ways that we expect the user to use them.
Like Unit tests, Integration tests should be in the package which is being tested and should not bring in non-tool test dependencies unless absolutely necessary, i.e. all non-tool dependencies should only be allowed under extreme scrutiny so they should be avoided if possible.

System tests are designed to test end-to-end situations between packages and should be in their own packages to avoid bloating or coupling packages and to avoid circular dependencies.

In general minimizing external or cross package test dependencies should be avoided to prevent circular dependencies and tightly coupled test packages.

All packages should have some unit tests and possibly integration tests, but the degree to which they should have them is based on the package's category (described later).

Test Coverage
^^^^^^^^^^^^^

Some packages should have a mechanism setup to capture test coverage information (if applicable to the language).
Coverage tools exist for some of the languages described here including C, C++, and Python, but possibly others.
When possible coverage should be measured in terms of branch coverage, as opposed to statement or function coverage.

Versioning
----------

*(Planned; not yet used)*

We will use the `Semantic Versioning guidelines <http://semver.org/>`__ for versioning.

Anything below version ``1.0.0`` is free to make changes at will and for most of our near-term development this will be the case.
In general though for versions less than ``1.0.0`` we should increment the ``minor`` (as ``major.minor.patch``) when we break existing API and increment ``patch`` for anything else.

Another part of adhering to the Semantic Versioning guidelines is that every package must declare a public API.
The declaration for most C and C++ packages is simple, it is any header that it installs, but it is acceptable to define a set of symbols which are considered private.
When ever possible having private symbols in public headers should be avoided.
For other languages like Python, a public API must be explicitly defined, so that it is clear what symbols can be relied on with respect to the versioning guidelines.
The public API can also be extended to build artifacts like configuration variables, CMake config files, etc. as well as executables and command line options and output.
Any elements of the public API should be clearly stated in the package's documentation.
If something you are using is not explicitly listed as part of the public API in the package's documentation, then you cannot depend on it not changing between minor or patch versions.

With respect to library versioning, we will version all libraries within a package together.
This means that libraries inherit their version from the package.
This keeps library and package versions from diverging and shares reasoning with the policy of releasing packages which share a repository together.
If you need libraries to have different versions then consider splitting them into different packages.

Filesystem Layout
-----------------

The filesystem layout of packages and repositories should follow the same conventions in order to provide a consistent experience for users browsing our source code.

Package layout
^^^^^^^^^^^^^^


* ``src``: contains all C and C++ code

  * Also contains C/C++ headers which are not installed

* ``include``: contains all C and C++ headers which are installed

  * ``<package name>``: for all C and C++ installed headers they should be folder namespaced by the package name

* ``<package_name>``: contains all Python code
* ``test``: contains all automated tests and test data
* ``doc``: contains all the documentation
* ``package.xml``: as defined by `REP-0140 <http://www.ros.org/reps/rep-0140.html>`_ (may be updated for prototyping)
* ``CMakeLists.txt``: only ROS packages which use CMake
* ``setup.py``: only ROS packages which use Python code only
* ``README``: README which can be rendered on Github as a landing page for the project

  * This can be as short or detailed as is convenient, but it should at least link to project documentation
  * Consider putting a CI or code coverage tag in this readme
  * It can also be ``.rst`` or anything else that Github supports

* ``LICENSE``: A copy of the license or licenses for this package
* ``CHANGELOG.rst``: `REP-0132 <http://www.ros.org/reps/rep-0132.html>`_ compliant changelog

Repository layout
^^^^^^^^^^^^^^^^^

Each package should be in a subfolder which has the same name as the package.
If a repository contains only a single package it can optionally be in the root of the repository.

The root of the repository should have a ``CONTRIBUTING`` file describing the contribution guidelines.
This might include license implication when using e.g. the Apache 2 License.

Documentation
-------------

*(API docs are not yet being automatically generated)*

All packages should have these documentation elements:

* Description and purpose
* Definition and description of the public API
* Examples
* How to build and install (should reference external tools/workflows)
* How to build and run tests
* How to build documentation
* How to develop (useful for describing things like ``python setup.py develop``)

Each package should describe itself and its purpose or how it is used in the larger scope.
The description should be written, as much as possible, assuming that the reader has stumbled onto it without previous knowledge of ROS or other related projects.

Each package should define and describe its public API so that there is a reasonable expectation for users what is covered by the semantic versioning policy.
Even in C and C++, where the public API can be enforced by API and ABI checking, it is a good opportunity to describe the layout of the code and the function of each part of the code.

It should be easy to take any package and from that package's documentation understand how to build, run, build and run tests, and build the documentation.
Obviously we should avoid repeating ourselves for common workflows, like build a package in a workspace, but the basic workflows should be either described or referenced.

Finally, it should include any documentation for developers.
This might include workflows for testing the code using something like ``python setup.py develop``, or it might mean describing how to make use of extension points provided by you package.

Examples:

* capabilities: http://docs.ros.org/hydro/api/capabilities/html/

  * This one gives an example of docs which describe the public API

* catkin_tools: https://catkin-tools.readthedocs.org/en/latest/development/extending_the_catkin_command.html

  * This is an example of describing an extension point for a package


Package Quality Categories
--------------------------

*(proposed; not yet being used)*

*Note: this section is planned to be escalated to a REP eventually*

This section describes a set of categories which are meant to convey the quality, or at least the maturity, of packages in the ROS ecosystem.
Inclusion in one category or another is based on the policies to which the packages adhere.
The categories are meant to give some expectation as to the quality of a package and allows the maintainers to be more strict with some packages and less so with others.

The purpose of these categories is not to enforce quality, but to set expectations for consumers of the packages and to encourage maintainers of the packages to document how their package's policies achieve that quality level.
The documented policies allow consumers of the packages to consider any caveats for the package or its dependencies when deciding whether or not the package meets the standards for their project.

The categories also provide rough goals for packages to strive towards, encouraging better quality across the ecosystem.

There are four quality levels described below, each roughly described as:

* Quality Level 1:

  * highest quality level
  * packages which are needed for production systems
  * e.g. ``rclcpp``, ``urdf``, ``tf2``, etc.

* Quality Level 2:

  * high quality packages which are either:

    * on the way to level 1 or
    * are general solutions used by many people, but are only sometimes used for production systems

  * e.g. ``navigation2``, ``rosbag2``, etc.

* Quality Level 3:

  * tooling quality packages
  * e.g. ``ros2cli``, ``rviz``, ``rqt``, etc.

* Quality Level 4:

  * demos, tutorials, and experiments
  * e.g. research packages, ``demo_nodes_cpp``, ``examples_rclcpp_minimal_publisher``, etc.

While each quality level will have different requirements, it's always possible to overachieve in certain requirements even if other requirements prevent a package from moving up to the next quality level.

Quality Level 1
^^^^^^^^^^^^^^^

This category should be used for packages which are required for a reasonable ROS system in a production environment.
That is to say that after you remove development tools, build tools, and introspection tools, these packages are still left over as requirements for a basic ROS system to run.
However, that does not mean that packages that would not normally fit this description should never be called 'Level 1'.
If there is a need for a particular package in a reasonable production scenario, then that package should be considered for this category as well.
However, packages which we consider essential to getting a robot up and running quickly, but perhaps is a generic solution to the problem should probably not start out as 'Level 1' due to the high effort in getting a package to 'Level 1' and maintaining it there.

For example, the packages which provide intra-process communication, inter-process communication, generated message runtime code, node lifecycle, etc. should probably all be considered for 'Level 1'.
However, a package which provides pose estimation (like ``robot_pose_ekf``\ ) is a generic solution for something that most people need, but is often replaced with a domain specific solution in production, and therefore it should probably not start out as 'Level 1'.
However, it may upgrade to it at a later date, if it proves to be a solution that people want to use in their products.

Tools, like ``rostopic``\ , generally do not fall into this category either, but are not categorically excluded.
For example, it may be the case the tool which launches and verifies a ROS graph (``ros2launch``\ ) may need to be considered 'Level 1' for use in production systems.

Package Requirements
~~~~~~~~~~~~~~~~~~~~

*Note: bullets below that start with [ROS Core], will be the prescription for what we do in the core packages in order to meet the associated requirements*

Requirements to be considered a 'Level 1' package:

* Version Policy:

  * Must have a version policy (e.g. ``semver``)
  * Must be at a stable version (e.g. for ``semver`` that means have a version >= 1.0.0)
  * Must have a strictly declared public API
  * Must have a policy for API stability
  * Must have a policy for ABI stability
  * Must have a policy that keeps API and ABI stability within a released ROS Distribution
  * [ROS Core] will use ``semver``, will maintain API and ABI stability according to ``semver`` and will be ABI (and therefore API) stable within a ROS distribution

* Change Control Process:

  * Must have all code changes occur through a change request (e.g. pull request, merge request, etc.)
  * Must have peer review policy for all change requests (e.g. require one or more reviewer)
  * Must have Continuous Integration (CI) policy for all change requests
  * Must have documentation policy for all change requests
  * [ROS Core]:

    * All changes will go through a pull request
    * All pull requests will require at least one reviewer who did not author the pr (package may choose to increase this number)
    * All pull requests will be tested via CI, and on all tier 1 platforms (if applicable)
    * Any required changes to documentation (API documentation, feature documentation, release notes, etc.) must be proposed before merging related changes

* Documentation:

  * Must have documentation for each "feature" (e.g. for ``rclcpp``: create a node, publish a message, spin, etc.)
  * Must have documentation for each item in the public API (e.g. functions, classes, etc.)
  * Must have a declared license or set of licenses
  * Must have a copyright statement in each source file
  * Must have a "quality declaration" document, which declares the quality level and justifies how the package meets each of the requirements

    * Must have a section in the repository's ``README`` which contains the "quality declaration" or links to it
    * Must register with a centralized list of 'Level 1' packages, if one exists, to allow for peer review of the claim

  * [ROS Core] will use the Apache 2.0 license, unless the package has an existing permissive license (e.g. rviz uses three-clause BSD)

* Testing:

  * Must have system tests which cover all items in the "feature" documentation
  * Must have system, integration, and/or unit tests which cover all of the public API
  * Code coverage:

    * Must have code coverage tracking for the package
    * Must have and enforce a code coverage policy for new changes
    * [ROS Core]:

      * Must provide line coverage
      * Must achieve a line coverage above 95%
      * May pick a lower percentage target with justification, but must document it prominently
      * May provide branch coverage
      * May exclude code from coverage (test code, debug code, etc.)
      * Must require coverage to increase or stay the same before merging a change, but...
      * May accept a change that decreases coverage with proper justification (e.g. deleting code that was previously covered can cause the percentage to drop)

  * Performance:

    * Must have performance tests (exceptions allowed if they don't make sense to have)
    * Must have a performance regression policy (i.e. blocking either changes or releases on unexpected performance regressions)
    * [ROS Core]:

      * May have performance tests, strongly recommended, but for some packages it doesn't make sense
      * If there are performance tests, must choose to either check each change or before each release or both
      * If there are performance tests, must require justification for merging a change or making a release that lowers performance

* Dependencies:

  * Must not have direct runtime "ROS" dependencies which are not 'Level 1' dependencies, but...
  * May have optional direct runtime "ROS" dependencies which are not 'Level 1', e.g. tracing or debugging features that can be disabled
  * Must have justification for why each direct runtime "non-ROS" dependency is equivalent to a 'Level 1' package in terms of quality

* Platform Support:

  * Must support all tier 1 platforms for ROS 2, as defined in `REP-2000 <https://www.ros.org/reps/rep-2000.html#support-tiers>`_

If the above points are satisfied then a package can be considered 'Level 1'.
Below are some details on the above points.

Version Policy
""""""""""""""

The most important thing is to have some version policy which developers may use to anticipate and understand changes to the version of the package.
We recommend the use of ``semver`` as it covers all the important points that a version policy should cover, is well thought out, and is popular in the open source community broadly.

The policy should link changes to API and ABI to the version scheme.

Additionally, specifically for the ROS ecosystem, the policy should state that API and ABI will be maintained within a stable ROS distribution.
For ``semver``, this means only patch and minor increases only into an existing ROS distribution.

Public API
""""""""""

The package should also state what the public API includes, and/or state what parts of the API are excluded intentionally.

For C++, it's somewhat obvious that all installed headers are part of the public API, but it's acceptable to have parts of the accessible API not be stable.
For example, having an "experimental" namespace or a "detail" namespace which does not adhere to the API and ABI stability rules is allowed, but they must be clearly documented as such.
Changes to these excluded API's, especially something like a "detail" namespace, should still not break API or ABI for other public API's indirectly.

For Python, it's more important to explicitly declare which parts of the API is public, because all modules are typically installed and accessible to users.
One easy thing to do is to say all of the API is public and therefore API stable, but "impl" or "detail" namespaces can be used if needed, they just need to be clearly documented as not public and therefore not stable.

For yet other languages the details will be different, but the important thing is that the public API be obviously documented, and that the public API adheres to an API and ABI stability as described in the version policy, and that they are documented and tested.

Feature Documentation
"""""""""""""""""""""

For each feature provided by the public API of the package, or by a tool in the package, there must be corresponding user documentation.
The term "feature", and the scope of the documentation, is intentionally vague because it's difficult to quantitatively measure this metric.
However, the spirit of this requirement is that, for a 'Level 1' quality package, all of the things a user might do with the package needs at least basic documentation or a snippet of code as an example on how to use it.
The `roscpp Overview <https://wiki.ros.org/roscpp/Overview>`_ from the ROS 1 wiki is a good example of this kind of documentation.

Feature Testing and Code Coverage Policy
""""""""""""""""""""""""""""""""""""""""

This policy should aim for a "high" coverage standard, but the exact number and rules will vary depending on the package in question.
The policy may be influenced by factors like:

- what programming languages are being used, and whether or not there are multiple languages in use
- what coverage information is available (statement vs. line vs. branch vs condition/path coverage)
- what strategy is preferred for dealing with difficult to reach statements/branches

This StackOverflow question is a good summary of the issues:

https://stackoverflow.com/questions/90002/what-is-a-reasonable-code-coverage-for-unit-tests-and-why

In particular, this answer does a good job of summarizing the issue:

https://stackoverflow.com/a/34698711/671658

Importantly, this answer points out that tracking and enforcing code coverage statistics is strictly empirical (rather than theoretical) and that there are different reasons for using them.
Among those reasons listed is "To satisfy stakeholders", which is the main goal of requiring a code coverage policy for these high quality packages.
It is summarized nicely:

    For many projects, there are various actors who have an interest in software quality who may not be involved in the day-to-day development of the software (managers, technical leads, etc.)
    Saying "we're going to write all the tests we really need" is not convincing:
    They either need to trust entirely, or verify with ongoing close oversight (assuming they even have the technical understanding to do so.)
    Providing measurable standards and explaining how they reasonably approximate actual goals is better.

The other two reasons "To normalize team behavior" and "To keep yourself honest" are nice reasons to have code coverage goals, but are out of scope for this document.

The general recommendation is to have at least line coverage and aim to achieve and maintain a high percentage of coverage (e.g. above 90%).
This at least gives you and your stakeholders some confidence that all feature have basic tests.
Any assurances beyond that would require branch coverage statistics and independent investigation of the tests and how they test the code.

Performance Testing
"""""""""""""""""""

There are some cases where performance testing does not make sense to have.
For example, it may be a good idea to have performance tests for a code generator (like ``rosidl_generator_cpp``), but it is not strictly required since its performance does not affect a runtime production system, and so in that case the package could claim to be 'Level 1' without performance tests if properly justified in the "quality declaration".

However, if performance is a reasonable concern for use in a production system, then there must be performance tests and they should be used in conjunction with a regression policy which aims to prevent new versions of the package to be considerably slower without cause.
Note, the performance regression policy should not prevent regressions, but instead should aim to detect them and either address them directly, plan to address them in the future, or when unavoidable (e.g. fixing a bug required more resources to be safe) explain why the regression has occurred in the memorandum of the change request that introduced it.

Dependencies
""""""""""""

Each package should examine their direct runtime dependencies for their quality levels.
Packages should not claim a quality level higher than their dependencies, unless it can be reasonably explained why they do not affect the quality of the package in question.

An example of this would be build or "build tool" dependencies, which are only used during build time and do not impact the runtime quality of the package.
This would not include, however, build dependencies which, for example, contribute only headers to a C++ library or a static library, as the quality of those headers or static library also impact the quality of the runtime product directly.
This would include, for another example, something like CMake, which in most ways does not impact the quality of the product.

There's obviously a lot of ambiguity in this area, as you could argue for or against a variety of dependencies and how they impact the package.
However, the point is to require the maintainers of the package to examine each dependency, justify why they do or do not impact the quality, and document that so that peer reviewers and consumers of the package can make their own evaluation.

Dependencies which are other "ROS" packages should have these quality standards applied to them and should meet or exceed the quality level claimed by the package in question.

Dependencies which are not other "ROS" packages should be individually examined for quality.
You may either try to apply the requirements for the quality levels described here, or you may wish to simply argue the quality without using these requirements as a ruler.
In either case, for each direct "non-ROS" dependency your "quality declaration" should include a justification as to why it is acceptable to depend on this software and still claim your package's level of quality.
This may simply be text justification, or it may link to other analysis or discussions had by community members rationalizing the choice.
The important point is that each dependency is considered, justified, and that the justification is documented, so that users of the package can read the justification and decide for themselves if it is acceptable or not.

Any important caveats or justified exceptions for your dependencies should be mentioned (or referenced) in your own package's "quality declaration" document.

For example, if your package depends on ``rclcpp``, and ``rclcpp`` claims 'level 1' quality with the caveat that this requires you use an rmw implementation that also meets the 'level 1' quality standard, then your package's "quality declaration" document should mention this as well.
Perhaps just saying that one of your dependencies, ``rclcpp``, has some caveats and then link to ``rclcpp``'s own "quality declaration".

In this way, caveats and justifications that may be important for peer reviewers and consumers of your package to understand can "bubble up" from any part of the system.

The goal here is for the maintainer of a package to "make the case" to potential users or stakeholders that their dependencies are at least as high quality as the package in question, and to make a best effort attempt to make them aware of any issues or caveats.
It's up to those users and stakeholders to evaluate that justification and to look at the dependencies themselves as well.

Claiming a Quality Level and Documenting Package Policies
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Each package claiming a quality level should have a "quality declaration" documented somewhere.
This declaration should include a claimed quality level and then should have a section for each of the requirements in that claimed quality level justifying how the package meets each of those requirements.

Sometimes the justification will be a link to a policy documented in the package itself or it may link to a common policy used by a group of packages.
If there is additional evidence that these policies are being followed, that should be included as well, e.g. a link to the coverage statistics for the package to show that coverage is being tracked and maintained.
Other times, justification will be an explanation as to why a requirement was not met or does not apply, e.g. if performance tests do not make sense for the package in question, it should be satisfactorily explained.

There is no enforcement or checking of these claims, but instead it's just sufficient to present this information to potential users.
If the users feel that the justifications are insufficient or incorrect, they can open issues against the repository and resolve it with the maintainers.

There should be one or more communal lists of 'Level 1' (and maybe 'Level 2' or 'Level 3') quality level packages.
These lists should be modified via change requests (maybe a text document in a repository) so that there can be peer review.
This document will not prescribe how or where these lists should be hosted, but one thought is that the list could live on the main ROS 2 documentation website.

Quality Level 2
^^^^^^^^^^^^^^^

These are packages which need to be solidly developed and might be used in production environments, but are not strictly required, or are commonly replaced by custom solutions.
This can also include packages which are not yet up to 'Level 1' but intend to be in the future.

Package Requirements
~~~~~~~~~~~~~~~~~~~~

*Note: bullets below that start with [ROS Core], will be the prescription for what we do in the core packages in order to meet the associated requirements*

Requirements to be considered a 'Level 2' package:

* Version Policy:

  * The same as 'Level 1' packages

* Change Control Process:

  * Must have all code changes occur through a change request (e.g. pull request, merge request, etc.)
  * Must have Continuous Integration (CI) policy for all change requests
  * [ROS Core]:

    * All changes will go through a pull request
    * All pull requests will be tested via CI

* Documentation:

  * Must have documentation for each "feature" (e.g. for ``rclcpp``: create a node, publish a message, spin, etc.)
  * Must have a declared license or set of licenses
  * Must have a copyright statement in each source file
  * Must have a "quality declaration" document, which declares the quality level and justifies how the package meets each of the requirements

    * Must have a section in the repository's ``README`` which contains the "quality declaration" or links to it
    * Must register with a centralized list of 'Level 2' packages, if one exists, to allow for peer review of the claim

  * [ROS Core] will use the Apache 2.0 license, unless the package has an existing permissive license (e.g. rviz uses three-clause BSD)

* Testing:

  * Must have system tests which cover all items in the "feature" documentation
  * Code coverage:

    * Must have code coverage tracking for the package
    * [ROS Core]:

      * Must provide line coverage statistics
      * May provide branch coverage
      * May exclude code from coverage (test code, debug code, etc.)

* Dependencies:

  * Must not have direct runtime "ROS" dependencies which are not 'Level 2' dependencies, but...
  * May have optional direct runtime "ROS" dependencies which are not 'Level 2', e.g. tracing or debugging features that can be disabled
  * Must have justification for why each direct runtime "non-ROS" dependency is equivalent to a 'Level 2' package in terms of quality

* Platform Support:

  * Must support all tier 1 platforms for ROS 2, as defined in `REP-2000 <https://www.ros.org/reps/rep-2000.html#support-tiers>`_

If the above points are satisfied then a package can be considered 'Level 2'.
Refer to the detailed description of the requirements in the Quality Level 1 section above for more information.

Quality Level 3
^^^^^^^^^^^^^^^

These are packages which are useful for development purposes or introspection, but are not recommended for use in embedded products or mission critical scenarios.
These packages are more lax on documentation, testing, and scope of public API's in order to make development time lower or foster addition of new features.

Package Requirements
~~~~~~~~~~~~~~~~~~~~

*Note: bullets below that start with [ROS Core], will be the prescription for what we do in the core packages in order to meet the associated requirements*

Requirements to be considered a 'Level 3' package:

* Version Policy:

  * The same as 'Level 1' packages, except:

    * No public API needs to be explicitly declared, though this can make it harder to maintain API and ABI stability
    * No requirement to keep API/ABI stability within a stable ROS release, but it is recommended still

* Change Control Process:

  * Must have all code changes occur through a change request (e.g. pull request, merge request, etc.)
  * Must have Continuous Integration (CI) policy for all change requests
  * [ROS Core]:

    * All changes will go through a pull request
    * All pull requests will be tested via CI

* Documentation:

  * Must have a declared license or set of licenses
  * Must have a copyright statement in each source file
  * May have a "quality declaration" document, which declares the quality level and justifies how the package meets each of the requirements

    * Must have a section in the repository's ``README`` which contains the "quality declaration" or links to it
    * May register with a centralized list of 'Level 3' packages, if one exists, to allow for peer review of the claim

  * [ROS Core] will use the Apache 2.0 license, unless the package has an existing permissive license (e.g. rviz uses three-clause BSD)

* Testing:

  * No explicit testing requirements, though covering some if not all of the features with tests is recommended

* Dependencies:

  * May have direct runtime "ROS" dependencies which are not 'Level 3' dependencies, but they should be documented

* Platform Support:

  * Must support all tier 1 platforms for ROS 2, as defined in `REP-2000 <https://www.ros.org/reps/rep-2000.html#support-tiers>`_

If the above points are satisfied then a package can be considered 'Level 3'.
Refer to the detailed description of the requirements in the Quality Level 1 section above for more information.

Quality Level 4
^^^^^^^^^^^^^^^

These are demos, tutorials, or experiments.
They don't have strict requirements, but are not excluded from having good documentation or tests.
For example, this might be a tutorial package which is not intended for reuse but has excellent documentation because it serves primarily as an example to others.

Package Requirements
~~~~~~~~~~~~~~~~~~~~

*Note: bullets below that start with [ROS Core], will be the prescription for what we do in the core packages in order to meet the associated requirements*

Requirements to be considered a 'Level 4' package:

* Version Policy:

  * No requirements, but having a policy is still recommended (e.g. ``semver``), even if the version is not yet stable (e.g. >= 1.0.0 for ``semver``)

* Change Control Process:

  * No explicit change control process required, but still recommended

* Documentation:

  * Must have a declared license or set of licenses
  * Must have a copyright statement in each source file
  * [ROS Core] will use the Apache 2.0 license, unless the package has an existing permissive license (e.g. rviz uses three-clause BSD)

* Testing:

  * No explicit testing requirements, though covering some if not all of the features with tests is recommended

* Dependencies:

  * No restrictions

* Platform Support:

  * May support all tier 1 platforms for ROS 2, as defined in `REP-2000 <https://www.ros.org/reps/rep-2000.html#support-tiers>`_

Any package that does not claim to be 'Level 3' or higher is automatically 'Level 4'.
Refer to the detailed description of the requirements in the Quality Level 1 section above for more information.

Quality Level 5
^^^^^^^^^^^^^^^

Packages in this category simply do not meet even the 'Level 4' requirements, and for that reason should not be used.
The rationale being that all packages should have at least a declare license or licenses and should include copyright statements in each file.

Repository Organization
^^^^^^^^^^^^^^^^^^^^^^^

Since these categories are applied on a per package basis, and since there may be more than one package per source repository, it's recommended that the strictest set of policies apply to the whole repository.
This is recommended, rather than trying to mix processes depending on which packages are changed in a given change request (pull request or merge request, etc.).
If this is too onerous, then it's recommended to split lower quality packages out into a separate repository.
