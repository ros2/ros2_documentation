.. redirect-from::

    Developer-Guide

ROS 2 developer guide
=====================

.. contents:: Table of Contents
   :depth: 2
   :local:

This page defines the practices and policies we employ when developing ROS 2.

General Principles
------------------

Some principles are common to all ROS 2 development:


* **Shared ownership**:
  Everybody working on ROS 2 should feel ownership over all parts of the system.
  The original author of a chunk of code does not have any special permission or obligation to control or maintain that chunk of code.
  Everyone is free to propose changes anywhere, to handle any type of ticket, and to review any pull request.
* **Be willing to work on anything**:
  As a corollary to shared ownership, everybody should be willing to take on any available task and contribute to any aspect of the system.
* **Ask for help**:
  If you run into trouble on something, ask your fellow developers for help, via tickets, comments, or email, as appropriate.

Quality Practices
-----------------

Packages can ascribe to different levels of quality based on the development practices they adhere to, as per the guidelines in `REP 2004: Package Quality Categories <https://www.ros.org/reps/rep-2004.html>`_.
The categories are differentiated by their policies on versioning, testing, documentation, and more.

The following sections are the specific development rules we follow to ensure core packages are of the highest quality ('Level 1').
We recommend all ROS developers strive to adhere to the following policies to ensure quality across the ROS ecosystem.

Versioning
^^^^^^^^^^

We will use the `Semantic Versioning guidelines <http://semver.org/>`__ (``semver``) for versioning.

We will also adhere to some ROS-specific rules built on top of ``semver's`` full meaning:

* Major version increments (i.e. breaking changes) should not be made within a released ROS distribution.

  * Patch (interface-preserving) and minor (non-breaking) version increments do not break compatibility, so these sorts of changes *are* allowed within a release.

  * Major ROS releases are the best time to release breaking changes.
    If a core package needs multiple breaking changes, they should be merged into their integration branch (e.g. master) to allow catching problems in CI quickly, but released together to reduce the number of major releases for ROS users.

  * Though major increments require a new distribution, a new distribution does not necessarily require a major bump (if development and release can happen without breaking API).

* For compiled code, the ABI is considered part of the public interface.
  Any change that requires recompiling dependent code is considered major (breaking).

  * ABI breaking changes *can* be made in a minor version bump *before* a distribution release (getting added to the rolling release).

* We enforce API stability for core packages in Dashing and Eloquent even though their major version components are ``0``, despite `SemVer's specification <https://semver.org/#spec-item-4>`_ regarding initial development.

  * Subsequently, packages should strive to reach a mature state and increase to version ``1.0.0`` so to match ``semver's`` specifications.

Caveats
~~~~~~~

These rules are *best-effort*.
In unlikely, extreme cases, it may be necessary to break API within a major version/distribution.
Whether an unplanned break increments the major or minor version will be assessed on a case-by-case basis.

For example, consider a situation involving released X-turtle, corresponding to major version ``1.0.0``, and released Y-turtle, corresponding to major version ``2.0.0``.

If an API-breaking fix is identified to be absolutely necessary in X-turtle, bumping to ``2.0.0`` is obviously not an option because ``2.0.0`` already exists.

The solutions for handling X-turtle's version in such a case, both non-ideal, are:

1. Bumping X-turtle's minor version: non-ideal because it violates SemVer's principle that breaking changes must bump the major version.

2. Bumping X-turtle's major version past Y-turtle (to ``3.0.0``): non-ideal because the older distro's version would become higher than the already-available version of a newer distro, which would invalidate/break version-specific conditional code.

The developer will have to decide which solution to use, or more importantly, which principle they are willing to break.
We cannot suggest one or the other, but in either case we do require that explicit measures be taken to communicate the disruption and its explanation to users manually (beyond just the version increment).

If there were no Y-turtle, even though the fix would technically just be a patch, X-turtle would have to bump to ``2.0.0``.
This case adheres to SemVer, but breaks from our own rule that major increments should not be introduced in a released distribution.

This is why we consider the versioning rules *best-effort*.
As unlikely as the examples above are, it is important to accurately define our versioning system.

Public API declaration
~~~~~~~~~~~~~~~~~~~~~~

According to ``semver``, every package must clearly declare a public API.
We will use the "Public API Declaration" section of the quality declaration of a package to declare what symbols are part of the public API.

For most C and C++ packages the declaration is any header that it installs.
However, it is acceptable to define a set of symbols which are considered private.
Avoiding private symbols in headers can help with ABI stability, but is not required.

For other languages like Python, a public API must be explicitly defined, so that it is clear what symbols can be relied on with respect to the versioning guidelines.
The public API can also be extended to build artifacts like configuration variables, CMake config files, etc. as well as executables and command-line options and output.
Any elements of the public API should be clearly stated in the package's documentation.
If something you are using is not explicitly listed as part of the public API in the package's documentation, then you cannot depend on it not changing between minor or patch versions.

Deprecation strategy
~~~~~~~~~~~~~~~~~~~~

Where possible, we will also use the tick-tock deprecation and migration strategy for major version increments.
New deprecations will come in a new distribution release, accompanied by compiler warnings expressing that the functionality is being deprecated.
In the next release, the functionality will be completely removed (no warnings).

Example of function ``foo`` deprecated and replaced by function ``bar``:

=========  ========================================================
 Version    API
=========  ========================================================
X-turtle   void foo();
Y-turtle   [[deprecated("use bar()")]] void foo(); <br> void bar();
Z-turtle   void bar();
=========  ========================================================

We must not add deprecations after a distribution is released.
Deprecations do not necessarily require a major version bump, though.
A deprecation can be introduced in a minor version bump if the bump happens before the distro is released (similar to ABI breaking changes).

For example, if X-turtle begins development as ``2.0.0``, a deprecation can be added in ``2.1.0`` before X-turtle is released.

We will attempt to maintain compatibility across distros as much as possible.
However, like the caveats associated with SemVer, tick-tock or even deprecation in general may be impossible to completely adhere to in certain cases.

Change control process
^^^^^^^^^^^^^^^^^^^^^^

* All changes must go through a pull request.

* We will enforce the `Developer Certificate of Origin (DCO) <https://developercertificate.org/>`_ on pull requests in ROSCore repositories.

  * It requires all commit messages to contain the ``Signed-off-by`` line with an email address that matches the commit author.

  * You can pass ``-s`` / ``--signoff`` to the ``git commit`` invocation or write the expected message manually (e.g. ``Signed-off-by: Your Name Developer <your.name@example.com>``).

  * DCO is *not* required for pull requests that only address whitespace removal, typo correction, and other `trivial changes <http://cr.openjdk.java.net/~jrose/draft/trivial-fixes.html>`_.

* Always run CI jobs for all `tier 1 platforms <https://www.ros.org/reps/rep-2000.html#support-tiers>`_ for every pull request and include links to jobs in the pull request.
  (If you don't have access to the Jenkins jobs someone will trigger the jobs for you.)

* A minimum of 1 approval from a fellow developer who did not author the pull request is required to consider it approved.
  Approval is required before merging.

  * Packages may choose to increase this number.

* Any required changes to documentation (API documentation, feature documentation, release notes, etc.) must be proposed before merging related changes.

Guidelines for backporting PRs
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When changing an older version of ROS:

* Make sure the features or fixes are accepted and merged in the master branch before opening a PR to backport the changes to older versions.
* When backporting to older versions, also backport to any [newer, still supported versions](https://index.ros.org/doc/ros2/Releases/), even non-LTS versions.
* If you are backporting a single PR in its entirety, title the backport PR "[Distro] <name of original PR>".
  If backporting a subset of changes from one or multiple PRs, the title should be "[Distro] <description of changes>".
* Link to all PRs whose changes you're backporting from the description of your backport PR.
  In a Dashing backport of a Foxy change, you do not need to link to the Eloquent backport of the same change.

Documentation
^^^^^^^^^^^^^

All packages should have these documentation elements present in their README or linked to from their README:

* Description and purpose
* Definition and description of the public API
* Examples
* How to build and install (should reference external tools/workflows)
* How to build and run tests
* How to build documentation
* How to develop (useful for describing things like ``python setup.py develop``)
* License and copyright statements

Each source file must have a license and copyright statement, checked with an automated linter.

Each package must have a LICENSE file, typically the Apache 2.0 license, unless the package has an existing permissive license (e.g. rviz uses three-clause BSD).

Each package should describe itself and its purpose assuming, as much as possible, that the reader has stumbled onto it without previous knowledge of ROS or other related projects.

Each package should define and describe its public API so that there is a reasonable expectation for users about what is covered by the semantic versioning policy.
Even in C and C++, where the public API can be enforced by API and ABI checking, it is a good opportunity to describe the layout of the code and the function of each part of the code.

It should be easy to take any package and from that package's documentation understand how to build, run, build and run tests, and build the documentation.
Obviously we should avoid repeating ourselves for common workflows, like building a package in a workspace, but the basic workflows should be either described or referenced.

Finally, it should include any documentation for developers.
This might include workflows for testing the code using something like ``python setup.py develop``, or it might mean describing how to make use of extension points provided by your package.

Examples:

* capabilities: https://docs.ros.org/hydro/api/capabilities/html/

  * This one gives an example of docs which describe the public API

* catkin_tools: https://catkin-tools.readthedocs.org/en/latest/development/extending_the_catkin_command.html

  * This is an example of describing an extension point for a package

*(API docs are not yet being automatically generated)*

Testing
^^^^^^^

All packages should have some level of system, integration, and/or unit tests.

**Unit tests** should always be in the package which is being tested and should make use of tools like ``Mock`` to try and test narrow parts of the code base in constructed scenarios.
Unit tests should not bring in test dependencies that are not testing tools, e.g. gtest, nosetest, pytest, mock, etc...

**Integration tests** can test interactions between parts of the code or between parts of the code and the system.
They often test software interfaces in ways that we expect the user to use them.
Like Unit tests, Integration tests should be in the package which is being tested and should not bring in non-tool test dependencies unless absolutely necessary, i.e. all non-tool dependencies should only be allowed under extreme scrutiny so they should be avoided if possible.

**System tests** are designed to test end-to-end situations between packages and should be in their own packages to avoid bloating or coupling packages and to avoid circular dependencies.

In general minimizing external or cross package test dependencies should be avoided to prevent circular dependencies and tightly coupled test packages.

All packages should have some unit tests and possibly integration tests, but the degree to which they should have them is based on the package's quality category.
The following subsections apply to 'Level 1' packages:

Code coverage
~~~~~~~~~~~~~

We will provide line coverage, and achieve line coverage above 95%.
If a lower percentage target is justifiable, it must be prominently documented.
We may provide branch coverage, or exclude code from coverage (test code, debug code, etc.).
We require that coverage increase or stay the same before merging a change, but it may be acceptable to make a change that decreases code coverage with proper justification (e.g. deleting code that was previously covered can cause the percentage to drop).

Performance
~~~~~~~~~~~

We strongly recommend performance tests, but recognize they don't make sense for some packages.
If there are performance tests, we will choose to either check each change or before each release or both.
We will also require justification for merging a change or making a release that lowers performance.

Linters and static analysis
~~~~~~~~~~~~~~~~~~~~~~~~~~~

We will use :ref:`ROS code style <CodeStyle>` and enforce it with linters from `ament_lint_common <https://github.com/ament/ament_lint/tree/master/ament_lint_common/doc/index.rst>`_.
All linters/static analysis that are part of ``ament_lint_common`` must be used.

The `ament_lint_auto <https://github.com/ament/ament_lint/blob/master/ament_lint_auto/doc/index.rst>`_ documentation provides information on running ``ament_lint_common``.

General Practices
-----------------

Some practices are common to all ROS 2 development.

These practices don't affect package quality level as described in `REP 2004 <https://www.ros.org/reps/rep-2004.html>`_, but are still highly recommended for the development process.

Issues
^^^^^^

When filing an issue please make sure to:

- Include enough information for another person to understand the issue.
  In ROS 2, the following points are needed for narrowing down the cause of an issue.
  Testing with as many alternatives in each category as feasible will be especially helpful.

  - **The operating system and version.**
    Reasoning: ROS 2 supports multiple platforms, and some bugs are specific to particular versions of operating systems/compilers.
  - **The installation method.**
    Reasoning: Some issues only manifest if ROS 2 has been installed from "fat archives" or from Debians.
    This can help us determine if the issue is with the packaging process.
  - **The specific version of ROS 2.**
    Reasoning: Some bugs may be present in a particular ROS 2 release and later fixed.
    It is important to know if your installation includes these fixes.
  - **The DDS/RMW implementation being used** (see `this page <../Tutorials/Working-with-multiple-RMW-implementations>` for how to determine which one).
    Reasoning: Communication issues may be specific to the underlying ROS middleware being used.
  - **The ROS 2 client library being used.**
    Reasoning: This helps us narrow down the layer in the stack at which the issue might be.

- Include a list of steps to reproduce the issue.
- In case of a bug consider to provide a `short, self contained, correct (compilable), example <http://sscce.org/>`__.
  Issues are much more likely to be resolved if others can reproduce them easily.

- Mention troubleshooting steps that have been tried already, including:

  - Upgrading to the latest version of the code, which may include bug fixes that have not been released yet.
    See `this section <building-from-source>` and follow the instructions to get the "master" branches.
  - Trying with a different RMW implementation.
    See `this page <../Tutorials/Working-with-multiple-RMW-implementations>` for how to do that.

Pull requests
^^^^^^^^^^^^^

* A pull request should only focus on one change.
  Separate changes should go into separate pull requests.
  See `GitHub's guide to writing the perfect pull request <https://github.com/blog/1943-how-to-write-the-perfect-pull-request>`__.

* A patch should be minimal in size and avoid any kind of unnecessary changes.

* A pull request must contain minimum number of meaningful commits.

  * You can create new commits while the pull request is under review.

* Before merging a pull request all changes should be squashed into a small number of semantic commits to keep the history clear.

  * But avoid squashing commits while a pull request is under review.
    Your reviewers might not notice that you made the change, thereby introducing potential for confusion.
    Plus, you're going to squash before merging anyway; there's no benefit to doing it early.

* Any developer is welcome to review and approve a pull request (see `General Principles`_).

* When you start reviewing a pull request, comment on the pull request so that other developers know that you're reviewing it.

* Pull-request review is not read-only, with the reviewer making comments and then waiting for the author to address them.
  As a reviewer, feel free to make minor improvements (typos, style issues, etc.) in-place.
  As the opener of a pull-request, if you are working in a fork, checking the box to `allow edits from upstream contributors <https://github.com/blog/2247-improving-collaboration-with-forks>`__ will assist with the aforementioned.
  As a reviewer, also feel free to make more substantial improvements, but consider putting them in a separate branch (either mention the new branch in a comment, or open another pull request from the new branch to the original branch).

* Any developer (the author, the reviewer, or somebody else) can merge any approved pull request.

Library versioning
^^^^^^^^^^^^^^^^^^

We will version all libraries within a package together.
This means that libraries inherit their version from the package.
This keeps library and package versions from diverging and shares reasoning with the policy of releasing packages which share a repository together.
If you need libraries to have different versions then consider splitting them into different packages.

Development process
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
For example, a new function ``rmw_foo()`` introduced to the RMW API must be implemented in the following packages (as of ROS Foxy):

* `rmw_fastrtps <https://github.com/ros2/rmw_fastrtps>`__
* `rmw_connext <https://github.com/ros2/rmw_connext>`__
* `rmw_cyclonedds <https://github.com/ros2/rmw_cyclonedds>`__

Updates for non-Tier 1 middleware libraries should also be considered if feasible (e.g. depending on the size of the change).
See `REP-2000 <https://www.ros.org/reps/rep-2000.html#crystal-clemmys-december-2018-december-2019>`__ for the list of middleware libraries and their tiers.

Tracking tasks
^^^^^^^^^^^^^^

To help organize work on ROS 2, the core ROS 2 development team uses kanban-style `GitHub project boards <https://github.com/orgs/ros2/projects>`_.

Not all issues and pull requests are tracked on the project boards, however.
A board usually represents an upcoming release or specific project.
Tickets can be browsed on a per-repo basis by browsing the `ROS 2 repositories' <https://github.com/ros2>`_ individual issue pages.

The names and purposes of columns in any given ROS 2 project board vary, but typically follow the same general structure:

* **To do**:
  Issues that are relevant to the project, ready to be assigned
* **In progress**:
  Active pull requests on which work is currently in progress
* **In review**:
  Pull requests where work is complete and ready for review, and for those currently under active review
* **Done**:
  Pull requests and related issues are merged/closed (for informational purposes)

To request permission to make changes, simply comment on the tickets you're interested in.
Depending on the complexity, it might be useful to describe how you plan to address it.
We will update the status (if you don't have the permission) and you can start working on a pull request.
If you contribute regularly we will likely just grant you permission to manage the labels etc. yourself.

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
* Avoid using references for ``std::shared_ptr`` since that subverts the reference counting.
  If the original instance goes out of scope and the reference is being used it accesses freed memory.

Filesystem layout
^^^^^^^^^^^^^^^^^

The filesystem layout of packages and repositories should follow the same conventions in order to provide a consistent experience for users browsing our source code.

Package layout
~~~~~~~~~~~~~~

* ``src``: contains all C and C++ code

  * Also contains C/C++ headers which are not installed

* ``include``: contains all C and C++ headers which are installed

  * ``<package name>``: for all C and C++ installed headers they should be folder namespaced by the package name

* ``<package_name>``: contains all Python code
* ``test``: contains all automated tests and test data
* ``doc``: contains all the documentation
* ``package.xml``: as defined by `REP-0140 <https://www.ros.org/reps/rep-0140.html>`_ (may be updated for prototyping)
* ``CMakeLists.txt``: only ROS packages which use CMake
* ``setup.py``: only ROS packages which use Python code only
* ``README``: can be rendered on GitHub as a landing page for the project

  * This can be as short or detailed as is convenient, but it should at least link to project documentation
  * Consider putting a CI or code coverage tag in this README
  * It can also be ``.rst`` or anything else that GitHub supports

* ``CONTRIBUTING``: describes the contribution guidelines

  * This might include license implication, e.g. when using the Apache 2 License.

* ``LICENSE``: a copy of the license or licenses for this package
* ``CHANGELOG.rst``: `REP-0132 <https://www.ros.org/reps/rep-0132.html>`_ compliant changelog

Repository layout
~~~~~~~~~~~~~~~~~

Each package should be in a subfolder which has the same name as the package.
If a repository contains only a single package it can optionally be in the root of the repository.

Aspirational Practices
----------------------

Presently, we don't use adhere to the practices in this section, but believe they would be beneficial to the development process and hope to employ them officially in the future.

Software Development Lifecycle
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This section describes step-by-step how to plan, design, and implement a new feature:

1. Task Creation
2. Creating the Design Document
3. Design Review
4. Implementation
5. Code Review

Task creation
~~~~~~~~~~~~~

Tasks requiring changes to critical parts of ROS 2 should have design reviews during early stages of the release cycle.
If a design review is happening in the later stages, the changes will be part of a future release.

* An issue should be created in the appropriate `ros2 repository <https://github.com/ros2/>`__, clearly describing the task being worked on.

  * It should have a clear success criteria and highlight the concrete improvements expected from it.
  * If the feature is targeting a ROS release, ensure this is tracked in the ROS release ticket (`example <https://github.com/ros2/ros2/issues/607>`__).

Writing the design document
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Design docs must never include confidential information.
Whether or not a design document is required for your change depends on how big the task is.

1. You are making a small change or fixing a bug:

  * A design document is not required, but an issue should be opened in the appropriate repository to track the work and avoid duplication of efforts.

2. You are implementing a new feature or would like to contribute to OSRF-owned infrastructure (like Jenkins CI):

  * Design doc is required and should be contributed to `ros2/design <https://github.com/ros2/design/>`__ to be made accessible on https://design.ros2.org/.
  * You should fork the repository and submit a pull request detailing the design.

  Mention the related ros2 issue (for example, ``Design doc for task ros2/ros2#<issue id>``) in the pull request or the commit message.
  Detailed instructions are on the `ROS 2 Contribute <https://design.ros2.org/contribute.html>`__ page.
  Design comments will be made directly on the pull request.

If the task is planned to be released with a specific version of ROS, this information should be included in the pull request.

Design document review
~~~~~~~~~~~~~~~~~~~~~~

Once the design is ready for review, a pull request should be opened and appropriate reviewers should be assigned.
It is recommended to include project owner(s) -
maintainers of all impacted packages (as defined by ``package.xml`` maintainer field, see `REP-140 <https://www.ros.org/reps/rep-0140.html#maintainer-multiple-but-at-least-one>`__) - as reviewers.

* If the design doc is complex or reviewers have conflicting schedules, an optional design review meeting can be set up.
  In this case,

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
  * Update and close the GitHub issue associated with this design task

Implementation
~~~~~~~~~~~~~~

Before starting, go through the `Pull requests`_ section for best practices.

* For each repo to be modified:

  * Modify the code, go to the next step if finished or at regular intervals to backup your work.
  * `Self-review <https://git-scm.com/book/en/v2/Git-Tools-Interactive-Staging>`__ your changes using ``git add -i``.
  * Create a new signed commit using ``git commit -s``.

    * A pull request should contain minimal semantically meaningful commits (for instance, a large number of 1-line commits is not acceptable).
      Create new fixup commits while iterating on feedback, or optionally, amend existing commits using ``git commit --amend`` if you don't want to create a new commit every time.
    * Each commit must have a properly written, meaningful, commit message.
      More instructions `here <https://chris.beams.io/posts/git-commit/>`__.
    * Moving files must be done in a separate commit, otherwise git may fail to accurately track the file history.
    * Either the pull request description or the commit message must contain a reference to the related ros2 issue, so it gets automatically closed when the pull request is merged.
      See this `doc <https://help.github.com/articles/closing-issues-using-keywords/>`__ for more details.
    * Push the new commits.

Code review
~~~~~~~~~~~

Once the change is ready for code review:

* Open a pull request for each modified repository.

  * Remember to follow `Pull requests`_ best practices.
  * `GitHub <https://hub.github.com/>`__ can be used to create pull requests from the command-line.
  * If the task is planned to be released with a specific version of ROS, this information should be included in each pull request.

* Package owners who reviewed the design document should be mentioned in the pull request.
* Code review SLO: although reviewing pull requests is best-effort,
  it is helpful to have reviewers comment on pull requests within a week and
  code authors to reply back to comments within a week, so there is no loss of context.
* Iterate on feedback as usual, amend and update the development branch as needed.
* Once the PR is approved, package maintainers will merge the changes in.
