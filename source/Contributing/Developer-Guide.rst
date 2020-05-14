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


* **Shared ownership**: Everybody working on ROS 2 should feel ownership over all parts of the system.
  The original author of a chunk of code does not have any special permission or obligation to control or maintain that chunk of code.
  Everyone is free to propose changes anywhere, to handle any type of ticket, and to review any pull request.
* **Be willing to work on anything**: As a corollary to shared ownership, everybody should be willing to take on any available task and contribute to any aspect of the system.
* **Ask for help**: If you run into trouble on something, ask your fellow developers for help, via tickets, comments, or email, as appropriate.

Quality Practices
-----------------

Packages can ascribe to different levels of quality based on the development practices they adhere to, as per the guidelines in the `Package Quality Categories`_ section.
The categories are differentiated by their policies on versioning, testing, documentation, and more.

.. change above link to `REP 2004: Package Quality Categories <>`_ once merged

The following sections are the specific development rules we follow to ensure core packages are of the highest quality ('Level 1').
We recommend all ROS developers strive to adhere to the following policies to ensure quality across the ROS ecosystem.

Versioning
^^^^^^^^^^

We will use the `Semantic Versioning guidelines <http://semver.org/>`__ (``semver``) for versioning.

We will also adhere to some ROS-specific rules built on top of ``semver's`` full meaning:

* Major version increments (i.e. breaking changes) should not be made within a released ROS distribution.

  * Patch (interface-preserving) and minor (non-breaking) version increments do not break compatibility, so these sorts of changes *are* allowed within a release.

  * Major ROS releases are the best time to release breaking changes. If a core package needs multiple breaking changes, they should be merged into their integration branch (e.g. master) to allow catching problems in CI quickly, but released together to reduce the number of major releases for ROS users.

* For compiled code, the ABI is considered part of the public interface. Any change that requires recompiling dependent code is considered major (breaking).

* Core packages in Dashing and Eloquent are *not* in their initial development phase. Despite SemVer <https://semver.org/#spec-item-4>`_, we enforce API stability, even though the major version component is `0`.

  * Subsequently, packages should strive to reach a mature state and increase to version ``1.0.0`` so to match ``semver's`` specifications.

These rules are *best-effort*.
In unlikely, extreme cases, it may be necessary to break API within a major version/distribution.
Whether an unplanned break increments the major or minor version will be assessed on a case-by-case basis.

Public API declaration
~~~~~~~~~~~~~~~~~~~~~~

According to ``semver``, every package must clearly declare a public API.
We will use the "Public API Declaration" section of the quality declaration of a package to declare what symbols are part of the public API.

For most C and C++ packages the declaration is any header that it installs.
However, it is acceptable to define a set of symbols which are considered private.
Avoiding private symbols in headers can help with ABI stability, but is not required.

For other languages like Python, a public API must be explicitly defined, so that it is clear what symbols can be relied on with respect to the versioning guidelines.
The public API can also be extended to build artifacts like configuration variables, CMake config files, etc. as well as executables and command line options and output.
Any elements of the public API should be clearly stated in the package's documentation.
If something you are using is not explicitly listed as part of the public API in the package's documentation, then you cannot depend on it not changing between minor or patch versions.

Deprecation strategy
~~~~~~~~~~~~~~~~~~~~

Where possible, we will also use the tick-tock deprecation and migration strategy for major version increments.
New deprecations will come in a new distribution release, accompanied by compiler warnings expressing that the functionality is being deprecated.
In the next release, the functionality will be completely removed (no warnings).
We will not add deprecations after a distribution is released.

Example of function ``foo`` deprecated and replaced by function ``bar``:

=========  ========================================================
 Version    API
=========  ========================================================
X-turtle   void foo();
Y-turtle   [[deprecated("use bar()")]] void foo(); <br> void bar();
Z-turtle   void bar();
=========  ========================================================

Change control process
^^^^^^^^^^^^^^^^^^^^^^

* All changes must go through a pull request.

* We will enforce the `Developer Certificate of Origin (DCO) <https://developercertificate.org/>`_ on pull requests in ROSCore repositories.
  It requires all commit messages to contain the ``Signed-off-by`` line with an email address that matches the commit author.
  You can pass ``-s`` / ``--signoff`` to the ``git commit`` invocation or write the expected message manually (e.g. ``Signed-off-by: Your Name Developer <your.name@example.com>``).

* Always run CI jobs for all `tier 1 platforms <https://www.ros.org/reps/rep-2000.html#support-tiers>`_ for every pull request and include links to jobs in the pull request.
  (If you don't have access to the Jenkins job someone will trigger the jobs for you.)

* A minimum of 1 approval from a fellow developer who did not author the pull request is required to consider it approved.
  Approval is required before merging.

  * Packages may choose to increase this number.

* Any required changes to documentation (API documentation, feature documentation, release notes, etc.) must be proposed before merging related changes.

Guidelines for backport PRs
~~~~~~~~~~~~~~~~~~~~~~~~~~~

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

  * Each source file must have a license and copyright statement, checked with an automated linter.
  * Each package must have a LICENSE file, typically the Apache 2.0 license, unless the package has an existing permissive license (e.g. rviz uses three-clause BSD)

Each package should describe itself and its purpose assuming, as much as possible, that the reader has stumbled onto it without previous knowledge of ROS or other related projects.

Each package should define and describe its public API so that there is a reasonable expectation for users about what is covered by the semantic versioning policy.
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

.. Uncomment when REP is published: These practices don't affect the categories described in `REP 2004 <>`_, but are still highly recommended for the development process.

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
For example, a new function ``rmw_foo()`` introduced to the RMW API must be implemented in the following packages (as of ROS Crystal):

* `rmw_fastrtps <https://github.com/ros2/rmw_fastrtps/tree/master/rmw_fastrtps_cpp>`__
* `rmw_connext <https://github.com/ros2/rmw_connext>`__

Updates for non-Tier 1 middleware libraries should also be considered if feasible (e.g. depending on the size of the change).
See `REP-2000 <http://www.ros.org/reps/rep-2000.html#crystal-clemmys-december-2018-december-2019>`__ for the list of middleware libraries and their tiers.

Tracking tasks
^^^^^^^^^^^^^^

To help organize work on ROS 2, the core ROS 2 development team uses kanban-style `GitHub project boards <https://github.com/orgs/ros2/projects>`_.

Not all issues and pull requests are tracked on the project boards, however.
A board usually represents an upcoming release or specific project.
Tickets can be browsed on a per-repo basis by browsing the `ROS 2 repositories' <https://github.com/ros2>`_ individual issue pages.

The names and purposes of columns in any given ROS 2 project board vary, but typically follow the same general structure:

* **To do**: Issues that are relevant to the project, ready to be assigned
* **In progress**: Active pull requests on which work is currently in progress
* **In review**: Pull requests where work is complete and ready for review, and for those currently under active review
* **Done**: Pull requests and related issues are merged/closed (for informational purposes)

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
* Avoid using references for ``std::shared_ptr`` since that subverts the reference counting. If the original instance goes out of scope and the reference is being used it accesses freed memory.

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
* ``package.xml``: as defined by `REP-0140 <http://www.ros.org/reps/rep-0140.html>`_ (may be updated for prototyping)
* ``CMakeLists.txt``: only ROS packages which use CMake
* ``setup.py``: only ROS packages which use Python code only
* ``README``: can be rendered on Github as a landing page for the project

  * This can be as short or detailed as is convenient, but it should at least link to project documentation
  * Consider putting a CI or code coverage tag in this README
  * It can also be ``.rst`` or anything else that Github supports

* ``CONTRIBUTING``: describes the contribution guidelines

  * This might include license implication, e.g. when using the Apache 2 License.

* ``LICENSE``: A copy of the license or licenses for this package
* ``CHANGELOG.rst``: `REP-0132 <http://www.ros.org/reps/rep-0132.html>`_ compliant changelog

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

  * Design doc is required and should be contributed to `ros2/design <https://github.com/ros2/design/>`__ to be made accessible on http://design.ros2.org/.
  * You should fork the repository and submit a pull request detailing the design.

  Mention the related ros2 issue (for example, ``Design doc for task ros2/ros2#<issue id>``) in the pull request or the commit message.
  Detailed instructions are on the `ROS 2 Contribute <http://design.ros2.org/contribute.html>`__ page.
  Design comments will made directly on the pull request.

If the task is planned to be released with a specific version of ROS, this information should be included in the pull request.

Design document review
~~~~~~~~~~~~~~~~~~~~~~

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
~~~~~~~~~~~~~~

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

Code review
~~~~~~~~~~~

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


Package Quality Categories
--------------------------

.. remove once REP 2004 is complete

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

  * [ROS Core]:

    * Must have automated checks for copyright statements and licenses
    * Must use the Apache 2.0 license, unless the package has an existing permissive license (e.g. rviz uses three-clause BSD)

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

  * Linters and Static Analysis

    * Must have a code style and enforce it.
    * Must use static analysis tools where applicable.
    * [ROS Core]:

      * Must use ROS code style and use linters from ``ament_lint_common`` to enforce it
      * Must use all linters/static analysis that are part of ``ament_lint_common``

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

There are also other, non-API, things which should be considered and optionally documented as part of the "stable interface" of the package.
This includes, but isn't limited to, message definitions, command line tools (arguments and output format), ROS names (topic, service, node, etc.), and behaviors of the applications.

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

  * [ROS Core]:

    * Must have automated checks for copyright statements and licenses
    * Must use the Apache 2.0 license, unless the package has an existing permissive license (e.g. rviz uses three-clause BSD)

* Testing:

  * Must have system tests which cover all items in the "feature" documentation
  * Code coverage:

    * Must have code coverage tracking for the package
    * [ROS Core]:

      * Must provide line coverage statistics
      * May provide branch coverage
      * May exclude code from coverage (test code, debug code, etc.)

  * Linters and Static Analysis

    * Must have a code style and enforce it.
    * Must use static analysis tools where applicable.
    * [ROS Core]:

      * Must use ROS code style and use linters from ``ament_lint_common`` to enforce it
      * Must use all linters/static analysis that are part of ``ament_lint_common``

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

  * [ROS Core]:

    * Must have automated checks for copyright statements and licenses
    * Must use the Apache 2.0 license, unless the package has an existing permissive license (e.g. rviz uses three-clause BSD)

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
  * [ROS Core]:

    * Must have automated checks for copyright statements and licenses
    * Must use the Apache 2.0 license, unless the package has an existing permissive license (e.g. rviz uses three-clause BSD)

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
