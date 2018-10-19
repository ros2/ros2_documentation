
This section tries to give guidance about how to improve the software quality of ROS2 packages. The guide uses a pattern language based approach to improve the readers experience ("read little, understand fast, understand much, apply easily").

**What this sections is about:**


* ROS2 core, application and ecosystem packages.
* ROS2 core client libraries C++ and Python (right now: mainly C++)
* Design and implementation considerations to improve quality attributes like "Reliability", "Security", "Maintainability", "Determinism", etc. which relate to non-functional requirements (right now: mainly "Reliability").

**What this section is not about:**


* Design and implementation considerations which go beyond a single ROS2 package and a single ROS2 node (means no integration considerations w.r.t. ROS2 graphs, etc.).
* Organizational considerations to improve software quality (an organizations structure and processes, etc.).
* Infrastructural considerations which go beyond a single repository (overall continuous integration infrastructure, etc.)

**Relation to other sections:**


* The `Design Guide <Design-Guide>` summarizes design patterns for ROS2 packages. As quality is highly impacted by design it is a good idea to have a look into there before.
* The `Developer Guide <Developer-Guide>` explains what to consider when contributing to ROS2 packages w.r.t. to contribution workflow (organizational), coding conventions, documentation considerations, etc. All these consideration may have an impact on single or several quality attributes.

Patterns
--------


* Static code analysis

  * Static code analysis using a single tool
  * Static code analysis using multiple tools complementary
  * Static code analysis as part of the ament package build

* Dynamic code analysis
* ROS2 library test

  * (referencing of generic unit test patterns like from `xUnitPatterns <http://xunitpatterns.com/Book%20Outline%20Diagrams.html>`__ with references to C++ gtest+gmock/Python unittest implementations)
  * (ROS2 specific unit test use cases)
  * Property based test (C++ `RapidCheck <https://github.com/emil-e/rapidcheck>`__ / Python `hypothesis <https://github.com/HypothesisWorks/hypothesis-python>`__\ )
  * Code coverage analysis

* ROS2 node unit test

  * (generic use cases of ``launch`` based tests)

Static code analysis as part of the ament package build
-------------------------------------------------------

**Context**\ :


* You have developed your C++ production code.
* You have created a ROS2 package with build support with ``ament``.

**Problem**\ :


* Library level static code analysis is not run as part of the package build procedure.
* Library level static code analysis needs to be executed manually.
* Risk of forgetting to execute library level static code analysis before building
  a new package version.

**Solution**\ :


* Use the integration capabilities of ``ament`` to execute static code analysis as
  part of the package build procedure.

**Implementation**\ :


* Insert into the packages ``CMakeLists.txt`` file.

.. code-block:: bash

   ...
   if(BUILD_TESTING)
     find_package(ament_lint_auto REQUIRED)
     ament_lint_auto_find_test_dependencies()
     ...
   endif()
   ...


* Insert the ``ament_lint`` test dependencies into the packages ``package.xml`` file.

.. code-block:: bash

   ...
   <package format="2">
     ...
     <test_depend>ament_lint_auto</test_depend>
     <test_depend>ament_lint_common</test_depend>
     ...
   </package>

**Examples**\ :


* ``rclcpp``\ :

  * `rclcpp/rclcpp/CMakeLists.txt <https://github.com/ros2/rclcpp/blob/master/rclcpp/CMakeLists.txt>`__
  * `rclcpp/rclcpp/package.xml <https://github.com/ros2/rclcpp/blob/master/rclcpp/package.xml>`__

* ``rclcpp_lifecycle``\ :

  * `rclcpp/rclcpp_lifecycle/CMakeLists.txt <https://github.com/ros2/rclcpp/blob/master/rclcpp_lifecycle/CMakeLists.txt>`__
  * `rclcpp/rclcpp_lifecycle/package.xml <https://github.com/ros2/rclcpp/blob/master/rclcpp_lifecycle/package.xml>`__

**Resulting context**\ :


* The static code analysis tools supported by ``ament`` are run as part of the package build.
* Static code analysis tools not supported by ``ament`` need to be executed separately.

Dynamic analysis (data races & deadlocks)
-----------------------------------------

**Context:**


* You are developing/debugging your multithreaded C++ production code.
* You use pthreads or C++11 threading + llvm libc++ (in case of ThreadSanatizer).
* You do not use Libc/libstdc++ static linking (in case of ThreadSanatizer).
* You do not build non-position-independent executables (in case of ThreadSanatizer).

**Problem:**


* Data races and deadlocks can lead to critical bugs.
* Data races and deadlocks cannot be detected using static analysis (reason: limitation of static analysis).
* Data races and deadlocks must not show up during development debugging / testing (reason: usually not all possible control paths through production code exercised).

**Solution:**


* Use a dynamic analysis tool which focuses on finding data races and deadlocks (here clang ThreadSanatizer).

**Implementation:**


* Compile and link the production code with clang using the option ``-fsanitize=thread`` (this instruments the production code).
* In case different production code shall be executed during anaylsis consider conditional compilation e.g. `ThreadSanatizers _has_feature(thread_sanitizer) <https://clang.llvm.org/docs/ThreadSanitizer.html#has-feature-thread-sanitizer>`__.
* In case some code shall not be instrumented consider `ThreadSanatizers _/\ *attribute*\ /_((no_sanitize("thread"))) <https://clang.llvm.org/docs/ThreadSanitizer.html#attribute-no-sanitize-thread>`__.
* In case some files shall not be instrumented consider file or function level exclusion `ThreadSanatizers blacklisting <https://clang.llvm.org/docs/ThreadSanitizer.html#blacklist>`__\ , more specific: `ThreadSanatizers Sanitizer Special Case List <https://clang.llvm.org/docs/SanitizerSpecialCaseList.html>`__ or with `ThreadSanatizers no_sanitize("thread") <https://clang.llvm.org/docs/ThreadSanitizer.html#blacklist>`__ and use the option ``--fsanitize-blacklist``.

**Resulting context:**


* Higher chance to find data races and deadlocks in production code before deploying it.
* Analysis result may lack reliability, tool in beta phase stage (in case of ThreadSanatizer).
* Overhead due to production code instrumentation (maintenance of separate branches for instrumented/not instrumented production code, etc.).
* Instrumented code needs more memory per thread (in case of ThreadSanatizer).
* Instrumented code maps a lot virtual address space (in case of ThreadSanatizer).

Code coverage analysis
----------------------

**Context**

You have written tests for the library level production code of a ROS2 package (usually refered to as "unit tests").

**Problem**

You do not know how much of the production code is exercised during the execution of the tests.

**Solution**

Select and use a code coverage analysis tool to determine the code coverage.

**Forces**


* Is it possible to integrate the tool with your source code editor?
* If not web service based: Is it possible to integrate the tool with your continuous integration infrastructure?
* What type(s) of coverage measurements (e.g. statement coverage) does the tool support?

**Example**


* C++

  * `gcov <https://gcc.gnu.org/onlinedocs/gcc/Gcov.html>`__ + `lcov <http://ltp.sourceforge.net/coverage/lcov.php>`__
  * `coveralls.io <https://coveralls.io>`__

* Python

  * `coveralls.io <https://coveralls.io>`__

**Resulting context**


* You know how much of your production code was exercised during the execution of the unit tests.
* You have a more or less immediate feedback about the code coverage (editor integration / web service front end).
* You do not know anything about the quality of your tests. (The only way to figure that out is some kind of review).
