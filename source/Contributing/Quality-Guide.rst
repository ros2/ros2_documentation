.. redirect-from::

    Quality-Guide

Quality guide: ensuring code quality
====================================

This section tries to give guidance about how to improve the software quality of ROS 2 packages. The guide uses a pattern language based approach to improve the reader's experience ("read little, understand fast, understand much, apply easily").

**What this section is about:**


* ROS 2 core, application and ecosystem packages.
* ROS 2 core client libraries C++ and Python (right now: mainly C++)
* Design and implementation considerations to improve quality attributes like "Reliability", "Security", "Maintainability", "Determinism", etc. which relate to non-functional requirements (right now: mainly "Reliability").

**What this section is not about:**


* Design and implementation considerations which go beyond a single ROS 2 package and a single ROS 2 node (means no integration considerations w.r.t. ROS 2 graphs, etc.).
* Organizational considerations to improve software quality (an organizations structure and processes, etc.).
* Infrastructural considerations which go beyond a single repository (overall continuous integration infrastructure, etc.)

**Relation to other sections:**


* The `Design Guide <Design-Guide>` summarizes design patterns for ROS 2 packages. As quality is highly impacted by design it is a good idea to have a look into it before.
* The `Developer Guide <Developer-Guide>` explains what to consider when contributing to ROS 2 packages w.r.t. to contribution workflow (organizational), coding conventions, documentation considerations, etc. All these considerations may have an impact on single or several quality attributes.

Patterns
--------

* Static code analysis

  * Static code analysis using a single tool
  * Static code analysis using multiple tools complementary
  * Static code analysis as part of the ament package build

* Dynamic code analysis
* ROS 2 library test

  * (referencing of generic unit test patterns like from `xUnitPatterns <http://xunitpatterns.com/Book%20Outline%20Diagrams.html>`__ with references to C++ gtest+gmock/Python unittest implementations)
  * (ROS 2 specific unit test use cases)
  * Property based test (C++ `RapidCheck <https://github.com/emil-e/rapidcheck>`__ / Python `hypothesis <https://github.com/HypothesisWorks/hypothesis-python>`__)
  * Code coverage analysis

* ROS 2 node unit test

  * (generic use cases of ``launch`` based tests)

Static code analysis as part of the ament package build
-------------------------------------------------------

**Context**:


* You have developed your C++ production code.
* You have created a ROS 2 package with build support with ``ament``.

**Problem**:


* Library level static code analysis is not run as part of the package build procedure.
* Library level static code analysis needs to be executed manually.
* Risk of forgetting to execute library level static code analysis before building
  a new package version.

**Solution**:


* Use the integration capabilities of ``ament`` to execute static code analysis as
  part of the package build procedure.

**Implementation**:


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

**Examples**:


* ``rclcpp``:

  * `rclcpp/rclcpp/CMakeLists.txt <https://github.com/ros2/rclcpp/blob/master/rclcpp/CMakeLists.txt>`__
  * `rclcpp/rclcpp/package.xml <https://github.com/ros2/rclcpp/blob/master/rclcpp/package.xml>`__

* ``rclcpp_lifecycle``:

  * `rclcpp/rclcpp_lifecycle/CMakeLists.txt <https://github.com/ros2/rclcpp/blob/master/rclcpp_lifecycle/CMakeLists.txt>`__
  * `rclcpp/rclcpp_lifecycle/package.xml <https://github.com/ros2/rclcpp/blob/master/rclcpp_lifecycle/package.xml>`__

**Resulting context**:


* The static code analysis tools supported by ``ament`` are run as part of the package build.
* Static code analysis tools not supported by ``ament`` need to be executed separately.

Static Thread Safety Analysis via Code Annotation
-------------------------------------------------

**Context:**


* You are developing/debugging your multithreaded C++ production code
* You access data from multiple threads in C++ code

**Problem:**


* Data races and deadlocks can lead to critical bugs.

**Solution:**


* Utilize Clang's static `Thread Safety Analysis <https://clang.llvm.org/docs/ThreadSafetyAnalysis.html>`__ by annotating threaded code

**Context For Implementation:**


To enable Thread Safety Analysis, code must be annotated to let the compiler know more about the semantics of the code. These annotations are Clang-specific attributes - e.g. ``__attribute__(capability()))``. Instead of using those attributes directly, ROS 2 provides preprocessor macros that are erased when using other compilers.

These macros can be found in `rcpputils/thread_safety_annotations.h <https://github.com/ros2/rcpputils/blob/master/include/rcpputils/thread_safety_annotations.h>`__

The Thread Safety Analysis documentation states
  Thread safety analysis can be used with any threading library, but it does require that the threading API be wrapped in classes and methods which have the appropriate annotations

We have decided that we want ROS 2 developers to be able to use ``std::`` threading primitives directly for their development. We do not want to provide our own wrapped types as is suggested above.

There are three C++ standard libraries to be aware of
* The GNU standard library ``libstdc++`` - default on Linux, explicitly via the compiler option ``-stdlib=libstdc++``
* The LLVM standard library ``libc++`` (also called ``libcxx`` ) - default on macOS,  explicitly set by the compiler option ``-stdlib=libc++``
* The Windows C++ Standard Library - not relevant to this use case

``libcxx`` annotates its ``std::mutex`` and ``std::lock_guard`` implementations for Thread Safety Analysis. When using GNU ``libstdc++`` , those annotations are not present, so Thread Safety Analysis cannot be used on non-wrapped ``std::`` types.

*Therefore, to use Thread Safety Analysis directly with* ``std::`` *types, we must use* ``libcxx``


**Implementation:**


The code migration suggestions here are by no means complete - when writing (or annotating existing) threaded code, you are encouraged to utilize as many of the annotations as is logical for your use case. However, this step-by-step is a great place to start!

* Enabling Analysis for Package/Target

  When the C++ compiler is Clang, enable the ``-Wthread-safety`` flag. Example below for CMake-based projects

  .. code-block:: cmake

     if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
       add_compile_options(-Wthread-safety)   # for your whole package
       target_compile_options(${MY_TARGET} PUBLIC -Wthread-safety)  # for a single library or executable
     endif()

* Annotating Code

  * Step 1 - Annotate data members

    * Find anywhere that ``std::mutex`` is used to protect some member data
    * Add the ``RCPPUTILS_TSA_GUARDED_BY(mutex_name)`` annotation to the data that is protected by the mutex

    .. code-block:: cpp

      class Foo {
      public:
        void incr(int amount) {
          std::lock_guard<std::mutex> lock(mutex_);
          bar += amount;
        }

        void get() const {
          return bar;
        }

      private:
        mutable std::mutex mutex_;
        int bar RCPPUTILS_TSA_GUARDED_BY(mutex_) = 0;
      };

  * Step 2 - Fix Warnings

    * In the above example - ``Foo::get`` will produce a compiler warning! To fix it, lock before returning bar

    .. code-block:: cpp

      void get() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return bar;
      }

  * Step 3 - (Optional but Recommended) Refactor Existing Code to Private-Mutex Pattern

    A recommended pattern in threaded C++ code is to always keep your ``mutex`` as a ``private:`` member of the data structure. This makes data safety the concern of the containing structure, offloading that responsibility from users of the structure and minimizing the surface area of affected code.

    Making your locks private may require rethinking the interfaces to your data. This is a great exercise - here are a few things to consider

    * You may want to provide specialized interfaces for performing analysis that requires complex locking logic, e.g. counting members in a filtered set of a mutex-guarded map structure, instead of actually returning the underlying structure to consumers
    * Consider copying to avoid blocking, where the amount of data is small. This can let other threads get on with accessing the shared data, which can potentially lead to better overall performance.

  * Step 4 - (Optional) Enable Negative Capability Analysis

    https://clang.llvm.org/docs/ThreadSafetyAnalysis.html#negative-capabilities

    Negative Capability Analysis lets you specify “this lock must not be held when calling this function”. It can reveal potential deadlock cases that other annotations cannot.

    * Where you specified ``-Wthread-safety``, add the additional flag ``-Wthread-safety-negative``
    * On any function that acquires a lock, use the ``RCPPUTILS_TSA_REQUIRES(!mutex)`` pattern



* How to run the analysis

  * The ROS CI build farm runs a nightly job with ``libcxx``, which will surface any issues in the ROS 2 core stack by being marked "Unstable" when Thread Safety Analysis raises warnings
  * For local runs, you have the following options, all equivalent

    * Use the colcon `clang-libcxx mixin <https://github.com/colcon/colcon-mixin-repository/blob/master/clang-libcxx.mixin>`__

      * ``colcon build --mixin clang-libcxx``
      * You may only use this if you have `configured mixins for your colcon installation <https://github.com/colcon/colcon-mixin-repository/blob/master/README.md>`__

    * Passing compiler to CMake

      * ``colcon build --cmake-args -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_CXX_FLAGS='-stdlib=libc++ -D_LIBCPP_ENABLE_THREAD_SAFETY_ANNOTATIONS' -DFORCE_BUILD_VENDOR_PKG=ON --no-warn-unused-cli``

    * Overriding system compiler

      * ``CC=clang CXX=clang++ colcon build --cmake-args -DCMAKE_CXX_FLAGS='-stdlib=libc++ -D_LIBCPP_ENABLE_THREAD_SAFETY_ANNOTATIONS' -DFORCE_BUILD_VENDOR_PKG=ON --no-warn-unused-cli``



**Resulting Context:**


* Potential deadlocks and race conditions will be surfaced at compile time, when using Clang and ``libcxx``


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
* In case different production code shall be executed during analysis consider conditional compilation e.g. `ThreadSanatizers _has_feature(thread_sanitizer) <https://clang.llvm.org/docs/ThreadSanitizer.html#has-feature-thread-sanitizer>`__.
* In case some code shall not be instrumented consider `ThreadSanatizers _/*attribute*/_((no_sanitize("thread"))) <https://clang.llvm.org/docs/ThreadSanitizer.html#attribute-no-sanitize-thread>`__.
* In case some files shall not be instrumented consider file or function-level exclusion `ThreadSanatizers blacklisting <https://clang.llvm.org/docs/ThreadSanitizer.html#blacklist>`__, more specific: `ThreadSanatizers Sanitizer Special Case List <https://clang.llvm.org/docs/SanitizerSpecialCaseList.html>`__ or with `ThreadSanatizers no_sanitize("thread") <https://clang.llvm.org/docs/ThreadSanitizer.html#blacklist>`__ and use the option ``--fsanitize-blacklist``.

**Resulting context:**


* Higher chance to find data races and deadlocks in production code before deploying it.
* Analysis result may lack reliability, tool in beta phase stage (in case of ThreadSanatizer).
* Overhead due to production code instrumentation (maintenance of separate branches for instrumented/not instrumented production code, etc.).
* Instrumented code needs more memory per thread (in case of ThreadSanatizer).
* Instrumented code maps a lot virtual address space (in case of ThreadSanatizer).

Code coverage analysis
----------------------

**Context**

You have written tests for the library level production code of a ROS 2 package (usually referred to as "unit tests").

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
