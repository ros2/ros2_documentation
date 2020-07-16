.. redirect-from::

    Alpha-Overview

ROS 2 alpha releases (Aug 2015 - Oct 2016)
==========================================

.. contents:: Table of Contents
   :depth: 1
   :local:

This is a merged version of the previously separated pages for the 8 alpha releases of ROS 2.

We hope that you try them out and `provide feedback <../Contact>`.

ROS 2 alpha8 release (code name *Hook-and-Loop*; October 2016)
----------------------------------------------------------------

.. contents:: Table of Contents
   :local:

Changes to supported DDS vendors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

ROS 2 supports multiple middleware implementations (see `this page <../Concepts/DDS-and-ROS-middleware-implementations>` for more details).
Until Alpha 8, ROS 2 was supporting ROS middleware implementations for eProsima's Fast RTPS, RTI's Connext and PrismTech's OpenSplice.
To streamline our efforts, as of Alpha 8, Fast RTPS and Connext (static) will be supported, with Fast RTPS (`now Apache 2.0-licensed <http://www.eprosima.com/index.php/company-all/news/61-eprosima-goes-apache>`__) shipped as the default.

Scope
^^^^^

As the "alpha" qualifier suggests, this release of ROS 2 is far from complete.
You should not expect to switch from ROS 1 to ROS 2, nor should you expect to build a new robot control system with ROS 2.
Rather, you should expect to try out some demos, explore the code, and perhaps write your own demos.

The improvements included in this release are:


* Several improvements to Fast RTPS and its rmw implementation

  * Support for large (image) messages in Fast RTPS
  * ``wait_for_service`` functionality in Fast RTPS

* Support for all ROS 2 message types in Python and C
* Added support for Quality of Service (QoS) settings in Python
* Fixed various bugs with the previous alpha release

Pretty much anything not listed above is not included in this release.
The next steps are described in the `Roadmap <../Roadmap>`.

ROS 2 alpha7 release (code name *Glue Gun*\ ; July 2016)
--------------------------------------------------------

.. contents:: Table of Contents
   :local:

New version of Ubuntu required
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Until Alpha 6 ROS 2 was targeting Ubuntu Trusty Tahr (14.04). As of this Alpha ROS 2 is targeting Ubuntu Xenial Xerus (16.04) to benefit from newer versions of the compiler, CMake, Python, etc.

Scope
^^^^^

As the "alpha" qualifier suggests, this release of ROS 2 is far from complete.
You should not expect to switch from ROS 1 to ROS 2, nor should you expect to build a new robot control system with ROS 2.
Rather, you should expect to try out some demos, explore the code, and perhaps write your own demos.

The major features included in this release are:


* Graph API functionality: wait_for_service

  * Added interfaces in rclcpp and make use of them in examples, demos, and tests

* Improved support for large messages in both Connext and Fast-RTPS (partial for Fast-RTPS)
* Turtlebot demo using ported code from ROS 1

  * See: https://github.com/ros2/turtlebot2_demo

Pretty much anything not listed above is not included in this release.
The next steps are described in the `Roadmap <../Roadmap>`.

ROS 2 alpha6 release (code name *Fastener*; June 2016)
------------------------------------------------------

.. contents:: Table of Contents
   :local:

Scope
^^^^^

As the "alpha" qualifier suggests, this release of ROS 2 is far from
complete.
You should not expect to switch from ROS 1 to ROS 2, nor should
you expect to build a new robot control system with ROS 2.
Rather, you
should expect to try out some demos, explore the code, and perhaps write
your own demos.

The major features included in this release are:


* Graph API functionality: wait_for_service

  * Added graph guard condition to nodes for waiting on graph changes
  * Added ``rmw_service_server_is_available`` for verifying if a service is available

* Refactored ``rclcpp`` to use ``rcl``
* Improved support for complex message types in Python

  * Nested messages
  * Arrays
  * Strings

Pretty much anything not listed above is not included in this release.
The next steps are described in the `Roadmap <../Roadmap>`.

ROS 2 alpha5 release (code name *Epoxy*; April 2016)
------------------------------------------------------

.. contents:: Table of Contents
   :local:


Scope
^^^^^

As the "alpha" qualifier suggests, this release of ROS 2 is far from
complete.
You should not expect to switch from ROS 1 to ROS 2, nor should
you expect to build a new robot control system with ROS 2.
Rather, you
should expect to try out some demos, explore the code, and perhaps write
your own demos.

The major features included in this release are:


* Support for C data structures in Fast RTPS and Connext Dynamic rmw implementations.
* Support services in C.
* Added 32-bit and 64-bit ARM as experimentally supported platforms.

Pretty much anything not listed above is not included in this release.
The next steps are described in the `Roadmap <../Roadmap>`.

ROS 2 alpha4 release (code name *Duct tape*; February 2016)
-----------------------------------------------------------

.. contents:: Table of Contents
   :local:

Background
^^^^^^^^^^

As explained in a `design article <https://design.ros2.org/articles/why_ros2.html>`__,
we are engaged in the development of a new major version of ROS, called "ROS 2."
While the underlying concepts (e.g., publish / subscribe messaging) and goals
(e.g., flexibility and reusability) are the same as for ROS 1, we are taking this
opportunity to make substantial changes to the system, including changing
some of the core APIs.
For a deeper treatment of those changes and their rationale, consult the other
`ROS 2 design articles <https://design.ros2.org>`__.

Status
^^^^^^

On February 17, 2016, we are releasing ROS 2 alpha4,
code-named **Duct tape**.
Our primary goal with this release is to add more features, while also addressing the feedback we received for the previous releases.
To that end, we built a set of `demos <../Tutorials>` that
show some of the key features of ROS 2.
We encourage you to try out those
demos, look at the code that implements them, and `provide
feedback <../Contact>`.
We're especially interested to know how well (or
poorly) we're addressing use cases that are important to you.

Intended audience
^^^^^^^^^^^^^^^^^

While everyone is welcome to try out the demos and look through the code, we're aiming this release at people who are already experienced with ROS 1 development.
At this point, the ROS 2 documentation is pretty sparse and much of the system is explained by way of how it compares to ROS 1.

Scope
^^^^^

As the "alpha" qualifier suggests, this release of ROS 2 is far from
complete.
You should not expect to switch from ROS 1 to ROS 2, nor should
you expect to build a new robot control system with ROS 2.
Rather, you
should expect to try out some demos, explore the code, and perhaps write
your own demos.

The major features included in this release are:


* Improved type support infrastructure, including support for C
* Preliminary Python client library, only publishers and subscriptions are supported. Beware, the API is subject to change and is far from complete!
* Added structures for ROS time in C API (still needs C++ API)

  * New concept of extensible "time sources" for ROS Time, the default time source will be like ROS 1 (implementation pending)

Pretty much anything not listed above is not included in this release.
The next steps are described in the `Roadmap <../Roadmap>`.

ROS 2 alpha3 release (code name *Cement*; December 2015)
----------------------------------------------------------

.. contents:: Table of Contents
   :local:


Background
^^^^^^^^^^

As explained in a `design article <https://design.ros2.org/articles/why_ros2.html>`__,
we are engaged in the development of a new major version of ROS, called "ROS 2."
While the underlying concepts (e.g., publish / subscribe messaging) and goals
(e.g., flexibility and reusability) are the same as for ROS 1, we are taking this
opportunity to make substantial changes to the system, including changing
some of the core APIs.
For a deeper treatment of those changes and their rationale, consult the other
`ROS 2 design articles <https://design.ros2.org>`__.

Status
^^^^^^

On December 18, 2015, we are releasing ROS 2 alpha3,
code-named **Cement**.
Our primary goal with this release is to add more features, while also addressing the feedback we received for the previous releases.
To that end, we built a set of `demos <../Tutorials>` that
show some of the key features of ROS 2.
We encourage you to try out those
demos, look at the code that implements them, and `provide
feedback <../Contact>`.
We're especially interested to know how well (or
poorly) we're addressing use cases that are important to you.

Intended audience
^^^^^^^^^^^^^^^^^

While everyone is welcome to try out the demos and look through the code, we're aiming this release at people who are already experienced with ROS 1 development.
At this point, the ROS 2 documentation is pretty sparse and much of the system is explained by way of how it compares to ROS 1.

Scope
^^^^^

As the "alpha" qualifier suggests, this release of ROS 2 is far from
complete.
You should not expect to switch from ROS 1 to ROS 2, nor should
you expect to build a new robot control system with ROS 2.
Rather, you
should expect to try out some demos, explore the code, and perhaps write
your own demos.

The major features included in this release are:


* Updated ``rcl`` interface.

  * This interface will be wrapped in order to create language bindings, e.g. ``rclpy``.
  * This interface has improved documentation and test coverage over existing interfaces we currently have, e.g. ``rmw`` and ``rclcpp``.
  * See `rcl headers <https://github.com/ros2/rcl/tree/release-alpha3/rcl/include/rcl>`__.

* Added support in rclcpp for using the TLSF (two-level segregate fit) allocator, a memory allocator design for embedded and real-time systems.
* Improved efficiency of MultiThreadedExecutor and fixed numerous bugs with multi-threaded execution, which is now test on CI.
* Added ability to cancel an Executor from within a callback called in spin.
* Added ability for a timer to cancel itself by supporting a Timer callback that accepts a reference to itself as a function parameter.
* Added checks for disallowing multiple threads to enter Executor::spin.
* Improved reliability of numerous tests that had been sporadically failing.
* Added support for using Fast RTPS (instead of, e.g., OpenSplice or Connext).
* A partial port of tf2 including the core libraries and core command line tools.

Pretty much anything not listed above is not included in this release.
The next steps are described in the `Roadmap <../Roadmap>`.

ROS 2 alpha2 release (code name *Baling wire*; October 2015)
--------------------------------------------------------------

.. contents:: Table of Contents
   :local:

Background
^^^^^^^^^^

As explained in a `design
article <https://design.ros2.org/articles/why_ros2.html>`__, we are engaged in
the development of a new major version of ROS, called "ROS 2." While the
underlying concepts (e.g., publish / subscribe messaging) and goals (e.g.,
flexibility and reusability) are the same as for ROS 1, we are taking this
opportunity to make substantial changes to the system, including changing
some of the core APIs.
For a deeper treatment of those changes and their
rationale, consult the other `ROS 2 design
articles <https://design.ros2.org>`__.


Status
^^^^^^

On November 3, 2015, we are releasing ROS 2 alpha2,
code-named **Baling wire**.
Our primary goal with this release is to add more features, while also addressing the feedback we received for the previous alpha 1 release.
To that end, we built a set of `demos <../Tutorials>` that
show some of the key features of ROS 2.
We encourage you to try out those
demos, look at the code that implements them, and `provide
feedback <../Contact>`.
We're especially interested to know how well (or
poorly) we're addressing use cases that are important to you.


Intended audience
^^^^^^^^^^^^^^^^^

While everyone is welcome to try out the demos and look through the code, we're aiming this release at people who are already experienced with ROS 1 development.
At this point, the ROS 2 documentation is pretty sparse and much of the system is explained by way of how it compares to ROS 1.


Scope
^^^^^

As the "alpha" qualifier suggests, this release of ROS 2 is far from
complete.
You should not expect to switch from ROS 1 to ROS 2, nor should
you expect to build a new robot control system with ROS 2.
Rather, you
should expect to try out some demos, explore the code, and perhaps write
your own demos.

The major features included in this release are:


* Support for custom allocators in rclcpp, useful for real-time messaging
* Feature parity of Windows with Linux/OSX, including workspace management, services and parameters
* rclcpp API improvements
* FreeRTPS improvements

Pretty much anything not listed above is not included in this release.
The next steps are described in the `Roadmap <../Roadmap>`.

ROS 2 alpha1 release (code name *Anchor*; August 2015)
--------------------------------------------------------

.. contents:: Table of Contents
   :local:

Background
^^^^^^^^^^

As explained in a `design
article <https://design.ros2.org/articles/why_ros2.html>`__, we are engaged in
the development of a new major version of ROS, called "ROS 2." While the
underlying concepts (e.g., publish / subscribe messaging) and goals (e.g.,
flexibility and reusability) are the same as for ROS 1, we are taking this
opportunity to make substantial changes to the system, including changing
some of the core APIs.
For a deeper treatment of those changes and their
rationale, consult the other `ROS 2 design
articles <https://design.ros2.org>`__.


Status
^^^^^^

On August 31, 2015, we are releasing ROS 2 alpha1,
code-named **Anchor**.
Our primary goal with this release is to give
you the opportunity to understand how ROS 2 works, in particular how it
differs from ROS 1.
To that end, we built a set of `demos <../Tutorials>` that
show some of the key features of ROS 2.
We encourage you to try out those
demos, look at the code that implements them, and `provide
feedback <../Contact>`.
We're especially interested to know how well (or
poorly) we're addressing use cases that are important to you.


Intended audience
^^^^^^^^^^^^^^^^^

While everyone is welcome to try out the demos and look through the code, we're aiming this release at people who are already experienced with ROS 1 development.
At this point, the ROS 2 documentation is pretty sparse and much of the system is explained by way of how it compares to ROS 1.


Scope
^^^^^

As the "alpha" qualifier suggests, this release of ROS 2 is far from
complete.
You should not expect to switch from ROS 1 to ROS 2, nor should
you expect to build a new robot control system with ROS 2.
Rather, you
should expect to try out some demos, explore the code, and perhaps write
your own demos.

The major features included in this release are:


* Discovery, transport, and serialization `use DDS <https://design.ros2.org/articles/ros_on_dds.html>`__
* Support `multiple DDS vendors <https://design.ros2.org/articles/ros_on_dds.html#vendors-and-licensing>`__
* Support messaging primitives: topics (publish / subscribe), services (request / response), and parameters
* Support Linux (Ubuntu Trusty), OS X (Yosemite) and Windows (8)
* `Use quality-of-service settings to handle lossy networks <../Tutorials/Quality-of-Service>`
* `Communicate inter-process or intra-process with the same API <../Tutorials/Intra-Process-Communication>`
* `Write real-time safe code that uses the ROS 2 APIs <../Tutorials/Real-Time-Programming>`
* `Run ROS 2 on "bare-metal" microcontrollers (no operating system) <https://github.com/ros2/freertps/wiki>`__
* `Bridge communication between ROS 1 and ROS 2 <https://github.com/ros2/ros1_bridge/blob/master/README>`__

Pretty much anything not listed above is not included in this release.
The next steps are described in the `Roadmap <../Roadmap>`.
