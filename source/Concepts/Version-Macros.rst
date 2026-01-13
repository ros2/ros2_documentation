Version Macros for API Negotiation
=================================

ROS 2 guarantees ABI compatibility within a single distribution.
However, new APIs and features may be introduced through patch releases
or backports. This means that within the same ROS 2 distribution,
different API versions may be available.

To support robust applications that can adapt to these differences,
ROS 2 provides auto-generated version macros that allow developers to
check for the availability of features at compile time.

Why Version Macros Matter
-------------------------

Version macros enable developers to:

* Write code that works across multiple patch versions of the same distribution
* Conditionally use new APIs when they are available
* Maintain compatibility without relying on fragile workarounds

These checks are especially useful for libraries and applications that
are intended to support multiple ROS 2 distributions or patch releases.

Available Version Macros
------------------------

The following version macros are auto-generated during the build process:

* ``RCLCPP_VERSION_MAJOR``
* ``RCLCPP_VERSION_MINOR``
* ``RCLCPP_VERSION_PATCH``

* ``RCL_VERSION_MAJOR``
* ``RCL_VERSION_MINOR``
* ``RCL_VERSION_PATCH``

* ``RMW_VERSION_MAJOR``
* ``RMW_VERSION_MINOR``
* ``RMW_VERSION_PATCH``

These macros are defined in the corresponding package headers and can be
used in C and C++ code.

Example Usage
-------------

The following example demonstrates how to conditionally use an API
based on the available ``rclcpp`` version:

.. code-block:: cpp

   #include <rclcpp/version.h>

   #if RCLCPP_VERSION_MAJOR > 10 || \
       (RCLCPP_VERSION_MAJOR == 10 && RCLCPP_VERSION_MINOR >= 1)
     // Use newer API
   #else
     // Fallback for older API
   #endif

This approach allows applications to safely adapt to evolving APIs
while remaining compatible within a ROS 2 distribution.

Related Notes
-------------

Although ROS 2 distributions guarantee ABI stability, API extensions
may still be introduced. Version macros provide a reliable mechanism
for detecting such changes at compile time.
