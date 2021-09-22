About Cross-compilation
=======================

Overview
--------

Open Robotics provides pre-built ROS 2 packages for multiple platforms, but a number of developers still rely on `cross-compilation <https://en.wikipedia.org/wiki/Cross_compiler>`__ for different reasons such as:
 - The development machine does not match the target system.
 - Tuning the build for specific core architecture (e.g. setting -mcpu=cortex-a53 -mfpu=neon-fp-armv8 when building for Raspberry Pi3).
 - Targeting a different file systems other than the ones supported by the pre-built images released by Open Robotics.

How does it work ?
------------------

Cross-compiling simple software (e.g. no dependencies on external libraries) is relatively simple and only requiring a cross-compiler toolchain to be used instead of the native toolchain.

There are a number of factors which make this process more complex:
- The software being built must support the target architecture. Architecture specific code must be properly isolated and enabled during the build according to the target architecture. Examples include assembly code.
- All dependencies (e.g. libraries) must be present, either as pre-built packages or also cross-compiled before the target software using them is cross-compiled.
- When building software stacks (as opposed to standalone software) using build tools (e.g. colcon), it is expected that the build tool provides a mechanism to allow the developer to enable cross-compilation on the underlying build system used by each piece of software in the stack.

Cross-compiling ROS 2
---------------------

The ROS 2 cross-compile tool is under shared ownership of Open Robotics and ROS Tooling Working Group.
It is a Python script that compiles ROS 2 source files for supported target architectures using an emulator in a docker container.
Detailed design of the tool can be found on `ROS 2 design <https://design.ros2.org/articles/cc_build_tools.html>`__.
Instructions to use the tool are in the `cross_compile package <https://github.com/ros-tooling/cross_compile>`__.

If you are using an older version, please follow the `cross-compilation guide <../How-To-Guides/Cross-compilation>`.
