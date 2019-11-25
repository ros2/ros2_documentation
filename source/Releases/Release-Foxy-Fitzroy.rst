
ROS 2 Foxy Fitzroy (codename 'foxy'; May 23rd, 2020)
====================================================

.. contents:: Table of Contents
   :depth: 2
   :local:

*Foxy Fitzroy* is the sixth release of ROS 2.

Supported Platforms
-------------------

Foxy Fitzroy is primarily supported on the following platforms:

Tier 1 platforms:

* Ubuntu 20.04 (Focal): ``amd64`` and ``arm64``
* Mac OS X 10.14 (Mojave)
* Windows 10 (Visual Studio 2019)

Tier 2 platforms:

* Ubuntu 20.04 (Focal): ``arm32``

Tier 3 platforms:

* Debian Buster (10): ``amd64``, ``arm64`` and ``arm32``
* OpenEmbedded Thud (2.6) / webOS OSE: ``arm32`` and ``x86``

For more information about RMW implementations, compiler / interpreter versions, and system dependency versions see `REP 2000 <http://www.ros.org/reps/rep-2000.html>`__.


New features in this ROS 2 release
----------------------------------

During the development the `Foxy meta-ticket <https://github.com/ros2/ros2/issues/830>`__ on GitHub contains an up-to-date state of the ongoing high level tasks as well as references specific tickets with more details.

Changes since the Eloquent release
----------------------------------

Default working directory for ament_add_test
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The default working directory for tests added with ``ament_add_test`` has been changed to ``CMAKE_CURRENT_BINARY_DIR`` to match the behavior of CMake ``add_test``.
Either update the tests to work with the new default or pass ``WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}`` to restore the previous value.

Default Console Logging Format
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The default console logging output format was changed to include the timestamp by default, see:

- `https://github.com/ros2/rcutils/pull/190 <https://github.com/ros2/rcutils/pull/190>`_
- `https://discourse.ros.org/t/ros2-logging-format/11549 <https://discourse.ros.org/t/ros2-logging-format/11549>`_

Timeline before the release
---------------------------

TBD
