
Troubleshooting
===============

Enable Multicast
----------------

In order to communicate successfully via DDS, the used network interface has to be multicast enabled.
We've seen in past experiences that this might not necessarily be enabled by default (on Ubuntu or OSX) when using the loopback adapter.
See the `original issue <https://github.com/ros2/ros2/issues/552>`__ or a `conversation on ros-answers <https://answers.ros.org/question/300370/ros2-talker-cannot-communicate-with-listener/>`__.
You can verify that your current setup allows multicast with the ROS 2 tool:

In Terminal 1:

.. code-block:: bash

   ros2 multicast receive

In Terminal 2:

.. code-block:: bash

   ros2 multicast send

Import failing even with library present on the system
------------------------------------------------------

Sometimes ``rclpy`` fails to be imported because of some missing DLLs on your system.
If so make sure to install all the dependencies listed in the "Installing prerequisites" sections of the installation instructions (`Windows <windows-install-binary-installing-prerequisites>`, `MacOS <osx-install-binary-installling-prerequisites>`).

If you are installing from binaries, you may need to update your dependencies: they must be the same version as those used to build the binaries.
