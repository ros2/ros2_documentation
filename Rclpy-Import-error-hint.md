# Import failing even with library present on the system

Sometimes `rclpy` fails to be imported because of some missing DLLs on your system.
If so make sure to install all the dependencies listed in the "Installing prerequisites" sections of the installation instructions ([Windows](https://github.com/ros2/ros2/wiki/Windows-Install-Binary#installing-prerequisites), [MacOS](https://github.com/ros2/ros2/wiki/OSX-Install-Binary#installing-prerequisites), [Linux](https://github.com/ros2/ros2/wiki/Linux-Install-Binary#installing-prerequisites)).

If you are installing from binaries, you may need to update your dependencies: they must be the same version as those used to build the binaries.
