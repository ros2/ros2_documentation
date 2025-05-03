You will need to add the ROS 2 apt repository to your system.

First ensure that the `Ubuntu Universe repository <https://help.ubuntu.com/community/Repositories/Ubuntu>`_ is enabled.

.. code-block:: console

   $ sudo apt install software-properties-common
   $ sudo add-apt-repository universe

The `ros-apt-source <https://github.com/ros-infrastructure/ros-apt-source/>`_ packages provide keys and apt source configuration for the various ROS repositories.

Installing the ros2-apt-source package will configure ROS 2 repositories for your system.
Updates to repository configuration will occur automatically when new versions of this package are released to the ROS repositories.

.. code-block:: console

   $ sudo apt update && sudo apt install curl -y
   $ curl -o /tmp/ros2-testing-apt-source.deb "https://ftp.osuosl.org/pub/ros/packages.ros.org/ros2-testing/ubuntu/pool/main/r/ros-apt-source/ros2-testing-apt-source_1.0.0~$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
   $ sudo apt install /tmp/ros2-testing-apt-source.deb
