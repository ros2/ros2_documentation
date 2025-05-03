You will need to add the ROS 2 apt repository to your system.

First ensure that the `Ubuntu Universe repository <https://help.ubuntu.com/community/Repositories/Ubuntu>`_ is enabled.

.. code-block:: console

   $ sudo apt install software-properties-common
   $ sudo add-apt-repository universe

Now add the ROS 2 GPG key with apt.

.. code-block:: console

   $ sudo apt update && sudo apt install curl -y
<<<<<<< HEAD
   $ sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

Then add the repository to your sources list.

.. code-block:: console

   $ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
=======
   $ curl -o /tmp/ros2-testing-apt-source.deb "https://ftp.osuosl.org/pub/ros/packages.ros.org/ros2-testing/ubuntu/pool/main/r/ros-apt-source/ros2-testing-apt-source_1.0.0~$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
   $ sudo apt install /tmp/ros2-testing-apt-source.deb
>>>>>>> 40d2ba6 (Double quote ros-apt-source pkg URL (#5521))
