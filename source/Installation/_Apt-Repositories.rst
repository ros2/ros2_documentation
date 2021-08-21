You will need to add the ROS 2 apt repository to your system.
First, make sure that the `Ubuntu Universe repository <https://help.ubuntu.com/community/Repositories/Ubuntu>`_ is enabled by checking the output of this command.

.. code-block:: bash

   apt-cache policy | grep universe
    500 http://us.archive.ubuntu.com/ubuntu focal/universe amd64 Packages
        release v=20.04,o=Ubuntu,a=focal,n=focal,l=Ubuntu,c=universe,b=amd64

If you don't see an output line like the one above, then enable the Universe repository with these instructions.

.. code-block:: bash

   sudo apt install software-properties-common
   sudo add-apt-repository universe


Now add the ROS 2 apt repository to your system.
First authorize our GPG key with apt.

.. code-block:: bash

   sudo apt update && sudo apt install curl gnupg lsb-release
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

Then add the repository to your sources list.

.. code-block:: bash

   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
