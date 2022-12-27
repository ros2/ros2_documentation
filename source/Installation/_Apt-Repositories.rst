Debes agregar el repositorio apt de ROS 2 a tu sistema.

Primero asegúrate que el repositorio `Ubuntu Universe <https://help.ubuntu.com/community/Repositories/Ubuntu>`_ esté habilitado.

.. code-block:: bash

   sudo apt install software-properties-common
   sudo add-apt-repository universe

Ahora agrega la clave GPG de ROS 2 con apt.

.. code-block:: bash

   sudo apt update && sudo apt install curl
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

Luego añade el repositorio a tu lista de fuentes.

.. code-block:: bash

   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
