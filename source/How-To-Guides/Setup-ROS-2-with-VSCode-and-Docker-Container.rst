Setup ROS 2 with VSCode and Docker [community-contributed]
==========================================================


.. contents:: Contents
    :depth: 2
    :local:


Install VS Code and Docker
--------------------------


Using Visual Studio Code and Docker Containers will enable you to run your favorite ROS 2 Distribution without the necessity to change your operating system or use a virtual machine.
With this tutorial you can set up a docker container, which can be used for your future ROS 2 projects.


Install Docker
^^^^^^^^^^^^^^


To install docker and set the correct user rights please use the following commands.

.. code-block:: console

    sudo apt install docker.io git python3-pip
    pip3 install vcstool
    echo export PATH=$HOME/.local/bin:$PATH >> ~/.bashrc
    source ~/.bashrc
    sudo groupadd docker
    sudo usermod -aG docker $USER
    newgrp docker

Now you can check if the installation was successful by running the following command:

.. code-block:: console

    docker run hello-world

You might need to start the Docker Daemon first, if you cannot run hello-world out of the box:

.. code-block:: console

    sudo systemctl start docker

Install VS Code
^^^^^^^^^^^^^^^

To install VS Code please use the following commands:

.. code-block:: console

    sudo apt update
    sudo apt install software-properties-common apt-transport-https wget -y
    wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo apt-key add -
    sudo add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"
    sudo apt install code


You can run VS Code by typing ``code`` in a terminal.


Install Remote Development Extension
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


Within VS Code search in Extensions (CTRL+SHIFT+X) for the "Remote Development" Extension and install it.


Configure workspace in Docker and VS Code
-----------------------------------------

Add your ROS 2 workspace
^^^^^^^^^^^^^^^^^^^^^^^^


Add a workspace in order to build and open them in a container, e.g.:

.. code-block:: console

    cd ~/
    mkdir ws_[project]
    cd ws_[project]
    mkdir src

Now create a .devcontainer folder in the root of your workspace and add a devcontainer.json and Dockerfile to this .devcontainer folder.
Additionally, you need to create a cache folder in which you can cache the build and install folders for different ROS 2 distros.
The workspace structure should look like this:

::

    ws_[project]
    ├── cache
    |   ├── [ROS2_DISTRO]
    |   |   ├── build
    |   |   ├── install
    |   |   └── log
    |   └── ...
    |
    ├── src
        ├── .devcontainer
        │   ├── devcontainer.json
        │   └── Dockerfile
        ├── package1
        └── package2


With ``File->Open Folder...`` or ``Ctrl+K Ctrl+O``, open the ``src`` folder of your workspace in VS Code.

Edit devcontainer.json for your environment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For the Dev Container to function properly, we have to build it with the correct user.
Therefore add the following to ``.devcontainer/devcontainer.json``:

.. code-block:: json

    {
        "name": "ROS 2 Development Container",
        "privileged": true,
        "remoteUser": "USERNAME",
        "build": {
            "dockerfile": "Dockerfile",
            "args": {
                "USERNAME": "USERNAME"
            }
        },
        "workspaceFolder": "/home/ws",
        "workspaceMount": "source=${localWorkspaceFolder},target=/home/ws/src,type=bind",
        "customizations": {
            "vscode": {
                "extensions":[
                    "ms-vscode.cpptools",
                    "ms-vscode.cpptools-themes",
                    "twxs.cmake",
                    "donjayamanne.python-extension-pack",
                    "eamodio.gitlens",
                    "ms-iot.vscode-ros"
                ]
            }
        },
        "containerEnv": {
            "DISPLAY": "unix:0",
            "ROS_LOCALHOST_ONLY": "1",
            "ROS_DOMAIN_ID": "42"
        },
        "runArgs": [
            "--net=host",
            "-e", "DISPLAY=${env:DISPLAY}"
        ],
        "mounts": [
           "source=/tmp/.X11-unix,target=/tmp/.X11-unix,type=bind,consistency=cached",
            "source=/dev/dri,target=/dev/dri,type=bind,consistency=cached",
            "source=${localWorkspaceFolder}/../cache/ROS_DISTRO/build,target=/home/ws/build,type=bind",
            "source=${localWorkspaceFolder}/../cache/ROS_DISTRO/install,target=/home/ws/install,type=bind",
            "source=${localWorkspaceFolder}/../cache/ROS_DISTRO/log,target=/home/ws/log,type=bind"
        ],
        "postCreateCommand": "sudo rosdep update && sudo rosdep install --from-paths src --ignore-src -y && sudo chown -R USERNAME /home/ws/"
    }



Use ``Ctrl+F`` to open the search and replace menu.
Search for ``USERNAME`` and replace it with your ``Linux username``.
If you do not know your username, you can find it by running ``echo $USERNAME`` in the terminal.
Also replace ``ROS_DISTRO``, with the ROS 2 distribution that you want to use and added to the cache previously, for example, "humble" or "foxy".



Edit Dockerfile
^^^^^^^^^^^^^^^

Open the Dockerfile and add the following contents:


.. code-block:: bash

    FROM ros:ROS_DISTRO
    ARG USERNAME=USERNAME
    ARG USER_UID=1000
    ARG USER_GID=$USER_UID

    # Create the user
    RUN groupadd --gid $USER_GID $USERNAME \
        && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
        #
        # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
        && apt-get update \
        && apt-get install -y sudo \
        && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
        && chmod 0440 /etc/sudoers.d/$USERNAME
    RUN apt-get update && apt-get upgrade -y
    RUN apt-get install -y python3-pip
    ENV SHELL /bin/bash

    # ********************************************************
    # * Anything else you want to do like clean up goes here *
    # ********************************************************

    # [Optional] Set the default user. Omit if you want to keep the default as root.
    USER $USERNAME
    CMD ["/bin/bash"]

Search here also for the ``USERNAME`` and replace it with your ``Linux username`` and the ``ROS_DISTRO`` with the ROS 2 distribution you wish to use and added to the cache previously.


Open and Build Development Container
------------------------------------

Use ``View->Command Palette...`` or ``Ctrl+Shift+P`` to open the command palette.
Search for the command ``Dev Containers: (Re-)build and Reopen in Container`` and execute it.
This will build your development docker container for your. It will take a while - sit back or go for a coffee.


Test Container
^^^^^^^^^^^^^^

To test if everything worked correctly, open a terminal in the container using ``View->Terminal`` or ``Ctrl+Shift+``` and ``New Terminal`` in VS Code.
Inside the terminal do the following:

.. code-block:: console

    sudo apt install ros-$ROS_DISTRO-rviz2 -y
    source /opt/ros/$ROS_DISTRO/setup.bash
    rviz2

.. Note:: There might be a problem with displaying RVIZ. If no window pops up, then check the value of ``echo $DISPLAY`` - if the output is 1, you can fix this problem with ``echo "export DISPLAY=unix:1" >> /etc/bash.bashrc`` and then test it again. You can also change the DISPLAY value in the devcontainer.json and rebuild it.
