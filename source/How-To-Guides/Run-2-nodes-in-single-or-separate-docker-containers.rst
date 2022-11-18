.. redirect-from::

    Tutorials/Run-2-nodes-in-a-single-docker-container
    Tutorials/Run-2-nodes-in-two-separate-docker-containers
    Guides/Run-2-nodes-in-two-separate-docker-containers

Running ROS 2 nodes in Docker [community-contributed]
=====================================================

Run two nodes in a single docker container
------------------------------------------

Pull the ROS docker image with tag "{DISTRO}-desktop".

.. code-block:: bash

   docker pull osrf/ros:{DISTRO}-desktop


Run the image in a container in interactive mode.

.. code-block:: bash

   $ docker run -it osrf/ros:{DISTRO}-desktop
   root@<container-id>:/#


Your best friend is the ``ros2`` command line help now.

.. code-block:: bash

   root@<container-id>:/# ros2 --help


E.g. list all installed packages.

.. code-block:: bash

   root@<container-id>:/# ros2 pkg list
   (you will see a list of packages)


E.g. list all executables:

.. code-block:: bash

   root@<container-id>:/# ros2 pkg executables
   (you will see a list of <package> <executable>)


Run a minimal example of 2 C++ nodes (1 topic subscriber ``listener``, 1 topic publisher ``talker``) from the package ``demo_nodes_cpp`` in this container:

.. code-block:: bash

   ros2 run demo_nodes_cpp listener &
   ros2 run demo_nodes_cpp talker

Run two nodes in two separate docker containers
-----------------------------------------------

Open a terminal. Run the image in a container in interactive mode and launch a topic publisher (executable ``talker`` from the package ``demo_nodes_cpp``) with ``ros2 run``:

.. code-block:: bash

   docker run -it --rm osrf/ros:{DISTRO}-desktop ros2 run demo_nodes_cpp talker

Open a second terminal. Run the image in a container in interactive mode and launch a topic subscriber (executable ``listener`` from the package ``demo_nodes_cpp``)  with ``ros2 run``:

.. code-block:: bash

   docker run -it --rm osrf/ros:{DISTRO}-desktop ros2 run demo_nodes_cpp listener

As an alternative to the command line invocation, you can create a ``docker-compose.yml`` file (here version 2) with the following (minimal) content:

.. code-block:: yaml

   version: '2'

   services:
     talker:
       image: osrf/ros:{DISTRO}-desktop
       command: ros2 run demo_nodes_cpp talker
     listener:
       image: osrf/ros:{DISTRO}-desktop
       command: ros2 run demo_nodes_cpp listener
       depends_on:
         - talker

To run the containers call ``docker-compose up`` in the same directory. You can close the containers with ``Ctrl+C``.

Visualising ROS 2 GUIs with Docker and noVNC
--------------------------------------------

We can use `noVNC <https://novnc.com/info.html>`_, a remote-desktop web client, to visualize the GUIs of ROS 2 applications in a browser.

Set up noVNC
^^^^^^^^^^^^

First of all we need to create a Docker network that our containers can use to communicate. This is not required for ROS 2 itself but other inter-container communication, such as remote-desktop data, will require it. Let's create a Docker network called ``ros``:

.. code-block:: bash

   docker network create ros

Thanks to `this project <https://github.com/theasp/docker-novnc>`_, we can launch noVNC in a Docker container using the Docker image ``theasp/novnc:latest``. Download the image:

.. code-block:: bash

   docker pull theasp/novnc:latest

Then run this image in a new container:

.. code-block:: bash

   docker run -d --rm --net=ros \
       --env="DISPLAY_WIDTH=3000" --env="DISPLAY_HEIGHT=1800" --env="RUN_XTERM=no" \
       --name=novnc -p=8080:8080 \
       theasp/novnc:latest

Here's an explanation of what all those options mean:

-d                    Detach. I.E. run the container in background rather than hogging the terminal.
--rm                  Automatically remove the container when it exits.
--net                 Connect the container to the Docker network that we created above.
--env                 Set environment properties in the container to control the display size and prevent an example terminal appearing.
--name                Give this container a name so that other containers can connect their displays to it.
--p                   Publish the container's port 8080 (the value after the colon) onto the host's port 8080 (the value before the colon).

noVNC should now be running as a web application inside the container and listening on port 8080. Since we have mapped that port to 8080 on the host, we should be able to see the noVNC interface at *http://<host name>:8080/vnc.html* . For example, if the host is our local machine then that will be `http://localhost:8080/vnc.html <http://localhost:8080/vnc.html>`_ . Open this in a modern web browser (not IE) and click the *Connect* button. You should see see a blank desktop.

Launch a GUI application and direct its display to noVNC
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

We can then launch containers that run GUI programs and direct them via the noVNC server to our browser. To do this we need to include the parameters ``--net=ros --env="DISPLAY=novnc:0.0"``. For example to run the ``turtlesim_node`` program:

.. code-block:: bash

   docker run -d --rm --net=ros --env="DISPLAY=novnc:0.0" osrf/ros:rolling-desktop ros2 run turtlesim turtlesim_node

Here's an explanation of what those options mean:

-d                    Detach. I.E. run the container in background rather than hogging the terminal.
--rm                  Automatically remove the container when it exits.
--net                 Connect the container to the Docker network that we created above.
--env                 Set an environment property in the container to direct the display to the first display and screen on our container ``novnc``.

The ``turtlesim_node`` UI should appear in the browser.

Launch non-GUI applications on the same network
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Just for completeness, let's run the ``turtle_teleop_key`` program and drive the turtle so that we can see what sort of delay we get with UI updates:

.. code-block:: bash

   docker run -it --rm --net=ros osrf/ros:rolling-desktop ros2 run turtlesim turtle_teleop_key

Here's an explanation of what those options mean:

-it                   Run the container interactively (-i) and allocate a terminal (-t) to this interaction.
--rm                  Automatically remove the container when it exits.
--net                 Connect the container to the Docker network that we created above.

Since we kept the container connected to our terminal, you should be able to use the keyboard to drive the turtle around in the GUI window that you see in the browser.

Notice that, since ``turtle_teleop_key`` has no GUI, there was no need to set the ``DISPLAY`` environment variable this time.
