.. redirect-from::

    Run-2-nodes-in-two-separate-docker-containers

Running 2 nodes in 2 separate docker containers [community-contributed]
=======================================================================

Open a terminal. Run the image in a container in interactive mode and launch a topic publisher (executable ``talker`` from the package ``demo_nodes_cpp``) with ``ros2 run``:

.. code-block:: bash

   docker run -it --rm osrf/ros:rolling-desktop ros2 run demo_nodes_cpp talker

Open a second terminal. Run the image in a container in interactive mode and launch a topic subscriber (executable ``listener`` from the package ``demo_nodes_cpp``)  with ``ros2 run``:

.. code-block:: bash

   docker run -it --rm osrf/ros:rolling-desktop ros2 run demo_nodes_cpp listener

As an alternative to the command line invocation, you can create a ``docker-compose.yml`` file (here version 2) with the following (minimal) content:

.. code-block:: yaml

   version: '2'

   services:
     talker:
       image: osrf/ros:rolling-desktop
       command: ros2 run demo_nodes_cpp talker
     listener:
       image: osrf/ros:rolling-desktop
       command: ros2 run demo_nodes_cpp listener
       depends_on:
         - talker

To run the containers call ``docker-compose up`` in the same directory. You can close the containers with ``Ctrl+C``.
