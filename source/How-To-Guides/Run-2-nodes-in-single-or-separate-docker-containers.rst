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

To run the containers call ``docker compose up`` in the same directory. You can close the containers with ``Ctrl+C``.
