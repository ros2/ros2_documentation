.. redirect-from::

    Run-2-nodes-in-a-single-docker-container

Running 2 nodes in a single docker container [community-contributed]
====================================================================

Pull the ROS docker image with tag "rolling-desktop".

.. code-block:: bash

   docker pull osrf/ros:rolling-desktop


Run the image in a container in interactive mode.

.. code-block:: bash

   $ docker run -it osrf/ros:rolling-desktop
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
