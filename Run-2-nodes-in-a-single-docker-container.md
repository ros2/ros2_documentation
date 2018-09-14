Pull the ROS2 docker image with tag "ardent-basic".

    docker pull osrf/ros2:ardent-basic

Run the image in a container in interactive mode.

    $ docker run -it osrf/ros2:ardent-basic
    root@<container-id>:/#

Your best friend is the `ros2` command line help now.

    root@<container-id>:/# ros2 --help

E.g. list all installed packages.

    root@<container-id>:/# ros2 pkg list
    (you will see a list of packages)

E.g. list all executables:

    root@<container-id>:/# ros2 pkg executables
    (you will see a list of <package> <executable>)

Run a minimal example of 2 C++ nodes (1 topic subscriber `listener`, 1 topic publisher `talker`) from the package `demo_nodes_cpp` in this container.

    ros2 run demo_nodes_cpp listener &
    ros2 run demo_nodes_cpp talker