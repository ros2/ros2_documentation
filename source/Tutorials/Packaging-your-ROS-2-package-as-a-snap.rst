Packaging your ROS 2 package as a snap [community-contributed]
==============================================================

.. contents:: Table of Contents
   :depth: 2
   :local:

What are snaps?
---------------

`Snaps <https://snapcraft.io/>`_ are containers that bundle an application and all its dependencies.
They offer several features that address important concerns as one gets closer to shipping a robotic platform:

- Container solution: Snaps bundle all your dependencies and assets in one package (including ROS 2) making your application installable on dozens of Linux distributions and across distro versions.
- Strict confinement: Snaps are designed to be `secure and isolated <https://snapcraft.io/docs/snap-confinement>`_ from the underlying system and other applications, with `dedicated interfaces <https://snapcraft.io/docs/supported-interfaces>`_ to access the host machine.
- Managing updates: Snaps can update `automatically and transactionally <https://snapcraft.io/docs/keeping-snaps-up-to-date>`_, making sure the device is never broken.
- Release management: Snaps' `multiple release channels <https://snapcraft.io/docs/channels>`_ allow you to have role-based access controls and application versioning, making A/B testing easy and releasing fixes faster.

Creating a snap
---------------

This tutorial will demonstrate how to use `snapcraft <https://github.com/snapcore/snapcraft>`_ to create a new snap, and then how to use it.

First, install snapcraft.
It's recommended to install it from the Snap Store:

.. code-block:: bash

    sudo snap install --classic snapcraft

(Note that using the apt repositories for snapcraft is not recommended and this tutorial will assume that you installed the snap.)

Snapcraft has built-in support for ``Colcon``.
You point it to your package, and tell it what commands to include in the snap.

For our example, we will use demo_nodes_cpp from the `ros2_demos <https://github.com/ros2/demos/tree/{DISTRO}>`_.

Initialize a new snapcraft project here:

.. code-block:: bash

    snapcraft init

This will create a file in a subdirectory ``snap/snapcraft.yaml``.

The snapcraft file
^^^^^^^^^^^^^^^^^^

Open that ``snap/snapcraft.yaml`` file and make it look like this:

.. code-block:: yaml

    name: ros2-talker-listener
    version: '0.1'
    summary: ROS 2 Talker/Listener example
    description: |
      This example launches a ROS 2 talker and listener.

    confinement: devmode
    base: core20

    parts:
      ros-demos:
        plugin: colcon
        source: https://github.com/ros2/demos.git
        source-branch: {DISTRO}
        colcon-packages: [demo_nodes_cpp]
        build-packages: [make, gcc, g++]
        stage-packages: [ros-{DISTRO}-ros2launch]

    apps:
      ros2-talker-listener:
        command: opt/ros/{DISTRO}/bin/ros2 launch demo_nodes_cpp talker_listener.launch.py
        extensions: [ros2-{DISTRO}]

Let's break it down.

Metadata
""""""""

.. code-block:: yaml

    name: ros2-talker-listener
    version: '0.1'
    summary: ROS 2 Talker/Listener example
    description: |
      This example launches a ROS 2 talker and listener.

This is the basic `metadata <https://snapcraft.io/docs/snapcraft-top-level-metadata>`_ that all snaps require.
These fields are fairly self-explanatory, but note that the name must be globally unique among all snaps.

Base
""""

.. code-block:: yaml

    base: core20

The `base <https://snapcraft.io/docs/base-snaps>`_ keyword defines a special kind of snap that provides a run-time environment with a minimal set of libraries that are common to most applications.
`Core20 <https://snapcraft.io/core20>`_ is the current standard base for snap building and is based on `Ubuntu 20.04 LTS <http://releases.ubuntu.com/20.04/>`_.

Security model
""""""""""""""

.. code-block:: yaml

    confinement: devmode

To get started, we won't confine this application.
Unconfined applications, specified with ``devmode``,
can only be released to the “edge” channel of the snapcraft store.

Parts
"""""

.. code-block:: yaml

    parts:
      ros-demos:
        plugin: colcon
        source: https://github.com/ros2/demos.git
        source-branch: {DISTRO}
        colcon-packages: [demo_nodes_cpp]
        build-packages: [make, gcc, g++]
        stage-packages: [ros-{DISTRO}-ros2launch]

Parts define how to build your app.
In this case, we have one: ``ros2-demos``.
Parts can point to local directories, remote git repositories, or tarballs.

Apps
""""

.. code-block:: yaml

    apps:
      ros2-talker-listener:
        command: opt/ros/{DISTRO}/bin/ros2 launch demo_nodes_cpp talker_listener.launch.py
        extensions: [ros2-{DISTRO}]

Apps are the commands exposed to end users.
Each key under apps is the command name that should be made available on users' systems.
The command specifies the path to the binary to be run.
The extensions `ros2-{DISTRO} basically <https://snapcraft.io/docs/ros2-extension>`_ adds the ROS 2 APT package repository.

Building the snap
^^^^^^^^^^^^^^^^^

From the directory you launched the first snapcraft command, run:

.. code-block:: bash

    snapcraft --enable-experimental-extensions

Giving:

.. code-block:: bash

    *EXPERIMENTAL* extensions enabled.
    Launching a VM.
    Launched: snapcraft-ros2-talker-listener
    [...]
    Snapped ros2-talker-listener_0.1_amd64.snap

That will take a few minutes.
You'll see snapcraft using `rosdep <http://docs.ros.org/independent/api/rosdep/html/>`_ to pull the dependencies of your package.
Finally, it builds the package, and installs them into the snap.

Testing the snap
^^^^^^^^^^^^^^^^

This snap is completely standalone: it includes ROS 2,
meaning that you don't even need to install ROS 2 on your system.
Test it out yourself:

.. code-block:: bash

    # We use --devmode here because the snap is devmode confinement
    sudo snap install ros2-talker-listener_0.1_amd64.snap --devmode

Then try it.

.. code-block:: bash

    ros2-talker-listener

And you'll see the familiar output:

.. code-block:: bash

    [talker-1] [INFO] [1646934735.523191674] [talker]: Publishing: 'Hello World: 1'
    [listener-2] [INFO] [1646934735.524428480] [listener]: I heard: [Hello World: 1]
    [talker-1] [INFO] [1646934736.523025881] [talker]: Publishing: 'Hello World: 2'
    [listener-2] [INFO] [1646934736.523614075] [listener]: I heard: [Hello World: 2]

You can find more information about snap
on the `snapcraft documentation <https://snapcraft.io/docs>`_ and `ROS 2 snap page <https://snapcraft.io/docs/ros2-applications>`_.
