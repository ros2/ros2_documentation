Packaging your ROS 2 application as a snap [community-contributed]
==================================================================

.. contents:: Table of Contents
   :depth: 2
   :local:

What are snaps?
---------------

`Snaps <https://snapcraft.io/docs/robotics>`_ are containers that bundle an application and all its dependencies.
They offer several features that address important concerns as one gets closer to shipping a robotic platform:

- **Container solution**: Snaps bundle your application along with all the necessary dependencies and assets in one package including ROS 2. Your application is then easily installable on dozens of Linux distributions and across distro versions.
- **Strict confinement**: Snaps are designed to be `secure and isolated <https://snapcraft.io/docs/snap-confinement>`_ from the underlying system and other applications, with `dedicated interfaces <https://snapcraft.io/docs/supported-interfaces>`_ to access the host machine.
- **Managing updates**: Snaps can update `automatically and transactionally <https://snapcraft.io/docs/keeping-snaps-up-to-date>`_, making sure your robot is never broken and always up-to-date.
- **Release management**: Snaps' `multiple release channels <https://snapcraft.io/docs/channels>`_ allow you to have role-based access controls and application versioning, making A/B testing easy and releasing fixes faster.

Creating a snap
---------------

This tutorial will demonstrate how to use `snapcraft <https://github.com/snapcore/snapcraft>`_ to package your ROS 2 application as a snap, and then how to use it.

First, let us install snapcraft.

.. code-block:: bash

    sudo snap install --classic snapcraft

(Note that the snapcraft debian package from the apt repositories is largely deprecated. One should use the snap package.)

Snapcraft has built-in support for ``Colcon``.

For our example, we will use the ``demo_nodes_cpp`` from the `ros2_demos <https://github.com/ros2/demos/tree/{DISTRO}>`_.

Initialize a new snapcraft project here:

.. code-block:: bash

    mkdir ~/demo_nodes_cpp_snap
    cd ~/demo_nodes_cpp_snap
    snapcraft init

This will create a file in a subdirectory ``snap/snapcraft.yaml``.

The snapcraft file
^^^^^^^^^^^^^^^^^^

Open the freshly created ``snap/snapcraft.yaml`` file and copy over the following:

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

Don't worry, we will break it down together.

Metadata
""""""""

.. code-block:: yaml

    name: ros2-talker-listener
    version: '0.1'
    summary: ROS 2 Talker/Listener example
    description: |
      This example launches a ROS 2 talker and listener.

This is the basic `metadata <https://snapcraft.io/docs/snapcraft-top-level-metadata>`_ that all snaps require.
These fields are fairly self-explanatory but note that the name must be globally unique across all snaps.

Base
""""

.. code-block:: yaml

    base: core20

The `base <https://snapcraft.io/docs/base-snaps>`_ keyword defines a special kind of snap that provides a run-time environment with a minimal set of libraries that are common to most applications.
`Core20 <https://snapcraft.io/core20>`_ is the current standard base for snap building and is akin to `Ubuntu 20.04 LTS <http://releases.ubuntu.com/20.04/>`_.
It is, therefore, the base used for {DISTRO}.

Security model
""""""""""""""

.. code-block:: yaml

    confinement: devmode

To get started, we won't confine this application.
Unconfined applications, specified with ``devmode``,
can only be released to the ``edge`` channel of the snapcraft store.
For more information about snaps Security model, please refer to the `online documentation <https://snapcraft.io/docs/choosing-a-security-model>`_

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

Parts define how to build the app.
In this case, we have one: ``ros-demos``.
Parts can point to local directories, remote git repositories, or tarballs.
Here, we specify our source as a GitHub repository at a specific branch.
We also specifically tell ``Colcon`` to build the ``demo_nodes_cpp`` package.
Furthermore we tell snapcraft that packages such as ``make`` are necessary at build time while the package ``ros-{DISTRO}-ros2launch`` is necessary at run time.
For more information about the plugin and it options, please refer to the `online documentation <https://snapcraft.io/docs/the-colcon-plugin>`_.

Apps
""""

.. code-block:: yaml

    apps:
      ros2-talker-listener:
        command: opt/ros/{DISTRO}/bin/ros2 launch demo_nodes_cpp talker_listener.launch.py
        extensions: [ros2-{DISTRO}]

Apps are the commands exposed to end users.
Each key under apps is the command name that should be made available on users' systems.
The ``command`` keyword specifies the command to be run as its name suggests.
Finally, the extensions `ros2-{DISTRO} <https://snapcraft.io/docs/ros2-extension>`_ essentially sets up the ROS 2 apt package repository together with the necessary environment variables.

Building the snap
^^^^^^^^^^^^^^^^^

Now that we are all set up, let's build the snap:

.. code-block:: bash

    cd ~/demo_nodes_cpp_snap
    snapcraft --enable-experimental-extensions

Giving:

.. code-block:: bash

    *EXPERIMENTAL* extensions enabled.
    Launching a VM.
    Launched: snapcraft-ros2-talker-listener
    [...]
    Snapped ros2-talker-listener_0.1_amd64.snap

That will take a few minutes.
From the logs, and among other things, we can see snapcraft using `rosdep <http://docs.ros.org/independent/api/rosdep/html/>`_ to pull the dependencies for our example but also ``Colcon`` building the application.

Testing the snap
^^^^^^^^^^^^^^^^

This snap is completely standalone: it includes ROS 2 and our application, meaning that one doesn't even need to install ROS 2 on the host system.
Let's test it out:

.. code-block:: bash

    sudo snap install ros2-talker-listener_0.1_amd64.snap --devmode

Note that we use ``--devmode`` here because the snap confinement is set as ``devmode``.
The moment of truth, will it run?

.. code-block:: bash

    ros2-talker-listener

.. code-block:: bash

    [talker-1] [INFO] [1646934735.523191674] [talker]: Publishing: 'Hello World: 1'
    [listener-2] [INFO] [1646934735.524428480] [listener]: I heard: [Hello World: 1]
    [talker-1] [INFO] [1646934736.523025881] [talker]: Publishing: 'Hello World: 2'
    [listener-2] [INFO] [1646934736.523614075] [listener]: I heard: [Hello World: 2]

It does! We see the expected output!

You can find more information about snap on the `snapcraft documentation <https://snapcraft.io/docs>`_ and `ROS 2 snap page <https://snapcraft.io/docs/ros2-applications>`_.
