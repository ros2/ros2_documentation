Deployment Guidelines
=====================

**Goal:** Understand the best practices when deploying security artifacts into production systems.

**Tutorial level:** Advanced

**Time:** 20 minutes

.. contents:: Contents
   :depth: 2
   :local:


Background
----------

Typical deployment scenarios oftenly involve shipping containerized applications, or packages, into remote systems.
Special attention should be payed when deploying security enabled applications, requiring users to reason about the sensitivity of packaged files.

Complying with the `DDS Security standard <https://www.omg.org/spec/DDS-SECURITY/1.1/About-DDS-SECURITY/>`_,
the ``sros2`` package provides a collection of utilities for managing security under ROS 2 environments in a highly modular and flexible fashion.

Basic core guidelines on how to organize the different certificates, keys and directories remains a critical factor to avoid compromising the security of the system.
This includes protection-awareness and criteria for selecting the minimum set of necessary files to be deployed upon remote production systems for minimizing security exposure.

Prerequisites
-------------

* A docker installation available. Please refer to the installation steps detailed in `Docker installation <https://docs.docker.com/engine/install/>`_.
* (Recommended) A basic understanding on `ROS2 Security design <https://design.ros2.org/articles/ros2_dds_security.html>`_.
* (Recommended) Previous security tutorials completion. In particular:

    * :doc:`Introducing-ros2-security`
    * :doc:`The-Keystore`
    * :doc:`Access-Controls`

General Guidelines
------------------

ROS 2 leverages DDS Security extensions to ensure security on message exchanges within the same enclave.
The different signed files and certificates within an enclave are generated from the private keys and certificates of a `Certificate Authority (CA) <https://en.wikipedia.org/wiki/Certificate_authority>`_ trusted entity.
In fact, two different CA's can be selected for identity and permissions, per enclave.
Those CA artifacts are stored inside ``private/`` and ``public/`` sub-directories of a `Keystore <https://design.ros2.org/articles/ros2_security_enclaves.html>`_ with the following folder structure:

.. code-block:: text

  keystore
  ├── enclaves
  │   └── ...
  │       └── ...
  ├── private
  │   └── ...
  └── public
      └── ...

A good practice for the creation and usage of a certain Certificate Authority on a typical deployment for a production system, is to:

#. Create it within the organization system intended for internal use only.
#. Generate/modify desired enclaves bearing in mind that:

    A. Not all the generated enclaves should be embarked into all target devices.
    #. A reasonable way to proceed would be having one enclave per application, allowing for a separation of concerns.

#. Ship ``public/`` alongside with corresponding ``enclaves/`` into the different remote production devices during setup.
#. Keep and protect ``private/`` keys and/or certification requests in the organization.

It is important to note that if ``private/`` files are lost, it won't be possible to change access permissions, add or modify security profiles anymore.

In addition, further practices may be taken into consideration:

* Granting read-only permissions to the ``enclaves/`` directory contents.
* If a PKCS#11 compliant URI is given for generating enclave's private keys, a `Hardware Security Module (HSM) <https://en.wikipedia.org/wiki/Hardware_security_module>`_ could be used to store them.

The following table depicts a summary of the previous statements relating the Keystore directory with the Recommended location:

+------------------------+--------------+---------------+---------------------+
| Directory / Location   | Organization | Target Device | Material Sensitivity|
+========================+==============+===============+=====================+
| public                 |       ✓      |       ✓       |         Low         |
+------------------------+--------------+---------------+---------------------+
| private                |       ✓      |       ✕       |         High        |
+------------------------+--------------+---------------+---------------------+
| enclaves               |       ✓      |       ✓       |        Medium       |
+------------------------+--------------+---------------+---------------------+


Building a deployment scenario
------------------------------

To illustrate a simple deployment scenario, a docker image will be built which is intented to be the remote production target device actor.
In this example, localhost will serve as the organization's system.
To test security capabilities, a secure listener will be launched on the remote system, whereas a secure talker will be launched on the local host.
Let us start by creating a workspace tree with the following sub-folders:

.. code-block:: bash

  mkdir -p ~/deploy_gd_tutorial/remote_system
  mkdir ~/deploy_gd_tutorial/keystore

Generating a Keystore and necessary Enclaves
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Similarly to previous tutorials, intialize a new keystore tree directory.
This will create *enclaves/* *public/* and *private/* directories, which are explained in more detail in `ROS2 Security enclaves <https://design.ros2.org/articles/ros2_security_enclaves.html>`_.

.. code-block:: bash

  # Source ROS installation
  source /opt/ros/${ROS_DISTRO}/setup.bash
  # Initialize a new keystore directory
  ros2 security create_keystore ~/deploy_gd_tutorial/keystore

Next, create an enclave for the local talker node within the */keystore* directory.

.. code-block:: bash

  # Create secure talker's enclave
  ros2 security create_enclave ~/deploy_gd_tutorial/keystore /talker_listener/talker

At this point, step into the remote_system workspace, create the corresponding enclave and copy just the *public/* and *enclaves/* directories to the current one.
Those security artifacts will be needed by the remote system to enable listener's security.
For the sake of simplicity, the same CA is used within this enclave for both identity and permissions.
Note that *private/* folder is not moved but left in localhost (organization).

.. code-block:: bash

  # Create an enclave for the secure listener's enclave
  ros2 security create_enclave ~/deploy_gd_tutorial/keystore /talker_listener/listener

At the end of these steps, the structure of */enclaves* sub-directory within *~/deploy_gd_tutorial/keystore* should look like the following:

.. code-block:: text

  enclaves
  ├── governance.p7s
  ├── governance.xml
  └── talker_listener
      ├── listener
      │   ├── cert.pem
      │   ├── governance.p7s
      │   ├── identity_ca.cert.pem
      │   ├── key.pem
      │   ├── permissions_ca.cert.pem
      │   ├── permissions.p7s
      │   └── permissions.xml
      └── talker
          ├── cert.pem
          ├── governance.p7s
          ├── identity_ca.cert.pem
          ├── key.pem
          ├── permissions_ca.cert.pem
          ├── permissions.p7s
          └── permissions.xml

Now, create and populate the */keystore* directory that will be copied onto the remote system with only the necessary files.

.. code-block:: bash

  # Move to remote system path
  cd ~/deploy_gd_tutorial/remote_system
  # Ship governance files, listener enclave and public/ directories only
  # minimizing security threat
  mkdir -p keystore/enclaves/talker_listener
  cp -R ../keystore/public keystore
  cp -R ../keystore/enclaves/governance.* keystore/enclaves
  cp -R ../keystore/enclaves/talker_listener/listener keystore/enclaves/talker_listener

After the former commands, the current ``~/deploy_gd_tutorial/remote_system`` directory should be:

.. code-block:: text

  remote_system
  └── keystore
      ├── enclaves
      │   ├── governance.p7s
      │   ├── governance.xml
      │   └── talker_listener
      │       └── listener
      │           ├── cert.pem
      │           ├── governance.p7s
      │           ├── identity_ca.cert.pem
      │           ├── key.pem
      │           ├── permissions_ca.cert.pem
      │           ├── permissions.p7s
      │           └── permissions.xml
      └── public
          ├── ca.cert.pem
          ├── identity_ca.cert.pem
          └── permissions_ca.cert.pem


Creating remote's system docker image
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
To get started, change into the remotes's workspace path with:

.. code-block:: bash

  cd ~/deploy_gd_tutorial/remote_system

For the purpose of running a secure listener at the docker image startup, a new ``entrypoint.sh`` file is required, in the current directory, with the following content:

.. code-block:: bash

  #!/bin/bash
  source /opt/ros/${ROS_DISTRO}/setup.bash
  ros2 run demo_nodes_cpp listener --ros-args --enclave /talker_listener/listener $@

In order to build a new docker image, a Dockerfile is also needed.
Create a new file ``Dockerfile`` in the same directory with preferred text editor.

.. code-block:: bash

  ARG ROS_DISTRO=humble
  FROM ros:${ROS_DISTRO}-ros-base

  RUN apt-get update && apt-get install -y \
        ros-${ROS_DISTRO}-demo-nodes-cpp \
        ros-${ROS_DISTRO}-demo-nodes-py && \
      rm -rf /var/lib/apt/lists/*

  ARG KEYSTORE_DIR=/keystore

  RUN mkdir -p ${KEYSTORE_DIR}/enclaves \
    mkdir ${KEYSTORE_DIR}/public

  COPY keystore ${KEYSTORE_DIR}

  ENV ROS_SECURITY_KEYSTORE=${KEYSTORE_DIR}
  ENV ROS_SECURITY_ENABLE=true
  ENV ROS_SECURITY_STRATEGY=Enforce

  COPY entrypoint.sh /entrypoint.sh
  RUN chmod +x /entrypoint.sh

  ENTRYPOINT ["/entrypoint.sh"]

The resultant directory structure should be the one depicted below:

.. code-block:: text

  remote_system
    ├── Dockerfile
    ├── entrypoint.sh
    └── keystore
        ├── enclaves
        │   ├── ...
        └── public
            ├── ...

Build the docker image with the command:

.. code-block:: bash

  # Build remote's system image
  docker build -t ros2_security/deployment_tutorial .


Running the example
-------------------

Launch the following commands in two different terminals:

Open a new terminal and run:

.. code-block:: bash

    # Start remote system container
    docker run -it ros2_security/deployment_tutorial

Then, open a second terminal and run the following commands:

.. code-block:: bash

    # Export ROS security environment variables
    export ROS_SECURITY_KEYSTORE=~/deploy_gd_tutorial/keystore
    export ROS_SECURITY_ENABLE=true
    export ROS_SECURITY_STRATEGY=Enforce

    # Source ROS installation and run the talker
    source /opt/ros/${ROS_DISTRO}/setup.bash
    ros2 run demo_nodes_cpp talker --ros-args --enclave /talker_listener/talker

With the realization of above steps, subsequent output is obtained:

* On host's talker node: ``Publishing: 'Hello World: <number>'``
* While on the remote system side: ``I heard: [Hello World: <number>]``


