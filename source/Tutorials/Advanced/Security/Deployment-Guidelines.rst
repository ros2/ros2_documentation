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

Typical deployment scenarios often involve shipping containerized applications, or packages, into remote systems.
Special attention should be payed when deploying security enabled applications, requiring users to reason about the sensitivity of packaged files.

Complying with the `DDS Security standard <https://www.omg.org/spec/DDS-SECURITY/1.1/About-DDS-SECURITY/>`_,
the ``sros2`` package provides a collection of utilities for managing security under ROS 2 environments in a highly modular and flexible fashion.

Basic core guidelines on how to organize the different certificates, keys and directories remains a critical factor to avoid compromising the security of the system.
This includes protection-awareness and criteria for selecting the minimum set of necessary files to be deployed upon remote production systems for minimizing security exposure.

Prerequisites
-------------

* A docker installation with the compose plugin.
  Please refer to the installation steps detailed in `Docker installation <https://docs.docker.com/engine/install/>`_ and `Compose Plugin <https://docs.docker.com/compose/install>`_.
* (Recommended) A basic understanding on `ROS 2 Security design <https://design.ros2.org/articles/ros2_dds_security.html>`_.
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

    * Not all the generated enclaves should be deployed to all target devices.
    * A reasonable way to proceed would be having one enclave per application, allowing for a separation of concerns.

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

To illustrate a simple deployment scenario, a new docker image will be built on top of the one provided by ``ros:<DISTRO>``.
Starting from the image, three containers will be created with the aim of:

* Initializing the keystore in a local host's shared volume.
* Simulating two deployed remote devices that interact with each other in a secure way.

In this example, the local host serves as the organization's system.
Let us start by creating a workspace folder:

.. code-block:: bash

  mkdir ~/security_gd_tutorial
  cd ~/security_gd_tutorial

Generating the Docker Image
^^^^^^^^^^^^^^^^^^^^^^^^^^^

In order to build a new docker image, a Dockerfile is required.
The one proposed for this tutorial can be retrieved with the following command:

.. code-block:: bash

  # Download the Dockerfile
  wget https://raw.githubusercontent.com/ros2/ros2_documentation/{DISTRO}/source/Tutorials/Advanced/Security/resources/deployment_gd/Dockerfile

Now, build the docker image with the command:

.. code-block:: bash

  # Build the base image
  docker build -t ros2_security/deployment_tutorial --build-arg ROS_DISTRO={DISTRO} .

Understanding the compose file
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A compose configration file takes an image to create containers as services.
In this tutorial, three services are defined within the configuration:

* *keystore-creator*: That, similarly to previous tutorials, it internally initializes a new keystore tree directory.
  This will create *enclaves/* *public/* and *private/*, which are explained in more detail in `ROS 2 Security enclaves <https://design.ros2.org/articles/ros2_security_enclaves.html>`_.
  The ``keystore`` directory is configured to be a shared volume across containers.

* *listener* and *talker*: Act as the remote device actors in this tutorial.
  Required ``Security`` environment variables are sourced as well as the necessary keystore files from the shared volume.

The compose configuration yaml file can be downloaded with:

.. code-block:: bash

  # Download the compose file
  wget https://raw.githubusercontent.com/ros2/ros2_documentation/{DISTRO}/source/Tutorials/Advanced/Security/resources/deployment_gd/compose.deployment.yaml

Running the example
-------------------

In the same working directory ``~/security_gd_tutorial``, run:

.. code-block:: bash

  # Start the example
  docker compose -f compose.deployment.yaml up

This should result in the following output:

- *tutorial-listener-1*: ``Found security directory: /keystore/enclaves/talker_listener/listener``
- *tutorial-talker-1*: ``Found security directory: /keystore/enclaves/talker_listener/talker``
- *tutorial-listener-1*: ``Publishing: 'Hello World: <number>'``
- *tutorial-talker-1*: ``I heard: [Hello World: <number>]``

Examining the containers
^^^^^^^^^^^^^^^^^^^^^^^^

While having the containers running that simulate the two remote devices for this tutorial, attach to each of them by opening two different terminals and enter:

.. code-block:: bash

  # Terminal 1
  docker exec -it tutorial-listener-1 bash
  cd keystore
  tree

  # Terminal 2
  docker exec -it tutorial-talker-1 bash
  cd keystore
  tree

A similar output to the one depicted below should be obtained:

.. code-block:: bash

  # Terminal 1
  keystore
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

  # Terminal 2
  keystore
   ├── enclaves
   │   ├── governance.p7s
   │   ├── governance.xml
   │   └── talker_listener
   │       └── talker
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

Note that:

* *private/* folder is not moved but left in the local host (organization).
* Each one of the deployed devices contain its own minimum enclave required for its application.

.. note::

  For the sake of simplicity, the same CA is used within this enclave for both identity and permissions.
