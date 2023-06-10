.. _ImprovedDynamicDiscovery:

Improved Dynamic Discovery
==========================

**Goal:** This tutorial will show how to use the improved dynamic discovery configuration.

**Tutorial level:** Advanced

**Time:** 15 minutes

.. contents:: Table of Contents
   :depth: 2
   :local:

Overview
--------

By default, ROS 2 will attempt to find all nodes on all hosts on the same subnet automatically.
However, the following options are available to control the ROS 2 discovery range.


Configuration Parameters
------------------------

* ``ROS_AUTOMATIC_DISCOVERY_RANGE``: controls how far ROS nodes will try to discover each other.

   Valid options are:

   * ``SUBNET`` is the default, and for DDS based middleware it means it will discover any node reachable via multicast.
   * ``LOCALHOST`` means a node will only try to discover other nodes on the same machine.
   * ``OFF`` means the node won't discover any other nodes, even on the same machine.
   * ``SYSTEM_DEFAULT`` means "don't change any discovery settings".

* ``ROS_STATIC_PEERS``: is a semicolon (``;``) separated list of addresses that ROS should try to discover nodes on.
  This allows connecting to nodes on specific machines (as long as their discovery range is not set to ``OFF``).

The combination of these two environment variables for local and remote nodes will enable and control the ROS 2 communication discovery range.
The following tables highlight the discovery range behavior for possible combination.

A ``X`` indicates that nodes A and B will not discover each other and communicate.
A ``O`` indicates that nodes A and B will discover each other and communicate.

.. list-table:: Node A and B running in the same host
   :widths: 20 20 20 20 20 20 20 20 20
   :header-rows: 1

   * - Same host
     -
     -
     - Node B setting
     -
     -
     -
     -
     -
   * -
     -
     -
     - No static peer
     -
     -
     - With static peer
     -
     -
   * -
     -
     -
     - Off
     - Localhost
     - Subnet
     - Off
     - Localhost
     - Subnet
   * - Node A setting
     - No static peer
     - Off
     - ``X``
     - ``X``
     - ``X``
     - ``X``
     - ``X``
     - ``X``
   * -
     -
     - Localhost
     - ``X``
     - ``O``
     - ``O``
     - ``X``
     - ``O``
     - ``O``
   * -
     -
     - Subnet
     - ``X``
     - ``O``
     - ``O``
     - ``X``
     - ``O``
     - ``O``
   * -
     - With static peer
     - Off
     - ``X``
     - ``X``
     - ``X``
     - ``X``
     - ``X``
     - ``X``
   * -
     -
     - Localhost
     - ``X``
     - ``O``
     - ``O``
     - ``X``
     - ``O``
     - ``O``
   * -
     -
     - Subnet
     - ``X``
     - ``O``
     - ``O``
     - ``X``
     - ``O``
     - ``O``


.. list-table:: Node A and B running in the different hosts
   :widths: 20 20 20 20 20 20 20 20 20
   :header-rows: 1

   * - Different hosts
     -
     -
     - Node B setting
     -
     -
     -
     -
     -
   * -
     -
     -
     - No static peer
     -
     -
     - With static peer
     -
     -
   * -
     -
     -
     - Off
     - Localhost
     - Subnet
     - Off
     - Localhost
     - Subnet
   * - Node A setting
     - No static peer
     - Off
     - ``X``
     - ``X``
     - ``X``
     - ``X``
     - ``X``
     - ``X``
   * -
     -
     - Localhost
     - ``X``
     - ``X``
     - ``X``
     - ``X``
     - ``O``
     - ``O``
   * -
     -
     - Subnet
     - ``X``
     - ``X``
     - ``O``
     - ``X``
     - ``O``
     - ``O``
   * -
     - With static peer
     - Off
     - ``X``
     - ``X``
     - ``X``
     - ``X``
     - ``X``
     - ``X``
   * -
     -
     - Localhost
     - ``X``
     - ``O``
     - ``O``
     - ``X``
     - ``O``
     - ``O``
   * -
     -
     - Subnet
     - ``X``
     - ``O``
     - ``O``
     - ``X``
     - ``O``
     - ``O``


Examples
--------

For example, the following commands will limit the ROS 2 communication only with localhost and specific peers:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
        export ROS_STATIC_PEERS=192.168.0.1;remote.com

      To maintain this setting between shell sessions, you can add the command to your shell startup script:

      .. code-block:: console

        echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" >> ~/.bashrc
        echo "export ROS_STATIC_PEERS=192.168.0.1;remote.com" >> ~/.bashrc

   .. group-tab:: macOS

      .. code-block:: console

        export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
        export ROS_STATIC_PEERS=192.168.0.1;remote.com

      To maintain this setting between shell sessions, you can add the command to your shell startup script:

      .. code-block:: console

        echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" >> ~/.bash_profile
        echo "export ROS_STATIC_PEERS=192.168.0.1;remote.com" >> ~/.bash_profile

   .. group-tab:: Windows

      .. code-block:: console

        set ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
        set ROS_STATIC_PEERS=192.168.0.1;remote.com

      If you want to make this permanent between shell sessions, also run:

      .. code-block:: console

        setx ROS_AUTOMATIC_DISCOVERY_RANGE LOCALHOST
        setx ROS_STATIC_PEERS 192.168.0.1;remote.com
