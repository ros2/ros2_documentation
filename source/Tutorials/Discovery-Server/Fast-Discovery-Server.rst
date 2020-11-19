.. redirect-from::

    Fast-Discovery-Server

Use ROS 2 with Fast DDS Discovery Server
=========================================

**Goal:** Demo to show how to launch ROS 2 Nodes using the Fast DDS Discovery Server discovery protocol.

**Tutorial level:** Intermediate

**Time:** 20 minutes

.. contents:: Table of Contents
   :depth: 2
   :local:

Background
----------

The **Discovery Server** protocol is a **eProsima** `Fast DDS <https://eprosima.com>`__
feature that offers a dynamical discovering communication.
This tutorial explains how to run some ROS 2 examples using the Discovery Server Fast DDS feature
as discovery communication.

In order to get more information about the available discovery configuration, please check
the `Fast DDS documentation <https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/discovery.html>`_
or read the `Discovery Server specific documentation
<https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/discovery_server.html#discovery-server>`__.

The `Simple Discovery Protocol <https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/simple.html>`__ is the
standard protocol defined in the `DDS standard <https://www.omg.org/omg-dds-portal/>`__.
However, it has certain known disadvantages in some scenarios, mainly:

* It does not **Scale** efficiently, as the number of exchanged packets highly increases as new nodes are added.
* It requires **Multicasting** capabilities that may not work reliably in some scenarios, e.g. WiFi.

The **Discovery Server** provides a Client-Server Architecture that allows
the nodes to connect with each other using an intermediate server.
Each node will work as a *Client*, sharing its info with the *Discovery Server* and receiving
the discovery information from it.
This means that the network traffic is highly reduced in big systems, and it does not require *Multicasting*.

.. image:: figures/ds_explanation.svg
    :align: center

These **Discovery Servers** can be independent, duplicated or connected with each other in order to create
redundancy over the network and avoid having a *Single-Point-Of-Failure*.

Discovery Server v2
-------------------
The new version **v2** of Discovery Server, available from *Fast DDS* v2.0.2, implements a new filter feature
that allows to further reduce the number of discovery messages sent.
This version uses the *topic* of the different nodes to decide if two nodes must be connected, or they
could be left unmatched.
The following schema represents the decrease of the discovery packages:

.. image:: figures/ds1vs2.svg
    :align: center

This architecture reduces the number of packages sent between the server and the different clients dramatically.
In the following graph, the reduction in traffic network over the discovery phase for a
RMF Clinic demo use case, is shown:

.. image:: figures/discovery_server_v2_performance.svg
    :align: center


In order to use this functionality, **Fast-DDS Discovery Server** can be set using
the `XML configuration for Participants <https://fast-dds.docs.eprosima.com/en/latest/
fastdds/discovery/discovery_server.html#discovery-server>`__.
Furthermore, Fast DDS provides an easier way to set a **Discovery Server** communication using
the ``fastdds`` `tool <https://fast-dds.docs.eprosima.com/en/latest/fastddscli/cli/cli.html#discovery>`__
and an `environment variable <https://fast-dds.docs.eprosima.com/en/latest/fastdds/env_vars/env_vars.html>`__,
which are going to be used along this tutorial.
For a more detailed explanation about the configuration of the Discovery Server,
visit Fast DDS `Discovery Server section <https://fast-dds.docs.eprosima.com/en/latest/
fastdds/discovery/discovery_server.html#discovery-server>`__.


Prerequisites
-------------

This tutorial assumes you have a :ref:`working Foxy ROS 2 installation <InstallationGuide>`
In case your installation is using a Fast DDS version lower than v2.0.2 you could not use the ``fastdds`` tool.
You could update your repository to use a different Fast DDS version,
or set the discovery service by `Fast-DDS XML QoS configuration <https://fast-dds.docs.eprosima.com/en/latest/
fastdds/discovery/discovery_server.html#discovery-server>`__.


Run the demo
------------

The ``talker-listener`` ROS 2 demo allows to create a *talker* node that publishes a *Hello World* message every second,
and a *listener* node that listens to these messages.

By `Sourcing ROS 2 <ConfigROS2>`
you will get access to the CLI of *Fast DDS*: ``fastdds``.
This CLI gives access to the `discovery tool <https://fast-dds.docs.eprosima.com/en/latest/
fastddscli/cli/cli.html#discovery>`__,
which allows to launch a server. This server will manage the discovery process for the nodes that connect to it.

.. important::

    Do not forget to `source ROS 2 <ConfigROS2>` in every new terminal opened.


Setup Discovery Server
^^^^^^^^^^^^^^^^^^^^^^^

Start by launching a server with id 0, with port 11811 and listening on all available interfaces.

Open a new terminal and run:

.. code-block:: console

    fastdds discovery -i 0


Launch node listener
^^^^^^^^^^^^^^^^^^^^

Execute the listener demo, that will listen in ``/chatter`` topic.

In a new terminal, set the environment variable ``ROS_DISCOVERY_SERVER`` to use *Discovery Server*.
(Do not forget to source ROS 2 in every new terminal)

.. code-block:: console

    export ROS_DISCOVERY_SERVER=127.0.0.1:11811

Afterwards, launch the listener node. Use the argument ``--remap __node:=listener_discovery_server``
to change the node's name for future purpose.

.. code-block:: console

    ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_discovery_server

This process will create a ROS 2 node, that will automatically create a client for the *Discovery Server*
and use the server created previously to run the discovery protocol.


Launch node talker
^^^^^^^^^^^^^^^^^^

Open a new terminal and set the environment variable as before, so the node raises a client for the discovery protocol.

.. code-block:: console

    export ROS_DISCOVERY_SERVER=127.0.0.1:11811
    ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_discovery_server

Now, we should see the talker publishing *Hello World* messages, and the listener receiving these messages.



Demonstrate Discovery Server execution
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

So far, there is not proof that this example and the standard talker-listener example run differently.
For this purpose, run another node that is not connected to our Discovery Server.
Just run a new listener (listening in ``/chatter`` topic by default) in a new terminal and check that it is
not connected to the talker already running.

.. code-block:: console

    ros2 run demo_nodes_cpp listener --ros-args --remap __node:=simple_listener

In this case, we should not see the listener receiving the messages.

To finally verify that everything is running correctly, a new talker can be created using the
*simple discovery protocol*.

.. code-block:: console

    ros2 run demo_nodes_cpp talker --ros-args --remap __node:=simple_talker

Now we should see the listener *simple_listener* receiving the messages from *simple_talker* but not the other
messages from *talker_discovery_server*.


Visualization tool ``rqt_graph``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The tool ``rqt_graph`` can be used to verify the nodes and structure of this example.
Remember, in order to use the ``rqt_graph`` with the *Discovery Server Protocol*
(i.e., to see the ``listener/talker_discovery_server``)
the environment variable ``ROS_DISCOVERY_SERVER`` must be set beforehand.



Advance user cases
------------------

The following paragraphs are going to show different features of the Discovery Server
that allow to hold a robust structure over the node's network.

Server Redundancy
^^^^^^^^^^^^^^^^^

By using the Fast DDS tool, several servers can be created, and the nodes can be connected to as many
servers as desired. This allows to have a safe redundancy network that will work even if some servers or
nodes shut down unexpectedly.
Next schema shows a simple architecture that will work with server redundancy:

.. image:: figures/ds_redundancy_example.svg
    :align: center

In different terminals, run the next code to establish a communication over redundant servers.

.. code-block:: console

    fastdds discovery -i 0 -l 127.0.0.1 -p 11811

.. code-block:: console

    fastdds discovery -i 1 -l 127.0.0.1 -p 11888

``-i N`` means server with id N. When referencing the servers with ``ROS_DISCOVERY_SERVER``,
server ``0`` must be in first place and server ``1`` in second place.

.. code-block:: console

    export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11888"
    ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker

.. code-block:: console

    export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11888"
    ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener

Now, if one of these servers fails, there would still be discovery communication between nodes.


Backup Server
^^^^^^^^^^^^^

*Fast DDS* Discovery Server allows to easily build a server with a **backup** functionality.
This allows the server to retake the last state it saved in case of a shutdown.

.. image:: figures/ds_backup_example.svg
    :align: center

In different terminals, run the next code to establish a communication over a backup server.

.. code-block:: console

    fastdds discovery -i 0 -l 127.0.0.1 -p 11811 -b

.. code-block:: console

    export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
    ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker

.. code-block:: console

    export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
    ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener

Several backup files are created in the path the server has run.
Two ``SQLite`` files and two ``json`` files that contains the information required to
raise a new server in case of failure, avoiding the whole discovery process to happen again and
without losing information.


Discovery partitions
^^^^^^^^^^^^^^^^^^^^

The **Discovery Server** communication could be used with different servers to split in virtual
partitions the discovery info.
This means that two endpoints only would know each other if there is a server or a server network
between them.
We are going to execute an example with two different independent servers.
The following image shows a schema of the architecture desired:

.. image:: figures/ds_partition_example.svg
    :align: center

With this schema *Listener 1* will be connected to *Talker 1* and *Talker 2*, as they
share *Server 1*.
*Listener 2* will connect with *Talker 1* as they share *Server 2*.
But *Listener 2* will not hear the messages from *Talker 2* because they do not
share any server or servers' network that connect them.

Run the first server listening in localhost in default port 11811.

.. code-block:: console

    fastdds discovery -i 0 -l 127.0.0.1 -p 11811

In another terminal run the second server listening in localhost in port another port, in this case 11888.

.. code-block:: console

    fastdds discovery -i 1 -l 127.0.0.1 -p 11888

Now, run each node in a different terminal. Use the *environment variable* ``ROS_DISCOVERY_SERVER`` to decide which
server they are connected to. Be aware that the `ids must match
<https://fast-dds.docs.eprosima.com/en/latest/fastdds/env_vars/env_vars.html>`__.

.. code-block:: console

    export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11888"
    ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_1

.. code-block:: console

    export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11888"
    ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_1

.. code-block:: console

    export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
    ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_2

.. code-block:: console

    export ROS_DISCOVERY_SERVER=";127.0.0.1:11888"
    ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_2

We should see how *Listener 1* is receiving double messages, while *Listener 2* is in a different
partition from *Talker 2* and so it does not listen to it.

.. note::

    Once two endpoints know each other, they do not need the server network between them to
    listen to each other messages.



Compare Discovery Server with Simple Discovery
-----------------------------------------------

In order to compare the ROS2 execution using *Simple Discovery* or *Discovery Service*, two scripts that
execute a talker and many listeners and analyze the network traffic during this time are provided.
For this experiment, ``tshark`` is required to be installed on your system.
The configuration file is mandatory in order to avoid using intraprocess mode.

These scripts' functionalities are references for advance purpose and their study is left to the user.

:download:`bash network traffic generator <scripts/generate_discovery_packages.bash>`

:download:`python3 graph generator <scripts/discovery_packets.py>`

:download:`XML configuration <scripts/no_intraprocess_configuration.xml>`

Run the bash script with the *setup* path to source ROS2 as argument.
This will generate the traffic trace for simple discovery.
Executing the same script with second argument ``SERVER``, it will generates the trace for service discovery.

.. note::

    Depending on your configuration of ``tcpdump``, this script may require ``sudo`` privileges to read traffic across
    your network device.

After both executions are done, run the python script to generates a graph similar to the one below:

.. code-block:: console

    $ export FASTRTPS_DEFAULT_PROFILES_FILE="no_intraprocess_configuration.xml"
    $ sudo bash generate_discovery_packages.bash ~/ros2_foxy/install/local_setup.bash
    $ sudo bash generate_discovery_packages.bash ~/ros2_foxy/install/local_setup.bash SERVER
    $ python3 discovery_packets.py

.. image:: figures/discovery_packets.svg
    :align: center

This graph is the result of an specific example, the user can execute the scripts and watch their own results.
It can easily be seen how the network traffic is reduced when using *Discovery Service*.

The reduction in traffic is a result of avoiding every node announcing itself and waiting a response from every other
node in the net.
This creates a huge amount of traffic in large architectures.
This reduction from this method increases with the number of Nodes, making this architecture more scalable than the
simple one.

Since *Fast DDS* v2.0.2 the new Discovery Server v2 is available, substituting the old Discovery Server.
In this new version, those nodes that do not share topics will not know each other, saving the whole discovery data
required to connect them and their endpoints.
Notice that this is not this example case, but even though the massive reduction could be appreciate
due to the hidden architecture topics of ROS 2 nodes.
