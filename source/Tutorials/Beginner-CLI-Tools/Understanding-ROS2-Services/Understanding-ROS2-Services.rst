.. redirect-from::

    Tutorials/Services/Understanding-ROS2-Services

.. _ROS2Services:

Learning about services - tutorial
===================================

Services are one of the communication types used to exchange data in a ROS system.
This article walks you through using command-line tools to examine and call services between nodes.
A hands-on exercise with Turtlesim helps you understand how the call-and-response model works in the ROS graph.

**Area: Framework | Content-type: tutorial | Experience: beginner**

.. contents:: Contents
   :depth: 2
   :local:

Summary
-------

Services use a call-and-response model: a client sends a request to a server, which processes it and returns a response.
Unlike topics, services only provide data when specifically called, so they are not suited for continuous data streams.

Background
----------

Services are another method of communication for nodes in the ROS graph.
Services are based on a call-and-response model, as opposed to the publisher-subscriber model of topics.
While topics allow nodes to subscribe to data streams and get continual updates, services only provide data when specifically called by a client.

.. image:: images/Service-SingleServiceClient.gif

.. image:: images/Service-MultipleServiceClient.gif

Prerequisites
-------------

#. Make sure you understand the concepts of :doc:`nodes <../Understanding-ROS2-Nodes/Understanding-ROS2-Nodes>` and :doc:`topics <../Understanding-ROS2-Topics/Understanding-ROS2-Topics>`.
#. You will need the :doc:`turtlesim package <../Introducing-Turtlesim/Introducing-Turtlesim>`.

Steps
-----

.. note::
    Do not forget to source ROS in every new terminal you open.
    See :doc:`Configuring environment <../Configuring-ROS2-Environment>`.

1 Setup
^^^^^^^

Start up the two Turtlesim nodes, ``/turtlesim`` and ``/teleop_turtle``.

Open a new terminal and run:

.. code-block:: console

  $ ros2 run turtlesim turtlesim_node

Open another terminal and run:

.. code-block:: console

  $ ros2 run turtlesim turtle_teleop_key

2 ros2 service list
^^^^^^^^^^^^^^^^^^^

To list all the services currently active in the system, run ``ros2 service list`` in a new terminal:

.. code-block:: console

  $ ros2 service list
  /clear
  /kill
  /reset
  /spawn
  /teleop_turtle/describe_parameters
  /teleop_turtle/get_parameter_types
  /teleop_turtle/get_parameters
  /teleop_turtle/list_parameters
  /teleop_turtle/set_parameters
  /teleop_turtle/set_parameters_atomically
  /turtle1/set_pen
  /turtle1/teleport_absolute
  /turtle1/teleport_relative
  /turtlesim/describe_parameters
  /turtlesim/get_parameter_types
  /turtlesim/get_parameters
  /turtlesim/list_parameters
  /turtlesim/set_parameters
  /turtlesim/set_parameters_atomically

You will see that both nodes have the same six services with ``parameters`` in their names.
Nearly every node in ROS has these infrastructure services that parameters are built off of.
Parameters are covered in :doc:`Learning about parameters <../Understanding-ROS2-Parameters/Understanding-ROS2-Parameters>`.
In this tutorial, the parameter services will not be discussed in detail.

The turtlesim-specific services are ``/clear``, ``/kill``, ``/reset``, ``/spawn``, ``/turtle1/set_pen``, ``/turtle1/teleport_absolute``, and ``/turtle1/teleport_relative``.
Some of these services were covered in the :doc:`Using turtlesim, ros2, and rqt <../Introducing-Turtlesim/Introducing-Turtlesim>` tutorial.


3 ros2 service type
^^^^^^^^^^^^^^^^^^^

Services have types that describe how the request and response data of a service is structured.
Service types are defined similarly to topic types, except service types have two parts: one message for the request and another for the response.

To find out the type of a service, run:

.. code-block:: console

  $ ros2 service type <service_name>

For example, to check Turtlesim's ``/clear`` service, open a new terminal and run:

.. code-block:: console

  $ ros2 service type /clear
  std_srvs/srv/Empty

The ``Empty`` type means the service call sends no data when making a request and receives no data when receiving a response.

3.1 ros2 service list -t
~~~~~~~~~~~~~~~~~~~~~~~~

To see the types of all the active services at the same time, append the ``--show-types`` option (abbreviated as ``-t``) to the ``list`` command:

.. code-block:: console

  $ ros2 service list -t
  /clear [std_srvs/srv/Empty]
  /kill [turtlesim_msgs/srv/Kill]
  /reset [std_srvs/srv/Empty]
  /spawn [turtlesim_msgs/srv/Spawn]
  ...
  /turtle1/set_pen [turtlesim_msgs/srv/SetPen]
  /turtle1/teleport_absolute [turtlesim_msgs/srv/TeleportAbsolute]
  /turtle1/teleport_relative [turtlesim_msgs/srv/TeleportRelative]
  ...

4 ros2 service info
^^^^^^^^^^^^^^^^^^^

To see information about a particular service, run:

.. code-block:: console

  $ ros2 service info <service_name>

This returns the service type and the count of service clients and servers.

For example, to check the ``/clear`` service:

.. code-block:: console

   $ ros2 service info /clear
   Type: std_srvs/srv/Empty
   Clients count: 0
   Services count: 1

4.1 ros2 service info --verbose
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For more detailed information about a service, append the ``--verbose`` (or ``-v``) option to the command:

.. code-block:: console

  $ ros2 service info --verbose <service_name>

For example, to get verbose information about the ``/clear`` service:

.. code-block:: console

  $ ros2 service info --verbose /clear

The verbose output includes additional information such as the node name and namespace of the service server, as well as the underlying middleware (RMW) implementation details.
Note that ``Endpoint count`` is 2 for DDS-based RMW implementations (connextdds, cyclone, fastrtps), because DDS creates two endpoints per service server: one for request and one for response.

.. code-block:: console

    Type: std_srvs/srv/Empty
    Clients count: 0
    Services count: 1
    Node name: turtlesim
    Node namespace: /
    Service type: std_srvs/srv/Empty
    Service type hash: RIHS01_5888399dedec5ccc85ea6451949fd2c9f97bfdf963f9a588821639fcd31b5d19
    Endpoint type: SERVER
    Endpoint count: 2
    GIDs:
    - Request Reader : 01.0f.93.f0.07.92.53.47.00.00.00.00.00.00.13.04
    - Response Writer : 01.0f.93.f0.07.92.53.47.00.00.00.00.00.00.14.03
    QoS profiles:
    - Request Reader :
          Reliability: RELIABLE
          History (Depth): KEEP_LAST (10)
          Durability: VOLATILE
          Lifespan: Infinite
          Deadline: Infinite
          Liveliness: AUTOMATIC
          Liveliness lease duration: Infinite
    - Response Writer :
          Reliability: RELIABLE
          History (Depth): KEEP_LAST (10)
          Durability: VOLATILE
          Lifespan: Infinite
          Deadline: Infinite
          Liveliness: AUTOMATIC
          Liveliness lease duration: Infinite

For non-DDS RMW implementations such as ``rmw_zenoh_cpp``, ``Endpoint count`` is 1 because a single endpoint handles both request and response.

.. code-block:: console

    Type: std_srvs/srv/Empty
    Clients count: 0
    Services count: 1
    Node name: turtlesim
    Node namespace: /
    Service type: std_srvs/srv/Empty
    Service type hash: RIHS01_5888399dedec5ccc85ea6451949fd2c9f97bfdf963f9a588821639fcd31b5d19
    Endpoint type: SERVER
    Endpoint count: 1
    GID: 59.b0.ea.78.57.3c.52.b4.c6.e9.af.44.22.3d.7c.f5
    QoS profile:
      Reliability: RELIABLE
      History (Depth): KEEP_LAST (10)
      Durability: VOLATILE
      Lifespan: Infinite
      Deadline: Infinite
      Liveliness: AUTOMATIC
      Liveliness lease duration: Infinite

To learn more about different RMW implementations, see :doc:`About Different Middleware Vendors <../../../Concepts/Intermediate/About-Different-Middleware-Vendors>`.

5 ros2 service find
^^^^^^^^^^^^^^^^^^^

To find all the services of a specific type, run:

.. code-block:: console

  $ ros2 service find <type_name>

For example, to find all ``Empty`` typed services:

.. code-block:: console

  $ ros2 service find std_srvs/srv/Empty
  /clear
  /reset

6 ros2 interface show
^^^^^^^^^^^^^^^^^^^^^

You can call services from the command line, but first you need to know the structure of the input arguments.

To inspect a service type's structure, run:

.. code-block:: console

  $ ros2 interface show <type_name>

Try the following command on the ``/clear`` service's type, ``Empty``:

.. code-block:: console

  $ ros2 interface show std_srvs/srv/Empty
  ---

The ``---`` separates the request structure from the response structure.
The ``Empty`` type doesn't send or receive any data, so its structure is blank on both sides.

Let's introspect a service with a type that sends and receives data, like ``/spawn``.
From the results of ``ros2 service list -t``, we know ``/spawn``'s type is ``turtlesim_msgs/srv/Spawn``.

To see the request and response arguments of the ``/spawn`` service, run:

.. code-block:: console

  $ ros2 interface show turtlesim_msgs/srv/Spawn
  float32 x
  float32 y
  float32 theta
  string name # Optional.  A unique name will be created and returned if this is empty
  ---
  string name

The fields above the ``---`` line are the arguments needed to call ``/spawn``:
``x``, ``y`` and ``theta`` determine the 2D pose of the spawned turtle, and ``name`` is optional.

The field below the line is the response: the name assigned to the new turtle.

7 ros2 service call
^^^^^^^^^^^^^^^^^^^

To call a service from the command line, use:

.. code-block:: console

  $ ros2 service call <service_name> <service_type> <arguments>

The ``<arguments>`` part is optional.
For example, ``Empty`` typed services don't have any arguments:

.. code-block:: console

  $ ros2 service call /clear std_srvs/srv/Empty

This clears the Turtlesim window of any lines drawn by the turtle.

.. image:: images/clear.png

Now let's spawn a new turtle by calling ``/spawn`` and setting arguments.
Arguments in a service call from the command line must be in YAML syntax.

To spawn a new turtle, run:

.. code-block:: console

  $ ros2 service call /spawn turtlesim_msgs/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"
  requester: making request: turtlesim_msgs.srv.Spawn_Request(x=2.0, y=2.0, theta=0.2, name='')

  response:
  turtlesim_msgs.srv.Spawn_Response(name='turtle2')

The terminal shows the request that was sent and the service response.

The Turtlesim window updates with the newly spawned turtle:

.. image:: images/spawn.png

8 ros2 service echo
^^^^^^^^^^^^^^^^^^^

To monitor the data communication between a service client and a service server, use:

.. code-block:: console

  $ ros2 service echo <service_name | service_type> <arguments>

``ros2 service echo`` depends on service introspection, which is disabled by default.
To enable it, call ``configure_introspection`` after creating a service client or server.

To try this, start up the ``introspection_client`` and ``introspection_service`` demo:

.. code-block:: console

  $ ros2 launch demo_nodes_cpp introspect_services_launch.py

Open another terminal and enable service introspection for both nodes:

.. code-block:: console

  $ ros2 param set /introspection_service service_configure_introspection contents
  $ ros2 param set /introspection_client client_configure_introspection contents

To see the live communication between ``introspection_client`` and ``introspection_service``, run:

.. code-block:: console

  $ ros2 service echo --flow-style /add_two_ints
   info:
     event_type: REQUEST_SENT
     stamp:
       sec: 1709408301
       nanosec: 423227292
     client_gid: [1, 15, 0, 18, 250, 205, 12, 100, 0, 0, 0, 0, 0, 0, 21, 3]
     sequence_number: 618
   request: [{a: 2, b: 3}]
   response: []
   ---
   info:
     event_type: REQUEST_RECEIVED
     stamp:
       sec: 1709408301
       nanosec: 423601471
     client_gid: [1, 15, 0, 18, 250, 205, 12, 100, 0, 0, 0, 0, 0, 0, 20, 4]
     sequence_number: 618
   request: [{a: 2, b: 3}]
   response: []
   ---
   info:
     event_type: RESPONSE_SENT
     stamp:
       sec: 1709408301
       nanosec: 423900744
     client_gid: [1, 15, 0, 18, 250, 205, 12, 100, 0, 0, 0, 0, 0, 0, 20, 4]
     sequence_number: 618
   request: []
   response: [{sum: 5}]
   ---
   info:
     event_type: RESPONSE_RECEIVED
     stamp:
       sec: 1709408301
       nanosec: 424153133
     client_gid: [1, 15, 0, 18, 250, 205, 12, 100, 0, 0, 0, 0, 0, 0, 21, 3]
     sequence_number: 618
   request: []
   response: [{sum: 5}]
   ---

Related content
---------------

More articles:

* :doc:`Learning about topics <../Understanding-ROS2-Topics/Understanding-ROS2-Topics>`
* :doc:`Learning about nodes <../Understanding-ROS2-Nodes/Understanding-ROS2-Nodes>`
* :doc:`Interfaces (topics, services, actions) </Concepts/Basic/Interfaces-Topics-Services-Actions>`

External resources:

* `Calling ROS 2 services <https://discourse.ubuntu.com/t/call-services-in-ros-2/15261>`_: a realistic application of services using a Robotis robot arm.

FAQs
----

What is the difference between a service and a topic?
   Topics use a publish-subscribe model where data is broadcast continuously to any subscriber.
   Services use a call-and-response model: a client sends a specific request and waits for a single response.
   Use topics for continuous data streams and services for discrete, one-time interactions.

Why do I see so many services with ``parameters`` in their names?
   Nearly every ROS node automatically exposes a set of parameter services.
   These are infrastructure services that the parameter system is built on.
   This tutorial focuses on application-specific services; parameter services are covered in the parameters tutorial.

What does the ``---`` separator mean in ``ros2 interface show`` for a service?
   The ``---`` separates the request fields from the response fields.
   For ``Empty`` typed services, both halves are blank because no data is sent or received.

When should I use a service instead of a topic?
   Use a service when you need a discrete, one-time interaction with a guaranteed response, such as spawning an entity or resetting a node's state.
   For continuous data streams, use a topic.
   For long-running goals that provide feedback, consider using an action.

Why does ``ros2 service echo`` show no output?
   Service introspection is disabled by default.
   Enable it by calling ``configure_introspection`` on both the client and server nodes, which in this tutorial is done via ``ros2 param set`` on the demo nodes.
