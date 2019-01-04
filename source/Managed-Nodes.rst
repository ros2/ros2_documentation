
Introduction
------------

ROS2 introduces the concept of managed nodes, also called ``LifecycleNode``\ s. In the following tutorial, we explain the purpose of these nodes, what makes them different from regular nodes and how they comply to a lifecycle management.
Managed nodes are scoped within a state machine of a finite amount of states. These states can be changed by invoking a transition id which indicates the succeeding consecutive state.
The state machine is implemented as described at the `ROS2 design page <http://design.ros2.org/articles/node_lifecycle.html>`__.

Our implementation differentiates between ``Primary States`` and ``Transition States``. Primary States are supposed to be steady states in which any node can do the respected task. On the other hand, Transition States are meant as temporary intermediate states attached to a transition. The result of these intermediate states are used to indicate whether a transition between two primary states is considered successful or not. Thus, any managed node can be in one of the following states:

Primary States (steady states):


* unconfigured
* inactive
* active
* shutdown

Transition States (intermediate states):


* configuring
* activating
* deactivating
* cleaningup
* shuttingdown

The possible transitions to invoke are:


* configure
* activate
* deactivate
* cleanup
* shutdown

For a more verbose explanation on the applied state machine, we refer to the design page which provides an in-detail explanation about each state and transition.

The demo
--------

What's happening
^^^^^^^^^^^^^^^^

The demo is split into 3 different separate applications.


* lifecycle_talker
* lifecycle_listener
* lifecycle_service_client 

The ``lifecycle_talker`` represents a managed node and publishes according to which state the node is in. We split the tasks of the talker node into separate pieces and execute them as followed.


#. configuring: We initialize our publisher and timer
#. activate: We activate the publisher and timer in order to enable a publishing
#. deactivate: We stop the publisher and timer
#. cleanup: We destroy the publisher and timer

The principle is implemented in this demo as the typical talker/listener demo. However, imaging a real scenario with attached hardware which may have a rather long booting phase, i.e. a laser or camera. One could image bringing up the device driver in the configuring state, start and stop only the publishing of the device's data and only in the cleanup/shutdown phase actually shutdown the device. 

The ``lifecycle_listener`` is a simple listener which shows the characteristics of the lifecycle talker. The talker enables the message publishing only in the active state and thus making the listener receiving only messages when the talker is in an active state.

The ``lifecycle_service_client`` is a script calling different transitions on the ``lifecycle_talker``. This is meant as the external user controlling the lifecycle of nodes.   

Run the demo
------------

In order to run this demo, we open three terminals and source our ROS2 environment variables either from the binary distributions or the workspace we compiled from source.

.. list-table::
   :header-rows: 1

   * - lifecycle_talker
     - lifecycle_listener
     - lifecycle_service_client
   * - ``$ ros2 run lifecycle lifecycle_talker``
     - ``$ ros2 run lifecycle lifecycle_listener``
     - ``$ ros2 run lifecycle lifecycle_service_client``
   * - .. image:: https://asciinema.org/a/e0f11qvpberltp8r1w04wzw9t.png
          :target: https://asciinema.org/a/e0f11qvpberltp8r1w04wzw9t
          :alt: asciicast
     - .. image:: https://asciinema.org/a/442pjcu729t3vsld7n225orl7.png
          :target: https://asciinema.org/a/442pjcu729t3vsld7n225orl7
          :alt: asciicast
     - .. image:: https://asciinema.org/a/6o20wbnhx6tk3y2hr5dk8fwm5.png
          :target: https://asciinema.org/a/6o20wbnhx6tk3y2hr5dk8fwm5
          :alt: asciicast
     

Alternatively, these three programs can be run together in the same terminal using the launch file (as of ROS 2 Bouncy):

.. code-block:: bash

   ros2 launch lifecycle lifecycle_demo.launch.py

If we look at the output of the ``lifecycle_talker``\ , we notice that nothing seems to happen. And this does make sense, since every node starts as ``unconfigured``. The lifecycle_talker is not configured yet and in our example, no publishers and timers are created yet.
The same behavior can be seen for the ``lifecycle_listener``\ , which is less surprising given that no publishers are available at this moment.
The interesting part starts with the third terminal. In there we launch our ``lifecycle_service_client`` which is responsible for changing the states of the ``lifecycle_talker``. 

Triggering transition 1 (configure)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   [lc_client] Transition 1 successfully triggered.
   [lc_client] Node lc_talker has current state inactive.

makes the lifecycle talker change its state to inactive. Inactive means that all publishers and timers are created and configured. However, the node is still not active. Therefore no messages are getting published.

.. code-block:: bash

   [lc_talker] on_configure() is called.
   Lifecycle publisher is currently inactive. Messages are not published.
   ...

The lifecycle listener on the same time receives a notification as it listens to every state change notification of the lifecycle talker. In fact, the listener receives two consecutive notifications. One for changing from the primary state "unconfigured" to "configuring". Because the configuring step was successful within the lifecycle talker, a second notification from "configuring" to "inactive". 

.. code-block:: bash

   [lc_listener] notify callback: Transition from state unconfigured to configuring
   [lc_listener] notify callback: Transition from state configuring to inactive

Triggering transition 2 (activate)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   [lc_client] Transition 2 successfully triggered.
   [lc_client] Node lc_talker has current state active.

makes the lifecycle talker change its state to active. Active means that all publishers and timers are now activated. Therefore the messages are now getting published. 

.. code-block:: bash

   [lc_talker] on_activate() is called.
   [lc_talker] Lifecycle publisher is active. Publishing: [Lifecycle HelloWorld #11]
   [lc_talker] Lifecycle publisher is active. Publishing: [Lifecycle HelloWorld #12]
   ...

The lifecycle listener receives the same set of notifications as before. Lifecycle talker changed its state from inactive to active.

.. code-block:: bash

   [lc_listener] notify callback: Transition from state unconfigured to configuring
   [lc_listener] notify callback: Transition from state configuring to inactive

The difference to the transition event before is that our listener now also receives the actual published data.

.. code-block:: bash

   [lc_listener] data_callback: Lifecycle HelloWorld #11
   [lc_listener] data_callback: Lifecycle HelloWorld #12
   ...

Please note that the index of the published message is already at 11. The purpose of this demo is to show that even though we call ``publish`` at every state of the lifecycle talker, only when the state in active, the messages are actually published. As for the beta1, all other messages are getting ignored. This behavior may change in future versions in order to provide better error handling.

For the rest of the demo, you will see similar output as we deactivate and activate the lifecycle talker and finally shut it down. 

The demo code
-------------

lifecycle_talker, lifecycle_listener and lifecycle_service_client
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If we have a look at the code, there is one significant change for the lifecycle talker compared to a regular talker. Our node does not inherit from the regular ``rclcpp::node::Node`` but from ``rclcpp_lifecycle::LifecycleNode``.

.. code-block:: bash

   class LifecycleTalker : public rclcpp_lifecycle::LifecycleNode

Every child of LifecycleNodes have a set of callbacks provided. These callbacks go along with the applied state machine attached to it. These callbacks are:


* ``rcl_lifecycle_ret_t on_configure(const rclcpp_lifecycle::State & previous_state)``
* ``rcl_lifecycle_ret_t on_activate(const rclcpp_lifecycle::State & previous_state)``
* ``rcl_lifecycle_ret_t on_deactivate(const rclcpp_lifecycle::State & previous_state)``
* ``rcl_lifecycle_ret_t on_cleanup(const rclcpp_lifecycle::State & previous_state)``
* ``rcl_lifecycle_ret_t on_shutdown(const rclcpp_lifecycle::State & previous_state)``

All these callbacks have a positive default return value (\ ``return RCL_LIFECYCLE_RET_OK``\ ). This allows a lifecycle node to change its state even though no explicit callback function was overwritten. 
There is one other callback function for error handling. Whenever a state transition throws an uncaught exception, we call ``on_error``. 


* ``rcl_lifecycle_ret_t on_error(const rclcpp_lifecycle::State & previous_state)``

This gives room for executing a custom error handling. Only (!) in the case that this function returns ``RCL_LIFECYCLE_RET_OK``\ , the state machine transitions to the state ``unconfigured``. By default, the ``on_error`` returns ``RCL_LIFECYCLE_RET_ERROR`` and the state machine transitions into ``finalized``. 

At the same time, every lifecycle node has by default 5 different communication interfaces.


* Publisher ``<node_name>__transition_event``\ : publishes in case a transition is happening. This allows users to get notified of transition events within the network.
* Service ``<node_name>__get_state``\ : query about the current state of the node. Return either a primary or transition state.
* Service ``<node_name>__change_state``\ : triggers a transition for the current node. This service call takes a transition id. Only in the case, that this transition ID is a valid transition of the current state, the transition is fulfilled. All other cases are getting ignored.
* Service ``<node_name>__get_available_states``\ : This is meant to be an introspection tool. It returns a list of all possible states this node can be. 
* Service ``<node_name>__get_available_transitions``\ : Same as above, meant to an introspection tool. It returns a list of all possible transitions this node can execute.

ros2 lifecycle
^^^^^^^^^^^^^^

The ``lifecycle_service_client`` application is a fixed order script for this demo purpose only. It explains the use and the API calls made for this lifecycle implementation, but may be inconvenient to use otherwise. For this reason we implemented a command line tool which lets you dynamically change states or various nodes.

In the case you want to get the current state of the ``lc_talker`` node, you would call:

.. code-block:: bash

   $ ros2 lifecycle get /lc_talker
   unconfigured [1]

The next step would be to execute a state change:

.. code-block:: bash

   $ ros2 lifecycle set /lc_talker configure
   Transitioning successful

All of the above commands are nothing else than calling the lifecycle node's services. With that being said, we can also call these services directly with the ros2 command line interface:

.. code-block:: bash

   $ ros2 service call /lc_talker/get_state lifecycle_msgs/GetState 
   requester: making request: lifecycle_msgs.srv.GetState_Request()

   response:
   lifecycle_msgs.srv.GetState_Response(current_state=lifecycle_msgs.msg.State(id=1, label='unconfigured'))

In order to trigger a transition, we call the ``change_state`` service

.. code-block:: bash

   $ ros2 service call /lc_talker/change_state lifecycle_msgs/ChangeState "{transition: {id: 1}}"
   requester: making request: lifecycle_msgs.srv.ChangeState_Request(transition=lifecycle_msgs.msg.Transition(id=1, label=''))

   response:
   lifecycle_msgs.srv.ChangeState_Response(success=True)

It is slightly less convenient, because you have to know the IDs which correspond to each transition. You can find them though in the lifecycle_msgs package.

.. code-block:: bash

   $ ros2 msg show lifecycle_msgs/Transition

Outlook
-------

The above description points to the current state of the development as for beta1. The future todo list for this topic comprises:


* Python lifecycle nodes
* Lifecycle manager: A global node, handling and dispatching trigger requests for multiple nodes.
* LifeyclceSubscriber/LifecycleWalltimer/... add more lifecycle controlled entities.
