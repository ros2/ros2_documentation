.. redirect-from::

    Intra-Process-Communication
    Tutorials/Intra-Process-Communication

Setting up efficient intra-process communication
================================================

.. contents:: Table of Contents
   :depth: 1
   :local:

Background
----------

ROS applications typically consist of a composition of individual "nodes" which perform narrow tasks and are decoupled from other parts of the system.
This promotes fault isolation, faster development, modularity, and code reuse, but it often comes at the cost of performance.
After ROS 1 was initially developed, the need for efficient composition of nodes became obvious and Nodelets were developed.
In ROS 2 we aim to improve on the design of Nodelets by addressing some fundamental problems that required restructuring of nodes.

In this demo we'll be highlighting how nodes can be composed manually, by defining the nodes separately but combining them in different process layouts without changing the node's code or limiting its abilities.

Installing the demos
--------------------

See the :doc:`installation instructions <../../Installation>` for details on installing ROS 2.

If you've installed ROS 2 from packages, ensure that you have ``ros-{DISTRO}-intra-process-demo`` installed.
If you downloaded the archive or built ROS 2 from source, it will already be part of the installation.

Running and understanding the demos
-----------------------------------

There are a few different demos: some are toy problems designed to highlight features of the intra process communications functionality and some are end to end examples which use OpenCV and demonstrate the ability to recombine nodes into different configurations.

The two node pipeline demo
^^^^^^^^^^^^^^^^^^^^^^^^^^

This demo is designed to show that the intra process publish/subscribe connection can result in zero-copy transport of messages when publishing and subscribing with ``std::unique_ptr``\ s.

First let's take a look at the source:

.. rli:: https://github.com/ros2/demos/raw/93e3cf89b3d4882b2733c63a008d13672517cab9/intra_process_demo/src/two_node_pipeline/two_node_pipeline.cpp
    :caption: `intra_process_demo/src/two_node_pipeline/two_node_pipeline.cpp <https://github.com/ros2/demos/blob/93e3cf89b3d4882b2733c63a008d13672517cab9/intra_process_demo/src/two_node_pipeline/two_node_pipeline.cpp>`_
    :language: c++
    :lines: 15-

As you can see by looking at the ``main`` function, we have a producer and a consumer node, we add them to a single threaded executor, and then call spin.

If you look at the "producer" node's implementation in the ``Producer`` struct, you can see that we have created a publisher which publishes on the "number" topic and a timer which periodically creates a new message, prints out its address in memory and its content's value and then publishes it.

The "consumer" node is a bit simpler, you can see its implementation in the ``Consumer`` struct, as it only subscribes to the "number" topic and prints the address and value of the message it receives.

The expectation is that the producer will print out an address and value and the consumer will print out a matching address and value.
This demonstrates that intra process communication is indeed working and unnecessary copies are avoided, at least for simple graphs.

Let's run the demo by executing ``ros2 run intra_process_demo two_node_pipeline`` executable (don't forget to source the setup file first):

.. code-block:: bash

   $ ros2 run intra_process_demo two_node_pipeline
   Published message with value: 0, and address: 0x7fb02303faf0
   Published message with value: 1, and address: 0x7fb020cf0520
    Received message with value: 1, and address: 0x7fb020cf0520
   Published message with value: 2, and address: 0x7fb020e12900
    Received message with value: 2, and address: 0x7fb020e12900
   Published message with value: 3, and address: 0x7fb020cf0520
    Received message with value: 3, and address: 0x7fb020cf0520
   Published message with value: 4, and address: 0x7fb020e12900
    Received message with value: 4, and address: 0x7fb020e12900
   Published message with value: 5, and address: 0x7fb02303cea0
    Received message with value: 5, and address: 0x7fb02303cea0
   [...]

One thing you'll notice is that the messages tick along at about one per second.
This is because we told the timer to fire at about once per second.

Also you may have noticed that the first message (with value ``0``) does not have a corresponding "Received message ..." line.
This is because publish/subscribe is "best effort" and we do not have any "latching" like behavior enabled.
This means that if the publisher publishes a message before the subscription has been established, the subscription will not receive that message.
This race condition can result in the first few messages being lost.
In this case, since they only come once per second, usually only the first message is lost.

Finally, you can see that "Published message..." and "Received message ..." lines with the same value also have the same address.
This shows that the address of the message being received is the same as the one that was published and that it is not a copy.
This is because we're publishing and subscribing with ``std::unique_ptr``\ s which allow ownership of a message to be moved around the system safely.
You can also publish and subscribe with ``const &`` and ``std::shared_ptr``, but zero-copy will not occur in that case.

The cyclic pipeline demo
^^^^^^^^^^^^^^^^^^^^^^^^

This demo is similar to the previous one, but instead of the producer creating a new message for each iteration, this demo only ever uses one message instance.
This is achieved by creating a cycle in the graph and "kicking off" communication by externally making one of the nodes publish before spinning the executor:

.. rli:: https://github.com/ros2/demos/raw/9c4ced3c5be392145312e0c0d3653140a2e29cc0/intra_process_demo/src/cyclic_pipeline/cyclic_pipeline.cpp
    :caption: `intra_process_demo/src/cyclic_pipeline/cyclic_pipeline.cpp <https://github.com/ros2/demos/blob/9c4ced3c5be392145312e0c0d3653140a2e29cc0/intra_process_demo/src/cyclic_pipeline/cyclic_pipeline.cpp>`_
    :language: c++
    :lines: 15-

Unlike the previous demo, this demo uses only one Node, instantiated twice with different names and configurations.
The graph ends up being ``pipe1`` -> ``pipe2`` -> ``pipe1`` ... in a loop.

The line ``pipe1->pub->publish(msg);`` kicks the process off, but from then on the messages are passed back and forth between the nodes by each one calling publish within its own subscription callback.

The expectation here is that the nodes pass the message back and forth, once a second, incrementing the value of the message each time.
Because the message is being published and subscribed to as a ``unique_ptr`` the same message created at the beginning is continuously used.

To test those expectations, let's run it:

.. code-block:: bash

   $ ros2 run intra_process_demo cyclic_pipeline
   Published first message with value:  42, and address: 0x7fd2ce0a2bc0
   Received message with value:         42, and address: 0x7fd2ce0a2bc0
     sleeping for 1 second...
     done.
   Incrementing and sending with value: 43, and address: 0x7fd2ce0a2bc0
   Received message with value:         43, and address: 0x7fd2ce0a2bc0
     sleeping for 1 second...
     done.
   Incrementing and sending with value: 44, and address: 0x7fd2ce0a2bc0
   Received message with value:         44, and address: 0x7fd2ce0a2bc0
     sleeping for 1 second...
     done.
   Incrementing and sending with value: 45, and address: 0x7fd2ce0a2bc0
   Received message with value:         45, and address: 0x7fd2ce0a2bc0
     sleeping for 1 second...
     done.
   Incrementing and sending with value: 46, and address: 0x7fd2ce0a2bc0
   Received message with value:         46, and address: 0x7fd2ce0a2bc0
     sleeping for 1 second...
     done.
   Incrementing and sending with value: 47, and address: 0x7fd2ce0a2bc0
   Received message with value:         47, and address: 0x7fd2ce0a2bc0
     sleeping for 1 second...
   [...]

You should see ever increasing numbers on each iteration, starting with 42... because 42, and the whole time it reuses the same message, as demonstrated by the pointer addresses which do not change, which avoids unnecessary copies.

The image pipeline demo
^^^^^^^^^^^^^^^^^^^^^^^

In this demo we'll use OpenCV to capture, annotate, and then view images.

.. note::

  If you are on macOS and these examples do not work or you receive an error like ``ddsi_conn_write failed -1``, then you'll need to increase your system wide UDP packet size:

  .. code-block:: bash

    $ sudo sysctl -w net.inet.udp.recvspace=209715
    $ sudo sysctl -w net.inet.udp.maxdgram=65500

  These changes will not persist after a reboot.

Simple pipeline
~~~~~~~~~~~~~~~

First we'll have a pipeline of three nodes, arranged as such: ``camera_node`` -> ``watermark_node`` -> ``image_view_node``

The ``camera_node`` reads from camera device ``0`` on your computer, writes some information on the image and publishes it.
The ``watermark_node`` subscribes to the output of the ``camera_node`` and adds more text before publishing it too.
Finally, the ``image_view_node`` subscribes to the output of the ``watermark_node``, writes more text to the image and then visualizes it with ``cv::imshow``.

In each node the address of the message which is being sent, or which has been received, or both, is written to the image.
The watermark and image view nodes are designed to modify the image without copying it and so the addresses imprinted on the image should all be the same as long as the nodes are in the same process and the graph remains organized in a pipeline as sketched above.

.. note::

   On some systems (we've seen it happen on Linux), the address printed to the screen might not change.
   This is because the same unique pointer is being reused. In this situation, the pipeline is still running.

Let's run the demo by executing the following executable:

.. code-block:: bash

   ros2 run intra_process_demo image_pipeline_all_in_one

You should see something like this:


.. image:: images/intra-process-demo-pipeline-single-window.png


You can pause the rendering of the image by pressing the spacebar and you can resume by pressing the spacebar again.
You can also press ``q`` or ``ESC`` to exit.

If you pause the image viewer, you should be able to compare the addresses written on the image and see that they are the same.

Pipeline with two image viewers
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Now let's look at an example just like the one above, except it has two image view nodes.
All the nodes are still in the same process, but now two image view windows should show up. (Note for macOS users: your image view windows might be on top of each other).
Let's run it with the command:

.. code-block:: bash

   ros2 run intra_process_demo image_pipeline_with_two_image_view


.. image:: images/intra-process-demo-pipeline-two-windows-copy.png


Just like the last example, you can pause the rendering with the spacebar and continue by pressing the spacebar a second time. You can stop the updating to inspect the pointers written to the screen.

As you can see in the example image above, we have one image with all of the pointers the same and then another image with the same pointers as the first image for the first two entries, but the last pointer on the second image is different. To understand why this is happening consider the graph's topology:

.. code-block:: bash

   camera_node -> watermark_node -> image_view_node
                                 -> image_view_node2

The link between the ``camera_node`` and the ``watermark_node`` can use the same pointer without copying because there is only one intra process subscription to which the message should be delivered. But for the link between the ``watermark_node`` and the two image view nodes the relationship is one to many, so if the image view nodes were using ``unique_ptr`` callbacks then it would be impossible to deliver the ownership of the same pointer to both. It can be, however, delivered to one of them. Which one would get the original pointer is not defined, but instead is simply the last to be delivered.

Note that the image view nodes are not subscribed with ``unique_ptr`` callbacks. Instead they are subscribed with ``const shared_ptr``\ s. This means the system deliveres the same ``shared_ptr`` to both callbacks. When the first intraprocess subscription is handled, the internally stored ``unique_ptr`` is promoted to a ``shared_ptr``. Each of the callbacks will receive shared ownership of the same message.

Pipeline with interprocess viewer
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

One other important thing to get right is to avoid interruption of the intra process zero-copy behavior when interprocess subscriptions are made. To test this we can run the first image pipeline demo, ``image_pipeline_all_in_one``, and then run an instance of the stand alone ``image_view_node`` (don't forget to prefix them with ``ros2 run intra_process_demo`` in the terminal). This will look something like this:


.. image:: images/intra-process-demo-pipeline-inter-process.png


It's hard to pause both images at the same time so the images may not line up, but the important thing to notice is that the ``image_pipeline_all_in_one`` image view shows the same address for each step. This means that the intra process zero-copy is preserved even when an external view is subscribed as well. You can also see that the interprocess image view has different process IDs for the first two lines of text and the process ID of the standalone image viewer in the third line of text.
