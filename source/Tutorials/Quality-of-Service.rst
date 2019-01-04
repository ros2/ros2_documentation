
Quality of service
==================

Background
----------

Please read the `About Quality of Service Settings <About-Quality-of-Service-Settings>` page for background information about the Quality of Service settings available in ROS 2.

In this demo, we will spawn a node that publishes a camera image and another that subscribes to the image and shows it on the screen.
We will then simulate a lossy network connection between them and show how different quality of service settings handle the bad link.

Build/install the demo
----------------------

From pre-compiled binaries
^^^^^^^^^^^^^^^^^^^^^^^^^^

Simply download the binary packages for your OS from the `installation page <../Installation>`.

From source
^^^^^^^^^^^

OpenCV is a prerequisite for the QoS demo.
See the `OpenCV documentation <http://docs.opencv.org/doc/tutorials/introduction/table_of_content_introduction/table_of_content_introduction.html#table-of-content-introduction>`__ for installation instructions.
Follow the instructions on the `installation page <building-from-source>` for your particular platform.

Run the demo
------------

Before running the demo, make sure you have a working webcam connected to your computer.

Once you've installed ROS 2, if you're on Linux, source your setup.bash file:

.. code-block:: bash

   . <path to ROS 2 install space>/setup.bash

or if you're on Windows *\ *cmd*\ :

.. code-block:: bash

   call <path to ROS 2 install space>/local_setup.bat

Then run:

.. code-block:: bash

   ros2 run image_tools showimage

Nothing will happen yet.
``showimage`` is a subscriber node that is waiting for a publisher on the ``image`` topic.

Note: you have to close the ``showimage`` process with ``Ctrl-C`` later.
You can't just close the window.

In a separate terminal, source the install file and run the publisher node:

.. code-block:: bash

   ros2 run image_tools cam2image

This will publish an image from your webcam. In case you don't have a camera attached to your computer, there is a commandline option which publishes predefined images.

.. code-block:: bash

   ros2 run image_tools cam2image -b

In this window, you'll see terminal output:

.. code-block:: bash

   Publishing image #1
   Publishing image #2
   Publishing image #3
   ...

A window will pop up with the title "view" showing your camera feed.
In the first window, you'll see output from the subscriber:

.. code-block:: bash

   Received image #1
   Received image #2
   Received image #3
   ...

Note for OS X users: If you these examples do not work or you receive an error like ``ddsi_conn_write failed -1`` then you'll need to increase your system wide UDP packet size:

.. code-block:: bash

   $ sudo sysctl -w net.inet.udp.recvspace=209715
   $ sudo sysctl -w net.inet.udp.maxdgram=65500

These changes will not persist a reboot. If you want the changes to persist, add these lines to ``/etc/sysctl.conf`` (create the file if it doesn't exist already):

.. code-block:: bash

   net.inet.udp.recvspace=209715
   net.inet.udp.maxdgram=65500

Command line options
^^^^^^^^^^^^^^^^^^^^

In one of your terminals, add a -h flag to the original command:

.. code-block:: bash

   ros2 run image_tools showimage -- -h

You'll see a list of the possible options you can pass to the demo.

``-h``\ : The help message.

``-r``\ : Reliability.
There are two options for this policy: reliable or best effort.
Reliable means that values may be reset and the underlying DDS publisher might block, in order for messages to get delivered in order.
Best effort means that messages will get sent as is, and they may get dropped or lost without effecting the behavior of the publisher.

``-k``\ : History policy (the "k" stands for "keep").
Determines how DDS buffers messages in the time between the user code that called ``publish`` and the time when the message actually gets sent.
There are two options for history: KEEP_ALL and KEEP_LAST.
KEEP_ALL will buffer all messages before they get sent.
KEEP_LAST limits the number of buffered messages to a depth specified by the user.

``-d``\ : Queue depth.
Only used if the history policy is set to KEEP_LAST.
The queue depth determines the maximum number of not yet received messages that get buffered on the sender's side before messages start getting dropped.

``-t TOPIC``\ : Topic to use.
The topic to use (default: image)

If you run ``cam2image -h``\ , you'll see the same set of command line options and some additional ones:

``-s``\ : Toggle displaying the input camera stream.
If you run ``cam2image -s`` by itself, you'll see a camera window.
If you also run ``showimage``\ , you'll see two camera windows.

``-x`` and ``-y``\ : Set the size of the camera feed (x sets the width, y sets the height).

``-b``\ : Produce images of burgers rather than connecting to a camera

``-f``\ : Publish frequency in Hz. (default: 30)

The default quality of service settings are tuned for maximum reliability: the reliability policy is reliable, and the history policy is "keep all".

It's worth noting that both ends must have the same reliability settings for this to work.
If the consumer requires the publisher to be reliable, DDS will not match them and there won't be any exchange between them.

We won't see much of a difference if we change the quality of service settings, since the publisher and subscriber are passing messages over inter-process communication, and messages are unlikely to get dropped if they are travelling within the same machine.

Add network traffic
^^^^^^^^^^^^^^^^^^^

This next section is Linux-specific.
However, for OS X and Windows you can achieve a similar effect with the utilities "Network Link Conditioner" (part of the xcode tool suite) and "Clumsy" (http://jagt.github.io/clumsy/index.html), respectively, but they will not be covered in this tutorial.

We are going to use the Linux network traffic control utility, ``tc`` (http://linux.die.net/man/8/tc).

.. code-block:: bash

   sudo tc qdisc add dev lo root netem loss 5%

This magical incantation will simulate 5% packet loss over the local loopback device.
If you use a higher resolution of the images (e.g. ``-x 640 -y 480``\ ) you might want to try a lower packet loss rate (e.g. ``1%``\ ).

Next we start the ``cam2image`` and ``showimage``\ , and we'll soon notice that both programs seem to have slowed down the rate at which images are transmitted.
This is caused by the behavior of the default QoS settings.
Enforcing reliability on a lossy channel means that the publisher (in this case, ``cam2image``\ ) will resend the network packets until it receives acknowledgement from the consumer (i.e. ``showimage``\ ).

Let's now try running both programs, but with more suitable settings.
First of all, we'll use the ``-r 0`` option to enable best effort communication.
The publisher will now just attempt to deliver the network packets, and don't expect acknowledgement from the consumer.
We see now that some of the frame on the ``showimage`` side were dropped, the frame numbers in the shell running ``showimage`` won't be consecutive anymore:


.. image:: https://raw.githubusercontent.com/ros2/demos/master/image_tools/doc/qos-best-effort.png
   :target: https://raw.githubusercontent.com/ros2/demos/master/image_tools/doc/qos-best-effort.png
   :alt: Best effort image transfer


When you're done, remember to delete the queueing discipline:

.. code-block:: bash

   sudo tc qdisc delete dev lo root netem loss 5%
