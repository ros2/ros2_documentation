.. redirect-from::

    Quality-of-Service

Use quality-of-service settings to handle lossy networks
========================================================

.. contents:: Table of Contents
   :depth: 2
   :local:

Background
----------

Please read the documentationg page `about QoS settings <../Concepts/About-Quality-of-Service-Settings>` for background information on available support in ROS 2.

In this demo, we will spawn a node that publishes a camera image and another that subscribes to the image and shows it on the screen.
We will then simulate a lossy network connection between them and show how different quality of service settings handle the bad link.


Prerequisites
-------------
This tutorial assumes you have a `working ROS 2 installation <../Installation>` and OpenCV.
See the `OpenCV documentation <http://docs.opencv.org/doc/tutorials/introduction/table_of_content_introduction/table_of_content_introduction.html#table-of-content-introduction>`__ for its installation instructions.
You will also need the ROS package ``image_tools``.

.. tabs::

   .. group-tab:: Linux Binaries

      .. code-block:: bash

        sudo apt-get install ros-rolling-image-tools

   .. group-tab:: From Source

      .. code-block:: bash

        # Clone and build the demos repo using the branch that matches your installation
        git clone https://github.com/ros2/demos.git -b master


Run the demo
------------

Before running the demo, make sure you have a working webcam connected to your computer.

Once you've installed ROS 2, source your setup file:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

       . <path to ROS 2 install space>/setup.bash

  .. group-tab:: macOS

    .. code-block:: bash

       . <path to ROS 2 install space>/setup.bash

  .. group-tab:: Windows

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

This will publish an image from your webcam.
In case you don't have a camera attached to your computer, there is a commandline option which publishes predefined images.


.. code-block:: bash

   ros2 run image_tools cam2image --ros-args -p burger_mode:=True


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

.. note::

   macOS users: If these examples do not work or you receive an error like ``ddsi_conn_write failed -1`` then you'll need to increase your system wide UDP packet size:

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

   ros2 run image_tools showimage -h



Add network traffic
^^^^^^^^^^^^^^^^^^^

.. warning::

  This section of the demo won't work on RTI's Connext DDS.
  When running multiple nodes in the same host, the RTI Connext implementation uses shared memory along with the loopback interface.
  Degrading the loopback interface throughput won't affect shared memory, thus traffic between the two nodes won't be affected.
  See RMW implementation `code <https://github.com/ros2/rmw_connext/blob/e71347b944a8cd92acfb89b129b2b10f2575f273/rmw_connext_shared_cpp/src/node.cpp#L96>`__ and `RTI Connext Documentation <https://community.rti.com/static/documentation/connext-dds/5.3.1/doc/api/connext_dds/api_dotnet/group__UDPv4Transport__Property__t__ignore__loopback__interface.html>`__  for further reference.

.. note::

   This next section is Linux-specific.

   However, for macOS and Windows you can achieve a similar effect with the utilities "Network Link Conditioner" (part of the xcode tool suite) and "Clumsy" (http://jagt.github.io/clumsy/index.html), respectively, but they will not be covered in this tutorial.

We are going to use the Linux network traffic control utility, ``tc`` (http://linux.die.net/man/8/tc).

.. code-block:: bash

   sudo tc qdisc add dev lo root netem loss 5%

This magical incantation will simulate 5% packet loss over the local loopback device.
If you use a higher resolution of the images (e.g. ``--ros-args -p width:=640 -p height:=480``) you might want to try a lower packet loss rate (e.g. ``1%``).

Next we start the ``cam2image`` and ``showimage``, and we'll soon notice that both programs seem to have slowed down the rate at which images are transmitted.
This is caused by the behavior of the default QoS settings.
Enforcing reliability on a lossy channel means that the publisher (in this case, ``cam2image``) will resend the network packets until it receives acknowledgement from the consumer (i.e. ``showimage``).

Let's now try running both programs, but with more suitable settings.
First of all, we'll use the ``-p reliability:=best_effort`` option to enable best effort communication.
The publisher will now just attempt to deliver the network packets, and don't expect acknowledgement from the consumer.
We see now that some of the frame on the ``showimage`` side were dropped, the frame numbers in the shell running ``showimage`` won't be consecutive anymore:


.. image:: https://raw.githubusercontent.com/ros2/demos/master/image_tools/doc/qos-best-effort.png
   :target: https://raw.githubusercontent.com/ros2/demos/master/image_tools/doc/qos-best-effort.png
   :alt: Best effort image transfer


When you're done, remember to delete the queueing discipline:

.. code-block:: bash

   sudo tc qdisc delete dev lo root netem loss 5%
