.. redirect-from::

    Design-Guide


Design guide: common patterns in ROS 2
======================================

Composable nodes as shared libraries
------------------------------------

**Context**

You want to export a composable node as a shared library from a package and use that node in another package that does link-time composition.

**Solution**


* Add code to the CMake file which imports the actual targets in downstream packages

  * Install the generated file
  * Export the generated file

**Example**

`ROS Discourse - Ament best practice for sharing libraries <https://discourse.ros.org/t/ament-best-practice-for-sharing-libraries/3602>`__

Fast RTPS large data transfer
-----------------------------

**Context**

You want to transfer large data via Fast RTPS.

**Problem**

DDS/RTPS uses UDP with a maximum message size of 64k

**Solution**

Configure the middleware so that it fragments large data into messages

**Implementation**

Use Asynchronous publication mode:

.. code-block:: bash

   <publishMode>
     <kind>ASYNCHRONOUS</kind>
   </publishMode>

`ROS2 Fine Tuning <https://roscon.ros.org/2017/presentations/ROSCon%202017%20ROS2%20Fine%20Tuning.pdf>`__

Fast RTPS Best Effort Video Streaming
-------------------------------------

**Context**

You want to transfer video streams and provide up-to-date data. It is OK to lose some of the images.

**Problem**

Acknowledged data transmission mechanisms (also known as "reliable" delivery) ensure every packet is delivered, which can cause the video stream to fall behind.

**Solution**

Use "best effort" communication (instead of the usual acknowledgement based
mechanism) and prioritize the last frame.

**Implementation**

* Configure "best effort" reliability mechanism
* Configure Quality of service history to keep last frame

.. code-block:: bash

   <reliability>
     <kind>BEST_EFFORT</kind>
   </reliability>

   <historyQos>
     <kind>KEEP_LAST</kind>
     <depth>1</depth>
   </historyQos>

`ROS2 Fine Tuning <https://roscon.ros.org/2017/presentations/ROSCon%202017%20ROS2%20Fine%20Tuning.pdf>`__

Fast RTPS Reliable Video Streaming
----------------------------------

**Context**

You want to transfer video streams in unreliable network settings.

**Solution**

Use a reliable communication mechanism. Use fast response by writer and reader.

**Implementation**

* Configure "reliable" reliability mechanism
* Configure NACK response delay and suppression duration of writer to 0
* Configure heartbeat response delay of reader to 0

.. code-block:: bash

   <reliability>
     <kind>RELIABLE</kind>
   </reliability>

   # writer
   <times>
     <nackResponseDelay>
       <durationbyname>ZERO</durationbyname>
     </nackResponseDelay>
     <nackSupressionDuration>
       <durationbyname>ZERO</durationbyname>
     </nackSupressionDuration>
   </times>

   # reader
   <times>
     <heartbeatResponseDelay>
       <durationbyname>ZERO</durationbyname>
     </heartbeatResponseDelay>
   </times>

`ROS2 Fine Tuning <https://roscon.ros.org/2017/presentations/ROSCon%202017%20ROS2%20Fine%20Tuning.pdf>`__
