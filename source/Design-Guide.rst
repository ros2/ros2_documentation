
Composable nodes as shared libraries
====================================

**Context**

You want to export composable nodes as a shared libraries from some packages and using those in another package that does link-time composition.

**Solution**


* add code to the CMake file which imports the actual targets in downstream packages

  * install the generated file
  * export the generated file

**Example**

`ROS Discourse - Ament best practice for sharing libraries <https://discourse.ros.org/t/ament-best-practice-for-sharing-libraries/3602>`__

FastRTPS large data transfer
============================

**Context**

You want to transfer large data via FastRTPS.

**Problem**

DDS/RTPS uses UDP with a maximum message size of 64k

**Solution**

configure the middleware that it fragements large data into messages

**Implementation**

use Asynchronous publication mode:

.. code-block:: bash

   <publishMode>
     <kind>ASYNCHRONOUS</kind>
   </publishMode>

`ROS2 Fine Tuning <https://roscon.ros.org/2017/presentations/ROSCon%202017%20ROS2%20Fine%20Tuning.pdf>`__

FastRTPS Best Effort Video Streaming
====================================

**Context**

You want to transfer video streams and provide up to date data. It is ok to loose
some packages.

**Problem**

Acknowledged data transmission mechanisms prevent from being able to provide
up to date packages.

**Solution**

Use "best effort" communication (instead of the usual acknowledgement based
mechanism) and prioritize the last frame.

**Implementation**


* configure "best effort" reliability mechanism
* configure Quality of service history to keep last frame

.. code-block:: bash

   <reliability>
     <kind>BEST_EFFORT</kind>
   </reliability>

   <historyQos>
     <kind>KEEP_LAST</kind>
     <depth>1</depth>
   </historyQos>

`ROS2 Fine Tuning <https://roscon.ros.org/2017/presentations/ROSCon%202017%20ROS2%20Fine%20Tuning.pdf>`__

FastRTPS Reliable Video Streaming
=================================

**Context**

You want to transfer video streams in unreliable network settings.

**Solution**

Use a reliable communication mechanism. Use fast response by writer and reader.

**Implementation**


* configure "reliable" reliability mechanism
* configure NACK reponse delay and suppression duration of writer to 0
* configure heartbeat response delay of reader to 0

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
