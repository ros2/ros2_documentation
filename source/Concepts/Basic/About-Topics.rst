Topics
======

.. contents:: Table of Contents
   :local:

Topics are one of the three primary styles of interfaces provided by ROS 2.
Topics should be used for continuous data streams, like sensor data, robot state, etc.

As stated earlier, ROS 2 is a strongly-typed, anonymous publish/subscribe system.
Let's break down that sentence and explain it a bit more.

Publish/Subscribe
-----------------

A publish/subscribe system is one in which there are producers of data (publishers) and consumers of data (subscribers).
The publishers and subscribers know how to contact each other through the concept of a "topic", which is a common name so that the entites can find each other.
For instance, when you create a publisher, you must also give it a string that is the name of the topic; the same goes for the subscriber.
Any publishers and subscribers that are on the same topic name can directly communicate with each other.
There may be zero or more publishers and zero or more subscribers on any particular topic.
When data is published to the topic by any of the publishers, all subscribers in the system will receive the data.
This system is also known as a "bus", since it somewhat resembles a device bus from electrical engineering.
This concept of a bus is part of what makes ROS 2 a powerful and flexible system.
Publishers and subscribers can come and go as needed, meaning that debugging and introspection are natural extensions to the system.
For instance, if you want to record data, you can use the ``ros2 bag record`` command.
Under the hood, ``ros2 bag record`` creates a new subscriber to whatever topic you tell it, without interrupting the flow of data to the other parts of the system.

Anonymous
---------

Another fact mentioned in the introduction is that ROS 2 is "anonymous".
This means that when a subscriber gets a piece of data, it doesn't generally know or care which publisher originally sent it (though it can find out if it wants).
The benefit to this architecture is that publishers and subscribers can be swapped out at will without affecting the rest of the system.

Strongly-typed
--------------

Finally, the introduction also mentioned that the publish/subscribe system is "strongly-typed".
That has two meanings in this context:

1. The types of each field in a ROS message are typed, and that type is enforced at various levels.
   For instance, if the ROS message contains:

   .. code::

      uint32 field1
      string field2

   Then the code will ensure that ``field`` is always an unsigned integer and that ``field2`` is always a string.

2. The semantics of each field are well-defined.  There is no automated mechanism to ensure this, but all of the core ROS types have strong semantics associated with them.  For instance, the IMU message contains a 3-dimensional vector for the measured angular velocity, and each of the dimensions is specified to be in radians/second.  Other interpretations should not be placed into the message.
