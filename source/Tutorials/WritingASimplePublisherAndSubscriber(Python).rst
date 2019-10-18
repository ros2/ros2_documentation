.. _PyPubSub:

Writing a simple publisher and subscriber (Python)
==================================================

**Goal:** Create and run a publisher and subscriber node using Python

**Tutorial level:** Beginner

**Time:** 20 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

Nodes are executable processes that communicate over the ROS graph.
In this tutorial, you will create nodes that pass information in the form of string messages to each other over a :ref:`topic <ROS2Topics>`.
The example used here is a simple “talker” and “listener” system;
one node publishes data and the other subscribes to the topic so it can receive that data.

.. link nodes and topics tutorials


Prerequisites
-------------

.. In previous tutorials, you learned how to :ref:`create a workspace <>` and :ref:`create a package <>`.

Tasks
-----

1 Create a package
^^^^^^^^^^^^^^^^^^

Open a new terminal and :ref:`source your ROS 2 installation <ConfigROS2>` so that ``ros2`` commands will work.

Navigate into the ``dev_ws`` directory created in a previous tutorial.

.. link creating a workspace tutorial

Recall that packages should be created in the ``src`` directory, not the root of the workspace.
So, navigate into ``dev_ws/src``, and run the package creation command:

.. code-block:: bash

  ros2 pkg create --build-type ament_python py_pubsub

Your terminal will return a message verifying the creation of your package ``py_pubsub`` and all its necessary files and folders.

Navigate into ``dev_ws/py_pubsub/py_pubsub``.
Recall that this is the directory in any ROS Python package (the nested directory with the same name as the package) where executables belong, adjacent to the ``__init__.py`` file.


2 Write the publisher node
^^^^^^^^^^^^^^^^^^^^^^^^^^

Download the example talker code by entering the following command:

.. code-block:: bash

    wget https://raw.githubusercontent.com/ros2/examples/master/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py

Now there will be a new file named ``publisher_member_function.py`` adjacent to ``__init__.py``.

Open the file using your preferred text editor.

.. code-block:: python
  :linenos:

  # Copyright 2016 Open Source Robotics Foundation, Inc.
  #
  # Licensed under the Apache License, Version 2.0 (the "License");
  # you may not use this file except in compliance with the License.
  # You may obtain a copy of the License at
  #
  #     http://www.apache.org/licenses/LICENSE-2.0
  #
  # Unless required by applicable law or agreed to in writing, software
  # distributed under the License is distributed on an "AS IS" BASIS,
  # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  # See the License for the specific language governing permissions and
  # limitations under the License.

  import rclpy
  from rclpy.node import Node

  from std_msgs.msg import String


  class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


    def main(args=None):
      rclpy.init(args=args)

      minimal_publisher = MinimalPublisher()

      rclpy.spin(minimal_publisher)

      # Destroy the node explicitly
      # (optional - otherwise it will be done automatically
      # when the garbage collector destroys the node object)
      minimal_publisher.destroy_node()
      rclpy.shutdown()


      if __name__ == '__main__':
        main()


2.1 Examine the code
~~~~~~~~~~~~~~~~~~~~

The first lines of code after the comments, lines 15-16, import ``rclpy`` so its ``Node`` class can be used.

.. code-block:: python

    import rclpy
    from rclpy.node import Node

Line 18 imports the existing string message type that the node uses to structure the data it passes to the topic.

.. code-block:: python

    from std_msgs.msg import String

These lines represent the node’s dependencies.
Recall that dependencies have to be added to ``package.xml``, which you’ll do in the next section.

Line 21 creates the ``MinimalPublisher`` class, which inherits from (or is a subclass of) ``Node``.

.. code-block:: python

    class MinimalPublisher(Node):

Lines 23-28 define the class’s constructor.
Line 24 calls the ``Node`` class’s constructor and gives it your node name, in this case ``minimal_publisher``.

.. code-block:: python

    def __init__(self):
            super().__init__('minimal_publisher')
            self.publisher_ = self.create_publisher(String, 'topic', 10)
            timer_period = 0.5  # seconds
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.i = 0

Line 25 declares that the node publishes messages of type ``String`` (from the imported ``std_msgs.msg`` class, over a topic named ``topic``, and that the “queue size" is 10.
Queue size is a required QoS (quality of service) setting that limits the amount of queued messages if a subscriber is not receiving them fast enough.

Lines 26-27 create a timer with a callback to execute every 0.5 seconds.
Line 28 is a counter used in the callback, which is defined on lines 30-35.

.. code-block:: python

    def timer_callback(self):
            msg = String()
            msg.data = 'Hello World: %d' % self.i
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
            self.i += 1

``timer_callback`` creates a message along with the counter value (line 32) that is published to the console (line 34).

.. link rqt_console tutorial for explanation of logger levels.

Lines 38-49 define the main function.

.. code-block:: python

  def main(args=None):
        rclpy.init(args=args)

        minimal_publisher = MinimalPublisher()

        rclpy.spin(minimal_publisher)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        minimal_publisher.destroy_node()
        rclpy.shutdown()

Line 39 initializes the ``rclpy`` library, line 41 creates the publisher, and line 43 “spins” the publisher so that the timer and callbacks begin.

2.2 Add dependencies
~~~~~~~~~~~~~~~~~~~~

Navigate one level back to the ``dev_ws/src/py_pubsub`` directory, where the ``setup.py``, ``setup.cfg``, and ``package.xml`` files have been created for you.

Open ``package.xml`` with your text editor.

As mentioned in the previous tutorial, make sure to fill in the ``<description>``, ``<maintainer>`` and ``<license>`` tags on lines 6-8:

.. code-block:: xml

    <description>Examples of minimal publisher/subscriber using rclpy</description>
    <maintainer email="you@email.com">Your Name</maintainer>
    <license>Apache License 2.0</license>

Add a new line after line 10 and paste the following dependencies corresponding to your node’s import statements:

.. code-block:: xml

    <exec_depend>rclpy</exec_depend>
    <exec_depend>std_msgs</exec_depend>

Make sure to save the file.

2.3 Add an entry point
~~~~~~~~~~~~~~~~~~~~~~

Open the ``setup.py`` file.
Again, match the ``maintainer``, ``maintainer_email``, ``description`` and ``license`` fields to your ``package.xml``:

.. code-block:: python

    maintainer='YourName',
    maintainer_email='you@email.com',
    description='Examples of minimal publisher/subscriber using rclpy',
    license='Apache License 2.0',

Now focus on line 21, where you’ll see the field ``entry_points``.
Add the following line within the ``console_scripts`` brackets:

.. code-block:: python

    entry_points={
            'console_scripts': [
                    'talker = py_pubsub.publisher_member_function:main',
            ],
    },

Don’t forget to save.

2.4 Check setup.cfg
~~~~~~~~~~~~~~~~~~~

The contents of the ``setup.cfg`` file should be correctly populated automatically, like so:

.. code-block::

    [develop]
    script-dir=$base/lib/py_pubsub
    [install]
    install-scripts=$base/lib/py_pubsub

This is simply telling the setup tools to put your executables in ``lib``, because ``ros2 run`` will look for them there.

You could build your package now, source the local setup files, and run it, but let’s create the subscriber node first so you can see the full system at work.

3 Write the subscriber node
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Return to ``dev_ws/src/py_pubsub/py_pubsub`` to create the next node.
Enter the following code in your terminal:

.. code-block:: bash

    wget https://raw.githubusercontent.com/ros2/examples/master/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py

Entering ``ls`` in the console will now return:

.. code-block:: bash

    __init__.py  publisher_member_function.py  subscriber_member_function.py

3.1 Examine the code
~~~~~~~~~~~~~~~~~~~~

Open the ``subscriber_member_function.py`` with your text editor.

.. code-block:: python
  :linenos:

  # Copyright 2016 Open Source Robotics Foundation, Inc.
  #
  # Licensed under the Apache License, Version 2.0 (the "License");
  # you may not use this file except in compliance with the License.
  # You may obtain a copy of the License at
  #
  #     http://www.apache.org/licenses/LICENSE-2.0
  #
  # Unless required by applicable law or agreed to in writing, software
  # distributed under the License is distributed on an "AS IS" BASIS,
  # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  # See the License for the specific language governing permissions and
  # limitations under the License.

  import rclpy
  from rclpy.node import Node

  from std_msgs.msg import String


  class MinimalSubscriber(Node):

    def __init__(self):
      super().__init__('minimal_subscriber')
      self.subscription = self.create_subscription(
          String,
          'topic',
          self.listener_callback,
          10)
      self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
      self.get_logger().info('I heard: "%s"' % msg.data)


    def main(args=None):
      rclpy.init(args=args)

      minimal_subscriber = MinimalSubscriber()

      rclpy.spin(minimal_subscriber)

      # Destroy the node explicitly
      # (optional - otherwise it will be done automatically
      # when the garbage collector destroys the node object)
      minimal_subscriber.destroy_node()
      rclpy.shutdown()


      if __name__ == '__main__':
        main()


The subscriber node’s code is nearly identical to the publisher’s.
Line 25 is the same, except now it’s creating a subscriber, with the same arguments as the publisher.
Recall from the :ref:`topics tutorial <ROS2Topics>` that the topic name and message type used by the publisher and subscriber must match to allow them to communicate.

.. code-block:: python

    self.subscription = self.create_subscription(
          String,
          'topic',
          self.listener_callback,
          10)

The subscriber’s constructor and callback don’t include any timer definition.

On lines 32-33 the callback simply prints an info message to the console declaring that it received a message.
It also adds the message data to the info message.
Recall that the publisher defines ``msg.data = 'Hello World: %d' % self.i``

.. code-block:: python

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

The ``main`` definition is almost exactly the same, except now line 39 creates a ``minimal_subscriber`` node, and line 41 spins the subscriber.

.. code-block:: python

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

Since this node has the same dependencies as the publisher, there’s nothing new to add to ``package.xml``.
The ``setup.cfg`` file can also remain untouched.


3.2 Add an entry point
~~~~~~~~~~~~~~~~~~~~~~

Reopen ``setup.py`` and add the entry point for the subscriber node below the publisher’s entry point.
The ``entry_points`` field should now look like this:

.. code-block:: python

    entry_points={
            'console_scripts': [
                    'talker = py_pubsub.publisher_member_function:main',
                    'listener = py_pubsub.subscriber_member_function:main',
            ],
    },

Make sure to save the file, and then your pub/sub system should be ready for use.

4 Build and run
^^^^^^^^^^^^^^^

Navigate back to the root of your workspace, ``dev_ws``, and build your new package:

.. code-block:: bash

    colcon build --packages-select py_pubsub

Open a new terminal, navigate to ``dev_ws``, and source the setup files:

.. code-block:: bash

    . install/setup.bash

Now run the talker node:

.. code-block:: bash

     ros2 run py_pubsub talker

The terminal should start publishing info messages every 0.5 seconds, like so:

.. code-block:: bash

    [INFO] [minimal_publisher]: Publishing: "Hello World: 0"
    [INFO] [minimal_publisher]: Publishing: "Hello World: 1"
    [INFO] [minimal_publisher]: Publishing: "Hello World: 2"
    [INFO] [minimal_publisher]: Publishing: "Hello World: 3"
    [INFO] [minimal_publisher]: Publishing: "Hello World: 4"
    ...

Open another terminal, source the setup files from inside ``dev_ws`` again, and then start the listener node:

.. code-block:: bash

     ros2 run py_pubsub listener

The listener will start printing messages to the console, starting at whatever message count the publisher is on at that time, like so:

.. code-block:: bash

  [INFO] [minimal_subscriber]: I heard: "Hello World: 10"
  [INFO] [minimal_subscriber]: I heard: "Hello World: 11"
  [INFO] [minimal_subscriber]: I heard: "Hello World: 12"
  [INFO] [minimal_subscriber]: I heard: "Hello World: 13"
  [INFO] [minimal_subscriber]: I heard: "Hello World: 14"

Enter ``Ctrl+C`` in each terminal to stop the nodes from spinning.


Summary
-------

You created two nodes to publish and subscribe to data over a topic.
Before being able to run them, you needed to add their dependencies and entry points to the package setup files.

.. todo: "Next steps section" once all tutorials are done
