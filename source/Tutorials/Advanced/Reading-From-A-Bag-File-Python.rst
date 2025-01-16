Reading from a bag file (Python)
================================

**Goal:** Read data from a bag file to your own Python node.

**Tutorial level:** Advanced

**Time:** 10 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

``rosbag2`` doesn't just provide the ``ros2 bag`` command line tool.
It also provides a Python API for reading from and writing to a bag from your own source code.
This allows you to read the contents from a bag without having to play the bag, which can sometimes be useful.

Prerequisites
-------------

You should have the ``rosbag2`` packages installed as part of your regular ROS 2 setup.

If you need to install ROS 2, see the :doc:`Installation instructions <../../Installation>`.

You should have already completed the :doc:`basic ROS 2 bag tutorial <../Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data>`, and we will be using the ``subset`` bag you created there.

Tasks
-----

1 Create a Package
^^^^^^^^^^^^^^^^^^

Open a new terminal and :doc:`source your ROS 2 installation <../Beginner-CLI-Tools/Configuring-ROS2-Environment>` so that ``ros2`` commands will work.

In a new or existing :ref:`workspace <new-directory>`, navigate to the ``src`` directory and create
a new package:

.. code-block:: console

  ros2 pkg create --build-type ament_python --license Apache-2.0 bag_reader_node_py --dependencies rclpy rosbag2_py std_msgs

Your terminal will return a message verifying the creation of your package ``bag_reader_node_py`` and all its necessary files and folders.
The ``--dependencies`` argument will automatically add the necessary dependency lines to ``package.xml``.
In this case, the package will use the ``rosbag2_py`` package as well as the ``rclpy`` package.
A dependency on the ``std_msgs`` package is also required for message definitions.

1.1 Update ``package.xml`` and ``setup.py``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Because you used the ``--dependencies`` option during package creation, you don't have to manually add dependencies to ``package.xml``.
As always, though, make sure to add the description, maintainer email and name, and license information to ``package.xml``.

.. code-block:: xml

  <description>Python bag reading tutorial</description>
  <maintainer email="you@email.com">Your Name</maintainer>
  <license>Apache-2.0</license>

Also be sure to add this information to the ``setup.py`` file as well.

.. code-block:: Python

   maintainer='Your Name',
   maintainer_email='you@email.com',
   description='Python bag reading tutorial',
   license='Apache-2.0',

2 Write the Python Reader
^^^^^^^^^^^^^^^^^^^^^^^^^

Inside the ``ros2_ws/src/bag_reader_node_py/bag_reader_node_py`` directory, create a new file called ``simple_bag_reader.py`` and paste the following code into it.

.. code-block:: Python

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    import rosbag2_py
    from std_msgs.msg import String


    class SimpleBagReader(Node):

        def __init__(self):
            super().__init__('simple_bag_reader')
            self.reader = rosbag2_py.SequentialReader()
            storage_options = rosbag2_py._storage.StorageOptions(
                uri='my_bag',
                storage_id='mcap')
            converter_options = rosbag2_py._storage.ConverterOptions('', '')
            self.reader.open(storage_options, converter_options)

            self.publisher = self.create_publisher(String, 'chatter', 10)
            self.timer = self.create_timer(0.1, self.timer_callback)

        def timer_callback(self):
            while self.reader.has_next():
                msg = self.reader.read_next()
                if msg[0] != 'chatter':
                    continue
                self.publisher.publish(msg[1])
                self.get_logger().info('Publish serialized data to ' + msg[0])
                break


    def main(args=None):
        try:
            with rclpy.init(args=args):
                sbr = SimpleBagReader()
                rclpy.spin(sbr)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass


    if __name__ == '__main__':
        main()

2.1 Examine the code
~~~~~~~~~~~~~~~~~~~~

The ``import`` statements at the top are the package dependencies.
Note the importation of the ``rosbag2_py`` package for the functions and structures necessary to work with bag files.

In the class constructor, we begin by creating the bag reader object that we will use to read from the bag.
We are creating a ``SequentialReader``, which reads messages from the bag in the order they are stored.
Other readers with different behaviors may be available in `rosbag2_py reader <https://github.com/ros2/rosbag2/tree/{REPOS_FILE_BRANCH}/rosbag2_py/rosbag2_py/_reader.pyi>`__.

.. code-block:: Python

   self.reader = rosbag2_py.SequentialReader()

Now that we have a bag reader object, we can open the bag using it.
We specify the URI of the bag and the format (``mcap``), leaving other options at their defaults.
The default conversion options are used, which will perform no conversion and store the messages in the serialization format they are received in.

.. code-block:: Python

   storage_options = rosbag2_py._storage.StorageOptions(
       uri='my_bag',
       storage_id='mcap')
   converter_options = rosbag2_py._storage.ConverterOptions('', '')
   self.reader.open(storage_options, converter_options)

Next, we create a publisher and a timer to publish the data that reader object reads from the bag file.

.. code-block:: Python

   self.publisher = self.create_publisher(String, 'chatter', 10)
   self.timer = self.create_timer(0.1, self.timer_callback)

The timer callback publishes all messages from the bag file only to the ``chatter`` topic as long as it can read out the data.

.. code-block:: Python

   def timer_callback(self):
       while self.reader.has_next():
           msg = self.reader.read_next()
           if msg[0] != 'chatter':
               continue
           self.publisher.publish(msg[1])
           self.get_logger().info('Publish serialized data to ' + msg[0])

Finally, it finishes with the ``main`` function used to create an instance of the node and start ROS processing it.

.. code-block:: Python

   def main(args=None):
       try:
           with rclpy.init(args=args):
               sbr = SimpleBagReader()
               rclpy.spin(sbr)
       except (KeyboardInterrupt, ExternalShutdownException):
           pass

2.2 Add executable
~~~~~~~~~~~~~~~~~~

Open the ``setup.py`` file in the ``bag_reader_node_py`` package and add an entry point for your node.

.. code-block:: Python

   entry_points={
       'console_scripts': [
           'simple_bag_reader = bag_reader_node_py.simple_bag_reader:main',
       ],
   },

3 Build and run
^^^^^^^^^^^^^^^

Navigate back to the root of your workspace, ``ros2_ws``, and build your new package.

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      colcon build --packages-select bag_reader_node_py

  .. group-tab:: macOS

    .. code-block:: console

      colcon build --packages-select bag_reader_node_py

  .. group-tab:: Windows

    .. code-block:: console

      colcon build --merge-install --packages-select bag_reader_node_py

Open a new terminal, navigate to ``ros2_ws``, and source the setup files.

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      source install/setup.bash

  .. group-tab:: macOS

    .. code-block:: console

      source install/setup.bash

  .. group-tab:: Windows

    .. code-block:: console

      call install/setup.bat

Now run the node:

.. code-block:: console

   ros2 run bag_reader_node_py simple_bag_reader

This will start publishing data on the ``chatter`` topic from the ``my_bag`` bag file.
If the ``my_bag`` directory does not exists, ``simple_bag_reader`` will return failure since it can not open the bag file.

Open a second terminal and run the ``listener`` example node.

.. code-block:: console

   ros2 run demo_nodes_py listener

This will start receiving data on the ``chatter`` topic, published by ``simple_bag_reader`` from the bag file.

Summary
-------

You created a Python node that reads data from a bag.
You tested reading a bag using the node, and publishing the data by playing back the bag.
This approach can be used to read and publish the data with additional data from the bag file, for example modifying the original data stored in the bag file.
