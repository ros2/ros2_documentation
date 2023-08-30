.. redirect-from::

   Migration-Guide-Python
   The-ROS2-Project/Contributing/Migration-Guide-Python

Migrating Python Packages
=========================

.. contents:: Table of Contents
   :depth: 2
   :local:

Build tool
----------

Instead of using ``catkin_make``, ``catkin_make_isolated`` or ``catkin build`` ROS 2 uses the command line tool `colcon <https://design.ros2.org/articles/build_tool.html>`__ to build and install a set of packages.
See the :doc:`beginner tutorial <../../Tutorials/Beginner-Client-Libraries/Colcon-Tutorial>` to get started with ``colcon``.

Build system
------------

For pure Python packages, ROS 2 uses the standard ``setup.py`` installation mechanism familiar to Python developers.

Update the files to use *setup.py*
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If the ROS 1 package uses CMake only to invoke the ``setup.py`` file and does not contain anything beside Python code (e.g. no messages, services, etc.) it should be converted into a pure Python package in ROS 2:

*
  Update or add the build type in the ``package.xml`` file:

  .. code-block:: xml

     <export>
       <build_type>ament_python</build_type>
     </export>

*
  Remove the ``CMakeLists.txt`` file

*
  Update the ``setup.py`` file to be a standard Python setup script

ROS 2 supports Python 3 only.
While each package can choose to also support Python 2 it must invoke executables with Python 3 if it uses any API provided by other ROS 2 packages.

Update source code
------------------

Node Initialization
^^^^^^^^^^^^^^^^^^^

In ROS 1:

.. code-block:: python

   rospy.init_node('asdf')

   rospy.loginfo('Created node')

In ROS 2:

.. code-block:: python

   rclpy.init(args=sys.argv)
   node = rclpy.create_node('asdf')

   node.get_logger().info('Created node')

ROS Parameters
^^^^^^^^^^^^^^

In ROS 1:

.. code-block:: python

   port = rospy.get_param('port', '/dev/ttyUSB0')
   assert isinstance(port, str), 'port parameter must be a str'

   baudrate = rospy.get_param('baudrate', 115200)
   assert isinstance(baudrate, int), 'baudrate parameter must be an integer'

  rospy.logwarn('port: ' + port)

In ROS 2:

.. code-block:: python

   port = node.declare_parameter('port', '/dev/ttyUSB0').value
   assert isinstance(port, str), 'port parameter must be a str'

   baudrate = node.declare_parameter('baudrate', 115200).value
   assert isinstance(baudrate, int), 'baudrate parameter must be an integer'

   node.get_logger().warn('port: ' + port)

Creating a Publisher
^^^^^^^^^^^^^^^^^^^^

In ROS 1:

.. code-block:: python

   pub = rospy.Publisher('chatter', String)
   # or
   pub = rospy.Publisher('chatter', String, queue_size=10)

In ROS 2:

.. code-block:: python

   pub = node.create_publisher(String, 'chatter', rclpy.qos.QoSProfile())
   # or
   pub = node.create_publisher(String, 'chatter', 10)

Creating a Subscriber
^^^^^^^^^^^^^^^^^^^^^

In ROS 1:

.. code-block:: python

   sub = rospy.Subscriber('chatter', String, callback)
   # or
   sub = rospy.Subscriber('chatter', String, callback, queue_size=10)

In ROS 2:

.. code-block:: python

   sub = node.create_subscription(String, 'chatter', callback, rclpy.qos.QoSProfile())
   # or
   sub = node.create_subscription(String, 'chatter', callback, 10)

Creating a Service
^^^^^^^^^^^^^^^^^^

In ROS 1:

.. code-block:: python

   srv = rospy.Service('add_two_ints', AddTwoInts, add_two_ints_callback)

In ROS 2:

.. code-block:: python

   srv = node.create_service(AddTwoInts, 'add_two_ints', add_two_ints_callback)

Creating a Service Client
^^^^^^^^^^^^^^^^^^^^^^^^^

In ROS 1:

.. code-block:: python

   rospy.wait_for_service('add_two_ints')
   add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
   resp = add_two_ints(req)

In ROS 2:

.. code-block:: python

   add_two_ints = node.create_client(AddTwoInts, 'add_two_ints')
   while not add_two_ints.wait_for_service(timeout_sec=1.0):
       node.get_logger().info('service not available, waiting again...')
   resp = add_two_ints.call_async(req)
   rclpy.spin_until_future_complete(node, resp)
