.. redirect-from::

    Migration-Guide-Python

Python migration guide from ROS 1
=================================

Node Initialization
-------------------

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
--------------

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
--------------------

In ROS 1:

.. code-block:: python

   pub = rospy.Publisher('chatter', String)

In ROS 2:

.. code-block:: python

   pub = node.create_publisher(String, 'chatter')


Creating a Subscriber
---------------------

In ROS 1:

.. code-block:: python

   sub = rospy.Subscriber('chatter', String, callback)

In ROS 2:

.. code-block:: python

   sub = node.create_subscription(String, 'chatter', callback)


Creating a Service
------------------

In ROS 1:

.. code-block:: python

   srv = rospy.Service('add_two_ints', AddTwoInts, add_two_ints_callback)

In ROS 2:

.. code-block:: python

   srv = node.create_service(AddTwoInts, 'add_two_ints', add_two_ints_callback)


Creating a Service Client
-------------------------

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
