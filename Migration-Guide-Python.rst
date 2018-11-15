Python Migration guide from ROS 1
=================================


+----------------------------------------------------------------------------+----------------------------------------------------------------------------------+
| rospy                                                                      | rclpy                                                                            |
+----------------------------------------------------------------------------+----------------------------------------------------------------------------------+
| ``rospy.init_node('asdf')``                                                | ``rclpy.init(args=sys.argv)``                                                    |
|                                                                            | ``node = rclpy.create_node('asdf')``                                             |
+----------------------------------------------------------------------------+----------------------------------------------------------------------------------+
| ``pub = rospy.Publisher('chatter', String)``                               | ``pub = node.create_publisher(String, 'chatter')``                               |
+----------------------------------------------------------------------------+----------------------------------------------------------------------------------+
| ``sub = rospy.Subscriber('chatter', String, callback)``                    | ``sub = node.create_subscription(String, 'chatter', callback)``                  |
+----------------------------------------------------------------------------+----------------------------------------------------------------------------------+
| ``srv = rospy.Service('add_two_ints', AddTwoInts, add_two_ints_callback)`` | ``srv = node.create_service(AddTwoInts, 'add_two_ints', add_two_ints_callback)`` |
+----------------------------------------------------------------------------+----------------------------------------------------------------------------------+
| ``rospy.wait_for_service('add_two_ints')``                                 | ``cli = node.create_client(AddTwoInts, 'add_two_ints')``                         |
| ``add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)``          | ``while not cli.wait_for_service(timeout_sec=1.0):``                             |
| ``resp1 = add_two_ints(req)``                                              | ``    node.get_logger().info('service not available, waiting again...')``        |
|                                                                            | ``future = cli.call_async(req)``                                                 |
|                                                                            | ``rclpy.spin_until_future_complete(node, future)``                               |
+----------------------------------------------------------------------------+----------------------------------------------------------------------------------+