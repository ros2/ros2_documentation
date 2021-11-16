import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist


MAX_RANGE = 0.15


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')

        self.__publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        self.create_subscription(Range, 'l_sensor', self.__l_sensor_callback, 1)
        self.create_subscription(Range, 'r_sensor', self.__r_sensor_callback, 1)

    def __l_sensor_callback(self, msg):
        self.__l_sensor_value = msg.range

    def __r_sensor_callback(self, msg):
        self.__r_sensor_value = msg.range

        command_message = Twist()

        if self.__r_sensor_value < 0.9 * MAX_RANGE:
            command_message.linear.x = 0.0
            command_message.linear.y = 10.0

            self.__publisher.publish(command_message)

        elif self.__l_sensor_value < 0.9 * MAX_RANGE:
            command_message.linear.x = 10.0
            command_message.linear.y = 10.0

            self.__publisher.publish(command_message)


def main(args=None):
    rclpy.init(args=args)
    avoider = ObstacleAvoider()
    rclpy.spin(avoider)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    avoider.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
