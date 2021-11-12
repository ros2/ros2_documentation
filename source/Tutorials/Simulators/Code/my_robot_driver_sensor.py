import rclpy
from sensor_msgs.msg import Range

MAX_SPEED = 5
MAX_RANGE = 0.15

class MyRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
               
        self.__left_motor = self.__robot.getDevice('left wheel motor')
        self.__right_motor = self.__robot.getDevice('right wheel motor')

        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)

        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')
        self.__node.create_subscription(Range, 'l_sensor', self.__l_sensor_callback, 1)
        self.__node.create_subscription(Range, 'r_sensor', self.__r_sensor_callback, 1)

    def __l_sensor_callback(self, msg):
        self.__l_sensor_value = msg.range

    def __r_sensor_callback(self, msg):
        self.__r_sensor_value = msg.range

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        command_left = MAX_SPEED
        command_right = MAX_SPEED

        if self.__r_sensor_value < 0.9 * MAX_RANGE:
            command_left = MAX_SPEED
            command_right = -MAX_SPEED
        elif self.__l_sensor_value < 0.9 * MAX_RANGE:
            command_left = MAX_SPEED
            command_right = 0

        self.__left_motor.setVelocity(command_left)
        self.__right_motor.setVelocity(command_right)
