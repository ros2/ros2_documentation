import math
import rclpy
from geometry_msgs.msg import Twist

MAX_SPEED = 5

def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)

class MyRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        
        self.__left_motor = self.__robot.getDevice('left wheel motor')
        self.__right_motor = self.__robot.getDevice('right wheel motor')

        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)

        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        command_left = 0
        command_right = 0

        if self.__target_twist.linear.x != 0.0 or self.__target_twist.linear.y != 0.0:
            if self.__target_twist.linear.x <= 0.0:
                if self.__target_twist.linear.y > 0.0:
                    command_left = MAX_SPEED
                    command_right = -MAX_SPEED
                else:
                    command_left = -MAX_SPEED
                    command_right = MAX_SPEED
            else:
                vector_move_norm = math.sqrt(math.pow(self.__target_twist.linear.x, 2) + math.pow(self.__target_twist.linear.y, 2))
                ratio_turn = self.__target_twist.linear.y / vector_move_norm
                command_left = MAX_SPEED * clamp(1 + ratio_turn, 0, 1)
                command_right = MAX_SPEED * clamp(1 - ratio_turn, 0, 1)

        self.__left_motor.setVelocity(command_left)
        self.__right_motor.setVelocity(command_right)
