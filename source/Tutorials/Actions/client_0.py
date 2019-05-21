import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_tutorials.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()


if __name__ == '__main__':
    main()
