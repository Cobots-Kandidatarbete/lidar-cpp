from custom.srv import LidarService

import rclpy
from rclpy.node import Node


class ExampleServer(Node):

    def __init__(self):
        super().__init__('lidar_service_server')
        self.srv = self.create_service(LidarService, 'lidar_service', self.do_nothing)

    def do_nothing(self, request, response):
        self.get_logger().info('Doing nothing in the example service')
        return response


def main():
    rclpy.init()

    example_server = ExampleServer()
    rclpy.spin(example_server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()