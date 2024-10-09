from service_interface.srv import ServiceInterface

import rclpy
from rclpy.node import Node


class ExampleService(Node):
    def __init__(self):
        super().__init__("example_service")
        self.srv = self.create_service(
            ServiceInterface, "example_service", self.callback
        )

    def callback(self, request, response):
        response.success = True
        self.get_logger().info("ExampleService called and returned true")

        return response


def main(args=None):
    rclpy.init(args=args)

    example_service = ExampleService()

    rclpy.spin(example_service)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
