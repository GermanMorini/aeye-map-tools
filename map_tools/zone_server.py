#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class ZoneServer(Node):
    def __init__(self) -> None:
        super().__init__("zone_server")
        self.get_logger().info("map_tools zone_server started")


def main() -> None:
    rclpy.init()
    node = ZoneServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
