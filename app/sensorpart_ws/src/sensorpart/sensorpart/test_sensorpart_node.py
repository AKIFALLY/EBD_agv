import rclpy
from rclpy.node import Node
from sensorpart.sensorpart import SensorPart
import threading


class TestSensorPartNode(Node):
    def __init__(self):
        super().__init__('sensorpart_node')
        self.get_logger().info("TestSensorPartNode initialized.")

        # Initialize the TCP client
        self.tcp_client = SensorPart()
        self.tcp_client.start()

        # Create a timer to periodically log data
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # Log the current state of position data and OCR result
        self.get_logger().info(
            f"Position Data: {self.tcp_client.position_data}")
        self.get_logger().info(f"OCR Result: {self.tcp_client.ocr_result}")

    def destroy_node(self):
        # Stop the TCP client before shutting down the node
        self.tcp_client.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TestSensorPartNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down SensorPartNode...")
    finally:
        node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
