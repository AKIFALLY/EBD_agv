import rclpy
from rclpy.node import Node
from plc_proxy.plc_client import PlcClient


class PlcClientNode(Node):
    def __init__(self, node_name="plc_client", namespace=""):
        if not rclpy.ok():
            rclpy.init()

        super().__init__(node_name, namespace=namespace)
        self.client = PlcClient(self)

    def force_on(self, device_type, address):
        return self.client.force_on(device_type, address)

    def force_off(self, device_type, address):
        return self.client.force_off(device_type, address)

    def read_data(self, device_type, address):
        return self.client.read_data(device_type, address)

    def write_data(self, device_type, address, value):
        return self.client.write_data(device_type, address, value)

    def read_continuous_data(self, device_type, start_address, count):
        return self.client.read_continuous_data(device_type, start_address, count)

    def write_continuous_data(self, device_type, start_address, values):
        return self.client.write_continuous_data(device_type, start_address, values)

    def read_continuous_byte(self, device_type, start_address, count):
        return self.client.read_continuous_byte(device_type, start_address, count)

    def write_continuous_byte(self, device_type, start_address, values):
        return self.client.write_continuous_byte(device_type, start_address, values)

    # 非同步方法（與plclient相同）
    def async_force_on(self, device_type, address, callback):
        self.client.async_force_on(device_type, address, callback)

    def async_force_off(self, device_type, address, callback):
        self.client.async_force_off(device_type, address, callback)

    def async_read_data(self, device_type, address, callback):
        self.client.async_read_data(device_type, address, callback)

    def async_write_data(self, device_type, address, value, callback):
        self.client.async_write_data(device_type, address, value, callback)

    def async_read_continuous_data(self, device_type, start_address, count, callback):
        self.client.async_read_continuous_data(
            device_type, start_address, count, callback)

    def async_write_continuous_data(self, device_type, start_address, values, callback):
        self.client.async_write_continuous_data(
            device_type, start_address, values, callback)

    def async_read_continuous_byte(self, device_type, start_address, count, callback):
        self.client.async_read_continuous_byte(
            device_type, start_address, count, callback)

    def async_write_continuous_byte(self, device_type, start_address, values, callback):
        self.client.async_write_continuous_byte(
            device_type, start_address, values, callback)


# --------------------
# 🔧 主程式入口點
# --------------------
def main(args=None):
    rclpy.init(args=args)
    node = PlcClientNode()

    # 測試同步 read_data（你可以改成其他方法）
    try:
        result = node.read_data(device_type="DM", address="7600")
        # node.get_logger().info(f"Read Result: {result}")  # 已註解：正常讀取不需要 log
    except Exception as e:
        node.get_logger().error(f"Service call failed: {e}")

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
