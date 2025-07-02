import rclpy
from rclpy.node import Node
from agv_base.hokuyo_dms_8bit import HokuyoDMS8Bit


def main():
    rclpy.init()
    node = rclpy.create_node("test_hokuyo_dms_8bit")

    try:
        # 測試初始化 HokuyoDMS8Bit
        hokuyo_dms = HokuyoDMS8Bit(
            node, "/app/config/hokuyo_dms_config.yaml", "hokuyo_dms_cargo_1"
        )
        node.get_logger().info("HokuyoDMS8Bit initialized successfully.")
    except Exception as e:
        node.get_logger().error(f"Failed to initialize HokuyoDMS8Bit: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
