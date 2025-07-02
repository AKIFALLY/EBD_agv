import rclpy
from rclpy.node import Node
from db_proxy.agvc_database_client import AGVCDatabaseClient


def main():
    rclpy.init()
    node = Node("test_query_node")
    client = AGVCDatabaseClient(node)

    def on_query_result(result):
        if result:
            node.get_logger().info(f"✅ 查詢成功: {result.success}")
            node.get_logger().info(f"📦 回傳資料: {result.results}")
        else:
            node.get_logger().warn("❌ 查詢失敗")

    client.async_generic_query(
        table_name='carrier',
        columns=[],
        data=[],
        condition='id=4',
        mode='select',
        callback=on_query_result
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()