from agv_base.base_context import BaseContext
from agv_base.states.manual_state import ManualState
from agv_base.agv_node_base import AgvNodebase
from rclpy.executors import MultiThreadedExecutor
import rclpy
from rclpy.node import Node
import time


class AgvNodeTest(AgvNodebase):
    def __init__(self,node_name='agv_node_base', **kwargs):
        super().__init__(node_name=node_name, **kwargs)

        # 創建 BaseContext 並傳入初始狀態 (ManualState)
        self.base_context.set_state(ManualState(self))  # 初始狀態為 ManualState

        # 輸出日誌信息
        self.get_logger().info("ROS 2 Test AGV狀態機啟動 ManualState")


def main():
    # 初始化 rclpy，設置 ROS 2 節點
    rclpy.init()
    node = AgvNodeTest()

    # 使用 MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("🛑 偵測到 Ctrl+C，正在關閉節點...")
    finally:
        node.stop()
        node.get_logger().info("🛑 節點已關閉，ROS 2 即將關閉。")
        executor.shutdown()
        node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
