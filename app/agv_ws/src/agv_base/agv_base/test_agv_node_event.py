# import debugpy
# print("Waiting for debugger to attach...")
# debugpy.listen(("0.0.0.0", 5678))  # 監聽 Debug 連線
# debugpy.wait_for_client()  # 等待 VS Code 連線
# print("Debugger attached!")
#

from agv_base.base_context import BaseContext
from agv_base.states.test_none_state import NoneState
import rclpy
from rclpy.node import Node
import time


class AgvNodeEventTest(Node):
    def __init__(self):
        super().__init__('agv_node_test')

        # 創建 BaseContext 並傳入初始狀態 (NoneState)
        self.base_context = BaseContext(NoneState(self))  # 初始狀態為 NoneState

        self.base_context.on_state_changing += self.on_state_changing
        self.base_context.on_state_changed += self.on_state_changed
        self.base_context.before_handle += self.before_handle
        self.base_context.after_handle += self.after_handle
        # 輸出日誌信息
        self.get_logger().info("ROS 2 Test AGV BASE 狀態機啟動 NoneState")

    def destroy_node(self):
        self.base_context.on_state_changing -= self.on_state_changing
        self.base_context.on_state_changed -= self.on_state_changed
        self.base_context.before_handle -= self.before_handle
        self.base_context.after_handle -= self.after_handle

        self.get_logger().info("ROS 2 Test AGV BASE 狀態機關閉 解除事件綁定")

    def context_handle(self):
        try:
            # 處理當前的 AGV 狀態邏輯
            self.base_context.handle()

        except Exception as e:
            self.get_logger().error(f"未處理的 AGV 狀態機異常: {e}")

    def on_state_changing(self, old_state, new_state):
        self.get_logger().info(
            f"狀態變更: {old_state.__class__.__name__} -> {new_state.__class__.__name__}")

    def on_state_changed(self, old_state, new_state):
        self.get_logger().info(
            f"狀態已變更: {old_state.__class__.__name__} -> {new_state.__class__.__name__}")

    def before_handle(self, state):
        self.get_logger().info(f"處理狀態: {state.__class__.__name__}")

    def after_handle(self, state):
        self.get_logger().info(f"狀態處理完成: {state.__class__.__name__}")


def main():
    # 初始化 rclpy，設置 ROS 2 節點
    rclpy.init()
    node = AgvNodeEventTest()

    try:
        while rclpy.ok():
            time.sleep(0.1)
            node.context_handle()  # 處理當前狀態邏輯
            rclpy.spin_once(node, timeout_sec=0.1)  # 讓節點運行並處理回調

    except KeyboardInterrupt:
        node.get_logger().info("🛑 偵測到 Ctrl+C，正在關閉節點...agv node event test")
        node.destroy_node()

    finally:
        if rclpy.ok():  # 確保 ROS 還沒 shutdown
            rclpy.shutdown()
        node.get_logger().info("🛑 節點已關閉，ROS 2 已關閉。")


if __name__ == "__main__":
    main()
