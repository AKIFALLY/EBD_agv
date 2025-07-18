from agv_base.base_context import BaseContext
from agv_base.agv_node_base import AgvNodebase
from loader_agv.robot_states.idle_state import IdleState
from loader_agv.robot_context import RobotContext
from loader_agv.robot_states.put_cleaner.agv_port_check_have_state import AgvPortCheckHaveState
from agv_base.hokuyo_dms_8bit import HokuyoDMS8Bit
import rclpy
from rclpy.node import Node
import time
from db_proxy_interfaces.msg import Task as TaskMsg


class TestAgvCoreNode(AgvNodebase):
    def __init__(self):
        super().__init__()
        self.room_id = 2  # 設定房間ID (loader_agv 通常使用 room_id = 2)
        self.work_id = 2010101  # 設定工作ID (loader_agv 的工作ID格式)
        self.hokuyo_dms_8bit_1 = HokuyoDMS8Bit(
            self, "/app/config/hokuyo_dms_config.yaml", "hokuyo_dms_loader02")
        self.agv_state_context = RobotContext(
            AgvPortCheckHaveState(self))
        self.agv_state_context.on_state_changing += self.on_state_changing
        self.agv_state_context.on_state_changed += self.on_state_changed
        self.agv_state_context.before_handle += self.before_handle
        self.agv_state_context.after_handle += self.after_handle

        self.task = TaskMsg()

        self.get_logger().info("Test Loader AGV 狀態機啟動 AgvIdleState")
        # 測試cycle timer
        # self.timer.destroy()
        # self.timer = self.create_timer(0.01, self.context_handle)

    def destroy_node(self):
        self.agv_state_context.on_state_changing -= self.on_state_changing
        self.agv_state_context.on_state_changed -= self.on_state_changed
        self.agv_state_context.before_handle -= self.before_handle
        self.agv_state_context.after_handle -= self.after_handle
        self.get_logger().info("Test Loader AGV 狀態機關閉 解除事件綁定")

    def context_handle(self):
        try:
            # 處理當前的 Loader AGV 狀態邏輯
            self.agv_state_context.handle()

        except Exception as e:
            self.get_logger().error(f"未處理的 Loader AGV 狀態機異常: {e}")

    def on_state_changing(self, old_state, new_state):
        self.get_logger().info(
            f"Loader AGV狀態變更: {old_state.__class__.__name__} -> {new_state.__class__.__name__}")

    def on_state_changed(self, old_state, new_state):
        self.get_logger().info(
            f"Loader AGV狀態已變更: {old_state.__class__.__name__} -> {new_state.__class__.__name__}")

    def before_handle(self, state):
        self.get_logger().info(f"Loader AGV狀態處理: {state.__class__.__name__}")

    def after_handle(self, state):
        self.get_logger().info(f"Loader AGV狀態處理完成: {state.__class__.__name__}")


def main():
    # 初始化 rclpy，設置 ROS 2 節點
    rclpy.init()
    node = TestAgvCoreNode()

    try:

        rclpy.spin(node)  # 這行會阻塞，直到節點關閉
    except KeyboardInterrupt:
        node.get_logger().info("🛑 偵測到 Ctrl+C，正在關閉節點...loader agv node event test")
        node.destroy_node()

    finally:
        if rclpy.ok():  # 確保 ROS 還沒 shutdown
            rclpy.shutdown()
        node.get_logger().info("🛑 節點已關閉，ROS 2 已關閉。")


if __name__ == "__main__":
    main()
