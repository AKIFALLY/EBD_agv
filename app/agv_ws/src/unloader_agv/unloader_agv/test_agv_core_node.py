from agv_base.base_context import BaseContext
from agv_base.agv_node_base import AgvNodebase
from unloader_agv.robot_states.idle_state import IdleState
from unloader_agv.robot_context import RobotContext
from agv_base.hokuyo_dms_8bit import HokuyoDMS8Bit
import rclpy
from rclpy.node import Node
import time
from db_proxy_interfaces.msg import Task as TaskMsg


class TestAgvCoreNode(AgvNodebase):
    def __init__(self):
        super().__init__()
        self.room_id = 2  # 設定房間ID (unloader_agv 通常使用 room_id = 3)
        self.work_id = 2020102  # 設定工作ID (unloader_agv 的工作ID格式)
        self.hokuyo_dms_8bit_1 = HokuyoDMS8Bit(
            self, "/app/config/hokuyo_dms_config.yaml", "hokuyo_dms_unloader02")
        self.agv_state_context = RobotContext(
            IdleState(self))
        self.agv_state_context.on_state_changing += self.on_state_changing
        self.agv_state_context.on_state_changed += self.on_state_changed
        self.agv_state_context.before_handle += self.before_handle
        self.agv_state_context.after_handle += self.after_handle

        self.task = TaskMsg()

        self.get_logger().info("Test Unloader AGV 狀態機啟動 AgvIdleState")
        # 測試cycle timer
        # self.timer.destroy()
        # self.timer = self.create_timer(0.01, self.context_handle)

    def destroy_node(self):
        self.agv_state_context.on_state_changing -= self.on_state_changing
        self.agv_state_context.on_state_changed -= self.on_state_changed
        self.agv_state_context.before_handle -= self.before_handle
        self.agv_state_context.after_handle -= self.after_handle
        self.get_logger().info("Test Unloader AGV 狀態機關閉 解除事件綁定")

    def context_handle(self):
        try:
            # 處理當前的 Unloader AGV 狀態邏輯
            self.agv_state_context.handle()

        except Exception as e:
            self.get_logger().error(f"未處理的 Unloader AGV 狀態機異常: {e}")

    def on_state_changing(self, old_state, new_state):
        self.get_logger().info(
            f"🔄 Unloader AGV 狀態轉換中: {old_state.__class__.__name__} -> {new_state.__class__.__name__}")

    def on_state_changed(self, old_state, new_state):
        self.get_logger().info(
            f"✅ Unloader AGV 狀態已轉換: {old_state.__class__.__name__} -> {new_state.__class__.__name__}")

    def before_handle(self, state):
        # 調試用日誌（如需調試請取消註解）
        # self.get_logger().debug(f"🔍 Unloader AGV 狀態處理前: {state.__class__.__name__}")
        pass

    def after_handle(self, state):
        # 調試用日誌（如需調試請取消註解）
        # self.get_logger().debug(f"✅ Unloader AGV 狀態處理後: {state.__class__.__name__}")
        pass


def main():
    # 初始化 rclpy，設置 ROS 2 節點
    rclpy.init()
    node = TestAgvCoreNode()

    try:

        rclpy.spin(node)  # 這行會阻塞，直到節點關閉
    except KeyboardInterrupt:
        node.get_logger().info("🛑 偵測到 Ctrl+C，正在關閉節點...unloader agv node event test")
        node.destroy_node()

    finally:
        if rclpy.ok():  # 確保 ROS 還沒 shutdown
            rclpy.shutdown()
        node.get_logger().info("🛑 節點已關閉，ROS 2 已關閉。")


if __name__ == "__main__":
    main()
