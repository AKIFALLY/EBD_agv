from agv_base.base_context import BaseContext
from agv_base.agv_node_base import AgvNodebase
from cargo_mover_agv.robot_context import RobotContext
from cargo_mover_agv.robot_states.entrance.select_rack_port_state import SelectRackPortState
from cargo_mover_agv.robot_states.entrance.check_rack_side_state import CheckRackSideState
from cargo_mover_agv.robot_states.entrance.put_tranfer_state import PutTranferState
from cargo_mover_agv.robot_states.entrance.transfer_vision_position_state import TransferVisionPositionState
from cargo_mover_agv.robot_states.exit.transfer_check_have_state import TransferCheckHaveState
from cargo_mover_agv.robot_states.entrance.transfer_check_empty_state import TransferCheckEmptyState
from cargo_mover_agv.robot_states.entrance.take_rack_port_state import TakeRackPortState
from agv_base.hokuyo_dms_8bit import HokuyoDMS8Bit
import rclpy
from rclpy.node import Node
import time


class TestAgvCoreNode(AgvNodebase):
    def __init__(self):
        super().__init__()
        self.hokuyo_dms_8bit_1 = HokuyoDMS8Bit(
            self, "/app/config/hokuyo_dms_config.yaml", "hokuyo_dms_cargo02_1")
        self.hokuyo_dms_8bit_2 = HokuyoDMS8Bit(
            self, "/app/config/hokuyo_dms_config.yaml", "hokuyo_dms_cargo02_2")
        self.agv_state_context = RobotContext(
            TransferCheckHaveState(self))
        self.agv_state_context.on_state_changing += self.on_state_changing
        self.agv_state_context.on_state_changed += self.on_state_changed
        self.agv_state_context.before_handle += self.before_handle
        self.agv_state_context.after_handle += self.after_handle

        self.get_logger().info("Test AGV 狀態機啟動 AgvIdleState")
        # 測試cycle timer
        # self.timer.destroy()
        # self.timer = self.create_timer(0.01, self.context_handle)

    def destroy_node(self):
        self.agv_state_context.on_state_changing -= self.on_state_changing
        self.agv_state_context.on_state_changed -= self.on_state_changed
        self.agv_state_context.before_handle -= self.before_handle
        self.agv_state_context.after_handle -= self.after_handle
        self.get_logger().info("Test AGV 狀態機關閉 解除事件綁定")

    def context_handle(self):
        try:
            # 處理當前的 AGV 狀態邏輯
            self.agv_state_context.handle()

        except Exception as e:
            self.get_logger().error(f"未處理的 AGV 狀態機異常: {e}")

    def on_state_changing(self, old_state, new_state):
        self.get_logger().info(
            f"AGV狀態變更: {old_state.__class__.__name__} -> {new_state.__class__.__name__}")

    def on_state_changed(self, old_state, new_state):
        self.get_logger().info(
            f"AGV狀態已變更: {old_state.__class__.__name__} -> {new_state.__class__.__name__}")

    def before_handle(self, state):
        self.get_logger().info(f"AGV狀態處理: {state.__class__.__name__}")

    def after_handle(self, state):
        self.get_logger().info(f"AGV狀態處理完成: {state.__class__.__name__}")


def main():
    # 初始化 rclpy，設置 ROS 2 節點
    rclpy.init()
    node = TestAgvCoreNode()

    try:

        rclpy.spin(node)  # 這行會阻塞，直到節點關閉
    except KeyboardInterrupt:
        node.get_logger().info("🛑 偵測到 Ctrl+C，正在關閉節點...agv node event test")
        node.destroy_node()

    finally:
        if rclpy.ok():  # 確保 ROS 還沒 shutdown
            rclpy.shutdown()
        node.get_logger().info("🛑 節點已關閉，ROS 2 已關閉。")


if __name__ == "__main__":
    main()
