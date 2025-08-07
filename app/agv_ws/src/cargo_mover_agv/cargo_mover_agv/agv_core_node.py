# 調試代碼已註解 - 如需調試請取消註解
# import debugpy
# print("Waiting for debugger to attach...")
# debugpy.listen(("0.0.0.0", 5678))  # 監聽 Debug 連線
# debugpy.wait_for_client()  # 等待 VS Code 連線
# print("Debugger attached!")

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter  # Import Parameter class
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import SingleThreadedExecutor
import signal
from agv_base.hokuyo_dms_8bit import HokuyoDMS8Bit
from agv_base.robot import Robot
from plc_proxy.plc_client import PlcClient
import agv_base.states.auto_state
import agv_base.states.error_state
import agv_base.states.idle_state
import agv_base.states.manual_state
from agv_base.agv_states.mission_select_state import MissionSelectState
from agv_base.agv_states.wait_robot_state import WaitRobotState
from agv_base.agv_node_base import AgvNodebase
from agv_base.base_context import BaseContext
from cargo_mover_agv.cargo_context import CargoContext
from cargo_mover_agv.robot_context import RobotContext
# AGVs 和 TaskMsg 現在由 AgvNodebase 提供
import cargo_mover_agv.robot_states.idle_state


class AgvCoreNode(AgvNodebase):
    def __init__(self, node_name='agv_node_base', **kwargs):
        super().__init__(node_name=node_name, **kwargs)
        self.room_id = 0  # 設定房間ID
        self.work_id = 0  # 設定工作ID
        # 使用共用方法設置參數和訂閱
        self.setup_common_parameters()
        self.setup_agv_subscription()

        self.robot = Robot(self, parameter=None)
        self.hokuyo_dms_8bit_1 = HokuyoDMS8Bit(
            self, "/app/config/hokuyo_dms_config.yaml", "hokuyo_dms_cargo02_1")
        self.hokuyo_dms_8bit_2 = HokuyoDMS8Bit(
            self, "/app/config/hokuyo_dms_config.yaml", "hokuyo_dms_cargo02_2")

        self.base_context = BaseContext(
            agv_base.states.idle_state.IdleState(self))
        self.cargo_context = CargoContext(
            MissionSelectState(self))
        self.robot_context = RobotContext(
            cargo_mover_agv.robot_states.idle_state.IdleState(self))

        # 輸出日誌信息
        self.get_logger().info("🚚Cargo mover AGV狀態機啟動")

        # base狀態機
        self.base_context.after_handle += self.base_after_handle  # after_handle
        self.base_context.on_state_changed += self.state_changed  # 狀態切換訊息

        self.cargo_context.after_handle += self.agv_after_handle  # AGV狀態機
        self.cargo_context.on_state_changed += self.state_changed  # 狀態切換訊息

        self.robot_context.after_handle += self.robot_after_handle  # 手臂狀態機
        self.robot_context.on_state_changed += self.state_changed  # 狀態切換訊息

    def state_changed(self, old_state, new_state):
        self.common_state_changed(old_state, new_state)

    def base_after_handle(self, state):
        data = self.robot.read_robot_status()
        #self.get_logger().info(f"[Robot]-讀取PGNO: {data}")
        # 只能在 Auto 狀態下執行cargo_context.handle
        if isinstance(state, agv_base.states.idle_state.IdleState):
            pass
            # self.get_logger().info("[BASE]-Idle")
            # self.base_context.handle()
        if isinstance(state, agv_base.states.manual_state.ManualState):
            pass
            # self.get_logger().info("[BASE]-Manual")
            # self.base_context.handle()
        if isinstance(state, agv_base.states.auto_state.AutoState):
            # self.get_logger().info("[BASE]-Auto")
            self.cargo_context.handle()
        if isinstance(state, agv_base.states.error_state.ErrorState):
            # self.get_logger().info("[BASE]-Error")
            pass

    def agv_after_handle(self, state):
        if isinstance(state, WaitRobotState):
            # self.get_logger().info("[CARGO]-WaitRobot")
            # self.get_logger().info("[CARGO]-Idle")
            self.robot_context.handle()

    def robot_after_handle(self, state):
        if isinstance(state, cargo_mover_agv.robot_states.idle_state.IdleState):
            # self.get_logger().info("[Robot]-Idle")
            # self.robot_context.handle()
            pass

    # agvs_callback 現在由 AgvNodebase 提供


def main():
    # 初始化 rclpy
    rclpy.init()

    # 創建 AgvCoreNode 實例
    node = AgvCoreNode()

    # 使用 MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
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
