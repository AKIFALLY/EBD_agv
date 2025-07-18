# 調試代碼已註解 - 如需調試請取消註解
# import debugpy
# print("Waiting for debugger to attach...")
# debugpy.listen(("0.0.0.0", 5678))  # 監聽 Debug 連線
# debugpy.wait_for_client()  # 等待 VS Code 連線
# print("Debugger attached!")

from agv_base.hokuyo_dms_8bit import HokuyoDMS8Bit
from agv_base.robot import Robot
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import SingleThreadedExecutor
import signal
from plc_proxy.plc_client import PlcClient
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter  # Import Parameter class
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import SingleThreadedExecutor
from agv_base.hokuyo_dms_8bit import HokuyoDMS8Bit
import agv_base.states.auto_state
import agv_base.states.error_state
import agv_base.states.idle_state
import agv_base.states.manual_state
from agv_base.agv_states.mission_select_state import MissionSelectState
from agv_base.agv_states.wait_robot_state import WaitRobotState
from agv_base.robot import Robot
from agv_base.agv_node_base import AgvNodebase
from agv_base.base_context import BaseContext
from cargo_mover_agv.cargo_context import CargoContext
from cargo_mover_agv.robot_context import RobotContext
from db_proxy_interfaces.msg import AGVs
from db_proxy_interfaces.msg import Task as TaskMsg


import cargo_mover_agv.cargo_states.idle_state
import cargo_mover_agv.robot_states.idle_state


class AgvCoreNode(AgvNodebase):
    def __init__(self, node_name='agv_node_base', **kwargs):
        super().__init__(node_name=node_name, **kwargs)

        # 初始化變數屬性
        self.declare_parameter("room_id", 0)  # 預設房間ID為0
        self.room_id = self.get_parameter(
            "room_id").get_parameter_value().integer_value  # 取得room_id參數值
        self.get_logger().info(f"✅ 已接收 room_id: {self.room_id}")

        self.pathdata = None  # 路徑資料
        self.mission_id = None  # 任務ID
        self.node_id = None  # 任務目標節點
        self.AGV_id = 0  # AGV ID
        self.task = TaskMsg()
        self.agvsubscription = self.create_subscription(
            AGVs, '/agvc/agvs', self.agvs_callback, 10)  # QoS profile depth=10

        self.robot = Robot(self, parameter=None)
        self.hokuyo_dms_8bit_1 = HokuyoDMS8Bit(
            self, "/app/config/hokuyo_dms_config.yaml", "hokuyo_dms_cargo02_1")
        self.hokuyo_dms_8bit_2 = HokuyoDMS8Bit(
            self, "/app/config/hokuyo_dms_config.yaml", "hokuyo_dms_cargo02_2")

        self.base_context = BaseContext(
            agv_base.states.idle_state.IdleState(self))
        self.loader_context = CargoContext(
            MissionSelectState(self))
        self.robot_context = RobotContext(
            cargo_mover_agv.robot_states.idle_state.IdleState(self))

        # 輸出日誌信息
        self.get_logger().info("Cargo mover AGV狀態機啟動")

        # base狀態機
        self.base_context.after_handle += self.base_after_handle  # after_handle
        self.base_context.on_state_changed += self.state_changed  # 狀態切換訊息

        self.cargo_context.after_handle += self.agv_after_handle  # AGV狀態機
        self.cargo_context.on_state_changed += self.state_changed  # 狀態切換訊息

        self.robot_context.after_handle += self.robot_after_handle  # 手臂狀態機
        self.robot_context.on_state_changed += self.state_changed  # 狀態切換訊息

    def state_changed(self, old_state, new_state):
        self.get_logger().info(
            f"狀態變更: {old_state.__class__.__name__} -> {new_state.__class__.__name__}")

    def base_after_handle(self, state):
        data = self.robot.read_robot_status()
        self.get_logger().info(f"[Robot]-讀取PGNO: {data}")
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

    def agvs_callback(self, msg: AGVs):
        """處理 AGVs 訂閱消息"""
        namespace = self.get_namespace().lstrip('/')
        self.get_logger().info(f"📥 當前命名空間: {namespace}")
        self.get_logger().info(f"📦 接收 AGVs 數量: {len(msg.datas)}")

        # 調試用：列出所有 AGV 資訊（如需調試請取消註解）
        # for i, a in enumerate(msg.datas):
        #    self.get_logger().debug(f"[{i}] AGV: id={a.id}, name={a.name}")

        agv = next((a for a in msg.datas if a.name == namespace), None)

        if agv:
            self.AGV_id = agv.id
            self.get_logger().info(f"✅ 訂閱到 AGV_ID: {self.AGV_id} Name: {agv.name}")
            self.destroy_subscription(self.agvsubscription)
            self.get_logger().info("✅ 停止訂閱 AGVs 訊息")
        else:
            self.get_logger().warn("⚠️ 找不到符合命名空間的 AGV")


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
