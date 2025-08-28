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
from unloader_agv.unloader_context import UnloaderContext
from unloader_agv.robot_context import RobotContext
import unloader_agv.robot_states.idle_state
from unloader_agv.status_json_recorder import UnloaderAgvStatusJsonRecorder
# AGVs 和 TaskMsg 現在由 AgvNodebase 提供


class AgvCoreNode(AgvNodebase):
    def __init__(self, node_name='agv_node_base', **kwargs):
        super().__init__(node_name=node_name, **kwargs)

        # 使用共用方法設置參數和訂閱
        self.setup_common_parameters()
        self.setup_agv_subscription()

        self.robot = Robot(self, parameter=None)
        self.hokuyo_dms_8bit_1 = HokuyoDMS8Bit(
            self, "/app/config/hokuyo_dms_config.yaml", "hokuyo_dms_unloader02")

        self.base_context = BaseContext(
            agv_base.states.idle_state.IdleState(self))
        self.unloader_context = UnloaderContext(MissionSelectState(self))
        self.robot_context = RobotContext(
            unloader_agv.robot_states.idle_state.IdleState(self))

        # 輸出日誌信息
        self.get_logger().info("Unloader AGV狀態機啟動")

        # base狀態機
        self.base_context.after_handle += self.base_after_handle  # after_handle
        self.base_context.on_state_changed += self.state_changed  # 狀態切換訊息

        self.unloader_context.after_handle += self.agv_after_handle  # AGV狀態機
        self.unloader_context.on_state_changed += self.state_changed  # 狀態切換訊息

        self.robot_context.after_handle += self.robot_after_handle  # 手臂狀態機
        self.robot_context.on_state_changed += self.state_changed  # 狀態切換訊息
        
        # 初始化 JSON 狀態記錄器
        self.get_logger().info("🔍 開始初始化 Unloader AGV JSON 狀態記錄器...")
        try:
            import os
            output_dir = "/tmp"  # 統一輸出到 /tmp 目錄
            self.get_logger().info(f"🔍 檢查輸出目錄: {output_dir}")
            
            # 檢查目錄是否存在
            if os.path.exists(output_dir):
                self.get_logger().info(f"✅ 輸出目錄存在: {output_dir}")
            else:
                self.get_logger().warn(f"⚠️ 輸出目錄不存在，將嘗試創建: {output_dir}")
                
            self.json_recorder = UnloaderAgvStatusJsonRecorder(output_dir=output_dir)
            self.get_logger().info("✅ Unloader AGV JSON 狀態記錄器物件已創建")
            
            # 設置定時覆蓋保存 (每1秒更新一次)
            self.get_logger().info("🔍 創建定時器...")
            self.json_save_timer = self.create_timer(1.0, self._update_json_status_file)
            self.get_logger().info("✅ Unloader AGV JSON 狀態文件定時更新已啟動 (1秒間隔)")
            
            # 立即執行一次測試
            self.get_logger().info("🔍 執行首次 JSON 狀態更新測試...")
            self._update_json_status_file()
            
        except Exception as e:
            self.get_logger().error(f"❌ Unloader AGV JSON 狀態記錄器初始化失敗: {e}")
            import traceback
            self.get_logger().error(f"❌ 錯誤詳細堆疊: {traceback.format_exc()}")
            self.json_recorder = None
            self.json_save_timer = None

    def state_changed(self, old_state, new_state):
        self.common_state_changed(old_state, new_state)

    def base_after_handle(self, state):
        # data = self.robot.read_pgno()
        # self.get_logger().info(f"[Robot]-讀取PGNO: {data}")
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
            self.unloader_context.handle()
        if isinstance(state, agv_base.states.error_state.ErrorState):
            # self.get_logger().info("[BASE]-Error")
            pass

    def agv_after_handle(self, state):
        if isinstance(state, WaitRobotState):
            # self.get_logger().info("[UNLOADER]-WaitRobot")
            self.robot_context.handle()

    def robot_after_handle(self, state):
        if isinstance(state, unloader_agv.robot_states.idle_state.IdleState):
            # self.get_logger().info("[Robot]-Idle")
            # self.robot_context.handle()
            pass
    
    def _update_json_status_file(self):
        """定時更新 JSON 狀態文件"""
        if self.json_recorder is None:
            self.get_logger().error("❌ JSON 記錄器為 None，無法更新狀態文件")
            return
            
        try:
            # 使用包含 AGV ID 的檔案名稱，統一格式
            agv_id = self.agv_id if hasattr(self, 'agv_id') and self.agv_id else "unloader01"
            filename = f"agv_status_{agv_id}.json"
            
            filepath = self.json_recorder.save_complete_status_to_file(self, filename)
            
            # 驗證文件是否真的被創建 (每10次才打印一次，避免日誌過多)
            import os
            if not hasattr(self, '_json_update_count'):
                self._json_update_count = 0
            self._json_update_count += 1
            
            if os.path.exists(filepath):
                file_size = os.path.getsize(filepath)
                if self._json_update_count % 10 == 1:  # 第1次，第11次，第21次...打印
                    self.get_logger().info(f"📝 Unloader AGV JSON 狀態文件更新正常 (第{self._json_update_count}次): {filepath}, 大小: {file_size} bytes")
            else:
                self.get_logger().error(f"❌ 文件未被創建: {filepath}")
            
        except Exception as e:
            self.get_logger().error(f"❌ 定時更新 Unloader AGV JSON 狀態文件失敗: {e}")
            import traceback
            self.get_logger().error(f"❌ 錯誤詳細堆疊: {traceback.format_exc()}")

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