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
from cargo_mover_agv.status_json_recorder import CargoAgvStatusJsonRecorder
# AGVs 和 TaskMsg 現在由 AgvNodebase 提供
import cargo_mover_agv.robot_states.idle_state


class AgvCoreNode(AgvNodebase):
    def __init__(self, node_name='agv_node_base', **kwargs):
        super().__init__(node_name=node_name, **kwargs)
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

        # 初始化 JSON 狀態記錄器
        self.get_logger().info("🔍 開始初始化 JSON 狀態記錄器...")
        try:
            import os
            output_dir = "/tmp"  # 統一輸出到 /tmp 目錄
            self.get_logger().info(f"🔍 檢查輸出目錄: {output_dir}")
            
            # 檢查目錄是否存在
            if os.path.exists(output_dir):
                self.get_logger().info(f"✅ 輸出目錄存在: {output_dir}")
            else:
                self.get_logger().warn(f"⚠️ 輸出目錄不存在，將嘗試創建: {output_dir}")
                
            self.json_recorder = CargoAgvStatusJsonRecorder(output_dir=output_dir)
            self.get_logger().info("✅ JSON 狀態記錄器物件已創建")
            
            # 檢查記錄器是否正常
            if self.json_recorder:
                self.get_logger().info("✅ JSON 狀態記錄器驗證通過")
            else:
                raise Exception("記錄器物件為 None")
            
            # 設置定時覆蓋保存 (每1秒更新一次，使用固定文件名)
            self.get_logger().info("🔍 創建定時器...")
            self.json_save_timer = self.create_timer(1.0, self._update_json_status_file)
            self.get_logger().info("✅ JSON 狀態文件定時更新已啟動 (1秒間隔)")
            
            # 立即執行一次測試
            self.get_logger().info("🔍 執行首次 JSON 狀態更新測試...")
            self._update_json_status_file()
            
        except Exception as e:
            self.get_logger().error(f"❌ JSON 狀態記錄器初始化失敗: {e}")
            import traceback
            self.get_logger().error(f"❌ 錯誤詳細堆疊: {traceback.format_exc()}")
            self.json_recorder = None
            self.json_save_timer = None

    def state_changed(self, old_state, new_state):
        self.common_state_changed(old_state, new_state)

    def _handle_non_auto_state(self, base_state_name):
        """處理非 AutoState 時的邏輯"""
        # 1. 轉換 Robot 層到 IdleState
        if not isinstance(self.robot_context.state, cargo_mover_agv.robot_states.idle_state.IdleState):
            try:
                from cargo_mover_agv.robot_states.idle_state import IdleState
                self.get_logger().info(f"🔄 Base {base_state_name} → Robot Idle 狀態轉換")
                self.robot_context.set_state(IdleState(self))
            except Exception as e:
                self.get_logger().error(f"❌狀態轉換失敗 (Base {base_state_name} → Robot Idle): {str(e)}")

        # 2. 檢查 AGV 層是否在 WaitRobotState，若是則轉換到 MissionSelectState
        if isinstance(self.cargo_context.state, WaitRobotState):
            try:
                from agv_base.agv_states.mission_select_state import MissionSelectState
                self.get_logger().info(f"🔄 跳出 AutoState 時，AGV WaitRobot → MissionSelect 狀態轉換")
                self.cargo_context.set_state(MissionSelectState(self))
            except Exception as e:
                self.get_logger().error(f"❌AGV 狀態轉換失敗 (WaitRobot → MissionSelect): {str(e)}")
        
        # 3. 繼續執行一次 AGV 層狀態機，確保狀態轉換生效
        try:
            self.cargo_context.handle()
        except Exception as e:
            self.get_logger().error(f"❌非 AutoState 下執行 cargo_context.handle() 失敗: {str(e)}")

    def base_after_handle(self, state):
        data = self.robot.read_robot_status()
        #self.get_logger().info(f"[Robot]-讀取PGNO: {data}")
        # 只能在 Auto 狀態下執行cargo_context.handle
        if isinstance(state, agv_base.states.idle_state.IdleState):
            # 非 AutoState 狀態下的處理
            self._handle_non_auto_state("Idle")
        if isinstance(state, agv_base.states.manual_state.ManualState):
            # 非 AutoState 狀態下的處理
            self._handle_non_auto_state("Manual")
        if isinstance(state, agv_base.states.auto_state.AutoState):
            # self.get_logger().info("[BASE]-Auto")
            self.cargo_context.handle()
        if isinstance(state, agv_base.states.error_state.ErrorState):
            # 非 AutoState 狀態下的處理
            self._handle_non_auto_state("Error")

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

    # ==================== JSON 狀態記錄功能 ====================

    def save_status_snapshot(self, filename=None):
        """
        保存當前完整狀態快照到 JSON 文件
        
        Args:
            filename: 可選的文件名，如果未提供則使用時間戳
            
        Returns:
            保存的文件路徑，如果失敗則返回 None
        """
        if self.json_recorder is None:
            self.get_logger().error("❌ JSON 記錄器未初始化，無法保存狀態")
            return None
            
        try:
            filepath = self.json_recorder.save_status_to_file(self, filename)
            self.get_logger().info(f"✅ 狀態快照已保存到: {filepath}")
            return filepath
        except Exception as e:
            self.get_logger().error(f"❌ 保存狀態快照失敗: {e}")
            return None
    
    def get_status_summary_json(self):
        """
        獲取狀態摘要的 JSON 字符串
        
        Returns:
            JSON 字符串，如果失敗則返回 None
        """
        if self.json_recorder is None:
            self.get_logger().error("❌ JSON 記錄器未初始化，無法獲取狀態摘要")
            return None
            
        try:
            import json
            summary = self.json_recorder.get_status_summary(self)
            return json.dumps(summary, indent=2, ensure_ascii=False)
        except Exception as e:
            self.get_logger().error(f"❌ 獲取狀態摘要失敗: {e}")
            return None
    
    def print_status_summary(self):
        """在日誌中打印狀態摘要"""
        summary_json = self.get_status_summary_json()
        if summary_json:
            self.get_logger().info(f"📊 當前狀態摘要:\n{summary_json}")
        else:
            self.get_logger().error("❌ 無法獲取狀態摘要")
    
    def start_continuous_logging(self, interval_seconds=5.0, max_files=100):
        """
        開始持續的狀態記錄
        
        Args:
            interval_seconds: 記錄間隔 (秒)
            max_files: 最大記錄文件數量
            
        Returns:
            記錄執行緒物件，如果失敗則返回 None
        """
        if self.json_recorder is None:
            self.get_logger().error("❌ JSON 記錄器未初始化，無法開始持續記錄")
            return None
            
        try:
            thread = self.json_recorder.save_status_continuously(self, interval_seconds, max_files)
            self.get_logger().info(f"🔄 已開始持續狀態記錄 (間隔: {interval_seconds}秒, 最大文件: {max_files})")
            return thread
        except Exception as e:
            self.get_logger().error(f"❌ 開始持續記錄失敗: {e}")
            return None
    
    def save_status_on_state_change(self):
        """在狀態改變時自動保存狀態"""
        try:
            from datetime import datetime
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"cargo_agv_state_change_{timestamp}.json"
            return self.save_status_snapshot(filename)
        except Exception as e:
            self.get_logger().error(f"❌ 狀態改變時保存失敗: {e}")
            return None
    
    def _update_json_status_file(self):
        """定時更新 JSON 狀態文件 (覆蓋模式)"""
        if self.json_recorder is None:
            self.get_logger().error("❌ JSON 記錄器為 None，無法更新狀態文件")
            return
            
        try:
            # 使用包含 AGV ID 的檔案名稱，統一格式
            agv_id = self.agv_id if hasattr(self, 'agv_id') and self.agv_id else "cargo01"
            filename = f"agv_status_{agv_id}.json"
            
            filepath = self.json_recorder.save_complete_frontend_status_to_file(self, filename)
            
            # 驗證文件是否真的被創建 (每10次才打印一次，避免日誌過多)
            import os
            if not hasattr(self, '_json_update_count'):
                self._json_update_count = 0
            self._json_update_count += 1
            
            if os.path.exists(filepath):
                file_size = os.path.getsize(filepath)
                if self._json_update_count % 10 == 1:  # 第1次，第11次，第21次...打印
                    self.get_logger().info(f"📝 JSON 狀態文件更新正常 (第{self._json_update_count}次): {filepath}, 大小: {file_size} bytes")
            else:
                self.get_logger().error(f"❌ 文件未被創建: {filepath}")
            
        except Exception as e:
            self.get_logger().error(f"❌ 定時更新 JSON 狀態文件失敗: {e}")
            import traceback
            self.get_logger().error(f"❌ 錯誤詳細堆疊: {traceback.format_exc()}")
    
    def stop(self):
        """停止節點時的清理工作"""
        try:
            # 停止 JSON 更新定時器
            if hasattr(self, 'json_save_timer') and self.json_save_timer:
                self.json_save_timer.cancel()
                self.get_logger().info("⏰ JSON 狀態定時更新已停止")
                
            # 不再保存最終狀態文件，只使用 current_status.json
                
        except Exception as e:
            self.get_logger().error(f"❌ 節點停止時保存最終狀態失敗: {e}")
            
        # 調用父類的停止方法
        if hasattr(super(), 'stop'):
            super().stop()


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
