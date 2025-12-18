import time
from agv_base.base_context import BaseContext
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import SingleThreadedExecutor
import threading
from rclpy.clock import Clock
from plc_interfaces.srv import ReadContinuousByte  # 替換成你的實際 service 名稱
from keyence_plc.keyence_plc_memory import PlcMemory
from plc_proxy.plc_client import PlcClient
from agv_base.states.idle_state import IdleState
from agv_base.agv_status import AgvStatus
from agv_interfaces.msg import AgvStatus as AgvStatusMsg
from db_proxy_interfaces.msg import Task as TaskMsg
import json
import os
import yaml
import requests


class AgvNodebase(Node):
    def __init__(self, node_name='agv_node_base', **kwargs):
        super().__init__(node_name=node_name, **kwargs)

        # 初始化讀取PLC狀態
        self.dMmemory = PlcMemory(
            65536 * 2
        )  # PLC 記憶體大小 65536 word (1 word = 2 byte)
        # 創建服務客戶端讀--讀取AGV_PLC狀態
        self.plc_client = PlcClient(self)

        # self.client = self.create_client(ReadContinuousByte, 'read_continuous_byte')
        # 輸出日誌信息
        self.get_logger().info("🤖ROS 2  AGV狀態機啟動 None_State")
        self.agv_status = AgvStatus()  # 初始化 AGV 狀態
        # 等待服務可用

        # ✅ 初始化 BaseContext 的狀態類變數（如果尚未初始化）
        # 注意：AGV 層不使用 IdleState，直接從 MissionSelectState 開始
        if BaseContext.MissionSelectState is None:
            from agv_base.agv_states.mission_select_state import MissionSelectState
            from agv_base.agv_states.write_path_state import WritePathState
            from agv_base.agv_states.Running_state import RunningState
            from agv_base.agv_states.wait_robot_state import WaitRobotState

            BaseContext.MissionSelectState = MissionSelectState
            BaseContext.WritePathState = WritePathState
            BaseContext.RunningState = RunningState
            BaseContext.WaitRobotState = WaitRobotState

        # 創建 BaseContext 並傳入初始狀態 (IdleState)
        self.base_context = BaseContext(IdleState(self))  # 初始狀態為 Idle
        # 50ms 執行一次主迴圈(read plc data , context.handle)
        self.timer = self.create_timer(0.05, self.main_loop_timer)
        self.timer_to_write_status = self.create_timer(0.2, self.toWriteStatus)
        self._status_publisher = self.create_publisher(AgvStatusMsg, "/agv/status", 10)
        self.clock = Clock()  # 建立 ROS 2 時鐘
        self.start_time = self.clock.now()  # 記錄請求開始時間
        self.requesting = False  # 重置請求狀態
        self.read_cycle_time_ms = 200  # 設定循環時間為 200ms
        
        self._running = False
        self._thread = None
        self.plc_heartbeat = 0  # PLC 心跳計數器
        self.BaseState = 0  # 狀態機狀態
        self.writing_status = False  # 狀態寫入 PLC 標誌
        self.count = 0  # 計數器，用於執行次數

        # self.start(one_cycle_ms=50)
        self.last_one_sec = int(time.time() * 1000)  # 取得現在時間（ms）
        self.state_display_counter = 0  # 狀態顯示計數器（每5秒輸出一次）
        
        # 共用變數初始化
        self.pathdata = None  # 路徑資料
        self.mission_id = None  # 任務ID (已廢棄，保留向後兼容，請使用 self.task.id)
        self.node_id = None  # 任務目標節點
        self.agv_id = 0  # AGV ID (數據庫 agv 表主键，已廢棄)
        self.agv_name = self.get_namespace().lstrip('/')  # AGV 名稱 (例如 "loader01")
        self.robot_finished = False  # 機器人是否完成動作
        self.task = TaskMsg()

        # 全局 tasks 相關變數（使用 Web API 輪詢）
        self.latest_tasks = []  # 全局任務列表（所有狀態共享）
        self.last_tasks_fetch_time = None  # 最後取得 tasks 的時間
        self.last_tasks_callback_time = None  # 向後兼容：MissionSelectState 使用此變數名稱
        self.last_tasks_log_time = None  # 最後輸出 tasks 日誌的時間
        self.tasks_api_interval = 2.0  # Web API 輪詢間隔（秒）

        # AGVC Web API 配置（從 config 載入）
        self.agvc_api_config = self._load_agvc_api_config()
        self.agvc_api_base_url = f"http://{self.agvc_api_config['ip']}:{self.agvc_api_config['port']}"

        # 建立 tasks Web API 輪詢 timer
        self._setup_tasks_api_polling()

    def start(self, one_cycle_ms=50):
        """啟動搖桿監聽 (獨立執行緒)"""
        if self._running:
            return

        self._running = True
        self._thread = threading.Thread(
            target=self.main_loop, args=(one_cycle_ms,))
        self._thread.daemon = True
        self._thread.start()

    def stop(self):
        """停止"""
        print("[STOP] Called stop()...")
        self._running = False
        if self._thread and self._thread.is_alive():
            if threading.current_thread() != self._thread:
                self._thread.join(timeout=1.0)
            else:
                print("⚠️ 無法在 agv_node_base 執行緒內 join 自己，略過 join()")

    def main_loop(self, one_cycle_ms=50):
        """主迴圈"""
        last_cycle_time = int(time.time() * 1000)  # 取得現在時間（ms）
        while self._running and rclpy.ok():
            now = int(time.time() * 1000)  # 取得現在時間（ms）
            elapsed = now - last_cycle_time  # 經過的毫秒數
            if elapsed >= one_cycle_ms:
                last_cycle_time = now  # 更新上次執行時間
                try:
                    self.get_logger().info(
                        f"✅ main_loop fired! elapsed = {elapsed} ms")
                    self.read_plc_data()
                    self.context_handle()
                except Exception as e:
                    msg = str(e)
                    if not rclpy.ok() or "context is invalid" in msg:
                        self.get_logger().warn("🛑 ROS context 無效或已關閉，將退出執行迴圈。")
                        break
                    self.get_logger().error(f"⚠️ AGV 狀態機異常: {msg}")

            else:
                time.sleep(0.001)  # 避免 CPU 空轉
        self._running = False

    def main_loop_timer(self):
        """主迴圈"""

        now = int(time.time() * 1000)  # 取得現在時間（ms）
        elapsed = now - self.last_one_sec  # 經過的毫秒數
        if elapsed >= 1000:
            self.last_one_sec = now  # 更新上次執行時間
            # 1000 # 1秒發佈一次狀態
            self.publish_agv_status()

        try:
            # self.get_logger().info(f"requesting!{self.requesting}")
            if self.requesting:
                self.count += 1
            self.read_plc_data()

            # 🔄 檢查訂閱超時並啟用資料庫備援
            #self._check_subscription_timeout_and_fallback() #直連資料庫的方式在insert或update操作時不正常斷開(斷電之類的)的情況可能造成資料表鎖無法釋放,先不使用,建議改成web api task 備援

            self.context_handle()

            # 📊 每5秒輸出一次當前狀態 (100次 × 50ms = 5000ms)
            self.state_display_counter += 1
            if self.state_display_counter >= 100:
                self.state_display_counter = 0
                try:
                    # 📋 三層狀態機狀態
                    base_state = self.base_context.state.__class__.__name__ if self.base_context and self.base_context.state else "Unknown"

                    # AGV 層狀態 (loader_context, unloader_context, cargo_context)
                    agv_state = "None"
                    if hasattr(self, 'loader_context') and self.loader_context and hasattr(self.loader_context, 'state') and self.loader_context.state:
                        agv_state = self.loader_context.state.__class__.__name__
                    elif hasattr(self, 'unloader_context') and self.unloader_context and hasattr(self.unloader_context, 'state') and self.unloader_context.state:
                        agv_state = self.unloader_context.state.__class__.__name__
                    elif hasattr(self, 'cargo_context') and self.cargo_context and hasattr(self.cargo_context, 'state') and self.cargo_context.state:
                        agv_state = self.cargo_context.state.__class__.__name__

                    # Robot 層狀態
                    robot_state = "None"
                    if hasattr(self, 'robot_context') and self.robot_context and hasattr(self.robot_context, 'state') and self.robot_context.state:
                        robot_state = self.robot_context.state.__class__.__name__

                    # 📍 其他狀態資訊
                    has_path = "是" if getattr(self.agv_status, 'AGV_PATH', False) else "否"
                    fpgv = getattr(self.agv_status, 'AGV_FPGV', None)
                    position = f"前PGV={fpgv}" if fpgv is not None else "前PGV=None"
                    mission_info = f"任務ID={self.task.id}" if hasattr(self, 'task') and self.task and hasattr(self.task, 'id') and self.task.id else "無任務"

                    self.get_logger().info(
                        f"📍 [三層狀態機] "
                        f"Base={base_state} | "
                        f"AGV={agv_state} | "
                        f"Robot={robot_state} | "
                        f"位置({position}), "
                        f"有路徑={has_path}, "
                        f"{mission_info}"
                    )
                except Exception as e:
                    self.get_logger().warn(f"⚠️ 狀態顯示異常: {e}")

            if self.count > 30:
                self.requesting = False  # 重置請求狀態
                self.count = 0  # 重置計數器
        except Exception as e:
            self.get_logger().error(f"❌ 主迴圈異常: {e}")

    def context_handle(self):
        try:
            # 處理當前的 AGV 狀態邏輯
            self.base_context.handle()
            pass
        except Exception as e:
            self.get_logger().error(f"未處理的 AGV 狀態機異常: {e}")

    def read_plc_data(self):  # 讀取AGV_PLC資料
        # 請求中不再請求
        if self.requesting:
            return
        # 計算時間差 到達循環時間200ms 才讀取PLC
        elapsed_time_ms = (self.clock.now() -
                           self.start_time).nanoseconds / 1e6
        if elapsed_time_ms < self.read_cycle_time_ms:
            return

        self.requesting = True
        self.start_time = self.clock.now()  # 使用 ROS 時鐘記錄時間
        start_address = "7600"
        self.plc_client.async_read_continuous_byte(
            device_type="DM",
            start_address="7600",
            count=200,
            callback=lambda res, sa=start_address: self.response_callback(
                res, sa),
        )
        # self.get_logger().info(f"📡 發送請求到 PLC: DM {start_address}，請求數量: 200")

    def publish_agv_status(self):
        try:
            msg = AgvStatusMsg()
            msg.agv_id = self.agv_status.AGV_ID or ""
            msg.slam_x = float(self.agv_status.AGV_SLAM_X) if self.agv_status.AGV_SLAM_X is not None else 0.0
            msg.slam_y = float(self.agv_status.AGV_SLAM_Y) if self.agv_status.AGV_SLAM_Y is not None else 0.0
            msg.slam_theta = float(self.agv_status.AGV_SLAM_THETA) if self.agv_status.AGV_SLAM_THETA is not None else 0.0
            msg.power = float(self.agv_status.POWER) if self.agv_status.POWER is not None else 0.0
            msg.x_speed = float(self.agv_status.AGV_X_SPEED) if self.agv_status.AGV_X_SPEED is not None else 0.0
            msg.y_speed = float(self.agv_status.AGV_Y_SPEED) if self.agv_status.AGV_Y_SPEED is not None else 0.0
            msg.theta_speed = float(self.agv_status.AGV_THETA_SPEED) if self.agv_status.AGV_THETA_SPEED is not None else 0.0
            msg.front_pgv = self.agv_status.AGV_FPGV or 0
            msg.back_pgv = self.agv_status.AGV_BPGV or 0
            msg.start_point = self.agv_status.AGV_START_POINT or 0
            msg.end_point = self.agv_status.AGV_END_POINT or 0
            msg.action = self.agv_status.AGV_ACTION or 0
            msg.zone = self.agv_status.AGV_ZONE or 0
            msg.status1 = self.agv_status.AGV_STATUS1 or 0
            msg.status2 = self.agv_status.AGV_STATUS2 or 0
            msg.status3 = self.agv_status.AGV_STATUS3 or 0
            msg.alarm1 = self.agv_status.AGV_ALARM1 or 0
            msg.alarm2 = self.agv_status.AGV_ALARM2 or 0
            msg.alarm3 = self.agv_status.AGV_ALARM3 or 0
            msg.alarm4 = self.agv_status.AGV_ALARM4 or 0
            msg.alarm5 = self.agv_status.AGV_ALARM5 or 0
            msg.alarm6 = self.agv_status.AGV_ALARM6 or 0
            msg.magic = self.agv_status.MAGIC or 0
            msg.layer = self.agv_status.AGV_LAYER or 0

            self._status_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing AGV_PLC data: {e}")
            pass
    # 讀取AGV_PLC資料回傳

    def response_callback(self, response, start_address):
        try:
            end_time = self.clock.now()  # 取得回應時間
            elapsed_time = (
                end_time - self.start_time
            ).nanoseconds / 1e6  # 轉換為毫秒 (ms)
            # self.get_logger().info(
            #    f"Service call duration: {elapsed_time:.3f} ms"
            # )  # 顯示執行時間
            if response.success:
                self.dMmemory.set_memory(int(start_address), response.values)
                self.agv_status.get_agv_status(self.dMmemory)
                self.agv_status.get_agv_bitstatus(self.dMmemory)
                self.agv_status.get_agv_door_open_status(self.dMmemory)
                self.agv_status.get_agv_inputs(self.dMmemory)
                self.agv_status.get_agv_outputs(self.dMmemory)
                self.agv_status.get_alarm_status(self.dMmemory)
                
                # 寫入完整狀態到 JSON 檔案
                self.write_status_to_file()

                # self.get_logger().info(f"Received string: {response.values}")
                # self.get_logger().info(f"Service call duration: {elapsed_time:.3f} ms")  # 顯示執行時間
            else:
                self.get_logger().error(
                    f"Failed to read data: {response.message}")
                pass
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")
        finally:
            self.requesting = False  # 重置請求狀態

    def toWriteStatus(self):
        """寫入AGV狀態到PLC (DM7800: heartbeat, DM7801: BaseState, DM7802: AGV state)"""
        # 心跳溢位保護：大於等於 50000 時重置為 0
        if self.plc_heartbeat >= 50000:
            self.plc_heartbeat = 0
        else:
            self.plc_heartbeat += 1

        # write_countinuous_byte
        if self.writing_status:  # 寫入狀態標誌為 True，則不執行寫入
            return

        # 根據當前狀態設定 BaseState
        if self.base_context.state.__class__.__name__ == "IdleState":
            self.BaseState = 1
        elif self.base_context.state.__class__.__name__ == "ManualState":
            self.BaseState = 2
        elif self.base_context.state.__class__.__name__ == "AutoState":
            self.BaseState = 3
        elif self.base_context.state.__class__.__name__ == "ErrorState":
            self.BaseState = 4

        # 偵測 AGV 層狀態並映射到數值
        agv_state = 0  # 預設值（未知狀態）
        if hasattr(self, 'agv_context') and self.agv_context and self.agv_context.state:
            agv_state_name = self.agv_context.state.__class__.__name__
            if agv_state_name == "MissionSelectState":
                agv_state = 1
            elif agv_state_name == "WritePathState":
                agv_state = 2
            elif agv_state_name == "RunningState":
                agv_state = 3
            elif agv_state_name == "WaitRobotState":
                agv_state = 4

        # 寫入資料到DM7800 (三個暫存器: heartbeat, BaseState, agv_state)
        valuedata = [str(self.plc_heartbeat), str(self.BaseState), str(agv_state)]
        try:
            self.plc_client.async_write_continuous_data(
                device_type='DM',
                start_address='7800',
                values=valuedata,
                callback=self.plc_write_status_callback
            )
            self.writing_status = True  # 設置寫入狀態標誌
            # self.get_logger().info("✅ AGV 狀態已寫入 PLC (DM7800)")
        except Exception as e:
            self.get_logger().error(f"寫入 PLC 狀態失敗: {str(e)}")
            return

    def plc_write_status_callback(self, response):
        self.writing_status = False  # 重置寫入狀態標誌
        if response.success:
            # self.get_logger().info("✅ 狀態寫入PLC成功")
            pass
        else:
            self.get_logger().warn("⚠️ 狀態寫入PLC失敗")
    
    def write_status_to_file(self):
        """寫入完整 AGV 狀態到 JSON 檔案，使用巢狀結構分類儲存"""
        try:
            # 取得 AGV ID 和類型
            agv_id = getattr(self.agv_status, 'AGV_ID', 'unknown')
            if not agv_id or agv_id == 'unknown':
                agv_id = self.get_namespace().strip('/')  # 從 namespace 取得
            
            # 判斷 AGV 類型（根據節點名稱或 ID）
            agv_type = 'Unknown'
            if 'loader' in agv_id.lower() or 'loader' in self.get_name().lower():
                agv_type = 'Loader'
            elif 'unloader' in agv_id.lower() or 'unloader' in self.get_name().lower():
                agv_type = 'Unloader'
            elif 'cargo' in agv_id.lower() or 'cargo' in self.get_name().lower():
                agv_type = 'Cargo Mover'
            
            # 建立巢狀狀態字典
            status_dict = {
                'metadata': {
                    'agv_id': agv_id,
                    'agv_type': agv_type,
                    'timestamp': self.clock.now().nanoseconds / 1e9,  # 轉換為秒
                    'version': '1.0',
                    'namespace': self.get_namespace(),
                    'node_name': self.get_name()
                },
                'agv_status': {},      # 基本狀態
                'contexts': {          # 三層狀態機狀態
                    'base_context': {
                        'current_state': 'UNKNOWN'
                    },
                    'agv_context': {
                        'current_state': 'UNKNOWN'
                    },
                    'robot_context': {
                        'current_state': 'UNKNOWN'
                    }
                },
                'type_specific': {},   # 車型特定資料
                'door_status': {},     # 門控狀態
                'io_data': {},         # IO 資料 (包含輸入和輸出)
                'alarms': {},          # 警報狀態
                'other': {}            # 其他未分類屬性
            }
            
            # 更新 context 狀態（如果存在）
            if hasattr(self, 'base_context') and self.base_context and hasattr(self.base_context, 'state'):
                if self.base_context.state:
                    status_dict['contexts']['base_context']['current_state'] = self.base_context.state.__class__.__name__
            
            # 嘗試取得其他 context（子類別可能有）
            if hasattr(self, 'loader_context'):
                if self.loader_context and hasattr(self.loader_context, 'state') and self.loader_context.state:
                    status_dict['contexts']['agv_context']['current_state'] = self.loader_context.state.__class__.__name__
            elif hasattr(self, 'unloader_context'):
                if self.unloader_context and hasattr(self.unloader_context, 'state') and self.unloader_context.state:
                    status_dict['contexts']['agv_context']['current_state'] = self.unloader_context.state.__class__.__name__
            elif hasattr(self, 'cargo_context'):
                if self.cargo_context and hasattr(self.cargo_context, 'state') and self.cargo_context.state:
                    status_dict['contexts']['agv_context']['current_state'] = self.cargo_context.state.__class__.__name__
                    
            if hasattr(self, 'robot_context'):
                if self.robot_context and hasattr(self.robot_context, 'state') and self.robot_context.state:
                    status_dict['contexts']['robot_context']['current_state'] = self.robot_context.state.__class__.__name__
            
            # 將 AgvStatus 物件的所有屬性根據分類存入對應類別
            for attr_name in dir(self.agv_status):
                # 排除私有屬性和方法
                if not attr_name.startswith('_') and not callable(getattr(self.agv_status, attr_name)):
                    value = getattr(self.agv_status, attr_name)
                    
                    # 處理特殊類型
                    if value is not None:
                        if not isinstance(value, (int, float, str, bool, list, dict)):
                            value = str(value)
                    
                    # 根據屬性名稱分類
                    if attr_name.startswith('DOOR_'):
                        # 門控狀態 (DOOR_OPEN_1, DOOR_CLOSE_1 等)
                        status_dict['door_status'][attr_name] = value
                    elif attr_name.startswith('AGV_INPUT_') or attr_name.startswith('IN_'):
                        # 輸入狀態 (AGV_INPUT_1_1, IN_1 等)
                        status_dict['io_data'][attr_name] = value
                    elif attr_name.startswith('AGV_OUTPUT_') or attr_name.startswith('OUT_'):
                        # 輸出狀態 (AGV_OUTPUT_1_1 等)
                        status_dict['io_data'][attr_name] = value
                    elif attr_name.startswith('ALARM_STATUS_'):
                        # 警報狀態 (ALARM_STATUS_1 到 ALARM_STATUS_100)
                        status_dict['alarms'][attr_name] = value
                    elif attr_name in ['AGV_Auto', 'AGV_MANUAL', 'AGV_IDLE', 'AGV_ALARM', 
                                      'AGV_MOVING']:
                        # 這些狀態要同時放在 agv_status 中（與測試腳本一致）
                        status_dict['agv_status'][attr_name] = value
                    elif attr_name in ['AGV_PATH', 'AGV_PATH_REQ', 'AGV_IN_MISSION',
                                      'AGV_LOCAL', 'AGV_LD_COMPLETE', 'AGV_UD_COMPLETE', 
                                      'LOW_POWER', 'MISSION_CANCEL', 'TRAFFIC_STOP', 
                                      'TRAFFIC_ALLOW', 'PS_RETRUN', 'AGV_2POSITION',
                                      'BARCODE_READER_FINISH', 'TAG_REQ']:
                        # 其他位元狀態
                        status_dict['other'][attr_name] = value
                    elif attr_name in ['AGV_ID', 'POWER', 'X_DIST', 'Y_DIST', 'THETA',
                                      'AGV_SLAM_X', 'AGV_SLAM_Y', 'AGV_SLAM_THETA',
                                      'AGV_X_SPEED', 'AGV_Y_SPEED', 'AGV_THETA_SPEED',
                                      'AGV_FPGV', 'AGV_BPGV', 'AGV_START_POINT', 'AGV_END_POINT',
                                      'AGV_ACTION', 'AGV_ZONE', 'AGV_STATUS1', 'AGV_STATUS2', 
                                      'AGV_STATUS3', 'AGV_ALARM1', 'AGV_ALARM2', 'AGV_ALARM3',
                                      'AGV_ALARM4', 'AGV_ALARM5', 'AGV_ALARM6', 'MAGIC', 'AGV_LAYER',
                                      'TASK_ID', 'TASK_ACTION', 'START_POINT', 'END_POINT', 
                                      'WORK_ID', 'KUKA_NODE_ID']:
                        # 基本 AGV 狀態 (來自 get_agv_status)
                        status_dict['agv_status'][attr_name] = value
                    else:
                        # 其他未分類的屬性
                        status_dict['other'][attr_name] = value
            
            # 添加車型特定資料
            if agv_type == 'Loader':
                status_dict['type_specific'] = {
                    'agv_ports': {
                        'port1': getattr(self.agv_status, 'AGV_INPUT_1_1', False) if hasattr(self.agv_status, 'AGV_INPUT_1_1') else False,
                        'port2': getattr(self.agv_status, 'AGV_INPUT_1_2', False) if hasattr(self.agv_status, 'AGV_INPUT_1_2') else False,
                        'port3': getattr(self.agv_status, 'AGV_INPUT_1_3', False) if hasattr(self.agv_status, 'AGV_INPUT_1_3') else False,
                        'port4': getattr(self.agv_status, 'AGV_INPUT_1_4', False) if hasattr(self.agv_status, 'AGV_INPUT_1_4') else False
                    },
                    'work_id': getattr(self.agv_status, 'WORK_ID', None),
                    'task_progress': None  # 可從 work_id 解析
                }
            elif agv_type == 'Unloader':
                status_dict['type_specific'] = {
                    'batch_processing': {
                        'batch_size': 2,  # 預設值
                        'current_batch': 0,
                        'total_batches': 0
                    },
                    'agv_carrier_status': {
                        'position_1': False,
                        'position_2': False
                    },
                    'station_status': {
                        'pre_dryer': [False] * 8,
                        'oven_upper': [False] * 4,
                        'oven_lower': [False] * 4
                    }
                }
            elif agv_type == 'Cargo Mover':
                status_dict['type_specific'] = {
                    'hokuyo_status': {
                        'hokuyo_1': 'connected',  # 可從實際狀態取得
                        'hokuyo_2': 'connected'
                    },
                    'rack_rotation': False,
                    'completed': False
                }
            
            # 移除空的分類
            for key in list(status_dict.keys()):
                if isinstance(status_dict[key], dict) and len(status_dict[key]) == 0:
                    del status_dict[key]
            
            # 寫入 JSON 檔案
            status_file = '/tmp/agv_status.json'
            with open(status_file, 'w', encoding='utf-8') as f:
                json.dump(status_dict, f, ensure_ascii=False, indent=2)
            
            # 偶爾記錄成功寫入（避免過多日誌）
            if self.count % 100 == 0:  # 每100次記錄一次
                self.get_logger().debug(f"✅ 完整 AGV 狀態已寫入 {status_file}")
                
        except Exception as e:
            self.get_logger().error(f"❌ 寫入狀態檔案失敗: {e}")

    # 共用方法
    def setup_common_parameters(self):
        """設置共用參數"""
        self.declare_parameter("room_id", 0)  # 預設房間ID為0
        self.room_id = self.get_parameter(
            "room_id").get_parameter_value().integer_value  # 取得room_id參數值
        self.get_logger().info(f"✅ 已接收 room_id: {self.room_id}")

    def common_state_changed(self, old_state, new_state):
        """共用的狀態變更日誌"""
        self.get_logger().info(
            f"狀態變更: {old_state.__class__.__name__} -> {new_state.__class__.__name__}")

    def _load_agvc_api_config(self):
        """從配置檔載入 AGVC API 設定"""
        config_path = '/app/config/agvc/base_config.yaml'
        default_config = {'ip': '192.168.10.3', 'port': 8000}

        try:
            if os.path.exists(config_path):
                with open(config_path, 'r') as f:
                    config = yaml.safe_load(f)
                network = config.get('base_agvc_parameters', {}).get('network', {})
                return {
                    'ip': network.get('agvc_server_ip', default_config['ip']),
                    'port': network.get('web_api_port', default_config['port'])
                }
        except Exception as e:
            self.get_logger().warn(f"⚠️ 載入 AGVC API 配置失敗: {e}，使用預設值")

        return default_config

    def _setup_tasks_api_polling(self):
        """建立 tasks Web API 輪詢 timer"""
        self.tasks_api_timer = self.create_timer(
            self.tasks_api_interval, self._fetch_tasks_from_api
        )
        self.get_logger().info(
            f"🌐 Tasks Web API 輪詢已建立\n"
            f"   - API URL: {self.agvc_api_base_url}/api/v1/task/\n"
            f"   - 輪詢間隔: {self.tasks_api_interval} 秒"
        )

    def _fetch_tasks_from_api(self):
        """從 AGVC Web API 取得任務列表

        只在 Base=AutoState 且 AGV=MissionSelectState 時才執行查詢
        """
        import time
        from agv_base.states.auto_state import AutoState
        from agv_base.agv_states.mission_select_state import MissionSelectState

        # 🔒 狀態檢查：只在特定狀態下才查詢 task
        # 檢查 Base 層是否為 AutoState
        if not isinstance(self.base_context.state, AutoState):
            return

        # 檢查 AGV 層是否為 MissionSelectState
        agv_context = None
        if hasattr(self, 'loader_context'):
            agv_context = self.loader_context
        elif hasattr(self, 'unloader_context'):
            agv_context = self.unloader_context
        elif hasattr(self, 'cargo_context'):
            agv_context = self.cargo_context

        if agv_context is None or not isinstance(agv_context.state, MissionSelectState):
            return

        try:
            # 取得當前 AGV 的 namespace 作為 agv_name
            agv_name = self.get_namespace().lstrip('/')

            # 呼叫 Web API
            url = f"{self.agvc_api_base_url}/api/v1/task/"
            params = {'agv_name': agv_name}  # 可選：按 agv_name 過濾

            response = requests.get(url, params=params, timeout=5.0)

            if response.status_code == 200:
                tasks_data = response.json()
                self._handle_api_tasks_response(tasks_data)
                self.last_tasks_fetch_time = time.time()
                self.last_tasks_callback_time = self.last_tasks_fetch_time  # 同步更新（向後兼容）

                # 每 5 秒輸出一次日誌
                current_time = time.time()
                if self.last_tasks_log_time is None or (current_time - self.last_tasks_log_time) >= 5.0:
                    self.last_tasks_log_time = current_time
                    self.get_logger().info(
                        f"📊 Tasks API: 取得 {len(self.latest_tasks)} 筆任務 (agv_name={agv_name})"
                    )
            else:
                self.get_logger().warn(
                    f"⚠️ Tasks API 請求失敗: HTTP {response.status_code}"
                )

        except requests.exceptions.Timeout:
            self.get_logger().warn("⚠️ Tasks API 請求逾時")
        except requests.exceptions.ConnectionError:
            self.get_logger().warn(
                f"⚠️ Tasks API 連接失敗: {self.agvc_api_base_url}"
            )
        except Exception as e:
            self.get_logger().error(f"❌ Tasks API 請求異常: {e}")

    def _handle_api_tasks_response(self, tasks_data):
        """處理 Web API 回傳的任務資料"""
        try:
            # 將 API 回傳格式轉換為內部格式
            # API 格式: id, parent_task_id, work_id, from_port, to_port, status_id,
            #           agv_name, priority, material_code, parameter, created_at, updated_at
            tasks = []
            for task_json in tasks_data:
                task_dict = {
                    'id': task_json.get('id', 0),
                    'parent_task_id': task_json.get('parent_task_id', 0),
                    'work_id': task_json.get('work_id', 0),
                    'from_port': task_json.get('from_port', 'na'),
                    'to_port': task_json.get('to_port', 'na'),
                    'status_id': task_json.get('status_id', 0),
                    'agv_name': task_json.get('agv_name', 'na'),
                    'priority': task_json.get('priority', 0),
                    'material_code': task_json.get('material_code', 'na'),
                    'parameter': task_json.get('parameter', {}),
                    'created_at': task_json.get('created_at', ''),
                    'updated_at': task_json.get('updated_at', '')
                }
                tasks.append(task_dict)

            # 更新全局任務列表
            self.latest_tasks = tasks

        except Exception as e:
            self.get_logger().error(f"❌ 處理 Tasks API 回應異常: {e}")
            import traceback
            self.get_logger().error(f"   詳細錯誤: {traceback.format_exc()}")

    def destroy_node(self):
        self.stop()
        self.plc_client.destroy()
        super().destroy_node()


def main():
    rclpy.init()
    node = AgvNodebase()

    # 使用 MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("🛑 偵測到 Ctrl+C，正在關閉節點...")
    finally:
        node.destroy_node()
        node.get_logger().info("🛑 節點已關閉，ROS 2 即將關閉。")
        executor.shutdown()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
