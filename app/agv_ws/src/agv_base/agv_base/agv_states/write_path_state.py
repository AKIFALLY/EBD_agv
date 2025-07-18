from agv_base.states.state import State
from rclpy.node import Node
from astar_algorithm.astar_algorithm import AStarAlgorithm
from db_proxy_interfaces.msg import Task as TaskMsg
from db_proxy.agvc_database_client import AGVCDatabaseClient
from plc_proxy.plc_client import PlcClient
import time


class WritePathState(State):
    def __init__(self, node: Node):
        super().__init__(node)
        self.StationID = None  # 站點ID
        self.TagNo = None  # TAG No
        self.path = []  # 路徑資料
        self.plc_client = PlcClient(node)
        self.agvdbclient = AGVCDatabaseClient(node)
        self.source_data = None  # 初始點位資料
        self.cantomove_tag = None  # 可移動標籤
        self.act = []  # 動作
        self.pgv = 0  # PGV
        self.speed = []  # 速度
        self.shift = []  # 偏移
        self.inposition = []  # 進位
        self.safe_sensor_setting = []  # 安全感測器設定
        # 2000長度的list，初始值為0
        self.dataValue = [0] * 2000
        self.count = 0  # 計數器，用於執行次數
        self.step = 0  # 步驟計數器

    def enter(self):
        self.node.get_logger().info("AGV 進入: WritePathState 狀態")

    def leave(self):
        self.node.get_logger().info("AGV 離開 WritePathState 狀態")

    def handle(self, context):

        # self.node.get_logger().info("AGV WritePathState 狀態")
        # 檢查寫入次數是否超過5次
        if self.count > 5:
            self.node.get_logger().error("❌ 路徑資料寫入失敗過多，寫入異常到PLC")
            self.plc_client.async_force_on('MR', '3204', self.force_callback)  # PLC寫入異常
            from agv_base.agv_states.mission_select_state import MissionSelectState
            context.set_state(MissionSelectState(self.node))  # 切換狀態

        # 檢查是否已經有路徑資料
        # 如果已經有路徑資料，則直接切換到下一個狀態
        if self.node.agv_status.AGV_PATH:
            self.node.get_logger().info("AGV 已有路徑資料，離開 WritePathState 狀態")
            # 跳過寫入路徑狀態，直接切換到下一個狀態
            from agv_base.agv_states.Running_state import RunningState
            context.set_state(RunningState(self.node))  # 切換狀態

        if self.step >= 3:
            self.step += 1  # 增加步驟計數器
            if self.step >= 100:
                self.step = 0  # 重置步驟計數器

        # 檢查是否已經有路徑資料
        # 如果沒有路徑資料，則計算路徑並寫入PLC
        # self.node.get_logger().info(f"✅ 準備計算路徑, 執行次數: {self.count}, 當前步驟: {self.step},路徑:{self.node.agv_status.AGV_PATH}")
        if not self.node.agv_status.AGV_PATH and self.step == 0:
            # self.node.get_logger().info("AGV WritePathState 狀態")
            # 將站點ID轉換成TAG No
            self.StationID = "Washing"
            self.TagNo = self.node.agv_status.AGV_FPGV

            try:
                self.As = AStarAlgorithm(self.TagNo, self.node.node_id)
                self.node.get_logger().info(
                    f"✅ A*演算法初始化成功, 現在位置: {self.TagNo} ,目標節點: {self.node.node_id}")
                self.path = self.As.run()  # 執行A*演算法計算路徑
                self.node.pathdata = self.path  # 將路徑傳到外面
                self.node.get_logger().info(f"✅ 計算路徑成功: {self.path}")
            except Exception as e:
                self.node.get_logger().error(
                    f"❌ 計算路徑失敗- 現在位置: {self.TagNo} ,目標節點: {self.node.node_id}")

                self.count += 1  # 增加計數器

            self.source_data = self.As.source_data

            # 將路徑資料寫入PLC
            for i in range(len(self.path)):

                x = 0
                y = False
                if len(self.path)-1 == i:
                    x = self.path[i]
                    y = True
                else:
                    x = self.path[i+1]
                    y = False

                for tag in self.source_data:

                    if tag.get('TagNo') == x:
                        cantomove_tag = tag.get('CanToMoveSet')
                        for j in range(len(cantomove_tag)):
                            if cantomove_tag[j].get('CanToMoveTag') == self.path[i]:
                                self.cantomove_tag = cantomove_tag[j].get('CanToMoveTag')
                                self.pgv = cantomove_tag[j].get('PGV')
                                self.act = cantomove_tag[j].get('Act')
                                self.speed = cantomove_tag[j].get('Speed')
                                self.shift = cantomove_tag[j].get('SHIFT')
                                self.inposition = cantomove_tag[j].get('Inposition')
                                self.safe_sensor_setting = cantomove_tag[j].get('SafeSensorSetting')

                                # 將 TagNo, Tag_X, Tag_Y 寫入 dataValue
                                # 假設每個 tag 有 'TagNo', 'Tag_X', 'Tag_Y' 等屬性
                                # 並且每個 tag 的索引是 i*20 (20 是每個 tag 的資料長度)

                        if y:
                            self.dataValue[i*20] = tag.get('TagNo')  # Tag No_Index=0
                        else:
                            self.dataValue[i*20] = self.cantomove_tag  # Tag No_Index=0

                        # 如果是最後一個點，則使用站點ID，否則使用act[0]
                        if y:
                            self.dataValue[i*20+2] = tag.get('Station')+20  # Station_Index=2
                            break  # 跳出迴圈
                        else:
                            if len(self.act) >= 1:
                                self.dataValue[i*20+2] = self.act[0]  # ACT_Index=2

                        self.dataValue[i*20+4], self.dataValue[i*20 +
                                                               # Tag_X_Index=4
                                                               5] = self.split_32_to_16(tag.get('Tag_X'))
                        self.dataValue[i*20+9], self.dataValue[i*20 +
                                                               # Tag_Y_Index=9
                                                               10] = self.split_32_to_16(tag.get('Tag_Y'))
                        self.dataValue[i*20+1] = self.pgv  # PGV_Index=1

                        self.dataValue[i*20+7] = 12  # ACT_Index=7
                        self.dataValue[i*20+12] = 12

                        if len(self.safe_sensor_setting) >= 1:
                            # SafeSensorSetting_Index=6
                            self.dataValue[i*20+6] = self.safe_sensor_setting[0]
                        if len(self.safe_sensor_setting) >= 2:
                            self.dataValue[i*20+11] = self.safe_sensor_setting[1]
                        if len(self.safe_sensor_setting) >= 3:
                            self.dataValue[i*20+16] = self.safe_sensor_setting[2]

                        if len(self.speed) >= 1:
                            self.dataValue[i*20+3] = self.speed[0]  # Speed_Index=3
                        if len(self.speed) >= 2:
                            self.dataValue[i*20+8] = self.speed[1]
                        if len(self.speed) >= 3:
                            self.dataValue[i*20+13] = self.speed[2]

                        if len(self.shift) >= 3:
                            self.dataValue[i*20+14], self.dataValue[i*20 +
                                                                    # 旋轉角度
                                                                    15] = self.split_32_to_16(self.shift[2])
                        break

            string_values = [str(v) for v in self.dataValue]
            string_values_1 = string_values[:1000]    # 前 1000 筆
            string_values_2 = string_values[1000:2000]  # 後 1000 筆

            # 更新tasks table的狀態

            self.node.task.status_id = 2  # 更新狀態為執行中
            self.node.task.agv_id = self.node.AGV_id  # 更新AGV ID
            self.agvdbclient.async_update_task(
                self.node.task, self.task_update_callback)  # 更新任務狀態為執行中

            # 將路徑資料寫入PLC
            self.plc_client.async_write_continuous_data(
                'DM', '3000', string_values_1, self.write_path_callback)  # PLC寫入路徑
            self.plc_client.async_write_continuous_data(
                'DM', '4000', string_values_2, self.write_path_callback)

            self.count += 1  # 增加計數器
            self.node.get_logger().info(f"✅ PLC 路徑資料寫入, 執行次數: {self.count}")
            self.step = 1  # 增加步驟計數器
            # 做完延遲兩

    def task_update_callback(self, response):
        if response is None:
            print("❌ 未收到任務更新的回應（可能逾時或錯誤）")
            return

        if response.success:
            print(f"✅ 任務更新成功，訊息: {response.message}")
        else:
            print(f"⚠️ 任務更新失敗，訊息: {response.message}")

    # 將 32 位元整數分割成兩個 16 位元整數

    def split_32_to_16(self, value):
        # 確保是無符號 32 位元整數（如有需要）
        value &= 0xFFFFFFFF
        low = value & 0xFFFF          # 取低 16 位元
        high = (value >> 16) & 0xFFFF  # 取高 16 位元
        return low, high

    def write_path_callback(self, response):
        if response.success:
            self.node.get_logger().info("✅ PLC 路徑資料寫入成功")
            self.step += 1  # 增加步驟計數器
        else:
            self.node.get_logger().warn("⚠️ PLC 路徑資料寫入失敗")

    def force_callback(self, response):
        if response.success:
            self.node.get_logger().info("✅ PLC force寫入成功")
        else:
            self.node.get_logger().warn("⚠️ PLC force寫入失敗")


"""
[{'TagNo': 2, 'Tag_X': 17510, 'Tag_Y': 11200, 'Station': 5, 'CanToMoveSet': [{'CanToMoveTag': 21, 'PGV': 0, '加權': 0, 'Act': [12, 12, 12], 'Speed': [], 'SHIFT': [0, 0, -904], 'Inposition': [], 'SafeSensorSetting': []}, 
{'CanToMoveTag': 0, 'PGV': 0, '加權': 0, 'Act': [12, 12, 12], 'Speed': [], 'SHIFT': [0, 0, 0], 'Inposition': [], 'SafeSensorSetting': []}, 
{'CanToMoveTag': 0, 'PGV': 0, '加權': 0, 'Act': [12, 12, 12], 'Speed': [], 'SHIFT': [0, 0, 0], 'Inposition': [], 'SafeSensorSetting': []}, 
{'CanToMoveTag': 0, 'PGV': 0, '加權': 0, 'Act': [12, 12, 12], 'Speed': [], 'SHIFT': [0, 0, 0], 'Inposition': [], 'SafeSensorSetting': []}, 
{'CanToMoveTag': 0, 'PGV': 0, '加權': 0, 'Act': [0, 12, 12], 'Speed': [], 'SHIFT': [0, 0, 0], 'Inposition': [], 'SafeSensorSetting': []}]},
]
"""
