from agv_base.states.state import State
from rclpy.node import Node
from astar_algorithm.astar_algorithm import AStarAlgorithm
import requests


class WritePathState(State):
    def __init__(self, node: Node):
        super().__init__(node)
        self.plc_client = node.plc_client  # 引用 node 的 plc_client
        self.StationID = None  # 站點ID
        self.TagNo = None  # TAG No
        self.path = []  # 路徑資料
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
        self.path_calculated = False  # 路徑是否已計算完成並準備好 dataValue
        self.status_updated = False  # 狀態是否已更新（確保只更新一次）

    def enter(self):
        self.node.get_logger().info("AGV 進入: WritePathState 狀態")

    def leave(self):
        self.node.get_logger().info("AGV 離開 WritePathState 狀態")

    def handle(self, context):
        # 檢查寫入次數是否超過5次
        if self.count > 5:
            self.node.get_logger().error("❌ 路徑資料寫入失敗過多，寫入異常到PLC")
            self.plc_client.async_force_on('MR', '3204', self.force_callback)  # PLC寫入異常
            context.set_state(context.MissionSelectState(self.node))  # 切換狀態

        # 條件1: 檢查是否已經有路徑資料且 LAYER 已確認
        # 如果已經有路徑資料且 AGV_LAYER > 0，則更新狀態並切換到下一個狀態
        if self.node.agv_status.AGV_PATH and self.node.agv_status.AGV_LAYER > 0:
            # 在確認 AGV_PATH=1 && AGV_LAYER>0 時才更新任務狀態（只更新一次）
            if not self.status_updated:
                self._update_task_status_on_path_confirmed(context)
                self.status_updated = True

            self.node.get_logger().info(
                f"AGV 已有路徑資料且 LAYER={self.node.agv_status.AGV_LAYER}，"
                f"離開 WritePathState-->RunningState"
            )
            # 跳過寫入路徑狀態，直接切換到下一個狀態
            context.set_state(context.RunningState(self.node))  # 切換狀態
            return

        # 條件2: LOCAL 模式跳轉 (LOCAL=1 && MAGIC>0 && AGV_PATH=1)
        # 當 AGV 處於 LOCAL 模式且有有效 MAGIC 值時，可直接跳轉到 RunningState
        if (self.node.agv_status.AGV_LOCAL == 1 and
                self.node.agv_status.MAGIC > 0 and
                self.node.agv_status.AGV_PATH):
            self.node.get_logger().info(
                f"LOCAL 模式跳轉: LOCAL={self.node.agv_status.AGV_LOCAL}, "
                f"MAGIC={self.node.agv_status.MAGIC}, AGV_PATH={self.node.agv_status.AGV_PATH}，"
                f"離開 WritePathState-->RunningState"
            )
            context.set_state(context.RunningState(self.node))  # 切換狀態
            return

        # 條件3: LOCAL 模式下無路徑且無終點 → 跳回 MissionSelectState
        # 當 LOCAL=1, PATH=0, END_POINT=0 時，無法計算路徑，返回等待
        end_point = self.node.agv_status.AGV_END_POINT if self.node.agv_status.AGV_END_POINT is not None else 0
        if (self.node.agv_status.AGV_LOCAL == 1 and
                not self.node.agv_status.AGV_PATH and
                end_point == 0):
            self.node.get_logger().warn(
                f"⚠️ LOCAL 模式無終點: LOCAL=1, PATH=0, END_POINT=0，"
                f"跳回 MissionSelectState 等待終點設定"
            )
            context.set_state(context.MissionSelectState(self.node))  # 切換狀態
            return

        if self.step >= 3:
            self.step += 1  # 增加步驟計數器
            if self.step >= 100:
                self.step = 0  # 重置步驟計數器

        # 檢查是否已經有路徑資料
        # 如果沒有路徑資料，則計算路徑並寫入PLC
        # self.node.get_logger().info(f"✅ 準備計算路徑, 執行次數: {self.count}, 當前步驟: {self.step},路徑:{self.node.agv_status.AGV_PATH}")
        if not self.node.agv_status.AGV_PATH and self.step == 0 and not self.path_calculated:
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
                        if y:  # 最後一個點
                            # 取得 status_id（支援 dict 格式）
                            task_status_id = self.node.task.get('status_id', 0) if isinstance(self.node.task, dict) else getattr(self.node.task, 'status_id', 0)
                            if self.node.agv_status.MAGIC == 21 or task_status_id == 21:
                                self.dataValue[i*20+2] = 21  # MAGIC=21 或 status_id=21 特殊處理：最後一個點直接給21
                                reason = "MAGIC=21" if self.node.agv_status.MAGIC == 21 else "status_id=21"
                                self.node.get_logger().info(f"✅ {reason} 特殊模式：最後一個點設定 dataValue[{i*20+2}] = 21")
                            else:
                                self.dataValue[i*20+2] = tag.get('Station')+20  # 正常情況：Station_Index=2
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

            # ⚠️ 路徑計算和 dataValue 準備完成
            self.path_calculated = True

            # 寫入 layer 到 DM7645（在路徑資料之前）
            layer_value = getattr(self.node, 'task_layer', 0)
            self.plc_client.async_write_data(
                device_type='DM',
                address='7645',
                value=str(layer_value),
                callback=self._write_layer_callback
            )
            self.node.get_logger().info(f"📤 寫入 LAYER={layer_value} 到 DM7645")

            # 將路徑資料寫入PLC
            self.plc_client.async_write_continuous_data(
                'DM', '3000', string_values_1, self.write_path_callback)  # PLC寫入路徑
            self.plc_client.async_write_continuous_data(
                'DM', '4000', string_values_2, self.write_path_callback)

            self.count += 1  # 增加計數器
            self.node.get_logger().info(f"✅ PLC 路徑資料寫入, 執行次數: {self.count}")
            self.step = 1  # 增加步驟計數器
            # 做完延遲兩

    def _update_task_status_on_path_confirmed(self, context):
        """當 AGV_PATH=1 且 AGV_LAYER>0 時更新任務狀態

        條件：
        - MAGIC != 21（特殊模式不更新）
        - 非執行中狀態（2,4,12,14,22）才更新
        - 狀態更新為 current_status + 1
        """
        # MAGIC=21 特殊處理：不更改 task status
        if self.node.agv_status.MAGIC == 21:
            self.node.get_logger().info("🎯 MAGIC=21 特殊模式：跳過任務狀態更新，維持原始狀態")
            return

        # 取得 task_id 和當前 status_id（支援 dict 格式）
        task_id = self.node.task.get('id') if isinstance(self.node.task, dict) else getattr(self.node.task, 'id', 0)
        current_status = self.node.task.get('status_id') if isinstance(self.node.task, dict) else getattr(self.node.task, 'status_id', 0)

        # 檢查是否為執行中狀態（2,4,12,14,22）
        from shared_constants.task_status import TaskStatus
        if TaskStatus.is_task_executing_status(current_status):
            # 執行中狀態：跳過狀態更新，僅重算路徑
            self.node.get_logger().info(
                f"🔄 執行中狀態 (status={current_status})：跳過狀態更新，僅重算路徑"
            )
            return

        # 開始狀態：正常更新 status+1
        # 1→2, 11→12, 13→14, 21→22, 3→4
        next_status = current_status + 1

        self.node.get_logger().info(
            f"📤 路徑確認完成 (AGV_PATH=1, LAYER={self.node.agv_status.AGV_LAYER})，更新任務狀態 {current_status} → {next_status}"
        )

        # 透過 Web API 更新任務狀態
        update_success = self._update_task_status_via_api(task_id, status_id=next_status)

        if not update_success:
            self.node.get_logger().error("❌ 任務狀態更新失敗")
            return

        # 更新本地任務狀態
        if isinstance(self.node.task, dict):
            self.node.task['status_id'] = next_status
        else:
            self.node.task.status_id = next_status

    def _update_task_status_via_api(self, task_id: int, status_id: int) -> bool:
        """透過 Web API 更新任務狀態

        API: PUT /api/v1/task/{task_id}/status
        Body: {"status_id": <status_id>}

        Args:
            task_id: 任務 ID
            status_id: 新的狀態 ID

        Returns:
            bool: 更新成功返回 True，失敗返回 False
        """
        try:
            url = f"{self.node.agvc_api_base_url}/api/v1/task/{task_id}/status"
            payload = {"status_id": status_id}

            self.node.get_logger().info(f"⏳ 更新任務狀態: task_id={task_id}, status_id={status_id}")

            response = requests.put(url, json=payload, timeout=5.0)

            if response.status_code == 200:
                self.node.get_logger().info(f"✅ 任務狀態更新成功: task_id={task_id} → status_id={status_id}")
                return True
            else:
                self.node.get_logger().error(
                    f"❌ 任務狀態更新失敗: HTTP {response.status_code}, {response.text}"
                )
                return False

        except requests.exceptions.Timeout:
            self.node.get_logger().error(f"❌ 任務狀態更新逾時: task_id={task_id}")
            return False
        except requests.exceptions.ConnectionError:
            self.node.get_logger().error(
                f"❌ 無法連接 AGVC API: {self.node.agvc_api_base_url}"
            )
            return False
        except Exception as e:
            self.node.get_logger().error(f"❌ 任務狀態更新異常: {e}")
            return False

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

            # ⚠️ 重置路徑計算標記，為下一次路徑計算做準備
            if self.path_calculated:
                self.path_calculated = False
                self.node.get_logger().info("🔄 路徑計算標記已重置")
        else:
            self.node.get_logger().warn("⚠️ PLC 路徑資料寫入失敗")

    def force_callback(self, response):
        if response.success:
            self.node.get_logger().info("✅ PLC force寫入成功")
        else:
            self.node.get_logger().warn("⚠️ PLC force寫入失敗")

    def _write_layer_callback(self, response):
        """Layer 寫入回調"""
        if response.success:
            self.node.get_logger().info("✅ DM7645 LAYER 寫入成功")
        else:
            self.node.get_logger().warn(f"⚠️ DM7645 LAYER 寫入失敗: {response.message}")


"""
[{'TagNo': 2, 'Tag_X': 17510, 'Tag_Y': 11200, 'Station': 5, 'CanToMoveSet': [{'CanToMoveTag': 21, 'PGV': 0, '加權': 0, 'Act': [12, 12, 12], 'Speed': [], 'SHIFT': [0, 0, -904], 'Inposition': [], 'SafeSensorSetting': []}, 
{'CanToMoveTag': 0, 'PGV': 0, '加權': 0, 'Act': [12, 12, 12], 'Speed': [], 'SHIFT': [0, 0, 0], 'Inposition': [], 'SafeSensorSetting': []}, 
{'CanToMoveTag': 0, 'PGV': 0, '加權': 0, 'Act': [12, 12, 12], 'Speed': [], 'SHIFT': [0, 0, 0], 'Inposition': [], 'SafeSensorSetting': []}, 
{'CanToMoveTag': 0, 'PGV': 0, '加權': 0, 'Act': [12, 12, 12], 'Speed': [], 'SHIFT': [0, 0, 0], 'Inposition': [], 'SafeSensorSetting': []}, 
{'CanToMoveTag': 0, 'PGV': 0, '加權': 0, 'Act': [0, 12, 12], 'Speed': [], 'SHIFT': [0, 0, 0], 'Inposition': [], 'SafeSensorSetting': []}]},
]
"""
