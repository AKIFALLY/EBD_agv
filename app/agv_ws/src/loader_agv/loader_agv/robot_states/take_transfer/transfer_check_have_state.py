from db_proxy.carrier_query_client import CarrierQueryClient
from rclpy.node import Node
from loader_agv.robot_context import RobotContext
from agv_base.hokuyo_dms_8bit import HokuyoDMS8Bit
from loader_agv.robot_states.base_robot_state import BaseRobotState
from shared_constants.equipment_stations import EquipmentStations


class TransferCheckHaveState(BaseRobotState):
    RobotContext.boxin_up_both_have = True

    # 8 BIT STEP
    IDLE = 0
    WRITE_VALID = 1
    WRITE_PORT_NUMBER = 2
    WAIT_UNLOAD_REQ = 3
    WRITE_TR_REQ = 4
    WAIT_READY = 5

    def __init__(self, node: Node):
        super().__init__(node)
        self.hokuyo_dms_8bit_1: HokuyoDMS8Bit = self.node.hokuyo_dms_8bit_1
        self.step = self.IDLE
        self.carrier_query_client = CarrierQueryClient(node)

        # 動態計算基礎地址
        self.base_port_id = self.node.room_id * 1000 + 10  # 例如: 2010

        # Station 相關變數
        self.current_station = None  # 當前 station (1 or 3)
        self.current_ports = []      # 當前 ports ([1, 2] or [3, 4])

        # 狀態標誌
        self.check_ok = False
        self.sent = False
        self.carrier_queried = False

        # Carrier 查詢結果
        self.carrier_ids = [None, None]  # 記錄 2 個 port 的 carrier_id
        self.carriers_data = []          # 完整的 carrier 資料

    def _reset_state(self):
        """重置所有狀態變數"""
        self.check_ok = False
        self.sent = False
        self.carrier_queried = False
        self.carrier_ids = [None, None]
        self.carriers_data = []
        self.current_station = None
        self.current_ports = []

    def enter(self):
        self.node.get_logger().info("Robot Take Transfer 目前狀態: TranferCheckHave")
        self._reset_state()

    def leave(self):
        self.node.get_logger().info("Robot Take Transfer 離開 TranferCheckHave 狀態")
        self._reset_state()

    def carrier_query_callback(self, response):
        """處理 carrier 查詢回應 - 記錄當前 station 2個port的carrier_id"""
        if not (response and response.success and response.datas):
            self.node.get_logger().error("❌ Carrier 查詢失敗或沒有資料")
            self.carrier_queried = True
            return

        # 保存完整資料
        self.carriers_data = response.datas

        # 計算當前 station 的實際 port_id
        actual_port_ids = [
            self.base_port_id + self.current_ports[0],  # 例如: 2011 或 2013
            self.base_port_id + self.current_ports[1]   # 例如: 2012 或 2014
        ]

        # 記錄這 2 個 port 的 carrier_id
        for carrier in response.datas:
            if carrier.port_id == actual_port_ids[0]:
                self.carrier_ids[0] = carrier.id
                self.node.get_logger().info(
                    f"✅ Port {self.current_ports[0]} carrier_id = {carrier.id}")
            elif carrier.port_id == actual_port_ids[1]:
                self.carrier_ids[1] = carrier.id
                self.node.get_logger().info(
                    f"✅ Port {self.current_ports[1]} carrier_id = {carrier.id}")

        self.carrier_queried = True
        self.node.get_logger().info(
            f"Carrier IDs: Port{self.current_ports[0]}={self.carrier_ids[0]}, "
            f"Port{self.current_ports[1]}={self.carrier_ids[1]}")

    def _handle_hokuyo_write(self, operation, value, success_flag, failed_flag, next_step):
        """處理 Hokuyo 寫入操作的通用方法"""
        if not self.sent:
            getattr(self.hokuyo_dms_8bit_1, operation)(value)
            self.sent = True

        if getattr(self.hokuyo_dms_8bit_1, success_flag):
            self.node.get_logger().info(f"✅{operation}寫入成功")
            setattr(self.hokuyo_dms_8bit_1, success_flag, False)
            self.sent = False
            self.step = next_step
        elif getattr(self.hokuyo_dms_8bit_1, failed_flag):
            self.node.get_logger().info(f"❌{operation}寫入失敗")
            setattr(self.hokuyo_dms_8bit_1, failed_flag, False)
            self.sent = False
        else:
            self.node.get_logger().info(f"⏳等待{operation}寫入")

    def handle(self, context: RobotContext):
        # 1. 首次執行：從 work_id 解析 station 和 ports
        # Work ID 格式（7位數 Station-based）：
        # - 2010101: Station 01 取入口箱（Port 1-2，2格批量）
        # - 2010301: Station 03 取入口箱（Port 3-4，2格批量）
        if self.current_station is None:
            # 驗證 Work ID 格式（必須是7位數）
            work_id_str = str(context.work_id)
            if len(work_id_str) != 7:
                self.node.get_logger().error(
                    f"❌ Work ID 格式錯誤: {context.work_id}，必須是7位數格式（REESSAA）")
                return

            station, ports = self._extract_station_and_ports_from_work_id(context.work_id)
            if station is None:
                self.node.get_logger().error(
                    f"❌ 無法從 work_id 解析 station: {context.work_id}")
                return

            self.current_station = station
            self.current_ports = ports
            self.node.get_logger().info(
                f"✅ [Station-based] Work ID {context.work_id} → Station {station:02d}, Ports {ports} (批量{len(ports)}格)")

        # 2. 查詢 Carrier（只執行一次）
        if not self.carrier_queried and not self.sent:
            # 計算當前 station 的 port_id 範圍
            # Equipment 201（入口傳送箱）格式：room_id * 1000 + 10 + port
            # 例如：room_id=2, Station 01 → Port 1-2 → port_id 2011-2012
            port_id_min = self.base_port_id + self.current_ports[0]
            port_id_max = self.base_port_id + self.current_ports[1]

            self.node.get_logger().info(
                f"查詢 Carrier: Station {self.current_station:02d}, "
                f"port_id 範圍 {port_id_min}-{port_id_max} (Equipment 201 入口傳送箱)")

            self.carrier_query_client.search_carrier_port_id(
                port_id_min=port_id_min,
                port_id_max=port_id_max,
                callback=self.carrier_query_callback
            )
            self.sent = True

        # 3. 更新 Hokuyo Input
        self._handle_hokuyo_input()

        # 4. 檢查：2個carrier_id都有值（標準設備批量處理）
        if self.carrier_queried and not self.check_ok:
            if self.carrier_ids[0] and self.carrier_ids[1]:
                self.node.get_logger().info(
                    f"✅ [批量檢查] Station {self.current_station:02d} 檢查成功: "
                    f"carrier_ids={self.carrier_ids} (2格批量)")

                # 設置 context（批量取料配置）
                # 第1次取料使用第1個 port
                context.boxin_number = self.current_ports[0]
                context.get_boxin_port = self.current_ports[0]
                context.carrier_id = self.carrier_ids[0]

                # 批量取料所需變數（新的 Station-based 設計）
                context.transfer_take_count = 0  # 批量計數器 (0=第1次, 1=第2次)
                context.transfer_carrier_ids = self.carrier_ids  # [carrier_id1, carrier_id2]
                context.transfer_ports = self.current_ports  # Station-based ports (例如: [1, 2] 或 [3, 4])

                self.node.get_logger().info(
                    f"[批量配置] Work ID {context.work_id}: "
                    f"Station {self.current_station:02d} → Ports {self.current_ports}, "
                    f"Carriers {self.carrier_ids}")

                self.check_ok = True
            else:
                self.node.get_logger().warn(
                    f"❌ [批量檢查] Station {self.current_station:02d} 檢查失敗: "
                    f"carrier_ids={self.carrier_ids}（需要2個carrier才能批量取料）")
                # 重置並等待下次檢查
                self._reset_state()
                return

        # 5. 執行 8-bit 步驟
        if self.check_ok:
            match self.step:
                case self.IDLE:
                    self.step = self.WRITE_VALID
                    self.sent = False
                case self.WRITE_VALID:
                    self._handle_hokuyo_write(
                        "write_valid", "1", "valid_success", "valid_failed", self.WRITE_PORT_NUMBER)
                case self.WRITE_PORT_NUMBER:
                    self._handle_hokuyo_write("write_port_number", context.boxin_number,
                                              "port_number_success", "port_number_failed", self.WAIT_UNLOAD_REQ)
                case self.WAIT_UNLOAD_REQ:
                    if self.hokuyo_dms_8bit_1.unload_req:
                        self.node.get_logger().info("✅收到unload_req")
                        self.step = self.WRITE_TR_REQ
                    else:
                        self.node.get_logger().info("⏳等待unload_req")
                case self.WRITE_TR_REQ:
                    self._handle_hokuyo_write(
                        "write_tr_req", "1", "tr_req_success", "tr_req_failed", self.WAIT_READY)
                case self.WAIT_READY:
                    if self.hokuyo_dms_8bit_1.ready:
                        self.node.get_logger().info("✅收到ready")
                        self.step = self.IDLE
                        from loader_agv.robot_states.take_transfer.agv_port_check_empty_state import AgvPortCheckEmptyState
                        context.set_state(AgvPortCheckEmptyState(self.node))
                    else:
                        self.node.get_logger().info("⏳等待ready")
