from db_proxy.carrier_query_client import CarrierQueryClient
from db_proxy.eqp_signal_query_client import EqpSignalQueryClient
from rclpy.node import Node
from loader_agv.robot_context import RobotContext
from agv_base.hokuyo_dms_8bit import HokuyoDMS8Bit
from loader_agv.robot_states.base_robot_state import BaseRobotState
from shared_constants.equipment_stations import EquipmentStations


class CleanerCheckHaveState(BaseRobotState):

    # 8 BIT STEP
    IDLE = 0
    WRITE_VALID = 1
    WRITE_PORT_NUMBER = 2
    WAIT_LOAD_REQ = 3
    WRITE_TR_REQ = 4
    WAIT_READY = 5

    def __init__(self, node: Node):
        super().__init__(node)
        self.node = node
        self.hokuyo_dms_8bit_1: HokuyoDMS8Bit = self.node.hokuyo_dms_8bit_1
        self.eqp_signal_query_client = EqpSignalQueryClient(node)
        self.carrier_query_client = CarrierQueryClient(node)

        # 動態計算基礎地址
        self.base_port_id = self.node.room_id * 1000 + 10  # 例如: 2010

        # Station 相關變數
        self.current_station = None  # 當前 station (1 for TAKE Cleaner)
        self.current_ports = []      # 當前 ports ([1, 2] for Station 01)

        # 狀態標誌
        self.check_ok = False
        self.sent = False
        self.search_eqp_signal_ok = False
        self.carrier_queried = False

        # Carrier 查詢結果
        self.carrier_ids = [None, None]  # 記錄 2 個 port 的 carrier_id
        self.carriers_data = []          # 完整的 carrier 資料

    def _reset_state(self):
        """重置所有狀態變數"""
        self.check_ok = False
        self.step = self.IDLE
        self.sent = False
        self.search_eqp_signal_ok = False
        self.carrier_queried = False
        self.carrier_ids = [None, None]
        self.carriers_data = []
        self.current_station = None
        self.current_ports = []

    def enter(self):
        self.node.get_logger().info(
            "[Station-based 固定方向] Loader Robot Take Cleaner 目前狀態: CleanerCheckHave")
        self._reset_state()

    def leave(self):
        self.node.get_logger().info(
            "[Station-based 固定方向] Loader Robot Take Cleaner 離開 CleanerCheckHave 狀態")
        self._reset_state()

    def eqp_signal_query_callback(self, response):
        """處理 EqpSignal 查詢回應 - 檢查當前 station 的 2 個 port 是否有貨"""
        if not response or not response.datas:
            self.node.get_logger().error("❌ EqpSignal 查詢失敗或沒有資料")
            self.search_eqp_signal_ok = True
            return

        # 計算當前 station 的實際 port_id
        actual_port_ids = [
            self.base_port_id + 20 + self.current_ports[0],  # 例如: 2031 (base_port_id=2010, +20=2030, +1=2031)
            self.base_port_id + 20 + self.current_ports[1]   # 例如: 2032
        ]

        # 檢查這 2 個 port 是否有貨
        port_has_cargo = [False, False]
        for eqp_signal in response.datas:
            if eqp_signal.port_id == actual_port_ids[0]:
                port_has_cargo[0] = eqp_signal.have_boxin
            elif eqp_signal.port_id == actual_port_ids[1]:
                port_has_cargo[1] = eqp_signal.have_boxin

        self.node.get_logger().info(
            f"[Station-based] EqpSignal 檢查: Port {self.current_ports[0]}={port_has_cargo[0]}, "
            f"Port {self.current_ports[1]}={port_has_cargo[1]}")

        # TAKE 操作：檢查是否至少有一個 port 有貨（2格批量需要都有貨）
        if not (port_has_cargo[0] or port_has_cargo[1]):
            self.node.get_logger().warn(
                f"⚠️ [Station-based 批量] Station {self.current_station:02d} 沒有貨物可取")

        self.search_eqp_signal_ok = True

    def carrier_query_callback(self, response):
        """處理 carrier 查詢回應 - 記錄當前 station 2個port的carrier_id

        說明：TAKE Cleaner 批量操作需要檢查2個port都有 carrier
        - 固定方向設計：上層(Station 01)只能 TAKE，下層(Station 03)只能 PUT
        - 批量處理：Cleaner Port 1 → AGV Port 1, Cleaner Port 2 → AGV Port 3
        """
        if not (response and response.success and response.datas):
            self.node.get_logger().error("❌ Carrier 查詢失敗或沒有資料")
            self.carrier_queried = True
            return

        # 保存完整資料
        self.carriers_data = response.datas

        # 計算當前 station 的實際 port_id
        actual_port_ids = [
            self.base_port_id + 20 + self.current_ports[0],  # 例如: 2031
            self.base_port_id + 20 + self.current_ports[1]   # 例如: 2032
        ]

        # 記錄這 2 個 port 的 carrier_id
        for carrier in response.datas:
            if carrier.port_id == actual_port_ids[0]:
                self.carrier_ids[0] = carrier.id
                self.node.get_logger().info(
                    f"✅ [Station-based] Cleaner Port {self.current_ports[0]} carrier_id = {carrier.id}")
            elif carrier.port_id == actual_port_ids[1]:
                self.carrier_ids[1] = carrier.id
                self.node.get_logger().info(
                    f"✅ [Station-based] Cleaner Port {self.current_ports[1]} carrier_id = {carrier.id}")

        self.carrier_queried = True
        self.node.get_logger().info(
            f"[Station-based 批量] Carrier IDs: Port{self.current_ports[0]}={self.carrier_ids[0]}, "
            f"Port{self.current_ports[1]}={self.carrier_ids[1]}")

    def _extract_station_and_ports_from_work_id(self, work_id: int):
        """從 work_id 解析 station 和 ports"""
        try:
            room_id, eqp_id, ports, action_type = EquipmentStations.work_id_to_ports(
                work_id, agv_type="loader")

            # 從 ports 推導 station
            if ports == [1, 2]:
                station = 1
            elif ports == [3, 4]:
                station = 3
            else:
                self.node.get_logger().error(f"❌ 無效的 ports: {ports}")
                return None, None

            self.node.get_logger().info(
                f"Work ID {work_id} 解析: room_id={room_id}, eqp_id={eqp_id}, "
                f"station={station}, ports={ports}, action_type={action_type}")

            return station, ports
        except Exception as e:
            self.node.get_logger().error(f"❌ Work ID {work_id} 解析失敗: {e}")
            return None, None

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
        # Work ID 格式（7位數 Station-based，固定方向設計）：
        # - 2030101: Station 01 取清洗機（Port 1-2，上層，只能 TAKE，2格批量）
        # 清洗機固定方向：下層進料（PUT）→ 清洗制程 → 上層出料（TAKE）
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
                f"✅ [Station-based 固定方向] Work ID {context.work_id} → "
                f"Station {station:02d} (上層), Ports {ports} (批量{len(ports)}格/只 TAKE)")

        # 2. 查詢 EqpSignal（只執行一次）- 用於雙重驗證
        if not self.search_eqp_signal_ok and not self.sent:
            # 計算當前 station 的 eqp_id (例如: 203)
            eqp_id = self.node.room_id * 100 + 3
            self.node.get_logger().info(
                f"[Station-based] 查詢 EqpSignal: eqp_id={eqp_id}, Station {self.current_station:02d}")

            self.eqp_signal_query_client.search_eqp_signal_eqp_id(
                eqp_id, self.eqp_signal_query_callback)
            self.sent = True

        # 3. 查詢 Carrier（只執行一次）
        if self.search_eqp_signal_ok and not self.carrier_queried and not self.sent:
            # 計算當前 station 的 port_id 範圍
            port_id_min = self.base_port_id + 20 + self.current_ports[0]
            port_id_max = self.base_port_id + 20 + self.current_ports[1]

            self.node.get_logger().info(
                f"[Station-based 批量] 查詢 Carrier: port_id 範圍 {port_id_min}-{port_id_max} "
                f"(Station {self.current_station:02d}, Ports {self.current_ports})")

            self.carrier_query_client.search_carrier_port_id(
                port_id_min=port_id_min,
                port_id_max=port_id_max,
                callback=self.carrier_query_callback
            )
            self.sent = True

        # 4. 更新 Hokuyo Input
        self._handle_hokuyo_input()

        # 5. 檢查：2個carrier_id都有值（TAKE 操作必須有貨）
        if self.carrier_queried and not self.check_ok:
            if self.carrier_ids[0] and self.carrier_ids[1]:
                self.node.get_logger().info(
                    f"✅ [Station-based 批量] Station {self.current_station:02d} 檢查成功: "
                    f"carrier_ids={self.carrier_ids} (都有貨物)")

                # 批量取料配置（2格批量操作）
                # Station-based 設計：
                # - Station 01 包含 2 個 port (Port 1-2)
                # - 批量處理：Cleaner Port 1 → AGV Port 1, Cleaner Port 2 → AGV Port 3
                context.cleaner_take_count = 0  # 計數器 (0=第1次, 1=第2次)
                context.cleaner_carrier_ids = self.carrier_ids  # [carrier_id1, carrier_id2]
                context.cleaner_ports = self.current_ports  # [1, 2] for Station 01
                context.get_cleaner_port = self.current_ports[0]  # 第1次使用 port 1
                context.carrier_id = self.carrier_ids[0]

                self.node.get_logger().info(
                    f"[Station-based 批量] 配置完成: Work ID {context.work_id} → "
                    f"批量取料 2/2 次, Ports {context.cleaner_ports}")
                self.node.get_logger().info(
                    f"第 1 次: Cleaner Port {context.cleaner_ports[0]} → AGV Port 1")
                self.node.get_logger().info(
                    f"第 2 次: Cleaner Port {context.cleaner_ports[1]} → AGV Port 3")

                self.check_ok = True
            else:
                self.node.get_logger().warn(
                    f"❌ [Station-based 批量] Station {self.current_station:02d} 檢查失敗: "
                    f"carrier_ids={self.carrier_ids} (未全部有貨)")
                # 重置並等待下次檢查
                self._reset_state()
                return

        # 6. 執行 8-bit 步驟
        if self.check_ok:
            match self.step:
                case self.IDLE:
                    self.step = self.WRITE_VALID
                    self.sent = False
                case self.WRITE_VALID:
                    self._handle_hokuyo_write(
                        "write_valid", "1", "valid_success", "valid_failed", self.WRITE_PORT_NUMBER)
                case self.WRITE_PORT_NUMBER:
                    self._handle_hokuyo_write("write_port_number", context.get_cleaner_port,
                                              "port_number_success", "port_number_failed", self.WAIT_LOAD_REQ)
                case self.WAIT_LOAD_REQ:
                    if self.hokuyo_dms_8bit_1.load_req:
                        self.node.get_logger().info("✅收到load_req")
                        self.step = self.WRITE_TR_REQ
                    else:
                        self.node.get_logger().info("⏳等待load_req")
                case self.WRITE_TR_REQ:
                    self._handle_hokuyo_write(
                        "write_tr_req", "1", "tr_req_success", "tr_req_failed", self.WAIT_READY)
                case self.WAIT_READY:
                    if self.hokuyo_dms_8bit_1.ready:
                        self.node.get_logger().info("✅收到ready")
                        self.step = self.IDLE
                        from loader_agv.robot_states.take_cleaner.agv_port_check_empty_state import AgvPortCheckEmptyState
                        context.set_state(AgvPortCheckEmptyState(self.node))
                    else:
                        self.node.get_logger().info("⏳等待ready")
