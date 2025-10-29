from db_proxy.carrier_query_client import CarrierQueryClient
from db_proxy.eqp_signal_query_client import EqpSignalQueryClient
from rclpy.node import Node
from unloader_agv.robot_context import RobotContext
from agv_base.hokuyo_dms_8bit import HokuyoDMS8Bit
from unloader_agv.robot_states.base_robot_state import BaseRobotState
from shared_constants.equipment_stations import EquipmentStations


class OvenCheckEmptyState(BaseRobotState):
    # 8 BIT STEP
    IDLE = 0
    WRITE_VALID = 1
    WRITE_PORT_NUMBER = 2
    WAIT_LOAD_REQ = 3
    WRITE_TR_REQ = 4
    WAIT_READY = 5

    def __init__(self, node: Node):
        super().__init__(node)
        self.hokuyo_dms_8bit_1: HokuyoDMS8Bit = self.node.hokuyo_dms_8bit_1
        self.eqp_signal_query_client = EqpSignalQueryClient(node)
        self.carrier_query_client = CarrierQueryClient(node)

        # 動態計算 port_address 和 eqp_id (烤箱)
        self.port_address = self.node.room_id * 1000 + 60  # OVEN port address
        self.eqp_id = self.node.room_id * 100 + 6  # OVEN eqp_id

        self._reset_state()

    def _reset_state(self):
        """重置所有狀態變數"""
        self.check_ok = False
        self.step = self.IDLE
        self.sent = False
        self.search_eqp_signal_ok = False
        self.carrier_query_sended = False
        self.carrier_query_success = False
        self.port_carriers = [True] * 8  # 烤箱八個port的狀態 (True表示有貨物)
        self.select_oven_port = None  # 選定的 port_start (1, 3, 5, 7)
        self.selected_port_pair = None  # 選定的 port pair，例如 [5, 6]
        self.selected_pair_name = None  # 選定組合的名稱，例如 "port5+6"
        self.carrier_id = None

    def enter(self):
        self.node.get_logger().info("Unloader Robot Put Oven 目前狀態: OvenCheckEmpty")
        self._reset_state()

    def leave(self):
        self.node.get_logger().info("Unloader Robot Put Oven 離開 OvenCheckEmpty 狀態")
        self._reset_state()

    def eqp_signal_query_callback(self, response):
        """EQP信號查詢回調"""
        if response.success:
            self.node.get_logger().info("✅ 烤箱EQP信號查詢成功")
            # 解析烤箱port狀態 (port 1-8)
            self.port_carriers[0] = response.oven_port1  # port 1
            self.port_carriers[1] = response.oven_port2  # port 2
            self.port_carriers[2] = response.oven_port3  # port 3
            self.port_carriers[3] = response.oven_port4  # port 4
            self.port_carriers[4] = response.oven_port5  # port 5
            self.port_carriers[5] = response.oven_port6  # port 6
            self.port_carriers[6] = response.oven_port7  # port 7
            self.port_carriers[7] = response.oven_port8  # port 8

            self.search_eqp_signal_ok = True
            self.sent = False
        else:
            self.node.get_logger().error(f"❌ 烤箱EQP信號查詢失敗: {response.message}")

    def carrier_callback(self, response):
        """Carrier查詢回調"""
        if response.success:
            self.node.get_logger().info("✅ 烤箱Carrier查詢成功")
            if not response.carriers:  # 空的port
                self.node.get_logger().info(f"✅ 烤箱Port {self.select_oven_port} 為空，可以放置")
                self.carrier_query_success = True
            else:
                self.node.get_logger().info("⚠️ 選定的烤箱port有貨物")
                self.carrier_query_success = False
        else:
            self.node.get_logger().error(f"❌ 烤箱Carrier查詢失敗: {response.message}")

    def _update_context_states(self, context: RobotContext):
        """更新context中的狀態"""
        for i in range(8):
            setattr(context, f'oven_port{i+1}', self.port_carriers[i])

    def _validate_eqp_states(self, context: RobotContext):
        """驗證 Carrier 查詢結果與 EQP 狀態的一致性（PUT 操作空位檢查）"""
        validation_passed = True
        validation_errors = []

        # 驗證選定的 port (carrier_id 應為 None，對應 EQP 狀態應為 False)
        if self.carrier_id is None:
            port_number = self.oven_number
            eqp_state = getattr(context, f'oven_port{port_number}')
            if eqp_state:
                validation_passed = False
                validation_errors.append(f"OVEN_PORT{port_number}: Carrier查詢顯示空位但EQP狀態顯示有貨")
                self.node.get_logger().error(
                    f"❌ 資料不一致 - OVEN_PORT{port_number}: Carrier查詢=空位, EQP狀態={eqp_state}")
            else:
                self.node.get_logger().info(
                    f"✅ OVEN_PORT{port_number} 驗證通過: Carrier查詢=空位, EQP狀態={eqp_state}")

        return validation_passed, validation_errors

    def _extract_station_from_work_id(self, context: RobotContext):
        """從 work_id 中提取 station 並映射到 port pair (使用 EquipmentStations 模組)

        Returns:
            tuple: (station, port_pair)
            例如: (5, [5, 6]) 或 (7, [7, 8])
        """
        # 調用基類通用方法（通過 context.work_id 訪問，符合狀態模式）
        station, ports = self._extract_station_and_ports_from_work_id(context.work_id)
        if station is None:
            return None, None
        return station, ports

    def _check_port_pair_empty(self, port_pair):
        """檢查 port pair 是否兩個都空（PUT 操作的條件）

        Args:
            port_pair: [port1, port2]，例如 [5, 6] 或 [7, 8]

        Returns:
            bool: True 表示兩個 port 都空，False 表示至少一個有貨
        """
        port1, port2 = port_pair
        port1_has_cargo = self.port_carriers[port1 - 1]  # port 1-8 對應 index 0-7
        port2_has_cargo = self.port_carriers[port2 - 1]

        port1_empty = not port1_has_cargo
        port2_empty = not port2_has_cargo

        self.node.get_logger().debug(
            f"檢查 port pair [{port1}, {port2}]: "
            f"port{port1}={'空' if port1_empty else '有貨'}, "
            f"port{port2}={'空' if port2_empty else '有貨'}")

        # 核心邏輯：兩個都空才返回 True
        both_empty = port1_empty and port2_empty

        if both_empty:
            self.node.get_logger().info(
                f"✅ Port pair [{port1}, {port2}] 兩個都空，可以執行 PUT 操作")
        else:
            self.node.get_logger().warn(
                f"❌ Port pair [{port1}, {port2}] 未同時為空，無法執行 PUT 操作")

        return both_empty

    def _handle_port_selection(self, context: RobotContext):
        """處理 port 選擇邏輯 - 使用統一的 station-based 檢查"""
        if self.search_eqp_signal_ok and not self.check_ok:
            # 從 work_id 解析 station 和 port pair
            station, port_pair = self._extract_station_from_work_id(context)
            if port_pair is None:
                self.node.get_logger().error("無法從 work_id 解析 station，重置狀態")
                self._reset_state()
                return

            # 檢查 port pair 是否兩個都空
            if self._check_port_pair_empty(port_pair):
                # 保存選定的 port pair 資訊
                self.selected_port_pair = port_pair
                self.selected_pair_name = f"Station{station:02d}(port{port_pair[0]}+{port_pair[1]})"
                self.select_oven_port = station  # select_port 就是 station 編號
                context.oven_number = self.select_oven_port
                self.check_ok = True

                self.node.get_logger().info(
                    f"✅ 選擇 {self.selected_pair_name} (select_port={self.select_oven_port})，準備查詢 Carrier")
            else:
                self.node.get_logger().warn(
                    f"❌ Station{station:02d} port pair {port_pair} 未同時為空，無法執行 PUT 操作")
                # 沒有可用空位，進入完成狀態
                from unloader_agv.robot_states.complete_state import CompleteState
                context.set_state(CompleteState(self.node))

    def handle(self, context: RobotContext):
        self._update_context_states(context)

        # 查詢EQP信號
        if not self.search_eqp_signal_ok and not self.sent:
            self.eqp_signal_query_client.search_eqp_signal_eqp_id(
                self.eqp_id, self.eqp_signal_query_callback)
            self.sent = True

        # 更新 Hokuyo Input
        self._handle_hokuyo_input()

        self._handle_port_selection(context)

        # 查詢Carrier
        if self.check_ok and not self.carrier_query_sended:
            self.oven_number = context.oven_number
            port_id_target = self.port_address + self.oven_number
            self.carrier_query_client.search_carrier_port_id(
                port_id_min=port_id_target, port_id_max=port_id_target, callback=self.carrier_callback)
            self.carrier_query_sended = True

        # 處理查詢結果
        if self.carrier_query_sended and self.carrier_query_success:
            # 進行 EQP 信號狀態驗證
            self.node.get_logger().info(f"烤箱 Port {self.oven_number} 為空位，開始 EQP 狀態驗證。")

            # 執行 EQP 狀態驗證
            validation_passed, validation_errors = self._validate_eqp_states(context)

            if validation_passed:
                # EQP 狀態驗證通過，可以執行 PUT 操作
                self.node.get_logger().info(
                    f"✅ 烤箱 Port {self.oven_number} EQP 狀態驗證通過，可以執行烤箱操作")
                self._handle_8bit_steps(context)
            else:
                # EQP 狀態驗證失敗，重置狀態
                self.node.get_logger().error(f"❌ 烤箱 Port {self.oven_number} EQP 狀態驗證失敗:")
                for error in validation_errors:
                    self.node.get_logger().error(f"   - {error}")
                self.node.get_logger().error("Carrier 查詢結果與 EQP 硬體信號狀態不一致，重置狀態")
                self._reset_state()

    def _handle_8bit_steps(self, context: RobotContext):
        """處理8bit步驟"""
        match self.step:
            case self.IDLE:
                self.step = self.WRITE_VALID
                self.sent = False

            case self.WRITE_VALID:
                self._handle_step_operation(context, "valid寫入",
                                            lambda: self.hokuyo_dms_8bit_1.write_valid("1"),
                                            "valid_success", "valid_failed", self.WRITE_PORT_NUMBER)

            case self.WRITE_PORT_NUMBER:
                self._handle_step_operation(context, "port number寫入",
                                            lambda: self.hokuyo_dms_8bit_1.write_port_number(
                                                context.oven_number),
                                            "port_number_success", "port_number_failed", self.WAIT_LOAD_REQ)

            case self.WAIT_LOAD_REQ:
                if self.hokuyo_dms_8bit_1.load_req:
                    self.node.get_logger().info("✅收到load_req")
                    self.step = self.WRITE_TR_REQ
                else:
                    self.node.get_logger().debug("⏳等待load_req")

            case self.WRITE_TR_REQ:
                self._handle_step_operation(context, "tr_req寫入",
                                            lambda: self.hokuyo_dms_8bit_1.write_tr_req("1"),
                                            "tr_req_success", "tr_req_failed", self.WAIT_READY)

            case self.WAIT_READY:
                if self.hokuyo_dms_8bit_1.ready:
                    self.node.get_logger().info("✅收到ready")
                    self.step = self.IDLE
                    from unloader_agv.robot_states.put_oven.take_agv_state import TakeAgvState
                    context.set_state(TakeAgvState(self.node))
                else:
                    self.node.get_logger().debug("⏳等待ready")

    def _handle_step_operation(self, context, operation_name, operation_func, success_flag, failed_flag, next_step):
        """統一處理步驟操作"""
        if not self.sent:
            operation_func()
            self.sent = True

        hokuyo = self.hokuyo_dms_8bit_1
        if getattr(hokuyo, success_flag):
            self.node.get_logger().info(f"✅{operation_name}成功")
            setattr(hokuyo, success_flag, False)
            self.sent = False
            self.step = next_step
        elif getattr(hokuyo, failed_flag):
            self.node.get_logger().error(f"❌{operation_name}失敗")
            setattr(hokuyo, failed_flag, False)
            self.sent = False
        else:
            self.node.get_logger().debug(f"⏳等待{operation_name}")
