from db_proxy.carrier_query_client import CarrierQueryClient
from db_proxy.eqp_signal_query_client import EqpSignalQueryClient
from rclpy.node import Node
from unloader_agv.robot_context import RobotContext
from agv_base.hokuyo_dms_8bit import HokuyoDMS8Bit
from unloader_agv.robot_states.base_robot_state import BaseRobotState
from shared_constants.equipment_stations import EquipmentStations


class PreDryerCheckHaveState(BaseRobotState):
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
        self.eqp_signal_query_client = EqpSignalQueryClient(node)
        self.carrier_query_client = CarrierQueryClient(node)

        # 動態計算 port_address 和 eqp_id (預乾燥機)
        self.port_address = self.node.room_id * 1000 + 50  # PRE_DRYER port address
        self.eqp_id = self.node.room_id * 100 + 5  # PRE_DRYER eqp_id

        self._reset_state()

    def _reset_state(self):
        """重置所有狀態變數"""
        self.check_ok = False
        self.step = self.IDLE
        self.sent = False
        self.search_eqp_signal_ok = False
        self.carrier_query_sended = False
        self.carrier_query_success = False
        self.port_carriers = [False] * 8  # 預乾燥機八個port的狀態
        # 存儲所有4個port的carrier_id
        self.carrier_id_1 = None
        self.carrier_id_2 = None
        self.carrier_id_3 = None
        self.carrier_id_4 = None
        self.workstation_ports = None  # 存儲選定的4個PORT組合
        self.selected_pair_name = None  # 存儲選定組合的名稱
        self.selected_port = None  # 存儲選定組合的 select_port 值

    def enter(self):
        self.node.get_logger().info("Unloader Robot Take Pre Dryer 目前狀態: PreDryerCheckHave")
        self._reset_state()

    def leave(self):
        self.node.get_logger().info("Unloader Robot Take Pre Dryer 離開 PreDryerCheckHave 狀態")
        self._reset_state()

    def eqp_signal_query_callback(self, response):
        """EQP信號查詢回調"""
        if response.success:
            self.node.get_logger().info("✅ EQP信號查詢成功")
            # 解析預乾燥機port狀態 (port 1-8) - 使用迴圈簡化
            for i in range(8):
                self.port_carriers[i] = getattr(response, f'pre_dryer_port{i+1}')

            self.search_eqp_signal_ok = True
            self.sent = False
        else:
            self.node.get_logger().error(f"❌ EQP信號查詢失敗: {response.message}")

    def carrier_callback(self, response):
        """Carrier查詢回調 - 處理所有4個PORT的查詢結果"""
        self.carrier_query_success = response.success
        if response.success:
            self.node.get_logger().info(f"✅ 預乾燥機 {self.selected_pair_name} Carrier查詢成功")

            # 計算所有4個port的PORT ID並查詢carrier
            if self.workstation_ports and len(self.workstation_ports) == 4:
                port_id_1 = self.port_address + self.workstation_ports[0]
                port_id_2 = self.port_address + self.workstation_ports[1]
                port_id_3 = self.port_address + self.workstation_ports[2]
                port_id_4 = self.port_address + self.workstation_ports[3]

                # 使用 CarrierQueryClient 的靜態方法獲取所有4個port的carrier_id
                self.carrier_id_1 = CarrierQueryClient.carrier_port_id_carrier_id(
                    response, port_id_1)
                self.carrier_id_2 = CarrierQueryClient.carrier_port_id_carrier_id(
                    response, port_id_2)
                self.carrier_id_3 = CarrierQueryClient.carrier_port_id_carrier_id(
                    response, port_id_3)
                self.carrier_id_4 = CarrierQueryClient.carrier_port_id_carrier_id(
                    response, port_id_4)

                # 記錄所有4個port的查詢結果
                for i, (port, carrier_id) in enumerate([
                    (self.workstation_ports[0], self.carrier_id_1),
                    (self.workstation_ports[1], self.carrier_id_2),
                    (self.workstation_ports[2], self.carrier_id_3),
                    (self.workstation_ports[3], self.carrier_id_4)
                ], 1):
                    if carrier_id is not None:
                        self.node.get_logger().info(
                            f"PORT {port} 有 Carrier ID: {carrier_id}")
                    else:
                        self.node.get_logger().warn(f"PORT {port} 沒有 Carrier")
        else:
            self.node.get_logger().error(
                f"❌ 預乾燥機 {self.selected_pair_name} Carrier查詢失敗: {response.message}")

    def _update_context_states(self, context: RobotContext):
        """更新context中的狀態"""
        if not self.search_eqp_signal_ok:
            return

        # 設定 pre_dryer port 狀態 (使用迴圈簡化)
        for i in range(8):
            setattr(context, f'pre_dryer_port{i+1}', self.port_carriers[i])

    def _extract_station_from_work_id(self, context: RobotContext):
        """從 work_id 中提取 station 並映射到 PORT 範圍 (使用 EquipmentStations 模組)"""
        # 調用基類通用方法（通過 context.work_id 訪問，符合狀態模式）
        station, ports = self._extract_station_and_ports_from_work_id(context.work_id)
        if station is None:
            return None, None, None
        # station 就是 select_port
        return station, ports, station

    def _calculate_port_ids(self, selected_ports):
        """計算選定port組合的所有4個PORT ID"""
        if not selected_ports or len(selected_ports) != 4:
            return None, None, None, None
        return (self.port_address + selected_ports[0],
                self.port_address + selected_ports[1],
                self.port_address + selected_ports[2],
                self.port_address + selected_ports[3])

    def _validate_eqp_states(self, context: RobotContext):
        """驗證所有4個Port的 Carrier 查詢結果與 EQP 狀態的一致性"""
        validation_passed = True
        validation_errors = []

        # 驗證所有4個ports
        carrier_ids = [self.carrier_id_1, self.carrier_id_2, self.carrier_id_3, self.carrier_id_4]

        for i, carrier_id in enumerate(carrier_ids):
            port_number = self.workstation_ports[i]
            eqp_state = getattr(context, f'pre_dryer_port{port_number}')

            if carrier_id is not None:
                if not eqp_state:
                    validation_passed = False
                    validation_errors.append(f"PORT{port_number}: Carrier查詢有貨(ID:{carrier_id})但EQP狀態顯示無貨")
                    self.node.get_logger().error(
                        f"❌ 資料不一致 - PORT{port_number}: Carrier ID={carrier_id}, EQP狀態={eqp_state}")
                else:
                    self.node.get_logger().info(
                        f"✅ PORT{port_number} 驗證通過: Carrier ID={carrier_id}, EQP狀態={eqp_state}")
            else:
                # carrier_id 為 None
                if eqp_state:
                    validation_passed = False
                    validation_errors.append(f"PORT{port_number}: EQP狀態顯示有貨但Carrier查詢無結果")
                    self.node.get_logger().error(
                        f"❌ 資料不一致 - PORT{port_number}: Carrier ID=None, EQP狀態={eqp_state}")

        return validation_passed, validation_errors

    def _handle_port_selection(self, context: RobotContext):
        """處理port選擇邏輯 - 從 work_id 解析 station 並檢查所有4個PORT是否都有貨物"""
        if self.search_eqp_signal_ok and not self.check_ok:
            # 從 work_id 中解析 station 和對應的 PORT 範圍
            station, available_ports, select_port = self._extract_station_from_work_id(context)
            if station is None or available_ports is None or select_port is None:
                self.node.get_logger().error("無法從 work_id 解析 station，重置狀態")
                self._reset_state()
                return

            # 確保解析到4個ports
            if len(available_ports) != 4:
                self.node.get_logger().error(
                    f"❌ 從 work_id 解析的 ports 數量不正確: {available_ports}, 預期4個")
                self._reset_state()
                return

            # 檢查所有4個ports
            port1, port2, port3, port4 = available_ports[0], available_ports[1], available_ports[2], available_ports[3]
            port1_has_cargo = self.port_carriers[port1 - 1]  # port 1-8 對應 index 0-7
            port2_has_cargo = self.port_carriers[port2 - 1]
            port3_has_cargo = self.port_carriers[port3 - 1]
            port4_has_cargo = self.port_carriers[port4 - 1]

            pair_name = f"Station{station:02d}(ports {port1},{port2},{port3},{port4})"

            self.node.get_logger().info(
                f"檢查 {pair_name}: PORT{port1}={port1_has_cargo}, PORT{port2}={port2_has_cargo}, "
                f"PORT{port3}={port3_has_cargo}, PORT{port4}={port4_has_cargo}")

            # 檢查選擇條件：必須所有4個ports都有貨物
            selected = False
            priority_msg = ""

            if port1_has_cargo and port2_has_cargo and port3_has_cargo and port4_has_cargo:
                selected = True
                priority_msg = "所有4個port都有貨物，確認完成"
            else:
                selected = False
                missing_ports = []
                if not port1_has_cargo:
                    missing_ports.append(port1)
                if not port2_has_cargo:
                    missing_ports.append(port2)
                if not port3_has_cargo:
                    missing_ports.append(port3)
                if not port4_has_cargo:
                    missing_ports.append(port4)
                priority_msg = f"缺少貨物的port: {missing_ports}，無法執行任務"

            if selected:
                # 保存選定的4個port組合和相關資訊
                self.workstation_ports = available_ports
                self.selected_pair_name = pair_name
                self.selected_port = select_port
                self.check_ok = True

                # ✅ 將4個ports分成兩組（兩次取放操作）
                # station 11: [1,2,5,6] → [[1,2], [5,6]]
                # station 13: [3,4,7,8] → [[3,4], [7,8]]
                port_groups = [
                    [available_ports[0], available_ports[1]],  # 第1次: 前2個ports
                    [available_ports[2], available_ports[3]]   # 第2次: 後2個ports
                ]

                # 初始化兩次取放循環控制（保存到 self 以便後續使用）
                self.port_groups = port_groups

                self.node.get_logger().info(
                    f"✅ {priority_msg} - {pair_name} (select_port={select_port})")
                self.node.get_logger().info(
                    f"📋 初始化兩次取放操作："
                    f"\n  第1次: 取 pre_dryer ports {port_groups[0]}"
                    f"\n  第2次: 取 pre_dryer ports {port_groups[1]}")
                self.node.get_logger().info(f"準備查詢所有4個port的 Carrier 驗證")
            else:
                self.node.get_logger().warn(
                    f"⚠️ {priority_msg} - {pair_name}")
                self.node.get_logger().warn(
                    f"❌ {pair_name} 未滿足所有port都有貨物的條件，無法執行 TAKE 操作")
                self._reset_state()
                return

    def handle(self, context: RobotContext):
        self._update_context_states(context)

        # 查詢EQP信號
        if not self.search_eqp_signal_ok and not self.sent:
            self.eqp_signal_query_client.search_eqp_signal_eqp_id(
                self.eqp_id, self.eqp_signal_query_callback)
            self.sent = True

        # 更新 Hokuyo Input - 使用統一方法
        self._handle_hokuyo_input()

        self._handle_port_selection(context)

        # 查詢選定4個PORT的 Carrier
        if self.check_ok and not self.carrier_query_sended and self.workstation_ports:
            port_id_1, port_id_2, port_id_3, port_id_4 = self._calculate_port_ids(self.workstation_ports)
            self.node.get_logger().info(
                f"查詢預乾燥機 {self.selected_pair_name} Carrier：")
            self.node.get_logger().info(
                f"  PORT {self.workstation_ports[0]} (ID: {port_id_1})")
            self.node.get_logger().info(
                f"  PORT {self.workstation_ports[1]} (ID: {port_id_2})")
            self.node.get_logger().info(
                f"  PORT {self.workstation_ports[2]} (ID: {port_id_3})")
            self.node.get_logger().info(
                f"  PORT {self.workstation_ports[3]} (ID: {port_id_4})")
            self.carrier_query_client.search_carrier_port_id(
                port_id_min=port_id_1, port_id_max=port_id_4, callback=self.carrier_callback)
            self.carrier_query_sended = True

        # 處理 Carrier 查詢結果
        if self.carrier_query_sended and self.carrier_query_success and self.workstation_ports:
            port_id_1, port_id_2, port_id_3, port_id_4 = self._calculate_port_ids(self.workstation_ports)

            # 檢查所有4個port是否都有carrier
            if (self.carrier_id_1 is not None and self.carrier_id_2 is not None and
                    self.carrier_id_3 is not None and self.carrier_id_4 is not None):
                # 所有4個port都有貨，進行 EQP 信號狀態驗證
                self.node.get_logger().info(
                    f"{self.selected_pair_name} 所有4個port都有貨物，開始 EQP 狀態驗證。")

                # 執行 EQP 狀態驗證
                validation_passed, validation_errors = self._validate_eqp_states(context)

                if validation_passed:
                    # EQP 狀態驗證通過，可以執行 TAKE 操作
                    self.node.get_logger().info(
                        f"✅ {self.selected_pair_name} EQP 狀態驗證通過，可以執行預乾燥機操作")

                    self.node.get_logger().info(
                        f"所有4個Carrier已確認:")
                    self.node.get_logger().info(
                        f"  PORT {self.workstation_ports[0]} - Carrier ID: {self.carrier_id_1}")
                    self.node.get_logger().info(
                        f"  PORT {self.workstation_ports[1]} - Carrier ID: {self.carrier_id_2}")
                    self.node.get_logger().info(
                        f"  PORT {self.workstation_ports[2]} - Carrier ID: {self.carrier_id_3}")
                    self.node.get_logger().info(
                        f"  PORT {self.workstation_ports[3]} - Carrier ID: {self.carrier_id_4}")

                    # 設定 context 變數（所有4個carrier）
                    context.carrier_id[0] = self.carrier_id_1
                    context.carrier_id[1] = self.carrier_id_2
                    context.carrier_id[2] = self.carrier_id_3
                    context.carrier_id[3] = self.carrier_id_4

                    # ✅ 初始化兩次取放循環控制
                    context.take_put_port_groups = self.port_groups
                    context.take_put_cycle_count = 0
                    context.take_put_current_batch = self.port_groups[0]  # 第1次使用前2個ports
                    context.take_put_max_cycles = 2

                    # 設定第1次操作的參數
                    context.get_pre_dryer_port = self.port_groups[0][0]  # 使用第1組的第一個port
                    context.get_unloader_agv_port_back = 1  # 第1次放到AGV port 1

                    self.node.get_logger().info("=" * 80)
                    self.node.get_logger().info("✅ PreDryerCheckHave 驗證通過")
                    self.node.get_logger().info(
                        f"📦 已保存4個Carrier ID到context: {context.carrier_id}")
                    self.node.get_logger().info(
                        f"📋 將執行兩次取放操作："
                        f"\n  第1次: 取 pre_dryer ports {context.take_put_port_groups[0]} → 放到 AGV ports [1,2]"
                        f"\n  第2次: 取 pre_dryer ports {context.take_put_port_groups[1]} → 放到 AGV ports [3,4]")
                    self.node.get_logger().info("=" * 80)
                    self.node.get_logger().info(
                        f"預乾燥機 {self.selected_pair_name} 檢查完成，進入下一個狀態")
                    self._handle_8bit_steps(context)
                else:
                    # EQP 狀態驗證失敗，重置狀態
                    self.node.get_logger().error(f"❌ {self.selected_pair_name} EQP 狀態驗證失敗:")
                    for error in validation_errors:
                        self.node.get_logger().error(f"   - {error}")
                    self.node.get_logger().error("Carrier 查詢結果與 EQP 硬體信號狀態不一致，重置狀態")
                    self._reset_state()
            else:
                # 不是所有port都有貨，無法執行 TAKE 操作
                missing_carriers = []
                if self.carrier_id_1 is None:
                    missing_carriers.append(f"PORT{self.workstation_ports[0]}")
                if self.carrier_id_2 is None:
                    missing_carriers.append(f"PORT{self.workstation_ports[1]}")
                if self.carrier_id_3 is None:
                    missing_carriers.append(f"PORT{self.workstation_ports[2]}")
                if self.carrier_id_4 is None:
                    missing_carriers.append(f"PORT{self.workstation_ports[3]}")

                self.node.get_logger().error(
                    f"❌ Carrier 查詢成功，但以下Port沒有貨物: {missing_carriers}")
                self.node.get_logger().error(
                    f"{self.selected_pair_name} 未滿足所有4個port都有貨物的條件，無法執行預乾燥機操作。")
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
                                                context.get_pre_dryer_port),
                                            "port_number_success", "port_number_failed", self.WAIT_UNLOAD_REQ)

            case self.WAIT_UNLOAD_REQ:
                if self.hokuyo_dms_8bit_1.unload_req:
                    self.node.get_logger().info("✅收到unload_req")
                    self.step = self.WRITE_TR_REQ
                else:
                    self.node.get_logger().debug("⏳等待unload_req")

            case self.WRITE_TR_REQ:
                self._handle_step_operation(context, "tr_req寫入",
                                            lambda: self.hokuyo_dms_8bit_1.write_tr_req("1"),
                                            "tr_req_success", "tr_req_failed", self.WAIT_READY)

            case self.WAIT_READY:
                if self.hokuyo_dms_8bit_1.ready:
                    self.node.get_logger().info("✅收到ready")
                    self.step = self.IDLE
                    from unloader_agv.robot_states.take_pre_dryer.agv_port_check_empty_state import AgvPortCheckEmptyState
                    context.set_state(AgvPortCheckEmptyState(self.node))
                else:
                    self.node.get_logger().debug("⏳等待ready")

    def _handle_step_operation(self, _, operation_name, operation_func, success_flag, failed_flag, next_step):
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
