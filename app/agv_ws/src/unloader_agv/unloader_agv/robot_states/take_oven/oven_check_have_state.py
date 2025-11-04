from db_proxy.carrier_query_client import CarrierQueryClient
from db_proxy.eqp_signal_query_client import EqpSignalQueryClient
from rclpy.node import Node
from unloader_agv.robot_context import RobotContext
from agv_base.hokuyo_dms_8bit import HokuyoDMS8Bit
from unloader_agv.robot_states.base_robot_state import BaseRobotState
from shared_constants.equipment_stations import EquipmentStations


class OvenCheckHaveState(BaseRobotState):
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
        self.port_carriers = [False] * 8  # 烤箱八個port的狀態
        self.carrier_id_1 = None  # Oven Port 1 的 carrier_id
        self.carrier_id_2 = None  # Oven Port 2 的 carrier_id
        self.carrier_id_3 = None  # Oven Port 3 的 carrier_id
        self.carrier_id_4 = None  # Oven Port 4 的 carrier_id
        self.workstation_ports = None  # 存儲選定的 PORT 組合（所有4個 ports）
        self.selected_pair_name = None  # 存儲選定組合的名稱
        self.selected_port = None  # 存儲選定組合的 select_port 值

    def enter(self):
        self.node.get_logger().info("Unloader Robot Take Oven 目前狀態: OvenCheckHave")
        self._reset_state()

    def leave(self):
        self.node.get_logger().info("Unloader Robot Take Oven 離開 OvenCheckHave 狀態")
        self._reset_state()

    def eqp_signal_query_callback(self, response):
        """EQP信號查詢回調"""
        if response.success:
            self.node.get_logger().info("✅ EQP信號查詢成功")
            # 解析烤箱port狀態 (port 1-8) - 使用迴圈簡化
            for i in range(8):
                self.port_carriers[i] = getattr(response, f'oven_port{i+1}')

            self.search_eqp_signal_ok = True
            self.sent = False
        else:
            self.node.get_logger().error(f"❌ EQP信號查詢失敗: {response.message}")

    def carrier_callback(self, response):
        """Carrier查詢回調 - 處理所有4個 OVEN PORT 的查詢結果"""
        self.carrier_query_success = response.success
        if response.success:
            self.node.get_logger().info("✅ 烤箱所有4個 PORT Carrier查詢成功")

            # 查詢所有4個 OVEN PORT 的 carrier_id (ports 1-4)
            for i in range(4):
                port_id = self.port_address + i + 1  # 2061, 2062, 2063, 2064
                carrier_id = CarrierQueryClient.carrier_port_id_carrier_id(response, port_id)

                # 存儲到對應的變數
                if i == 0:
                    self.carrier_id_1 = carrier_id
                elif i == 1:
                    self.carrier_id_2 = carrier_id
                elif i == 2:
                    self.carrier_id_3 = carrier_id
                elif i == 3:
                    self.carrier_id_4 = carrier_id

                # 記錄查詢結果
                if carrier_id is not None:
                    self.node.get_logger().info(
                        f"烤箱 PORT {i+1} (ID={port_id}) 有 Carrier ID: {carrier_id}")
                else:
                    self.node.get_logger().warn(
                        f"烤箱 PORT {i+1} (ID={port_id}) 沒有 Carrier")
        else:
            self.node.get_logger().error(
                f"❌ 烤箱所有 PORT Carrier查詢失敗: {response.message}")

    def _update_context_states(self, context: RobotContext):
        """更新context中的狀態"""
        if not self.search_eqp_signal_ok:
            return

        # 設定 oven port 狀態 (使用迴圈簡化)
        for i in range(8):
            setattr(context, f'oven_port{i+1}', self.port_carriers[i])

    def _extract_station_from_work_id(self, context: RobotContext):
        """從 work_id 中提取 station 並映射到 port pair (使用 EquipmentStations 模組)

        Returns:
            tuple: (station, port_pair)
            例如: (1, [1, 2]) 或 (5, [5, 6])
        """
        # 調用基類通用方法（通過 context.work_id 訪問，符合狀態模式）
        station, ports = self._extract_station_and_ports_from_work_id(context.work_id)
        if station is None:
            return None, None
        return station, ports

    def _check_port_pair_have_cargo(self, port_pair):
        """檢查 port pair 是否兩個都有貨（TAKE 操作的條件）

        Args:
            port_pair: [port1, port2]，例如 [1, 2] 或 [3, 4]

        Returns:
            bool: True 表示兩個 port 都有貨，False 表示至少一個沒貨
        """
        port1, port2 = port_pair
        port1_has_cargo = self.port_carriers[port1 - 1]  # port 1-8 對應 index 0-7
        port2_has_cargo = self.port_carriers[port2 - 1]

        self.node.get_logger().debug(
            f"檢查 port pair [{port1}, {port2}]: "
            f"port{port1}={'有貨' if port1_has_cargo else '無貨'}, "
            f"port{port2}={'有貨' if port2_has_cargo else '無貨'}")

        # 核心邏輯：兩個都有貨才返回 True
        both_have_cargo = port1_has_cargo and port2_has_cargo

        if both_have_cargo:
            self.node.get_logger().info(
                f"✅ Port pair [{port1}, {port2}] 兩個都有貨，可以執行 TAKE 操作")
        else:
            self.node.get_logger().warn(
                f"❌ Port pair [{port1}, {port2}] 未同時有貨，無法執行 TAKE 操作")

        return both_have_cargo

    def _get_select_port_from_physical_port(self, port_number):
        """根據實際 PORT 號碼獲取對應的 select_port 值"""
        # 烤箱 port 到 select_port 的對應關係：
        # port1+2 → select_port = 1
        # port3+4 → select_port = 3
        # port5+6 → select_port = 5
        # port7+8 → select_port = 7
        if port_number in [1, 2]:
            return 1
        elif port_number in [3, 4]:
            return 3
        elif port_number in [5, 6]:
            return 5
        elif port_number in [7, 8]:
            return 7
        else:
            return 0

    def _calculate_port_ids(self, selected_port_pair):
        """計算選定 port 組合的兩個 PORT ID"""
        if not selected_port_pair:
            return None, None
        return (self.port_address + selected_port_pair[0],
                self.port_address + selected_port_pair[1])

    def _validate_eqp_states(self, context: RobotContext):
        """驗證 Carrier 查詢結果與 EQP 狀態的一致性（所有4個 PORT 檢查）"""
        validation_passed = True
        validation_errors = []

        # 驗證所有4個 port
        carrier_ids = [self.carrier_id_1, self.carrier_id_2, self.carrier_id_3, self.carrier_id_4]
        for i, carrier_id in enumerate(carrier_ids):
            port_number = i + 1
            eqp_state = getattr(context, f'oven_port{port_number}')

            if carrier_id is not None:
                # Carrier 有貨，檢查 EQP 狀態是否一致
                if not eqp_state:
                    validation_passed = False
                    validation_errors.append(
                        f"OVEN_PORT{port_number}: Carrier查詢有貨但EQP狀態顯示無貨")
                    self.node.get_logger().error(
                        f"❌ 資料不一致 - OVEN_PORT{port_number}: "
                        f"Carrier ID={carrier_id}, EQP狀態={eqp_state}")
                else:
                    self.node.get_logger().info(
                        f"✅ OVEN_PORT{port_number} 驗證通過: "
                        f"Carrier ID={carrier_id}, EQP狀態={eqp_state}")
            else:
                # Carrier 無貨
                if eqp_state:
                    validation_passed = False
                    validation_errors.append(
                        f"OVEN_PORT{port_number}: Carrier查詢無貨但EQP狀態顯示有貨")
                    self.node.get_logger().error(
                        f"❌ 資料不一致 - OVEN_PORT{port_number}: "
                        f"Carrier查詢無貨, EQP狀態={eqp_state}")

        return validation_passed, validation_errors

    def _handle_port_selection(self, context: RobotContext):
        """處理 port 選擇邏輯 - 檢查所有4個 oven port 是否都有貨"""
        if self.search_eqp_signal_ok and not self.check_ok:
            # 從 work_id 解析 station 和所有相關 ports
            station, all_ports = self._extract_station_from_work_id(context)
            if all_ports is None or len(all_ports) < 4:
                self.node.get_logger().error(
                    f"無法從 work_id 解析足夠的 ports (需要4個)，實際={all_ports}，重置狀態")
                self._reset_state()
                return

            # 檢查所有4個 port 是否都有貨
            all_ports_have_cargo = all(self.port_carriers[port - 1] for port in all_ports)

            if all_ports_have_cargo:
                # 所有4個 port 都有貨，可以執行兩次 TAKE 操作
                # 將4個 ports 分成兩組：[[port1, port2], [port3, port4]]
                port_groups = [
                    [all_ports[0], all_ports[1]],  # 第1次
                    [all_ports[2], all_ports[3]]   # 第2次
                ]

                # 保存選定的 port 資訊
                self.workstation_ports = all_ports  # 保存所有4個 ports
                self.selected_pair_name = (
                    f"Station{station}(ports {all_ports[0]},{all_ports[1]},{all_ports[2]},{all_ports[3]})")
                self.selected_port = all_ports[0]  # 第1次從第一個 port 開始

                self.check_ok = True

                self.node.get_logger().info(
                    f"✅ 所有4個 Oven PORT 都有貨，準備兩次 TAKE 操作：\n"
                    f"   第1次: ports {port_groups[0]}\n"
                    f"   第2次: ports {port_groups[1]}")
            else:
                # 記錄哪些 port 沒有貨
                empty_ports = [port for port in all_ports if not self.port_carriers[port - 1]]
                self.node.get_logger().warn(
                    f"❌ Oven PORT {empty_ports} 沒有貨，無法執行 TAKE_OVEN 操作（需要所有4個 PORT 都有貨）")
                self._reset_state()

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

        # 查詢所有4個 OVEN PORT 的 Carrier
        if self.check_ok and not self.carrier_query_sended:
            port_id_min = self.port_address + 1  # 2061
            port_id_max = self.port_address + 4  # 2064
            self.node.get_logger().info(
                f"查詢烤箱所有4個 PORT Carrier：PORT 1-4 (ID: {port_id_min}-{port_id_max})")
            self.carrier_query_client.search_carrier_port_id(
                port_id_min=port_id_min, port_id_max=port_id_max, callback=self.carrier_callback)
            self.carrier_query_sended = True

        # 處理 Carrier 查詢結果
        if self.carrier_query_sended and self.carrier_query_success:
            # 檢查所有4個 port 是否都有 carrier
            carrier_ids = [self.carrier_id_1, self.carrier_id_2, self.carrier_id_3, self.carrier_id_4]
            all_have_carrier = all(cid is not None for cid in carrier_ids)

            if all_have_carrier:
                # 所有4個 port 都有貨，進行 EQP 信號狀態驗證
                self.node.get_logger().info("所有4個 Oven PORT 都有 Carrier，開始 EQP 狀態驗證")

                # 執行 EQP 狀態驗證
                validation_passed, validation_errors = self._validate_eqp_states(context)

                if validation_passed:
                    # EQP 狀態驗證通過，初始化兩次循環控制
                    self.node.get_logger().info("✅ 所有 Oven PORT EQP 狀態驗證通過")

                    # 將4個 ports 分成兩組：[[1,2], [3,4]]
                    port_groups = [[1, 2], [3, 4]]

                    # 初始化循環控制變數
                    context.take_put_port_groups = port_groups
                    context.take_put_cycle_count = 0
                    context.take_put_current_batch = port_groups[0]  # 第1次從 port 1,2 開始

                    # 存儲所有4個 carrier_id 到 context
                    context.carrier_id[0] = self.carrier_id_1
                    context.carrier_id[1] = self.carrier_id_2
                    context.carrier_id[2] = self.carrier_id_3
                    context.carrier_id[3] = self.carrier_id_4

                    # 設定 get_oven_port 為第一個 port
                    context.get_oven_port = self.selected_port

                    self.node.get_logger().info(
                        f"✅ 烤箱檢查完成，初始化兩次循環控制：\n"
                        f"   第1次: ports {port_groups[0]}\n"
                        f"   第2次: ports {port_groups[1]}\n"
                        f"   Carrier IDs: {carrier_ids}")

                    # 進入 8bit 步驟
                    self._handle_8bit_steps(context)
                else:
                    # EQP 狀態驗證失敗，重置狀態
                    self.node.get_logger().error("❌ Oven PORT EQP 狀態驗證失敗:")
                    for error in validation_errors:
                        self.node.get_logger().error(f"   - {error}")
                    self.node.get_logger().error("Carrier 查詢結果與 EQP 硬體信號狀態不一致，重置狀態")
                    self._reset_state()
            else:
                # 有 port 沒有貨
                missing_ports = [i + 1 for i, cid in enumerate(carrier_ids) if cid is None]
                self.node.get_logger().error(
                    f"❌ Carrier 查詢成功，但 Oven PORT {missing_ports} 沒有貨物")
                self.node.get_logger().error(
                    "所有4個 PORT 都需要有貨物才能執行 TAKE_OVEN 操作")
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
                # 根據當前循環次數動態獲取 port number
                port_number = context.take_put_port_groups[context.take_put_cycle_count][0]
                self._handle_step_operation(context, "port number寫入",
                                            lambda: self.hokuyo_dms_8bit_1.write_port_number(port_number),
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
                    from unloader_agv.robot_states.take_oven.agv_port_check_empty_state import AgvPortCheckEmptyState
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
