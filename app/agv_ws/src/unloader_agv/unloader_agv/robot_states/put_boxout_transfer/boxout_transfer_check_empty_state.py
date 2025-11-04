from db_proxy.carrier_query_client import CarrierQueryClient
from db_proxy.eqp_signal_query_client import EqpSignalQueryClient
from rclpy.node import Node
from unloader_agv.robot_context import RobotContext  # 新增的匯入
from std_msgs.msg import Bool  # 匯入 ROS 2 的 Bool 訊息型態
from agv_base.hokuyo_dms_8bit import HokuyoDMS8Bit


from unloader_agv.robot_states.base_robot_state import BaseRobotState


class BoxoutTransferCheckEmptyState(BaseRobotState):
    RobotContext.boxout_up_both_empty = True

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

        # 動態計算 port_address 和 eqp_id
        self.port_address = self.node.room_id * 1000 + 20
        self.eqp_id = self.node.room_id * 100 + 2

        self._reset_state()

    def _reset_state(self):
        """重置所有狀態變數"""
        self.check_ok = False
        self.step = self.IDLE
        self.sent = False
        self.search_eqp_signal_ok = False
        self.carrier_query_sended = False
        self.carrier_query_success = False
        self.port_carriers = [True] * 4  # 統一管理四個port的狀態
        self.workstation_ports = []  # 儲存從work_id解析出的4個port numbers
        self.port_groups = []  # 儲存分組後的port IDs [[2021,2022], [2023,2024]]
        # 存儲所有4個carrier_id
        self.carrier_id_port1 = None
        self.carrier_id_port2 = None
        self.carrier_id_port3 = None
        self.carrier_id_port4 = None

    def enter(self):
        self.node.get_logger().info("Unloader Robot Put BoxoutTransfer 目前狀態: BoxoutTransferCheckEmpty")
        self._reset_state()

    def leave(self):
        self.node.get_logger().info("Unloader Robot Put BoxoutTransfer 離開 BoxoutTransferCheckEmpty 狀態")
        self._reset_state()

    def eqp_signal_query_callback(self, response):
        """查詢 EQP 信號回調 - 查詢所有4個ports狀態"""
        if not self.workstation_ports or len(self.workstation_ports) != 4:
            self.node.get_logger().error("❌ workstation_ports 未正確初始化")
            return

        # 查詢這4個ports的狀態
        for i in range(4):
            port_number = self.workstation_ports[i]
            port_id = self.port_address + port_number
            self.port_carriers[i] = EqpSignalQueryClient.eqp_signal_port(response, port_id)
            self.node.get_logger().info(
                f"Boxout Transfer Port {port_number} (ID: {port_id}) 有無貨: {self.port_carriers[i]}")

        self.search_eqp_signal_ok = True

    def carrier_callback(self, response):
        """查詢 Carrier 回調 - 查詢所有4個ports的carrier_id"""
        self.carrier_query_success = response.success

        if not self.workstation_ports or len(self.workstation_ports) != 4:
            self.node.get_logger().error("❌ workstation_ports 未正確初始化")
            return

        # 計算所有4個port IDs並查詢carrier
        port_ids = [self.port_address + port_num for port_num in self.workstation_ports]

        self.carrier_id_port1 = CarrierQueryClient.carrier_port_id_carrier_id(response, port_ids[0])
        self.carrier_id_port2 = CarrierQueryClient.carrier_port_id_carrier_id(response, port_ids[1])
        self.carrier_id_port3 = CarrierQueryClient.carrier_port_id_carrier_id(response, port_ids[2])
        self.carrier_id_port4 = CarrierQueryClient.carrier_port_id_carrier_id(response, port_ids[3])

        self.node.get_logger().info(
            f"查詢 Boxout Transfer Carrier 結果："
            f"\n  Port {self.workstation_ports[0]} (ID: {port_ids[0]}): carrier_id={self.carrier_id_port1}"
            f"\n  Port {self.workstation_ports[1]} (ID: {port_ids[1]}): carrier_id={self.carrier_id_port2}"
            f"\n  Port {self.workstation_ports[2]} (ID: {port_ids[2]}): carrier_id={self.carrier_id_port3}"
            f"\n  Port {self.workstation_ports[3]} (ID: {port_ids[3]}): carrier_id={self.carrier_id_port4}"
        )

    def _extract_station_from_work_id(self, context: RobotContext):
        """從 work_id 中提取 station 並計算 4 個 port numbers

        work_id 格式: REESSAA
        - RE: room_id (20)
        - E: equipment_type (02 = boxout_transfer)
        - SS: station (01)
        - AA: action (02 = PUT)

        Station 01 → ports [1, 2, 3, 4]
        """
        work_id_str = str(context.work_id)
        if len(work_id_str) != 7:
            self.node.get_logger().error(f"❌ work_id 格式錯誤: {context.work_id}")
            return None, None

        try:
            station = int(work_id_str[4:6])  # 取第5-6位數字
            self.node.get_logger().info(f"從 work_id {context.work_id} 解析 station: {station}")

            # Boxout transfer: Station 01 → ports [1, 2, 3, 4]
            # 預留擴展: Station 02 → ports [5, 6, 7, 8] (未來可能需要)
            base_port = (station - 1) * 4
            available_ports = [base_port + 1, base_port + 2, base_port + 3, base_port + 4]

            self.node.get_logger().info(
                f"Station {station:02d} 對應 boxout transfer ports: {available_ports}")

            return station, available_ports
        except (ValueError, IndexError) as e:
            self.node.get_logger().error(f"❌ 解析 work_id 失敗: {e}")
            return None, None

    def _calculate_port_ids(self, selected_ports):
        """計算選定port組合的所有4個PORT ID

        Args:
            selected_ports: port numbers列表 [1, 2, 3, 4]

        Returns:
            tuple: 4個port IDs (port_id_1, port_id_2, port_id_3, port_id_4)
        """
        if not selected_ports or len(selected_ports) != 4:
            return None, None, None, None
        return (self.port_address + selected_ports[0],
                self.port_address + selected_ports[1],
                self.port_address + selected_ports[2],
                self.port_address + selected_ports[3])

    def _update_context_states(self, context: RobotContext):
        """更新context中的狀態"""
        if not self.search_eqp_signal_ok:
            return

        # 上層狀態
        context.boxout_up_both_empty = not self.port_carriers[0] and not self.port_carriers[1]
        context.boxout_up_left_empty = not self.port_carriers[0] and self.port_carriers[1]
        context.boxout_up_right_empty = self.port_carriers[0] and not self.port_carriers[1]

        # 下層狀態
        context.boxout_down_both_empty = not self.port_carriers[2] and not self.port_carriers[3]
        context.boxout_down_left_empty = not self.port_carriers[2] and self.port_carriers[3]
        context.boxout_down_right_empty = self.port_carriers[2] and not self.port_carriers[3]

    def _validate_eqp_states(self, context: RobotContext):
        """驗證所有4個Port的 Carrier 查詢結果與 EQP 狀態的一致性（PUT 操作空位檢查）"""
        validation_passed = True
        validation_errors = []

        # 存儲所有4個carrier_id以便統一驗證
        carrier_ids = [
            self.carrier_id_port1,
            self.carrier_id_port2,
            self.carrier_id_port3,
            self.carrier_id_port4
        ]

        # 逐一驗證所有4個ports
        for i, (port_number, carrier_id, eqp_state) in enumerate(zip(
            self.workstation_ports,
            carrier_ids,
            self.port_carriers
        )):
            port_id = self.port_address + port_number

            # PUT 操作：carrier_id 應為 None (空位)，EQP 狀態應為 False (無貨)
            if carrier_id is None:
                if eqp_state:
                    # 不一致：Carrier 顯示空位，但 EQP 顯示有貨
                    validation_passed = False
                    validation_errors.append(
                        f"PORT {port_number} (ID: {port_id}): Carrier查詢顯示空位但EQP狀態顯示有貨")
                    self.node.get_logger().error(
                        f"❌ 資料不一致 - PORT {port_number} (ID: {port_id}): "
                        f"Carrier查詢=空位(None), EQP狀態={eqp_state}(有貨)")
                else:
                    # 一致：都顯示空位
                    self.node.get_logger().info(
                        f"✅ PORT {port_number} (ID: {port_id}) 驗證通過: "
                        f"Carrier查詢=空位(None), EQP狀態={eqp_state}(無貨)")
            else:
                # carrier_id 不為 None，表示有貨物，無法執行 PUT 操作
                validation_passed = False
                validation_errors.append(
                    f"PORT {port_number} (ID: {port_id}): 有貨物(carrier_id={carrier_id})，無法執行PUT操作")
                self.node.get_logger().error(
                    f"❌ PORT {port_number} (ID: {port_id}): "
                    f"carrier_id={carrier_id}，EQP狀態={eqp_state}")

        return validation_passed, validation_errors

    def _handle_port_selection(self, context: RobotContext):
        """處理port選擇邏輯 - 從 work_id 解析並驗證所有4個ports"""
        if self.check_ok or not self.search_eqp_signal_ok:
            return

        # 從 work_id 解析 station 和 ports
        station, available_ports = self._extract_station_from_work_id(context)

        if station is None or not available_ports:
            self.node.get_logger().error("❌ 無法從 work_id 解析 station 和 ports")
            self._reset_state()
            return

        # 保存解析結果
        self.workstation_ports = available_ports

        # 檢查所有4個ports是否都為空
        port1_empty = not self.port_carriers[0]
        port2_empty = not self.port_carriers[1]
        port3_empty = not self.port_carriers[2]
        port4_empty = not self.port_carriers[3]

        if port1_empty and port2_empty and port3_empty and port4_empty:
            selected = True
            priority_msg = "所有4個port都是空的，確認完成"
        else:
            selected = False
            occupied_ports = []
            if not port1_empty:
                occupied_ports.append(available_ports[0])
            if not port2_empty:
                occupied_ports.append(available_ports[1])
            if not port3_empty:
                occupied_ports.append(available_ports[2])
            if not port4_empty:
                occupied_ports.append(available_ports[3])
            priority_msg = f"有貨物的port: {occupied_ports}，無法執行任務"

        if selected:
            # 保存選定的4個port組合
            self.check_ok = True

            # ✅ 將4個ports分成兩組（兩次取放操作）
            # Station 01: [1,2,3,4] → [[1,2], [3,4]]
            # 保存 port IDs 而非 port numbers
            port_ids = [self.port_address + p for p in available_ports]
            port_groups = [
                [port_ids[0], port_ids[1]],  # 第1次: 前2個port IDs
                [port_ids[2], port_ids[3]]   # 第2次: 後2個port IDs
            ]

            # 初始化兩次取放循環控制（保存到 self 以便後續使用）
            self.port_groups = port_groups

            self.node.get_logger().info(f"✅ {priority_msg}")
            self.node.get_logger().info(
                f"📋 初始化兩次取放操作："
                f"\n  第1次: 從 AGV ports [1,2] 放到 boxout ports {port_groups[0]}"
                f"\n  第2次: 從 AGV ports [3,4] 放到 boxout ports {port_groups[1]}")
            self.node.get_logger().info(f"準備查詢所有4個port的 Carrier 驗證")
        else:
            self.node.get_logger().warn(f"⚠️ {priority_msg}")
            self.node.get_logger().warn(
                f"❌ Boxout Transfer 未滿足所有port都是空的條件，無法執行 PUT 操作")
            self._reset_state()
            return

    def _handle_step_operation(self, context: RobotContext, step_name: str, operation_func, success_attr: str, failed_attr: str, next_step: int):
        """統一處理步驟操作的邏輯"""
        if not self.sent:
            operation_func()
            self.sent = True

        if getattr(self.hokuyo_dms_8bit_1, success_attr):
            self.node.get_logger().info(f"✅{step_name}成功")
            setattr(self.hokuyo_dms_8bit_1, success_attr, False)
            self.sent = False
            self.step = next_step
        elif getattr(self.hokuyo_dms_8bit_1, failed_attr):
            self.node.get_logger().error(f"❌{step_name}失敗")
            setattr(self.hokuyo_dms_8bit_1, failed_attr, False)
            self.sent = False
        else:
            self.node.get_logger().info(f"⏳等待{step_name}")

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

        # 查詢所有4個PORT的 Carrier
        if self.check_ok and not self.carrier_query_sended and self.workstation_ports:
            port_id_1, port_id_2, port_id_3, port_id_4 = self._calculate_port_ids(self.workstation_ports)
            self.node.get_logger().info(f"查詢 Boxout Transfer Carrier：")
            self.node.get_logger().info(f"  PORT {self.workstation_ports[0]} (ID: {port_id_1})")
            self.node.get_logger().info(f"  PORT {self.workstation_ports[1]} (ID: {port_id_2})")
            self.node.get_logger().info(f"  PORT {self.workstation_ports[2]} (ID: {port_id_3})")
            self.node.get_logger().info(f"  PORT {self.workstation_ports[3]} (ID: {port_id_4})")
            self.carrier_query_client.search_carrier_port_id(
                port_id_min=port_id_1, port_id_max=port_id_4, callback=self.carrier_callback)
            self.carrier_query_sended = True

        # 處理Carrier查詢結果
        if self.check_ok and self.carrier_query_success:
            # 執行 EQP 狀態驗證
            self.node.get_logger().info("開始驗證所有4個port的 Carrier 與 EQP 狀態一致性")
            validation_passed, validation_errors = self._validate_eqp_states(context)

            if validation_passed:
                # EQP 狀態驗證通過，初始化循環控制變數並執行 PUT 操作
                self.node.get_logger().info("✅ 所有4個port EQP 狀態驗證通過，可以執行 Boxout Transfer 操作")

                # 初始化兩次取放循環控制變數到 context
                context.take_put_port_groups = self.port_groups
                context.take_put_cycle_count = 0
                context.take_put_current_batch = [1, 2]  # AGV port numbers [1,2]
                context.take_put_max_cycles = 2

                self.node.get_logger().info(
                    f"✅ 初始化循環控制變數："
                    f"\n  port_groups: {context.take_put_port_groups}"
                    f"\n  cycle_count: {context.take_put_cycle_count}"
                    f"\n  current_batch: {context.take_put_current_batch}"
                    f"\n  max_cycles: {context.take_put_max_cycles}")

                self._handle_8bit_steps(context)
            else:
                # EQP 狀態驗證失敗，重置狀態
                self.node.get_logger().error("❌ Boxout Transfer EQP 狀態驗證失敗:")
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
                                            lambda: self.hokuyo_dms_8bit_1.write_valid(
                                                "1"),
                                            "valid_success", "valid_failed", self.WRITE_PORT_NUMBER)

            case self.WRITE_PORT_NUMBER:
                # 根據當前循環次數動態獲取 boxout transfer port number
                # port_groups 存儲的是 port IDs，需要轉換為 port numbers
                port_id = context.take_put_port_groups[context.take_put_cycle_count][0]
                port_number = port_id - self.port_address  # 例如：2021 - 2020 = 1
                self._handle_step_operation(context, "port number寫入",
                                            lambda: self.hokuyo_dms_8bit_1.write_port_number(port_number),
                                            "port_number_success", "port_number_failed", self.WAIT_LOAD_REQ)

            case self.WAIT_LOAD_REQ:
                if self.hokuyo_dms_8bit_1.load_req:
                    self.node.get_logger().info("✅收到load_req")
                    self.step = self.WRITE_TR_REQ
                else:
                    self.node.get_logger().info("⏳等待load_req")

            case self.WRITE_TR_REQ:
                self._handle_step_operation(context, "tr_req寫入",
                                            lambda: self.hokuyo_dms_8bit_1.write_tr_req(
                                                "1"),
                                            "tr_req_success", "tr_req_failed", self.WAIT_READY)

            case self.WAIT_READY:
                if self.hokuyo_dms_8bit_1.ready:
                    self.node.get_logger().info("✅收到ready")
                    self.step = self.IDLE
                    from unloader_agv.robot_states.put_boxout_transfer.agv_port_check_have_state import AgvPortCheckHaveState
                    context.set_state(AgvPortCheckHaveState(self.node))
                else:
                    self.node.get_logger().info("⏳等待ready")
