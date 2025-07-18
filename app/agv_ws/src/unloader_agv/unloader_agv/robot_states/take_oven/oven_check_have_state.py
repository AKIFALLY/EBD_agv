from db_proxy.carrier_query_client import CarrierQueryClient
from db_proxy.eqp_signal_query_client import EqpSignalQueryClient
from rclpy.node import Node
from unloader_agv.robot_context import RobotContext
from agv_base.hokuyo_dms_8bit import HokuyoDMS8Bit
from unloader_agv.robot_states.base_robot_state import BaseRobotState


class OvenCheckHaveState(BaseRobotState):
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

        # 動態計算 port_address 和 eqp_id (烘乾機)
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
        self.port_carriers = [False] * 8  # 烘乾機八個port的狀態
        self.carrier_id_min = None  # 存儲選定組合第一個 port 的 carrier_id
        self.carrier_id_max = None  # 存儲選定組合第二個 port 的 carrier_id
        self.workstation_ports = None  # 存儲選定的 PORT 組合 (port1, port2)
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
            # 解析烘乾機port狀態 (port 1-8) - 使用迴圈簡化
            for i in range(8):
                self.port_carriers[i] = getattr(response, f'oven_port{i+1}')

            self.search_eqp_signal_ok = True
            self.sent = False
        else:
            self.node.get_logger().error(f"❌ EQP信號查詢失敗: {response.message}")

    def carrier_callback(self, response):
        """Carrier查詢回調 - 處理選定 PORT 組合查詢結果"""
        self.carrier_query_success = response.success
        if response.success:
            self.node.get_logger().info(f"✅ 烘乾機 {self.selected_pair_name} Carrier查詢成功")

            # 計算選定 port 組合的兩個 PORT ID
            if self.workstation_ports:
                port_id_min = self.port_address + self.workstation_ports[0]
                port_id_max = self.port_address + self.workstation_ports[1]

                # 使用 CarrierQueryClient 的靜態方法獲取對應 port 的 carrier_id
                self.carrier_id_min = CarrierQueryClient.carrier_port_id_carrier_id(
                    response, port_id_min)
                self.carrier_id_max = CarrierQueryClient.carrier_port_id_carrier_id(
                    response, port_id_max)

                # 記錄查詢結果
                if self.carrier_id_min is not None:
                    self.node.get_logger().info(
                        f"PORT {self.workstation_ports[0]} 有 Carrier ID: {self.carrier_id_min}")
                else:
                    self.node.get_logger().debug(f"PORT {self.workstation_ports[0]} 沒有 Carrier")

                if self.carrier_id_max is not None:
                    self.node.get_logger().info(
                        f"PORT {self.workstation_ports[1]} 有 Carrier ID: {self.carrier_id_max}")
                else:
                    self.node.get_logger().debug(f"PORT {self.workstation_ports[1]} 沒有 Carrier")

                # 保持向後兼容性，設定主要的 carrier_id
                self.carrier_id = self.carrier_id_min if self.carrier_id_min is not None else self.carrier_id_max
        else:
            self.node.get_logger().error(
                f"❌ 烘乾機 {self.selected_pair_name} Carrier查詢失敗: {response.message}")

    def _update_context_states(self, context: RobotContext):
        """更新context中的狀態"""
        if not self.search_eqp_signal_ok:
            return

        # 設定 oven port 狀態 (使用迴圈簡化)
        for i in range(8):
            setattr(context, f'oven_port{i+1}', self.port_carriers[i])

    def _extract_workstation_and_ports_from_work_id(self, _: RobotContext):
        """從 work_id 中提取工作站編號並推算對應的 PORT 範圍"""
        try:
            work_id = self.node.work_id
            work_id_str = str(work_id)

            # work_id 格式: room_id + OVEN + workstation_number + TAKE
            # 例如: room_id=1, OVEN="06", workstation="01", TAKE="01" -> 1060101
            # 提取倒數第4和第3位數字作為工作站編號
            if len(work_id_str) >= 4:
                workstation_str = work_id_str[-4:-2]  # 提取工作站編號部分
                workstation_number = int(workstation_str)

                # 根據工作站編號推算對應的 PORT 範圍
                # 工作站 1：PORT 1+2 (上層), PORT 5+6 (下層)
                # 工作站 2：PORT 3+4 (上層), PORT 7+8 (下層)
                if workstation_number == 1:
                    ports = [1, 2, 5, 6]  # 工作站 1 管理的所有 port
                    self.node.get_logger().info(
                        f"從 work_id {work_id} 解析出工作站 {workstation_number}，對應 PORT {ports}")
                    return workstation_number, ports
                elif workstation_number == 2:
                    ports = [3, 4, 7, 8]  # 工作站 2 管理的所有 port
                    self.node.get_logger().info(
                        f"從 work_id {work_id} 解析出工作站 {workstation_number}，對應 PORT {ports}")
                    return workstation_number, ports
                else:
                    self.node.get_logger().error(f"工作站編號 {workstation_number} 超出範圍 (1-2)")
                    return None, None
            else:
                self.node.get_logger().error(f"work_id {work_id} 格式不正確，無法解析工作站編號")
                return None, None
        except Exception as e:
            self.node.get_logger().error(f"解析 work_id 時發生錯誤: {e}")
            return None, None

    def _get_select_port_from_physical_port(self, port_number):
        """根據實際 PORT 號碼獲取對應的 select_port 值"""
        # 烘乾機 port 到 select_port 的對應關係：
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
        """驗證 Carrier 查詢結果與 EQP 狀態的一致性"""
        validation_passed = True
        validation_errors = []

        # 驗證第一個 port (carrier_id_min)
        if self.carrier_id_min is not None:
            port_number = self.workstation_ports[0]
            eqp_state = getattr(context, f'oven_port{port_number}')
            if not eqp_state:
                validation_passed = False
                validation_errors.append(f"PORT{port_number}: Carrier查詢有貨但EQP狀態顯示無貨")
                self.node.get_logger().error(
                    f"❌ 資料不一致 - PORT{port_number}: Carrier ID={self.carrier_id_min}, EQP狀態={eqp_state}")
            else:
                self.node.get_logger().info(
                    f"✅ PORT{port_number} 驗證通過: Carrier ID={self.carrier_id_min}, EQP狀態={eqp_state}")

        # 驗證第二個 port (carrier_id_max)
        if self.carrier_id_max is not None:
            port_number = self.workstation_ports[1]
            eqp_state = getattr(context, f'oven_port{port_number}')
            if not eqp_state:
                validation_passed = False
                validation_errors.append(f"PORT{port_number}: Carrier查詢有貨但EQP狀態顯示無貨")
                self.node.get_logger().error(
                    f"❌ 資料不一致 - PORT{port_number}: Carrier ID={self.carrier_id_max}, EQP狀態={eqp_state}")
            else:
                self.node.get_logger().info(
                    f"✅ PORT{port_number} 驗證通過: Carrier ID={self.carrier_id_max}, EQP狀態={eqp_state}")

        return validation_passed, validation_errors

    def _get_selected_carrier_info(self):
        """獲取選定的 carrier 資訊"""
        if self.carrier_id_min is not None:
            return self.workstation_ports[0], self.carrier_id_min
        else:
            return self.workstation_ports[1], self.carrier_id_max

    def _handle_port_selection(self, context: RobotContext):
        """處理port選擇邏輯 - 從 work_id 解析工作站編號並選擇有貨物的 PORT 組合"""
        if self.search_eqp_signal_ok and not self.check_ok:
            # 從 work_id 中解析工作站編號和對應的 PORT 範圍
            workstation_number, available_ports = self._extract_workstation_and_ports_from_work_id(
                context)
            if workstation_number is None or available_ports is None:
                self.node.get_logger().error("無法從 work_id 解析工作站編號，重置狀態")
                self._reset_state()
                return

            # 根據工作站編號定義 port 組合和對應的 select_port 值
            if workstation_number == 1:
                # 工作站 1：port1+2, port5+6
                port_pairs = [(1, 2), (5, 6)]
                select_ports = [1, 5]  # 對應的 select_port 值
                pair_names = ["port1+2", "port5+6"]
            elif workstation_number == 2:
                # 工作站 2：port3+4, port7+8
                port_pairs = [(3, 4), (7, 8)]
                select_ports = [3, 7]  # 對應的 select_port 值
                pair_names = ["port3+4", "port7+8"]
            else:
                self.node.get_logger().error(f"不支援的工作站編號: {workstation_number}")
                self._reset_state()
                return

            # 第一優先：找出兩個 port 都有貨物的組合
            selected_index = None
            for i, (port1, port2) in enumerate(port_pairs):
                port1_has_cargo = self.port_carriers[port1 - 1]  # port 1-8 對應 index 0-7
                port2_has_cargo = self.port_carriers[port2 - 1]

                self.node.get_logger().debug(
                    f"檢查工作站{workstation_number} {pair_names[i]}: PORT{port1}={port1_has_cargo}, PORT{port2}={port2_has_cargo}")

                # 第一優先：兩個 port 都有貨物
                if port1_has_cargo and port2_has_cargo:
                    selected_index = i
                    self.node.get_logger().info(
                        f"✅ 第一優先選擇工作站{workstation_number} {pair_names[i]}，兩個 port 都有貨物")
                    break

            # 第二優先：找出至少有一個 port 有貨物的組合
            if selected_index is None:
                for i, (port1, port2) in enumerate(port_pairs):
                    port1_has_cargo = self.port_carriers[port1 - 1]
                    port2_has_cargo = self.port_carriers[port2 - 1]

                    # 第二優先：至少有一個 port 有貨物
                    if port1_has_cargo or port2_has_cargo:
                        selected_index = i
                        self.node.get_logger().info(
                            f"✅ 第二優先選擇工作站{workstation_number} {pair_names[i]}，至少有一個 port 有貨物")
                        break

            if selected_index is not None:
                # 保存選定的 port 組合和相關資訊
                self.workstation_ports = port_pairs[selected_index]
                self.selected_pair_name = pair_names[selected_index]
                self.selected_port = select_ports[selected_index]
                self.check_ok = True
                self.node.get_logger().info(
                    f"✅ 工作站{workstation_number} 選擇 {self.selected_pair_name} (select_port={self.selected_port})，準備查詢 Carrier 驗證")
            else:
                self.node.get_logger().warn(
                    f"❌ 工作站{workstation_number} 的所有 port 組合都沒有貨物，無法執行 TAKE 操作")
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

        # 查詢選定 PORT 組合的 Carrier
        if self.check_ok and not self.carrier_query_sended and self.workstation_ports:
            port_id_min, port_id_max = self._calculate_port_ids(self.workstation_ports)
            self.node.get_logger().info(
                f"查詢烘乾機 {self.selected_pair_name} Carrier：PORT {self.workstation_ports[0]}-{self.workstation_ports[1]} (ID: {port_id_min}-{port_id_max})")
            self.carrier_query_client.search_carrier_port_id(
                port_id_min=port_id_min, port_id_max=port_id_max, callback=self.carrier_callback)
            self.carrier_query_sended = True

        # 處理 Carrier 查詢結果
        if self.carrier_query_sended and self.carrier_query_success and self.workstation_ports:
            port_id_min, port_id_max = self._calculate_port_ids(self.workstation_ports)

            # 檢查兩個 port 是否至少有一個有 carrier（適合 TAKE 操作的有貨檢查）
            if self.carrier_id_min is not None or self.carrier_id_max is not None:
                # 至少有一個 port 有貨，進行 EQP 信號狀態驗證
                self.node.get_logger().info(
                    f"{self.selected_pair_name} 組合 {port_id_min}-{port_id_max} 至少有一個port有貨物，開始 EQP 狀態驗證。")

                # 執行 EQP 狀態驗證
                validation_passed, validation_errors = self._validate_eqp_states(context)

                if validation_passed:
                    # EQP 狀態驗證通過，可以執行 TAKE 操作
                    self.node.get_logger().info(
                        f"✅ {self.selected_pair_name} EQP 狀態驗證通過，可以執行烘乾機操作")

                    # 獲取選定的 carrier 資訊
                    selected_physical_port, selected_carrier_id = self._get_selected_carrier_info()
                    self.node.get_logger().info(
                        f"選擇烘乾機 PORT {selected_physical_port}，Carrier ID: {selected_carrier_id}")

                    # 設定 context 變數
                    context.get_oven_port = self.selected_port
                    context.carrier_id[0] = self.carrier_id_min
                    context.carrier_id[1] = self.carrier_id_max

                    self.node.get_logger().info(
                        f"烘乾機 {self.selected_pair_name} 檢查完成 (select_port={self.selected_port})，進入下一個狀態")
                    self._handle_8bit_steps(context)
                else:
                    # EQP 狀態驗證失敗，重置狀態
                    self.node.get_logger().error(f"❌ {self.selected_pair_name} EQP 狀態驗證失敗:")
                    for error in validation_errors:
                        self.node.get_logger().error(f"   - {error}")
                    self.node.get_logger().error("Carrier 查詢結果與 EQP 硬體信號狀態不一致，重置狀態")
                    self._reset_state()
            else:
                # 兩個 port 都沒有貨，無法執行 TAKE 操作
                self.node.get_logger().error(
                    f"Carrier 查詢成功，{self.selected_pair_name} 兩個 Port 都沒有貨物")
                self.node.get_logger().error(
                    f"{self.selected_pair_name} 組合 {port_id_min}-{port_id_max} 都沒有貨物，無法執行烘乾機操作。")
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
                                                context.get_oven_port),
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
