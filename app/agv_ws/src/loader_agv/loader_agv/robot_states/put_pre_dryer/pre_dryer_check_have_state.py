from db_proxy.carrier_query_client import CarrierQueryClient
from db_proxy.eqp_signal_query_client import EqpSignalQueryClient
from rclpy.node import Node
from loader_agv.robot_context import RobotContext
from agv_base.hokuyo_dms_8bit import HokuyoDMS8Bit
from loader_agv.robot_states.base_robot_state import BaseRobotState
from shared_constants.equipment_stations import EquipmentStations


class PreDryerCheckHaveState(BaseRobotState):

    # 8 BIT STEP
    IDLE = 0
    WRITE_VALID = 1
    WRITE_PORT_NUMBER = 2
    WAIT_LOAD_REQ = 3
    WRITE_TR_REQ = 4
    WAIT_READY = 5

    SELECT_PORT01, SELECT_PORT02, SELECT_PORT03, SELECT_PORT04, SELECT_PORT05, SELECT_PORT06, SELECT_PORT07, SELECT_PORT08, SELECT_NONE = 1, 2, 3, 4, 5, 6, 7, 8, 0

    def __init__(self, node: Node):
        super().__init__(node)
        self.node = node
        self.hokuyo_dms_8bit_1: HokuyoDMS8Bit = self.node.hokuyo_dms_8bit_1
        self.eqp_signal_query_client = EqpSignalQueryClient(node)
        self.carrier_query_client = CarrierQueryClient(node)

        # 動態計算 port_address 和 eqp_id
        self.port_address = self.node.room_id * 1000 + 50
        self.eqp_id = self.node.room_id * 100 + 5

        self._reset_state()

    def _reset_state(self):
        """重置所有狀態變數"""
        self.check_ok = False
        self.step = self.IDLE
        self.sent = False
        self.search_eqp_signal_ok = False
        self.carrier_query_sended = False
        self.carrier_query_success = False
        self.port_carriers = [False] * 8  # 預乾燥有8個port
        self.select_pre_dryer_port = self.SELECT_NONE
        self.carrier_id = None

    def enter(self):
        self.node.get_logger().info(
            "[Station-based 2格] Loader Robot Put PreDryer 目前狀態: PreDryerCheckHave")
        self._reset_state()

    def leave(self):
        self.node.get_logger().info(
            "[Station-based 2格] Loader Robot Put PreDryer 離開 PreDryerCheckHave 狀態")
        self._reset_state()

    def eqp_signal_query_callback(self, response):
        """處理 EqpSignal 查詢回應 - 檢查預乾燥機 8 個端口狀態

        說明：Put Pre-dryer 設備配置
        - 標準設備：4 個 Station，每個 Station 有 2 個 port
        - Station 配置：01(Port 1-2), 03(Port 3-4), 05(Port 5-6), 07(Port 7-8)
        - PUT 操作：需要檢查指定 station 的兩個 port 是否為空
        - 批量處理：一次任務處理2格（1 station = 2 ports）
        """
        for i in range(8):
            self.port_carriers[i] = EqpSignalQueryClient.eqp_signal_port(
                response, self.port_address + i + 1)
            self.node.get_logger().info(
                f"[Station-based 2格] PreDryer Port {i+1:02d} 有無貨: {self.port_carriers[i]}")

        self.search_eqp_signal_ok = True

    def carrier_callback(self, response):
        self.carrier_query_success = response.success
        port_id_target = self.port_address + self.pre_dryer_number
        self.carrier_id = CarrierQueryClient.carrier_port_id_carrier_id(
            response, port_id_target)

    def _update_context_states(self, context: RobotContext):
        """更新context中的狀態"""
        if not self.search_eqp_signal_ok:
            return

        # 設定 pre_dryer port 狀態
        context.pre_dryer_port1 = self.port_carriers[0]
        context.pre_dryer_port2 = self.port_carriers[1]
        context.pre_dryer_port3 = self.port_carriers[2]
        context.pre_dryer_port4 = self.port_carriers[3]
        context.pre_dryer_port5 = self.port_carriers[4]
        context.pre_dryer_port6 = self.port_carriers[5]
        context.pre_dryer_port7 = self.port_carriers[6]
        context.pre_dryer_port8 = self.port_carriers[7]

    def _extract_station_from_work_id(self, context: RobotContext):
        """從 work_id 中提取 station 並映射到 port (使用 EquipmentStations 模組)

        說明：標準設備的 Station-based 設計
        - 標準設備：1 station = 2 ports（Pre-dryer 是標準設備）
        - 4 個 Station：Station 01, 03, 05, 07
        - Station 到 Port 映射：
          * Station 01 → Port 1, 2
          * Station 03 → Port 3, 4
          * Station 05 → Port 5, 6
          * Station 07 → Port 7, 8
        - Work ID 範圍：2050102, 2050302, 2050502, 2050702（PUT 操作）
        - 此方法返回第一個 port 用於初始驗證，完整的 ports 列表用於批量配置
        """
        # 調用基類通用方法
        station, ports = self._extract_station_and_ports_from_work_id(context.work_id)
        if station is None:
            return None

        # 返回第一個 port 用於初始 EQP 驗證
        port_number = ports[0]
        return port_number

    def _handle_step_operation(self, step_name: str, operation_func, success_attr: str, failed_attr: str, next_step: int):
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
        # 1. 驗證 Work ID 格式（7位數）
        work_id_str = str(context.work_id)
        if len(work_id_str) != 7:
            self.node.get_logger().error(
                f"❌ [Station-based 2格] Work ID 格式錯誤: {context.work_id}，"
                f"必須是7位數格式（REESSAA）")
            return

        self._update_context_states(context)

        # 2. 查詢 EQP 信號（只執行一次）
        if not self.search_eqp_signal_ok and not self.sent:
            self.node.get_logger().info(
                f"[Station-based 2格] 查詢預乾燥機端口狀態 (eqp_id={self.eqp_id}), "
                f"Work ID {context.work_id}")
            self.eqp_signal_query_client.search_eqp_signal_eqp_id(
                self.eqp_id, self.eqp_signal_query_callback)
            self.sent = True

        # 3. 更新 Hokuyo Input - 使用統一方法
        self._handle_hokuyo_input()

        print("🔶=========================================================================🔶")

        # 4. 從 work_id 解析 station 並映射到 port，然後進行驗證
        if self.search_eqp_signal_ok and not self.check_ok:
            # 從 work_id 中解析 station 並取得對應的 port
            self.pre_dryer_number = self._extract_station_from_work_id(context)
            if self.pre_dryer_number is None:
                self.node.get_logger().error(
                    f"❌ [Station-based 2格] 無法從 work_id {context.work_id} 解析 station，重置狀態")
                self._reset_state()
                return

            # 檢查解析出的端口號碼是否有效（預乾燥機有8個port）
            if not (1 <= self.pre_dryer_number <= 8):
                self.node.get_logger().error(
                    f"❌ [Station-based 2格] 無效的端口號碼: {self.pre_dryer_number}，"
                    f"有效範圍: 1-8")
                self._reset_state()
                return

            # 重新解析以獲取完整 station 和 ports 資訊
            station, ports = self._extract_station_and_ports_from_work_id(context.work_id)
            self.node.get_logger().info(
                f"✅ [Station-based 2格] Work ID {context.work_id} → "
                f"Pre-dryer Station {station:02d}, Ports {ports} (批量2格)")

            # 使用 EQP 狀態進行驗證：檢查對應的 context.pre_dryer_portX 狀態
            port_eqp_empty = not self.port_carriers[self.pre_dryer_number - 1]

            if port_eqp_empty:
                self.node.get_logger().info(
                    f"✅ [Station-based 2格] 預乾燥端口 {self.pre_dryer_number} EQP 狀態顯示為空，"
                    f"準備查詢 Carrier")
                self.check_ok = True
            else:
                self.node.get_logger().warn(
                    f"❌ [Station-based 2格] 預乾燥端口 {self.pre_dryer_number} EQP 狀態顯示有貨物，"
                    f"無法執行 PUT 操作")
                self._reset_state()
                return

        # 5. 查詢 Carrier（只執行一次）
        if self.check_ok and not self.carrier_query_sended:
            port_id_target = self.port_address + self.pre_dryer_number
            self.node.get_logger().info(
                f"[Station-based 2格] 查詢預乾燥端口 {self.pre_dryer_number} "
                f"(port_id: {port_id_target}) 的 Carrier")
            self.carrier_query_client.search_carrier_port_id(
                port_id_min=port_id_target, port_id_max=port_id_target, callback=self.carrier_callback)
            self.carrier_query_sended = True

        # 6. 處理 Carrier 查詢結果（雙重驗證）
        if self.check_ok and self.carrier_query_success:
            port_id_target = self.port_address + self.pre_dryer_number
            port_eqp_empty = not self.port_carriers[self.pre_dryer_number - 1]

            # 與 Carrier 查詢結果比較：檢查 EQP 狀態與 carrier 查詢結果是否一致（PUT 操作：都應該為空）
            if self.carrier_id is None and port_eqp_empty:
                # 雙重驗證成功：EQP 狀態顯示空，carrier 查詢也顯示空
                self.node.get_logger().info(
                    f"✅ [Station-based 2格] 雙重驗證成功：預乾燥端口 {self.pre_dryer_number} "
                    f"EQP 狀態和 Carrier 查詢都顯示為空")

                # 從 work_id 重新解析以獲取完整的 ports 列表（用於批量配置）
                station, ports = self._extract_station_and_ports_from_work_id(context.work_id)
                if station and len(ports) == 2:
                    # Station-based 批量配置（標準設備：1 station = 2 ports）
                    self.node.get_logger().info(
                        f"✅ [Station-based 2格] 批量放料配置: Station {station:02d}, "
                        f"Ports {ports} (Work ID {context.work_id})")

                    # 設置批量配置變數
                    context.pre_dryer_take_count = 0           # 批量計數器 (0=第1次, 1=第2次)
                    context.pre_dryer_device_ports = ports     # [port1, port2] for station

                    # 設置第一次的初始值
                    context.get_pre_dryer_port = ports[0]
                    self.node.get_logger().info(
                        f"[Station-based 2格] 設定第1次放料: Pre-dryer Port {ports[0]}")
                else:
                    # 單次執行（回退邏輯）
                    context.get_pre_dryer_port = self.pre_dryer_number
                    self.node.get_logger().info(
                        f"[Station-based 2格] 設定 context.get_pre_dryer_port = {self.pre_dryer_number}")

                self._handle_8bit_steps(context)
            elif self.carrier_id is not None:
                # Carrier 查詢顯示有貨物
                self.node.get_logger().warn(
                    f"❌ [Station-based 2格] 預乾燥端口 {self.pre_dryer_number} 有 carrier "
                    f"(ID: {self.carrier_id})，無法執行 PUT 操作")
                self._reset_state()
            elif not port_eqp_empty:
                # EQP 狀態與 carrier 查詢結果不一致
                self.node.get_logger().warn(
                    f"❌ [Station-based 2格] 數據不一致：預乾燥端口 {self.pre_dryer_number} "
                    f"EQP 狀態顯示有貨，但 carrier 查詢顯示空")
                self._reset_state()
            else:
                # 其他未預期的情況
                self.node.get_logger().error(
                    f"❌ [Station-based 2格] 未預期的驗證結果：端口 {self.pre_dryer_number}")
                self._reset_state()

    def _handle_8bit_steps(self, context: RobotContext):
        """處理8bit步驟"""
        match self.step:
            case self.IDLE:
                self.step = self.WRITE_VALID
                self.sent = False

            case self.WRITE_VALID:
                self._handle_step_operation("valid寫入",
                                            lambda: self.hokuyo_dms_8bit_1.write_valid("1"),
                                            "valid_success", "valid_failed", self.WRITE_PORT_NUMBER)

            case self.WRITE_PORT_NUMBER:
                self._handle_step_operation("port number寫入",
                                            lambda: self.hokuyo_dms_8bit_1.write_port_number(
                                                context.get_pre_dryer_port),
                                            "port_number_success", "port_number_failed", self.WAIT_LOAD_REQ)

            case self.WAIT_LOAD_REQ:
                if self.hokuyo_dms_8bit_1.load_req:
                    self.node.get_logger().info("✅收到load_req")
                    self.step = self.WRITE_TR_REQ
                else:
                    self.node.get_logger().info("⏳等待load_req")

            case self.WRITE_TR_REQ:
                self._handle_step_operation("tr_req寫入",
                                            lambda: self.hokuyo_dms_8bit_1.write_tr_req("1"),
                                            "tr_req_success", "tr_req_failed", self.WAIT_READY)

            case self.WAIT_READY:
                if self.hokuyo_dms_8bit_1.ready:
                    self.node.get_logger().info("✅收到ready")
                    self.step = self.IDLE
                    # 預乾燥檢查完成，轉換到 AGV 端口檢查狀態
                    self.node.get_logger().info(
                        f"✅ [Station-based 2格] 預乾燥機檢查完成: 進入 AgvPortCheckHaveState")
                    self.node.get_logger().info(
                        f"檢查 AGV Port 2 和 4 是否有貨（Put Pre-dryer 使用偶數層）")
                    from loader_agv.robot_states.put_pre_dryer.agv_port_check_have_state import AgvPortCheckHaveState
                    context.set_state(AgvPortCheckHaveState(self.node))
                else:
                    self.node.get_logger().info("⏳等待ready")
