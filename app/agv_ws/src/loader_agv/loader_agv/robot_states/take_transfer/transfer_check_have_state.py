from db_proxy.carrier_query_client import CarrierQueryClient
from db_proxy.eqp_signal_query_client import EqpSignalQueryClient
from rclpy.node import Node
from loader_agv.robot_context import RobotContext
from agv_base.hokuyo_dms_8bit import HokuyoDMS8Bit
from loader_agv.robot_states.base_robot_state import BaseRobotState


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
        self.eqp_signal_query_client = EqpSignalQueryClient(node)
        self.carrier_query_client = CarrierQueryClient(node)

        # 動態計算 port_address 和 eqp_id
        self.port_address = self.node.room_id * 1000 + 10
        self.eqp_id = self.node.room_id * 100 + 1

        # 狀態標誌
        self.check_ok = False
        self.sent = False
        self.queries_completed = {
            'carrier': False,
            'eqp_signal': False
            # 'hokuyo_input' 移除，因為需要持續更新，不追蹤完成狀態
        }

        # 查詢結果
        self.earliest_carrier = None
        self.select_boxin_port = 0
        self.port_have_carrier = False
        self.search_eqp_signal_ok = False
        self.port_carriers = [False, False, False, False]  # Port 1-4 的狀態

    def _reset_state(self):
        """重置所有狀態變數"""
        self.check_ok = False
        self.sent = False
        self.queries_completed = {
            'carrier': False,
            'eqp_signal': False
            # 'hokuyo_input' 移除，因為需要持續更新，不追蹤完成狀態
        }
        self.earliest_carrier = None
        self.select_boxin_port = 0
        self.port_have_carrier = False
        self.search_eqp_signal_ok = False
        self.port_carriers = [False, False, False, False]  # Port 1-4 的狀態

    def enter(self):
        self.node.get_logger().info("Robot Take Transfer 目前狀態: TranferCheckHave")
        self._reset_state()

    def leave(self):
        self.node.get_logger().info("Robot Take Transfer 離開 TranferCheckHave 狀態")
        self._reset_state()

    def carrier_query_callback(self, response):
        """處理 carrier 查詢回應"""
        if not (response and response.success and response.datas):
            self.node.get_logger().error("❌ Carrier 查詢失敗或沒有資料")
            return

        # 找出最早的 carrier
        self.earliest_carrier = min(response.datas, key=lambda c: c.updated_at)
        self.select_boxin_port = self.earliest_carrier.port_id - self.port_address
        self.queries_completed['carrier'] = True

        self.node.get_logger().info(
            f"✅ 找到最早的 Carrier: port_id={self.earliest_carrier.port_id}, "
            f"carrier_id={self.earliest_carrier.id}")

    def eqp_signal_query_callback(self, response):
        """處理設備訊號查詢回應"""
        if response and response.success:
            self.port_have_carrier = EqpSignalQueryClient.eqp_signal_port(
                response, self.earliest_carrier.port_id)
            self.queries_completed['eqp_signal'] = True
            self.search_eqp_signal_ok = True

            # 更新所有 port 的狀態
            for i in range(4):
                port_id = self.port_address + i + 1
                self.port_carriers[i] = EqpSignalQueryClient.eqp_signal_port(response, port_id)

            self.node.get_logger().debug(
                f"Port {self.earliest_carrier.port_id} 有無貨: {self.port_have_carrier}")
            self.node.get_logger().debug(
                f"所有 Port 狀態: {self.port_carriers}")
        else:
            self.node.get_logger().info("❌ EqpSignal 查詢失敗")

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

    def _update_context_states(self, context: RobotContext):
        """更新context中的狀態 - 參考agv_port_check_empty_state.py"""
        if not self.search_eqp_signal_ok:
            return
        # 更新BOXIN_PORT層狀態
        context.boxin_port1 = self.port_carriers[0]
        context.boxin_port2 = self.port_carriers[1]
        context.boxin_port3 = self.port_carriers[2]
        context.boxin_port4 = self.port_carriers[3]

    def _check_take_transfer_continue(self, context: RobotContext):
        """檢查take transfer是否可以繼續的條件"""
        # 確保有足夠的資料進行判斷
        if not self.search_eqp_signal_ok or self.select_boxin_port == 0:
            context.take_transfer_continue = False
            return

        # 當select_boxin_port = 1 且 boxin_port2 = true 時，可以繼續
        if self.select_boxin_port == 1 and context.boxin_port2:
            context.take_transfer_continue = True
            self.node.get_logger().info("✅ Take Transfer 可以繼續: select_boxin_port=1, boxin_port2=True")
        # 當select_boxin_port = 3 且 boxin_port4 = true 時，可以繼續
        elif self.select_boxin_port == 3 and context.boxin_port4:
            context.take_transfer_continue = True
            self.node.get_logger().info("✅ Take Transfer 可以繼續: select_boxin_port=3, boxin_port4=True")
        # 其他情況不能繼續
        else:
            context.take_transfer_continue = False
            self.node.get_logger().info(
                f"❌ Take Transfer 不能繼續: select_boxin_port={self.select_boxin_port}, "
                f"boxin_port2={context.boxin_port2}, boxin_port4={context.boxin_port4}")

    def handle(self, context: RobotContext):
        # 執行查詢
        if not self.queries_completed['carrier']:
            self.carrier_query_client.search_carrier_port_id(
                port_id_min=self.port_address + 1,
                port_id_max=self.port_address + 4,
                callback=self.carrier_query_callback
            )

        if self.queries_completed['carrier'] and not self.queries_completed['eqp_signal']:
            self.eqp_signal_query_client.search_eqp_signal_eqp_id(
                self.eqp_id, self.eqp_signal_query_callback)

        # 更新 Hokuyo Input - 使用統一方法
        self._handle_hokuyo_input()

        # 更新 context 狀態
        self._update_context_states(context)

        # 檢查是否可以繼續
        if not self.check_ok and self.queries_completed['eqp_signal']:
            if self.port_have_carrier and 1 <= self.select_boxin_port <= 4:
                context.boxin_number = self.select_boxin_port
                context.get_boxin_port = self.select_boxin_port
                context.carrier_id = self.earliest_carrier.id

                # 檢查 take_transfer_continue 條件
                self._check_take_transfer_continue(context)

                self.check_ok = True
                self.node.get_logger().info(
                    f"Robot Take Transfer TranferCheckHave 狀態: 選定 PORT{self.select_boxin_port}")
                self.node.get_logger().info(
                    f"Take Transfer Continue: {context.take_transfer_continue}")
            else:
                self.node.get_logger().info("Robot Take Transfer TranferCheckHave 狀態: 選定的 port 沒有貨物或無效")
                # 重新查詢
                self.queries_completed['carrier'] = False
                self.queries_completed['eqp_signal'] = False

        # 執行 8-bit 步驟
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
