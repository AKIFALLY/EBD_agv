from agv_base.states.state import State
from db_proxy.carrier_query_client import CarrierQueryClient
from db_proxy.eqp_signal_query_client import EqpSignalQueryClient
from rclpy.node import Node
from cargo_mover_agv.robot_context import RobotContext  # 新增的匯入
from std_msgs.msg import Bool  # 匯入 ROS 2 的 Bool 訊息型態
from agv_base.hokuyo_dms_8bit import HokuyoDMS8Bit


class TransferCheckHaveState(State):
    RobotContext.boxout_up_both_have = True

    # 8 BIT STEP
    IDLE = 0
    WRITE_VAILD = 1
    WRITE_PORT_NUMBER = 2
    WAIT_UNLOAD_REQ = 3
    WRITE_TR_REQ = 4
    WAIT_READY = 5

    PORT_ADDRESS = 2020
    EQP_ID = 202

    def __init__(self, node: Node):
        super().__init__(node)
        self.hokuyo_dms_8bit_1: HokuyoDMS8Bit = self.node.hokuyo_dms_8bit_1
        self.step = self.IDLE
        self.eqp_signal_query_client = EqpSignalQueryClient(node)
        self.carrier_query_client = CarrierQueryClient(node)

        # 狀態標誌
        self.check_ok = False
        self.sent = False
        self.queries_completed = {
            'carrier': False,
            'eqp_signal': False,
            'hokuyo_input': False
        }

        # 查詢結果
        self.earliest_carrier = None
        self.select_boxin_port = 0
        self.port_have_cargo = False

    def _reset_state(self):
        """重置所有狀態變數"""
        self.check_ok = False
        self.sent = False
        self.queries_completed = {'carrier': False,
                                  'eqp_signal': False, 'hokuyo_input': False}
        self.earliest_carrier = None
        self.select_boxin_port = 0
        self.port_have_cargo = False

    def enter(self):
        self.node.get_logger().info("Robot Exit 目前狀態: TranferCheckHave")
        self._reset_state()

    def leave(self):
        self.node.get_logger().info("Robot Exit 離開 TranferCheckHave 狀態")
        self._reset_state()

    def carrier_query_callback(self, response):
        """處理 carrier 查詢回應"""
        if not (response and response.success and response.datas):
            self.node.get_logger().info("❌ Carrier 查詢失敗或沒有資料")
            return

        # 找出最早的 carrier
        self.earliest_carrier = min(response.datas, key=lambda c: c.updated_at)
        self.select_boxin_port = self.earliest_carrier.port_id - self.PORT_ADDRESS
        self.queries_completed['carrier'] = True

        self.node.get_logger().info(
            f"✅ 找到最早的 Carrier: port_id={self.earliest_carrier.port_id}, "
            f"carrier_id={self.earliest_carrier.id}")

    def eqp_signal_query_callback(self, response):
        """處理設備訊號查詢回應"""
        if response and response.success:
            self.port_have_cargo = EqpSignalQueryClient.eqp_signal_port(
                response, self.earliest_carrier.port_id)
            self.queries_completed['eqp_signal'] = True
            self.node.get_logger().info(
                f"Port {self.earliest_carrier.port_id} 有無貨: {self.port_have_cargo}")
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

    def handle(self, context: RobotContext):
        # 執行查詢
        if not self.queries_completed['carrier']:
            self.carrier_query_client.search_carrier_port_id(
                port_id_min=self.PORT_ADDRESS + 1,
                port_id_max=self.PORT_ADDRESS + 4,
                callback=self.carrier_query_callback
            )

        if self.queries_completed['carrier'] and not self.queries_completed['eqp_signal']:
            self.eqp_signal_query_client.search_eqp_signal_eqp_id(
                self.EQP_ID, self.eqp_signal_query_callback)

        # 更新 Hokuyo Input
        if not self.queries_completed['hokuyo_input']:
            self.hokuyo_dms_8bit_1.update_hokuyo_input()
            if self.hokuyo_dms_8bit_1.hokuyo_input_success:
                self.node.get_logger().info("Hokuyo Input 更新成功")
                self.hokuyo_dms_8bit_1.hokuyo_input_success = False
                self.queries_completed['hokuyo_input'] = True
            elif self.hokuyo_dms_8bit_1.hokuyo_input_failed:
                self.node.get_logger().info("Hokuyo Input 更新失敗")
                self.hokuyo_dms_8bit_1.hokuyo_input_failed = False

        print("🔶" + "=" * 73 + "🔶")

        # 檢查是否可以繼續
        if not self.check_ok and self.queries_completed['eqp_signal']:
            if self.port_have_cargo and 1 <= self.select_boxin_port <= 4:
                context.boxout_number = self.select_boxin_port
                context.get_boxout_port = self.select_boxin_port
                context.carrier_id = self.earliest_carrier.id
                self.check_ok = True
                self.node.get_logger().info(
                    f"Robot Exit TranferCheckHave 狀態: 選定 PORT{self.select_boxin_port}")
            else:
                self.node.get_logger().info("Robot Exit TranferCheckHave 狀態: 選定的 port 沒有貨物或無效")
                # 重新查詢
                self.queries_completed['carrier'] = False
                self.queries_completed['eqp_signal'] = False

        # 執行 8-bit 步驟
        if self.check_ok:
            match self.step:
                case self.IDLE:
                    self.step = self.WRITE_VAILD
                    self.sent = False
                case self.WRITE_VAILD:
                    self._handle_hokuyo_write(
                        "write_vaild", "1", "vaild_success", "vaild_failed", self.WRITE_PORT_NUMBER)
                case self.WRITE_PORT_NUMBER:
                    self._handle_hokuyo_write("write_port_number", context.boxout_number,
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
                        from cargo_mover_agv.robot_states.exit.select_rack_port_state import SelectRackPortState
                        context.set_state(SelectRackPortState(self.node))
                    else:
                        self.node.get_logger().info("⏳等待ready")
