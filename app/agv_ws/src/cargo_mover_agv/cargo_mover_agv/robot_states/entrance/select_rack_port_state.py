from agv_base.states.state import State
from db_proxy.carrier_query_client import CarrierQueryClient
from rclpy.node import Node
from cargo_mover_agv.robot_context import RobotContext  # 新增的匯入
from agv_base.robot import Robot
from agv_base.hokuyo_dms_8bit import HokuyoDMS8Bit

from cargo_mover_agv.robot_states.base_robot_state import BaseRobotState


class SelectRackPortState(BaseRobotState):

    def __init__(self, node: Node):
        super().__init__(node)
        self.carrier_query_client = CarrierQueryClient(node)
        self._reset_query_state()

    def _reset_query_state(self):
        """重置查詢狀態"""
        self.response_ok = False
        self.sent = False
        self.no_carrier = False
        self.carrier_id = None
        self.max_rack_index = 0
        self.min_rack_index = 0

    def enter(self):
        self.node.get_logger().info("Robot Entrance 目前狀態: SelectRackPort")
        self._reset_query_state()
        self._reset_common_state()

    def leave(self):
        self.node.get_logger().info("Robot Entrance 離開 SelectRackPort 狀態")

    def carrier_callback(self, response):
        self.response_ok = response.success
        self.no_carrier = CarrierQueryClient.no_carrier(response)
        self.carrier_id = CarrierQueryClient.carrier_min_rack_index_carrier_id(
            response)
        self.max_rack_index = CarrierQueryClient.carrier_max_rack_index(
            response)
        self.min_rack_index = CarrierQueryClient.carrier_min_rack_index(
            response)

    def handle(self, context: RobotContext):
        self._handle_hokuyo_input()
        self._print_separator()

        self.node.get_logger().info("Robot Entrance SelectRackPort 狀態")

        # 執行查詢
        if not self.response_ok and not self.sent:
            self.carrier_query_client.search_carrier_rack_id(
                rack_id=123, callback=self.carrier_callback)
            self.sent = True
            return

        if not self.response_ok:
            return

        context.get_rack_port = self.min_rack_index

        # 處理查詢結果
        if context.get_rack_port and context.get_rack_port != 0:
            self._log_success_info(context)
            from .rack_vision_position_state import RackVisionPositionState
            context.set_state(RackVisionPositionState(self.node))
        elif self.no_carrier:
            self.node.get_logger().info("⚠️ RACK上沒有Carrier")
            from cargo_mover_agv.robot_states.complete_state import CompleteState
            context.set_state(CompleteState(self.node))
            context.carrier_id = 0

    def _log_success_info(self, context):
        """記錄成功資訊"""
        self.node.get_logger().info(f"RACK PORT: {context.get_rack_port}")
        self.node.get_logger().info("✅取得RACK PORT成功")
        self.node.get_logger().info(f"📊 最大 Rack Index: {self.max_rack_index}")
        self.node.get_logger().info(f"📊 最小 Rack Index: {self.min_rack_index}")
        self.node.get_logger().info(f"📦 Carrier ID: {self.carrier_id}")
        context.carrier_id = self.carrier_id
