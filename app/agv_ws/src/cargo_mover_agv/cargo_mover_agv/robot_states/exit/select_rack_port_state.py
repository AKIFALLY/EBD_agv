from agv_base.states.state import State
from db_proxy.carrier_query_client import CarrierQueryClient
from rclpy.node import Node
from cargo_mover_agv.robot_context import RobotContext
from agv_base.robot import Robot
from agv_base.hokuyo_dms_8bit import HokuyoDMS8Bit

import numpy as np


class SelectRackPortState(State):

    def __init__(self, node: Node):
        super().__init__(node)

        self.hokuyo_dms_8bit_1: HokuyoDMS8Bit = self.node.hokuyo_dms_8bit_1

        self.carrier_query_client = CarrierQueryClient(node)

        self.response = None
        self.max_rack_index = 0
        self.min_rack_index = 0
        self.response_ok = False
        self.sent = False
        self.no_carrier = False
        self.hokuyo_input_updated = False
        self.before_carrier_list = {}
        self.after_carrier_list = {}

    def enter(self):
        self.node.get_logger().info("Robot Exit 目前狀態: SelectRackPort")
        self.response_ok = False
        self.sent = False
        self.hokuyo_input_updated = False

    def leave(self):
        self.node.get_logger().info("Robot Exit 離開 SelectRackPort 狀態")
        self.response_ok = False
        self.sent = False
        self.hokuyo_input_updated = False

    def carrier_callback(self, response):
        self.response = response

        self.response_ok = self.response.success
        self.no_carrier = CarrierQueryClient.no_carrier(response)
        self.carrier_id = CarrierQueryClient.carrier_min_rack_index_carrier_id(
            response)
        self.max_rack_index = CarrierQueryClient.carrier_max_rack_index(
            response)
        self.min_rack_index = CarrierQueryClient.carrier_min_rack_index(
            response)

    def handle(self, context: RobotContext):

        # 更新 Hokuyo Input
        if not self.hokuyo_input_updated:
            self.hokuyo_dms_8bit_1.update_hokuyo_input()
            self.hokuyo_input_updated = True
        if self.hokuyo_dms_8bit_1.hokuyo_input_success:
            self.node.get_logger().info("Hokuyo Input 更新成功")
            self.hokuyo_dms_8bit_1.hokuyo_input_success = False
            self.hokuyo_input_updated = False
        elif self.hokuyo_dms_8bit_1.hokuyo_input_failed:
            self.node.get_logger().info("Hokuyo Input 更新失敗")
            self.hokuyo_dms_8bit_1.hokuyo_input_failed = False
            self.hokuyo_input_updated = False
        else:
            self.node.get_logger().info("等待 Hokuyo Input 更新")

        print("🔶=========================================================================🔶")

        self.node.get_logger().info("Robot Exit SelectRackPort 狀態")
        if not self.response_ok and not self.sent:
            self.carrier_query_client.search_carrier_rack_id(
                rack_id=123, callback=self.carrier_callback)
            self.sent = True

        if self.response_ok and self.min_rack_index != 0:
            self.node.get_logger().info(f"RACK PORT: {context.get_rack_port}")
            self.node.get_logger().info("✅取得RACK PORT成功")
            self.node.get_logger().info(
                f"📊 最大 Rack Index: {self.max_rack_index}")
            self.node.get_logger().info(
                f"📊 最小 Rack Index: {self.min_rack_index}")

            # 實現遞減邏輯
            if self.min_rack_index == 1:
                from cargo_mover_agv.robot_states.complete_state import CompleteState
                context.set_state(CompleteState(self.node))
            else:
                context.get_rack_port = self.min_rack_index - 1
                from cargo_mover_agv.robot_states.exit.rack_vision_position_state import RackVisionPositionState
                context.set_state(RackVisionPositionState(self.node))
        elif self.response_ok and self.no_carrier:
            self.node.get_logger().info("⚠️ RACK上沒有Carrier ")
            context.get_rack_port = 32
