from db_proxy.carrier_query_client import CarrierQueryClient
from rclpy.node import Node
from cargo_mover_agv.robot_context import RobotContext
from agv_base.robot import Robot
from agv_base.hokuyo_dms_8bit import HokuyoDMS8Bit
from cargo_mover_agv.robot_states.base_robot_state import BaseRobotState

import numpy as np
import json


class SelectRackPortState(BaseRobotState):

    def __init__(self, node: Node):
        super().__init__(node)

        self.hokuyo_dms_8bit_2: HokuyoDMS8Bit = self.node.hokuyo_dms_8bit_2

        self.carrier_query_client = CarrierQueryClient(node)

        self.response = None
        self.max_rack_index = 0
        self.min_rack_index = 0
        self.response_ok = False
        self.sent = False
        self.no_carrier = False
        # hokuyo_input_updated 已移除，因為需要持續更新
        self.before_carrier_list = {}
        self.after_carrier_list = {}
        self.node = node

    def enter(self):
        self.node.get_logger().info("Robot Exit 目前狀態: SelectRackPort")
        self.response_ok = False
        self.sent = False
        # hokuyo_input_updated 已移除，因為需要持續更新

    def leave(self):
        self.node.get_logger().info("Robot Exit 離開 SelectRackPort 狀態")
        self.response_ok = False
        self.sent = False
        # hokuyo_input_updated 已移除，因為需要持續更新

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

    def update_rack_id(self, context: RobotContext):
        """
        從 task.parameters 中解析 rack_id 並設定到 context 中

        此方法會在每次執行時重新解析 task.parameters，支援 JSON 字串和字典格式，
        並包含完整的錯誤處理和日誌記錄功能。

        Args:
            context (RobotContext): 機器人上下文物件，用於存儲解析得到的 rack_id
        """
        # 解析 task.parameters 並設定 rack_id（每次執行時都重新解析）
        try:
            if self.node.task.parameters:
                # 如果 parameters 是字串，需要解析 JSON
                if isinstance(self.node.task.parameters, str):
                    parameters_dict = json.loads(self.node.task.parameters)
                else:
                    # 如果已經是字典，直接使用
                    parameters_dict = self.node.task.parameters

                # 從 parameters 中取得 rack_id
                rack_id = parameters_dict.get("rack_id")
                if rack_id is not None:
                    context.rack_id = rack_id
                    self.node.get_logger().info(
                        f"✅ 從 task.parameters 取得 rack_id: {context.rack_id}")
                else:
                    context.rack_id = None
                    self.node.get_logger().warn("⚠️ task.parameters 中沒有找到 rack_id，設為 None")
            else:
                context.rack_id = None
                self.node.get_logger().warn("⚠️ task.parameters 為空，rack_id 設為 None")
        except (json.JSONDecodeError, TypeError, AttributeError) as e:
            context.rack_id = None
            self.node.get_logger().error(f"❌ 解析 task.parameters 時發生錯誤: {e}，rack_id 設為 None")

    def handle(self, context: RobotContext):

        # 更新 Hokuyo Input - 使用統一方法
        self._handle_hokuyo_input_exit()

        self.node.get_logger().info("Robot Exit SelectRackPort 狀態")
        if not self.response_ok and not self.sent:
            # 使用從 task.parameters 解析的 rack_id，如果沒有則使用預設值 123
            rack_id_to_use = context.rack_id if context.rack_id is not None else 123
            self.node.get_logger().info(f"🔍 使用 rack_id: {rack_id_to_use} 進行查詢")
            self.carrier_query_client.search_carrier_rack_id(
                rack_id=rack_id_to_use, callback=self.carrier_callback)
            self.sent = True

        if self.response_ok and self.min_rack_index is not None:
            self.node.get_logger().info(f"RACK PORT: {context.get_rack_port}")
            self.node.get_logger().info("✅取得RACK PORT成功")
            self.node.get_logger().info(
                f"📊 最大 Rack Index: {self.max_rack_index}")
            self.node.get_logger().info(
                f"📊 最小 Rack Index: {self.min_rack_index}")

            # 實現遞減邏輯
            if self.min_rack_index == 1:
                self.node.get_logger().info("✅ 已到達最小 Rack Index，Exit 流程完成")
                from cargo_mover_agv.robot_states.complete_state import CompleteState
                context.set_state(CompleteState(self.node))
            else:
                context.get_rack_port = self.min_rack_index - 1
                self.node.get_logger().info(f"🔄 設定下一個 Rack Port: {context.get_rack_port}")
                # 加入 Rack 方向檢查，與 ENTRANCE 流程保持一致
                from cargo_mover_agv.robot_states.exit.rack_vision_position_state import RackVisionPositionState
                context.set_state(RackVisionPositionState(self.node))
        elif self.response_ok and self.no_carrier:
            self.node.get_logger().info("⚠️ RACK上沒有Carrier，Exit, 設定最大 Port 32")
            context.get_rack_port = 32
            self.node.get_logger().info(f"🔄 設定下一個 Rack Port: {context.get_rack_port}")
            from cargo_mover_agv.robot_states.exit.rack_vision_position_state import RackVisionPositionState
            context.set_state(RackVisionPositionState(self.node))
