from agv_base.states.state import State
from rclpy.node import Node
from cargo_mover_agv.robot_context import RobotContext
from db_proxy.carrier_query_client import CarrierQueryClient
from db_proxy.rack_query_client import RackQueryClient
from db_proxy.agvc_database_client import AGVCDatabaseClient


class CheckRackSideState(State):

    def __init__(self, node: Node):
        super().__init__(node)
        self.step = RobotContext.IDLE

        self.carrier_query_client = CarrierQueryClient(node)
        self.rack_query_client = RackQueryClient(node)
        self.agvc_client = AGVCDatabaseClient(node)
        self.carrier_response_ok = False
        self.rack_response_ok = False
        self.sent = False
        self.max_rack_index = 0
        self.min_rack_index = 0
        self.no_carrier = False
        self.carrier_id = None
        self.rack_direction = None
        self.update_task_success = False
        self.node = node

    def enter(self):
        self.node.get_logger().info("Robot Entrance 目前狀態: CheckRackSide")
        self.carrier_response_ok = False
        self.rack_response_ok = False
        self.sent = False
        self.update_task_success = False

    def leave(self):
        self.node.get_logger().info("Robot Entrance 離開 CheckRackSide 狀態")
        self.carrier_response_ok = False
        self.rack_response_ok = False
        self.sent = False
        self.update_task_success = False

    def carrier_callback(self, response):
        self.response = response

        self.carrier_response_ok = self.response.success
        self.no_carrier = CarrierQueryClient.no_carrier(response)
        self.carrier_id = CarrierQueryClient.carrier_min_rack_index_carrier_id(
            response)
        self.max_rack_index = CarrierQueryClient.carrier_max_rack_index(
            response)
        self.min_rack_index = CarrierQueryClient.carrier_min_rack_index(
            response)

    def rack_callback(self, response):
        # 檢查 response 是否為 None
        if response is None:
            self.node.get_logger().error("❌ Rack 查詢回應為 None")
            self.rack_response = None
            self.rack_response_ok = False
            return

        # 更安全的寫法，避免 IndexError
        rack = response.datas[0] if response and response.datas else None

        self.rack_response = response
        self.node.get_logger().info(f"{self.rack_response}")
        if self.rack_response.success:
            self.rack_response_ok = True
            # 安全地存取 rack.direction，避免 NoneType 錯誤
            if rack is not None:
                self.rack_direction = rack.direction
                self.node.get_logger().info(f"✅ 成功取得 Rack 方向: {self.rack_direction}")
            else:
                self.rack_direction = None
                self.node.get_logger().warn("⚠️ 查詢成功但未找到 Rack 資料，rack_direction 設為 None")
        else:
            self.node.get_logger().warn(
                f"⚠️ Rack 查詢失敗: {self.rack_response.message if hasattr(self.rack_response, 'message') else '未知錯誤'}")
            self.rack_response_ok = False

    def update_task_callback(self, result):
        """處理 update_task 的回調"""
        if result is not None:
            self.node.get_logger().info(
                f"✅ Task 更新成功: {result.success}, {result.message}")
            self.update_task_success = True
        else:
            self.node.get_logger().error("❌ Task 更新失敗")
            self.update_task_success = False

    def handle(self, context: RobotContext):
        self.node.get_logger().info("Robot Entrance CheckRackSide 狀態")

        match self.step:
            case 0:
                if not self.carrier_response_ok and not self.sent:
                    # 使用從 task.parameters 解析的 rack_id，如果沒有則使用預設值 123
                    rack_id_to_use = context.rack_id if context.rack_id is not None else 123
                    self.node.get_logger().info(
                        f"🔍 CheckRackSide 使用 rack_id: {rack_id_to_use} 進行 carrier 查詢")
                    self.carrier_query_client.search_carrier_rack_id(
                        rack_id=rack_id_to_use, callback=self.carrier_callback)
                    self.sent = True
                elif self.sent and self.carrier_response_ok:
                    context.get_rack_port = self.min_rack_index
                    self.sent = False
                    self.step = 1
            case 1:
                if not self.rack_response_ok and not self.sent:
                    # 使用從 task.parameters 解析的 rack_id，如果沒有則使用預設值 123
                    rack_id_to_use = context.rack_id if context.rack_id is not None else 123
                    self.node.get_logger().info(
                        f"🔍 CheckRackSide 使用 rack_id: {rack_id_to_use} 進行 rack 查詢")
                    self.rack_query_client.search_rack_id(
                        rack_id=rack_id_to_use, callback=self.rack_callback)
                    self.sent = True

                elif self.sent and self.rack_response_ok:
                    self.sent = False
                    self.step = 2

            case 2:
                if context.get_rack_port is None and self.no_carrier:
                    self.node.get_logger().info(
                        "沒有找到任何Carrier，完成。")
                    from cargo_mover_agv.robot_states.complete_state import CompleteState
                    context.set_state(CompleteState(self.node))
                    self.step = 0
                else:
                    self.step = 3
            case 3:
                # 檢查 rack_direction 是否為 None
                if self.rack_direction is None:
                    self.node.get_logger().error(
                        f"❌ Rack 方向資料無效 (None)，無法進行方向判斷。Rack Port: {context.get_rack_port}")
                    # 重新查詢 rack 方向資料，而不是進入完成狀態
                    self.node.get_logger().info("🔄 重新查詢 Rack 方向資料...")
                    self.rack_response_ok = False
                    self.sent = False
                    self.rack_direction = None
                    self.step = 1  # 回到 step 1 重新查詢 rack 資料
                elif self.rack_direction == 0 and 1 <= context.get_rack_port <= 16:
                    self.node.get_logger().info(
                        f"Rack方向是A面，不需要調整。Rack Port:{context.get_rack_port}")
                    # 直接進入下一狀態
                    from cargo_mover_agv.robot_states.entrance.transfer_check_empty_state import TransferCheckEmptyState
                    context.set_state(TransferCheckEmptyState(self.node))
                elif self.rack_direction == 180 and 17 <= context.get_rack_port <= 32:
                    self.node.get_logger().info(
                        f"Rack方向是B面，不需要調整。Rack Port:{context.get_rack_port}")
                    # 直接進入下一狀態
                    from cargo_mover_agv.robot_states.entrance.transfer_check_empty_state import TransferCheckEmptyState
                    context.set_state(TransferCheckEmptyState(self.node))
                elif self.rack_direction == 0 and 17 <= context.get_rack_port <= 32:
                    self.node.get_logger().info(
                        f"Rack方向是A面，請旋轉Rack至B面。Rack Port:{context.get_rack_port}")
                    # 轉換到等待旋轉狀態
                    from cargo_mover_agv.robot_states.entrance.wait_rotation_state import WaitRotationState
                    context.set_state(WaitRotationState(self.node))
                elif self.rack_direction == 180 and 1 <= context.get_rack_port <= 16:
                    self.node.get_logger().info(
                        f"Rack方向是B面，請旋轉Rack至A面。Rack Port:{context.get_rack_port}")
                    # 轉換到等待旋轉狀態
                    from cargo_mover_agv.robot_states.entrance.wait_rotation_state import WaitRotationState
                    context.set_state(WaitRotationState(self.node))
                else:
                    self.node.get_logger().error(
                        f"無法辨識Rack方向或Port範圍錯誤。Rack Direction: {self.rack_direction}, Rack Port: {context.get_rack_port}")
