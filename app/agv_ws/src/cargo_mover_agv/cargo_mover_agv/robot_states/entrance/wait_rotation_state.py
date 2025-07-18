from agv_base.states.state import State
from rclpy.node import Node
from cargo_mover_agv.robot_context import RobotContext
from db_proxy.rack_query_client import RackQueryClient


class WaitRotationState(State):

    def __init__(self, node: Node):
        super().__init__(node)
        self.step = 0
        self.rack_query_client = RackQueryClient(node)
        self.rack_response_ok = False
        self.sent = False
        self.rack_direction = None
        self.node = node
        self.rack_response = None

    def enter(self):
        self.node.get_logger().info("Robot Entrance 目前狀態: WaitRotation")
        self.rack_response_ok = False
        self.sent = False

    def leave(self):
        self.node.get_logger().info("Robot Entrance 離開 WaitRotation 狀態")
        self.rack_response_ok = False
        self.sent = False

    def rack_callback(self, response):
        # 檢查 response 是否為 None
        if response is None:
            self.node.get_logger().error("❌ Rack 查詢回應為 None")
            self.rack_response = None
            self.rack_response_ok = False
            self.rack_direction = None
            return

        # 更安全的寫法，避免 IndexError
        rack = response.datas[0] if response and response.datas else None

        self.rack_response = response
        self.node.get_logger().info(f"{self.rack_response}")
        if self.rack_response.success:
            self.rack_response_ok = True
            self.rack_direction = rack.direction
        else:
            self.node.get_logger().warn(
                f"⚠️ Rack 查詢失敗: {self.rack_response.message if hasattr(self.rack_response, 'message') else '未知錯誤'}")
            self.rack_response_ok = False

    def handle(self, context: RobotContext):
        self.node.get_logger().info("Robot Entrance WaitRotation 狀態")

        match self.step:
            case 0:
                # 等待旋轉狀態 - 參考 exit 版本的結構
                self.node.get_logger().info(
                    f"self.rack_response_ok: {self.rack_response_ok}, self.sent: {self.sent}")
                context.rack_photo_up_or_down_buffer = None

                # 持續查詢 rack_id 直到旋轉完成
                if not self.sent:
                    # 使用從 task.parameters 解析的 rack_id，如果沒有則使用預設值 123
                    rack_id_to_use = context.rack_id if context.rack_id is not None else 123
                    self.node.get_logger().info(f"🔍 WaitRotation 使用 rack_id: {rack_id_to_use} 進行查詢")
                    self.rack_query_client.search_rack_id(
                        rack_id=rack_id_to_use, callback=self.rack_callback)
                    self.sent = True
                    self.rack_response_ok = False
                elif self.sent and self.rack_response_ok:
                    # 檢查旋轉是否已完成
                    rotation_completed = False
                    if self.rack_direction is not None:
                        if self.rack_direction == 0 and 1 <= context.get_rack_port <= 16:
                            rotation_completed = True
                            self.node.get_logger().info(
                                f"✅ 旋轉已完成！Rack 方向: {self.rack_direction} (A面), Port: {context.get_rack_port}")
                        elif self.rack_direction == 180 and 17 <= context.get_rack_port <= 32:
                            rotation_completed = True
                            self.node.get_logger().info(
                                f"✅ 旋轉已完成！Rack 方向: {self.rack_direction} (B面), Port: {context.get_rack_port}")
                        else:
                            self.node.get_logger().info(
                                f"🔄 旋轉尚未完成，繼續等待。Rack 方向: {self.rack_direction}, Port: {context.get_rack_port}")
                    else:
                        self.node.get_logger().warn("⚠️ Rack 方向資料無效 (None)，繼續查詢")

                    if rotation_completed:
                        # 旋轉完成，進入下一步
                        self.step = 1
                    else:
                        # 旋轉尚未完成，重新查詢
                        self.sent = False
                        self.rack_response_ok = False
                        self.rack_response = None
                elif self.sent and not self.rack_response_ok:
                    # 查詢已發送但失敗或尚未收到回應，重新查詢
                    self.node.get_logger().info("🔄 查詢失敗或尚未收到回應，重新查詢")
                    self.sent = False
                    self.rack_response_ok = False
                    self.rack_response = None

                self.node.get_logger().info(
                    f"正在調整Rack方向，請等待。Rack Direction: {self.rack_direction}, Rack Port: {context.get_rack_port}")

            case 1:
                # 完成檢查 - 進入下一狀態
                self.node.get_logger().info("已完成所有檢查，準備進入下一狀態")
                # 旋轉完成後進入 TransferCheckEmptyState (entrance 流程)
                from cargo_mover_agv.robot_states.entrance.transfer_check_empty_state import TransferCheckEmptyState
                context.set_state(TransferCheckEmptyState(self.node))
