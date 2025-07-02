from agv_base.states.state import State
from rclpy.node import Node
from cargo_mover_agv.robot_context import RobotContext
from db_proxy.carrier_query_client import CarrierQueryClient
from db_proxy.rack_query_client import RackQueryClient
from db_proxy.agvc_database_client import AGVCDatabaseClient
from db_proxy_interfaces.msg import Task as TaskMsg


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
        self.rack_id = 123  # 假設的Rack ID，實際應根據需求設定
        self.update_task_success = False
        self.node = node

    def enter(self):
        self.node.get_logger().info("Robot Exit 目前狀態: CheckRackSide")
        self.carrier_response_ok = False
        self.rack_response_ok = False
        self.sent = False
        self.update_task_success = False

    def leave(self):
        self.node.get_logger().info("Robot Exit 離開 CheckRackSide 狀態")
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
        # 更安全的寫法，避免 IndexError
        rack = response.datas[0] if response and response.datas else None

        self.rack_response = response
        self.node.get_logger().info(f"{self.rack_response}")
        if self.rack_response.success:
            self.rack_response_ok = True
            self.rack_direction = rack.direction

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
        self.node.get_logger().info("Robot Exit CheckRackSide 狀態")

        match self.step:
            case 0:
                if not self.carrier_response_ok and not self.sent:
                    self.carrier_query_client.search_carrier_rack_id(
                        rack_id=self.rack_id, callback=self.carrier_callback)
                    self.sent = True
                elif self.sent and self.carrier_response_ok:
                    self.sent = False
                    self.step = 1
            case 1:
                if not self.rack_response_ok and not self.sent:
                    self.rack_query_client.search_rack_id(
                        rack_id=self.rack_id, callback=self.rack_callback)
                    self.sent = True

                elif self.sent and self.rack_response_ok:
                    self.sent = False
                    self.step = 2

            case 2:
                if self.no_carrier and self.carrier_response_ok:
                    self.node.get_logger().info(
                        "沒有找到任何Carrier，完成。")
                    context.get_rack_port = 32
                    self.step = 3
                elif self.min_rack_index == 1 and self.carrier_response_ok:
                    self.node.get_logger().info(
                        "已到達最小 Rack Index，完成。")
                    from cargo_mover_agv.robot_states.complete_state import CompleteState
                    context.set_state(CompleteState(self.node))
                    self.step = 0
                elif 1 < self.min_rack_index <= 32 and self.carrier_response_ok:
                    context.get_rack_port = self.min_rack_index - 1
                    self.step = 3
            case 3:
                if self.rack_direction == 0 and 1 <= context.get_rack_port <= 16:
                    self.node.get_logger().info(
                        f"Rack方向是A面，不需要調整。Rack Port:{context.get_rack_port}")
                    self.step = 5
                elif self.rack_direction == 180 and 17 <= context.get_rack_port <= 32:
                    self.node.get_logger().info(
                        f"Rack方向是B面，不需要調整。Rack Port:{context.get_rack_port}")
                    self.step = 5
                elif self.rack_direction == 0 and 17 <= context.get_rack_port <= 32:
                    self.node.get_logger().info(
                        f"Rack方向是A面，請旋轉Rack至B面。Rack Port:{context.get_rack_port}")
                    self.step = 4
                elif self.rack_direction == 180 and 1 <= context.get_rack_port <= 16:
                    self.node.get_logger().info(
                        f"Rack方向是B面，請旋轉Rack至A面。Rack Port:{context.get_rack_port}")
                    self.step = 4
                else:
                    self.node.get_logger().error(
                        f"無法辨識Rack方向或Port範圍錯誤。Rack Direction: {self.rack_direction}, Rack Port: {context.get_rack_port}")
            case 4:
                self.node.get_logger().info(
                    f"self.rack_response_ok: {self.rack_response_ok}, self.sent: {self.sent}")
                context.rack_photo_up_or_down_buffer = None
                if not self.sent:
                    self.rack_query_client.search_rack_id(
                        rack_id=self.rack_id, callback=self.rack_callback)
                    self.sent = True
                    self.rack_response_ok = False
                elif self.sent and self.rack_response_ok:
                    self.sent = False
                    self.rack_response_ok = False
                    self.rack_response = None

                # 新增 update_task 呼叫
                if not self.update_task_success:
                    task = TaskMsg()
                    task.id = self.node.task.id
                    task.work_id = self.node.task.work_id
                    task.status_id = 10001
                    task.room_id = self.node.task.room_id
                    task.node_id = self.node.task.room_id
                    task.name = self.node.task.name
                    task.description = self.node.task.description
                    task.agv_id = self.node.AGV_id
                    task.agv_name = self.node.task.agv_name
                    task.priority = self.node.task.priority
                    task.parameters = self.node.task.parameters

                    self.agvc_client.async_update_task(task, self.update_task_callback)
                    self.node.get_logger().info("✅ 已發送 update_task 請求")

                self.node.get_logger().info(
                    f"正在調整Rack方向，請等待。Rack Direction: {self.rack_direction}, Rack Port: {context.get_rack_port}")
                if self.rack_direction == 0 and 1 <= context.get_rack_port <= 16:
                    self.step = 5
                elif self.rack_direction == 180 and 17 <= context.get_rack_port <= 32:
                    self.step = 5
            case 5:
                self.node.get_logger().info("已完成所有檢查，準備進入下一狀態")
                # 根據 exit 流程，這裡應該轉換到適當的下一個狀態
                # 假設需要轉換到其他 exit 相關狀態
                from cargo_mover_agv.robot_states.exit.transfer_check_have_state import TransferCheckHaveState
                context.set_state(TransferCheckHaveState(self.node))
