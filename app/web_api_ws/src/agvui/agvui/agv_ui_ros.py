import os
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from agv_interfaces.msg import AgvStatus
from asyncio import run_coroutine_threadsafe

def ros_message_to_dict(msg):
    return {f: getattr(msg, f) for f in msg.get_fields_and_field_types()}

class AgvUiRos:
    def __init__(self, loop, agv_ui_socket, agv_id=None):
        self.loop = loop
        self.agv_ui_socket = agv_ui_socket

        # 確定命名空間：優先使用傳入的 agv_id，其次環境變數，最後預設值
        if agv_id is None:
            agv_id = os.environ.get('AGV_ID', 'agv')
        self.agv_id = agv_id

        rclpy.init()
        # 使用 agv_id 作為命名空間
        self.node = Node("agv_ui_server_node", namespace=agv_id)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

        print(f"✅ AGVUI ROS2 節點已啟動: /{agv_id}/agv_ui_server_node")

        # 所有agv傳回狀態的topic會相同 都是 /agv/status 其中agv_id 會不同
        self.node.create_subscription(
            AgvStatus,
            '/agv/status',
            self._agv_status_callback,
            10
        )

    def _agv_status_callback(self, msg):
        # ✅ 過濾非本機 AGV：只處理本機 AGV 的 ROS2 消息
        if hasattr(msg, 'agv_id') and msg.agv_id != self.agv_id:
            # 靜默跳過其他 AGV 的消息，避免日誌過多
            return

        self.node.get_logger().info(f"Ros heard: {msg}")
        status_dict = ros_message_to_dict(msg)

        run_coroutine_threadsafe(
            self.agv_ui_socket.notify_agv_status(status_dict),
            self.loop
        )

    def start(self):
        thread = threading.Thread(target=self.executor.spin, daemon=True)
        thread.start()
