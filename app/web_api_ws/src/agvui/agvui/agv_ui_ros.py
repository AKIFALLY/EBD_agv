import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from agv_interfaces.msg import AgvStatus
from asyncio import run_coroutine_threadsafe

def ros_message_to_dict(msg):
    return {f: getattr(msg, f) for f in msg.get_fields_and_field_types()}

class AgvUiRos:
    def __init__(self, loop, agv_ui_socket):
        self.loop = loop
        self.agv_ui_socket = agv_ui_socket

        rclpy.init()
        self.node = Node("agv_ui_server_node")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

        # 所有agv傳回狀態的topic會相同 都是 /agv/status 其中agv_id 會不同
        self.node.create_subscription(
            AgvStatus,
            '/agv/status',
            self._agv_status_callback,
            10
        )

    def _agv_status_callback(self, msg):
        self.node.get_logger().info(f"Ros heard: {msg}")
        status_dict = ros_message_to_dict(msg)
        
        run_coroutine_threadsafe(
            self.agv_ui_socket.notify_agv_status(status_dict),
            self.loop
        )

    def start(self):
        thread = threading.Thread(target=self.executor.spin, daemon=True)
        thread.start()
