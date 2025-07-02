import rclpy
from rclpy.node import Node
from agv_cmd_interfaces.srv import ManualCommand, GeneralCommand

class AgvCommandClient:
    def __init__(self, node: Node, namespace: str = ""):
        self.node = node
        self.namespace = '/' + namespace.lstrip('/')

        self.manual_command_client = self.node.create_client(
            ManualCommand,
            f"{self.namespace}/ManualCommand" if self.namespace else "ManualCommand"
        )
        self.general_command_client = self.node.create_client(
            GeneralCommand,
            f"{self.namespace}/GeneralCommand" if self.namespace else "GeneralCommand"
        )

        self.node.get_logger().info(f"📡 建立 AGV 指令客戶端（命名空間: {self.namespace}）")

        # 避免 constructor 卡住，最多重試 10 次
        for _ in range(10):
            if self.manual_command_client.wait_for_service(timeout_sec=1.0):
                break
            self.node.get_logger().warn("等待 ManualCommand 服務中...")
        else:
            self.node.get_logger().error("❌ ManualCommand 服務初始化逾時")

        for _ in range(10):
            if self.general_command_client.wait_for_service(timeout_sec=1.0):
                break
            self.node.get_logger().warn("等待 GeneralCommand 服務中...")
        else:
            self.node.get_logger().error("❌ GeneralCommand 服務初始化逾時")

    def send_manual_command(self, command: str, onoff: bool) -> bool:
        """發送手動命令"""
        self.node.get_logger().info(f"📤 發送手動命令: {command}, 開關: {onoff}")

        if not self.manual_command_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error("❌ ManualCommand 服務尚未就緒")
            return False

        try:
            request = ManualCommand.Request()
            request.command = command
            request.onoff = onoff
            future = self.manual_command_client.call_async(request)
            self.node.get_logger().info(f"[DEBUG] 進入 {request.command} 指令邏輯1")

            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            self.node.get_logger().info(f"[DEBUG] 進入 {request.command} 指令邏輯2")
            if future.done():
                result = future.result()
                if result is not None:
                    self.node.get_logger().info(f"✅ 執行成功: {result.success}")
                    return result.success
                else:
                    self.node.get_logger().error("❌ 未收到服務端回應")
            else:
                self.node.get_logger().warn(f"⌛ 執行逾時（未完成）{request.command}")

        except Exception as e:
            self.node.get_logger().error(f"❌ 發送手動命令錯誤: {e}")

        return False

    def send_general_command(self, command: str, parameter: str) -> bool:
        """發送一般命令"""
        self.node.get_logger().info(f"📤 發送一般命令: {command}, 參數: {parameter}")

        if not self.general_command_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error("❌ GeneralCommand 服務尚未就緒")
            return False

        try:
            request = GeneralCommand.Request()
            request.command = command
            request.parameter = parameter
            future = self.general_command_client.call_async(request)

            rclpy.spin_until_future_complete(self.node, future, timeout_sec=0.5)

            if future.done():
                result = future.result()
                if result is not None:
                    self.node.get_logger().info(f"✅ 一般命令執行結果: {result.success}")
                    return result.success
                else:
                    self.node.get_logger().error("❌ 未收到服務端回應")
            else:
                self.node.get_logger().warn("⌛ 一般命令執行逾時")

        except Exception as e:
            self.node.get_logger().error(f"❌ 發送一般命令錯誤: {e}")

        return False
