import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from agv_cmd_interfaces.srv import ManualCommand, GeneralCommand
from plc_proxy.plc_client import PlcClient


class AgvCommandService(Node):
    def __init__(self):
        super().__init__('agv_cmd_service_node')
        self.get_logger().info(f"[⚙️ 節點名稱] {self.get_name()} @ {self.get_namespace()}")

        self.declare_parameters('', [
            ('forward_address', '3708'),
            ('backward_address', '3709'),
            ('rotate_left_address', '3712'),
            ('rotate_right_address', '3713'),
            ('shift_left_address', '3801'),
            ('shift_right_address', '3802'),
            ('break_address', '3714'),
            ('enable_address', '3715'),
            ('auto_address1', '4001'),
            ('auto_address2', '0000'),
            ('stop_address', '3701'),
            ('reset_address', '302'),
            ('send_mission_from_address', '2990'),
            ('send_mission_to_address', '2991'),
            ('send_mission_magic_address', '2993'),
            ('cancel_mission_address', '7001'),
            ('traffic_stop_address', '7002'),
        ])

        for param in self._parameters:
            value = self.get_parameter(param).value
            setattr(self, param, value)
            self.get_logger().info(f"[參數] {param} = {value}")

        self.plc_comm_client = PlcClient(Node('node'), self.get_namespace())

        self.create_service(ManualCommand, 'ManualCommand', self.manual_command_callback)
        self.create_service(GeneralCommand, 'GeneralCommand', self.general_command_callback)

    def manual_command_callback(self, request, response):
        self.get_logger().info(f"📩 手動命令：{request.command} ON/OFF={request.onoff}")
        response.success = False

        command_map = {
            "forward": self.forward_address,
            "backward": self.backward_address,
            "rotate_left": self.rotate_left_address,
            "rotate_right": self.rotate_right_address,
            "shift_left": self.shift_left_address,
            "shift_right": self.shift_right_address,
            "break": self.break_address,
            "enable": self.enable_address
        }

        address = command_map.get(request.command)
        if not address:
            self.get_logger().warn(f"⚠️ 未知指令: {request.command}")
            return response

        try:
            if request.command in ["break", "enable"]:
                self.plc_comm_client.force_on("MR", address)
            elif request.onoff:
                self.plc_comm_client.force_on("MR", address)
            else:
                self.plc_comm_client.force_off("MR", address)
            response.success = True
        except Exception as e:
            self.get_logger().error(f"❌ 指令執行失敗: {e}")

        self.get_logger().info(f"✅ Response 回傳成功: {response.success}")
        return response

    def general_command_callback(self, request, response):
        self.get_logger().info(f"📩 一般命令：{request.command} 參數: {request.parameter}")
        response.success = False

        try:
            para = request.parameter.split(',')
            onoff = para[0] if len(para) > 0 else ''
            nfrom = para[1] if len(para) > 1 else ''
            nto = para[2] if len(para) > 2 else ''
            nmagic = para[3] if len(para) > 3 else ''

            if request.command == "auto":
                self.plc_comm_client.force_on("MR", self.auto_address1) if onoff == "on" else self.plc_comm_client.force_off("MR", self.auto_address1)
                self.plc_comm_client.force_on("MR", self.auto_address2) if onoff == "on" else self.plc_comm_client.force_off("MR", self.auto_address2)

            elif request.command == "stop":
                self.plc_comm_client.force_on("MR", self.stop_address)

            elif request.command == "reset":
                self.plc_comm_client.force_on("MR", self.reset_address)

            elif request.command == "send_mission":
                self.plc_comm_client.write_data("DM", self.send_mission_from_address, nfrom)
                self.plc_comm_client.write_data("DM", self.send_mission_to_address, nto)
                self.plc_comm_client.write_data("DM", self.send_mission_magic_address, nmagic)

            elif request.command == "cancel_mission":
                self.plc_comm_client.force_on("MR", self.cancel_mission_address)

            elif request.command == "traffic_stop":
                if onoff == "on":
                    self.plc_comm_client.force_on("MR", self.traffic_stop_address)
                else:
                    self.plc_comm_client.force_off("MR", self.traffic_stop_address)

            else:
                self.get_logger().warn(f"⚠️ 未知指令: {request.command}")
                return response

            response.success = True
        except Exception as e:
            self.get_logger().error(f"❌ 一般命令執行錯誤: {e}")

        return response

def main():
    rclpy.init()
    node = AgvCommandService()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("🛑 關閉節點")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()