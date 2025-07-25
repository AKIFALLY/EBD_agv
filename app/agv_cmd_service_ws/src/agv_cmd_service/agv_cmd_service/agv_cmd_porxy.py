from rclpy.node import Node
from plc_proxy.plc_client import PlcClient
import yaml
import os


class AGVCommandProxy:
    def __init__(self, node: Node):
        self.node = node
        self.namespace = node.get_namespace()

        config_path = "/app/agv_cmd_service_ws/src/agv_cmd_service/config/agv_cmd_service.yaml"
        self.address_map = {}

        if os.path.exists(config_path):
            try:
                with open(config_path, 'r') as f:
                    yaml_data = yaml.safe_load(f)
                    self.address_map = yaml_data
            except Exception as e:
                node.get_logger().error(f"❌ YAML 解析錯誤: {e}，使用預設值")
                self.address_map = self.default_map()
        else:
            node.get_logger().warn("⚠️ 找不到 YAML 設定檔，使用預設值")
            self.address_map = self.default_map()

        self.plc_client = PlcClient(node)

    def default_map(self):
        return dict([
            ('forward', '3708'),
            ('backward', '3709'),
            ('rotate_left', '3712'),
            ('rotate_right', '3713'),
            ('shift_left', '3801'),
            ('shift_right', '3802'),
            ('break', '3714'),
            ('enable', '3715'),
            ('auto1', '4001'),
            ('auto2', '0000'),
            ('stop', '3701'),
            ('reset', '302'),
            ('send_mission_from', '2990'),
            ('send_mission_to', '2991'),
            ('send_mission_magic', '2993'),
            ('cancel_mission', '7001'),
            ('traffic_stop', '7002'),
        ])

    def force_callback(self, success: bool):
        """Force callback"""
        # 這裡可擴充處理 callback 邏輯
        pass

    def send_movement_command(self, direction: str, onoff: bool) -> bool:
        key = f"{direction}"
        if key not in self.address_map:
            self.node.get_logger().error(f"❌ 找不到指令對應地址: {key}")
            return False
        addr = str(self.address_map[key])

        def plc_cb(success: bool):
            if success:
                self.node.get_logger().info(
                    f"✅ 指令傳送成功: {direction} {'ON' if onoff else 'OFF'}")
            else:
                self.node.get_logger().error(
                    f"❌ 指令傳送失敗: {direction} {'ON' if onoff else 'OFF'}")

        try:
            if onoff:
                self.plc_client.async_force_on("MR", addr, plc_cb)
            else:
                self.plc_client.async_force_off("MR", addr, plc_cb)
            return True
        except Exception as e:
            self.node.get_logger().error(f"❌ 指令傳送失敗: {e}")
            return False

    def stop(self) -> bool:
        addr = str(self.address_map.get('stop', '3701'))

        def plc_cb(success: bool):
            if success:
                self.node.get_logger().info("✅ Stop 指令成功")
            else:
                self.node.get_logger().error("❌ Stop 指令失敗")

        self.plc_client.async_force_on("MR", addr, plc_cb)
        return True

    def reset(self) -> bool:
        addr = str(self.address_map.get('reset', '302'))

        def plc_cb(success: bool):
            if success:
                self.node.get_logger().info("✅ Reset 指令成功")
            else:
                self.node.get_logger().error("❌ Reset 指令失敗")

        self.plc_client.async_force_on("MR", addr, plc_cb)
        return True

    def enable(self) -> bool:
        addr = str(self.address_map.get('enable', '3715'))

        def plc_cb(success: bool):
            if success:
                self.node.get_logger().info("✅ Enable 指令成功")
            else:
                self.node.get_logger().error("❌ Enable 指令失敗")

        self.plc_client.async_force_on("MR", addr, plc_cb)
        return True

    def send_mission(self, mfrom: int, mto: int, magic: int) -> bool:
        # 先定義一個內部 callback 用來接收三個寫入的結果
        results = {'from': None, 'to': None, 'magic': None}

        def check_done():
            # 全部寫入完成且都成功才算成功
            return all(value is not None for value in results.values())

        def report_result():
            if all(results.values()):
                self.node.get_logger().info("✅ 任務指令傳送成功")
            else:
                self.node.get_logger().error("❌ 任務指令失敗")

        def write_callback(key):
            def callback(success: bool):
                results[key] = success
                if check_done():
                    report_result()
            return callback

        try:
            self.plc_client.async_write_data(
                "DM", int(self.address_map['send_mission_from']), str(mfrom), write_callback('from'))
            self.plc_client.async_write_data(
                "DM", int(self.address_map['send_mission_to']), str(mto), write_callback('to'))
            self.plc_client.async_write_data(
                "DM", int(self.address_map['send_mission_magic']), str(magic), write_callback('magic'))
            return True
        except Exception as e:
            self.node.get_logger().error(f"❌ 任務指令失敗: {e}")
            return False

    def cancel_mission(self) -> bool:
        addr = str(self.address_map.get('cancel_mission', '7001'))

        def plc_cb(success: bool):
            if success:
                self.node.get_logger().info("✅ Cancel Mission 指令成功")
            else:
                self.node.get_logger().error("❌ Cancel Mission 指令失敗")

        self.plc_client.async_force_on("MR", addr, plc_cb)
        return True

    def traffic_stop(self, enable: bool) -> bool:
        addr = str(self.address_map.get('traffic_stop', '7002'))

        def plc_cb(success: bool):
            if success:
                self.node.get_logger().info(
                    f"✅ 交通停止指令成功: {'ON' if enable else 'OFF'}")
            else:
                self.node.get_logger().error(
                    f"❌ 交通停止指令失敗: {'ON' if enable else 'OFF'}")

        try:
            if enable:
                self.plc_client.async_force_on("MR", addr, plc_cb)
            else:
                self.plc_client.async_force_off("MR", addr, plc_cb)
            return True
        except Exception as e:
            self.node.get_logger().error(f"❌ 交通停止指令失敗: {e}")
            return False

    def destroy(self):
        self.plc_client.destroy()
        self.node.get_logger().info("🗑️ AGV 指令客戶端已銷毀")

