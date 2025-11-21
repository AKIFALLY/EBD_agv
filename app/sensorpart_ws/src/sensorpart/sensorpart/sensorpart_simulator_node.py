#!/usr/bin/env python3
"""
SensorPart Simulator Node

模擬 SensorPart 相機的 TCP 伺服器，用於測試和開發。
發送符合格式的 3D 定位和 OCR 測試資料。

使用方式：
    # 啟動模擬伺服器（預設 0.0.0.0:2005）
    ros2 run sensorpart sensorpart_simulator_node

    # 自訂 IP 和 Port
    ros2 run sensorpart sensorpart_simulator_node --ros-args \
        -p host:=0.0.0.0 -p port:=2005

資料格式：
    3D 定位: (005,P,x,y,z,rx,ry,rz)
    OCR 結果: (OCR,text)
"""

import rclpy
from rclpy.node import Node
import socket
import threading
import time
import random


class SensorPartSimulatorNode(Node):
    """SensorPart 相機模擬器節點"""

    def __init__(self):
        super().__init__('sensorpart_simulator')

        # 宣告參數
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 2005)
        self.declare_parameter('auto_send', True)
        self.declare_parameter('send_interval', 5.0)  # 秒

        # 取得參數值
        self.host = self.get_parameter('host').value
        self.port = self.get_parameter('port').value
        self.auto_send = self.get_parameter('auto_send').value
        self.send_interval = self.get_parameter('send_interval').value

        # TCP 伺服器
        self.server_socket = None
        self.client_socket = None
        self.client_address = None
        self.is_running = False
        self.server_thread = None

        # 測試資料集
        self.test_products = [
            "ABC12345",
            "DEF67890",
            "ABC54321",
            "DEF09876",
            "TEST0001",
            "PROD1234",
        ]

        self.current_product_index = 0

        # 啟動模擬伺服器
        self.start_server()

        # 自動發送定時器
        if self.auto_send:
            self.timer = self.create_timer(
                self.send_interval,
                self.auto_send_callback
            )
            self.get_logger().info(
                f'✅ 自動發送已啟用，間隔: {self.send_interval} 秒'
            )

        self.get_logger().info('✅ SensorPart 模擬器已啟動')

    def start_server(self):
        """啟動 TCP 伺服器"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(1)
            self.is_running = True

            self.get_logger().info(
                f'🌐 TCP 伺服器啟動: {self.host}:{self.port}'
            )

            # 在獨立執行緒中等待連接
            self.server_thread = threading.Thread(
                target=self.accept_connections,
                daemon=True
            )
            self.server_thread.start()

        except Exception as e:
            self.get_logger().error(f'❌ 伺服器啟動失敗: {e}')

    def accept_connections(self):
        """接受客戶端連接"""
        while self.is_running:
            try:
                self.get_logger().info('⏳ 等待客戶端連接...')
                self.client_socket, self.client_address = self.server_socket.accept()
                self.get_logger().info(
                    f'✅ 客戶端已連接: {self.client_address}'
                )

                # 保持連接，等待斷開
                while self.is_running:
                    time.sleep(1)
                    # 檢查連接是否仍然存在
                    if self.client_socket:
                        try:
                            # 嘗試發送一個空資料來檢查連接
                            self.client_socket.send(b'')
                        except (BrokenPipeError, ConnectionResetError):
                            self.get_logger().warn('⚠️ 客戶端已斷開')
                            self.client_socket = None
                            break

            except Exception as e:
                if self.is_running:
                    self.get_logger().error(f'❌ 連接錯誤: {e}')
                    time.sleep(1)

    def send_3d_position(self, x=None, y=None, z=None, rx=None, ry=None, rz=None):
        """
        發送 3D 定位資料

        Args:
            x, y, z: 位置座標（整數），None 則隨機生成
            rx, ry, rz: 旋轉角度（浮點數），None 則隨機生成
        """
        if not self.client_socket:
            self.get_logger().warn('⚠️ 沒有客戶端連接')
            return False

        # 生成隨機資料（如果未提供）
        x = x if x is not None else random.randint(1000, 2000)
        y = y if y is not None else random.randint(500, 1500)
        z = z if z is not None else random.randint(100, 500)
        rx = rx if rx is not None else round(random.uniform(-5.0, 5.0), 2)
        ry = ry if ry is not None else round(random.uniform(-5.0, 5.0), 2)
        rz = rz if rz is not None else round(random.uniform(-5.0, 5.0), 2)

        # 格式: (005,P,x,y,z,rx,ry,rz)
        message = f"(005,P,{x},{y},{z},{rx},{ry},{rz})\r\n"

        try:
            self.client_socket.send(message.encode('utf-8'))
            self.get_logger().info(
                f'📍 發送 3D 位置: x={x}, y={y}, z={z}, '
                f'rx={rx}, ry={ry}, rz={rz}'
            )
            return True
        except Exception as e:
            self.get_logger().error(f'❌ 發送失敗: {e}')
            self.client_socket = None
            return False

    def send_ocr_result(self, text=None):
        """
        發送 OCR 識別結果

        Args:
            text: OCR 文字，None 則從測試資料集中選取
        """
        if not self.client_socket:
            self.get_logger().warn('⚠️ 沒有客戶端連接')
            return False

        # 如果未提供文字，從測試資料集中選取
        if text is None:
            text = self.test_products[self.current_product_index]
            self.current_product_index = (
                (self.current_product_index + 1) % len(self.test_products)
            )

        # 格式: (OCR,text)
        message = f"(OCR,{text})\r\n"

        try:
            self.client_socket.send(message.encode('utf-8'))
            self.get_logger().info(f'📄 發送 OCR: {text}')
            return True
        except Exception as e:
            self.get_logger().error(f'❌ 發送失敗: {e}')
            self.client_socket = None
            return False

    def send_3d_position_fail(self):
        """發送 3D 定位失敗的資料"""
        if not self.client_socket:
            self.get_logger().warn('⚠️ 沒有客戶端連接')
            return False

        # 格式: (005,F,0,0,0,0.0,0.0,0.0)
        message = "(005,F,0,0,0,0.0,0.0,0.0)\r\n"

        try:
            self.client_socket.send(message.encode('utf-8'))
            self.get_logger().info('❌ 發送 3D 位置失敗訊號')
            return True
        except Exception as e:
            self.get_logger().error(f'❌ 發送失敗: {e}')
            self.client_socket = None
            return False

    def auto_send_callback(self):
        """定時自動發送測試資料"""
        if not self.client_socket:
            return

        # 每次發送：3D 位置 + OCR
        self.send_3d_position()
        time.sleep(0.1)  # 小延遲
        self.send_ocr_result()

    def destroy_node(self):
        """節點關閉時清理資源"""
        self.get_logger().info('🛑 停止模擬器...')
        self.is_running = False

        if self.client_socket:
            self.client_socket.close()

        if self.server_socket:
            self.server_socket.close()

        super().destroy_node()


def main(args=None):
    """節點主函數"""
    rclpy.init(args=args)
    node = SensorPartSimulatorNode()

    try:
        # 主循環
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

            # 互動式命令（可選）
            # 注意：在 ROS2 spin 中使用 input() 會阻塞，這裡僅作示範
            # 實際使用時建議透過 ROS2 service 或 topic 來控制

    except KeyboardInterrupt:
        node.get_logger().info('⌨️  收到鍵盤中斷')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
