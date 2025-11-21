#!/usr/bin/env python3
"""
SensorPart Publisher Node

發布 SensorPart 相機的 OCR 和 3D 位置資料到 ROS2 topics。
支援 namespace 和參數配置，可用於多 AGV 場景。

Topics 發布：
- {namespace}/sensor/ocr (std_msgs/String): OCR 識別結果
- {namespace}/sensor/position_3d (geometry_msgs/PoseStamped): 3D 位置資料

參數：
- host (string): 相機 IP 地址，預設 192.168.2.100
- port (int): 相機 Port，預設 2005
- publish_rate (float): 發布頻率 (Hz)，預設 10.0
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensorpart.sensorpart import SensorPart


class SensorPartPublisherNode(Node):
    """SensorPart 資料發布節點"""

    def __init__(self):
        super().__init__('sensorpart_publisher')

        # 宣告參數
        self.declare_parameter('host', '192.168.2.111')
        self.declare_parameter('port', 2005)
        self.declare_parameter('debounce_seconds', 1.0)  # 防抖時間窗口（秒）

        # 取得參數值
        host = self.get_parameter('host').value
        port = self.get_parameter('port').value
        self.debounce_seconds = self.get_parameter('debounce_seconds').value

        self.get_logger().info(
            f'📡 SensorPart 設定: {host}:{port}, 防抖窗口: {self.debounce_seconds} 秒'
        )

        # 建立 Publishers
        self.ocr_pub = self.create_publisher(String, 'sensor/ocr', 10)
        self.position_pub = self.create_publisher(
            PoseStamped, 'sensor/position_3d', 10
        )

        # 時間窗口防抖：記錄上次發布的內容和時間
        self.last_ocr_value = None
        self.last_ocr_time = None
        self.last_position_data = None
        self.last_position_time = None

        # 初始化 SensorPart TCP 客戶端（傳入回調函數，事件驅動）
        self.tcp_client = SensorPart(
            host=host,
            port=port,
            on_ocr_callback=self._ocr_callback,
            on_position_callback=self._position_callback
        )
        self.tcp_client.start()

        self.get_logger().info('✅ SensorPart Publisher 節點已啟動（事件驅動模式）')
        self.get_logger().info(f'   - OCR Topic: {self.get_namespace()}/sensor/ocr')
        self.get_logger().info(
            f'   - Position Topic: {self.get_namespace()}/sensor/position_3d'
        )

    def _ocr_callback(self, ocr_result):
        """
        OCR 回調函數（事件驅動）
        TCP 收到新 OCR 時觸發，實作時間窗口防抖機制
        """
        current_time = self.get_clock().now()
        should_publish = False

        # 判斷是否應該發布
        if self.last_ocr_value is None:
            # 第一次收到，必須發布
            should_publish = True
        elif ocr_result != self.last_ocr_value:
            # 內容不同，必須發布
            should_publish = True
        else:
            # 內容相同，檢查時間間隔
            time_diff = (current_time - self.last_ocr_time).nanoseconds / 1e9
            if time_diff > self.debounce_seconds:
                # 超過防抖窗口，視為新的拍攝事件
                should_publish = True

        if should_publish:
            # 發布 OCR topic
            msg = String()
            msg.data = ocr_result
            self.ocr_pub.publish(msg)
            self.get_logger().info(f'📄 發布 OCR: {ocr_result}')

            # 更新記錄
            self.last_ocr_value = ocr_result
            self.last_ocr_time = current_time
        else:
            self.get_logger().debug(
                f'⏸️  OCR 防抖過濾: {ocr_result}（1秒內重複）'
            )

    def _position_callback(self, position_data):
        """
        3D 位置回調函數（事件驅動）
        TCP 收到新 3D 位置時觸發，實作時間窗口防抖機制
        """
        current_time = self.get_clock().now()
        should_publish = False

        # 判斷是否應該發布
        if self.last_position_data is None:
            # 第一次收到，必須發布
            should_publish = True
        elif position_data != self.last_position_data:
            # 內容不同，必須發布
            should_publish = True
        else:
            # 內容相同，檢查時間間隔
            time_diff = (current_time - self.last_position_time).nanoseconds / 1e9
            if time_diff > self.debounce_seconds:
                # 超過防抖窗口，視為新的測量事件
                should_publish = True

        if should_publish:
            # 發布 3D 位置 topic
            pose = PoseStamped()
            pose.header.stamp = current_time.to_msg()
            pose.header.frame_id = 'sensor_frame'

            # 位置 (x, y, z)
            pose.pose.position.x = float(position_data['x'])
            pose.pose.position.y = float(position_data['y'])
            pose.pose.position.z = float(position_data['z'])

            # 旋轉（使用歐拉角轉換為四元數）
            from math import radians, sin, cos
            rx = radians(position_data['rx'])
            ry = radians(position_data['ry'])
            rz = radians(position_data['rz'])

            # 簡化處理：只記錄 rz（yaw）到四元數
            # 完整實作應使用 tf_transformations.quaternion_from_euler(rx, ry, rz)
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = sin(rz / 2.0)
            pose.pose.orientation.w = cos(rz / 2.0)

            self.position_pub.publish(pose)
            self.get_logger().info(
                f'📍 發布 3D 位置: x={position_data["x"]}, '
                f'y={position_data["y"]}, z={position_data["z"]}'
            )

            # 更新記錄
            self.last_position_data = position_data.copy()
            self.last_position_time = current_time
        else:
            self.get_logger().debug(
                '⏸️  3D 位置防抖過濾（1秒內重複）'
            )

    def destroy_node(self):
        """節點關閉時停止 TCP 客戶端"""
        self.get_logger().info('🛑 停止 SensorPart TCP 客戶端...')
        self.tcp_client.stop()
        super().destroy_node()


def main(args=None):
    """節點主函數"""
    rclpy.init(args=args)
    node = SensorPartPublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('⌨️  收到鍵盤中斷')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
