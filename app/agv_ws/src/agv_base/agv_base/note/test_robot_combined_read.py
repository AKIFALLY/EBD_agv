#!/usr/bin/env python3
"""
測試 Robot 類別的合併讀取功能
驗證 read_pgno 方法同時讀取 PGNO 和 Error Number 的功能
"""

import rclpy
from rclpy.node import Node
from agv_base.robot import Robot
from agv_base.robot_parameter_abc import RobotParameterABC


class TestRobotParameter(RobotParameterABC):
    """測試用的 Robot Parameter 實作"""

    def values(self):
        return [1, 2, 3, 4, 5]  # 測試參數


class TestRobotCombinedRead(Node):
    """測試 Robot 合併讀取功能的節點"""

    def __init__(self):
        super().__init__('test_robot_combined_read')

        # 建立 Robot 實例
        self.test_parameter = TestRobotParameter()
        self.robot = Robot(self, self.test_parameter)

        # 建立定時器來測試讀取功能
        self.timer = self.create_timer(2.0, self.test_combined_read)
        self.test_count = 0

        self.get_logger().info("🚀 Robot 合併讀取測試節點啟動")

    def test_combined_read(self):
        """測試合併讀取功能"""
        self.test_count += 1
        self.get_logger().info(f"📊 執行第 {self.test_count} 次測試")

        # 測試原有的調用方式
        read_pgno = self.robot.read_pgno_response
        self.robot.read_robot_status()

        # 檢查 PGNO 回應
        if read_pgno is not None:
            self.get_logger().info(f"✅ PGNO 值: {read_pgno.value}")
        else:
            self.get_logger().info("⏳ 等待 PGNO 回應...")

        # 檢查 Error Number 回應
        error_number = self.robot.get_error_number()
        if error_number is not None:
            self.get_logger().info(f"🔍 Error Number: {error_number}")
            if self.robot.has_error():
                self.get_logger().warn(f"⚠️ 檢測到錯誤: {self.robot.get_error_info()}")
            else:
                self.get_logger().info("✅ 無錯誤")
        else:
            self.get_logger().info("⏳ 等待 Error Number 回應...")

        # 測試獨立的 Error Number 讀取
        if self.test_count % 3 == 0:
            self.get_logger().info("🔧 測試獨立 Error Number 讀取")
            self.robot.read_robot_status()

        # 停止測試
        if self.test_count >= 10:
            self.get_logger().info("🏁 測試完成，關閉節點")
            rclpy.shutdown()


def main(args=None):
    """主函數"""
    rclpy.init(args=args)

    try:
        test_node = TestRobotCombinedRead()
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        print("測試被中斷")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
