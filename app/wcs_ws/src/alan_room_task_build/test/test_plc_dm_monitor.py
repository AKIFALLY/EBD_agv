"""
alan_room_task_build 套件單元測試
"""

import unittest
import rclpy
from alan_room_task_build.plc_dm_monitor_node import PlcDmMonitorNode


class TestPlcDmMonitor(unittest.TestCase):
    """PLC DM Monitor 測試類別"""

    @classmethod
    def setUpClass(cls):
        """測試開始前初始化 ROS 2"""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """測試結束後清理 ROS 2"""
        rclpy.shutdown()

    def setUp(self):
        """每個測試前建立節點"""
        self.node = PlcDmMonitorNode()

    def tearDown(self):
        """每個測試後銷毀節點"""
        self.node.destroy_node()

    def test_combine_32bit(self):
        """測試 32-bit 組合邏輯"""
        # 測試案例 1: low=100, high=200
        # 預期結果: 100 + (200 << 16) = 100 + 13107200 = 13107300
        result = self.node._combine_32bit(100, 200)
        self.assertEqual(result, 13107300)

        # 測試案例 2: low=101, high=205
        # 預期結果: 101 + (205 << 16) = 101 + 13435904 = 13436005
        result = self.node._combine_32bit(101, 205)
        self.assertEqual(result, 13436005)

        # 測試案例 3: low=0, high=0
        result = self.node._combine_32bit(0, 0)
        self.assertEqual(result, 0)

        # 測試案例 4: low=65535, high=65535 (最大值)
        # 預期結果: 65535 + (65535 << 16) = 65535 + 4294901760 = 4294967295
        result = self.node._combine_32bit(65535, 65535)
        self.assertEqual(result, 4294967295)

    def test_extract_room_id(self):
        """測試 room_id 提取（取第一位數字）"""
        # 測試案例 1: 2010101 → room_id = 2
        result = self.node._extract_room_id(2010101)
        self.assertEqual(result, 2)

        # 測試案例 2: 1234567 → room_id = 1
        result = self.node._extract_room_id(1234567)
        self.assertEqual(result, 1)

        # 測試案例 3: 9999999 → room_id = 9
        result = self.node._extract_room_id(9999999)
        self.assertEqual(result, 9)

        # 測試案例 4: 3050101 → room_id = 3
        result = self.node._extract_room_id(3050101)
        self.assertEqual(result, 3)

    def test_processed_work_ids_set(self):
        """測試已處理 work_id 集合"""
        # 初始應該是空的
        self.assertEqual(len(self.node.processed_work_ids), 0)

        # 加入 work_id
        self.node.processed_work_ids.add(2010101)
        self.assertEqual(len(self.node.processed_work_ids), 1)
        self.assertIn(2010101, self.node.processed_work_ids)

        # 再加入相同的 work_id（set 不會重複）
        self.node.processed_work_ids.add(2010101)
        self.assertEqual(len(self.node.processed_work_ids), 1)

        # 加入不同的 work_id
        self.node.processed_work_ids.add(2020202)
        self.assertEqual(len(self.node.processed_work_ids), 2)


if __name__ == '__main__':
    unittest.main()
