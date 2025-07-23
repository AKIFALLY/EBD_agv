#!/usr/bin/env python3
"""
測試 rack_status_process 功能
"""

import unittest
from unittest.mock import Mock, MagicMock, patch
import sys
import os

# 添加路徑以便匯入模組
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from wcs_base.database_manager import DatabaseManager


class TestRackStatusProcess(unittest.TestCase):
    """測試 rack_status_process 功能"""

    @patch('wcs_base.database_manager.ConnectionPoolManager')
    def setUp(self, mock_pool_manager):
        """設置測試環境"""
        # 建立 mock logger
        self.mock_logger = Mock()

        # 建立 DatabaseManager 實例
        self.db_manager = DatabaseManager(self.mock_logger, "postgresql://test")

        # Mock 資料庫連線
        self.db_manager.pool_agvc = Mock()

        # 設置測試資料
        self.setup_test_data()

    def setup_test_data(self):
        """設置測試資料"""
        # Mock product 資料
        self.db_manager.product_table = [
            Mock(id=1, size="S", name="Small Product"),
            Mock(id=2, size="L", name="Large Product"),
        ]
        
        # Mock rack 資料
        self.db_manager.rack_table = [
            Mock(id=1, product_id=1, room_id=101, status_id=1),  # 空架，應該保持空架
            Mock(id=2, product_id=1, room_id=102, status_id=1),  # 有32個carrier，應該變成滿料架-32
            Mock(id=3, product_id=2, room_id=103, status_id=1),  # 有16個carrier，應該變成滿料架-16
            Mock(id=4, product_id=1, room_id=104, status_id=1),  # 有10個carrier，應該變成未滿架-32
            Mock(id=5, product_id=2, room_id=105, status_id=1),  # 有8個carrier，應該變成未滿架-16
        ]
        
        # Mock carrier 資料
        self.db_manager.carrier_table = []
        
        # rack_id=1: 0個carrier (空架)
        # rack_id=2: 32個carrier (滿料架-32)
        for i in range(32):
            self.db_manager.carrier_table.append(Mock(rack_id=2, room_id=102))
        
        # rack_id=3: 16個carrier (滿料架-16)
        for i in range(16):
            self.db_manager.carrier_table.append(Mock(rack_id=3, room_id=103))
        
        # rack_id=4: 10個carrier (未滿架-32)
        for i in range(10):
            self.db_manager.carrier_table.append(Mock(rack_id=4, room_id=104))
        
        # rack_id=5: 8個carrier (未滿架-16)
        for i in range(8):
            self.db_manager.carrier_table.append(Mock(rack_id=5, room_id=105))
        
        # 其他必要的資料表
        self.db_manager.task_table = [Mock()]  # 至少要有一個元素
        self.db_manager.task_id_list = [1]  # 至少要有一個元素
        self.db_manager.work_table = [Mock()]  # 至少要有一個元素
        self.db_manager.location_table = [Mock()]  # 至少要有一個元素
        self.db_manager.kuka_node_table = [Mock()]  # 至少要有一個元素

    def test_calculate_rack_status_empty_rack(self):
        """測試空架狀態計算"""
        rack = Mock(id=1, product_id=1, room_id=101)
        product = Mock(size="S")
        
        result = self.db_manager._calculate_rack_status(rack, 0, product)
        self.assertEqual(result, 1, "空架應該返回狀態 1")

    def test_calculate_rack_status_full_rack_32(self):
        """測試滿料架-32狀態計算"""
        rack = Mock(id=2, product_id=1, room_id=102)
        product = Mock(size="S")
        
        result = self.db_manager._calculate_rack_status(rack, 32, product)
        self.assertEqual(result, 2, "32個S尺寸carrier應該返回狀態 2")

    def test_calculate_rack_status_full_rack_16(self):
        """測試滿料架-16狀態計算"""
        rack = Mock(id=3, product_id=2, room_id=103)
        product = Mock(size="L")
        
        result = self.db_manager._calculate_rack_status(rack, 16, product)
        self.assertEqual(result, 3, "16個L尺寸carrier應該返回狀態 3")

    def test_calculate_rack_status_partial_rack_32(self):
        """測試未滿架-32狀態計算"""
        rack = Mock(id=4, product_id=1, room_id=104)
        product = Mock(size="S")
        
        result = self.db_manager._calculate_rack_status(rack, 10, product)
        self.assertEqual(result, 4, "10個S尺寸carrier應該返回狀態 4")

    def test_calculate_rack_status_partial_rack_16(self):
        """測試未滿架-16狀態計算"""
        rack = Mock(id=5, product_id=2, room_id=105)
        product = Mock(size="L")
        
        result = self.db_manager._calculate_rack_status(rack, 8, product)
        self.assertEqual(result, 5, "8個L尺寸carrier應該返回狀態 5")

    def test_calculate_rack_status_no_product(self):
        """測試沒有product資訊的情況"""
        rack = Mock(id=6, product_id=None, room_id=106)
        
        result = self.db_manager._calculate_rack_status(rack, 10, None)
        self.assertEqual(result, 4, "沒有product資訊應該預設返回狀態 4")

    @patch('wcs_base.database_manager.select')
    def test_update_rack_status(self, mock_select):
        """測試更新rack狀態"""
        # Mock session 和 rack
        mock_session = Mock()
        mock_rack = Mock()
        mock_select.return_value = Mock()
        mock_session.exec.return_value.first.return_value = mock_rack
        
        self.db_manager.get_session = Mock(return_value=mock_session)
        self.db_manager.get_session.return_value.__enter__ = Mock(return_value=mock_session)
        self.db_manager.get_session.return_value.__exit__ = Mock(return_value=None)
        
        # 執行更新
        self.db_manager._update_rack_status(1, 2)
        
        # 驗證
        self.assertEqual(mock_rack.status_id, 2)
        mock_session.add.assert_called_once_with(mock_rack)
        mock_session.commit.assert_called_once()

    def test_rack_status_process_integration(self):
        """測試 rack_status_process 整合功能"""
        # Mock _update_rack_status 方法
        self.db_manager._update_rack_status = Mock()

        # 確保 has_all_data 返回 True
        self.assertTrue(self.db_manager.has_all_data(), "測試資料應該完整")

        # 執行 rack_status_process
        self.db_manager.rack_status_process()

        # 驗證更新呼叫
        # rack_id=1 狀態不變（已經是空架狀態1），不會呼叫更新
        # rack_id=2 應該從狀態1更新為狀態2 (滿料架-32)
        # rack_id=3 應該從狀態1更新為狀態3 (滿料架-16)
        # rack_id=4 應該從狀態1更新為狀態4 (未滿架-32)
        # rack_id=5 應該從狀態1更新為狀態5 (未滿架-16)

        expected_calls = [
            unittest.mock.call(2, 2),  # rack_id=2, status=2
            unittest.mock.call(3, 3),  # rack_id=3, status=3
            unittest.mock.call(4, 4),  # rack_id=4, status=4
            unittest.mock.call(5, 5),  # rack_id=5, status=5
        ]

        # 檢查實際的呼叫
        actual_calls = self.db_manager._update_rack_status.call_args_list
        self.assertEqual(len(actual_calls), 4, f"應該有4次更新呼叫，實際有{len(actual_calls)}次")

        # 驗證每個呼叫
        for expected_call in expected_calls:
            self.assertIn(expected_call, actual_calls, f"缺少預期的呼叫: {expected_call}")


if __name__ == '__main__':
    unittest.main()
