#!/usr/bin/env python3
"""
測試 OR 邏輯的條件檢查功能
"""

import unittest
from unittest.mock import Mock, MagicMock, patch
import sys
import os

# 添加路徑以便匯入模組
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from wcs_base.task_condition_checker import TaskConditionChecker


class TestORLogicConditionChecker(unittest.TestCase):
    """測試 OR 邏輯條件檢查功能"""

    def setUp(self):
        """設置測試環境"""
        # 建立 mock logger 和 db_manager
        self.mock_logger = Mock()
        self.mock_db_manager = Mock()

        # 建立 TaskConditionChecker 實例
        self.checker = TaskConditionChecker(
            db_manager=self.mock_db_manager,
            logger=self.mock_logger,
            real_time_mode=False  # 使用非即時模式避免資料庫連線
        )

        # Mock 必要的屬性
        self.checker.pool_agvc = Mock()
        self.checker.config = Mock()
        self.checker.config.cache_timeout = 300

    def test_or_logic_first_condition_true(self):
        """測試 OR 邏輯：第一個條件為 True"""
        # Mock 資料庫查詢結果
        mock_results = [
            # ID 100: 第一個條件滿足
            [{"result": "True", "next_id": "end", "location": "A1"}],
        ]
        
        self.checker.get_task_condition_results = Mock(side_effect=mock_results)
        self.checker.parse_condition_results = Mock(side_effect=[
            (True, [{"result": "True", "next_id": "end", "location": "A1"}]),
        ])
        
        # 執行測試
        success, collected_data = self.checker.check_conditions_from_id(start_id=100)
        
        # 驗證結果
        self.assertTrue(success)
        self.assertEqual(collected_data["location"], "A1")
        self.mock_logger.info.assert_any_call("✅ 遇到結束標記 'end'，條件檢查完成")

    def test_or_logic_first_false_second_true(self):
        """測試 OR 邏輯：第一個條件為 False，第二個條件為 True"""
        # Mock 資料庫查詢結果
        mock_results = [
            # ID 100: 第一個條件不滿足，但有 next_id
            [{"result": "False", "next_id": "101"}],
            # ID 101: 第二個條件滿足
            [{"result": "True", "next_id": "end", "location": "B1"}],
        ]
        
        self.checker.get_task_condition_results = Mock(side_effect=mock_results)
        self.checker.parse_condition_results = Mock(side_effect=[
            (True, [{"result": "False", "next_id": "101"}]),
            (True, [{"result": "True", "next_id": "end", "location": "B1"}]),
        ])
        
        # 執行測試
        success, collected_data = self.checker.check_conditions_from_id(start_id=100)
        
        # 驗證結果
        self.assertTrue(success)
        self.assertEqual(collected_data["location"], "B1")
        self.mock_logger.info.assert_any_call("📋 ID 100 條件不滿足，但有 next_id: 101，繼續探索")
        self.mock_logger.info.assert_any_call("✅ 遇到結束標記 'end'，條件檢查完成")

    def test_or_logic_all_false_with_next_id(self):
        """測試 OR 邏輯：所有條件都為 False 但都有 next_id"""
        # Mock 資料庫查詢結果
        mock_results = [
            # ID 100: 條件不滿足，跳到 101
            [{"result": "False", "next_id": "101"}],
            # ID 101: 條件不滿足，跳到 102
            [{"result": "False", "next_id": "102"}],
            # ID 102: 條件不滿足，但結束
            [{"result": "False", "next_id": "end"}],
        ]
        
        self.checker.get_task_condition_results = Mock(side_effect=mock_results)
        self.checker.parse_condition_results = Mock(side_effect=[
            (True, [{"result": "False", "next_id": "101"}]),
            (True, [{"result": "False", "next_id": "102"}]),
            (True, [{"result": "False", "next_id": "end"}]),
        ])
        
        # 執行測試
        success, collected_data = self.checker.check_conditions_from_id(start_id=100)
        
        # 驗證結果
        self.assertFalse(success)  # 所有條件都不滿足，但有探索路徑
        self.mock_logger.info.assert_any_call("📋 ID 100 條件不滿足，但有 next_id: 101，繼續探索")
        self.mock_logger.info.assert_any_call("📋 ID 101 條件不滿足，但有 next_id: 102，繼續探索")
        self.mock_logger.info.assert_any_call("⚠️ 條件不滿足但遇到結束標記，條件檢查結束")

    def test_or_logic_false_without_next_id(self):
        """測試 OR 邏輯：條件為 False 且沒有 next_id"""
        # Mock 資料庫查詢結果
        mock_results = [
            # ID 100: 條件不滿足，沒有 next_id，會回到起始點
            [{"result": "False", "next_id": None}],
            # 回到起始點後再次查詢（模擬循環）
            [{"result": "False", "next_id": None}],
        ]
        
        self.checker.get_task_condition_results = Mock(side_effect=mock_results)
        self.checker.parse_condition_results = Mock(side_effect=[
            (True, [{"result": "False", "next_id": None}]),
            (True, [{"result": "False", "next_id": None}]),
        ])
        
        # 設定較小的最大迭代次數以避免長時間等待
        self.checker.max_iterations = 3
        
        # 執行測試
        success, collected_data = self.checker.check_conditions_from_id(start_id=100)
        
        # 驗證結果
        self.assertFalse(success)
        self.mock_logger.warning.assert_called_with("⚠️ 達到最大迭代次數 3，停止條件檢查")

    def test_or_logic_mixed_scenario(self):
        """測試 OR 邏輯：混合場景 - 多個 False 後找到 True"""
        # Mock 資料庫查詢結果
        mock_results = [
            # ID 100: 檢查位置 A，不可用
            [{"result": "False", "next_id": "101", "checked_location": "A"}],
            # ID 101: 檢查位置 B，不可用
            [{"result": "False", "next_id": "102", "checked_location": "B"}],
            # ID 102: 檢查位置 C，可用！
            [{"result": "True", "next_id": "end", "location": "C", "checked_location": "C"}],
        ]
        
        self.checker.get_task_condition_results = Mock(side_effect=mock_results)
        self.checker.parse_condition_results = Mock(side_effect=[
            (True, [{"result": "False", "next_id": "101", "checked_location": "A"}]),
            (True, [{"result": "False", "next_id": "102", "checked_location": "B"}]),
            (True, [{"result": "True", "next_id": "end", "location": "C", "checked_location": "C"}]),
        ])
        
        # 執行測試
        success, collected_data = self.checker.check_conditions_from_id(start_id=100)
        
        # 驗證結果
        self.assertTrue(success)
        self.assertEqual(collected_data["location"], "C")
        self.assertEqual(collected_data["checked_location"], "C")
        
        # 驗證日誌
        self.mock_logger.info.assert_any_call("📋 ID 100 條件不滿足，但有 next_id: 101，繼續探索")
        self.mock_logger.info.assert_any_call("📋 ID 101 條件不滿足，但有 next_id: 102，繼續探索")
        self.mock_logger.info.assert_any_call("📝 ID 102 條件滿足，next_id: end")


if __name__ == '__main__':
    unittest.main()
