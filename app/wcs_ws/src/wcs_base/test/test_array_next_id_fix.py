#!/usr/bin/env python3
"""
測試修正後的陣列 next_id 處理邏輯
"""

import unittest
from unittest.mock import Mock, patch
import sys
import os

# 添加路徑以便匯入模組
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from wcs_base.task_condition_checker import TaskConditionChecker


class TestArrayNextIdFix(unittest.TestCase):
    """測試陣列 next_id 修正"""

    def setUp(self):
        """設置測試環境"""
        # 建立 mock logger 和 db_manager
        self.mock_logger = Mock()
        self.mock_db_manager = Mock()
        
        # 建立 TaskConditionChecker 實例
        self.checker = TaskConditionChecker(
            db_manager=self.mock_db_manager,
            logger=self.mock_logger,
            real_time_mode=False
        )
        
        # Mock 必要的屬性
        self.checker.pool_agvc = Mock()
        self.checker.config = Mock()
        self.checker.config.cache_timeout = 300

    def test_process_next_id_with_brackets(self):
        """測試處理帶方括號的 next_id 陣列"""
        # 測試帶方括號的格式
        next_id = "[105,205,305,405]"
        
        # Mock process_id_list 方法
        self.checker.process_id_list = Mock(return_value=205)
        
        result = self.checker.process_next_id(next_id)
        
        # 驗證結果
        self.assertEqual(result, 205)
        # 驗證 process_id_list 被正確調用（移除方括號）
        self.checker.process_id_list.assert_called_once_with("105,205,305,405")

    def test_process_next_id_without_brackets(self):
        """測試處理不帶方括號的 next_id 陣列"""
        # 測試不帶方括號的格式
        next_id = "105,205,305,405"
        
        # Mock process_id_list 方法
        self.checker.process_id_list = Mock(return_value=105)
        
        result = self.checker.process_next_id(next_id)
        
        # 驗證結果
        self.assertEqual(result, 105)
        # 驗證 process_id_list 被正確調用
        self.checker.process_id_list.assert_called_once_with("105,205,305,405")

    def test_check_single_id_condition_with_or_logic(self):
        """測試支援 OR 邏輯的單一 ID 條件檢查"""
        # Mock 資料庫查詢結果
        mock_condition_result = [{"result": "False", "next_id": "206"}]
        
        self.checker.get_task_condition_results = Mock(return_value=mock_condition_result)
        self.checker.parse_condition_results = Mock(return_value=(True, mock_condition_result))
        
        # 執行測試
        result = self.checker.check_single_id_condition(205)
        
        # 驗證結果：即使 result 為 False，但有 next_id 應該返回 True
        self.assertTrue(result)
        
        # 驗證日誌
        self.mock_logger.info.assert_called_with(
            "📋 ID 205 條件不滿足但有 next_id: 206，視為可繼續"
        )

    def test_check_single_id_condition_true_result(self):
        """測試 result 為 True 的情況"""
        # Mock 資料庫查詢結果
        mock_condition_result = [{"result": "True", "next_id": "207"}]
        
        self.checker.get_task_condition_results = Mock(return_value=mock_condition_result)
        self.checker.parse_condition_results = Mock(return_value=(True, mock_condition_result))
        
        # 執行測試
        result = self.checker.check_single_id_condition(205)
        
        # 驗證結果
        self.assertTrue(result)

    def test_check_single_id_condition_false_without_next_id(self):
        """測試 result 為 False 且沒有 next_id 的情況"""
        # Mock 資料庫查詢結果
        mock_condition_result = [{"result": "False", "next_id": None}]
        
        self.checker.get_task_condition_results = Mock(return_value=mock_condition_result)
        self.checker.parse_condition_results = Mock(return_value=(True, mock_condition_result))
        
        # 執行測試
        result = self.checker.check_single_id_condition(205)
        
        # 驗證結果：沒有 next_id 應該返回 False
        self.assertFalse(result)

    def test_process_id_list_with_or_logic(self):
        """測試 ID 列表處理支援 OR 邏輯"""
        # Mock check_single_id_condition 方法
        def mock_check_condition(check_id):
            if check_id == 105:
                return False  # 條件 105 失敗
            elif check_id == 205:
                return True   # 條件 205 成功（可能是 OR 邏輯）
            else:
                return False
        
        self.checker.check_single_id_condition = Mock(side_effect=mock_check_condition)
        
        # 執行測試
        result = self.checker.process_id_list("105,205,305,405")
        
        # 驗證結果：應該返回第一個成功的 ID
        self.assertEqual(result, 205)
        
        # 驗證調用順序
        expected_calls = [unittest.mock.call(105), unittest.mock.call(205)]
        self.checker.check_single_id_condition.assert_has_calls(expected_calls)

    def test_integration_scenario(self):
        """測試整合場景：模擬實際的條件檢查流程"""
        print("\n=== 整合測試場景 ===")
        
        # 模擬條件 2 的結果
        condition_2_result = {
            "result": "True",
            "next_id": "[105,205,305,405,505,605,705,805,905,1005]",
            "location": "51"
        }
        
        # 模擬各個條件的檢查結果
        def mock_check_condition(check_id):
            if check_id == 105:
                print(f"  檢查條件 {check_id}: 不存在")
                return False
            elif check_id == 205:
                print(f"  檢查條件 {check_id}: result=False, next_id=206 (OR邏輯成功)")
                return True  # OR 邏輯：有 next_id 視為成功
            else:
                print(f"  檢查條件 {check_id}: 不存在")
                return False
        
        self.checker.check_single_id_condition = Mock(side_effect=mock_check_condition)
        
        # 測試 next_id 處理
        print("\n1. 處理條件 2 的 next_id 陣列:")
        next_id = condition_2_result["next_id"]
        print(f"   原始 next_id: {next_id}")
        
        result = self.checker.process_next_id(next_id)
        print(f"   處理結果: {result}")
        
        # 驗證結果
        self.assertEqual(result, 205, "應該選擇條件 205")
        
        print("\n2. 預期流程:")
        print("   條件 2 (有空位) → 嘗試 105 (失敗) → 嘗試 205 (OR邏輯成功) → 跳轉到 206")
        
        self.assertTrue(True)  # 測試總是通過，這只是展示流程


if __name__ == '__main__':
    unittest.main(verbosity=2)
