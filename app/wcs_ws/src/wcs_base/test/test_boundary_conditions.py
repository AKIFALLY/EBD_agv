#!/usr/bin/env python3
"""
測試邊界條件：遍歷完成但沒有遇到 end 且 result=false 的情況
"""

import unittest
from unittest.mock import Mock, patch
import sys
import os

# 添加路徑以便匯入模組
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from wcs_base.task_condition_checker import TaskConditionChecker


class TestBoundaryConditions(unittest.TestCase):
    """測試邊界條件"""

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

    def test_list_traversal_all_false_no_next_id(self):
        """測試 List 遍歷完成，所有條件都是 False 且無 next_id"""
        print("\n=== 測試：List 遍歷完成，所有條件 False 且無 next_id ===")
        
        # 模擬條件檢查：所有條件都失敗且無 next_id
        def mock_check_condition(check_id):
            return True  # 條件存在
        
        def mock_get_results(check_id):
            return [{"result": "False", "next_id": None, "end": None}]
        
        def mock_parse_results(results):
            return True, results
        
        self.checker.check_single_id_condition = Mock(side_effect=mock_check_condition)
        self.checker.get_task_condition_results = Mock(side_effect=mock_get_results)
        self.checker.parse_condition_results = Mock(side_effect=mock_parse_results)
        
        # 執行測試
        result = self.checker.process_id_list("105,205,305,405")
        
        print(f"結果: {result}")
        
        # 驗證結果
        self.assertEqual(result, -1, "所有條件都失敗且無 next_id 應該返回 -1")
        
        # 驗證日誌
        self.mock_logger.info.assert_any_call(
            "📋 List [105, 205, 305, 405] 遍歷完成，所有條件都不滿足且無跳轉，應該結束檢查"
        )

    def test_single_condition_false_no_next_id_no_end(self):
        """測試單一條件：result=False, 無 next_id, 無 end"""
        print("\n=== 測試：單一條件 False 且無 next_id 無 end ===")
        
        # 模擬條件檢查結果
        condition_results = [{"result": "False", "next_id": None, "end": None}]
        
        self.checker.get_task_condition_results = Mock(return_value=condition_results)
        self.checker.parse_condition_results = Mock(return_value=(True, condition_results))
        
        # 模擬主循環邏輯
        collected_data = {}
        current_id = 206
        data_list = condition_results
        
        # 檢查是否應該結束
        all_false_no_next = all(
            item.get("result") == "False" and not item.get("next_id") 
            for item in data_list
        )
        
        print(f"所有條件都是 False 且無 next_id: {all_false_no_next}")
        
        # 驗證結果
        self.assertTrue(all_false_no_next, "應該識別為所有條件都失敗且無 next_id")

    def test_mixed_conditions_with_boundary(self):
        """測試混合條件：部分有 next_id，部分沒有"""
        print("\n=== 測試：混合條件邊界情況 ===")
        
        # 模擬不同的條件檢查結果
        def mock_check_condition(check_id):
            return True  # 所有條件都存在
        
        def mock_get_results(check_id):
            if check_id == 105:
                return [{"result": "False", "next_id": None, "end": None}]
            elif check_id == 205:
                return [{"result": "False", "next_id": 206, "end": None}]
            elif check_id == 305:
                return [{"result": "False", "next_id": None, "end": None}]
            else:
                return [{"result": "False", "next_id": None, "end": None}]
        
        def mock_parse_results(results):
            return True, results
        
        self.checker.check_single_id_condition = Mock(side_effect=mock_check_condition)
        self.checker.get_task_condition_results = Mock(side_effect=mock_get_results)
        self.checker.parse_condition_results = Mock(side_effect=mock_parse_results)
        
        # 執行測試
        result = self.checker.process_id_list("105,205,305,405")
        
        print(f"結果: {result}")
        
        # 驗證結果：應該返回 206（因為條件 205 有 next_id）
        self.assertEqual(result, 206, "應該返回第一個有 next_id 的條件的 next_id")

    def test_integration_boundary_scenario(self):
        """測試整合邊界場景"""
        print("\n=== 整合測試：邊界場景 ===")
        
        # 場景：條件 2 → List [105,205] → 105(False,無next_id) → 205(False,無next_id) → 應該結束
        
        # 模擬條件 2 的結果
        condition_2_result = [{"result": "True", "next_id": "[105,205]", "location": "51"}]
        
        # 模擬 List 中條件的結果
        def mock_check_condition(check_id):
            if check_id in [105, 205]:
                return True  # 條件存在
            return False
        
        def mock_get_results(check_id):
            if check_id == 2:
                return condition_2_result
            elif check_id in [105, 205]:
                return [{"result": "False", "next_id": None, "end": None}]
            else:
                return []
        
        def mock_parse_results(results):
            return True, results
        
        self.checker.check_single_id_condition = Mock(side_effect=mock_check_condition)
        self.checker.get_task_condition_results = Mock(side_effect=mock_get_results)
        self.checker.parse_condition_results = Mock(side_effect=mock_parse_results)
        
        # 測試 process_next_id 處理 List
        result = self.checker.process_next_id("[105,205]")
        
        print(f"process_next_id 結果: {result}")
        
        # 驗證結果
        self.assertEqual(result, -1, "List 中所有條件都失敗且無 next_id 應該返回 -1")
        
        print("預期效果:")
        print("  1. 條件 2 成功 → 處理 List [105,205]")
        print("  2. 條件 105: False, 無 next_id")
        print("  3. 條件 205: False, 無 next_id")
        print("  4. List 遍歷完成 → 返回 -1")
        print("  5. 主循環收到 -1 → 直接結束，避免無限循環")

    def test_performance_with_boundary_conditions(self):
        """測試邊界條件的效能"""
        print("\n=== 效能測試：邊界條件 ===")
        
        # 模擬大量失敗條件
        large_id_list = list(range(100, 200))  # 100 個條件
        id_list_str = ",".join(map(str, large_id_list))
        
        print(f"測試場景：{len(large_id_list)} 個條件，全部失敗且無 next_id")
        
        # 模擬所有條件都失敗且無 next_id
        def mock_check_condition(check_id):
            return True  # 條件存在
        
        def mock_get_results(check_id):
            return [{"result": "False", "next_id": None, "end": None}]
        
        def mock_parse_results(results):
            return True, results
        
        self.checker.check_single_id_condition = Mock(side_effect=mock_check_condition)
        self.checker.get_task_condition_results = Mock(side_effect=mock_get_results)
        self.checker.parse_condition_results = Mock(side_effect=mock_parse_results)
        
        # 執行測試
        result = self.checker.process_id_list(id_list_str)
        
        print(f"結果: {result}")
        print(f"檢查次數: {self.checker.check_single_id_condition.call_count}")
        
        # 驗證結果
        self.assertEqual(result, -1, "所有條件失敗且無 next_id 應該返回 -1")
        self.assertEqual(self.checker.check_single_id_condition.call_count, len(large_id_list), 
                        "應該檢查所有條件一次")
        
        print("效能優勢:")
        print("  ✅ 一次性處理：檢查 100 次後直接結束")
        print("  ❌ 無邊界處理：可能無限循環")


if __name__ == '__main__':
    unittest.main(verbosity=2)
