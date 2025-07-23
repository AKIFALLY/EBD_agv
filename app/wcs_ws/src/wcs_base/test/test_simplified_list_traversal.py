#!/usr/bin/env python3
"""
測試簡化版本的 List 遍歷邏輯
"""

import unittest
from unittest.mock import Mock, patch
import sys
import os

# 添加路徑以便匯入模組
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from wcs_base.task_condition_checker import TaskConditionChecker


class TestSimplifiedListTraversal(unittest.TestCase):
    """測試簡化版本的 List 遍歷邏輯"""

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

    def test_process_id_list_success(self):
        """測試 List 處理成功場景"""
        # Mock check_single_id_condition
        def mock_check_condition(check_id):
            if check_id == 205:
                return True  # 條件 205 成功
            else:
                return False
        
        self.checker.check_single_id_condition = Mock(side_effect=mock_check_condition)
        
        # 執行測試
        result = self.checker.process_id_list("105,205,305,405")
        
        # 驗證結果
        self.assertEqual(result, 205, "應該返回第一個滿足條件的 ID")
        
        # 驗證調用順序
        expected_calls = [unittest.mock.call(105), unittest.mock.call(205)]
        self.checker.check_single_id_condition.assert_has_calls(expected_calls)

    def test_process_id_list_all_fail(self):
        """測試 List 處理全部失敗場景"""
        # Mock 所有條件都失敗
        self.checker.check_single_id_condition = Mock(return_value=False)
        
        # 執行測試
        result = self.checker.process_id_list("105,205,305,405")
        
        # 驗證結果
        self.assertEqual(result, -1, "所有條件都失敗應該返回 -1")
        
        # 驗證所有 ID 都被檢查
        expected_calls = [
            unittest.mock.call(105),
            unittest.mock.call(205),
            unittest.mock.call(305),
            unittest.mock.call(405)
        ]
        self.checker.check_single_id_condition.assert_has_calls(expected_calls)

    def test_process_id_list_duplicate_prevention(self):
        """測試重複處理防止機制"""
        # Mock 條件檢查
        self.checker.check_single_id_condition = Mock(return_value=False)
        
        # 第一次處理
        result1 = self.checker.process_id_list("105,205,305,405")
        self.assertEqual(result1, -1)
        
        # 第二次處理相同的 List
        result2 = self.checker.process_id_list("105,205,305,405")
        self.assertEqual(result2, -1, "重複的 List 應該直接返回 -1")
        
        # 驗證第二次沒有實際檢查條件
        # 第一次檢查了 4 個 ID，第二次應該沒有額外檢查
        self.assertEqual(self.checker.check_single_id_condition.call_count, 4)

    def test_reset_processed_lists(self):
        """測試重置已處理 List 功能"""
        # 處理一個 List
        self.checker.check_single_id_condition = Mock(return_value=False)
        result1 = self.checker.process_id_list("105,205")
        self.assertEqual(result1, -1)
        
        # 重置
        self.checker.reset_processed_lists()
        
        # 再次處理相同的 List 應該重新執行
        result2 = self.checker.process_id_list("105,205")
        self.assertEqual(result2, -1)
        
        # 驗證兩次都執行了檢查
        self.assertEqual(self.checker.check_single_id_condition.call_count, 4)  # 2次 * 2個ID

    def test_integration_scenario_avoid_infinite_loop(self):
        """測試整合場景：避免無限循環"""
        print("\n=== 整合測試：避免無限循環 ===")
        
        # 模擬條件 2 的結果
        condition_2_result = [{"result": "True", "next_id": "[105,205,305,405]", "location": "51"}]
        
        # 模擬所有 List 中的條件都失敗
        def mock_check_condition(check_id):
            print(f"  檢查條件 {check_id}: 失敗")
            return False
        
        self.checker.check_single_id_condition = Mock(side_effect=mock_check_condition)
        self.checker.get_task_condition_results = Mock(return_value=condition_2_result)
        self.checker.parse_condition_results = Mock(return_value=(True, condition_2_result))
        
        # 模擬主循環邏輯
        print("1. 條件 2 成功，next_id: [105,205,305,405]")
        
        # 處理 next_id
        next_id = "[105,205,305,405]"
        result = self.checker.process_next_id(next_id)
        
        print(f"2. process_next_id 結果: {result}")
        
        if result == -1:
            print("3. ✅ List 遍歷完成，無滿足條件，應該結束檢查")
            success = False
        else:
            print("3. ❌ 未正確處理 List 完成狀態")
            success = True
        
        # 驗證結果
        self.assertEqual(result, -1, "List 全部失敗應該返回 -1")
        self.assertFalse(success, "應該正確結束，避免無限循環")
        
        print("4. 預期效果:")
        print("   - 一次性檢查完整個 List")
        print("   - 避免回到起始點重複檢查")
        print("   - 直接結束，不會無限循環")

    def test_performance_comparison(self):
        """測試效能比較"""
        print("\n=== 效能比較 ===")
        
        # 模擬大量條件都失敗的情況
        large_id_list = list(range(100, 200))  # 100 個條件
        id_list_str = ",".join(map(str, large_id_list))
        
        print(f"測試場景：{len(large_id_list)} 個條件，全部失敗")
        
        # 模擬所有條件都失敗
        self.checker.check_single_id_condition = Mock(return_value=False)
        
        # 使用簡化版本處理
        result = self.checker.process_id_list(id_list_str)
        
        print(f"簡化版本結果: {result}")
        print(f"檢查次數: {self.checker.check_single_id_condition.call_count}")
        
        # 驗證結果
        self.assertEqual(result, -1, "所有條件失敗應該返回 -1")
        self.assertEqual(self.checker.check_single_id_condition.call_count, len(large_id_list), 
                        "應該檢查所有條件一次")
        
        print("效能優勢:")
        print("  ✅ 一次性處理：檢查 100 次")
        print("  ❌ 傳統方式：可能檢查數千次（重複回到起始點）")

    def test_mixed_scenario(self):
        """測試混合場景：部分成功部分失敗"""
        print("\n=== 混合場景測試 ===")
        
        # 模擬條件檢查：105失敗，205成功
        def mock_check_condition(check_id):
            if check_id == 105:
                print(f"  檢查 ID {check_id}: 失敗")
                return False
            elif check_id == 205:
                print(f"  檢查 ID {check_id}: 成功")
                return True
            else:
                print(f"  檢查 ID {check_id}: 失敗")
                return False
        
        self.checker.check_single_id_condition = Mock(side_effect=mock_check_condition)
        
        # 執行測試
        print("1. 處理 List: [105,205,305,405]")
        result = self.checker.process_id_list("105,205,305,405")
        
        print(f"2. 結果: {result}")
        
        # 驗證結果
        self.assertEqual(result, 205, "應該返回第一個成功的 ID")
        
        # 驗證只檢查到成功為止
        expected_calls = [unittest.mock.call(105), unittest.mock.call(205)]
        self.checker.check_single_id_condition.assert_has_calls(expected_calls)
        
        print("3. 優化效果:")
        print("   - 找到成功條件後立即返回")
        print("   - 不會繼續檢查後續條件")
        print("   - 避免不必要的資料庫查詢")


if __name__ == '__main__':
    unittest.main(verbosity=2)
