#!/usr/bin/env python3
"""
測試智能 List 遍歷邏輯
"""

import unittest
from unittest.mock import Mock, patch
import sys
import os

# 添加路徑以便匯入模組
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from wcs_base.task_condition_checker import TaskConditionChecker


class TestSmartListTraversal(unittest.TestCase):
    """測試智能 List 遍歷邏輯"""

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

    def test_list_traversal_state_management(self):
        """測試 List 遍歷狀態管理"""
        id_list = [105, 205, 305, 405]
        
        # 初始化 List 遍歷
        list_key = self.checker._init_list_traversal(id_list)
        
        # 驗證初始狀態
        self.assertFalse(self.checker._is_list_completed(list_key))
        
        # 依序取得 ID
        ids = []
        for i in range(len(id_list)):
            next_id = self.checker._get_next_id_from_list(list_key)
            if next_id:
                ids.append(next_id)
        
        # 驗證取得的 ID 順序
        self.assertEqual(ids, id_list)
        
        # 驗證 List 已完成
        self.assertTrue(self.checker._is_list_completed(list_key))
        
        # 再次嘗試取得 ID 應該返回 None
        self.assertIsNone(self.checker._get_next_id_from_list(list_key))

    def test_smart_exit_logic_no_list(self):
        """測試智能跳出邏輯：無 List 遍歷問題"""
        # 沒有 List 上下文
        self.checker.current_list_context = None
        
        # 遇到 end 條件應該直接跳出
        should_exit = self.checker._should_exit_on_end_condition("True")
        self.assertTrue(should_exit)

    def test_smart_exit_logic_list_completed(self):
        """測試智能跳出邏輯：List 遍歷已完成"""
        id_list = [105, 205]
        list_key = self.checker._init_list_traversal(id_list)
        self.checker.current_list_context = list_key
        
        # 完成 List 遍歷
        self.checker._get_next_id_from_list(list_key)
        self.checker._get_next_id_from_list(list_key)
        
        # 遇到 end 條件應該跳出
        should_exit = self.checker._should_exit_on_end_condition("True")
        self.assertTrue(should_exit)

    def test_smart_exit_logic_list_not_completed(self):
        """測試智能跳出邏輯：List 遍歷未完成"""
        id_list = [105, 205, 305]
        list_key = self.checker._init_list_traversal(id_list)
        self.checker.current_list_context = list_key
        
        # 只取得第一個 ID
        self.checker._get_next_id_from_list(list_key)
        
        # 遇到 end 條件不應該跳出
        should_exit = self.checker._should_exit_on_end_condition("True")
        self.assertFalse(should_exit)

    def test_process_id_list_smart_traversal(self):
        """測試智能 List 處理"""
        # Mock check_single_id_condition
        def mock_check_condition(check_id):
            if check_id == 105:
                return False  # 第一個失敗
            elif check_id == 205:
                return True   # 第二個成功
            else:
                return False
        
        self.checker.check_single_id_condition = Mock(side_effect=mock_check_condition)
        
        # 第一次調用：應該返回 105（即使失敗，也要讓主循環處理）
        result1 = self.checker.process_id_list("105,205,305,405")
        self.assertEqual(result1, 105)
        
        # 檢查 List 狀態
        self.assertIsNotNone(self.checker.current_list_context)
        self.assertFalse(self.checker._is_list_completed(self.checker.current_list_context))

    def test_integration_scenario_success(self):
        """測試整合場景：成功找到條件"""
        print("\n=== 整合測試：成功場景 ===")
        
        # 模擬條件檢查結果
        def mock_check_condition(check_id):
            if check_id == 105:
                print(f"  檢查 ID {check_id}: 不存在")
                return False
            elif check_id == 205:
                print(f"  檢查 ID {check_id}: 條件滿足")
                return True
            else:
                print(f"  檢查 ID {check_id}: 不存在")
                return False
        
        self.checker.check_single_id_condition = Mock(side_effect=mock_check_condition)
        
        # 處理 List
        print("1. 開始處理 List: [105,205,305,405]")
        result = self.checker.process_id_list("105,205,305,405")
        print(f"2. 第一次處理結果: {result}")
        
        # 驗證結果
        self.assertEqual(result, 105)  # 返回第一個 ID 讓主循環處理
        
        print("3. 預期流程:")
        print("   - 主循環檢查 105 → 失敗 → 繼續 List 遍歷")
        print("   - 取得下一個 ID 205 → 成功 → 跳轉到 206")

    def test_integration_scenario_all_fail(self):
        """測試整合場景：所有條件都失敗"""
        print("\n=== 整合測試：全部失敗場景 ===")
        
        # 模擬所有條件都失敗
        def mock_check_condition(check_id):
            print(f"  檢查 ID {check_id}: 失敗")
            return False
        
        self.checker.check_single_id_condition = Mock(side_effect=mock_check_condition)
        
        # 模擬完整的 List 遍歷
        id_list = [105, 205, 305, 405]
        list_key = self.checker._init_list_traversal(id_list)
        self.checker.current_list_context = list_key
        
        print("1. 開始遍歷 List: [105,205,305,405]")
        
        # 依序處理每個 ID
        for i, expected_id in enumerate(id_list):
            next_id = self.checker._get_next_id_from_list(list_key)
            print(f"2.{i+1} 取得 ID: {next_id}")
            self.assertEqual(next_id, expected_id)
        
        # 檢查 List 是否完成
        is_completed = self.checker._is_list_completed(list_key)
        print(f"3. List 遍歷完成: {is_completed}")
        self.assertTrue(is_completed)
        
        # 再次嘗試取得 ID
        next_id = self.checker._get_next_id_from_list(list_key)
        print(f"4. 再次取得 ID: {next_id}")
        self.assertIsNone(next_id)
        
        print("5. 預期結果:")
        print("   - List 遍歷完成後，遇到 end=True 應該跳出")
        print("   - 避免無限循環")

    def test_performance_comparison(self):
        """測試效能比較"""
        print("\n=== 效能比較 ===")
        
        # 模擬大量條件
        large_id_list = list(range(100, 200))  # 100 個條件
        
        print(f"測試場景：{len(large_id_list)} 個條件，只有最後一個成功")
        
        # 模擬只有最後一個條件成功
        def mock_check_condition(check_id):
            return check_id == 199  # 只有最後一個成功
        
        self.checker.check_single_id_condition = Mock(side_effect=mock_check_condition)
        
        # 使用智能遍歷
        list_key = self.checker._init_list_traversal(large_id_list)
        self.checker.current_list_context = list_key
        
        # 模擬遍歷過程
        check_count = 0
        while not self.checker._is_list_completed(list_key):
            next_id = self.checker._get_next_id_from_list(list_key)
            if next_id:
                check_count += 1
                if self.checker.check_single_id_condition(next_id):
                    print(f"✅ 在第 {check_count} 次檢查找到成功條件: {next_id}")
                    break
        
        print(f"智能遍歷：檢查 {check_count} 次")
        print("傳統方式：可能需要數百次迴圈（回到起始點重複檢查）")
        
        self.assertEqual(check_count, len(large_id_list))


if __name__ == '__main__':
    unittest.main(verbosity=2)
