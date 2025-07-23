#!/usr/bin/env python3
"""
測試 AGV 旋轉狀態條件查詢
"""

import unittest
from unittest.mock import Mock, patch
import sys
import os

# 添加路徑以便匯入模組
current_dir = os.path.dirname(__file__)
parent_dir = os.path.join(current_dir, '..')
sys.path.append(parent_dir)

try:
    from db_proxy.sql.init_data.task_condition_samples import sample_conditions
except ImportError:
    # 如果上面的路徑不行，嘗試直接匯入
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "task_condition_samples",
        os.path.join(parent_dir, "db_proxy", "sql", "init_data", "task_condition_samples.py")
    )
    task_condition_samples = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(task_condition_samples)
    sample_conditions = task_condition_samples.sample_conditions


class TestAGVRotationCondition(unittest.TestCase):
    """測試 AGV 旋轉狀態條件查詢"""

    def setUp(self):
        """設置測試環境"""
        # 找到 AGV 旋轉狀態相關的條件
        self.condition_301 = None
        self.condition_302 = None
        
        for condition in sample_conditions:
            if condition["id"] == 301:
                self.condition_301 = condition
            elif condition["id"] == 302:
                self.condition_302 = condition

    def test_condition_301_exists(self):
        """測試條件 301 是否存在"""
        self.assertIsNotNone(self.condition_301, "條件 301 應該存在")
        self.assertEqual(self.condition_301["id"], 301)
        self.assertEqual(self.condition_301["description"], "檢查等待旋轉狀態的 AGV 是否有子任務")

    def test_condition_302_exists(self):
        """測試條件 302 是否存在"""
        self.assertIsNotNone(self.condition_302, "條件 302 應該存在")
        self.assertEqual(self.condition_302["id"], 302)
        self.assertEqual(self.condition_302["description"], "等待旋轉狀態 AGV 的子任務詳情")

    def test_condition_301_sql_structure(self):
        """測試條件 301 的 SQL 結構"""
        sql = self.condition_301["conditions"]
        
        # 檢查關鍵字
        self.assertIn("agv_context", sql, "應該包含 agv_context 表")
        self.assertIn("task", sql, "應該包含 task 表")
        self.assertIn("current_state = 'wait_rotation_state'", sql, "應該檢查等待旋轉狀態")
        self.assertIn("parent_task_id", sql, "應該檢查 parent_task_id")
        self.assertIn("EXISTS", sql, "應該使用 EXISTS 子查詢")
        
        # 檢查返回欄位
        self.assertIn("agv_id", sql, "應該返回 agv_id")
        self.assertIn("task_id", sql, "應該返回 task_id")
        self.assertIn("result", sql, "應該返回 result")
        self.assertIn("next_id", sql, "應該返回 next_id")
        self.assertIn("end", sql, "應該返回 end")

    def test_condition_302_sql_structure(self):
        """測試條件 302 的 SQL 結構"""
        sql = self.condition_302["conditions"]
        
        # 檢查關鍵字
        self.assertIn("agv_context", sql, "應該包含 agv_context 表")
        self.assertIn("LEFT JOIN", sql, "應該使用 LEFT JOIN")
        self.assertIn("GROUP BY", sql, "應該使用 GROUP BY")
        self.assertIn("COUNT", sql, "應該計算子任務數量")
        
        # 檢查返回欄位
        self.assertIn("parent_task_id", sql, "應該返回 parent_task_id")
        self.assertIn("child_task_id", sql, "應該返回 child_task_id")
        self.assertIn("child_task_count", sql, "應該返回子任務數量")

    def test_sql_logic_flow(self):
        """測試 SQL 邏輯流程"""
        print("\n=== 測試 SQL 邏輯流程 ===")
        
        # 條件 301 的邏輯
        print("條件 301 邏輯:")
        print("1. 查詢 agv_context 中 current_state = 'wait_rotation_state' 的記錄")
        print("2. JOIN task 表取得對應的 task.id")
        print("3. 使用 EXISTS 檢查是否有其他 task 的 parent_task_id = task.id")
        print("4. 如果有子任務 → result='True', next_id=302")
        print("5. 如果沒有子任務 → result='False', end='True'")
        
        # 條件 302 的邏輯
        print("\n條件 302 邏輯:")
        print("1. 查詢 agv_context 中 current_state = 'wait_rotation_state' 的記錄")
        print("2. JOIN task 表取得父任務")
        print("3. LEFT JOIN task 表取得子任務詳情")
        print("4. GROUP BY 統計子任務數量")
        print("5. 返回子任務的詳細資訊")

    def test_expected_scenarios(self):
        """測試預期場景"""
        print("\n=== 預期場景測試 ===")
        
        scenarios = [
            {
                "name": "場景 1：AGV 等待旋轉且有子任務",
                "agv_context": {"agv_id": "AGV001", "current_state": "wait_rotation_state"},
                "parent_task": {"id": 1001, "agv_id": "AGV001"},
                "child_tasks": [{"id": 1002, "parent_task_id": 1001}],
                "expected_301": {"result": "True", "next_id": 302},
                "expected_302": {"result": "True", "child_task_count": 1}
            },
            {
                "name": "場景 2：AGV 等待旋轉但無子任務",
                "agv_context": {"agv_id": "AGV002", "current_state": "wait_rotation_state"},
                "parent_task": {"id": 1003, "agv_id": "AGV002"},
                "child_tasks": [],
                "expected_301": {"result": "False", "end": "True"},
                "expected_302": {"result": "False", "child_task_count": 0}
            },
            {
                "name": "場景 3：AGV 非等待旋轉狀態",
                "agv_context": {"agv_id": "AGV003", "current_state": "working"},
                "parent_task": {"id": 1004, "agv_id": "AGV003"},
                "child_tasks": [],
                "expected_301": {"result": "無記錄"},
                "expected_302": {"result": "無記錄"}
            }
        ]
        
        for scenario in scenarios:
            print(f"\n{scenario['name']}:")
            print(f"  AGV 狀態: {scenario['agv_context']}")
            print(f"  父任務: {scenario['parent_task']}")
            print(f"  子任務: {scenario['child_tasks']}")
            print(f"  條件 301 預期: {scenario['expected_301']}")
            print(f"  條件 302 預期: {scenario['expected_302']}")

    def test_performance_considerations(self):
        """測試效能考量"""
        print("\n=== 效能考量 ===")
        
        print("索引建議:")
        print("1. agv_context.current_state - 加速狀態篩選")
        print("2. agv_context.agv_id - 加速 JOIN")
        print("3. task.agv_id - 加速 JOIN")
        print("4. task.parent_task_id - 加速子任務查詢")
        
        print("\nSQL 優化:")
        print("1. 使用 EXISTS 而非 IN - 效能更好")
        print("2. 適當的 JOIN 順序 - 先篩選再 JOIN")
        print("3. GROUP BY 只在必要時使用")

    def test_integration_with_condition_checker(self):
        """測試與條件檢查器的整合"""
        print("\n=== 與條件檢查器整合 ===")
        
        print("使用方式:")
        print("1. 條件檢查器調用條件 301")
        print("2. 如果 result='True' → 跳轉到條件 302")
        print("3. 如果 result='False' → end='True'，結束檢查")
        print("4. 條件 302 提供詳細的子任務資訊")
        
        print("\n返回資料結構:")
        print("條件 301: {agv_id, task_id, result, next_id, end}")
        print("條件 302: {agv_id, parent_task_id, child_task_id, child_work_id, child_status, child_task_count, result, next_id, end}")


if __name__ == '__main__':
    unittest.main(verbosity=2)
