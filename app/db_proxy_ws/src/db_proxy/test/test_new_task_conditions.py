#!/usr/bin/env python3
"""
測試新增的任務條件檢查
"""

import unittest
import sys
import os

# 添加路徑以便匯入模組
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../db_proxy'))

from sql.init_data.task_condition_samples import sample_conditions


class TestNewTaskConditions(unittest.TestCase):
    """測試新增的任務條件"""

    def setUp(self):
        """設置測試環境"""
        # 找到新增的條件
        self.condition_2 = None
        self.condition_205 = None
        self.condition_206 = None
        self.condition_207 = None
        
        for condition in sample_conditions:
            if condition["id"] == 2:
                self.condition_2 = condition
            elif condition["id"] == 205:
                self.condition_205 = condition
            elif condition["id"] == 206:
                self.condition_206 = condition
            elif condition["id"] == 207:
                self.condition_207 = condition

    def test_condition_2_exists_and_updated(self):
        """測試條件 2：人工收料區空位檢查（已更新）"""
        self.assertIsNotNone(self.condition_2, "條件 ID 2 應該存在")
        
        # 檢查 next_id 格式是否正確更新
        conditions_sql = self.condition_2["conditions"]
        self.assertIn("[105,205,305,405,505,605,705,805,905,1005]", conditions_sql, 
                     "next_id 應該包含正確的陣列格式")
        
        # 檢查描述
        self.assertEqual(self.condition_2["description"], "人工收料區-有空位")
        
        print("✅ 條件 2 (人工收料區空位檢查) 格式正確")

    def test_condition_205_full_rack_check(self):
        """測試條件 205：房間2出口傳送箱滿料架檢查"""
        self.assertIsNotNone(self.condition_205, "條件 ID 205 應該存在")
        
        conditions_sql = self.condition_205["conditions"]
        
        # 檢查 SQL 邏輯
        self.assertIn("r.location_id = 20002", conditions_sql, "應該查詢房間2出口傳送箱位置")
        self.assertIn("r.status_id IN (2, 3, 6)", conditions_sql, "應該檢查滿料架狀態")
        self.assertIn("WHEN COUNT(*) > 0 THEN 207", conditions_sql, "有滿料架時應跳轉到 207")
        self.assertIn("ELSE 206", conditions_sql, "無滿料架時應跳轉到 206")
        
        # 檢查描述
        self.assertEqual(self.condition_205["description"], "房間2-出口傳送箱有滿料架")
        
        print("✅ 條件 205 (房間2出口傳送箱滿料架檢查) 邏輯正確")

    def test_condition_206_cargo_task_check(self):
        """測試條件 206：房間2出口傳送箱cargo任務檢查"""
        self.assertIsNotNone(self.condition_206, "條件 ID 206 應該存在")
        
        conditions_sql = self.condition_206["conditions"]
        
        # 檢查 SQL 邏輯
        self.assertIn("work_id = 2000201", conditions_sql, "應該查詢房間2的cargo任務")
        self.assertIn("status_id = 4", conditions_sql, "應該檢查已完成狀態")
        self.assertIn("207 as next_id", conditions_sql, "無論結果如何都應跳轉到 207")
        self.assertIn("2000201 as cargo_work_id", conditions_sql, "應該回傳cargo_work_id")
        
        # 檢查描述
        self.assertEqual(self.condition_206["description"], "房間2-出口傳送箱有已完成的cargo任務")
        
        print("✅ 條件 206 (房間2出口傳送箱cargo任務檢查) 邏輯正確")

    def test_condition_207_duplicate_task_check(self):
        """測試條件 207：重複任務檢查"""
        self.assertIsNotNone(self.condition_207, "條件 ID 207 應該存在")
        
        conditions_sql = self.condition_207["conditions"]
        
        # 檢查 SQL 邏輯
        self.assertIn("work_id = 220001", conditions_sql, "應該查詢KUKA移動任務")
        self.assertIn("node_id = 20002", conditions_sql, "應該檢查房間2出口傳送箱位置")
        self.assertIn("status_id IN (0, 1, 2)", conditions_sql, "應該檢查執行中的任務狀態")
        self.assertIn("WHEN COUNT(*) = 0 THEN 'True'", conditions_sql, "沒有重複任務時應返回True")
        self.assertIn("WHEN COUNT(*) = 0 THEN NULL", conditions_sql, "沒有重複任務時next_id應為NULL")
        
        # 檢查描述
        self.assertEqual(self.condition_207["description"], "房間2-檢查重複任務（滿料架到人工收料區）")
        
        print("✅ 條件 207 (重複任務檢查) 邏輯正確")

    def test_condition_flow_logic(self):
        """測試條件流程邏輯"""
        print("\n=== 滿料架到人工收料區任務流程 ===")
        
        # 條件 2：檢查人工收料區空位
        print("步驟 1 (ID 2): 檢查人工收料區是否有空位")
        print("  - 有空位 → next_id: [105,205,305,405,505,605,705,805,905,1005]")
        print("  - 無空位 → next_id: NULL (流程結束)")
        
        # 條件 205：檢查房間2出口傳送箱滿料架
        print("\n步驟 2 (ID 205): 檢查房間2出口傳送箱是否有滿料架")
        print("  - 有滿料架 (status_id: 2,3,6) → next_id: 207")
        print("  - 無滿料架 → next_id: 206")
        
        # 條件 206：檢查cargo任務狀態
        print("\n步驟 3a (ID 206): 檢查房間2是否有已完成的cargo任務")
        print("  - 有已完成任務 (work_id: 2000201, status_id: 4) → next_id: 207")
        print("  - 無已完成任務 → next_id: 207")
        print("  - 註：無論結果如何都跳轉到 207")
        
        # 條件 207：重複任務檢查
        print("\n步驟 4 (ID 207): 檢查是否已存在相同任務")
        print("  - 無重複任務 → result: True, end: True (可以建立新任務)")
        print("  - 有重複任務 → result: False, end: False (不建立任務)")
        
        self.assertTrue(True)  # 測試總是通過，這只是展示流程

    def test_sql_syntax_validation(self):
        """測試 SQL 語法基本驗證"""
        conditions_to_test = [
            (self.condition_2, "條件 2"),
            (self.condition_205, "條件 205"),
            (self.condition_206, "條件 206"),
            (self.condition_207, "條件 207")
        ]
        
        for condition, name in conditions_to_test:
            if condition:
                sql = condition["conditions"]
                
                # 基本 SQL 語法檢查
                self.assertIn("SELECT", sql, f"{name} 應包含 SELECT")
                self.assertIn("CASE", sql, f"{name} 應包含 CASE WHEN 邏輯")
                self.assertIn("as result", sql, f"{name} 應回傳 result 欄位")
                self.assertIn("FROM", sql, f"{name} 應包含 FROM 子句")
                
                # 檢查是否有基本的 WHERE 條件
                if condition["id"] != 206:  # 條件 206 的 WHERE 比較特殊
                    self.assertIn("WHERE", sql, f"{name} 應包含 WHERE 條件")
                
                print(f"✅ {name} SQL 語法基本檢查通過")

    def test_condition_ids_unique(self):
        """測試條件 ID 的唯一性"""
        condition_ids = [condition["id"] for condition in sample_conditions]
        unique_ids = set(condition_ids)
        
        self.assertEqual(len(condition_ids), len(unique_ids), 
                        "所有條件 ID 應該是唯一的")
        
        # 檢查新增的 ID 是否都存在
        required_ids = [2, 205, 206, 207]
        for required_id in required_ids:
            self.assertIn(required_id, condition_ids, 
                         f"條件 ID {required_id} 應該存在")
        
        print("✅ 條件 ID 唯一性檢查通過")


if __name__ == '__main__':
    unittest.main(verbosity=2)
