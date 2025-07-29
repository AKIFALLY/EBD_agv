#!/usr/bin/env python3
"""
WorkIDCategory 單元測試
驗證 Work ID 分類系統的正確性
"""

import unittest
import sys

# 添加路徑
sys.path.append('/app/ai_wcs_ws/src')

from ai_wcs.unified_decision_engine import WorkIDCategory


class TestWorkIDCategory(unittest.TestCase):
    """Work ID 分類系統測試"""
    
    def test_work_id_values(self):
        """測試 Work ID 數值正確性"""
        expected_work_ids = {
            'MAIN_RACK_OPERATIONS': '220001',
            'WORKFLOW_OPERATIONS': '230001',
            'OPUI_CALL_EMPTY': '100001',
            'OPUI_DISPATCH_FULL': '100002',
            'CARGO_INLET': '2000102',
            'CARGO_OUTLET': '2000201'
        }
        
        for name, expected_value in expected_work_ids.items():
            with self.subTest(work_id=name):
                actual_value = getattr(WorkIDCategory, name).value
                self.assertEqual(actual_value, expected_value,
                               f"{name} 應該是 {expected_value}，實際是 {actual_value}")
    
    def test_enum_completeness(self):
        """測試枚舉完整性"""
        expected_count = 6
        actual_count = len(WorkIDCategory)
        self.assertEqual(actual_count, expected_count,
                        f"WorkIDCategory 應該有 {expected_count} 個項目，實際有 {actual_count} 個")
    
    def test_work_id_categories(self):
        """測試 Work ID 分類邏輯"""
        # KUKA 相關 Work ID
        kuka_work_ids = [
            WorkIDCategory.MAIN_RACK_OPERATIONS,
            WorkIDCategory.WORKFLOW_OPERATIONS
        ]
        
        for work_id in kuka_work_ids:
            with self.subTest(work_id=work_id):
                self.assertTrue(work_id.value.startswith('2'),
                              f"KUKA Work ID {work_id.name} 應該以 2 開頭")
        
        # OPUI 相關 Work ID
        opui_work_ids = [
            WorkIDCategory.OPUI_CALL_EMPTY,
            WorkIDCategory.OPUI_DISPATCH_FULL
        ]
        
        for work_id in opui_work_ids:
            with self.subTest(work_id=work_id):
                self.assertTrue(work_id.value.startswith('10'),
                              f"OPUI Work ID {work_id.name} 應該以 10 開頭")
        
        # Cargo AGV 相關 Work ID
        cargo_work_ids = [
            WorkIDCategory.CARGO_INLET,
            WorkIDCategory.CARGO_OUTLET
        ]
        
        for work_id in cargo_work_ids:
            with self.subTest(work_id=work_id):
                self.assertTrue(work_id.value.startswith('200'),
                              f"Cargo Work ID {work_id.name} 應該以 200 開頭")
    
    def test_work_id_uniqueness(self):
        """測試 Work ID 唯一性"""
        work_id_values = [item.value for item in WorkIDCategory]
        unique_values = set(work_id_values)
        
        self.assertEqual(len(work_id_values), len(unique_values),
                        "所有 Work ID 值應該是唯一的")
    
    def test_work_id_format(self):
        """測試 Work ID 格式"""
        for work_id in WorkIDCategory:
            with self.subTest(work_id=work_id):
                # 檢查是否為字串
                self.assertIsInstance(work_id.value, str,
                                    f"{work_id.name} 的值應該是字串")
                
                # 檢查是否為數字字串
                self.assertTrue(work_id.value.isdigit(),
                              f"{work_id.name} 的值應該是數字字串")
                
                # 檢查長度
                self.assertGreaterEqual(len(work_id.value), 6,
                                      f"{work_id.name} 的值長度應該至少為 6")
    
    def test_business_logic_mapping(self):
        """測試業務邏輯映射"""
        # 主要料架操作 (大部分業務流程使用)
        main_operations = WorkIDCategory.MAIN_RACK_OPERATIONS
        self.assertEqual(main_operations.value, '220001')
        
        # 流程觸發操作 (人工回收空料架專用)
        workflow_operations = WorkIDCategory.WORKFLOW_OPERATIONS
        self.assertEqual(workflow_operations.value, '230001')
        
        # OPUI 操作
        opui_call = WorkIDCategory.OPUI_CALL_EMPTY
        opui_dispatch = WorkIDCategory.OPUI_DISPATCH_FULL
        self.assertEqual(opui_call.value, '100001')
        self.assertEqual(opui_dispatch.value, '100002')
        
        # Cargo AGV 操作
        cargo_inlet = WorkIDCategory.CARGO_INLET
        cargo_outlet = WorkIDCategory.CARGO_OUTLET
        self.assertEqual(cargo_inlet.value, '2000102')
        self.assertEqual(cargo_outlet.value, '2000201')
    
    def test_enum_iteration(self):
        """測試枚舉迭代"""
        work_id_names = []
        work_id_values = []
        
        for work_id in WorkIDCategory:
            work_id_names.append(work_id.name)
            work_id_values.append(work_id.value)
        
        # 檢查是否包含所有預期項目
        expected_names = [
            'MAIN_RACK_OPERATIONS',
            'WORKFLOW_OPERATIONS', 
            'OPUI_CALL_EMPTY',
            'OPUI_DISPATCH_FULL',
            'CARGO_INLET',
            'CARGO_OUTLET'
        ]
        
        for name in expected_names:
            self.assertIn(name, work_id_names,
                         f"應該包含 {name}")
        
        # 檢查值的數量
        self.assertEqual(len(work_id_values), 6)


if __name__ == '__main__':
    unittest.main()