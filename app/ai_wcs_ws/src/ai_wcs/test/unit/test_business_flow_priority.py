#!/usr/bin/env python3
"""
BusinessFlowPriority 單元測試
驗證業務流程優先級枚舉的正確性
"""

import unittest
import sys
import os

# 添加路徑
sys.path.append('/app/ai_wcs_ws/src')

from ai_wcs.unified_decision_engine import BusinessFlowPriority


class TestBusinessFlowPriority(unittest.TestCase):
    """業務流程優先級測試"""
    
    def test_priority_values(self):
        """測試優先級數值正確性"""
        expected_priorities = {
            'AGV_ROTATION': 100,
            'NG_RECYCLING': 90,
            'MANUAL_TRANSPORT': 80,
            'SYSTEM_TO_ROOM': 60,
            'EMPTY_RACK_TRANSFER': 40
        }
        
        for name, expected_value in expected_priorities.items():
            with self.subTest(priority=name):
                actual_value = getattr(BusinessFlowPriority, name).value
                self.assertEqual(actual_value, expected_value,
                               f"{name} 優先級應該是 {expected_value}，實際是 {actual_value}")
    
    def test_priority_ordering(self):
        """測試優先級遞減順序"""
        priorities = [
            BusinessFlowPriority.AGV_ROTATION,
            BusinessFlowPriority.NG_RECYCLING,
            BusinessFlowPriority.MANUAL_TRANSPORT,
            BusinessFlowPriority.SYSTEM_TO_ROOM,
            BusinessFlowPriority.EMPTY_RACK_TRANSFER
        ]
        
        for i in range(len(priorities) - 1):
            with self.subTest(current=priorities[i], next=priorities[i + 1]):
                self.assertGreater(priorities[i].value, priorities[i + 1].value,
                                 f"{priorities[i].name} 應該比 {priorities[i + 1].name} 優先級更高")
    
    def test_enum_completeness(self):
        """測試枚舉完整性"""
        # IntEnum 的特殊行為：相同數值的項目會被視為別名，迭代時只返回唯一數值的項目
        unique_count = len(BusinessFlowPriority)  # 5 個唯一數值
        all_members_count = len(BusinessFlowPriority.__members__)  # 9 個定義（包括別名）
        
        self.assertEqual(unique_count, 5, f"應該有 5 個唯一優先級，實際有 {unique_count} 個")
        self.assertEqual(all_members_count, 9, f"應該有 9 個完整定義（包括別名），實際有 {all_members_count} 個")
    
    def test_enum_types(self):
        """測試枚舉類型"""
        for priority in BusinessFlowPriority:
            with self.subTest(priority=priority):
                self.assertIsInstance(priority.value, int,
                                    f"{priority.name} 的值應該是整數")
                self.assertGreater(priority.value, 0,
                                 f"{priority.name} 的值應該大於 0")
                self.assertLessEqual(priority.value, 100,
                                   f"{priority.name} 的值應該小於等於 100")
    
    def test_same_priority_levels(self):
        """測試同等優先級項目（別名機制）"""
        # 優先級 80 的項目（FULL_RACK_TO_MANUAL 是 MANUAL_TRANSPORT 的別名）
        self.assertEqual(BusinessFlowPriority.MANUAL_TRANSPORT.value, 80)
        self.assertEqual(BusinessFlowPriority.FULL_RACK_TO_MANUAL.value, 80)
        # 檢查別名關係
        self.assertIs(BusinessFlowPriority.FULL_RACK_TO_MANUAL, BusinessFlowPriority.MANUAL_TRANSPORT)
        
        # 優先級 40 的項目（其他三個都是 EMPTY_RACK_TRANSFER 的別名）
        priority_40_items = [
            BusinessFlowPriority.EMPTY_RACK_TRANSFER,
            BusinessFlowPriority.MANUAL_EMPTY_RECYCLING,
            BusinessFlowPriority.EMPTY_OPERATIONS,
            BusinessFlowPriority.OPUI_OPERATIONS
        ]
        
        for item in priority_40_items:
            self.assertEqual(item.value, 40)
        
        # 檢查別名關係
        self.assertIs(BusinessFlowPriority.MANUAL_EMPTY_RECYCLING, BusinessFlowPriority.EMPTY_RACK_TRANSFER)
        self.assertIs(BusinessFlowPriority.EMPTY_OPERATIONS, BusinessFlowPriority.EMPTY_RACK_TRANSFER)
        self.assertIs(BusinessFlowPriority.OPUI_OPERATIONS, BusinessFlowPriority.EMPTY_RACK_TRANSFER)
    
    def test_alias_access(self):
        """測試別名存取"""
        # 測試所有定義的名稱都可以存取
        all_names = [
            'AGV_ROTATION',
            'NG_RECYCLING', 
            'MANUAL_TRANSPORT',
            'FULL_RACK_TO_MANUAL',
            'SYSTEM_TO_ROOM',
            'EMPTY_RACK_TRANSFER',
            'MANUAL_EMPTY_RECYCLING',
            'EMPTY_OPERATIONS',
            'OPUI_OPERATIONS'
        ]
        
        for name in all_names:
            with self.subTest(name=name):
                # 檢查可以通過名稱存取
                priority = getattr(BusinessFlowPriority, name)
                self.assertIsInstance(priority, BusinessFlowPriority)
                # 檢查在 __members__ 中
                self.assertIn(name, BusinessFlowPriority.__members__)


if __name__ == '__main__':
    unittest.main()