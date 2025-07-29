#!/usr/bin/env python3
"""
AI WCS ROS 2 標準測試
符合 ROS 2 colcon test 框架的標準測試檔案
"""

import unittest
import sys
import os

# 添加路徑以便導入 AI WCS 模組
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from ai_wcs.unified_decision_engine import BusinessFlowPriority, TaskDecision, WorkIDCategory
from ai_wcs.unified_task_manager import WorkIDParameterManager


class TestAIWCSROS2(unittest.TestCase):
    """AI WCS ROS 2 標準測試類別"""
    
    def test_business_flow_priority_basic(self):
        """測試業務流程優先級基本功能"""
        # 測試優先級數值
        self.assertEqual(BusinessFlowPriority.AGV_ROTATION.value, 100)
        self.assertEqual(BusinessFlowPriority.NG_RECYCLING.value, 90)
        self.assertEqual(BusinessFlowPriority.MANUAL_TRANSPORT.value, 80)
        self.assertEqual(BusinessFlowPriority.SYSTEM_TO_ROOM.value, 60)
        self.assertEqual(BusinessFlowPriority.EMPTY_RACK_TRANSFER.value, 40)
        
        # 測試優先級順序
        self.assertGreater(
            BusinessFlowPriority.AGV_ROTATION.value,
            BusinessFlowPriority.NG_RECYCLING.value
        )
        self.assertGreater(
            BusinessFlowPriority.NG_RECYCLING.value,
            BusinessFlowPriority.MANUAL_TRANSPORT.value
        )
    
    def test_task_decision_creation(self):
        """測試任務決策創建"""
        decision = TaskDecision(
            work_id='220001',
            task_type='agv_rotation',
            priority=BusinessFlowPriority.AGV_ROTATION,
            source_location=10001,
            target_location=10001,
            nodes=[10001, 10011, 10001],
            agv_id=1,
            reason='ROS 2 測試'
        )
        
        self.assertEqual(decision.work_id, '220001')
        self.assertEqual(decision.task_type, 'agv_rotation')
        self.assertEqual(decision.priority, BusinessFlowPriority.AGV_ROTATION)
        self.assertEqual(decision.agv_id, 1)
        self.assertEqual(len(decision.nodes), 3)
    
    def test_work_id_category(self):
        """測試 Work ID 分類"""
        # 測試主要 Work ID
        self.assertEqual(WorkIDCategory.MAIN_RACK_OPERATIONS.value, '220001')
        self.assertEqual(WorkIDCategory.WORKFLOW_OPERATIONS.value, '230001')
        self.assertEqual(WorkIDCategory.OPUI_CALL_EMPTY.value, '100001')
        self.assertEqual(WorkIDCategory.OPUI_DISPATCH_FULL.value, '100002')
        
        # 測試枚舉完整性
        self.assertEqual(len(WorkIDCategory), 6)
    
    def test_parameter_manager_initialization(self):
        """測試參數管理器初始化"""
        manager = WorkIDParameterManager()
        
        # 測試基本屬性存在
        self.assertIsInstance(manager.WORK_ID_MAPPINGS, dict)
        self.assertIsInstance(manager.BUSINESS_FLOW_WORK_IDS, dict)
        self.assertIsInstance(manager.MACHINE_PARKING_CONFIG, dict)
        
        # 測試關鍵映射存在
        self.assertIn('220001', manager.WORK_ID_MAPPINGS)
        self.assertIn('100001', manager.WORK_ID_MAPPINGS)
        self.assertIn('agv_rotation', manager.BUSINESS_FLOW_WORK_IDS)
    
    def test_kuka_rack_move_parameters(self):
        """測試 KUKA 料架移動參數建構"""
        manager = WorkIDParameterManager()
        
        decision = TaskDecision(
            work_id='220001',
            task_type='agv_rotation',
            priority=BusinessFlowPriority.AGV_ROTATION,
            source_location=10001,
            target_location=10001,
            nodes=[10001, 10011, 10001],
            agv_id=1,
            reason='KUKA 參數測試'
        )
        
        params = manager.build_kuka_rack_move_parameters(decision)
        
        # 驗證關鍵參數
        self.assertEqual(params['work_id'], 220001)
        self.assertEqual(params['missionType'], 'RACK_MOVE')
        self.assertEqual(params['agv_id'], 1)
        self.assertEqual(params['nodes'], [10001, 10011, 10001])
    
    def test_opui_call_empty_parameters(self):
        """測試 OPUI 叫空車參數建構"""
        manager = WorkIDParameterManager()
        
        decision = TaskDecision(
            work_id='100001',
            task_type='opui_call_empty',
            priority=BusinessFlowPriority.EMPTY_RACK_TRANSFER,
            source_location=91,
            target_location=95,
            parameters={
                'machine_id': 1,
                'space_num': 1,
                'client_id': 'ros2_test'
            },
            reason='OPUI 測試'
        )
        
        params = manager.build_opui_call_empty_parameters(decision)
        
        # 驗證關鍵參數
        self.assertEqual(params['work_id'], 100001)
        self.assertEqual(params['task_type'], 'call_empty')
        self.assertEqual(params['machine_id'], 1)
        self.assertEqual(params['space_num'], 1)
        self.assertEqual(params['node_id'], 95)
    
    def test_priority_sorting(self):
        """測試優先級排序功能"""
        decisions = [
            TaskDecision(
                work_id='220001',
                task_type='empty_transfer',
                priority=BusinessFlowPriority.EMPTY_RACK_TRANSFER,
                source_location=10,
                target_location=20,
                reason='低優先級任務'
            ),
            TaskDecision(
                work_id='220001',
                task_type='agv_rotation',
                priority=BusinessFlowPriority.AGV_ROTATION,
                source_location=30,
                target_location=40,
                reason='高優先級任務'
            ),
            TaskDecision(
                work_id='220001',
                task_type='ng_recycling',
                priority=BusinessFlowPriority.NG_RECYCLING,
                source_location=50,
                target_location=60,
                reason='中優先級任務'
            )
        ]
        
        # 按優先級排序（高到低）
        sorted_decisions = sorted(decisions, key=lambda d: d.priority.value, reverse=True)
        
        # 驗證排序結果
        self.assertEqual(sorted_decisions[0].priority, BusinessFlowPriority.AGV_ROTATION)
        self.assertEqual(sorted_decisions[1].priority, BusinessFlowPriority.NG_RECYCLING)
        self.assertEqual(sorted_decisions[2].priority, BusinessFlowPriority.EMPTY_RACK_TRANSFER)
    
    def test_enum_alias_mechanism(self):
        """測試枚舉別名機制"""
        # 測試別名關係
        self.assertIs(
            BusinessFlowPriority.FULL_RACK_TO_MANUAL,
            BusinessFlowPriority.MANUAL_TRANSPORT
        )
        self.assertIs(
            BusinessFlowPriority.OPUI_OPERATIONS,
            BusinessFlowPriority.EMPTY_RACK_TRANSFER
        )
        
        # 測試所有名稱都可存取
        all_names = [
            'AGV_ROTATION', 'NG_RECYCLING', 'MANUAL_TRANSPORT',
            'FULL_RACK_TO_MANUAL', 'SYSTEM_TO_ROOM', 'EMPTY_RACK_TRANSFER',
            'MANUAL_EMPTY_RECYCLING', 'EMPTY_OPERATIONS', 'OPUI_OPERATIONS'
        ]
        
        for name in all_names:
            priority = getattr(BusinessFlowPriority, name)
            self.assertIsInstance(priority, BusinessFlowPriority)
        
        # 測試唯一數值數量 vs 總定義數量
        unique_count = len(BusinessFlowPriority)  # 5 個唯一數值
        all_members_count = len(BusinessFlowPriority.__members__)  # 9 個定義
        
        self.assertEqual(unique_count, 5)
        self.assertEqual(all_members_count, 9)


if __name__ == '__main__':
    unittest.main()