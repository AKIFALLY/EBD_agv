#!/usr/bin/env python3
"""
TaskDecision 單元測試
驗證任務決策資料結構的建立和序列化
"""

import unittest
import sys
import json
from dataclasses import asdict

# 添加路徑
sys.path.append('/app/ai_wcs_ws/src')

from ai_wcs.unified_decision_engine import TaskDecision, BusinessFlowPriority


class TestTaskDecision(unittest.TestCase):
    """任務決策資料結構測試"""
    
    def test_task_decision_creation(self):
        """測試任務決策基本建立"""
        decision = TaskDecision(
            work_id='220001',
            task_type='agv_rotation',
            priority=BusinessFlowPriority.AGV_ROTATION,
            source_location=10001,
            target_location=10001,
            nodes=[10001, 10011, 10001],
            agv_id=1,
            reason='測試 AGV 旋轉決策'
        )
        
        self.assertEqual(decision.work_id, '220001')
        self.assertEqual(decision.task_type, 'agv_rotation')
        self.assertEqual(decision.priority, BusinessFlowPriority.AGV_ROTATION)
        self.assertEqual(decision.source_location, 10001)
        self.assertEqual(decision.target_location, 10001)
        self.assertEqual(decision.nodes, [10001, 10011, 10001])
        self.assertEqual(decision.agv_id, 1)
        self.assertEqual(decision.reason, '測試 AGV 旋轉決策')
    
    def test_task_decision_defaults(self):
        """測試任務決策預設值"""
        decision = TaskDecision(
            work_id='220001',
            task_type='test',
            priority=BusinessFlowPriority.EMPTY_OPERATIONS,
            source_location=10,
            target_location=20
        )
        
        # 檢查預設值
        self.assertIsNone(decision.agv_id)
        self.assertEqual(decision.nodes, [])
        self.assertIsInstance(decision.parameters, dict)
        self.assertEqual(len(decision.parameters), 0)
        self.assertEqual(decision.reason, "")
    
    def test_task_decision_with_parameters(self):
        """測試帶參數的任務決策"""
        parameters = {
            'machine_id': 1,
            'space_num': 1,
            'client_id': 'test_client'
        }
        
        decision = TaskDecision(
            work_id='100001',
            task_type='opui_call_empty',
            priority=BusinessFlowPriority.OPUI_OPERATIONS,
            source_location=91,
            target_location=95,
            parameters=parameters,
            reason='OPUI 叫空車測試'
        )
        
        self.assertEqual(decision.parameters, parameters)
        self.assertEqual(decision.parameters['machine_id'], 1)
        self.assertEqual(decision.parameters['space_num'], 1)
        self.assertEqual(decision.parameters['client_id'], 'test_client')
    
    def test_task_decision_serialization(self):
        """測試任務決策序列化"""
        decision = TaskDecision(
            work_id='220001',
            task_type='agv_rotation',
            priority=BusinessFlowPriority.AGV_ROTATION,
            source_location=10001,
            target_location=10001,
            nodes=[10001, 10011, 10001],
            agv_id=1,
            reason='序列化測試'
        )
        
        # 轉換為字典
        decision_dict = asdict(decision)
        
        # 檢查關鍵欄位
        self.assertIn('work_id', decision_dict)
        self.assertIn('task_type', decision_dict)
        self.assertIn('priority', decision_dict)
        self.assertIn('source_location', decision_dict)
        self.assertIn('target_location', decision_dict)
        self.assertIn('nodes', decision_dict)
        self.assertIn('agv_id', decision_dict)
        self.assertIn('reason', decision_dict)
        
        # 檢查值的正確性
        self.assertEqual(decision_dict['work_id'], '220001')
        self.assertEqual(decision_dict['task_type'], 'agv_rotation')
        self.assertEqual(decision_dict['priority'], BusinessFlowPriority.AGV_ROTATION.value)
        self.assertEqual(decision_dict['nodes'], [10001, 10011, 10001])
    
    def test_different_work_ids(self):
        """測試不同 Work ID 的任務決策"""
        work_id_cases = [
            ('220001', 'kuka-移動貨架'),
            ('230001', 'kuka-流程觸發'),
            ('100001', 'opui-call-empty'),
            ('100002', 'opui-dispatch-full'),
            ('2000102', 'CargoAGV放入口'),
            ('2000201', 'CargoAGV拿出口')
        ]
        
        for work_id, description in work_id_cases:
            with self.subTest(work_id=work_id):
                decision = TaskDecision(
                    work_id=work_id,
                    task_type='test',
                    priority=BusinessFlowPriority.EMPTY_OPERATIONS,
                    source_location=10,
                    target_location=20,
                    reason=f'測試 {description}'
                )
                
                self.assertEqual(decision.work_id, work_id)
                self.assertIn(description, decision.reason)
    
    def test_task_decision_validation(self):
        """測試任務決策驗證"""
        # 測試必要欄位
        with self.assertRaises(TypeError):
            TaskDecision()  # 缺少必要參數
        
        # 測試類型驗證
        decision = TaskDecision(
            work_id='220001',
            task_type='test',
            priority=BusinessFlowPriority.AGV_ROTATION,
            source_location=10,
            target_location=20
        )
        
        # 驗證類型
        self.assertIsInstance(decision.work_id, str)
        self.assertIsInstance(decision.task_type, str)
        self.assertIsInstance(decision.priority, BusinessFlowPriority)
        self.assertIsInstance(decision.source_location, int)
        self.assertIsInstance(decision.target_location, int)
        self.assertIsInstance(decision.nodes, list)
        self.assertIsInstance(decision.parameters, dict)
        self.assertIsInstance(decision.reason, str)


if __name__ == '__main__':
    unittest.main()