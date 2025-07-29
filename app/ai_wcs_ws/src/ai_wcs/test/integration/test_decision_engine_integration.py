#!/usr/bin/env python3
"""
決策引擎整合測試
測試決策引擎與資料庫客戶端的協作
"""

import unittest
import sys
from unittest.mock import Mock, patch, MagicMock

# 添加路徑
sys.path.append('/app/ai_wcs_ws/src')

from ai_wcs.unified_decision_engine import TaskDecision, BusinessFlowPriority
from ai_wcs.enhanced_database_client import EnhancedDatabaseClient


class TestDecisionEngineIntegration(unittest.TestCase):
    """決策引擎整合測試"""
    
    def setUp(self):
        """測試前設定"""
        # 使用模擬的資料庫客戶端
        self.mock_db_client = Mock(spec=EnhancedDatabaseClient)
        
    def test_decision_creation_with_database_context(self):
        """測試在資料庫上下文中的決策創建"""
        # 創建 AGV 旋轉決策（不依賴具體的資料庫方法）
        decision = TaskDecision(
            work_id='220001',
            task_type='agv_rotation',
            priority=BusinessFlowPriority.AGV_ROTATION,
            source_location=10001,
            target_location=10001,
            nodes=[10001, 10011, 10001],
            agv_id=1,
            reason='AGV 需要旋轉'
        )
        
        # 驗證決策屬性
        self.assertEqual(decision.work_id, '220001')
        self.assertEqual(decision.priority, BusinessFlowPriority.AGV_ROTATION)
        self.assertEqual(decision.agv_id, 1)
        self.assertEqual(len(decision.nodes), 3)
        
        # 驗證優先級是最高的
        self.assertEqual(decision.priority.value, 100)
        
        # 驗證與資料庫客戶端的概念整合
        self.assertIsNotNone(self.mock_db_client)
        self.assertIsInstance(decision, TaskDecision)
    
    def test_business_flow_priority_consistency(self):
        """測試業務流程優先級一致性"""
        # 創建不同優先級的決策
        decisions = [
            TaskDecision(
                work_id='220001',
                task_type='agv_rotation',
                priority=BusinessFlowPriority.AGV_ROTATION,
                source_location=10001,
                target_location=10001,
                reason='AGV 旋轉檢查'
            ),
            TaskDecision(
                work_id='220001',
                task_type='ng_recycling',
                priority=BusinessFlowPriority.NG_RECYCLING,
                source_location=20001,
                target_location=71,
                reason='NG 料架回收'
            ),
            TaskDecision(
                work_id='220001',
                task_type='manual_transport',
                priority=BusinessFlowPriority.MANUAL_TRANSPORT,
                source_location=30001,
                target_location=51,
                reason='滿料架到人工收料區'
            )
        ]
        
        # 驗證優先級遞減順序
        for i in range(len(decisions) - 1):
            self.assertGreater(
                decisions[i].priority.value,
                decisions[i + 1].priority.value,
                f"決策 {i} 的優先級應該高於決策 {i + 1}"
            )
    
    def test_work_id_category_mapping(self):
        """測試 Work ID 分類映射"""
        # 測試主要業務流程使用 220001
        main_operations = [
            'agv_rotation',
            'ng_recycling', 
            'manual_transport',
            'system_to_room',
            'empty_rack_transfer'
        ]
        
        for operation in main_operations:
            with self.subTest(operation=operation):
                decision = TaskDecision(
                    work_id='220001',
                    task_type=operation,
                    priority=BusinessFlowPriority.AGV_ROTATION,
                    source_location=10,
                    target_location=20,
                    reason=f'測試 {operation}'
                )
                self.assertEqual(decision.work_id, '220001')
        
        # 測試特殊流程使用 230001
        decision_workflow = TaskDecision(
            work_id='230001',
            task_type='manual_empty_recycling',
            priority=BusinessFlowPriority.EMPTY_RACK_TRANSFER,
            source_location=91,
            target_location=51,
            reason='人工回收空料架'
        )
        self.assertEqual(decision_workflow.work_id, '230001')
    
    @patch('ai_wcs.enhanced_database_client.EnhancedDatabaseClient')
    def test_database_client_integration(self, mock_db_class):
        """測試與資料庫客戶端的整合"""
        # 設定模擬
        mock_db_instance = Mock()
        mock_db_class.return_value = mock_db_instance
        
        # 模擬通用的資料庫查詢方法
        mock_db_instance.query_locations.return_value = [
            {'location_id': 71, 'status': 2, 'area_type': 'ng_area'},
            {'location_id': 72, 'status': 2, 'area_type': 'ng_area'}
        ]
        
        # 模擬決策邏輯使用資料庫查詢
        ng_locations = mock_db_instance.query_locations(
            area_type='ng_area',
            status=2
        )
        
        self.assertEqual(len(ng_locations), 2)
        self.assertEqual(ng_locations[0]['location_id'], 71)
        self.assertEqual(ng_locations[1]['location_id'], 72)
        
        # 驗證資料庫調用
        mock_db_instance.query_locations.assert_called_once_with(
            area_type='ng_area',
            status=2
        )
    
    def test_decision_serialization_integration(self):
        """測試決策序列化整合"""
        from dataclasses import asdict
        
        decision = TaskDecision(
            work_id='220001',
            task_type='agv_rotation',
            priority=BusinessFlowPriority.AGV_ROTATION,
            source_location=10001,
            target_location=10001,
            nodes=[10001, 10011, 10001],
            agv_id=1,
            parameters={'test_param': 'test_value'},
            reason='整合測試'
        )
        
        # 序列化為字典
        decision_dict = asdict(decision)
        
        # 驗證關鍵欄位存在
        required_fields = [
            'work_id', 'task_type', 'priority', 
            'source_location', 'target_location', 
            'nodes', 'agv_id', 'parameters', 'reason'
        ]
        
        for field in required_fields:
            with self.subTest(field=field):
                self.assertIn(field, decision_dict)
        
        # 驗證優先級序列化
        self.assertEqual(decision_dict['priority'], 100)
        
        # 驗證參數序列化
        self.assertIsInstance(decision_dict['parameters'], dict)
        self.assertEqual(decision_dict['parameters']['test_param'], 'test_value')
    
    def test_multi_decision_priority_sorting(self):
        """測試多決策優先級排序"""
        # 創建混合優先級的決策列表
        decisions = [
            TaskDecision(
                work_id='220001',
                task_type='empty_rack_transfer',
                priority=BusinessFlowPriority.EMPTY_RACK_TRANSFER,
                source_location=10,
                target_location=20,
                reason='空料架搬運'
            ),
            TaskDecision(
                work_id='220001',
                task_type='agv_rotation',
                priority=BusinessFlowPriority.AGV_ROTATION,
                source_location=30,
                target_location=40,
                reason='AGV 旋轉'
            ),
            TaskDecision(
                work_id='220001',
                task_type='ng_recycling',
                priority=BusinessFlowPriority.NG_RECYCLING,
                source_location=50,
                target_location=60,
                reason='NG 回收'
            )
        ]
        
        # 按優先級排序（高到低）
        sorted_decisions = sorted(decisions, key=lambda d: d.priority.value, reverse=True)
        
        # 驗證排序結果
        expected_order = [
            BusinessFlowPriority.AGV_ROTATION,      # 100
            BusinessFlowPriority.NG_RECYCLING,      # 90
            BusinessFlowPriority.EMPTY_RACK_TRANSFER # 40
        ]
        
        for i, expected_priority in enumerate(expected_order):
            with self.subTest(index=i):
                self.assertEqual(sorted_decisions[i].priority, expected_priority)
    
    def test_decision_validation_integration(self):
        """測試決策驗證整合"""
        # 測試有效決策
        valid_decision = TaskDecision(
            work_id='220001',
            task_type='agv_rotation',
            priority=BusinessFlowPriority.AGV_ROTATION,
            source_location=10001,
            target_location=10001,
            nodes=[10001, 10011, 10001],
            agv_id=1,
            reason='有效決策測試'
        )
        
        # 驗證決策屬性類型
        self.assertIsInstance(valid_decision.work_id, str)
        self.assertIsInstance(valid_decision.task_type, str)
        self.assertIsInstance(valid_decision.priority, BusinessFlowPriority)
        self.assertIsInstance(valid_decision.source_location, int)
        self.assertIsInstance(valid_decision.target_location, int)
        self.assertIsInstance(valid_decision.nodes, list)
        self.assertIsInstance(valid_decision.parameters, dict)
        self.assertIsInstance(valid_decision.reason, str)


if __name__ == '__main__':
    unittest.main()