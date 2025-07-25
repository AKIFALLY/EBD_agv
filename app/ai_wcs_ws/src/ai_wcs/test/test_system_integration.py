"""
AI WCS 系統整合測試 - 統一組件版本
驗證統一決策引擎各個組件的整合功能
"""

import unittest
from unittest.mock import Mock, AsyncMock, patch
import asyncio
import sys
import os

# 添加模組路徑
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from ai_wcs.rack_analyzer import RackAnalyzer, RackStatus, CarrierInfo, RackSide
from ai_wcs.unified_decision_engine import UnifiedWCSDecisionEngine, TaskDecision, WorkIDCategory, BusinessFlowPriority
from ai_wcs.unified_task_manager import UnifiedTaskManager, TaskCreationResult
from ai_wcs.enhanced_database_client import EnhancedDatabaseClient


class TestUnifiedSystemIntegration(unittest.TestCase):
    """測試統一系統整合"""
    
    def setUp(self):
        """設置測試環境"""
        self.mock_logger = Mock()
    
    def test_task_decision_integration(self):
        """測試TaskDecision與各組件的整合"""
        # 創建測試決策
        decision = TaskDecision(
            work_id=WorkIDCategory.MAIN_RACK_OPERATIONS.value,
            task_type="agv_rotation",
            priority=BusinessFlowPriority.AGV_ROTATION,
            source_location=10,
            target_location=11,
            reason="整合測試"
        )
        
        # 驗證決策屬性
        self.assertEqual(decision.work_id, "220001")
        self.assertEqual(decision.priority, BusinessFlowPriority.AGV_ROTATION)
        self.assertIsNotNone(decision.created_at)
    
    def test_rack_analyzer_functionality(self):
        """測試Rack分析器功能"""
        analyzer = RackAnalyzer(self.mock_logger)
        
        # 準備測試資料
        rack_data = {
            'id': 1,
            'direction': 90,  # A面朝前
            'location_id': 10,
            'room_id': 1
        }
        
        carriers = [
            CarrierInfo(id=1, rack_id=1, room_id=1, port_id=1, rack_index=1, status_id=1, is_ng=False),
            CarrierInfo(id=2, rack_id=1, room_id=1, port_id=2, rack_index=2, status_id=1, is_ng=False),
        ]
        
        product_data = {'id': 1, 'size': 'S', 'name': 'Test Product'}
        
        # 執行分析
        status = analyzer.analyze_rack_status(rack_data, carriers, product_data)
        
        # 驗證結果
        self.assertIsInstance(status, RackStatus)
        self.assertEqual(status.rack_id, 1)
        self.assertEqual(status.total_carriers, 2)
    
    def test_unified_decision_engine_initialization(self):
        """測試統一決策引擎初始化"""
        # 不實際連接資料庫，只測試初始化邏輯
        with patch('ai_wcs.unified_decision_engine.EnhancedDatabaseClient'):
            engine = UnifiedWCSDecisionEngine(self.mock_logger)
            
            # 驗證初始化
            self.assertIsNotNone(engine.logger)
            self.assertIsNotNone(engine.condition_work_ids)
            self.assertIsNotNone(engine.location_mappings)
    
    def test_unified_task_manager_initialization(self):
        """測試統一任務管理器初始化"""
        # 不實際連接資料庫，只測試初始化邏輯
        with patch('ai_wcs.unified_task_manager.EnhancedDatabaseClient'):
            manager = UnifiedTaskManager(self.mock_logger)
            
            # 驗證初始化
            self.assertIsNotNone(manager.logger)
            self.assertIsNotNone(manager.param_manager)
            self.assertIsNotNone(manager.task_stats)
    
    def test_task_creation_result_workflow(self):
        """測試任務創建結果工作流程"""
        # 模擬成功的任務創建
        success_result = TaskCreationResult(
            success=True,
            task_id=123
        )
        
        self.assertTrue(success_result.success)
        self.assertEqual(success_result.task_id, 123)
        self.assertIsNotNone(success_result.created_at)
        
        # 模擬失敗的任務創建
        failure_result = TaskCreationResult(
            success=False,
            error_message="資料庫連接失敗"
        )
        
        self.assertFalse(failure_result.success)
        self.assertIsNone(failure_result.task_id)
        self.assertEqual(failure_result.error_message, "資料庫連接失敗")


class TestWorkIDParameterIntegration(unittest.TestCase):
    """測試Work ID參數整合"""
    
    def test_work_id_mappings(self):
        """測試Work ID映射"""
        # 測試主要操作Work ID
        self.assertEqual(WorkIDCategory.MAIN_RACK_OPERATIONS.value, "220001")
        
        # 測試工作流程Work ID
        self.assertEqual(WorkIDCategory.WORKFLOW_OPERATIONS.value, "230001")
        
        # 測試OPUI Work IDs
        self.assertEqual(WorkIDCategory.OPUI_CALL_EMPTY.value, "100001")
        self.assertEqual(WorkIDCategory.OPUI_DISPATCH_FULL.value, "100002")
    
    def test_business_flow_priorities(self):
        """測試業務流程優先級"""
        # 驗證七大業務流程的優先級順序
        priorities = [
            BusinessFlowPriority.AGV_ROTATION,        # 100
            BusinessFlowPriority.NG_RECYCLING,       # 90  
            BusinessFlowPriority.MANUAL_TRANSPORT,   # 80
            BusinessFlowPriority.SYSTEM_TO_ROOM,     # 60
        ]
        
        # 驗證優先級遞減
        for i in range(len(priorities) - 1):
            self.assertGreater(priorities[i].value, priorities[i + 1].value)


if __name__ == '__main__':
    unittest.main()