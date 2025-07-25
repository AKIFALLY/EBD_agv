"""
AI WCS 簡化功能測試 - 統一組件版本
測試統一決策引擎的核心邏輯，不依賴ROS2節點
"""

import unittest
import sys
import os
from datetime import datetime, timezone

# 添加模組路徑
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from ai_wcs.rack_analyzer import RackStatus, CarrierInfo, RackSide
from ai_wcs.unified_decision_engine import TaskDecision, WorkIDCategory, BusinessFlowPriority
from ai_wcs.unified_task_manager import TaskCreationResult


class TestRackStatusLogic(unittest.TestCase):
    """測試Rack狀態邏輯（不需要ROS2節點）"""
    
    def test_rack_status_creation(self):
        """測試RackStatus物件創建"""
        status = RackStatus(
            rack_id=1,
            total_carriers=16,
            max_capacity=32,
            a_side_count=16,
            b_side_count=0,
            current_side=RackSide.A,
            has_ng=False,
            is_empty=False,
            is_full=False,
            is_half_full=True,
            needs_rotation=False,
            product_size='S',
            location_id=10,
            room_id=1
        )
        
        self.assertEqual(status.rack_id, 1)
        self.assertEqual(status.total_carriers, 16)
        self.assertEqual(status.a_side_count, 16)
        self.assertEqual(status.b_side_count, 0)
        self.assertTrue(status.is_half_full)
        self.assertFalse(status.is_full)
    
    def test_carrier_info_creation(self):
        """測試CarrierInfo物件創建"""
        carrier = CarrierInfo(
            id=1,
            rack_id=1,
            room_id=1,
            port_id=1,
            rack_index=1,
            status_id=1,
            is_ng=False
        )
        
        self.assertEqual(carrier.id, 1)
        self.assertEqual(carrier.rack_index, 1)
        self.assertEqual(carrier.rack_id, 1)
        self.assertFalse(carrier.is_ng)


class TestUnifiedDecisionEngine(unittest.TestCase):
    """測試統一決策引擎邏輯"""
    
    def test_task_decision_creation(self):
        """測試TaskDecision物件創建"""
        decision = TaskDecision(
            work_id="220001",
            task_type="agv_rotation",
            priority=BusinessFlowPriority.AGV_ROTATION,
            source_location=10,
            target_location=11,
            nodes=[10, 11, 10],
            reason="測試旋轉決策"
        )
        
        self.assertEqual(decision.work_id, "220001")
        self.assertEqual(decision.task_type, "agv_rotation")
        self.assertEqual(decision.priority, BusinessFlowPriority.AGV_ROTATION)
        self.assertEqual(decision.source_location, 10)
        self.assertEqual(decision.target_location, 11)
        self.assertEqual(decision.nodes, [10, 11, 10])
        self.assertIsInstance(decision.created_at, datetime)
    
    def test_business_flow_priority_values(self):
        """測試業務流程優先級數值"""
        self.assertEqual(BusinessFlowPriority.AGV_ROTATION.value, 100)
        self.assertEqual(BusinessFlowPriority.NG_RECYCLING.value, 90)
        self.assertEqual(BusinessFlowPriority.MANUAL_TRANSPORT.value, 80)
        self.assertEqual(BusinessFlowPriority.SYSTEM_TO_ROOM.value, 60)
    
    def test_work_id_category_values(self):
        """測試Work ID分類值"""
        self.assertEqual(WorkIDCategory.MAIN_RACK_OPERATIONS.value, "220001")
        self.assertEqual(WorkIDCategory.WORKFLOW_OPERATIONS.value, "230001")
        self.assertEqual(WorkIDCategory.OPUI_CALL_EMPTY.value, "100001")
        self.assertEqual(WorkIDCategory.OPUI_DISPATCH_FULL.value, "100002")


class TestUnifiedTaskManager(unittest.TestCase):
    """測試統一任務管理器邏輯"""
    
    def test_task_creation_result(self):
        """測試任務創建結果物件"""
        result = TaskCreationResult(
            success=True,
            task_id=123,
            error_message="",
            created_at=datetime.now(timezone.utc)
        )
        
        self.assertTrue(result.success)
        self.assertEqual(result.task_id, 123)
        self.assertEqual(result.error_message, "")
        self.assertIsInstance(result.created_at, datetime)
    
    def test_task_creation_failure(self):
        """測試任務創建失敗結果"""
        result = TaskCreationResult(
            success=False,
            task_id=None,
            error_message="資料庫連接失敗"
        )
        
        self.assertFalse(result.success)
        self.assertIsNone(result.task_id)
        self.assertEqual(result.error_message, "資料庫連接失敗")


if __name__ == '__main__':
    unittest.main()