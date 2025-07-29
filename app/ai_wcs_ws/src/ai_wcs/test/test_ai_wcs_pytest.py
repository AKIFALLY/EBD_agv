#!/usr/bin/env python3
"""
AI WCS pytest測試
符合 ROS 2 colcon pytest 框架的標準測試檔案
"""

import pytest
import sys
import os

# 添加路徑以便導入 AI WCS 模組
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from ai_wcs.unified_decision_engine import BusinessFlowPriority, TaskDecision, WorkIDCategory
from ai_wcs.unified_task_manager import WorkIDParameterManager


def test_business_flow_priority_values():
    """測試業務流程優先級數值"""
    assert BusinessFlowPriority.AGV_ROTATION.value == 100
    assert BusinessFlowPriority.NG_RECYCLING.value == 90
    assert BusinessFlowPriority.MANUAL_TRANSPORT.value == 80
    assert BusinessFlowPriority.SYSTEM_TO_ROOM.value == 60
    assert BusinessFlowPriority.EMPTY_RACK_TRANSFER.value == 40


def test_business_flow_priority_ordering():
    """測試業務流程優先級順序"""
    priorities = [
        BusinessFlowPriority.AGV_ROTATION,
        BusinessFlowPriority.NG_RECYCLING,
        BusinessFlowPriority.MANUAL_TRANSPORT,
        BusinessFlowPriority.SYSTEM_TO_ROOM,
        BusinessFlowPriority.EMPTY_RACK_TRANSFER
    ]
    
    # 檢查每個優先級都比下一個高
    for i in range(len(priorities) - 1):
        assert priorities[i].value > priorities[i + 1].value


def test_task_decision_creation():
    """測試任務決策創建"""
    decision = TaskDecision(
        work_id='220001',
        task_type='agv_rotation',
        priority=BusinessFlowPriority.AGV_ROTATION,
        source_location=10001,
        target_location=10001,
        nodes=[10001, 10011, 10001],
        agv_id=1,
        reason='pytest測試'
    )
    
    assert decision.work_id == '220001'
    assert decision.task_type == 'agv_rotation'
    assert decision.priority == BusinessFlowPriority.AGV_ROTATION
    assert decision.agv_id == 1
    assert len(decision.nodes) == 3
    assert decision.nodes == [10001, 10011, 10001]


def test_work_id_category_values():
    """測試 Work ID 分類數值"""
    assert WorkIDCategory.MAIN_RACK_OPERATIONS.value == '220001'
    assert WorkIDCategory.WORKFLOW_OPERATIONS.value == '230001'
    assert WorkIDCategory.OPUI_CALL_EMPTY.value == '100001'
    assert WorkIDCategory.OPUI_DISPATCH_FULL.value == '100002'


def test_work_id_category_completeness():
    """測試 Work ID 分類完整性"""
    expected_categories = 6
    actual_categories = len(WorkIDCategory)
    assert actual_categories == expected_categories


def test_parameter_manager_initialization():
    """測試參數管理器初始化"""
    manager = WorkIDParameterManager()
    
    # 驗證基本屬性存在且為字典類型
    assert hasattr(manager, 'WORK_ID_MAPPINGS')
    assert isinstance(manager.WORK_ID_MAPPINGS, dict)
    assert hasattr(manager, 'BUSINESS_FLOW_WORK_IDS')
    assert isinstance(manager.BUSINESS_FLOW_WORK_IDS, dict)
    assert hasattr(manager, 'MACHINE_PARKING_CONFIG')
    assert isinstance(manager.MACHINE_PARKING_CONFIG, dict)


def test_parameter_manager_work_id_mappings():
    """測試參數管理器 Work ID 映射"""
    manager = WorkIDParameterManager()
    
    # 驗證關鍵 Work ID 存在
    assert '220001' in manager.WORK_ID_MAPPINGS
    assert '100001' in manager.WORK_ID_MAPPINGS
    assert '100002' in manager.WORK_ID_MAPPINGS


def test_parameter_manager_business_flows():
    """測試參數管理器業務流程映射"""
    manager = WorkIDParameterManager()
    
    # 驗證業務流程存在
    expected_flows = [
        'agv_rotation',
        'ng_rack_recycling',
        'full_rack_to_manual',
        'manual_area_transport',
        'system_to_room',
        'empty_rack_transfer',
        'manual_empty_recycling',
        'opui_call_empty',
        'opui_dispatch_full'
    ]
    
    for flow in expected_flows:
        assert flow in manager.BUSINESS_FLOW_WORK_IDS


def test_kuka_rack_move_parameters():
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
    assert params['work_id'] == 220001
    assert params['missionType'] == 'RACK_MOVE'
    assert params['agv_id'] == 1
    assert params['nodes'] == [10001, 10011, 10001]


def test_opui_call_empty_parameters():
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
            'client_id': 'pytest_test'
        },
        reason='OPUI 測試'
    )
    
    params = manager.build_opui_call_empty_parameters(decision)
    
    # 驗證關鍵參數
    assert params['work_id'] == 100001
    assert params['task_type'] == 'call_empty'
    assert params['machine_id'] == 1
    assert params['space_num'] == 1
    assert params['node_id'] == 95


def test_priority_sorting():
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
    assert sorted_decisions[0].priority == BusinessFlowPriority.AGV_ROTATION
    assert sorted_decisions[1].priority == BusinessFlowPriority.NG_RECYCLING
    assert sorted_decisions[2].priority == BusinessFlowPriority.EMPTY_RACK_TRANSFER


def test_enum_alias_mechanism():
    """測試枚舉別名機制"""
    # 測試別名關係
    assert BusinessFlowPriority.FULL_RACK_TO_MANUAL is BusinessFlowPriority.MANUAL_TRANSPORT
    assert BusinessFlowPriority.OPUI_OPERATIONS is BusinessFlowPriority.EMPTY_RACK_TRANSFER
    
    # 測試所有名稱都可存取
    all_names = [
        'AGV_ROTATION', 'NG_RECYCLING', 'MANUAL_TRANSPORT',
        'FULL_RACK_TO_MANUAL', 'SYSTEM_TO_ROOM', 'EMPTY_RACK_TRANSFER',
        'MANUAL_EMPTY_RECYCLING', 'EMPTY_OPERATIONS', 'OPUI_OPERATIONS'
    ]
    
    for name in all_names:
        priority = getattr(BusinessFlowPriority, name)
        assert isinstance(priority, BusinessFlowPriority)
    
    # 測試唯一數值數量 vs 總定義數量
    unique_count = len(BusinessFlowPriority)  # 5 個唯一數值
    all_members_count = len(BusinessFlowPriority.__members__)  # 9 個定義
    
    assert unique_count == 5
    assert all_members_count == 9


@pytest.mark.integration
def test_decision_serialization():
    """測試決策序列化功能"""
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
        assert field in decision_dict
    
    # 驗證優先級序列化
    assert decision_dict['priority'] == 100
    
    # 驗證參數序列化
    assert isinstance(decision_dict['parameters'], dict)
    assert decision_dict['parameters']['test_param'] == 'test_value'


if __name__ == '__main__':
    pytest.main([__file__])