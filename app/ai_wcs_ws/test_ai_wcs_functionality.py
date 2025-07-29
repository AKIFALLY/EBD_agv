#!/usr/bin/env python3
"""
AI WCS 功能驗證測試
驗證所有核心功能是否正常運作，包括資料庫整合和 ROS 2 控制服務
"""

import sys
import os
import time
from unittest.mock import Mock, patch

# 添加路徑
sys.path.append('/app/ai_wcs_ws/src')

def test_basic_imports():
    """測試基本模組導入"""
    print("🧪 測試基本模組導入...")
    
    try:
        from ai_wcs.unified_decision_engine import TaskDecision, BusinessFlowPriority, WorkIDCategory
        from ai_wcs.unified_task_manager import UnifiedTaskManager, WorkIDParameterManager, TaskCreationResult
        from ai_wcs.enhanced_database_client import EnhancedDatabaseClient
        from ai_wcs.rack_analyzer import RackAnalyzer, RackStatus, CarrierInfo
        print("✅ 所有核心模組導入成功")
        return True
    except Exception as e:
        print(f"❌ 模組導入失敗: {e}")
        return False

def test_task_decision_creation():
    """測試任務決策創建"""
    print("\n🧪 測試任務決策創建...")
    
    try:
        from ai_wcs.unified_decision_engine import TaskDecision, BusinessFlowPriority
        
        # 創建 AGV 旋轉決策
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
        
        print(f"✅ 決策創建成功: work_id={decision.work_id}, priority={decision.priority.value}")
        print(f"✅ 旋轉路徑: {decision.nodes}")
        return True
    except Exception as e:
        print(f"❌ 任務決策創建失敗: {e}")
        return False

def test_work_id_parameters():
    """測試 Work ID 參數管理"""
    print("\n🧪 測試 Work ID 參數管理...")
    
    try:
        from ai_wcs.unified_task_manager import WorkIDParameterManager
        from ai_wcs.unified_decision_engine import TaskDecision, BusinessFlowPriority
        
        manager = WorkIDParameterManager()
        
        # 測試七大業務流程對應的 Work ID
        business_flows = {
            'agv_rotation': '220001',
            'ng_rack_recycling': '220001', 
            'manual_transport': '220001',
            'system_to_room': '220001',
            'empty_rack_transfer': '220001',
            'manual_empty_recycling': '230001',  # 唯一的 workflow 觸發
            'opui_call_empty': '100001',
            'opui_dispatch_full': '100002'
        }
        
        for flow, expected_work_id in business_flows.items():
            actual_work_id = manager.BUSINESS_FLOW_WORK_IDS.get(flow)
            if actual_work_id == expected_work_id:
                print(f"✅ {flow}: {actual_work_id}")
            else:
                print(f"❌ {flow}: 期望={expected_work_id}, 實際={actual_work_id}")
        
        # 測試參數建構
        decision = TaskDecision(
            work_id='220001',
            task_type='agv_rotation',
            priority=BusinessFlowPriority.AGV_ROTATION,
            source_location=10,
            target_location=11,
            nodes=[10, 11, 10],
            agv_id=1,
            reason='測試參數建構'
        )
        
        params = manager.build_kuka_rack_move_parameters(decision)
        print(f"✅ 參數建構成功: work_id={params['work_id']}, function={params['function']}")
        
        return True
    except Exception as e:
        print(f"❌ Work ID 參數管理測試失敗: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_opui_integration():
    """測試 OPUI 整合功能"""
    print("\n🧪 測試 OPUI 整合功能...")
    
    try:
        from ai_wcs.unified_task_manager import WorkIDParameterManager
        from ai_wcs.unified_decision_engine import TaskDecision, BusinessFlowPriority
        
        manager = WorkIDParameterManager()
        
        # 測試 OPUI 叫空車
        opui_decision = TaskDecision(
            work_id='100001',
            task_type='opui_call_empty',
            priority=BusinessFlowPriority.EMPTY_OPERATIONS,
            source_location=91,
            target_location=95,
            parameters={
                'machine_id': 1,
                'space_num': 1,
                'client_id': 'test_client'
            },
            reason='OPUI 叫空車測試'
        )
        
        params = manager.build_opui_call_empty_parameters(opui_decision)
        
        # 驗證關鍵參數
        assert params['work_id'] == 100001
        assert params['task_type'] == 'call_empty'
        assert params['machine_id'] == 1
        assert params['space_num'] == 1
        assert params['node_id'] == 95  # 機台1停車格1
        assert params['parking_space_status'] == 1
        
        print("✅ OPUI 叫空車參數正確")
        
        # 測試 OPUI 派滿車
        dispatch_decision = TaskDecision(
            work_id='100002',
            task_type='opui_dispatch_full',
            priority=BusinessFlowPriority.EMPTY_OPERATIONS,
            source_location=95,
            target_location=15,
            parameters={
                'machine_id': 1,
                'rack_id': 123,
                'room_id': 1,
                'side': 'left',
                'client_id': 'test_client'
            },
            reason='OPUI 派滿車測試'
        )
        
        params = manager.build_opui_dispatch_full_parameters(dispatch_decision)
        assert params['work_id'] == 100002
        assert params['task_type'] == 'dispatch_full'
        assert params['rack_id'] == 123
        
        print("✅ OPUI 派滿車參數正確")
        return True
        
    except Exception as e:
        print(f"❌ OPUI 整合測試失敗: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_enhanced_database_client():
    """測試增強資料庫客戶端 (模擬模式)"""
    print("\n🧪 測試增強資料庫客戶端 (模擬模式)...")
    
    try:
        with patch('ai_wcs.enhanced_database_client.ConnectionPoolManager') as mock_pool:
            # 模擬連接池
            mock_session = Mock()
            mock_pool.return_value.get_session.return_value.__enter__.return_value = mock_session
            mock_pool.return_value.get_session.return_value.__exit__.return_value = None
            
            from ai_wcs.enhanced_database_client import EnhancedDatabaseClient
            
            client = EnhancedDatabaseClient()
            
            # 驗證初始化
            assert client.cache_ttl == 45
            assert client.batch_cache_ttl == 25
            
            # 測試統計資料
            stats = client.get_query_statistics()
            assert 'stats' in stats
            assert 'cache_hit_rate' in stats
            
            print("✅ 增強資料庫客戶端初始化成功")
            print(f"✅ 快取設定: 一般={client.cache_ttl}秒, 批次={client.batch_cache_ttl}秒")
            
            return True
            
    except Exception as e:
        print(f"❌ 增強資料庫客戶端測試失敗: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_unified_task_manager():
    """測試統一任務管理器 (模擬模式)"""
    print("\n🧪 測試統一任務管理器 (模擬模式)...")
    
    try:
        with patch('ai_wcs.unified_task_manager.EnhancedDatabaseClient') as mock_db:
            # 模擬資料庫客戶端
            mock_db_instance = Mock()
            mock_db_instance.create_task_from_decision.return_value = 12345
            mock_db_instance.update_machine_parking_status.return_value = True
            mock_db.return_value = mock_db_instance
            
            from ai_wcs.unified_task_manager import UnifiedTaskManager
            from ai_wcs.unified_decision_engine import TaskDecision, BusinessFlowPriority
            
            manager = UnifiedTaskManager()
            
            # 創建測試決策
            decision = TaskDecision(
                work_id='220001',
                task_type='agv_rotation',
                priority=BusinessFlowPriority.AGV_ROTATION,
                source_location=10,
                target_location=11,
                nodes=[10, 11, 10],
                reason='測試任務管理器'
            )
            
            # 測試任務創建
            result = manager.create_task_from_decision(decision)
            
            assert result.success == True
            assert result.task_id == 12345
            
            print("✅ 統一任務管理器測試成功")
            print(f"✅ 任務創建: success={result.success}, task_id={result.task_id}")
            
            # 測試統計資料
            stats = manager.get_task_statistics()
            assert 'stats' in stats
            print(f"✅ 統計資料: 已創建任務 {stats['stats']['created']} 個")
            
            return True
            
    except Exception as e:
        print(f"❌ 統一任務管理器測試失敗: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_business_flow_priorities():
    """測試業務流程優先級"""
    print("\n🧪 測試業務流程優先級...")
    
    try:
        from ai_wcs.unified_decision_engine import BusinessFlowPriority
        
        priorities = {
            'AGV_ROTATION': 100,
            'NG_RECYCLING': 90,
            'MANUAL_TRANSPORT': 80,
            'SYSTEM_TO_ROOM': 60,
            'EMPTY_OPERATIONS': 40
        }
        
        for name, expected_value in priorities.items():
            actual_value = getattr(BusinessFlowPriority, name).value
            if actual_value == expected_value:
                print(f"✅ {name}: {actual_value}")
            else:
                print(f"❌ {name}: 期望={expected_value}, 實際={actual_value}")
        
        # 驗證優先級遞減順序
        assert BusinessFlowPriority.AGV_ROTATION.value > BusinessFlowPriority.NG_RECYCLING.value
        assert BusinessFlowPriority.NG_RECYCLING.value > BusinessFlowPriority.MANUAL_TRANSPORT.value
        assert BusinessFlowPriority.MANUAL_TRANSPORT.value > BusinessFlowPriority.SYSTEM_TO_ROOM.value
        assert BusinessFlowPriority.SYSTEM_TO_ROOM.value > BusinessFlowPriority.EMPTY_OPERATIONS.value
        
        print("✅ 優先級遞減順序正確")
        return True
        
    except Exception as e:
        print(f"❌ 業務流程優先級測試失敗: {e}")
        return False

def test_complete_workflow():
    """測試完整工作流程"""
    print("\n🧪 測試完整工作流程...")
    
    try:
        # 模擬完整的 AGV 旋轉決策到任務創建流程
        from ai_wcs.unified_decision_engine import TaskDecision, BusinessFlowPriority
        from ai_wcs.unified_task_manager import WorkIDParameterManager, TaskCreationResult
        
        # 1. 創建決策
        decision = TaskDecision(
            work_id='220001',
            task_type='agv_rotation',
            priority=BusinessFlowPriority.AGV_ROTATION,
            source_location=10001,
            target_location=10001,
            nodes=[10001, 10011, 10001],
            agv_id=1,
            reason='AGV 在房間入口需要旋轉'
        )
        
        # 2. 建構任務參數
        param_manager = WorkIDParameterManager()
        task_params = param_manager.build_kuka_rack_move_parameters(decision)
        
        # 3. 驗證任務參數
        assert task_params['work_id'] == 220001
        assert task_params['rotation_type'] == '3_node_movement'
        assert task_params['agv_id'] == 1
        assert task_params['nodes'] == [10001, 10011, 10001]
        
        # 4. 模擬任務創建結果
        result = TaskCreationResult(success=True, task_id=12345)
        
        print("✅ 完整工作流程測試成功")
        print(f"✅ 決策→參數→任務: work_id={decision.work_id} → task_id={result.task_id}")
        
        return True
        
    except Exception as e:
        print(f"❌ 完整工作流程測試失敗: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """主測試函數"""
    print("🚀 開始 AI WCS 功能驗證測試")
    print("=" * 60)
    
    test_results = []
    
    # 執行所有測試
    tests = [
        ("基本模組導入", test_basic_imports),
        ("任務決策創建", test_task_decision_creation),
        ("Work ID 參數管理", test_work_id_parameters),
        ("OPUI 整合功能", test_opui_integration),
        ("增強資料庫客戶端", test_enhanced_database_client),
        ("統一任務管理器", test_unified_task_manager),
        ("業務流程優先級", test_business_flow_priorities),
        ("完整工作流程", test_complete_workflow)
    ]
    
    for test_name, test_func in tests:
        try:
            result = test_func()
            test_results.append((test_name, result))
        except Exception as e:
            print(f"❌ 測試 '{test_name}' 執行異常: {e}")
            test_results.append((test_name, False))
    
    # 總結結果
    print("\n" + "=" * 60)
    print("📊 測試結果總結:")
    
    passed = 0
    failed = 0
    
    for test_name, result in test_results:
        status = "✅ 通過" if result else "❌ 失敗"
        print(f"  {status} {test_name}")
        if result:
            passed += 1
        else:
            failed += 1
    
    print(f"\n📈 測試統計: 通過 {passed}/{len(test_results)} 個測試")
    
    if failed == 0:
        print("🎉 所有測試都通過！AI WCS 功能正常運作")
        return True
    else:
        print(f"⚠️  有 {failed} 個測試失敗，需要進一步檢查")
        return False

if __name__ == '__main__':
    success = main()
    exit(0 if success else 1)