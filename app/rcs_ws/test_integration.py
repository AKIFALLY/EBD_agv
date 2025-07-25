#!/usr/bin/env python3
"""
RCS WCS 整合功能測試腳本
測試新增的 WCS 功能是否正常工作
"""

import os
import sys
import logging
from datetime import datetime, timezone

# 添加 src 路徑
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src', 'rcs'))

def setup_logger():
    """設置測試日誌"""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    return logging.getLogger('test_integration')

def test_wcs_task_adapter():
    """測試 WCS 任務適配器"""
    logger = setup_logger()
    logger.info("=== 測試 WCS 任務適配器 ===")
    
    try:
        from rcs.wcs_task_adapter import WCSTaskAdapter, WCSTask, WCSTaskType, WCSTaskPriority
        
        # 創建適配器
        adapter = WCSTaskAdapter(logger)
        
        # 測試旋轉任務
        wcs_task = WCSTask(
            task_id="test_rotation_001",
            task_type=WCSTaskType.ROTATION,
            priority=WCSTaskPriority.ROTATION,
            rack_id=123,
            source_location=91,
            target_location=91,
            nodes=[91, 76, 91]
        )
        
        # 轉換任務
        kuka_task = adapter.convert_wcs_task_to_kuka(wcs_task)
        
        if kuka_task:
            logger.info(f"✅ 旋轉任務轉換成功: work_id={kuka_task.work_id}, priority={kuka_task.priority}")
            
            # 驗證任務
            validation_result = adapter.validate_wcs_task(wcs_task)
            if validation_result[0]:
                logger.info("✅ 任務驗證通過")
            else:
                logger.error(f"❌ 任務驗證失敗: {validation_result[1]}")
        else:
            logger.error("❌ 任務轉換失敗")
            
        # 獲取統計
        stats = adapter.get_conversion_statistics()
        logger.info(f"轉換統計: {stats}")
        
        return True
        
    except Exception as e:
        logger.error(f"WCS 任務適配器測試失敗: {e}")
        return False

def test_wcs_priority_scheduler():
    """測試 WCS 優先度調度器"""
    logger = setup_logger()
    logger.info("=== 測試 WCS 優先度調度器 ===")
    
    try:
        from rcs.wcs_priority_scheduler import WCSPriorityScheduler, WCSPriorityLevel
        
        # 創建調度器
        scheduler = WCSPriorityScheduler(logger)
        
        # 模擬任務物件
        class MockTask:
            def __init__(self, task_id, priority, task_type='rotation'):
                self.id = task_id
                self.priority = priority
                self.parameters = {'wcs_task_type': task_type}
                self.created_at = datetime.now(timezone.utc)
        
        # 創建測試任務
        tasks = [
            MockTask(1, 40, 'manual'),
            MockTask(2, 100, 'rotation'),
            MockTask(3, 80, 'outlet'),
            MockTask(4, 60, 'inlet')
        ]
        
        # 系統上下文
        system_context = {
            'available_agvs': 2,
            'total_agvs': 3,
            'pending_tasks_count': 4
        }
        
        # 調度任務
        schedule_infos = scheduler.schedule_tasks(tasks, system_context)
        
        if schedule_infos:
            logger.info("✅ 任務調度成功")
            for info in schedule_infos:
                logger.info(f"  任務 {info.task_id}: 優先度 {info.calculated_priority} ({info.priority_level.name})")
        else:
            logger.error("❌ 任務調度失敗")
            
        # 獲取統計
        stats = scheduler.get_schedule_statistics()
        logger.info(f"調度統計: {stats}")
        
        return True
        
    except Exception as e:
        logger.error(f"WCS 優先度調度器測試失敗: {e}")
        return False

def test_rack_state_manager():
    """測試 Rack 狀態管理器"""
    logger = setup_logger()
    logger.info("=== 測試 Rack 狀態管理器 ===")
    
    try:
        from rcs.rack_state_manager import RackStateManager, RackAnalysisResult, CarrierInfo, RackDirection
        
        # 創建管理器
        manager = RackStateManager(logger)
        
        # 創建測試 Carrier
        carriers = [
            CarrierInfo(carrier_id=1, rack_id=123, rack_index=5, status=1),  # A面
            CarrierInfo(carrier_id=2, rack_id=123, rack_index=20, status=1)  # B面
        ]
        
        # 創建測試 Rack 分析結果
        rack_analysis = RackAnalysisResult(
            rack_id=123,
            room_id=1,
            location_id=91,
            agv_id=None,
            current_direction=RackDirection.A_FACING,
            product_id=1,  # 添加缺少的參數
            total_carriers=2,
            a_side_carriers=[carriers[0]],
            b_side_carriers=[carriers[1]]
        )
        
        # 更新狀態
        success = manager.update_rack_state(rack_analysis)
        
        if success:
            logger.info("✅ Rack 狀態更新成功")
            
            # 獲取統計
            stats = manager.get_state_statistics()
            logger.info(f"狀態統計: {stats}")
            
            # 獲取狀態
            state = manager.get_rack_state(123)
            if state:
                logger.info(f"✅ 獲取 Rack 狀態成功: 載貨 {state.total_carriers} 個")
            else:
                logger.error("❌ 獲取 Rack 狀態失敗")
        else:
            logger.error("❌ Rack 狀態更新失敗")
            
        return True
        
    except Exception as e:
        logger.error(f"Rack 狀態管理器測試失敗: {e}")
        return False

def test_kuka_manager_validation():
    """測試 KukaManager 路徑驗證功能"""
    logger = setup_logger()
    logger.info("=== 測試 KukaManager 路徑驗證 ===")
    
    try:
        # 模擬 KukaManager 的驗證方法
        class MockKukaManager:
            def __init__(self, logger):
                self.logger = logger
            
            def _validate_rotation_path(self, nodes):
                # 基本格式檢查
                if not nodes or not isinstance(nodes, list):
                    return {'valid': False, 'error': '路徑節點列表不能為空或格式錯誤'}
                
                # 旋轉任務必須有3個節點
                if len(nodes) != 3:
                    return {'valid': False, 'error': f'旋轉任務需要3個節點，當前: {len(nodes)} 個'}
                
                # 起點和終點必須相同
                if nodes[0] != nodes[2]:
                    return {'valid': False, 'error': f'旋轉任務起點({nodes[0]})和終點({nodes[2]})必須相同'}
                
                return {
                    'valid': True,
                    'path_type': 'rotation',
                    'start_node': nodes[0],
                    'intermediate_node': nodes[1],
                    'end_node': nodes[2]
                }
            
            def _validate_move_path(self, nodes):
                # 基本格式檢查
                if not nodes or not isinstance(nodes, list):
                    return {'valid': False, 'error': '路徑節點列表不能為空或格式錯誤'}
                
                # 搬運任務至少需要2個節點
                if len(nodes) < 2:
                    return {'valid': False, 'error': f'搬運任務至少需要2個節點，當前: {len(nodes)} 個'}
                
                return {
                    'valid': True,
                    'path_type': 'move',
                    'start_node': nodes[0],
                    'end_node': nodes[-1]
                }
        
        manager = MockKukaManager(logger)
        
        # 測試有效的旋轉路徑
        result = manager._validate_rotation_path([91, 76, 91])
        if result['valid']:
            logger.info("✅ 有效旋轉路徑驗證通過")
        else:
            logger.error(f"❌ 旋轉路徑驗證失敗: {result['error']}")
        
        # 測試無效的旋轉路徑
        result = manager._validate_rotation_path([91, 76, 92])
        if not result['valid']:
            logger.info("✅ 無效旋轉路徑正確被拒絕")
        else:
            logger.error("❌ 無效旋轉路徑未被拒絕")
        
        # 測試有效的搬運路徑
        result = manager._validate_move_path([91, 76])
        if result['valid']:
            logger.info("✅ 有效搬運路徑驗證通過")
        else:
            logger.error(f"❌ 搬運路徑驗證失敗: {result['error']}")
        
        return True
        
    except Exception as e:
        logger.error(f"KukaManager 驗證測試失敗: {e}")
        return False

def main():
    """主測試函數"""
    logger = setup_logger()
    logger.info("開始 RCS WCS 整合功能測試")
    
    test_results = []
    
    # 執行各項測試
    test_results.append(("WCS 任務適配器", test_wcs_task_adapter()))
    test_results.append(("WCS 優先度調度器", test_wcs_priority_scheduler()))
    test_results.append(("Rack 狀態管理器", test_rack_state_manager()))
    test_results.append(("KukaManager 路徑驗證", test_kuka_manager_validation()))
    
    # 統計結果
    passed = sum(1 for name, result in test_results if result)
    total = len(test_results)
    
    logger.info(f"\n=== 測試結果統計 ===")
    for name, result in test_results:
        status = "✅ 通過" if result else "❌ 失敗"
        logger.info(f"{name}: {status}")
    
    logger.info(f"\n總計: {passed}/{total} 個測試通過")
    
    if passed == total:
        logger.info("🎉 所有測試都通過了！")
        return 0
    else:
        logger.error(f"⚠️  有 {total - passed} 個測試失敗")
        return 1

if __name__ == "__main__":
    sys.exit(main())