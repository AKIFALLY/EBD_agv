#!/usr/bin/env python3
"""
RCS WCS 整合全面測試
包含模組依賴性測試和邊界條件測試
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
    return logging.getLogger('test_comprehensive')

def test_full_integration_workflow():
    """測試完整的整合工作流程"""
    logger = setup_logger()
    logger.info("=== 測試完整整合工作流程 ===")
    
    try:
        # 導入所有必要模組
        from rcs.wcs_task_adapter import WCSTaskAdapter, WCSTask, WCSTaskType, WCSTaskPriority
        from rcs.wcs_priority_scheduler import WCSPriorityScheduler, WCSPriorityLevel
        from rcs.rack_state_manager import RackStateManager, RackAnalysisResult, CarrierInfo, RackDirection
        
        # 創建組件
        adapter = WCSTaskAdapter(logger)
        scheduler = WCSPriorityScheduler(logger)
        rack_manager = RackStateManager(logger)
        
        # 步驟 1: 創建 WCS 任務
        wcs_tasks = [
            WCSTask(
                task_id="wcs_001",
                task_type=WCSTaskType.ROTATION,
                priority=WCSTaskPriority.ROTATION,
                rack_id=123,
                source_location=91,
                target_location=91,
                nodes=[91, 76, 91]
            ),
            WCSTask(
                task_id="wcs_002", 
                task_type=WCSTaskType.RACK_MOVE,
                priority=WCSTaskPriority.OUTLET,
                rack_id=124,
                source_location=91,
                target_location=76,
                nodes=[91, 76]
            )
        ]
        
        # 步驟 2: 轉換為 KUKA 任務
        kuka_tasks = []
        for wcs_task in wcs_tasks:
            kuka_task = adapter.convert_wcs_task_to_kuka(wcs_task)
            if kuka_task:
                kuka_tasks.append(kuka_task)
        
        logger.info(f"✅ 成功轉換 {len(kuka_tasks)} 個 KUKA 任務")
        
        # 步驟 3: 模擬任務物件進行調度
        class MockTask:
            def __init__(self, kuka_task):
                self.id = int(kuka_task.parameters.get('wcs_task_id', '0').split('_')[-1])
                self.priority = kuka_task.priority
                self.parameters = kuka_task.parameters
                self.created_at = datetime.now(timezone.utc)
        
        mock_tasks = [MockTask(kt) for kt in kuka_tasks]
        
        # 步驟 4: 執行優先度調度
        system_context = {
            'available_agvs': 2,
            'total_agvs': 3,
            'pending_tasks_count': len(mock_tasks)
        }
        
        schedule_infos = scheduler.schedule_tasks(mock_tasks, system_context)
        logger.info(f"✅ 成功調度 {len(schedule_infos)} 個任務")
        
        # 步驟 5: 創建 Rack 狀態
        carriers = [
            CarrierInfo(carrier_id=1, rack_id=123, rack_index=5, status=1),
            CarrierInfo(carrier_id=2, rack_id=124, rack_index=15, status=1)
        ]
        
        for rack_id in [123, 124]:
            rack_analysis = RackAnalysisResult(
                rack_id=rack_id,
                room_id=1,
                location_id=91,
                agv_id=None,
                current_direction=RackDirection.A_FACING,
                product_id=1,
                total_carriers=1,
                a_side_carriers=[c for c in carriers if c.rack_id == rack_id],
                b_side_carriers=[]
            )
            
            success = rack_manager.update_rack_state(rack_analysis)
            if not success:
                logger.error(f"❌ Rack {rack_id} 狀態更新失敗")
                return False
        
        logger.info("✅ 所有 Rack 狀態更新成功")
        
        # 步驟 6: 獲取系統統計
        adapter_stats = adapter.get_conversion_statistics()
        scheduler_stats = scheduler.get_schedule_statistics()
        rack_stats = rack_manager.get_state_statistics()
        
        logger.info("=== 系統統計 ===")
        logger.info(f"任務轉換: {adapter_stats['success_rate']}% 成功率")
        logger.info(f"任務調度: {scheduler_stats['total_scheduled']} 個任務已調度")
        logger.info(f"Rack 管理: {rack_stats['total_racks_analyzed']} 個 Rack 已分析")
        
        return True
        
    except Exception as e:
        logger.error(f"完整整合工作流程測試失敗: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_error_handling():
    """測試錯誤處理機制"""
    logger = setup_logger()
    logger.info("=== 測試錯誤處理機制 ===")
    
    try:
        from rcs.wcs_task_adapter import WCSTaskAdapter, WCSTask, WCSTaskType, WCSTaskPriority
        
        adapter = WCSTaskAdapter(logger)
        
        # 測試無效任務
        invalid_task = WCSTask(
            task_id="",  # 空 ID
            task_type=WCSTaskType.ROTATION,
            priority=WCSTaskPriority.ROTATION,
            rack_id=0,  # 無效 rack_id
            source_location=91,
            target_location=91,
            nodes=[]  # 空節點列表
        )
        
        # 驗證應該失敗
        validation_result = adapter.validate_wcs_task(invalid_task)
        if not validation_result[0]:
            logger.info(f"✅ 正確識別無效任務: {validation_result[1]}")
        else:
            logger.error("❌ 未能識別無效任務")
            return False
        
        # 測試轉換應該失敗
        result = adapter.convert_wcs_task_to_kuka(invalid_task)
        if result is None:
            logger.info("✅ 正確拒絕無效任務轉換")
        else:
            logger.error("❌ 錯誤地轉換了無效任務")
            return False
        
        return True
        
    except Exception as e:
        logger.error(f"錯誤處理測試失敗: {e}")
        return False

def test_boundary_conditions():
    """測試邊界條件"""
    logger = setup_logger()
    logger.info("=== 測試邊界條件 ===")
    
    try:
        from rcs.wcs_priority_scheduler import WCSPriorityScheduler
        
        scheduler = WCSPriorityScheduler(logger)
        
        # 測試空任務列表
        result = scheduler.schedule_tasks([], {})
        if len(result) == 0:
            logger.info("✅ 正確處理空任務列表")
        else:
            logger.error("❌ 空任務列表處理錯誤")
            return False
        
        # 測試無效系統上下文
        class MockTask:
            def __init__(self, task_id, priority):
                self.id = task_id
                self.priority = priority
                self.parameters = {}
                self.created_at = datetime.now(timezone.utc)
        
        tasks = [MockTask(1, 100)]
        result = scheduler.schedule_tasks(tasks, None)  # None 上下文
        
        if len(result) > 0:
            logger.info("✅ 正確處理無效系統上下文")
        else:
            logger.error("❌ 無效系統上下文處理錯誤")
            return False
        
        return True
        
    except Exception as e:
        logger.error(f"邊界條件測試失敗: {e}")
        return False

def test_performance():
    """測試性能"""
    logger = setup_logger()
    logger.info("=== 測試性能 ===")
    
    try:
        from rcs.wcs_task_adapter import WCSTaskAdapter, WCSTask, WCSTaskType, WCSTaskPriority
        import time
        
        adapter = WCSTaskAdapter(logger)
        
        # 創建大量任務進行壓力測試
        start_time = time.time()
        
        for i in range(100):
            wcs_task = WCSTask(
                task_id=f"perf_test_{i}",
                task_type=WCSTaskType.RACK_MOVE,
                priority=WCSTaskPriority.OUTLET,
                rack_id=i + 1000,
                source_location=91,
                target_location=76,
                nodes=[91, 76]
            )
            
            kuka_task = adapter.convert_wcs_task_to_kuka(wcs_task)
            if not kuka_task:
                logger.error(f"❌ 任務 {i} 轉換失敗")
                return False
        
        end_time = time.time()
        duration = end_time - start_time
        
        logger.info(f"✅ 100 個任務轉換完成，耗時: {duration:.3f} 秒")
        logger.info(f"平均每個任務: {duration/100*1000:.2f} ms")
        
        # 檢查統計
        stats = adapter.get_conversion_statistics()
        if stats['total_converted'] == 100 and stats['success_rate'] == 100.0:
            logger.info("✅ 性能測試統計正確")
        else:
            logger.error(f"❌ 性能測試統計錯誤: {stats}")
            return False
        
        return True
        
    except Exception as e:
        logger.error(f"性能測試失敗: {e}")
        return False

def main():
    """主測試函數"""
    logger = setup_logger()
    logger.info("開始 RCS WCS 整合全面測試")
    
    test_results = []
    
    # 執行各項測試
    test_results.append(("完整整合工作流程", test_full_integration_workflow()))
    test_results.append(("錯誤處理機制", test_error_handling()))
    test_results.append(("邊界條件", test_boundary_conditions()))
    test_results.append(("性能測試", test_performance()))
    
    # 統計結果
    passed = sum(1 for name, result in test_results if result)
    total = len(test_results)
    
    logger.info(f"\n=== 全面測試結果統計 ===")
    for name, result in test_results:
        status = "✅ 通過" if result else "❌ 失敗"
        logger.info(f"{name}: {status}")
    
    logger.info(f"\n總計: {passed}/{total} 個測試通過")
    
    if passed == total:
        logger.info("🎉 所有全面測試都通過了！系統可投入使用！")
        return 0
    else:
        logger.error(f"⚠️  有 {total - passed} 個測試失敗，需要修復")
        return 1

if __name__ == "__main__":
    sys.exit(main())