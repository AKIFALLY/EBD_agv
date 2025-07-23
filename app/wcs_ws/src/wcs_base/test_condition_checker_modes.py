#!/usr/bin/env python3
"""
測試 TaskConditionChecker 的即時查詢模式與預存結果模式
比較兩種模式的效能和功能差異
"""

import sys
import os
import time
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..'))

from wcs_base.task_condition_checker import TaskConditionChecker
from db_proxy.connection import connection_pool
from db_proxy.crud.task_condition_crud import task_condition_crud


class MockLogger:
    """模擬日誌記錄器"""
    def info(self, msg): print(f"[INFO] {msg}")
    def debug(self, msg): print(f"[DEBUG] {msg}")
    def warning(self, msg): print(f"[WARNING] {msg}")
    def error(self, msg): print(f"[ERROR] {msg}")


class MockDBManager:
    """模擬資料庫管理器"""
    def get_session(self):
        return connection_pool.get_session()


def test_real_time_mode():
    """測試即時查詢模式"""
    print("\n🔍 測試即時查詢模式")
    print("=" * 50)
    
    db_manager = MockDBManager()
    logger = MockLogger()
    
    # 創建即時查詢模式的檢查器
    checker = TaskConditionChecker(
        db_manager=db_manager,
        logger=logger,
        real_time_mode=True,
        query_timeout=10
    )
    
    # 測試單一條件檢查
    print("\n📋 測試單一條件檢查...")
    start_time = time.time()
    
    # 檢查 ID 1 的條件
    result = checker.get_task_condition_results(1)
    
    end_time = time.time()
    execution_time = end_time - start_time
    
    print(f"⏱️ 執行時間: {execution_time:.3f} 秒")
    
    if result:
        print(f"✅ 查詢成功")
        print(f"   Success: {result.get('success')}")
        print(f"   Row count: {result.get('row_count', 0)}")
        print(f"   Columns: {result.get('columns', [])}")
        
        data = result.get('data', [])
        if data:
            print(f"   Data sample: {data[0] if len(data) > 0 else 'None'}")
    else:
        print("❌ 查詢失敗")
    
    return execution_time


def test_stored_results_mode():
    """測試預存結果模式"""
    print("\n🔍 測試預存結果模式")
    print("=" * 50)
    
    db_manager = MockDBManager()
    logger = MockLogger()
    
    # 創建預存結果模式的檢查器
    checker = TaskConditionChecker(
        db_manager=db_manager,
        logger=logger,
        real_time_mode=False
    )
    
    # 測試單一條件檢查
    print("\n📋 測試單一條件檢查...")
    start_time = time.time()
    
    # 檢查 ID 1 的條件
    result = checker.get_task_condition_results(1)
    
    end_time = time.time()
    execution_time = end_time - start_time
    
    print(f"⏱️ 執行時間: {execution_time:.3f} 秒")
    
    if result:
        print(f"✅ 查詢成功")
        print(f"   Success: {result.get('success')}")
        print(f"   Row count: {result.get('row_count', 0)}")
        print(f"   Columns: {result.get('columns', [])}")
        
        data = result.get('data', [])
        if data:
            print(f"   Data sample: {data[0] if len(data) > 0 else 'None'}")
    else:
        print("❌ 查詢失敗或無預存結果")
    
    return execution_time


def test_condition_flow():
    """測試完整的條件檢查流程"""
    print("\n🔍 測試完整條件檢查流程（即時查詢模式）")
    print("=" * 50)
    
    db_manager = MockDBManager()
    logger = MockLogger()
    
    # 創建即時查詢模式的檢查器
    checker = TaskConditionChecker(
        db_manager=db_manager,
        logger=logger,
        real_time_mode=True,
        query_timeout=10
    )
    
    # 測試完整流程
    start_time = time.time()
    
    success, collected_data = checker.check_conditions_from_id(start_id=1)
    
    end_time = time.time()
    execution_time = end_time - start_time
    
    print(f"⏱️ 總執行時間: {execution_time:.3f} 秒")
    print(f"✅ 流程結果: {'成功' if success else '失敗'}")
    print(f"📊 收集的資料: {collected_data}")
    
    return success, execution_time


def test_mode_switching():
    """測試模式切換功能"""
    print("\n🔍 測試模式切換功能")
    print("=" * 50)
    
    db_manager = MockDBManager()
    logger = MockLogger()
    
    # 創建檢查器
    checker = TaskConditionChecker(
        db_manager=db_manager,
        logger=logger,
        real_time_mode=True
    )
    
    # 顯示初始模式
    mode_info = checker.get_mode_info()
    print(f"📋 初始模式: {mode_info}")
    
    # 切換到預存結果模式
    checker.set_mode(False)
    mode_info = checker.get_mode_info()
    print(f"📋 切換後模式: {mode_info}")
    
    # 測試查詢超時設定
    checker.set_query_timeout(60)
    
    # 測試最大迭代次數設定
    checker.set_max_iterations(50)
    
    # 顯示最終配置
    mode_info = checker.get_mode_info()
    print(f"📋 最終配置: {mode_info}")


def performance_comparison():
    """效能比較測試"""
    print("\n🔍 效能比較測試")
    print("=" * 50)
    
    # 測試次數
    test_count = 5
    
    print(f"📊 進行 {test_count} 次測試...")
    
    real_time_times = []
    stored_times = []
    
    for i in range(test_count):
        print(f"\n--- 第 {i+1} 次測試 ---")
        
        # 測試即時查詢模式
        rt_time = test_real_time_mode()
        real_time_times.append(rt_time)
        
        # 測試預存結果模式
        st_time = test_stored_results_mode()
        stored_times.append(st_time)
    
    # 計算平均時間
    avg_real_time = sum(real_time_times) / len(real_time_times)
    avg_stored_time = sum(stored_times) / len(stored_times)
    
    print(f"\n📊 效能比較結果:")
    print(f"   即時查詢模式平均時間: {avg_real_time:.3f} 秒")
    print(f"   預存結果模式平均時間: {avg_stored_time:.3f} 秒")
    print(f"   效能差異: {abs(avg_real_time - avg_stored_time):.3f} 秒")
    
    if avg_real_time < avg_stored_time:
        print(f"   🏆 即時查詢模式較快")
    elif avg_stored_time < avg_real_time:
        print(f"   🏆 預存結果模式較快")
    else:
        print(f"   🤝 兩種模式效能相當")


def main():
    """主測試函數"""
    print("🚀 TaskConditionChecker 模式測試開始")
    
    try:
        # 測試模式切換
        test_mode_switching()
        
        # 測試即時查詢模式
        test_real_time_mode()
        
        # 測試預存結果模式
        test_stored_results_mode()
        
        # 測試完整流程
        test_condition_flow()
        
        # 效能比較
        performance_comparison()
        
        print("\n🎉 所有測試完成")
        
    except Exception as e:
        print(f"\n❌ 測試過程中發生錯誤: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
