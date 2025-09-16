#!/usr/bin/env python3
"""
測試檔案: test_task_exists.py
用途: 測試 check_task_exists 函數邏輯
創建日期: 2025-08-14
狀態: 臨時測試檔案
"""

import sys
sys.path.append('/app/flow_wcs_ws/src/flow_wcs')
sys.path.append('/app/db_proxy_ws/src')

from flow_wcs.database import DatabaseManager

def test_check_task_exists():
    """測試任務存在檢查功能"""
    db = DatabaseManager()
    
    print("\n測試 check_task_exists 函數...")
    print("="*60)
    
    # 測試每個位置是否有未完成的任務
    locations = [10001, 20001, 30001, 40001, 50001]
    racks = [101, 102, 103, 104, 105]
    
    for loc_id, rack_id in zip(locations, racks):
        # 檢查是否有任務（不指定status時應該檢查所有未完成狀態）
        exists = db.check_task_exists(
            type="RACK_ROTATION",
            location_id=loc_id,
            rack_id=rack_id
        )
        print(f"Location {loc_id}, Rack {rack_id}: {'✅ 任務已存在' if exists else '❌ 沒有任務'}")
        
        # 也測試只用 location_id 的情況
        exists_by_loc = db.check_task_exists(
            type="RACK_ROTATION",
            location_id=loc_id
        )
        print(f"  只用 location_id 檢查: {'✅ 任務已存在' if exists_by_loc else '❌ 沒有任務'}")
    
    print("\n" + "="*60)
    
    # 查詢資料庫中實際的任務
    with db.get_session() as session:
        from db_proxy.models.agvc_task import Task
        tasks = session.query(Task).filter(
            Task.type == "RACK_ROTATION",
            Task.status_id.in_([0, 1, 2, 3])
        ).all()
        
        print(f"\n資料庫中有 {len(tasks)} 個未完成的 RACK_ROTATION 任務:")
        for task in tasks[:5]:  # 只顯示前5個
            print(f"  Task ID: {task.id}, Location: {task.location_id}, Rack: {task.rack_id}, Status: {task.status_id}")

if __name__ == "__main__":
    test_check_task_exists()