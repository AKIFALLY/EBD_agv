#!/usr/bin/env python3
"""
測試 flow_wcs 使用統一模型的查詢功能
"""

import sys
sys.path.append('/app/db_proxy_ws/src/db_proxy')
sys.path.append('/app/flow_wcs_ws/src/flow_wcs')

from flow_wcs.database_unified import db_manager
import json

def test_queries():
    print("=== 測試 Flow WCS 統一模型查詢 ===\n")
    
    # 測試查詢 racks
    print("1. 測試查詢 Racks:")
    try:
        racks = db_manager.query_racks()
        print(f"   ✅ 成功查詢到 {len(racks)} 個 racks")
        if racks:
            print(f"   範例資料: {json.dumps(racks[0], indent=2, default=str)[:200]}...")
    except Exception as e:
        print(f"   ❌ 錯誤: {e}")
    
    print("\n2. 測試查詢 Tasks:")
    try:
        tasks = db_manager.query_tasks()
        print(f"   ✅ 成功查詢到 {len(tasks)} 個 tasks")
        if tasks:
            print(f"   範例資料: {json.dumps(tasks[0], indent=2, default=str)[:200]}...")
    except Exception as e:
        print(f"   ❌ 錯誤: {e}")
    
    print("\n3. 測試查詢 AGVs:")
    try:
        agvs = db_manager.query_agvs()
        print(f"   ✅ 成功查詢到 {len(agvs)} 個 AGVs")
        if agvs:
            print(f"   範例資料: {json.dumps(agvs[0], indent=2, default=str)[:200]}...")
    except Exception as e:
        print(f"   ❌ 錯誤: {e}")
    
    print("\n4. 測試查詢 Locations:")
    try:
        locations = db_manager.query_locations()
        print(f"   ✅ 成功查詢到 {len(locations)} 個 locations")
        if locations:
            print(f"   範例資料: {json.dumps(locations[0], indent=2, default=str)[:200]}...")
    except Exception as e:
        print(f"   ❌ 錯誤: {e}")
    
    print("\n5. 測試建立任務:")
    try:
        task_id = db_manager.create_task(
            type="test_flow",
            work_id="TEST_WORK_001",
            location_id=1,
            priority=100,
            metadata={"test": "data"}
        )
        print(f"   ✅ 成功建立任務: {task_id}")
    except Exception as e:
        print(f"   ❌ 錯誤: {e}")
    
    print("\n=== 測試完成 ===")

if __name__ == "__main__":
    test_queries()