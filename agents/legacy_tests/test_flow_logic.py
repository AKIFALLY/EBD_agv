#!/usr/bin/env python3
"""
測試檔案: test_flow_logic.py
用途: 模擬流程執行邏輯，找出為什麼所有架台都創建了任務
創建日期: 2025-08-14
狀態: 臨時測試檔案
"""

import sys
sys.path.append('/app/flow_wcs_ws/src/flow_wcs')
sys.path.append('/app/db_proxy_ws/src')

from flow_wcs.database import DatabaseManager

def test_flow_logic():
    """模擬流程執行邏輯"""
    db = DatabaseManager()
    
    print("\n模擬 rack_rotation_room_inlet 流程執行...")
    print("="*80)
    
    # 1. 查詢所有入口位置
    locations = db.query_locations(type="room_inlet", rooms=[1, 2, 3, 4, 5])
    print(f"\n找到 {len(locations)} 個入口位置")
    
    for location in locations:
        print(f"\n處理位置: {location['name']} (ID: {location['id']})")
        
        # 2. 查詢該位置的架台
        racks = db.query_racks(location_id=location['id'])
        
        if not racks:
            print(f"  ❌ 沒有架台")
            continue
            
        rack = racks[0]
        print(f"  找到架台: ID {rack['id']}")
        
        # 3. 檢查B面是否有待作業
        b_has_work = db.check_rack_status(rack['id'], 'b', 'has_work')
        print(f"  B面有待作業: {'✅' if b_has_work else '❌'}")
        
        # 4. 檢查A面是否全部完成
        a_complete = db.check_rack_status(rack['id'], 'a', 'all_complete')
        print(f"  A面全部完成: {'✅' if a_complete else '❌'}")
        
        # 5. 判斷是否需要翻轉
        needs_rotation = b_has_work and a_complete
        print(f"  需要翻轉: {'✅ 是' if needs_rotation else '❌ 否'}")
        
        # 6. 檢查是否已有任務
        if needs_rotation:
            task_exists = db.check_task_exists(
                type="RACK_ROTATION",
                location_id=location['id'],
                rack_id=rack['id']
            )
            print(f"  任務已存在: {'✅' if task_exists else '❌'}")
            
            if not task_exists:
                print(f"  → 將會創建翻轉任務")
            else:
                print(f"  → 跳過（任務已存在）")
        
        # 顯示詳細載具資訊
        with db.get_session() as session:
            from db_proxy.models.carrier import Carrier
            carriers = session.query(Carrier).filter(Carrier.rack_id == rack['id']).all()
            
            # 分析載具
            a_carriers = [c for c in carriers if c.rack_index and c.rack_index <= 16]
            b_carriers = [c for c in carriers if c.rack_index and c.rack_index > 16]
            
            print(f"  載具詳情:")
            print(f"    A面 (1-16): {len(a_carriers)}個")
            if a_carriers:
                status_counts = {}
                for c in a_carriers:
                    status_counts[c.status_id] = status_counts.get(c.status_id, 0) + 1
                for status_id, count in status_counts.items():
                    status_name = {1: "pending", 2: "processing", 3: "completed"}.get(status_id, f"status_{status_id}")
                    print(f"      - {count}個 {status_name}")
            
            print(f"    B面 (17-32): {len(b_carriers)}個")
            if b_carriers:
                status_counts = {}
                for c in b_carriers:
                    status_counts[c.status_id] = status_counts.get(c.status_id, 0) + 1
                for status_id, count in status_counts.items():
                    status_name = {1: "pending", 2: "processing", 3: "completed"}.get(status_id, f"status_{status_id}")
                    print(f"      - {count}個 {status_name}")

if __name__ == "__main__":
    test_flow_logic()