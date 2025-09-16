#!/usr/bin/env python3
"""
測試檔案: test_rotation_conditions.py
用途: 驗證哪些架台真正滿足翻轉條件
創建日期: 2025-08-14
狀態: 臨時測試檔案
"""

import sys
sys.path.append('/app/flow_wcs_ws/src/flow_wcs')
sys.path.append('/app/db_proxy_ws/src')

from flow_wcs.database import DatabaseManager

def test_rotation_conditions():
    """測試架台翻轉條件"""
    db = DatabaseManager()
    
    print("\n檢查架台翻轉條件...")
    print("="*80)
    
    # 測試每個架台
    racks = [
        (101, 10001, "Rack 101"),
        (102, 20001, "Rack 102"),
        (103, 30001, "Rack 103"),
        (104, 40001, "Rack 104"),
        (105, 50001, "Rack 105")
    ]
    
    rotation_needed = []
    
    for rack_id, location_id, name in racks:
        print(f"\n{name} (Location {location_id}):")
        
        # 檢查B面是否有待作業
        b_has_work = db.check_rack_status(rack_id, 'b', 'has_work')
        print(f"  B面有待作業: {'✅' if b_has_work else '❌'}")
        
        # 檢查A面是否全部完成
        a_complete = db.check_rack_status(rack_id, 'a', 'all_complete')
        print(f"  A面全部完成: {'✅' if a_complete else '❌'}")
        
        # 判斷是否需要翻轉
        needs_rotation = b_has_work and a_complete
        print(f"  需要翻轉: {'✅ 是' if needs_rotation else '❌ 否'}")
        
        if needs_rotation:
            rotation_needed.append((rack_id, location_id, name))
        
        # 詳細載具狀態
        with db.get_session() as session:
            from db_proxy.models.carrier import Carrier
            from db_proxy.models.rack import Rack
            from db_proxy.models.agvc_product import Product
            
            # 查詢架台
            rack = session.query(Rack).filter(Rack.id == rack_id).first()
            if rack:
                # 查詢產品
                product = None
                if rack.product_id:
                    product = session.query(Product).filter(Product.id == rack.product_id).first()
                    print(f"  產品: {product.name if product else 'Unknown'} (尺寸: {product.size if product else 'Unknown'})")
                
                # 查詢載具
                carriers = session.query(Carrier).filter(Carrier.rack_id == rack_id).all()
                
                # 根據產品尺寸分析載具
                if product and product.size == 'L':
                    a_carriers = [c for c in carriers if c.rack_index and c.rack_index <= 8]
                    b_carriers = [c for c in carriers if c.rack_index and c.rack_index > 8 and c.rack_index <= 16]
                else:
                    a_carriers = [c for c in carriers if c.rack_index and c.rack_index <= 16]
                    b_carriers = [c for c in carriers if c.rack_index and c.rack_index > 16]
                
                print(f"  A面載具: {len(a_carriers)}個")
                if a_carriers:
                    statuses = {}
                    for c in a_carriers:
                        statuses[c.status_id] = statuses.get(c.status_id, 0) + 1
                    for status_id, count in statuses.items():
                        status_name = {1: "pending", 2: "processing", 3: "completed"}.get(status_id, f"status_{status_id}")
                        print(f"    - {count}個 {status_name}")
                
                print(f"  B面載具: {len(b_carriers)}個")
                if b_carriers:
                    statuses = {}
                    for c in b_carriers:
                        statuses[c.status_id] = statuses.get(c.status_id, 0) + 1
                    for status_id, count in statuses.items():
                        status_name = {1: "pending", 2: "processing", 3: "completed"}.get(status_id, f"status_{status_id}")
                        print(f"    - {count}個 {status_name}")
    
    print("\n" + "="*80)
    print(f"\n總結: {len(rotation_needed)}個架台需要翻轉")
    for rack_id, location_id, name in rotation_needed:
        print(f"  - {name} at Location {location_id}")
    
    return rotation_needed

if __name__ == "__main__":
    test_rotation_conditions()