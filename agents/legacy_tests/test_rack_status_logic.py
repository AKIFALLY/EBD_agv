#!/usr/bin/env python3
"""
測試檔案: test_rack_status_logic.py
用途: 測試 check_rack_status 函數的判斷邏輯
創建日期: 2025-08-14
狀態: 臨時測試檔案
"""

import sys
sys.path.append('/app/flow_wcs_ws/src/flow_wcs')
sys.path.append('/app/db_proxy_ws/src')

from flow_wcs.database import DatabaseManager

def test_rack_status_logic():
    """測試架台狀態判斷邏輯"""
    db = DatabaseManager()
    
    print("\n測試 check_rack_status 邏輯...")
    print("="*80)
    
    # 測試 Rack 103
    rack_id = 103
    print(f"\n測試 Rack {rack_id}:")
    
    with db.get_session() as session:
        from db_proxy.models.carrier import Carrier
        from db_proxy.models.rack import Rack
        from db_proxy.models.agvc_product import Product
        
        # 查詢架台
        rack = session.query(Rack).filter(Rack.id == rack_id).first()
        if not rack:
            print(f"  架台 {rack_id} 不存在")
            return
            
        # 查詢產品
        product = None
        if rack.product_id:
            product = session.query(Product).filter(Product.id == rack.product_id).first()
            print(f"  產品: {product.name if product else 'Unknown'} (尺寸: {product.size if product else 'Unknown'})")
        
        # 查詢所有載具
        carriers = session.query(Carrier).filter(Carrier.rack_id == rack_id).all()
        print(f"  總載具數: {len(carriers)}")
        
        # 列出所有載具
        for c in carriers:
            print(f"    - Carrier ID {c.id}: rack_index={c.rack_index}, status_id={c.status_id}")
        
        # 根據產品尺寸分析
        if product and product.size == 'L':
            # L產品
            a_carriers = [c for c in carriers if c.rack_index and c.rack_index <= 8]
            b_carriers = [c for c in carriers if c.rack_index and c.rack_index > 8 and c.rack_index <= 16]
        else:
            # S產品
            a_carriers = [c for c in carriers if c.rack_index and c.rack_index <= 16]
            b_carriers = [c for c in carriers if c.rack_index and c.rack_index > 16]
        
        print(f"\n  A面載具 (rack_index <= 16): {len(a_carriers)}個")
        for c in a_carriers:
            status_name = {1: "pending", 2: "processing", 3: "completed"}.get(c.status_id, f"status_{c.status_id}")
            print(f"    - ID {c.id}: index={c.rack_index}, status={status_name}")
        
        print(f"\n  B面載具 (rack_index > 16): {len(b_carriers)}個")
        for c in b_carriers:
            status_name = {1: "pending", 2: "processing", 3: "completed"}.get(c.status_id, f"status_{c.status_id}")
            print(f"    - ID {c.id}: index={c.rack_index}, status={status_name}")
        
        # 檢查狀態
        print(f"\n  A面狀態檢查:")
        # has_work: 有狀態不是3的載具
        a_has_work = any(c.status_id != 3 for c in a_carriers)
        print(f"    - has_work (有待作業): {a_has_work}")
        
        # all_complete: 全部是3或沒有載具
        a_all_complete = all(c.status_id == 3 for c in a_carriers) if a_carriers else True
        print(f"    - all_complete (全部完成): {a_all_complete}")
        
        print(f"\n  B面狀態檢查:")
        b_has_work = any(c.status_id != 3 for c in b_carriers)
        print(f"    - has_work (有待作業): {b_has_work}")
        
        b_all_complete = all(c.status_id == 3 for c in b_carriers) if b_carriers else True
        print(f"    - all_complete (全部完成): {b_all_complete}")
        
        # 判斷是否需要翻轉
        needs_rotation = b_has_work and a_all_complete
        print(f"\n  需要翻轉判斷: B面有待作業({b_has_work}) AND A面全部完成({a_all_complete}) = {needs_rotation}")
    
    # 使用 check_rack_status 函數測試
    print("\n使用 check_rack_status 函數:")
    b_has_work = db.check_rack_status(rack_id, 'b', 'has_work')
    a_all_complete = db.check_rack_status(rack_id, 'a', 'all_complete')
    print(f"  B面有待作業: {b_has_work}")
    print(f"  A面全部完成: {a_all_complete}")
    print(f"  應該創建任務: {b_has_work and a_all_complete}")

if __name__ == "__main__":
    test_rack_status_logic()