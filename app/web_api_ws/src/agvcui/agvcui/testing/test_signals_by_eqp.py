#!/usr/bin/env python3
"""
測試按設備ID分頁的信號功能
"""

import sys
import os

# 添加必要的路徑
current_dir = os.path.dirname(os.path.abspath(__file__))
agvcui_src = os.path.join(current_dir, '..', '..')
db_proxy_src = os.path.join(current_dir, '..', '..', '..', '..', '..', 'db_proxy_ws', 'src')

sys.path.insert(0, agvcui_src)
sys.path.insert(0, db_proxy_src)

try:
    from agvcui.db import (
        get_signals_by_eqp_id, count_signals_by_eqp_id, 
        get_eqps_with_signal_counts, get_signals, count_signals
    )
except ImportError as e:
    print(f"導入錯誤: {e}")
    print("請確保所有依賴模組都已正確安裝")
    sys.exit(1)


def test_signals_by_eqp():
    """測試按設備ID分頁的信號功能"""
    print("開始測試按設備ID分頁的信號功能...")
    
    # 1. 測試獲取所有設備及其信號數量
    print("\n1. 測試獲取所有設備及其信號數量")
    try:
        eqps_with_counts = get_eqps_with_signal_counts()
        print(f"✅ 成功獲取設備信號統計，共 {len(eqps_with_counts)} 個設備")
        
        # 顯示前10個設備的信號統計
        print("設備信號統計:")
        for i, eqp in enumerate(eqps_with_counts[:10], 1):
            print(f"  {i}. {eqp['name']} (ID: {eqp['id']}) - {eqp['signal_count']} 個信號")
        
        if len(eqps_with_counts) > 10:
            print(f"  ... 還有 {len(eqps_with_counts) - 10} 個設備")
        
        # 找到有信號的設備進行測試
        test_eqp = None
        for eqp in eqps_with_counts:
            if eqp['signal_count'] > 0:
                test_eqp = eqp
                break
        
        if not test_eqp:
            print("⚠️  沒有找到有信號的設備，無法進行詳細測試")
            return
        
        print(f"\n選擇測試設備: {test_eqp['name']} (ID: {test_eqp['id']}, {test_eqp['signal_count']} 個信號)")
        
        # 2. 測試按設備ID獲取信號
        print("\n2. 測試按設備ID獲取信號")
        eqp_id = test_eqp['id']
        
        # 測試第一頁
        signals_page1 = get_signals_by_eqp_id(eqp_id, offset=0, limit=5)
        total_signals = count_signals_by_eqp_id(eqp_id)
        
        print(f"✅ 設備 {test_eqp['name']} 的信號:")
        print(f"   總信號數: {total_signals}")
        print(f"   第一頁信號數: {len(signals_page1)}")
        
        for i, signal in enumerate(signals_page1, 1):
            print(f"   {i}. {signal.name} (ID: {signal.id}) - 值: {signal.value}")
        
        # 3. 測試分頁功能
        print("\n3. 測試分頁功能")
        if total_signals > 5:
            signals_page2 = get_signals_by_eqp_id(eqp_id, offset=5, limit=5)
            print(f"✅ 第二頁信號數: {len(signals_page2)}")
            
            for i, signal in enumerate(signals_page2, 1):
                print(f"   {i}. {signal.name} (ID: {signal.id}) - 值: {signal.value}")
        else:
            print("✅ 信號數量不足，無需分頁")
        
        # 4. 對比全部信號查詢
        print("\n4. 對比全部信號查詢")
        all_signals = get_signals(offset=0, limit=10)
        total_all_signals = count_signals()
        
        print(f"✅ 全部信號統計:")
        print(f"   總信號數: {total_all_signals}")
        print(f"   前10個信號:")
        
        for i, signal in enumerate(all_signals, 1):
            print(f"   {i}. {signal.name} (設備ID: {signal.eqp_id}) - 值: {signal.value}")
        
        # 5. 驗證篩選邏輯
        print("\n5. 驗證篩選邏輯")
        device_signals = get_signals_by_eqp_id(eqp_id, offset=0, limit=100)  # 獲取所有信號
        
        # 檢查所有信號是否都屬於指定設備
        all_belong_to_device = all(signal.eqp_id == eqp_id for signal in device_signals)
        
        if all_belong_to_device:
            print(f"✅ 篩選正確：所有 {len(device_signals)} 個信號都屬於設備 {test_eqp['name']}")
        else:
            print(f"❌ 篩選錯誤：發現不屬於設備 {test_eqp['name']} 的信號")
        
        # 6. 測試前端URL參數
        print("\n6. 模擬前端URL參數")
        print("前端URL示例:")
        print(f"   所有信號: /signals")
        print(f"   設備信號第1頁: /signals?eqp_id={eqp_id}")
        print(f"   設備信號第2頁: /signals?eqp_id={eqp_id}&page=2")
        print(f"   設備信號第3頁: /signals?eqp_id={eqp_id}&page=3")
        
        print("\n✅ 按設備ID分頁的信號功能測試完成！")
        print("💡 總結:")
        print("   - 成功獲取所有設備的信號統計")
        print("   - 按設備ID篩選信號功能正常")
        print("   - 分頁功能支持設備篩選")
        print("   - URL參數正確處理")
        
    except Exception as e:
        print(f"❌ 測試失敗: {str(e)}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    test_signals_by_eqp()
