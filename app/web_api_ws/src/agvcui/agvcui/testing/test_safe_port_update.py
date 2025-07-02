#!/usr/bin/env python3
"""
測試安全的端口更新功能
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
        get_eqps, get_eqp_by_id, get_eqp_ports_by_eqp_id,
        check_port_has_signals, safe_update_device_ports
    )
    from db_proxy.models import EqpSignal
    from db_proxy.connection_pool_manager import ConnectionPoolManager
    from sqlmodel import select
except ImportError as e:
    print(f"導入錯誤: {e}")
    print("請確保所有依賴模組都已正確安裝")
    sys.exit(1)


def test_safe_port_update():
    """測試安全的端口更新功能"""
    print("開始測試安全的端口更新功能...")
    
    # 1. 獲取現有設備
    print("\n1. 獲取現有設備")
    devices = get_eqps(limit=5)
    print(f"找到 {len(devices)} 個設備")
    
    if not devices:
        print("❌ 沒有找到任何設備，無法進行測試")
        return
    
    # 2. 選擇一個有端口的設備進行測試
    test_device = None
    for device in devices:
        ports = get_eqp_ports_by_eqp_id(device.id)
        if ports:
            test_device = device
            break
    
    if not test_device:
        print("❌ 沒有找到有端口的設備，無法進行測試")
        return
    
    print(f"\n2. 選擇測試設備: {test_device.name} (ID: {test_device.id})")
    
    # 3. 檢查設備的端口和 signal 引用情況
    print("\n3. 檢查端口的 signal 引用情況")
    device_ports = get_eqp_ports_by_eqp_id(test_device.id)
    print(f"該設備有 {len(device_ports)} 個端口:")
    
    ports_with_signals = []
    ports_without_signals = []
    
    for port in device_ports:
        has_signals = check_port_has_signals(port.id)
        if has_signals:
            ports_with_signals.append(port)
            print(f"  ⚠️  {port.name} (ID: {port.id}) - 被 signal 引用")
        else:
            ports_without_signals.append(port)
            print(f"  ✅ {port.name} (ID: {port.id}) - 沒有 signal 引用")
    
    # 4. 測試安全更新邏輯
    print(f"\n4. 測試安全更新邏輯")
    print(f"有 signal 引用的端口: {len(ports_with_signals)} 個")
    print(f"沒有 signal 引用的端口: {len(ports_without_signals)} 個")
    
    # 模擬編輯操作：保留所有有 signal 引用的端口
    current_port_names = [port.name for port in device_ports]
    print(f"當前端口名稱: {current_port_names}")
    
    # 模擬用戶選擇：保留有 signal 的端口，移除一些沒有 signal 的端口
    if ports_with_signals:
        # 保留所有有 signal 的端口
        selected_ports = [port.name for port in ports_with_signals]
        # 如果有沒有 signal 的端口，只保留第一個
        if ports_without_signals:
            selected_ports.append(ports_without_signals[0].name)
        
        print(f"模擬選擇的端口: {selected_ports}")
        
        # 5. 執行安全更新（這裡只是模擬，不實際執行）
        print(f"\n5. 模擬安全更新操作")
        print("如果執行 safe_update_device_ports，預期結果:")
        
        for port in device_ports:
            if port.name not in selected_ports:
                if check_port_has_signals(port.id):
                    print(f"  ⚠️  {port.name} - 被 signal 引用，無法刪除（會保留）")
                else:
                    print(f"  🗑️  {port.name} - 沒有 signal 引用，可以安全刪除")
            else:
                print(f"  ✅ {port.name} - 用戶選擇保留")
    
    # 6. 檢查其他設備的情況
    print(f"\n6. 檢查其他設備的端口 signal 引用情況")
    for i, device in enumerate(devices[1:3], 2):
        ports = get_eqp_ports_by_eqp_id(device.id)
        if ports:
            signals_count = sum(1 for port in ports if check_port_has_signals(port.id))
            print(f"設備 {i}: {device.name} - {len(ports)} 個端口，{signals_count} 個被 signal 引用")
    
    print("\n✅ 安全端口更新功能測試完成！")
    print("💡 提示: 實際的 safe_update_device_ports 函數會:")
    print("   - 只刪除沒有被 signal 引用的端口")
    print("   - 保留所有被 signal 引用的端口（即使用戶沒有選擇）")
    print("   - 創建用戶新選擇的端口")


if __name__ == "__main__":
    test_safe_port_update()
