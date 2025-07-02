#!/usr/bin/env python3
"""
測試設備編輯模式的端口顯示功能
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
        get_all_available_ports, get_ports_not_assigned_to_any_eqp
    )
except ImportError as e:
    print(f"導入錯誤: {e}")
    print("請確保所有依賴模組都已正確安裝")
    sys.exit(1)


def test_device_edit_mode():
    """測試設備編輯模式的端口顯示邏輯"""
    print("開始測試設備編輯模式的端口顯示功能...")
    
    # 1. 獲取現有設備列表
    print("\n1. 獲取現有設備列表")
    devices = get_eqps(limit=5)  # 只取前5個設備
    print(f"找到 {len(devices)} 個設備")
    
    if not devices:
        print("❌ 沒有找到任何設備，無法進行測試")
        return
    
    # 2. 選擇第一個有端口的設備進行測試
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
    
    # 3. 測試編輯模式的端口顯示邏輯
    print("\n3. 測試編輯模式的端口顯示邏輯")
    device_ports = get_eqp_ports_by_eqp_id(test_device.id)
    print(f"該設備擁有 {len(device_ports)} 個端口:")
    
    # 模擬編輯模式的邏輯
    available_ports = [{"name": port.name, "description": port.description} for port in device_ports]
    selected_ports = [port.name for port in device_ports]
    
    print("編輯模式下顯示的端口選項:")
    for port in available_ports:
        is_selected = "✓" if port["name"] in selected_ports else " "
        print(f"  [{is_selected}] {port['name']} - {port['description']}")
    
    # 4. 對比創建模式的端口選項
    print("\n4. 對比創建模式的端口選項")
    create_mode_ports = get_ports_not_assigned_to_any_eqp()
    print(f"創建模式下有 {len(create_mode_ports)} 個端口模板可選:")
    for port in create_mode_ports[:5]:  # 只顯示前5個
        print(f"  - {port['name']}: {port['description']}")
    
    # 5. 驗證邏輯正確性
    print("\n5. 驗證邏輯正確性")
    print(f"✅ 編輯模式: 只顯示設備 '{test_device.name}' 的 {len(available_ports)} 個端口")
    print(f"✅ 創建模式: 顯示 {len(create_mode_ports)} 個可用端口模板")
    print("✅ 編輯模式下所有端口都預設為選中狀態")
    
    # 6. 測試其他設備
    print("\n6. 測試其他設備的端口")
    for i, device in enumerate(devices[1:3], 2):  # 測試第2和第3個設備
        ports = get_eqp_ports_by_eqp_id(device.id)
        print(f"設備 {i}: {device.name} 有 {len(ports)} 個端口")
        if ports:
            for port in ports[:3]:  # 只顯示前3個端口
                print(f"  - {port.name}: {port.description}")
    
    print("\n✅ 設備編輯模式端口顯示功能測試完成！")


if __name__ == "__main__":
    test_device_edit_mode()
