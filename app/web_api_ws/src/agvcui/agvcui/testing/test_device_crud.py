#!/usr/bin/env python3
"""
測試設備CRUD功能
"""

import sys
import os

# 添加必要的路徑
current_dir = os.path.dirname(os.path.abspath(__file__))
agvcui_src = os.path.join(current_dir, '..', '..')
db_proxy_src = os.path.join(current_dir, '..', '..',
                            '..', '..', '..', 'db_proxy_ws', 'src')

sys.path.insert(0, agvcui_src)
sys.path.insert(0, db_proxy_src)

try:
    from agvcui.db import (
        create_eqp, get_eqp_by_id, update_eqp, delete_eqp,
        create_eqp_port, get_eqp_ports_by_eqp_id, delete_eqp_port,
        get_all_available_ports, get_available_ports_for_eqp, get_ports_not_assigned_to_any_eqp
    )
except ImportError as e:
    print(f"導入錯誤: {e}")
    print("請確保所有依賴模組都已正確安裝")
    sys.exit(1)


def test_device_crud():
    """測試設備的CRUD操作"""
    print("開始測試設備CRUD功能...")

    # 測試獲取資料庫中的端口
    print("\n1. 測試獲取資料庫中的端口")
    all_ports = get_all_available_ports()
    print(f"資料庫中總端口數量: {len(all_ports)}")
    for port in all_ports[:5]:  # 只顯示前5個
        print(
            f"  - {port['name']} (設備ID: {port['eqp_id']}): {port['description']}")

    # 測試獲取可用的端口模板
    print("\n2. 測試獲取可用的端口模板")
    available_ports = get_ports_not_assigned_to_any_eqp()
    print(f"可用端口模板數量: {len(available_ports)}")
    for port in available_ports[:3]:  # 只顯示前3個
        print(f"  - {port['name']}: {port['description']}")

    # 測試創建設備
    print("\n3. 測試創建設備")
    device_data = {
        "name": "測試設備",
        "description": "這是一個測試設備",
        "location_id": 100
    }

    try:
        new_device = create_eqp(device_data)
        print(f"成功創建設備，ID: {new_device.id}")
        device_id = new_device.id

        # 測試創建端口
        print("\n4. 測試創建端口")
        selected_ports = ["Port01", "Port02", "Inport", "Outport"]
        for port_name in selected_ports:
            port_data = {
                "eqp_id": device_id,
                "name": port_name,
                "description": f"{device_data['name']} {port_name}"
            }
            port = create_eqp_port(port_data)
            print(f"成功創建端口: {port.name}")

        # 測試獲取設備
        print("\n5. 測試獲取設備")
        retrieved_device = get_eqp_by_id(device_id)
        if retrieved_device:
            print(f"成功獲取設備: {retrieved_device.name}")
            print(f"描述: {retrieved_device.description}")
            print(f"位置ID: {retrieved_device.location_id}")

        # 測試獲取設備端口
        print("\n6. 測試獲取設備端口")
        ports = get_eqp_ports_by_eqp_id(device_id)
        print(f"設備端口數量: {len(ports)}")
        for port in ports:
            print(f"  - {port.name}: {port.description}")

        # 測試更新設備
        print("\n7. 測試更新設備")
        update_data = {
            "name": "更新後的測試設備",
            "description": "這是更新後的測試設備",
            "location_id": 200
        }
        success = update_eqp(device_id, update_data)
        if success:
            print("成功更新設備")
            updated_device = get_eqp_by_id(device_id)
            print(f"更新後名稱: {updated_device.name}")
            print(f"更新後位置ID: {updated_device.location_id}")

        # 清理：刪除端口和設備
        print("\n8. 清理測試數據")
        for port in ports:
            delete_eqp_port(port.id)
            print(f"刪除端口: {port.name}")

        success = delete_eqp(device_id)
        if success:
            print("成功刪除設備")

        print("\n✅ 設備CRUD功能測試完成！")

    except Exception as e:
        print(f"❌ 測試失敗: {str(e)}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    test_device_crud()
