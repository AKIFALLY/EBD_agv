#!/usr/bin/env python3
"""調試 put 流程執行步驟"""

import sys
sys.path.insert(0, '/app/tafl_wcs_ws/src/tafl_wcs')
sys.path.insert(0, '/app/db_proxy_ws/install/db_proxy/lib/python3.12/site-packages')

from tafl_wcs.tafl_db_bridge import TAFLDatabaseBridge
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import Carrier

def main():
    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    db_bridge = TAFLDatabaseBridge(db_url)
    pool_manager = ConnectionPoolManager(db_url)

    print("="*60)
    print("調試 loader_put_soaker 流程步驟")
    print("="*60)

    # 步驟 1: 創建測試載具
    print("\n📝 步驟 1: 創建測試載具...")
    with pool_manager.get_session() as session:
        # 創建2個測試載具
        carrier1 = Carrier(
            id=99001,
            room_id=2,
            port_id=2101,  # Port 1
            status_id=303  # 清洗完成
        )
        carrier2 = Carrier(
            id=99002,
            room_id=2,
            port_id=2103,  # Port 3
            status_id=101  # 入口箱
        )
        session.add(carrier1)
        session.add(carrier2)
        session.commit()
        print("✅ 創建載具:")
        print(f"   Carrier {carrier1.id}: port_id={carrier1.port_id}, status_id={carrier1.status_id}")
        print(f"   Carrier {carrier2.id}: port_id={carrier2.port_id}, status_id={carrier2.status_id}")

    # 步驟 2: 查詢房間
    print("\n🔍 步驟 2: 查詢房間 (enable=1)...")
    rooms_result = db_bridge.query_rooms(enable=1)
    print(f"✅ 找到 {len(rooms_result['data'])} 個啟用的房間")
    for room in rooms_result['data']:
        print(f"   Room {room['id']}: {room['name']}")

    # 步驟 3: 查詢 Loader AGV
    print("\n🔍 步驟 3: 查詢 Loader AGV (model=LOADER, enable=1)...")
    agvs_result = db_bridge.query_agvs(model="LOADER", enable=1)
    print(f"✅ 找到 {len(agvs_result['data'])} 個 Loader AGV")
    for agv in agvs_result['data']:
        print(f"   AGV {agv['id']}: {agv['name']}, model={agv['model']}")

    # 步驟 4: 查詢 Port 1、3 的清洗完成載具
    print("\n🔍 步驟 4: 查詢 Port 1、3 的清洗完成載具 (port_in=[2101,2103], status_id=303)...")
    carriers_result = db_bridge.query_carriers(port_in=[2101, 2103], status_id=303)
    print(f"✅ 找到 {len(carriers_result['data'])} 個清洗完成載具")
    for carrier in carriers_result['data']:
        print(f"   Carrier {carrier['id']}: port_id={carrier['port_id']}, status_id={carrier['status_id']}")

    # 步驟 5: 檢查載具數量
    cleaner_carrier_count = len(carriers_result['data'])
    required_count = 1
    has_enough_carriers = cleaner_carrier_count >= required_count
    print(f"\n✅ 載具數量檢查:")
    print(f"   cleaner_carrier_count = {cleaner_carrier_count}")
    print(f"   required_count = {required_count}")
    print(f"   has_enough_carriers = {has_enough_carriers}")

    if not has_enough_carriers:
        print("❌ 載具數量不足，流程終止")
        return

    # 步驟 6: 查詢泡藥機空位（Station 1 為例）
    print("\n🔍 步驟 6: 查詢泡藥機 Station 1 空位 (equipment_id=204, port_in=[2041], status=empty)...")
    ports_result = db_bridge.query_eqp_ports(equipment_id=204, port_in=[2041], status="empty")
    print(f"✅ 找到 {len(ports_result['data'])} 個空位")
    for port in ports_result['data']:
        print(f"   Port {port['id']}: {port['name']}, status={port.get('status')}")

    # 步驟 7: 檢查空位數量
    empty_count = len(ports_result['data'])
    required_space = 1
    has_enough_space = empty_count >= required_space
    print(f"\n✅ 空位數量檢查:")
    print(f"   empty_count = {empty_count}")
    print(f"   required_space = {required_space}")
    print(f"   has_enough_space = {has_enough_space}")

    if not has_enough_space:
        print("❌ 空位數量不足，流程終止")
        return

    # 步驟 8: 檢查重複任務
    print("\n🔍 步驟 8: 檢查重複任務 (work_id=2040102, room_id=2, status_id_in=[0,1,2,3])...")
    tasks_result = db_bridge.query_tasks(work_id=2040102, room_id=2, status_id_in=[0,1,2,3])
    print(f"✅ 找到 {len(tasks_result['data'])} 個未完成任務")
    for task in tasks_result['data']:
        print(f"   Task {task['id']}: work_id={task['work_id']}, status_id={task['status_id']}")

    # 步驟 9: 決定是否創建任務
    if len(tasks_result['data']) == 0:
        print("\n✅ 沒有重複任務，可以創建新任務")
        print("🎯 結論: 所有條件都滿足，應該創建任務！")
    else:
        print("\n❌ 已存在未完成任務，不創建新任務")
        print("🎯 結論: 因重複任務檢查失敗，不會創建任務")

    # 清理測試資料
    print("\n🧹 清理測試載具...")
    with pool_manager.get_session() as session:
        from sqlmodel import delete
        session.exec(delete(Carrier).where(Carrier.id.in_([99001, 99002])))
        session.commit()
        print("✅ 清理完成")

    print("\n" + "="*60)

if __name__ == '__main__':
    main()
