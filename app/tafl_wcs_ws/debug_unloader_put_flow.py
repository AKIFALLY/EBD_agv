#!/usr/bin/env python3
"""調試 Unloader PUT 流程執行步驟"""

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
    print("調試 unloader_put_oven 流程步驟")
    print("="*60)

    TEST_AGV_ID = 9002  # 修正：與測試文件一致
    TEST_ROOM_ID = 992
    test_carrier_ids = [99001, 99002, 99003, 99004]

    # 步驟 0: 清理舊資料
    print("\n🧹 清理舊資料...")
    with pool_manager.get_session() as session:
        from sqlmodel import delete
        from db_proxy.models import AGV, Room, Task

        session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
        session.exec(delete(Carrier).where(Carrier.id.in_(test_carrier_ids)))
        session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
        session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
        session.commit()
    print("✅ 清理完成")

    # 步驟 1: 創建測試資料（Room, AGV, Carriers）
    print("\n📝 步驟 1: 創建測試資料...")
    with pool_manager.get_session() as session:
        from db_proxy.models import Room, AGV

        # 創建測試 Room
        room = Room(
            id=TEST_ROOM_ID,
            name="TEST_ROOM_UNLOADER",
            process_settings_id=1,
            enable=1
        )
        session.add(room)

        # 創建 Unloader AGV
        agv = AGV(
            id=TEST_AGV_ID,
            name="TEST_UNLOADER01",
            model="UNLOADER",
            x=0.0,
            y=0.0,
            heading=0.0,
            enable=1
        )
        session.add(agv)

        # 創建4個測試載具（在 AGV 車上）
        for carrier_id in test_carrier_ids:
            carrier = Carrier(
                id=carrier_id,
                room_id=TEST_ROOM_ID,
                agv_id=TEST_AGV_ID,  # 在 AGV 車上
                port_id=None,
                status_id=200  # 在運輸中
            )
            session.add(carrier)
        session.commit()
        print(f"✅ 測試資料創建完成:")
        print(f"   Room: {TEST_ROOM_ID} (enabled=1)")
        print(f"   AGV: TEST_UNLOADER01 (id={TEST_AGV_ID}, model=UNLOADER)")
        print(f"   Carriers: {len(test_carrier_ids)}個 (agv_id={TEST_AGV_ID}, port_id=None, status_id=200)")

    # 步驟 2: 查詢房間
    print("\n🔍 步驟 2: 查詢房間 (enable=1)...")
    rooms_result = db_bridge.query_rooms(enable=1)
    print(f"✅ 找到 {len(rooms_result['data'])} 個啟用的房間")
    for room in rooms_result['data'][:3]:
        print(f"   Room {room['id']}: {room['name']}")

    # 步驟 3: 查詢 Unloader AGV
    print("\n🔍 步驟 3: 查詢 Unloader AGV (model=UNLOADER, enable=1)...")
    agvs_result = db_bridge.query_agvs(model="UNLOADER", enable=1)
    print(f"✅ 找到 {len(agvs_result['data'])} 個 Unloader AGV")
    for agv in agvs_result['data']:
        print(f"   AGV {agv['id']}: {agv['name']}, model={agv['model']}")

    # 步驟 4: 查詢測試 AGV 車上載具
    print(f"\n🔍 步驟 4: 查詢測試 AGV 車上載具 (agv_id={TEST_AGV_ID})...")
    carriers_result = db_bridge.query_carriers(agv_id=TEST_AGV_ID)
    print(f"✅ 找到 {len(carriers_result['data'])} 個載具在測試 AGV 上")
    for carrier in carriers_result['data']:
        print(f"   Carrier {carrier['id']}: agv_id={carrier.get('agv_id')}, port_id={carrier.get('port_id')}, status_id={carrier.get('status_id')}")

    # 步驟 5: 檢查載具數量
    agv_carrier_count = len(carriers_result['data'])
    required_count = 4
    has_enough_carriers = agv_carrier_count >= required_count
    print(f"\n✅ 載具數量檢查:")
    print(f"   agv_carrier_count = {agv_carrier_count}")
    print(f"   required_count = {required_count}")
    print(f"   has_enough_carriers = {has_enough_carriers}")

    if not has_enough_carriers:
        print("❌ 載具數量不足，流程終止")
        cleanup(pool_manager, test_carrier_ids)
        return

    # 步驟 6: 查詢烤箱下排空位（Station 05）
    print("\n🔍 步驟 6: 查詢烤箱下排 Station 05 空位 (equipment_id=206, port_in=[2065,2066,2067,2068], status=empty)...")
    ports_result = db_bridge.query_eqp_ports(equipment_id=206, port_in=[2065, 2066, 2067, 2068], status="empty")
    print(f"✅ 找到 {len(ports_result['data'])} 個空位")
    for port in ports_result['data']:
        print(f"   Port {port['id']}: {port['name']}, status={port.get('status', 'N/A')}")

    # 步驟 7: 檢查空位數量
    empty_count = len(ports_result['data'])
    required_space = 4
    has_enough_space = empty_count >= required_space
    print(f"\n✅ 空位數量檢查:")
    print(f"   empty_count = {empty_count}")
    print(f"   required_space = {required_space}")
    print(f"   has_enough_space = {has_enough_space}")

    if not has_enough_space:
        print("❌ 空位數量不足，流程終止")
        cleanup(pool_manager, test_carrier_ids)
        return

    # 步驟 8: 檢查重複任務
    print("\n🔍 步驟 8: 檢查重複任務 (work_id=2060502, room_id=992, status_id_in=[0,1,2,3])...")
    tasks_result = db_bridge.query_tasks(work_id=2060502, room_id=992, status_id_in=[0,1,2,3])
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
    cleanup(pool_manager, test_carrier_ids, TEST_AGV_ID, TEST_ROOM_ID)

    print("\n" + "="*60)

def cleanup(pool_manager, test_carrier_ids, test_agv_id, test_room_id):
    """清理測試資料"""
    print("\n🧹 清理測試資料...")
    with pool_manager.get_session() as session:
        from sqlmodel import delete
        from db_proxy.models import AGV, Room, Task

        # 清理順序：Task → Carrier → AGV → Room
        session.exec(delete(Task).where(Task.room_id == test_room_id))
        session.exec(delete(Carrier).where(Carrier.id.in_(test_carrier_ids)))
        session.exec(delete(AGV).where(AGV.id == test_agv_id))
        session.exec(delete(Room).where(Room.id == test_room_id))
        session.commit()
        print("✅ 清理完成")

if __name__ == '__main__':
    main()
