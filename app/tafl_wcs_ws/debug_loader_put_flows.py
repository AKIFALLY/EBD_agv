#!/usr/bin/env python3
"""診斷 Loader PUT 流程失敗原因"""

import sys
sys.path.insert(0, '/app/tafl_wcs_ws/src')
sys.path.insert(0, '/app/db_proxy_ws/src')

from tafl_wcs.tafl_db_bridge import TAFLDatabaseBridge
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import AGV, Carrier, Task, Room

def main():
    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    db_bridge = TAFLDatabaseBridge(db_url)
    pool_manager = ConnectionPoolManager(db_url)

    TEST_AGV_ID = 9001
    TEST_ROOM_ID = 991
    TEST_CARRIER_ID = 90003

    print("="*60)
    print("診斷 Loader PUT 流程失敗原因")
    print("="*60)

    # 清理並創建測試資料（測試 2 PUT 清洗機）
    print("\n📝 創建測試環境 (測試 2: PUT 清洗機)...")
    with pool_manager.get_session() as session:
        from sqlmodel import delete

        session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
        session.exec(delete(Carrier).where(Carrier.id == TEST_CARRIER_ID))
        session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
        session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
        session.commit()

        # 創建 Room
        room = Room(
            id=TEST_ROOM_ID,
            name="TEST_ROOM_LOADER",
            process_settings_id=1,
            enable=1
        )
        session.add(room)

        # 創建 AGV
        agv = AGV(
            id=TEST_AGV_ID,
            name="TEST_LOADER01",
            model="LOADER",
            x=0.0,
            y=0.0,
            heading=0.0,
            enable=1
        )
        session.add(agv)

        # 創建 Carrier (在 AGV 車上，關鍵：包含 agv_id)
        carrier = Carrier(
            id=TEST_CARRIER_ID,
            room_id=TEST_ROOM_ID,
            agv_id=TEST_AGV_ID,  # 在 AGV 車上
            port_id=None,
            status_id=200  # 在運輸中
        )
        session.add(carrier)
        session.commit()

    print("✅ 測試環境創建完成")
    print(f"   Room: {TEST_ROOM_ID} (enabled=1)")
    print(f"   AGV: {TEST_AGV_ID} (model=LOADER)")
    print(f"   Carrier: {TEST_CARRIER_ID} (agv_id={TEST_AGV_ID}, status_id=200)")

    # 步驟 1: 檢查 Loader AGV
    print("\n🔍 步驟 1: 查詢 Loader AGV (model=LOADER, enable=1)...")
    agvs_result = db_bridge.query_agvs(model="LOADER", enable=1)
    print(f"✅ 找到 {len(agvs_result['data'])} 個 Loader AGV")
    for agv in agvs_result['data']:
        print(f"   AGV {agv['id']}: {agv['name']}, model={agv['model']}")

    # 步驟 2: 查詢測試 AGV 車上載具
    print(f"\n🔍 步驟 2: 查詢測試 AGV 車上載具 (agv_id={TEST_AGV_ID})...")
    carriers_result = db_bridge.query_carriers(agv_id=TEST_AGV_ID)
    print(f"✅ 找到 {len(carriers_result['data'])} 個載具在測試 AGV 上")
    for carrier in carriers_result['data']:
        print(f"   Carrier {carrier['id']}: agv_id={carrier.get('agv_id')}, port_id={carrier.get('port_id')}, status_id={carrier.get('status_id')}")

    # 步驟 3: 檢查載具數量
    agv_carrier_count = len(carriers_result['data'])
    required_count = 1
    has_enough_carriers = agv_carrier_count >= required_count
    print(f"\n✅ 載具數量檢查:")
    print(f"   agv_carrier_count = {agv_carrier_count}")
    print(f"   required_count = {required_count}")
    print(f"   has_enough_carriers = {has_enough_carriers}")

    if not has_enough_carriers:
        print("❌ 載具數量不足，這是問題原因！")
        cleanup(pool_manager)
        return

    # 步驟 4: 查詢清洗機下層空位
    print("\n🔍 步驟 4: 查詢清洗機下層空位 (equipment_id=203, port_in=[2033,2034], status=empty)...")
    ports_result = db_bridge.query_eqp_ports(equipment_id=203, port_in=[2033, 2034], status="empty")
    print(f"✅ 找到 {len(ports_result['data'])} 個空位")
    for port in ports_result['data']:
        print(f"   Port {port['id']}: {port['name']}, status={port.get('status', 'N/A')}")

    # 步驟 5: 檢查空位數量
    empty_count = len(ports_result['data'])
    required_space = 1
    has_enough_space = empty_count >= required_space
    print(f"\n✅ 空位數量檢查:")
    print(f"   empty_count = {empty_count}")
    print(f"   required_space = {required_space}")
    print(f"   has_enough_space = {has_enough_space}")

    if not has_enough_space:
        print("❌ 空位數量不足，這是 PUT 操作失敗的主要原因！")
        print("\n💡 建議: 檢查生產環境的 Port 狀態")
        print("   Port 2033, 2034 可能不是空的")
        cleanup(pool_manager)
        return

    # 步驟 6: 檢查重複任務
    print("\n🔍 步驟 6: 檢查重複任務 (work_id=2030302, room_id=991, status_id_in=[0,1,2,3])...")
    tasks_result = db_bridge.query_tasks(work_id=2030302, room_id=991, status_id_in=[0,1,2,3])
    print(f"✅ 找到 {len(tasks_result['data'])} 個未完成任務")
    for task in tasks_result['data']:
        print(f"   Task {task['id']}: work_id={task['work_id']}, status_id={task['status_id']}")

    # 結論
    if len(tasks_result['data']) == 0:
        print("\n✅ 沒有重複任務")
        print("🎯 所有條件都滿足，應該創建任務！")
        print("   如果測試仍失敗，問題可能在 TAFL 流程邏輯")
    else:
        print("\n❌ 已存在未完成任務")
        print("🎯 這是不創建任務的原因")

    cleanup(pool_manager)
    print("\n" + "="*60)

def cleanup(pool_manager):
    """清理測試資料"""
    print("\n🧹 清理測試資料...")
    with pool_manager.get_session() as session:
        from sqlmodel import delete
        from db_proxy.models import AGV, Room, Task, Carrier

        TEST_ROOM_ID = 991
        TEST_AGV_ID = 9001
        TEST_CARRIER_ID = 90003

        session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
        session.exec(delete(Carrier).where(Carrier.id == TEST_CARRIER_ID))
        session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
        session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
        session.commit()
        print("✅ 清理完成")

if __name__ == '__main__':
    main()
