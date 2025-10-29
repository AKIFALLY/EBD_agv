#!/usr/bin/env python3
"""測試 Loader AGV 流程 - 使用實際設備 ID"""

import sys
import asyncio

sys.path.insert(0, '/app/tafl_wcs_ws/src')
sys.path.insert(0, '/app/db_proxy_ws/src')
sys.path.insert(0, '/app/tafl_ws/src')

from tafl_wcs.tafl_executor_wrapper import TAFLExecutorWrapper
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import AGV, Carrier, Task, Room
from sqlmodel import select, delete

# 測試用 ID 範圍（只用於 Room, AGV, Carrier）
TEST_ROOM_ID = 991
TEST_AGV_ID = 9001
TEST_CARRIER_BASE = 90000

# 實際設備和端口 ID
REAL_EQUIPMENT = {
    'boxin': {
        'eqp_id': 201,
        'ports': [2011, 2012]  # 只用前2個端口測試
    },
    'cleaner': {
        'eqp_id': 203,
        'ports_lower': [2033, 2034],  # 下層PUT
        'ports_upper': [2031, 2032]   # 上層TAKE
    },
    'soaker': {
        'eqp_id': 204,
        'ports': [2041, 2042, 2043, 2044, 2045, 2046]  # 6個泡藥機
    },
    'dryer': {
        'eqp_id': 205,
        'ports': [2051, 2052, 2053, 2054, 2055, 2056, 2057, 2058]  # 8個端口
    }
}


async def test_1_loader_take_boxin_transfer():
    """測試 1: Loader AGV 從入口傳送箱取料（使用實際設備）"""
    print("\n" + "="*60)
    print("測試 1: Loader AGV 從入口傳送箱取料（使用實際設備 ID 201）")
    print("="*60)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    real_equipment_id = REAL_EQUIPMENT['boxin']['eqp_id']
    real_port_ids = REAL_EQUIPMENT['boxin']['ports']
    test_carrier_ids = [TEST_CARRIER_BASE + 1, TEST_CARRIER_BASE + 2]

    try:
        # 清理舊測試資料（不刪除設備）
        print("\n🧹 清理舊測試資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
            session.exec(delete(Carrier).where(Carrier.id.in_(test_carrier_ids)))
            session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
            session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
            session.commit()
        print("✅ 清理完成")

        # 創建測試資料（使用實際設備）
        print("\n📝 創建測試資料（使用實際設備 ID 201）...")
        with pool_manager.get_session() as session:
            # 創建測試 Room
            room = Room(
                id=TEST_ROOM_ID,
                name="TEST_ROOM_LOADER",
                process_settings_id=1,
                enable=1
            )
            session.add(room)

            # 創建 Loader AGV
            agv = AGV(
                id=TEST_AGV_ID,
                name="TEST_LOADER01",
                model="Loader",
                x=0.0,
                y=0.0,
                heading=0.0,
                enable=1
            )
            session.add(agv)

            # 創建 Carriers（在實際設備端口上）
            for i, carrier_id in enumerate(test_carrier_ids):
                carrier = Carrier(
                    id=carrier_id,
                    room_id=TEST_ROOM_ID,
                    port_id=real_port_ids[i],  # 使用實際端口 ID
                    status_id=201  # 在入口傳送箱
                )
                session.add(carrier)

            session.commit()
            print("✅ 測試資料創建完成")
            print(f"   Room: {TEST_ROOM_ID} (enabled=1)")
            print(f"   AGV: TEST_LOADER01 (model=LOADER, enable=1)")
            print(f"   Real Equipment: {real_equipment_id} (BoxIn)")
            print(f"   Real Ports: {real_port_ids}")
            print(f"   Carriers: 2個 (status=201, 在實際端口上)")
            print(f"   現有 Tasks: 0個")

        # 執行流程
        print("\n🚀 執行流程: loader_take_boxin_transfer.yaml")
        with open('/app/config/tafl/flows/loader_take_boxin_transfer.yaml', 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        result = await flow_executor.execute_flow(flow_yaml)
        print(f"📊 執行結果: {result.get('status')}")

        # 驗證任務創建
        print("\n✅ 測試後狀態:")
        with pool_manager.get_session() as session:
            tasks = session.exec(
                select(Task).where(Task.room_id == TEST_ROOM_ID)
            ).all()

            if tasks:
                task = tasks[0]
                print(f"  - 創建 Task: {len(tasks)}個")
                print(f"    * work_id: {task.work_id}")
                print(f"    * name: {task.name}")
                if task.parameters:
                    print(f"    * equipment_id: {task.parameters.get('equipment_id')}")
                    print(f"    * station: {task.parameters.get('station')}")
                    print(f"    * ports: {task.parameters.get('ports')}")
                    print(f"    * model: {task.parameters.get('model')}")

                # 驗證 work_id
                expected_work_ids = [2010101, 2010301]
                if task.work_id in expected_work_ids:
                    print(f"  - 目的地驗證: ✅ 正確 (入口傳送箱)")
                    return True
                else:
                    print(f"  - 目的地驗證: ❌ 錯誤 (work_id={task.work_id}, 應為 {expected_work_ids})")
                    return False
            else:
                print("  - ❌ 未創建任務")
                return False

    finally:
        # 清理（只刪除測試資料，不刪除實際設備）
        print("\n🧹 清理測試資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
            session.exec(delete(Carrier).where(Carrier.id.in_(test_carrier_ids)))
            session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
            session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
            session.commit()
        print("✅ 清理完成")
        pool_manager.shutdown()


async def test_2_loader_put_cleaner():
    """測試 2: Loader AGV 放料到清洗機（使用實際設備）"""
    print("\n" + "="*60)
    print("測試 2: Loader AGV 放料到清洗機（使用實際設備 ID 203）")
    print("="*60)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    real_equipment_id = REAL_EQUIPMENT['cleaner']['eqp_id']
    real_port_ids = REAL_EQUIPMENT['cleaner']['ports_lower']  # 下層PUT
    test_carrier_id = TEST_CARRIER_BASE + 3

    try:
        # 清理舊測試資料
        print("\n🧹 清理舊測試資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
            session.exec(delete(Carrier).where(Carrier.id == test_carrier_id))
            session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
            session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
            session.commit()
        print("✅ 清理完成")

        # 創建測試資料
        print("\n📝 創建測試資料（使用實際設備 ID 203）...")
        with pool_manager.get_session() as session:
            # 創建測試 Room
            room = Room(
                id=TEST_ROOM_ID,
                name="TEST_ROOM_LOADER",
                process_settings_id=1,
                enable=1
            )
            session.add(room)

            # 創建 Loader AGV
            agv = AGV(
                id=TEST_AGV_ID,
                name="TEST_LOADER01",
                model="Loader",
                x=0.0,
                y=0.0,
                heading=0.0,
                enable=1
            )
            session.add(agv)

            # 創建 Carrier（在 AGV 車上）
            carrier = Carrier(
                id=test_carrier_id,
                room_id=TEST_ROOM_ID,
                agv_id=TEST_AGV_ID,  # 在 AGV 車上
                port_id=None,
                status_id=200  # 在運輸中
            )
            session.add(carrier)

            session.commit()
            print("✅ 測試資料創建完成")
            print(f"   Room: {TEST_ROOM_ID} (enabled=1)")
            print(f"   AGV: TEST_LOADER01 (model=LOADER, enable=1, 車上載具: 1個)")
            print(f"   Real Equipment: {real_equipment_id} (Cleaner)")
            print(f"   Real Ports (lower): {real_port_ids} (空的)")
            print(f"   現有 Tasks: 0個")

        # 執行流程
        print("\n🚀 執行流程: loader_put_cleaner.yaml")
        with open('/app/config/tafl/flows/loader_put_cleaner.yaml', 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        result = await flow_executor.execute_flow(flow_yaml)
        print(f"📊 執行結果: {result.get('status')}")

        # 驗證任務創建
        print("\n✅ 測試後狀態:")
        with pool_manager.get_session() as session:
            tasks = session.exec(
                select(Task).where(Task.room_id == TEST_ROOM_ID)
            ).all()

            if tasks:
                task = tasks[0]
                print(f"  - 創建 Task: {len(tasks)}個")
                print(f"    * work_id: {task.work_id}")
                print(f"    * name: {task.name}")
                if task.parameters:
                    print(f"    * equipment_id: {task.parameters.get('equipment_id')}")
                    print(f"    * station: {task.parameters.get('station')}")
                    print(f"    * ports: {task.parameters.get('ports')}")
                    print(f"    * row: {task.parameters.get('row')}")

                # 驗證 work_id
                if task.work_id == 2030302:
                    print(f"  - 目的地驗證: ✅ 正確 (清洗機 Station 03 下層)")
                    return True
                else:
                    print(f"  - 目的地驗證: ❌ 錯誤 (work_id={task.work_id}, 應為 2030302)")
                    return False
            else:
                print("  - ❌ 未創建任務")
                return False

    finally:
        # 清理
        print("\n🧹 清理測試資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
            session.exec(delete(Carrier).where(Carrier.id == test_carrier_id))
            session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
            session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
            session.commit()
        print("✅ 清理完成")
        pool_manager.shutdown()


async def main():
    """執行測試"""
    print("\n" + "="*60)
    print("Loader AGV 流程測試（使用實際設備 ID）")
    print("="*60)

    results = []

    # 測試 1: 入口傳送箱取料
    results.append(("Loader 入口傳送箱取料", await test_1_loader_take_boxin_transfer()))

    # 測試 2: 放料到清洗機
    results.append(("Loader 放料到清洗機", await test_2_loader_put_cleaner()))

    # 總結
    print("\n" + "="*60)
    print("測試總結")
    print("="*60)
    for name, result in results:
        status = "✅ 通過" if result else "❌ 失敗"
        print(f"{status} - {name}")

    all_passed = all(r for _, r in results)
    if all_passed:
        print("\n🎉 所有測試通過！")
        return 0
    else:
        print("\n⚠️ 部分測試失敗")
        return 1


if __name__ == '__main__':
    exit_code = asyncio.run(main())
    sys.exit(exit_code)
