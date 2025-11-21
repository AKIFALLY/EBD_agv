#!/usr/bin/env python3
"""測試完成料架出口到人工收料區流程"""

import sys
import asyncio

sys.path.insert(0, '/app/tafl_wcs_ws/src')
sys.path.insert(0, '/app/db_proxy_ws/src')
sys.path.insert(0, '/app/tafl_ws/src')

from tafl_wcs.tafl_executor_wrapper import TAFLExecutorWrapper
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import Rack, Location, Task, Carrier, Product
from sqlmodel import select, delete
from sqlalchemy import update

async def test_full_rack_to_collection():
    """測試場景1: 滿載rack應搬到人工收料區"""
    print("\n" + "="*60)
    print("測試 1: 滿載rack到人工收料區")
    print("="*60)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    test_rack_id = 701
    outlet_id = 10002  # 房間1出口
    collection_location_id = 21  # 人工收料區第一位

    try:
        # 清理舊資料
        print("\n🧹 清理舊資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.rack_id == test_rack_id))
            session.exec(delete(Carrier).where(Carrier.rack_id == test_rack_id))
            session.exec(delete(Rack).where(Rack.id == test_rack_id))
            session.exec(
                update(Location)
                .where(Location.id.in_([outlet_id, collection_location_id]))
                .values(rack_id=None, location_status_id=2)
            )
            session.commit()
        print("✅ 清理完成")

        # 創建測試資料
        print("\n📝 創建測試資料...")
        with pool_manager.get_session() as session:
            # 查詢產品資訊（使用已有的產品）
            product = session.exec(select(Product).where(Product.id == 1)).first()
            max_per_side = 16 if product.size == 'S' else 8
            max_total = max_per_side * 2

            # 創建滿載rack在出口
            rack = Rack(
                id=test_rack_id,
                name="TEST_FULL_RACK",
                location_id=outlet_id,
                status_id=1,
                product_id=1,
                direction=0,
                is_carry=0
            )
            session.add(rack)

            # 創建滿載的carriers（A面+B面共32個，適用於S尺寸）
            for i in range(1, max_total + 1):
                carrier = Carrier(
                    rack_id=test_rack_id,
                    rack_index=i,
                    room_id=1,
                    port_id=1000 + i,
                    status_id=8  # 已完成
                )
                session.add(carrier)

            # 設定出口佔用
            session.exec(
                update(Location)
                .where(Location.id == outlet_id)
                .values(location_status_id=3, rack_id=test_rack_id)
            )

            # 設定收料區空閒
            session.exec(
                update(Location)
                .where(Location.id == collection_location_id)
                .values(location_status_id=2, rack_id=None)
            )
            session.commit()
            print(f"✅ 測試資料創建完成（rack滿載{max_total}個carrier）")

        # 執行流程
        print("\n🚀 執行流程...")
        with open('/app/config/tafl/flows/full_rack_outlet_to_manual_collection.yaml', 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        result = await flow_executor.execute_flow(flow_yaml)
        print(f"📊 執行結果: {result.get('status')}")

        # 驗證任務創建
        with pool_manager.get_session() as session:
            tasks = session.exec(
                select(Task).where(
                    Task.rack_id == test_rack_id,
                    Task.work_id == 220001
                )
            ).all()

            if tasks:
                task = tasks[0]
                print(f"✅ 成功創建任務")
                print(f"   任務 ID: {task.id}")
                print(f"   任務名稱: {task.name}")
                if task.parameters:
                    to_loc_name = task.parameters.get('to_location_name')
                    to_loc_id = task.parameters.get('to_location_id')
                    print(f"   目的地: {to_loc_name}")
                    print(f"   原因: {task.parameters.get('reason')}")
                    print(f"   載具數: {task.parameters.get('carrier_count')}")
                    print(f"   是否滿載: {task.parameters.get('is_full')}")
                    # 驗證目的地是否在人工收料區 (21-22)
                    if to_loc_id and 21 <= to_loc_id <= 22:
                        print(f"   ✅ 正確路由到人工收料區")
                        return True
                    elif to_loc_name and 'ManualCollectionArea' in to_loc_name:
                        print(f"   ✅ 正確路由到人工收料區")
                        return True
                    else:
                        print(f"   ❌ 錯誤：目的地不是人工收料區")
                        return False
                return True
            else:
                print("❌ 未創建任務")
                return False

    finally:
        # 清理
        print("\n🧹 清理測試資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.rack_id == test_rack_id))
            session.exec(delete(Carrier).where(Carrier.rack_id == test_rack_id))
            session.exec(
                update(Location)
                .where(Location.id.in_([outlet_id, collection_location_id]))
                .values(rack_id=None, location_status_id=2)
            )
            session.exec(delete(Rack).where(Rack.id == test_rack_id))
            session.commit()
        print("✅ 清理完成")

async def test_last_batch_to_collection():
    """測試場景2: 尾批rack（房間無可放carrier）應搬到人工收料區"""
    print("\n" + "="*60)
    print("測試 2: 尾批rack到人工收料區")
    print("="*60)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    test_rack_id = 702
    outlet_id = 10002
    collection_location_id = 21

    try:
        # 清理舊資料
        print("\n🧹 清理舊資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.rack_id == test_rack_id))
            session.exec(delete(Carrier).where(Carrier.rack_id == test_rack_id))
            session.exec(delete(Rack).where(Rack.id == test_rack_id))
            session.exec(
                update(Location)
                .where(Location.id.in_([outlet_id, collection_location_id]))
                .values(rack_id=None, location_status_id=2)
            )
            session.commit()
        print("✅ 清理完成")

        # 創建測試資料
        print("\n📝 創建測試資料...")
        with pool_manager.get_session() as session:
            # 創建尾批rack在出口（只有10個carrier）
            rack = Rack(
                id=test_rack_id,
                name="TEST_LAST_BATCH_RACK",
                location_id=outlet_id,
                status_id=1,
                product_id=1,
                direction=0,
                is_carry=0
            )
            session.add(rack)

            # 創建10個carriers（未滿載，但是尾批）
            for i in range(1, 11):
                carrier = Carrier(
                    rack_id=test_rack_id,
                    rack_index=i,
                    room_id=1,
                    port_id=2000 + i,
                    status_id=8  # 已完成
                )
                session.add(carrier)

            # 設定出口佔用
            session.exec(
                update(Location)
                .where(Location.id == outlet_id)
                .values(location_status_id=3, rack_id=test_rack_id)
            )

            # 設定收料區空閒
            session.exec(
                update(Location)
                .where(Location.id == collection_location_id)
                .values(location_status_id=2, rack_id=None)
            )

            # 關鍵：房間內無已完成的可放carrier（is_last_batch=true的條件）
            # 刪除房間1內所有rack_id為null且status_id=8的carrier
            session.exec(
                delete(Carrier).where(
                    Carrier.room_id == 1,
                    Carrier.rack_id == None,
                    Carrier.status_id == 8
                )
            )

            session.commit()
            print("✅ 測試資料創建完成（rack有10個carrier，房間無可放carrier=尾批）")

        # 執行流程
        print("\n🚀 執行流程...")
        with open('/app/config/tafl/flows/full_rack_outlet_to_manual_collection.yaml', 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        result = await flow_executor.execute_flow(flow_yaml)
        print(f"📊 執行結果: {result.get('status')}")

        # 驗證任務創建
        with pool_manager.get_session() as session:
            tasks = session.exec(
                select(Task).where(
                    Task.rack_id == test_rack_id,
                    Task.work_id == 220001
                )
            ).all()

            if tasks:
                task = tasks[0]
                print(f"✅ 成功創建任務")
                print(f"   任務 ID: {task.id}")
                print(f"   任務名稱: {task.name}")
                if task.parameters:
                    to_loc_name = task.parameters.get('to_location_name')
                    to_loc_id = task.parameters.get('to_location_id')
                    print(f"   目的地: {to_loc_name}")
                    print(f"   原因: {task.parameters.get('reason')}")
                    print(f"   載具數: {task.parameters.get('carrier_count')}")
                    print(f"   是否尾批: {task.parameters.get('is_last_batch')}")
                    # 驗證目的地是否在人工收料區 (21-22)
                    if to_loc_id and 21 <= to_loc_id <= 22:
                        print(f"   ✅ 正確路由到人工收料區")
                        return True
                    elif to_loc_name and 'ManualCollectionArea' in to_loc_name:
                        print(f"   ✅ 正確路由到人工收料區")
                        return True
                    else:
                        print(f"   ❌ 錯誤：目的地不是人工收料區")
                        return False
                return True
            else:
                print("❌ 未創建任務")
                return False

    finally:
        # 清理
        print("\n🧹 清理測試資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.rack_id == test_rack_id))
            session.exec(delete(Carrier).where(Carrier.rack_id == test_rack_id))
            session.exec(
                update(Location)
                .where(Location.id.in_([outlet_id, collection_location_id]))
                .values(rack_id=None, location_status_id=2)
            )
            session.exec(delete(Rack).where(Rack.id == test_rack_id))
            session.commit()
        print("✅ 清理完成")

async def main():
    """執行所有測試"""
    print("\n" + "="*60)
    print("完成料架出口→人工收料區流程測試")
    print("="*60)

    results = []

    # 測試 1: 滿載rack
    results.append(("滿載rack到收料區", await test_full_rack_to_collection()))

    # 測試 2: 尾批rack
    results.append(("尾批rack到收料區", await test_last_batch_to_collection()))

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