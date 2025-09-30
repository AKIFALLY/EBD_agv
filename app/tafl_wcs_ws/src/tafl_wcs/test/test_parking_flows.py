#!/usr/bin/env python3
"""測試空料架停車區流程"""

import sys
import asyncio

sys.path.insert(0, '/app/tafl_wcs_ws/src')
sys.path.insert(0, '/app/db_proxy_ws/src')
sys.path.insert(0, '/app/tafl_ws/src')

from tafl_wcs.tafl_executor_wrapper import TAFLExecutorWrapper
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import Rack, Carrier, Location, Task
from sqlmodel import select, delete
from sqlalchemy import update

async def test_inlet_to_outlet():
    """測試場景1: 入口→出口（出口空閒）"""
    print("\n" + "="*60)
    print("測試 1: 空料架從入口到出口（出口空閒）")
    print("="*60)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    rack_id = 501
    inlet_id = 10001  # 房間1入口
    outlet_id = 10002  # 房間1出口

    try:
        # 創建測試資料
        print("\n📝 創建測試資料...")
        with pool_manager.get_session() as session:
            # 創建空rack在入口
            rack = Rack(
                id=rack_id,
                name="TEST_EMPTY_RACK",
                location_id=inlet_id,
                status_id=1,
                product_id=1,
                direction=0,
                is_carry=0,
                is_docked=1
            )
            session.add(rack)

            # 設定入口佔用
            session.exec(
                update(Location)
                .where(Location.id == inlet_id)
                .values(location_status_id=3, rack_id=rack_id)
            )

            # 確保出口空閒
            session.exec(
                update(Location)
                .where(Location.id == outlet_id)
                .values(location_status_id=2, rack_id=None)
            )
            session.commit()
            print("✅ 測試資料創建完成（空rack在入口，出口空閒）")

        # 執行流程
        print("\n🚀 執行入口→出口流程...")
        with open('/app/config/tafl/flows/empty_rack_inlet_to_outlet.yaml', 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        result = await flow_executor.execute_flow(flow_yaml)

        print(f"\n📊 執行結果: {result.get('status')}")

        # 驗證任務創建
        with pool_manager.get_session() as session:
            tasks = session.exec(
                select(Task).where(Task.rack_id == rack_id)
            ).all()

            if tasks:
                print(f"✅ 成功創建 {len(tasks)} 個任務")
                for task in tasks:
                    print(f"   任務 {task.id}: {task.name}")
                    if task.parameters:
                        to_loc = task.parameters.get('to_location_name')
                        print(f"   目的地: {to_loc}")
                        # 驗證目的地是否正確（應該是房間出口）
                        if 'Unloader' in to_loc or 'outlet' in to_loc.lower():
                            print(f"   ✅ 正確路由到出口")
                            return True
                        else:
                            print(f"   ❌ 錯誤：目的地不是出口")
                            return False
                return True
            else:
                print("❌ 未創建任務")
                return False

    finally:
        # 清理
        print("\n🧹 清理測試資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.rack_id == rack_id))
            session.exec(
                update(Location)
                .where(Location.id.in_([inlet_id, outlet_id]))
                .values(rack_id=None, location_status_id=2)
            )
            session.exec(delete(Rack).where(Rack.id == rack_id))
            session.commit()
        print("✅ 清理完成")

async def test_inlet_to_parking():
    """測試場景2: 入口→停車區（出口佔用）"""
    print("\n" + "="*60)
    print("測試 2: 空料架從入口到停車區（出口佔用）")
    print("="*60)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    rack_id = 502
    blocking_rack_id = 503
    inlet_id = 10001
    outlet_id = 10002

    try:
        # 創建測試資料
        print("\n📝 創建測試資料...")
        with pool_manager.get_session() as session:
            # 創建空rack在入口
            rack = Rack(
                id=rack_id,
                name="TEST_EMPTY_RACK2",
                location_id=inlet_id,
                status_id=1,
                product_id=1,
                direction=0,
                is_carry=0,
                is_docked=1
            )
            session.add(rack)

            # 創建阻塞rack在出口
            blocking_rack = Rack(
                id=blocking_rack_id,
                name="BLOCKING_RACK",
                location_id=outlet_id,
                status_id=1,
                product_id=1,
                direction=0,
                is_carry=0,
                is_docked=1
            )
            session.add(blocking_rack)

            # 設定入口佔用
            session.exec(
                update(Location)
                .where(Location.id == inlet_id)
                .values(location_status_id=3, rack_id=rack_id)
            )

            # 設定出口佔用
            session.exec(
                update(Location)
                .where(Location.id == outlet_id)
                .values(location_status_id=3, rack_id=blocking_rack_id)
            )
            session.commit()
            print("✅ 測試資料創建完成（空rack在入口，出口被佔用）")

        # 執行流程
        print("\n🚀 執行入口→停車區流程...")
        with open('/app/config/tafl/flows/empty_rack_inlet_to_parking.yaml', 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        result = await flow_executor.execute_flow(flow_yaml)

        print(f"\n📊 執行結果: {result.get('status')}")

        # 驗證任務創建
        with pool_manager.get_session() as session:
            tasks = session.exec(
                select(Task).where(Task.rack_id == rack_id)
            ).all()

            if tasks:
                print(f"✅ 成功創建 {len(tasks)} 個任務")
                for task in tasks:
                    print(f"   任務 {task.id}: {task.name}")
                    if task.parameters:
                        to_loc = task.parameters.get('to_location_name')
                        print(f"   目的地: {to_loc}")
                        if 'SystemEmptyRackArea' in to_loc:
                            print(f"   ✅ 正確路由到停車區")
                            return True
                        else:
                            print(f"   ❌ 錯誤：未路由到停車區")
                            return False
            else:
                print("❌ 未創建任務")
                return False

    finally:
        # 清理
        print("\n🧹 清理測試資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.rack_id.in_([rack_id, blocking_rack_id])))
            session.exec(
                update(Location)
                .where(Location.id.in_([inlet_id, outlet_id, 31, 32, 33, 34]))
                .values(rack_id=None, location_status_id=2)
            )
            session.exec(delete(Rack).where(Rack.id.in_([rack_id, blocking_rack_id])))
            session.commit()
        print("✅ 清理完成")

async def test_parking_to_outlet():
    """測試場景3: 停車區→出口（房間有carrier，出口缺rack）"""
    print("\n" + "="*60)
    print("測試 3: 停車區空料架到出口（房間有carrier等待）")
    print("="*60)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    rack_id = 504
    parking_location_id = 31  # 停車區第1位
    outlet_id = 10002

    try:
        # 創建測試資料
        print("\n📝 創建測試資料...")
        with pool_manager.get_session() as session:
            # 創建空rack在停車區
            rack = Rack(
                id=rack_id,
                name="TEST_PARKING_RACK",
                location_id=parking_location_id,
                status_id=1,
                product_id=1,
                direction=0,
                is_carry=0,
                is_docked=1
            )
            session.add(rack)

            # 創建已完成的carrier在房間1（未放rack）
            carrier = Carrier(
                room_id=1,
                rack_id=None,  # 未放rack
                rack_index=None,
                port_id=9999,
                status_id=8  # 已完成
            )
            session.add(carrier)

            # 設定停車位佔用
            session.exec(
                update(Location)
                .where(Location.id == parking_location_id)
                .values(location_status_id=3, rack_id=rack_id)
            )

            # 設定出口空閒（無rack）
            session.exec(
                update(Location)
                .where(Location.id == outlet_id)
                .values(location_status_id=2, rack_id=None)
            )
            session.commit()
            print("✅ 測試資料創建完成（空rack在停車區，房間有carrier等待，出口空閒）")

        # 執行流程
        print("\n🚀 執行停車區→出口流程...")
        with open('/app/config/tafl/flows/parking_to_outlet.yaml', 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        result = await flow_executor.execute_flow(flow_yaml)

        print(f"\n📊 執行結果: {result.get('status')}")

        # 驗證任務創建 - 檢查任何停車區到出口的任務（不限定特定rack_id）
        with pool_manager.get_session() as session:
            # 查詢所有停車區rack（只檢查未被搬運）
            parking_racks = session.exec(
                select(Rack).where(
                    Rack.location_id.in_([31, 32, 33, 34]),
                    Rack.is_carry == 0
                )
            ).all()

            parking_rack_ids = [r.id for r in parking_racks]

            # 查詢這些rack的最新任務
            tasks = session.exec(
                select(Task).where(
                    Task.rack_id.in_(parking_rack_ids),
                    Task.work_id == 220001,
                    Task.status_id == 1  # PENDING
                )
            ).all()

            if tasks:
                print(f"✅ 成功創建 {len(tasks)} 個任務")
                for task in tasks:
                    print(f"   任務 {task.id}: {task.name} (rack_id={task.rack_id})")
                    if task.parameters:
                        from_loc = task.parameters.get('from_location_name')
                        to_loc = task.parameters.get('to_location_name')
                        print(f"   路徑: {from_loc} → {to_loc}")
                        if 'SystemEmptyRackArea' in str(from_loc) and 'Unloader' in str(to_loc):
                            print(f"   ✅ 正確路由：停車區→出口")
                            return True
                print("❌ 任務路徑不正確")
                return False
            else:
                print("❌ 未創建任務")
                return False

    finally:
        # 清理
        print("\n🧹 清理測試資料...")
        with pool_manager.get_session() as session:
            # 清理測試創建的任務和數據
            session.exec(delete(Task).where(Task.work_id == 220001, Task.status_id == 1))
            session.exec(delete(Carrier).where(Carrier.port_id == 9999))
            session.exec(
                update(Location)
                .where(Location.id.in_([parking_location_id, outlet_id]))
                .values(rack_id=None, location_status_id=2)
            )
            session.exec(delete(Rack).where(Rack.id == rack_id))
            session.commit()
        print("✅ 清理完成")

async def main():
    """執行所有測試"""
    print("\n" + "="*60)
    print("空料架停車區流程完整測試")
    print("="*60)

    results = []

    # 測試 1: 入口→出口
    results.append(("入口→出口", await test_inlet_to_outlet()))

    # 測試 2: 入口→停車區
    results.append(("入口→停車區", await test_inlet_to_parking()))

    # 測試 3: 停車區→出口
    results.append(("停車區→出口", await test_parking_to_outlet()))

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