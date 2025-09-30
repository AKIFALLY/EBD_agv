#!/usr/bin/env python3
"""測試房間投料調度流程"""

import sys
import asyncio

sys.path.insert(0, '/app/tafl_wcs_ws/src')
sys.path.insert(0, '/app/db_proxy_ws/src')
sys.path.insert(0, '/app/tafl_ws/src')

from tafl_wcs.tafl_executor_wrapper import TAFLExecutorWrapper
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import Rack, Location, Task, Room, Product
from sqlmodel import select, delete
from sqlalchemy import update

async def test_room_dispatch_success():
    """測試場景1: 準備區有已派車料架，房間入口空閒，應創建任務"""
    print("\n" + "="*60)
    print("測試 1: 準備區→房間入口（正常調度）")
    print("="*60)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    test_rack_id = 901
    prepare_location_id = 11  # 系統準備區第一位
    inlet_id = 10001  # 房間1入口

    try:
        # 清理舊資料
        print("\n🧹 清理舊資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.rack_id == test_rack_id))
            session.exec(delete(Rack).where(Rack.id == test_rack_id))
            session.exec(
                update(Location)
                .where(Location.id.in_([prepare_location_id, inlet_id]))
                .values(rack_id=None, location_status_id=2)
            )
            session.commit()
        print("✅ 清理完成")

        # 創建測試資料
        print("\n📝 創建測試資料...")
        with pool_manager.get_session() as session:
            # 查詢房間1和產品1的 process_settings_id
            room = session.exec(select(Room).where(Room.id == 1)).first()
            product = session.exec(select(Product).where(Product.id == 1)).first()

            # 創建料架在準備區（已派車到房間1，產品匹配）
            rack = Rack(
                id=test_rack_id,
                name="TEST_DISPATCH_RACK",
                location_id=prepare_location_id,
                room_id=1,  # 已派車到房間1
                product_id=1,  # 產品1
                status_id=1,
                direction=0,
                is_carry=0
            )
            session.add(rack)

            # 設定準備區佔用
            session.exec(
                update(Location)
                .where(Location.id == prepare_location_id)
                .values(location_status_id=3, rack_id=test_rack_id)
            )

            # 設定房間入口空閒
            session.exec(
                update(Location)
                .where(Location.id == inlet_id)
                .values(location_status_id=2, rack_id=None)
            )
            session.commit()
            print(f"✅ 測試資料創建完成（料架在準備區，已派車到房間1，入口空閒）")

        # 執行流程
        print("\n🚀 執行流程...")
        with open('/app/config/tafl/flows/room_dispatch_simple.yaml', 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        result = await flow_executor.execute_flow(flow_yaml)
        print(f"📊 執行結果: {result.get('status')}")

        # 驗證任務創建
        with pool_manager.get_session() as session:
            tasks = session.exec(
                select(Task).where(
                    Task.rack_id == test_rack_id,
                    Task.work_id == 210001
                )
            ).all()

            if tasks:
                task = tasks[0]
                print(f"✅ 成功創建任務")
                print(f"   任務 ID: {task.id}")
                print(f"   Work ID: {task.work_id}")
                print(f"   房間 ID: {task.room_id}")
                print(f"   優先級: {task.priority}")
                if task.parameters:
                    to_loc_name = task.parameters.get('to_location_name')
                    to_loc_id = task.parameters.get('to_location_id')
                    print(f"   目的地: {to_loc_name}")
                    # 驗證目的地是否為房間入口 (Loader Box)
                    if to_loc_name and 'Loader Box' in to_loc_name:
                        print(f"   ✅ 正確路由到房間入口")
                        return True
                    elif to_loc_id == 10001:  # 房間1入口
                        print(f"   ✅ 正確路由到房間入口")
                        return True
                    else:
                        print(f"   ❌ 錯誤：目的地不是房間入口")
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
            session.exec(
                update(Location)
                .where(Location.id.in_([prepare_location_id, inlet_id]))
                .values(rack_id=None, location_status_id=2)
            )
            session.exec(delete(Rack).where(Rack.id == test_rack_id))
            session.commit()
        print("✅ 清理完成")

async def test_room_dispatch_no_room_id():
    """測試場景2: 料架未派車（room_id為null），不應創建任務"""
    print("\n" + "="*60)
    print("測試 2: 料架未派車（room_id為null）")
    print("="*60)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    test_rack_id = 902
    prepare_location_id = 12

    try:
        # 清理舊資料
        print("\n🧹 清理舊資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.rack_id == test_rack_id))
            session.exec(delete(Rack).where(Rack.id == test_rack_id))
            session.exec(
                update(Location)
                .where(Location.id == prepare_location_id)
                .values(rack_id=None, location_status_id=2)
            )
            session.commit()
        print("✅ 清理完成")

        # 創建測試資料
        print("\n📝 創建測試資料...")
        with pool_manager.get_session() as session:
            # 創建料架在準備區（未派車，room_id為null）
            rack = Rack(
                id=test_rack_id,
                name="TEST_NO_ROOM_RACK",
                location_id=prepare_location_id,
                room_id=None,  # 未派車
                product_id=1,
                status_id=1,
                direction=0,
                is_carry=0
            )
            session.add(rack)

            # 設定準備區佔用
            session.exec(
                update(Location)
                .where(Location.id == prepare_location_id)
                .values(location_status_id=3, rack_id=test_rack_id)
            )
            session.commit()
            print("✅ 測試資料創建完成（料架在準備區，未派車）")

        # 執行流程
        print("\n🚀 執行流程...")
        with open('/app/config/tafl/flows/room_dispatch_simple.yaml', 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        result = await flow_executor.execute_flow(flow_yaml)
        print(f"📊 執行結果: {result.get('status')}")

        # 驗證不應創建任務
        with pool_manager.get_session() as session:
            tasks = session.exec(
                select(Task).where(
                    Task.rack_id == test_rack_id,
                    Task.work_id == 210001
                )
            ).all()

            if not tasks:
                print(f"✅ 正確：未派車料架不創建任務")
                return True
            else:
                print(f"❌ 錯誤：不應該創建任務")
                return False

    finally:
        # 清理
        print("\n🧹 清理測試資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.rack_id == test_rack_id))
            session.exec(
                update(Location)
                .where(Location.id == prepare_location_id)
                .values(rack_id=None, location_status_id=2)
            )
            session.exec(delete(Rack).where(Rack.id == test_rack_id))
            session.commit()
        print("✅ 清理完成")

async def main():
    """執行所有測試"""
    print("\n" + "="*60)
    print("房間投料調度流程測試")
    print("="*60)

    results = []

    # 測試 1: 正常調度
    results.append(("正常調度", await test_room_dispatch_success()))

    # 測試 2: 未派車不調度
    results.append(("未派車不調度", await test_room_dispatch_no_room_id()))

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