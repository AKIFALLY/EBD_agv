#!/usr/bin/env python3
"""測試射出機停車格到系統準備區調度流程"""

import sys
import asyncio

sys.path.insert(0, '/app/tafl_wcs_ws/src')
sys.path.insert(0, '/app/db_proxy_ws/src')
sys.path.insert(0, '/app/tafl_ws/src')

from tafl_wcs.tafl_executor_wrapper import TAFLExecutorWrapper
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import Rack, Location, Task, Machine
from sqlmodel import select, delete
from sqlalchemy import update

async def test_machine_to_prepare_success():
    """測試場景1: 機台停車格有料架，準備區有空位，應創建任務"""
    print("\n" + "="*60)
    print("測試 1: 機台停車格→準備區（正常調度）")
    print("="*60)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    test_rack_id = 601
    test_machine_id = 99
    parking_space_1 = 201  # 測試用停車格
    prepare_location_id = 11  # 系統準備區第一位

    try:
        # 清理舊資料
        print("\n🧹 清理舊資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.rack_id == test_rack_id))
            session.exec(delete(Rack).where(Rack.id == test_rack_id))
            session.exec(delete(Machine).where(Machine.id == test_machine_id))
            session.commit()
        print("✅ 清理完成")

        # 創建測試資料
        print("\n📝 創建測試資料...")
        with pool_manager.get_session() as session:
            # 創建測試機台
            machine = Machine(
                id=test_machine_id,
                name="TEST_MACHINE",
                enable=1,
                parking_space_1=parking_space_1,
                parking_space_2=202,
                process_settings_id=1
            )
            session.add(machine)

            # 創建料架在停車格（已派車，room_id不為null）
            rack = Rack(
                id=test_rack_id,
                name="TEST_RACK_PARKING",
                location_id=parking_space_1,
                room_id=1,  # 已派車到房間1
                status_id=1,
                product_id=1,
                direction=0,
                is_carry=0
            )
            session.add(rack)

            # 確保準備區有空位
            session.exec(
                update(Location)
                .where(Location.id == prepare_location_id)
                .values(location_status_id=2, rack_id=None)
            )
            session.commit()
            print("✅ 測試資料創建完成（機台停車格有已派車料架，準備區有空位）")

        # 執行流程
        print("\n🚀 執行流程...")
        with open('/app/config/tafl/flows/machine_to_prepare.yaml', 'r', encoding='utf-8') as f:
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
                print(f"   任務名稱: {task.name}")
                print(f"   Work ID: {task.work_id}")
                print(f"   優先級: {task.priority}")
                if task.parameters:
                    target_loc_id = task.parameters.get('target_location_id')
                    target_loc_name = task.parameters.get('target_location_name')
                    print(f"   源位置: {task.parameters.get('source_location_id')}")
                    print(f"   目標位置ID: {target_loc_id}")
                    print(f"   目標位置名稱: {target_loc_name}")
                    # 驗證目標位置是否在準備區 (11-18)
                    if target_loc_id and 11 <= target_loc_id <= 18:
                        print(f"   ✅ 正確路由到系統準備區")
                        return True
                    elif target_loc_name and 'SystemPrepareArea' in target_loc_name:
                        print(f"   ✅ 正確路由到系統準備區")
                        return True
                    else:
                        print(f"   ❌ 錯誤：目的地不是系統準備區")
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
                .where(Location.id == prepare_location_id)
                .values(rack_id=None, location_status_id=2)
            )
            session.exec(delete(Rack).where(Rack.id == test_rack_id))
            session.exec(delete(Machine).where(Machine.id == test_machine_id))
            session.commit()
        print("✅ 清理完成")

async def test_machine_to_prepare_no_room_id():
    """測試場景2: 料架未派車（room_id為null），不應創建任務"""
    print("\n" + "="*60)
    print("測試 2: 料架未派車（room_id為null）")
    print("="*60)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    test_rack_id = 602
    test_machine_id = 98
    parking_space_1 = 203

    try:
        # 清理舊資料
        print("\n🧹 清理舊資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.rack_id == test_rack_id))
            session.exec(delete(Rack).where(Rack.id == test_rack_id))
            session.exec(delete(Machine).where(Machine.id == test_machine_id))
            session.commit()
        print("✅ 清理完成")

        # 創建測試資料
        print("\n📝 創建測試資料...")
        with pool_manager.get_session() as session:
            # 創建測試機台
            machine = Machine(
                id=test_machine_id,
                name="TEST_MACHINE_2",
                enable=1,
                parking_space_1=parking_space_1,
                parking_space_2=204,
                process_settings_id=1
            )
            session.add(machine)

            # 創建料架在停車格（未派車，room_id為null）
            rack = Rack(
                id=test_rack_id,
                name="TEST_RACK_NO_ROOM",
                location_id=parking_space_1,
                room_id=None,  # 未派車
                status_id=1,
                product_id=1,
                direction=0,
                is_carry=0
            )
            session.add(rack)
            session.commit()
            print("✅ 測試資料創建完成（料架未派車）")

        # 執行流程
        print("\n🚀 執行流程...")
        with open('/app/config/tafl/flows/machine_to_prepare.yaml', 'r', encoding='utf-8') as f:
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
            session.exec(delete(Rack).where(Rack.id == test_rack_id))
            session.exec(delete(Machine).where(Machine.id == test_machine_id))
            session.commit()
        print("✅ 清理完成")

async def main():
    """執行所有測試"""
    print("\n" + "="*60)
    print("射出機停車格→系統準備區流程測試")
    print("="*60)

    results = []

    # 測試 1: 正常調度
    results.append(("正常調度", await test_machine_to_prepare_success()))

    # 測試 2: 未派車不調度
    results.append(("未派車不調度", await test_machine_to_prepare_no_room_id()))

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