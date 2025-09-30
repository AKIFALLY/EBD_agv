#!/usr/bin/env python3
"""測試架台翻轉流程"""

import sys
import asyncio

sys.path.insert(0, '/app/tafl_wcs_ws/src')
sys.path.insert(0, '/app/db_proxy_ws/src')
sys.path.insert(0, '/app/tafl_ws/src')

from tafl_wcs.tafl_executor_wrapper import TAFLExecutorWrapper
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import Rack, Location, Task, Carrier
from sqlmodel import select, delete
from sqlalchemy import update

async def test_inlet_rotation_aempty_bwork():
    """測試場景1: 房間入口翻轉（A面空，B面有待作業載具）"""
    print("\n" + "="*60)
    print("測試 1: 房間入口翻轉（A空B工作）")
    print("="*60)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    test_rack_id = 801
    inlet_id = 10001  # 房間1入口

    try:
        # 清理舊資料
        print("\n🧹 清理舊資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.rack_id == test_rack_id))
            session.exec(delete(Carrier).where(Carrier.rack_id == test_rack_id))
            session.exec(delete(Rack).where(Rack.id == test_rack_id))
            session.exec(
                update(Location)
                .where(Location.id == inlet_id)
                .values(rack_id=None, location_status_id=2)
            )
            session.commit()
        print("✅ 清理完成")

        # 創建測試資料
        print("\n📝 創建測試資料...")
        with pool_manager.get_session() as session:
            # 創建rack在入口
            rack = Rack(
                id=test_rack_id,
                name="TEST_ROTATION_INLET",
                location_id=inlet_id,
                status_id=1,
                product_id=1,
                direction=0,
                is_carry=0
            )
            session.add(rack)

            # A面(1-16)：空（無carriers）
            # B面(17-32)：有待作業載具（status_id != 8）
            for i in range(17, 25):  # B面創建8個待作業載具
                carrier = Carrier(
                    rack_id=test_rack_id,
                    rack_index=i,
                    room_id=1,
                    port_id=3000 + i,
                    status_id=1  # 待作業
                )
                session.add(carrier)

            # 設定入口佔用
            session.exec(
                update(Location)
                .where(Location.id == inlet_id)
                .values(location_status_id=3, rack_id=test_rack_id)
            )
            session.commit()
            print("✅ 測試資料創建完成（A面空，B面有8個待作業載具）")

        # 執行流程
        print("\n🚀 執行流程...")
        with open('/app/config/tafl/flows/rack_rotation_room_inlet_aempty_bwork.yaml', 'r', encoding='utf-8') as f:
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
                print(f"✅ 成功創建翻轉任務")
                print(f"   任務 ID: {task.id}")
                print(f"   任務名稱: {task.name}")
                if task.parameters:
                    rotation_angle = task.parameters.get('rotation_angle')
                    print(f"   翻轉角度: {rotation_angle}")
                    print(f"   原因: {task.parameters.get('reason')}")
                    print(f"   B面待作業數: {task.parameters.get('b_side_work_count')}")
                    # 驗證翻轉角度是否為180度
                    if rotation_angle == 180:
                        print(f"   ✅ 正確翻轉角度（180度）")
                        return True
                    else:
                        print(f"   ❌ 錯誤：翻轉角度不是180度")
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
                .where(Location.id == inlet_id)
                .values(rack_id=None, location_status_id=2)
            )
            session.exec(delete(Rack).where(Rack.id == test_rack_id))
            session.commit()
        print("✅ 清理完成")

async def test_outlet_rotation_afull_bempty():
    """測試場景2: 房間出口翻轉（A面滿，B面空，房間有載具）"""
    print("\n" + "="*60)
    print("測試 2: 房間出口翻轉（A滿B空）")
    print("="*60)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    test_rack_id = 802
    outlet_id = 10002  # 房間1出口
    test_carrier_id = 9999

    try:
        # 清理舊資料
        print("\n🧹 清理舊資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.rack_id == test_rack_id))
            session.exec(delete(Carrier).where(Carrier.rack_id == test_rack_id))
            session.exec(delete(Carrier).where(Carrier.id == test_carrier_id))
            session.exec(delete(Rack).where(Rack.id == test_rack_id))
            session.exec(
                update(Location)
                .where(Location.id == outlet_id)
                .values(rack_id=None, location_status_id=2)
            )
            session.commit()
        print("✅ 清理完成")

        # 創建測試資料
        print("\n📝 創建測試資料...")
        with pool_manager.get_session() as session:
            # 創建rack在出口
            rack = Rack(
                id=test_rack_id,
                name="TEST_ROTATION_OUTLET",
                location_id=outlet_id,
                status_id=1,
                product_id=1,
                direction=0,
                is_carry=0
            )
            session.add(rack)

            # A面(1-16)：滿載（16個carriers）
            for i in range(1, 17):
                carrier = Carrier(
                    rack_id=test_rack_id,
                    rack_index=i,
                    room_id=1,
                    port_id=4000 + i,
                    status_id=8  # 已完成
                )
                session.add(carrier)

            # B面(17-32)：空（無carriers）

            # 房間內有載具待處理（關鍵條件）
            carrier_in_room = Carrier(
                id=test_carrier_id,
                rack_id=None,  # 未放入rack
                room_id=1,
                port_id=5000,
                status_id=8  # 已完成
            )
            session.add(carrier_in_room)

            # 設定出口佔用
            session.exec(
                update(Location)
                .where(Location.id == outlet_id)
                .values(location_status_id=3, rack_id=test_rack_id)
            )
            session.commit()
            print("✅ 測試資料創建完成（A面滿16個，B面空，房間有載具待處理）")

        # 執行流程
        print("\n🚀 執行流程...")
        with open('/app/config/tafl/flows/rack_rotation_room_outlet_afull_bempty.yaml', 'r', encoding='utf-8') as f:
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
                print(f"✅ 成功創建翻轉任務")
                print(f"   任務 ID: {task.id}")
                print(f"   任務名稱: {task.name}")
                if task.parameters:
                    rotation_angle = task.parameters.get('rotation_angle')
                    print(f"   翻轉角度: {rotation_angle}")
                    print(f"   原因: {task.parameters.get('reason')}")
                    print(f"   A面載具數: {task.parameters.get('a_side_count')}")
                    print(f"   房間載具數: {task.parameters.get('room_carriers')}")
                    # 驗證翻轉角度是否為180度
                    if rotation_angle == 180:
                        print(f"   ✅ 正確翻轉角度（180度）")
                        return True
                    else:
                        print(f"   ❌ 錯誤：翻轉角度不是180度")
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
            session.exec(delete(Carrier).where(Carrier.id == test_carrier_id))
            session.exec(
                update(Location)
                .where(Location.id == outlet_id)
                .values(rack_id=None, location_status_id=2)
            )
            session.exec(delete(Rack).where(Rack.id == test_rack_id))
            session.commit()
        print("✅ 清理完成")

async def main():
    """執行所有測試"""
    print("\n" + "="*60)
    print("架台翻轉流程測試")
    print("="*60)

    results = []

    # 測試 1: 入口翻轉
    results.append(("入口翻轉（A空B工作）", await test_inlet_rotation_aempty_bwork()))

    # 測試 2: 出口翻轉
    results.append(("出口翻轉（A滿B空）", await test_outlet_rotation_afull_bempty()))

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