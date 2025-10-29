#!/usr/bin/env python3
"""測試 Cargo AGV 入口卸載和出口裝載流程"""

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

# ============================================================================
# 入口卸載測試（3個場景）
# ============================================================================

async def test_entrance_unload_with_work_carriers():
    """測試場景1: 房間入口有待處理 Carrier → 應創建卸載任務"""
    print("\n" + "="*70)
    print("測試 1: Cargo 入口卸載 - 正常卸載（有待處理 Carrier）")
    print("="*70)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    test_rack_id = 901
    inlet_id = 10001  # 房間1入口

    try:
        # 清理舊資料
        print("\n🧹 清理舊資料...")
        with pool_manager.get_session() as session:
            # 先重置位置狀態（解除外鍵約束）
            session.exec(
                update(Location)
                .where(Location.id == inlet_id)
                .values(rack_id=None, location_status_id=2)
            )
            session.commit()

            # 清理該位置上的所有 rack 和相關資料
            racks_at_location = session.exec(
                select(Rack).where(Rack.location_id == inlet_id)
            ).all()
            for rack in racks_at_location:
                session.exec(delete(Task).where(Task.rack_id == rack.id))
                session.exec(delete(Carrier).where(Carrier.rack_id == rack.id))
                session.exec(delete(Rack).where(Rack.id == rack.id))

            session.commit()
        print("✅ 清理完成")

        # 創建測試資料
        print("\n📝 創建測試資料...")
        with pool_manager.get_session() as session:
            # 創建 Rack 在入口
            rack = Rack(
                id=test_rack_id,
                name="TEST_CARGO_ENTRANCE",
                location_id=inlet_id,
                status_id=1,
                product_id=1,
                direction=0,
                is_carry=0
            )
            session.add(rack)

            # 創建 8 個待處理 Carrier (status_id=1)
            for i in range(1, 9):
                carrier = Carrier(
                    rack_id=test_rack_id,
                    rack_index=i,
                    room_id=1,
                    port_id=4000 + i,
                    status_id=1  # 待處理
                )
                session.add(carrier)

            # 設定入口佔用
            session.exec(
                update(Location)
                .where(Location.id == inlet_id)
                .values(location_status_id=3, rack_id=test_rack_id)
            )
            session.commit()
            print("✅ 測試資料創建完成（8個待處理 Carrier）")

        # 執行流程
        print("\n🚀 執行流程...")
        with open('/app/config/tafl/flows/cargo_entrance_unload.yaml', 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        result = await flow_executor.execute_flow(flow_yaml)
        print(f"📊 執行結果: {result.get('status')}")

        # 驗證任務創建
        with pool_manager.get_session() as session:
            tasks = session.exec(
                select(Task).where(
                    Task.rack_id == test_rack_id,
                    Task.work_id == 2000102
                )
            ).all()

            if tasks:
                task = tasks[0]
                print(f"✅ 成功創建卸載任務")
                print(f"   - 任務ID: {task.id}")
                print(f"   - Work ID: {task.work_id}")
                print(f"   - Rack ID: {task.rack_id}")
                print(f"   - Room ID: {task.room_id}")
                print(f"   - 描述: {task.description}")

                # 驗證任務屬性
                assert task.work_id == 2000102, "Work ID 應為 2000102"
                assert task.rack_id == test_rack_id, "Rack ID 不符"
                assert task.room_id == 1, f"Room ID 應為 1，實際為 {task.room_id}"
                assert task.status_id == 1, "任務狀態應為 PENDING"

                # 驗證 parameters 中的 room_id
                import json
                if task.parameters:
                    params = json.loads(task.parameters) if isinstance(task.parameters, str) else task.parameters
                    assert params.get('room_id') == 1, f"Parameters 中的 room_id 應為 1，實際為 {params.get('room_id')}"

                return True
            else:
                print("❌ 未創建卸載任務")
                return False

    except Exception as e:
        print(f"❌ 測試失敗: {e}")
        import traceback
        traceback.print_exc()
        return False

async def test_entrance_unload_no_work_carriers():
    """測試場景2: 房間入口無待處理 Carrier → 不應創建任務"""
    print("\n" + "="*70)
    print("測試 2: Cargo 入口卸載 - 無待處理 Carrier（全部已完成）")
    print("="*70)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    test_rack_id = 902
    inlet_id = 10001  # 房間1入口

    try:
        # 清理舊資料
        print("\n🧹 清理舊資料...")
        with pool_manager.get_session() as session:
            # 先重置位置狀態（解除外鍵約束）
            session.exec(
                update(Location)
                .where(Location.id == inlet_id)
                .values(rack_id=None, location_status_id=2)
            )
            session.commit()

            # 清理該位置上的所有 rack 和相關資料
            racks_at_location = session.exec(
                select(Rack).where(Rack.location_id == inlet_id)
            ).all()
            for rack in racks_at_location:
                session.exec(delete(Task).where(Task.rack_id == rack.id))
                session.exec(delete(Carrier).where(Carrier.rack_id == rack.id))
                session.exec(delete(Rack).where(Rack.id == rack.id))

            session.commit()
        print("✅ 清理完成")

        # 創建測試資料
        print("\n📝 創建測試資料...")
        with pool_manager.get_session() as session:
            # 創建 Rack 在入口
            rack = Rack(
                id=test_rack_id,
                name="TEST_CARGO_ENTRANCE_2",
                location_id=inlet_id,
                status_id=1,
                product_id=1,
                direction=0,
                is_carry=0
            )
            session.add(rack)

            # 創建 8 個已完成 Carrier (status_id=8)
            for i in range(1, 9):
                carrier = Carrier(
                    rack_id=test_rack_id,
                    rack_index=i,
                    room_id=1,
                    port_id=4100 + i,
                    status_id=8  # 已完成
                )
                session.add(carrier)

            # 設定入口佔用
            session.exec(
                update(Location)
                .where(Location.id == inlet_id)
                .values(location_status_id=3, rack_id=test_rack_id)
            )
            session.commit()
            print("✅ 測試資料創建完成（8個已完成 Carrier）")

        # 執行流程
        print("\n🚀 執行流程...")
        with open('/app/config/tafl/flows/cargo_entrance_unload.yaml', 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        result = await flow_executor.execute_flow(flow_yaml)
        print(f"📊 執行結果: {result.get('status')}")

        # 驗證不應創建任務
        with pool_manager.get_session() as session:
            tasks = session.exec(
                select(Task).where(
                    Task.rack_id == test_rack_id,
                    Task.work_id == 2000102
                )
            ).all()

            if not tasks:
                print("✅ 正確：未創建卸載任務（無待處理 Carrier）")
                return True
            else:
                print(f"❌ 錯誤：不應創建任務，但創建了 {len(tasks)} 個任務")
                return False

    except Exception as e:
        print(f"❌ 測試失敗: {e}")
        import traceback
        traceback.print_exc()
        return False

async def test_entrance_unload_duplicate_prevention():
    """測試場景3: 防重複創建卸載任務"""
    print("\n" + "="*70)
    print("測試 3: Cargo 入口卸載 - 防重複創建")
    print("="*70)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    test_rack_id = 903
    inlet_id = 10001  # 房間1入口

    try:
        # 清理舊資料
        print("\n🧹 清理舊資料...")
        with pool_manager.get_session() as session:
            # 先重置位置狀態（解除外鍵約束）
            session.exec(
                update(Location)
                .where(Location.id == inlet_id)
                .values(rack_id=None, location_status_id=2)
            )
            session.commit()

            # 清理該位置上的所有 rack 和相關資料
            racks_at_location = session.exec(
                select(Rack).where(Rack.location_id == inlet_id)
            ).all()
            for rack in racks_at_location:
                session.exec(delete(Task).where(Task.rack_id == rack.id))
                session.exec(delete(Carrier).where(Carrier.rack_id == rack.id))
                session.exec(delete(Rack).where(Rack.id == rack.id))

            session.commit()
        print("✅ 清理完成")

        # 創建測試資料
        print("\n📝 創建測試資料...")
        with pool_manager.get_session() as session:
            # 創建 Rack 在入口
            rack = Rack(
                id=test_rack_id,
                name="TEST_CARGO_DUP",
                location_id=inlet_id,
                status_id=1,
                product_id=1,
                direction=0,
                is_carry=0
            )
            session.add(rack)

            # 創建待處理 Carrier
            for i in range(1, 5):
                carrier = Carrier(
                    rack_id=test_rack_id,
                    rack_index=i,
                    room_id=1,
                    port_id=4200 + i,
                    status_id=1  # 待處理
                )
                session.add(carrier)

            # 設定入口佔用
            session.exec(
                update(Location)
                .where(Location.id == inlet_id)
                .values(location_status_id=3, rack_id=test_rack_id)
            )
            session.commit()
            print("✅ 測試資料創建完成")

        # 執行流程第1次
        print("\n🚀 執行流程（第1次）...")
        with open('/app/config/tafl/flows/cargo_entrance_unload.yaml', 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        result1 = await flow_executor.execute_flow(flow_yaml)
        print(f"📊 執行結果1: {result1.get('status')}")

        # 執行流程第2次
        print("\n🚀 執行流程（第2次）...")
        result2 = await flow_executor.execute_flow(flow_yaml)
        print(f"📊 執行結果2: {result2.get('status')}")

        # 驗證只創建1個任務
        with pool_manager.get_session() as session:
            tasks = session.exec(
                select(Task).where(
                    Task.rack_id == test_rack_id,
                    Task.work_id == 2000102
                )
            ).all()

            if len(tasks) == 1:
                print(f"✅ 正確：只創建了 1 個任務（防重複成功）")
                return True
            else:
                print(f"❌ 錯誤：創建了 {len(tasks)} 個任務（應該只有1個）")
                return False

    except Exception as e:
        print(f"❌ 測試失敗: {e}")
        import traceback
        traceback.print_exc()
        return False

# ============================================================================
# 出口裝載測試（4個場景）
# ============================================================================

async def test_exit_load_normal():
    """測試場景4: 房間有已完成 Carrier + Rack 有空位 → 應創建裝載任務"""
    print("\n" + "="*70)
    print("測試 4: Cargo 出口裝載 - 正常裝載")
    print("="*70)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    test_rack_id = 904
    outlet_id = 10002  # 房間1出口

    try:
        # 清理舊資料
        print("\n🧹 清理舊資料...")
        with pool_manager.get_session() as session:
            # 清理房間內所有 Carrier
            session.exec(delete(Carrier).where(Carrier.room_id == 1))

            # 先重置位置狀態（解除外鍵約束）
            session.exec(
                update(Location)
                .where(Location.id == outlet_id)
                .values(rack_id=None, location_status_id=2)
            )
            session.commit()

            # 清理該位置上的所有 rack 和相關資料
            racks_at_location = session.exec(
                select(Rack).where(Rack.location_id == outlet_id)
            ).all()
            for rack in racks_at_location:
                session.exec(delete(Task).where(Task.rack_id == rack.id))
                session.exec(delete(Carrier).where(Carrier.rack_id == rack.id))
                session.exec(delete(Rack).where(Rack.id == rack.id))

            session.commit()
        print("✅ 清理完成")

        # 創建測試資料
        print("\n📝 創建測試資料...")
        with pool_manager.get_session() as session:
            # 創建 Rack 在出口（有5個 Carrier）
            rack = Rack(
                id=test_rack_id,
                name="TEST_CARGO_EXIT",
                location_id=outlet_id,
                status_id=1,
                product_id=1,
                direction=0,
                is_carry=0
            )
            session.add(rack)

            # Rack 上有5個 Carrier
            for i in range(1, 6):
                carrier = Carrier(
                    rack_id=test_rack_id,
                    rack_index=i,
                    room_id=None,
                    port_id=4300 + i,
                    status_id=1
                )
                session.add(carrier)

            # 房間出口傳送箱有10個 Carrier（等待裝載回 Rack）
            for i in range(1, 11):
                carrier = Carrier(
                    rack_id=0,                    # 不在 Rack 上
                    rack_index=0,
                    room_id=1,
                    port_id=2021 + (i % 4),      # 出口傳送箱 port (2021-2024 循環)
                    status_id=202                 # 進入出入口傳送箱
                )
                session.add(carrier)

            # 設定出口佔用
            session.exec(
                update(Location)
                .where(Location.id == outlet_id)
                .values(location_status_id=3, rack_id=test_rack_id)
            )
            session.commit()
            print("✅ 測試資料創建完成（Rack 有5個 Carrier，房間出口傳送箱有10個 Carrier）")

        # 執行流程
        print("\n🚀 執行流程...")
        with open('/app/config/tafl/flows/cargo_exit_load.yaml', 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        result = await flow_executor.execute_flow(flow_yaml)
        print(f"📊 執行結果: {result.get('status')}")

        # 驗證任務創建
        with pool_manager.get_session() as session:
            tasks = session.exec(
                select(Task).where(
                    Task.rack_id == test_rack_id,
                    Task.work_id == 2000201
                )
            ).all()

            if tasks:
                task = tasks[0]
                print(f"✅ 成功創建裝載任務")
                print(f"   - 任務ID: {task.id}")
                print(f"   - Work ID: {task.work_id}")
                print(f"   - Rack ID: {task.rack_id}")
                print(f"   - Room ID: {task.room_id}")
                print(f"   - 描述: {task.description}")

                # 驗證任務屬性
                assert task.work_id == 2000201, "Work ID 應為 2000201"
                assert task.rack_id == test_rack_id, "Rack ID 不符"
                assert task.room_id == 1, f"Room ID 應為 1，實際為 {task.room_id}"
                assert task.status_id == 1, "任務狀態應為 PENDING"

                # 驗證 parameters 中的資料
                import json
                if task.parameters:
                    params = json.loads(task.parameters) if isinstance(task.parameters, str) else task.parameters
                    assert params.get('room_id') == 1, f"Parameters 中的 room_id 應為 1，實際為 {params.get('room_id')}"
                    assert params.get('boxout_carriers') == 10, f"出口傳送箱載具應為 10，實際為 {params.get('boxout_carriers')}"

                return True
            else:
                print("❌ 未創建裝載任務")
                return False

    except Exception as e:
        print(f"❌ 測試失敗: {e}")
        import traceback
        traceback.print_exc()
        return False

async def test_exit_load_rack_full():
    """測試場景5: Rack 已滿 → 不應創建任務"""
    print("\n" + "="*70)
    print("測試 5: Cargo 出口裝載 - Rack 已滿")
    print("="*70)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    test_rack_id = 905
    outlet_id = 10002  # 房間1出口

    try:
        # 清理舊資料
        print("\n🧹 清理舊資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.rack_id == test_rack_id))
            session.exec(delete(Carrier).where(Carrier.rack_id == test_rack_id))
            session.exec(delete(Carrier).where(Carrier.room_id == 1))
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
            # 創建 Rack 在出口
            rack = Rack(
                id=test_rack_id,
                name="TEST_CARGO_EXIT_FULL",
                location_id=outlet_id,
                status_id=1,
                product_id=1,
                direction=0,
                is_carry=0
            )
            session.add(rack)

            # Rack 上已有32個 Carrier（滿載）
            for i in range(1, 33):
                carrier = Carrier(
                    rack_id=test_rack_id,
                    rack_index=i,
                    room_id=None,
                    port_id=4500 + i,
                    status_id=1
                )
                session.add(carrier)

            # 房間內有5個已完成 Carrier
            for i in range(1, 6):
                carrier = Carrier(
                    rack_id=None,
                    rack_index=None,
                    room_id=1,
                    port_id=4600 + i,
                    status_id=8  # 已完成
                )
                session.add(carrier)

            # 設定出口佔用
            session.exec(
                update(Location)
                .where(Location.id == outlet_id)
                .values(location_status_id=3, rack_id=test_rack_id)
            )
            session.commit()
            print("✅ 測試資料創建完成（Rack 已滿32個，房間有5個已完成 Carrier）")

        # 執行流程
        print("\n🚀 執行流程...")
        with open('/app/config/tafl/flows/cargo_exit_load.yaml', 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        result = await flow_executor.execute_flow(flow_yaml)
        print(f"📊 執行結果: {result.get('status')}")

        # 驗證不應創建任務
        with pool_manager.get_session() as session:
            tasks = session.exec(
                select(Task).where(
                    Task.rack_id == test_rack_id,
                    Task.work_id == 2000201
                )
            ).all()

            if not tasks:
                print("✅ 正確：未創建裝載任務（Rack 已滿）")
                return True
            else:
                print(f"❌ 錯誤：不應創建任務，但創建了 {len(tasks)} 個任務")
                return False

    except Exception as e:
        print(f"❌ 測試失敗: {e}")
        import traceback
        traceback.print_exc()
        return False

async def test_exit_load_no_completed_carriers():
    """測試場景6: 房間無已完成 Carrier → 不應創建任務"""
    print("\n" + "="*70)
    print("測試 6: Cargo 出口裝載 - 無已完成 Carrier")
    print("="*70)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    test_rack_id = 906
    outlet_id = 10002  # 房間1出口

    try:
        # 清理舊資料
        print("\n🧹 清理舊資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.rack_id == test_rack_id))
            session.exec(delete(Carrier).where(Carrier.rack_id == test_rack_id))
            session.exec(delete(Carrier).where(Carrier.room_id == 1))
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
            # 創建 Rack 在出口（空的）
            rack = Rack(
                id=test_rack_id,
                name="TEST_CARGO_EXIT_NO_COMPLETED",
                location_id=outlet_id,
                status_id=1,
                product_id=1,
                direction=0,
                is_carry=0
            )
            session.add(rack)

            # 房間內沒有已完成 Carrier

            # 設定出口佔用
            session.exec(
                update(Location)
                .where(Location.id == outlet_id)
                .values(location_status_id=3, rack_id=test_rack_id)
            )
            session.commit()
            print("✅ 測試資料創建完成（Rack 為空，房間無已完成 Carrier）")

        # 執行流程
        print("\n🚀 執行流程...")
        with open('/app/config/tafl/flows/cargo_exit_load.yaml', 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        result = await flow_executor.execute_flow(flow_yaml)
        print(f"📊 執行結果: {result.get('status')}")

        # 驗證不應創建任務
        with pool_manager.get_session() as session:
            tasks = session.exec(
                select(Task).where(
                    Task.rack_id == test_rack_id,
                    Task.work_id == 2000201
                )
            ).all()

            if not tasks:
                print("✅ 正確：未創建裝載任務（無已完成 Carrier）")
                return True
            else:
                print(f"❌ 錯誤：不應創建任務，但創建了 {len(tasks)} 個任務")
                return False

    except Exception as e:
        print(f"❌ 測試失敗: {e}")
        import traceback
        traceback.print_exc()
        return False

async def test_exit_load_duplicate_prevention():
    """測試場景7: 防重複創建裝載任務"""
    print("\n" + "="*70)
    print("測試 7: Cargo 出口裝載 - 防重複創建")
    print("="*70)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    test_rack_id = 907
    outlet_id = 10002  # 房間1出口

    try:
        # 清理舊資料
        print("\n🧹 清理舊資料...")
        with pool_manager.get_session() as session:
            # Step 1: 先重置 location (清除外鍵)
            session.exec(
                update(Location)
                .where(Location.id == outlet_id)
                .values(rack_id=None, location_status_id=2)
            )
            session.commit()

            # Step 2: 刪除該位置的所有 racks 及其相關資料
            racks_at_location = session.exec(
                select(Rack).where(Rack.location_id == outlet_id)
            ).all()

            for rack in racks_at_location:
                session.exec(delete(Task).where(Task.rack_id == rack.id))
                session.exec(delete(Carrier).where(Carrier.rack_id == rack.id))
                session.exec(delete(Rack).where(Rack.id == rack.id))

            # Step 3: 清理房間內的 Carrier
            session.exec(delete(Carrier).where(Carrier.room_id == 1))
            session.commit()
        print("✅ 清理完成")

        # 創建測試資料
        print("\n📝 創建測試資料...")
        with pool_manager.get_session() as session:
            # 創建 Rack 在出口
            rack = Rack(
                id=test_rack_id,
                name="TEST_CARGO_EXIT_DUP",
                location_id=outlet_id,
                status_id=1,
                product_id=1,
                direction=0,
                is_carry=0
            )
            session.add(rack)

            # Rack 上有3個 Carrier
            for i in range(1, 4):
                carrier = Carrier(
                    rack_id=test_rack_id,
                    rack_index=i,
                    room_id=None,
                    port_id=4700 + i,
                    status_id=1
                )
                session.add(carrier)

            # 房間出口傳送箱有5個 Carrier
            for i in range(1, 6):
                carrier = Carrier(
                    rack_id=0,                    # 不在 Rack 上
                    rack_index=0,
                    room_id=1,
                    port_id=2021 + (i % 4),      # 出口傳送箱 port (2021-2024 循環)
                    status_id=202                 # 進入出入口傳送箱
                )
                session.add(carrier)

            # 設定出口佔用
            session.exec(
                update(Location)
                .where(Location.id == outlet_id)
                .values(location_status_id=3, rack_id=test_rack_id)
            )
            session.commit()
            print("✅ 測試資料創建完成")

        # 執行流程第1次
        print("\n🚀 執行流程（第1次）...")
        with open('/app/config/tafl/flows/cargo_exit_load.yaml', 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        result1 = await flow_executor.execute_flow(flow_yaml)
        print(f"📊 執行結果1: {result1.get('status')}")

        # 執行流程第2次
        print("\n🚀 執行流程（第2次）...")
        result2 = await flow_executor.execute_flow(flow_yaml)
        print(f"📊 執行結果2: {result2.get('status')}")

        # 驗證只創建1個任務
        with pool_manager.get_session() as session:
            tasks = session.exec(
                select(Task).where(
                    Task.rack_id == test_rack_id,
                    Task.work_id == 2000201
                )
            ).all()

            if len(tasks) == 1:
                print(f"✅ 正確：只創建了 1 個任務（防重複成功）")
                return True
            else:
                print(f"❌ 錯誤：創建了 {len(tasks)} 個任務（應該只有1個）")
                return False

    except Exception as e:
        print(f"❌ 測試失敗: {e}")
        import traceback
        traceback.print_exc()
        return False

async def test_room2_entrance_unload():
    """測試場景8: 房間2入口卸載 - 驗證多房間支援"""
    print("\n" + "="*70)
    print("測試 8: 房間2入口卸載 - 多房間支援驗證")
    print("="*70)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    test_rack_id = 920
    inlet_id = 20001  # 房間2入口

    try:
        # 清理舊資料
        print("\n🧹 清理舊資料...")
        with pool_manager.get_session() as session:
            session.exec(
                update(Location)
                .where(Location.id == inlet_id)
                .values(rack_id=None, location_status_id=2)
            )
            session.commit()

            racks_at_location = session.exec(
                select(Rack).where(Rack.location_id == inlet_id)
            ).all()
            for rack in racks_at_location:
                session.exec(delete(Task).where(Task.rack_id == rack.id))
                session.exec(delete(Carrier).where(Carrier.rack_id == rack.id))
                session.exec(delete(Rack).where(Rack.id == rack.id))

            session.commit()
        print("✅ 清理完成")

        # 創建測試資料
        print("\n📝 創建測試資料（房間2）...")
        with pool_manager.get_session() as session:
            rack = Rack(
                id=test_rack_id,
                name="TEST_ROOM2_ENTRANCE",
                location_id=inlet_id,
                status_id=1,
                product_id=1,
                direction=0,
                is_carry=0
            )
            session.add(rack)

            # 創建 5 個待處理 Carrier
            for i in range(1, 6):
                carrier = Carrier(
                    rack_id=test_rack_id,
                    rack_index=i,
                    room_id=2,  # 房間2
                    port_id=4000 + i,
                    status_id=1
                )
                session.add(carrier)

            session.exec(
                update(Location)
                .where(Location.id == inlet_id)
                .values(location_status_id=3, rack_id=test_rack_id)
            )
            session.commit()
            print("✅ 測試資料創建完成（房間2，5個待處理 Carrier）")

        # 執行流程
        print("\n🚀 執行流程...")
        with open('/app/config/tafl/flows/cargo_entrance_unload.yaml', 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        result = await flow_executor.execute_flow(flow_yaml)
        print(f"📊 執行結果: {result.get('status')}")

        # 驗證任務創建
        with pool_manager.get_session() as session:
            tasks = session.exec(
                select(Task).where(
                    Task.rack_id == test_rack_id,
                    Task.work_id == 2000102
                )
            ).all()

            if tasks:
                task = tasks[0]
                print(f"✅ 成功創建卸載任務（房間2）")
                print(f"   - 任務ID: {task.id}")
                print(f"   - Work ID: {task.work_id}")
                print(f"   - Rack ID: {task.rack_id}")
                print(f"   - Room ID: {task.room_id}")
                print(f"   - 描述: {task.description}")

                # 驗證任務屬性（關鍵：room_id 應為 2）
                assert task.work_id == 2000102, "Work ID 應為 2000102"
                assert task.rack_id == test_rack_id, "Rack ID 不符"
                assert task.room_id == 2, f"Room ID 應為 2，實際為 {task.room_id}"
                assert task.status_id == 1, "任務狀態應為 PENDING"

                # 驗證 parameters 中的 room_id
                import json
                if task.parameters:
                    params = json.loads(task.parameters) if isinstance(task.parameters, str) else task.parameters
                    assert params.get('room_id') == 2, f"Parameters 中的 room_id 應為 2，實際為 {params.get('room_id')}"

                return True
            else:
                print("❌ 未創建卸載任務")
                return False

    except Exception as e:
        print(f"❌ 測試失敗: {e}")
        import traceback
        traceback.print_exc()
        return False

async def test_room3_exit_load():
    """測試場景9: 房間3出口裝載 - 驗證多房間支援"""
    print("\n" + "="*70)
    print("測試 9: 房間3出口裝載 - 多房間支援驗證")
    print("="*70)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    test_rack_id = 930
    outlet_id = 30002  # 房間3出口

    try:
        # 清理舊資料
        print("\n🧹 清理舊資料...")
        with pool_manager.get_session() as session:
            session.exec(
                update(Location)
                .where(Location.id == outlet_id)
                .values(rack_id=None, location_status_id=2)
            )
            session.exec(delete(Carrier).where(Carrier.room_id == 3))
            session.commit()

            racks_at_location = session.exec(
                select(Rack).where(Rack.location_id == outlet_id)
            ).all()
            for rack in racks_at_location:
                session.exec(delete(Task).where(Task.rack_id == rack.id))
                session.exec(delete(Carrier).where(Carrier.rack_id == rack.id))
                session.exec(delete(Rack).where(Rack.id == rack.id))

            session.commit()
        print("✅ 清理完成")

        # 創建測試資料
        print("\n📝 創建測試資料（房間3）...")
        with pool_manager.get_session() as session:
            rack = Rack(
                id=test_rack_id,
                name="TEST_ROOM3_EXIT",
                location_id=outlet_id,
                status_id=1,
                product_id=1,
                direction=0,
                is_carry=0
            )
            session.add(rack)

            # Rack 上有 3 個 Carrier
            for i in range(1, 4):
                carrier = Carrier(
                    rack_id=test_rack_id,
                    rack_index=i,
                    room_id=None,
                    port_id=3000 + i,
                    status_id=8
                )
                session.add(carrier)

            # 房間3出口傳送箱有 6 個 Carrier
            for i in range(1, 7):
                carrier = Carrier(
                    rack_id=0,
                    rack_index=0,
                    room_id=3,  # 房間3
                    port_id=3021 + (i % 4),
                    status_id=202
                )
                session.add(carrier)

            session.exec(
                update(Location)
                .where(Location.id == outlet_id)
                .values(location_status_id=3, rack_id=test_rack_id)
            )
            session.commit()
            print("✅ 測試資料創建完成（房間3，Rack 有3個 Carrier，出口傳送箱有6個 Carrier）")

        # 執行流程
        print("\n🚀 執行流程...")
        with open('/app/config/tafl/flows/cargo_exit_load.yaml', 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        result = await flow_executor.execute_flow(flow_yaml)
        print(f"📊 執行結果: {result.get('status')}")

        # 驗證任務創建
        with pool_manager.get_session() as session:
            tasks = session.exec(
                select(Task).where(
                    Task.rack_id == test_rack_id,
                    Task.work_id == 2000201
                )
            ).all()

            if tasks:
                task = tasks[0]
                print(f"✅ 成功創建裝載任務（房間3）")
                print(f"   - 任務ID: {task.id}")
                print(f"   - Work ID: {task.work_id}")
                print(f"   - Rack ID: {task.rack_id}")
                print(f"   - Room ID: {task.room_id}")
                print(f"   - 描述: {task.description}")

                # 驗證任務屬性（關鍵：room_id 應為 3）
                assert task.work_id == 2000201, "Work ID 應為 2000201"
                assert task.rack_id == test_rack_id, "Rack ID 不符"
                assert task.room_id == 3, f"Room ID 應為 3，實際為 {task.room_id}"
                assert task.status_id == 1, "任務狀態應為 PENDING"

                # 驗證 parameters 中的資料
                import json
                if task.parameters:
                    params = json.loads(task.parameters) if isinstance(task.parameters, str) else task.parameters
                    assert params.get('room_id') == 3, f"Parameters 中的 room_id 應為 3，實際為 {params.get('room_id')}"
                    assert params.get('boxout_carriers') == 6, f"出口傳送箱載具應為 6，實際為 {params.get('boxout_carriers')}"

                return True
            else:
                print("❌ 未創建裝載任務")
                return False

    except Exception as e:
        print(f"❌ 測試失敗: {e}")
        import traceback
        traceback.print_exc()
        return False

# ============================================================================
# 主測試執行
# ============================================================================

async def main():
    """主測試執行函數"""
    print("\n" + "="*70)
    print("🧪 Cargo AGV 流程完整測試套件")
    print("="*70)

    test_results = []

    # 入口卸載測試
    print("\n" + "="*70)
    print("📥 入口卸載測試（房間1 - 3個場景）")
    print("="*70)

    result1 = await test_entrance_unload_with_work_carriers()
    test_results.append(("入口卸載 - 正常", result1))

    result2 = await test_entrance_unload_no_work_carriers()
    test_results.append(("入口卸載 - 無待處理", result2))

    result3 = await test_entrance_unload_duplicate_prevention()
    test_results.append(("入口卸載 - 防重複", result3))

    # 出口裝載測試
    print("\n" + "="*70)
    print("📤 出口裝載測試（房間1 - 4個場景）")
    print("="*70)

    result4 = await test_exit_load_normal()
    test_results.append(("出口裝載 - 正常", result4))

    result5 = await test_exit_load_rack_full()
    test_results.append(("出口裝載 - Rack已滿", result5))

    result6 = await test_exit_load_no_completed_carriers()
    test_results.append(("出口裝載 - 無已完成", result6))

    result7 = await test_exit_load_duplicate_prevention()
    test_results.append(("出口裝載 - 防重複", result7))

    # 多房間支援測試
    print("\n" + "="*70)
    print("🏢 多房間支援測試（2個場景）")
    print("="*70)

    result8 = await test_room2_entrance_unload()
    test_results.append(("房間2入口 - 多房間支援", result8))

    result9 = await test_room3_exit_load()
    test_results.append(("房間3出口 - 多房間支援", result9))

    # 統計結果
    print("\n" + "="*70)
    print("📊 測試結果總結")
    print("="*70)

    passed = sum(1 for _, result in test_results if result)
    total = len(test_results)

    for name, result in test_results:
        status = "✅ 通過" if result else "❌ 失敗"
        print(f"{status} - {name}")

    print(f"\n總計: {passed}/{total} 個測試通過")

    if passed == total:
        print("\n🎉 所有測試通過！")
        return True
    else:
        print(f"\n⚠️ 有 {total - passed} 個測試失敗")
        return False

if __name__ == "__main__":
    success = asyncio.run(main())
    sys.exit(0 if success else 1)
