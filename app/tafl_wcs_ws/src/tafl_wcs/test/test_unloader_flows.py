#!/usr/bin/env python3
"""測試 Unloader AGV 4 個流程的完整集成測試

⚠️ 已棄用 (DEPRECATED) - 2025-11-18
此測試文件使用已移除的 Task.type 字段，不再適用於當前系統。
TAFL WCS 系統已被 KUKA WCS 取代，請使用 kuka_wcs_ws 進行相關測試。
"""

import sys
import asyncio
from pathlib import Path

sys.path.insert(0, '/app/tafl_wcs_ws/src')
sys.path.insert(0, '/app/db_proxy_ws/src')
sys.path.insert(0, '/app/tafl_ws/src')

from tafl_wcs.tafl_executor_wrapper import TAFLExecutorWrapper
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import AGV, Carrier, Task, Room, Eqp, EqpPort
from sqlmodel import select, delete
from sqlalchemy import update

# 測試用 ID 範圍
TEST_ROOM_ID = 992
TEST_AGV_ID = 9002
TEST_CARRIER_BASE = 92000

# 生產環境設備 ID（與流程一致）
PRE_DRYER_EQP_ID = 205  # 預烘機
OVEN_EQP_ID = 206       # 烘箱
BOXOUT_EQP_ID = 202     # 出口傳送箱

# 生產環境 Port ID（與流程一致）
PRE_DRYER_STATION1_PORTS = [2051, 2052, 2055, 2056]  # 預烘機 Station 1
PRE_DRYER_STATION3_PORTS = [2053, 2054, 2057, 2058]  # 預烘機 Station 3
OVEN_UPPER_PORTS = [2061, 2062, 2063, 2064]          # 烘箱上排
OVEN_LOWER_PORTS = [2065, 2066, 2067, 2068]          # 烘箱下排
BOXOUT_PORTS = [2021, 2022, 2023, 2024]              # 出口傳送箱

# TAFL 流程文件目錄
FLOW_BASE_DIR = Path("/app/config/tafl/flows")


async def test_1_unloader_take_pre_dryer():
    """測試 1: Unloader AGV 從預烘機取料 (自定義 Port 映射)"""
    print("\n" + "="*60)
    print("測試 1: Unloader AGV 從預烘機取料 (自定義 Port 映射)")
    print("="*60)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    test_equipment_id = PRE_DRYER_EQP_ID  # 205
    # Station 1: ports [2051, 2052, 2055, 2056]
    test_carrier_ids = list(range(TEST_CARRIER_BASE + 1, TEST_CARRIER_BASE + 5))  # 4 carriers
    test_port_ids = PRE_DRYER_STATION1_PORTS  # [2051, 2052, 2055, 2056]

    try:
        # 清理舊資料（不刪除生產環境的設備和 Port）
        print("\n🧹 清理舊資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
            session.exec(delete(Carrier).where(Carrier.id.in_(test_carrier_ids)))
            # 清理測試 Port 上的生產環境 Carriers（避免 Port 被佔用）
            session.exec(delete(Carrier).where(Carrier.port_id.in_(test_port_ids)))
            session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
            session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
            session.commit()
        print("✅ 清理完成")

        # 創建測試資料
        print("\n📝 創建測試資料...")
        with pool_manager.get_session() as session:
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
                model="Unloader",
                x=0.0,
                y=0.0,
                heading=0.0,
                enable=1
            )
            session.add(agv)

            # 假設設備和 Port 已存在於生產環境（設備ID 205, Port [2051, 2052, 2055, 2056]）

            # 創建 Carriers (預烘完成)
            for i, carrier_id in enumerate(test_carrier_ids):
                carrier = Carrier(
                    id=carrier_id,
                    room_id=TEST_ROOM_ID,
                    port_id=test_port_ids[i],
                    status_id=503  # 預烘完成
                )
                session.add(carrier)

            session.commit()
            print("✅ 測試資料創建完成")
            print(f"   Room: {TEST_ROOM_ID} (enabled=1)")
            print(f"   AGV: TEST_UNLOADER01 (車上載具: 0個, 有空位)")
            print(f"   Equipment {test_equipment_id} Port {test_port_ids}: 4個 Carriers (status=503)")
            print(f"   現有 Tasks: 0個")

        # 執行流程
        print("\n🚀 執行流程: unloader_take_pre_dryer.yaml")
        with open('/app/config/tafl/flows/unloader_take_pre_dryer.yaml', 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        result = await flow_executor.execute_flow(flow_yaml)
        print(f"📊 執行結果: {result.get('status')}")

        # 驗證任務創建
        print("\n✅ 測試後狀態:")
        with pool_manager.get_session() as session:
            tasks = session.exec(
                select(Task).where(
                    Task.room_id == TEST_ROOM_ID,
                    Task.type == "unloader_take"
                )
            ).all()

            if tasks:
                task = tasks[0]
                print(f"  - 創建 Task: {len(tasks)}個 (work_id={task.work_id})")
                print(f"  - 任務參數:")
                if task.parameters:
                    print(f"    * equipment_id: {task.parameters.get('equipment_id')}")
                    print(f"    * station: {task.parameters.get('station')}")
                    print(f"    * ports: {task.parameters.get('ports')}")
                    print(f"    * batch_size: {task.parameters.get('batch_size')}")
                    print(f"    * model: {task.parameters.get('model')}")
                    print(f"    * carrier_count: {task.parameters.get('carrier_count')}")

                # 驗證 work_id 和 batch size
                expected_work_ids = [2051101, 2051301]
                if task.work_id in expected_work_ids:
                    if task.parameters and task.parameters.get('batch_size') == 4:
                        print(f"  - 目的地驗證: ✅ 正確 (預烘機，自定義映射，batch_size=4)")
                        return True
                    else:
                        print(f"  - 目的地驗證: ❌ 錯誤 (batch_size 應為 4)")
                        return False
                else:
                    print(f"  - 目的地驗證: ❌ 錯誤 (work_id 不在預期範圍)")
                    return False
            else:
                print("  - ❌ 未創建任務")
                return False

    finally:
        # 清理（不刪除生產環境的設備和 Port）
        print("\n🧹 清理測試資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
            session.exec(delete(Carrier).where(Carrier.id.in_(test_carrier_ids)))
            session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
            session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
            session.commit()
        print("✅ 清理完成")


async def test_2_unloader_put_oven():
    """測試 2: Unloader AGV 放料到烘箱 (固定方向：下排)"""
    print("\n" + "="*60)
    print("測試 2: Unloader AGV 放料到烘箱 (固定方向：下排)")
    print("="*60)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    test_equipment_id = OVEN_EQP_ID  # 206
    test_carrier_ids = list(range(TEST_CARRIER_BASE + 10, TEST_CARRIER_BASE + 14))  # 4 carriers
    test_port_ids = OVEN_LOWER_PORTS  # [2065, 2066, 2067, 2068]

    try:
        # 清理舊資料（不刪除生產環境的設備和 Port）
        print("\n🧹 清理舊資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
            session.exec(delete(Carrier).where(Carrier.id.in_(test_carrier_ids)))
            # 清理測試 Port 上的生產環境 Carriers（避免 Port 被佔用）
            session.exec(delete(Carrier).where(Carrier.port_id.in_(test_port_ids)))
            session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
            session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
            session.commit()
        print("✅ 清理完成")

        # 創建測試資料
        print("\n📝 創建測試資料...")
        with pool_manager.get_session() as session:
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
                model="Unloader",
                x=0.0,
                y=0.0,
                heading=0.0,
                enable=1
            )
            session.add(agv)

            # 假設設備和 Port 已存在於生產環境（設備ID 206, Port [2065, 2066, 2067, 2068]）

            # 創建 Carriers (在 AGV 車上，必須設定 agv_id）
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
            print("✅ 測試資料創建完成")
            print(f"   Room: {TEST_ROOM_ID} (enabled=1)")
            print(f"   AGV: TEST_UNLOADER01 (車上載具: 4個)")
            print(f"   Equipment {test_equipment_id} Port {test_port_ids}: Empty (下排)")
            print(f"   現有 Tasks: 0個")

        # 執行流程
        print("\n🚀 執行流程: unloader_put_oven.yaml")
        with open('/app/config/tafl/flows/unloader_put_oven.yaml', 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        result = await flow_executor.execute_flow(flow_yaml)
        print(f"📊 執行結果: {result.get('status')}")

        # 驗證任務創建
        print("\n✅ 測試後狀態:")
        with pool_manager.get_session() as session:
            tasks = session.exec(
                select(Task).where(
                    Task.room_id == TEST_ROOM_ID,
                    Task.type == "unloader_put"
                )
            ).all()

            if tasks:
                task = tasks[0]
                print(f"  - 創建 Task: {len(tasks)}個 (work_id={task.work_id})")
                print(f"  - 任務參數:")
                if task.parameters:
                    print(f"    * equipment_id: {task.parameters.get('equipment_id')}")
                    print(f"    * station: {task.parameters.get('station')}")
                    print(f"    * ports: {task.parameters.get('ports')}")
                    print(f"    * row: {task.parameters.get('row')}")
                    print(f"    * batch_size: {task.parameters.get('batch_size')}")
                    print(f"    * model: {task.parameters.get('model')}")

                # 驗證 work_id 和固定方向
                if task.work_id == 2060502:
                    if task.parameters and task.parameters.get('row') == 'lower':
                        print(f"  - 目的地驗證: ✅ 正確 (烘箱 Station 05 下排，固定方向)")
                        return True
                    else:
                        print(f"  - 目的地驗證: ❌ 錯誤 (row 應為 lower)")
                        return False
                else:
                    print(f"  - 目的地驗證: ❌ 錯誤 (work_id 應為 2060502)")
                    return False
            else:
                print("  - ❌ 未創建任務")
                return False

    finally:
        # 清理（不刪除生產環境的設備和 Port）
        print("\n🧹 清理測試資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
            session.exec(delete(Carrier).where(Carrier.id.in_(test_carrier_ids)))
            session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
            session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
            session.commit()
        print("✅ 清理完成")


async def test_3_unloader_take_oven():
    """測試 3: Unloader AGV 從烘箱取料 (固定方向：上排)"""
    print("\n" + "="*60)
    print("測試 3: Unloader AGV 從烘箱取料 (固定方向：上排)")
    print("="*60)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    test_equipment_id = OVEN_EQP_ID  # 206
    test_carrier_ids = list(range(TEST_CARRIER_BASE + 20, TEST_CARRIER_BASE + 24))  # 4 carriers
    test_port_ids = OVEN_UPPER_PORTS  # [2061, 2062, 2063, 2064]

    try:
        # 清理舊資料（不刪除生產環境的設備和 Port）
        print("\n🧹 清理舊資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
            session.exec(delete(Carrier).where(Carrier.id.in_(test_carrier_ids)))
            # 清理測試 Port 上的生產環境 Carriers（避免 Port 被佔用）
            session.exec(delete(Carrier).where(Carrier.port_id.in_(test_port_ids)))
            session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
            session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
            session.commit()
        print("✅ 清理完成")

        # 創建測試資料
        print("\n📝 創建測試資料...")
        with pool_manager.get_session() as session:
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
                model="Unloader",
                x=0.0,
                y=0.0,
                heading=0.0,
                enable=1
            )
            session.add(agv)

            # 假設設備和 Port 已存在於生產環境（設備ID 206, Port [2061, 2062, 2063, 2064]）

            # 創建 Carriers (烘乾完成)
            for i, carrier_id in enumerate(test_carrier_ids):
                carrier = Carrier(
                    id=carrier_id,
                    room_id=TEST_ROOM_ID,
                    port_id=test_port_ids[i],
                    status_id=603  # 烘乾完成
                )
                session.add(carrier)

            session.commit()
            print("✅ 測試資料創建完成")
            print(f"   Room: {TEST_ROOM_ID} (enabled=1)")
            print(f"   AGV: TEST_UNLOADER01 (車上載具: 0個, 有空位)")
            print(f"   Equipment {test_equipment_id} Port {test_port_ids}: 4個 Carriers (status=603)")
            print(f"   現有 Tasks: 0個")

        # 執行流程
        print("\n🚀 執行流程: unloader_take_oven.yaml")
        with open('/app/config/tafl/flows/unloader_take_oven.yaml', 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        result = await flow_executor.execute_flow(flow_yaml)
        print(f"📊 執行結果: {result.get('status')}")

        # 驗證任務創建
        print("\n✅ 測試後狀態:")
        with pool_manager.get_session() as session:
            tasks = session.exec(
                select(Task).where(
                    Task.room_id == TEST_ROOM_ID,
                    Task.type == "unloader_take"
                )
            ).all()

            if tasks:
                task = tasks[0]
                print(f"  - 創建 Task: {len(tasks)}個 (work_id={task.work_id})")
                print(f"  - 任務參數:")
                if task.parameters:
                    print(f"    * equipment_id: {task.parameters.get('equipment_id')}")
                    print(f"    * station: {task.parameters.get('station')}")
                    print(f"    * ports: {task.parameters.get('ports')}")
                    print(f"    * row: {task.parameters.get('row')}")
                    print(f"    * batch_size: {task.parameters.get('batch_size')}")
                    print(f"    * model: {task.parameters.get('model')}")
                    print(f"    * carrier_count: {task.parameters.get('carrier_count')}")

                # 驗證 work_id 和固定方向
                if task.work_id == 2060101:
                    if task.parameters and task.parameters.get('row') == 'upper':
                        print(f"  - 目的地驗證: ✅ 正確 (烘箱 Station 01 上排，固定方向)")
                        return True
                    else:
                        print(f"  - 目的地驗證: ❌ 錯誤 (row 應為 upper)")
                        return False
                else:
                    print(f"  - 目的地驗證: ❌ 錯誤 (work_id 應為 2060101)")
                    return False
            else:
                print("  - ❌ 未創建任務")
                return False

    finally:
        # 清理（不刪除生產環境的設備和 Port）
        print("\n🧹 清理測試資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
            session.exec(delete(Carrier).where(Carrier.id.in_(test_carrier_ids)))
            session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
            session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
            session.commit()
        print("✅ 清理完成")


async def test_4_unloader_put_boxout_transfer():
    """測試 4: Unloader AGV 放料到出口傳送箱 (完成後段制程)"""
    print("\n" + "="*60)
    print("測試 4: Unloader AGV 放料到出口傳送箱 (完成後段制程)")
    print("="*60)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    test_equipment_id = BOXOUT_EQP_ID  # 202
    test_carrier_ids = list(range(TEST_CARRIER_BASE + 30, TEST_CARRIER_BASE + 34))  # 4 carriers
    test_port_ids = BOXOUT_PORTS  # [2021, 2022, 2023, 2024]

    try:
        # 清理舊資料（不刪除生產環境的設備和 Port）
        print("\n🧹 清理舊資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
            session.exec(delete(Carrier).where(Carrier.id.in_(test_carrier_ids)))
            # 清理測試 Port 上的生產環境 Carriers（避免 Port 被佔用）
            session.exec(delete(Carrier).where(Carrier.port_id.in_(test_port_ids)))
            session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
            session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
            session.commit()
        print("✅ 清理完成")

        # 創建測試資料
        print("\n📝 創建測試資料...")
        with pool_manager.get_session() as session:
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
                model="Unloader",
                x=0.0,
                y=0.0,
                heading=0.0,
                enable=1
            )
            session.add(agv)

            # 假設設備和 Port 已存在於生產環境（設備ID 202, Port [2021, 2022, 2023, 2024]）

            # 創建 Carriers (在 AGV 車上，必須設定 agv_id）
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
            print("✅ 測試資料創建完成")
            print(f"   Room: {TEST_ROOM_ID} (enabled=1)")
            print(f"   AGV: TEST_UNLOADER01 (車上載具: 4個)")
            print(f"   Equipment {test_equipment_id} Port {test_port_ids}: Empty")
            print(f"   現有 Tasks: 0個")

        # 執行流程
        print("\n🚀 執行流程: unloader_put_boxout_transfer.yaml")
        with open('/app/config/tafl/flows/unloader_put_boxout_transfer.yaml', 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        result = await flow_executor.execute_flow(flow_yaml)
        print(f"📊 執行結果: {result.get('status')}")

        # 驗證任務創建
        print("\n✅ 測試後狀態:")
        with pool_manager.get_session() as session:
            tasks = session.exec(
                select(Task).where(
                    Task.room_id == TEST_ROOM_ID,
                    Task.type == "unloader_put"
                )
            ).all()

            if tasks:
                task = tasks[0]
                print(f"  - 創建 Task: {len(tasks)}個 (work_id={task.work_id})")
                print(f"  - 任務參數:")
                if task.parameters:
                    print(f"    * equipment_id: {task.parameters.get('equipment_id')}")
                    print(f"    * station: {task.parameters.get('station')}")
                    print(f"    * ports: {task.parameters.get('ports')}")
                    print(f"    * batch_size: {task.parameters.get('batch_size')}")
                    print(f"    * model: {task.parameters.get('model')}")
                    print(f"    * reason: {task.parameters.get('reason')}")

                # 驗證 work_id (完成後段制程，銜接 Cargo AGV)
                if task.work_id == 2020102:
                    print(f"  - 目的地驗證: ✅ 正確 (出口傳送箱，完成後段制程，銜接 Cargo AGV)")
                    return True
                else:
                    print(f"  - 目的地驗證: ❌ 錯誤 (work_id 應為 2020102)")
                    return False
            else:
                print("  - ❌ 未創建任務")
                return False

    finally:
        # 清理（不刪除生產環境的設備和 Port）
        print("\n🧹 清理測試資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
            session.exec(delete(Carrier).where(Carrier.id.in_(test_carrier_ids)))
            session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
            session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
            session.commit()
        print("✅ 清理完成")


# ============================================================================
# 測試 5-8: 重複防護測試
# ============================================================================

async def test_5_unloader_take_pre_dryer_duplicate():
    """
    測試 5: Unloader 從預烘機取料 - 重複執行防護

    驗證項目:
    - 連續執行流程 3 次
    - 應只創建 1 個任務（從 2 個可能的 Station 中選擇 1 個）
    - 防護機制正常運作
    """
    print("\n" + "="*60)
    print("測試 5: Unloader 從預烘機取料 - 重複執行防護")
    print("="*60)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    test_carrier_ids = list(range(TEST_CARRIER_BASE + 1, TEST_CARRIER_BASE + 15))  # 最多 14 個

    try:
        # 清理舊資料
        print("\n🧹 清理舊資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
            session.exec(delete(Carrier).where(Carrier.id.in_(test_carrier_ids)))
            session.exec(delete(Carrier).where(Carrier.port_id.in_(PRE_DRYER_STATION1_PORTS + PRE_DRYER_STATION3_PORTS)))
            session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
            session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
            session.commit()
        print("✅ 清理完成")

        # 讀取流程配置
        flow_file = FLOW_BASE_DIR / "unloader_take_pre_dryer.yaml"
        with open(flow_file, 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        # 創建測試資料
        print("\n📝 創建測試資料...")
        with pool_manager.get_session() as session:
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
                model="Unloader",
                x=0.0,
                y=0.0,
                heading=0.0,
                enable=1
            )
            session.add(agv)

            # 只準備 Station 1 的 4 個料架 (避免兩個 Station 同時滿足條件)
            for i, port_id in enumerate(PRE_DRYER_STATION1_PORTS):
                carrier_id = TEST_CARRIER_BASE + i + 1
                carrier = Carrier(
                    id=carrier_id,
                    room_id=TEST_ROOM_ID,
                    port_id=port_id,
                    status_id=503  # 預烘完成
                )
                session.add(carrier)

            session.commit()
            print("✅ 測試資料創建完成")
            print(f"   Room: {TEST_ROOM_ID} (enabled=1)")
            print(f"   AGV: {TEST_AGV_ID} (model=UNLOADER)")
            print(f"   預烘機 Station 1 Ports {PRE_DRYER_STATION1_PORTS}: 4個 Carriers")
            print(f"   預烘機 Station 3: 無料架 (確保只有一個 Station 滿足條件)")

        # 連續執行流程 3 次
        print("\n🚀 開始執行流程測試...")
        for i in range(1, 4):
            print(f"\n   第 {i} 次執行:")
            result = await flow_executor.execute_flow(flow_yaml)
            print(f"   📊 執行結果: {result.get('status', 'unknown')}")

        # 驗證結果
        print("\n✅ 測試後狀態:")
        with pool_manager.get_session() as session:
            tasks = session.exec(
                select(Task).where(
                    Task.room_id == TEST_ROOM_ID,
                    Task.work_id.in_([2051101, 2051301])
                )
            ).all()

            print(f"  - 創建 Task: {len(tasks)}個")

            if len(tasks) == 1:
                task = tasks[0]
                print(f"  - Task ID: {task.id}, Work ID: {task.work_id}")
                print(f"  - ✅ 防護機制正常！只創建了 1 個任務")

                if task.work_id in [2051101, 2051301]:
                    station_num = 1 if task.work_id == 2051101 else 3
                    print(f"  - 目的地驗證: ✅ 正確 (預烘機 Station {station_num})")
                    return True
                else:
                    print(f"  - 目的地驗證: ❌ 錯誤 (work_id 應為 2051101 或 2051301)")
                    return False
            else:
                print(f"  - ❌ 防護機制失敗！創建了 {len(tasks)} 個任務")
                return False

    finally:
        # 清理測試資料
        print("\n🧹 清理測試資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
            session.exec(delete(Carrier).where(Carrier.id.in_(test_carrier_ids)))
            session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
            session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
            session.commit()
        print("✅ 清理完成")


async def test_6_unloader_put_oven_duplicate():
    """
    測試 6: Unloader 放料到烘箱 - 重複執行防護

    驗證項目:
    - 連續執行流程 3 次
    - 應只創建 1 個任務（固定 Station 5）
    - 防護機制正常運作
    """
    print("\n" + "="*60)
    print("測試 6: Unloader 放料到烘箱 - 重複執行防護")
    print("="*60)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    test_carrier_ids = list(range(TEST_CARRIER_BASE + 1, TEST_CARRIER_BASE + 5))

    try:
        # 清理舊資料
        print("\n🧹 清理舊資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
            session.exec(delete(Carrier).where(Carrier.id.in_(test_carrier_ids)))
            session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
            session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
            session.commit()
        print("✅ 清理完成")

        # 讀取流程配置
        flow_file = FLOW_BASE_DIR / "unloader_put_oven.yaml"
        with open(flow_file, 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        # 創建測試資料
        print("\n📝 創建測試資料...")
        with pool_manager.get_session() as session:
            # 創建測試 Room
            room = Room(
                id=TEST_ROOM_ID,
                name="TEST_ROOM_UNLOADER",
                process_settings_id=1,
                enable=1
            )
            session.add(room)

            # 創建 Unloader AGV (車上有 4 個完成的料架)
            agv = AGV(
                id=TEST_AGV_ID,
                name="TEST_UNLOADER01",
                model="Unloader",
                x=0.0,
                y=0.0,
                heading=0.0,
                enable=1
            )
            session.add(agv)

            # 在 AGV 上放置 4 個料架
            for i in range(4):
                carrier_id = TEST_CARRIER_BASE + i + 1
                carrier = Carrier(
                    id=carrier_id,
                    room_id=TEST_ROOM_ID,
                    agv_id=TEST_AGV_ID,
                    status_id=603  # 烘烤完成
                )
                session.add(carrier)

            session.commit()
            print("✅ 測試資料創建完成")
            print(f"   Room: {TEST_ROOM_ID} (enabled=1)")
            print(f"   AGV: {TEST_AGV_ID} (model=UNLOADER, 載具=4個)")

        # 連續執行流程 3 次
        print("\n🚀 開始執行流程測試...")
        for i in range(1, 4):
            print(f"\n   第 {i} 次執行:")
            result = await flow_executor.execute_flow(flow_yaml)
            print(f"   📊 執行結果: {result.get('status', 'unknown')}")

        # 驗證結果
        print("\n✅ 測試後狀態:")
        with pool_manager.get_session() as session:
            tasks = session.exec(
                select(Task).where(
                    Task.room_id == TEST_ROOM_ID,
                    Task.work_id == 2060502
                )
            ).all()

            print(f"  - 創建 Task: {len(tasks)}個")

            if len(tasks) == 1:
                task = tasks[0]
                print(f"  - Task ID: {task.id}, Work ID: {task.work_id}")
                print(f"  - ✅ 防護機制正常！只創建了 1 個任務")
                print(f"  - 目的地驗證: ✅ 正確 (烘箱 Station 5 下層)")
                return True
            else:
                print(f"  - ❌ 防護機制失敗！創建了 {len(tasks)} 個任務")
                return False

    finally:
        # 清理測試資料
        print("\n🧹 清理測試資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
            session.exec(delete(Carrier).where(Carrier.id.in_(test_carrier_ids)))
            session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
            session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
            session.commit()
        print("✅ 清理完成")


async def test_7_unloader_take_oven_duplicate():
    """
    測試 7: Unloader 從烘箱取料 - 重複執行防護

    驗證項目:
    - 連續執行流程 3 次
    - 應只創建 1 個任務（固定 Station 1）
    - 防護機制正常運作
    """
    print("\n" + "="*60)
    print("測試 7: Unloader 從烘箱取料 - 重複執行防護")
    print("="*60)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    test_carrier_ids = list(range(TEST_CARRIER_BASE + 1, TEST_CARRIER_BASE + 5))

    try:
        # 清理舊資料
        print("\n🧹 清理舊資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
            session.exec(delete(Carrier).where(Carrier.id.in_(test_carrier_ids)))
            session.exec(delete(Carrier).where(Carrier.port_id.in_(OVEN_UPPER_PORTS)))
            session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
            session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
            session.commit()
        print("✅ 清理完成")

        # 讀取流程配置
        flow_file = FLOW_BASE_DIR / "unloader_take_oven.yaml"
        with open(flow_file, 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        # 創建測試資料
        print("\n📝 創建測試資料...")
        with pool_manager.get_session() as session:
            # 創建測試 Room
            room = Room(
                id=TEST_ROOM_ID,
                name="TEST_ROOM_UNLOADER",
                process_settings_id=1,
                enable=1
            )
            session.add(room)

            # 創建 Unloader AGV (空車)
            agv = AGV(
                id=TEST_AGV_ID,
                name="TEST_UNLOADER01",
                model="Unloader",
                x=0.0,
                y=0.0,
                heading=0.0,
                enable=1
            )
            session.add(agv)

            # 在烘箱 Station 1 上層放置 4 個料架
            for i, port_id in enumerate(OVEN_UPPER_PORTS):
                carrier_id = TEST_CARRIER_BASE + i + 1
                carrier = Carrier(
                    id=carrier_id,
                    room_id=TEST_ROOM_ID,
                    port_id=port_id,
                    status_id=603  # 烘烤完成
                )
                session.add(carrier)

            session.commit()
            print("✅ 測試資料創建完成")
            print(f"   Room: {TEST_ROOM_ID} (enabled=1)")
            print(f"   AGV: {TEST_AGV_ID} (model=UNLOADER)")
            print(f"   烘箱 Station 1 Ports {OVEN_UPPER_PORTS}: 4個 Carriers")

        # 連續執行流程 3 次
        print("\n🚀 開始執行流程測試...")
        for i in range(1, 4):
            print(f"\n   第 {i} 次執行:")
            result = await flow_executor.execute_flow(flow_yaml)
            print(f"   📊 執行結果: {result.get('status', 'unknown')}")

        # 驗證結果
        print("\n✅ 測試後狀態:")
        with pool_manager.get_session() as session:
            tasks = session.exec(
                select(Task).where(
                    Task.room_id == TEST_ROOM_ID,
                    Task.work_id == 2060101
                )
            ).all()

            print(f"  - 創建 Task: {len(tasks)}個")

            if len(tasks) == 1:
                task = tasks[0]
                print(f"  - Task ID: {task.id}, Work ID: {task.work_id}")
                print(f"  - ✅ 防護機制正常！只創建了 1 個任務")
                print(f"  - 目的地驗證: ✅ 正確 (烘箱 Station 1 上層)")
                return True
            else:
                print(f"  - ❌ 防護機制失敗！創建了 {len(tasks)} 個任務")
                return False

    finally:
        # 清理測試資料
        print("\n🧹 清理測試資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
            session.exec(delete(Carrier).where(Carrier.id.in_(test_carrier_ids)))
            session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
            session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
            session.commit()
        print("✅ 清理完成")


async def test_8_unloader_put_boxout_duplicate():
    """
    測試 8: Unloader 放料到出口傳送箱 - 重複執行防護

    驗證項目:
    - 連續執行流程 3 次
    - 應只創建 1 個任務（固定 Station 1）
    - 防護機制正常運作
    """
    print("\n" + "="*60)
    print("測試 8: Unloader 放料到出口傳送箱 - 重複執行防護")
    print("="*60)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    test_carrier_ids = list(range(TEST_CARRIER_BASE + 1, TEST_CARRIER_BASE + 5))

    try:
        # 清理舊資料
        print("\n🧹 清理舊資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
            session.exec(delete(Carrier).where(Carrier.id.in_(test_carrier_ids)))
            session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
            session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
            session.commit()
        print("✅ 清理完成")

        # 讀取流程配置
        flow_file = FLOW_BASE_DIR / "unloader_put_boxout_transfer.yaml"
        with open(flow_file, 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        # 創建測試資料
        print("\n📝 創建測試資料...")
        with pool_manager.get_session() as session:
            # 創建測試 Room
            room = Room(
                id=TEST_ROOM_ID,
                name="TEST_ROOM_UNLOADER",
                process_settings_id=1,
                enable=1
            )
            session.add(room)

            # 創建 Unloader AGV (車上有 4 個完成的料架)
            agv = AGV(
                id=TEST_AGV_ID,
                name="TEST_UNLOADER01",
                model="Unloader",
                x=0.0,
                y=0.0,
                heading=0.0,
                enable=1
            )
            session.add(agv)

            # 在 AGV 上放置 4 個料架
            for i in range(4):
                carrier_id = TEST_CARRIER_BASE + i + 1
                carrier = Carrier(
                    id=carrier_id,
                    room_id=TEST_ROOM_ID,
                    agv_id=TEST_AGV_ID,
                    status_id=603  # 烘烤完成
                )
                session.add(carrier)

            session.commit()
            print("✅ 測試資料創建完成")
            print(f"   Room: {TEST_ROOM_ID} (enabled=1)")
            print(f"   AGV: {TEST_AGV_ID} (model=UNLOADER, 載具=4個)")

        # 連續執行流程 3 次
        print("\n🚀 開始執行流程測試...")
        for i in range(1, 4):
            print(f"\n   第 {i} 次執行:")
            result = await flow_executor.execute_flow(flow_yaml)
            print(f"   📊 執行結果: {result.get('status', 'unknown')}")

        # 驗證結果
        print("\n✅ 測試後狀態:")
        with pool_manager.get_session() as session:
            tasks = session.exec(
                select(Task).where(
                    Task.room_id == TEST_ROOM_ID,
                    Task.work_id == 2020102
                )
            ).all()

            print(f"  - 創建 Task: {len(tasks)}個")

            if len(tasks) == 1:
                task = tasks[0]
                print(f"  - Task ID: {task.id}, Work ID: {task.work_id}")
                print(f"  - ✅ 防護機制正常！只創建了 1 個任務")
                print(f"  - 目的地驗證: ✅ 正確 (出口傳送箱，完成後段制程)")
                return True
            else:
                print(f"  - ❌ 防護機制失敗！創建了 {len(tasks)} 個任務")
                return False

    finally:
        # 清理測試資料
        print("\n🧹 清理測試資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
            session.exec(delete(Carrier).where(Carrier.id.in_(test_carrier_ids)))
            session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
            session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
            session.commit()
        print("✅ 清理完成")


async def main():
    """執行所有 Unloader AGV 測試"""
    print("\n" + "="*60)
    print("Unloader AGV 完整測試 (4個基本功能 + 4個重複防護)")
    print("="*60)

    results = []

    # 測試 1-4: 基本功能測試
    print("\n" + "="*60)
    print("第一部分：基本功能測試 (4個)")
    print("="*60)

    results.append(("Unloader 從預烘機取料", await test_1_unloader_take_pre_dryer()))
    results.append(("Unloader 放料到烘箱", await test_2_unloader_put_oven()))
    results.append(("Unloader 從烘箱取料", await test_3_unloader_take_oven()))
    results.append(("Unloader 放料到出口傳送箱", await test_4_unloader_put_boxout_transfer()))

    # 測試 5-8: 重複防護測試
    print("\n" + "="*60)
    print("第二部分：重複防護測試 (4個)")
    print("="*60)

    results.append(("Unloader 取料重複防護 (預烘機)", await test_5_unloader_take_pre_dryer_duplicate()))
    results.append(("Unloader 放料重複防護 (烘箱)", await test_6_unloader_put_oven_duplicate()))
    results.append(("Unloader 取料重複防護 (烘箱)", await test_7_unloader_take_oven_duplicate()))
    results.append(("Unloader 放料重複防護 (出口箱)", await test_8_unloader_put_boxout_duplicate()))

    # 總結
    print("\n" + "="*60)
    print("測試總結")
    print("="*60)

    passed_tests = 0
    total_tests = len(results)

    for name, result in results:
        status = "✅ 通過" if result else "❌ 失敗"
        print(f"{status} - {name}")
        if result:
            passed_tests += 1

    all_passed = all(r for _, r in results)
    if all_passed:
        print(f"\n🎉 所有 Unloader AGV 測試通過！({passed_tests}/{total_tests})")
        return 0
    else:
        print(f"\n⚠️ 部分測試失敗 ({passed_tests}/{total_tests})")
        return 1


if __name__ == '__main__':
    exit_code = asyncio.run(main())
    sys.exit(exit_code)
