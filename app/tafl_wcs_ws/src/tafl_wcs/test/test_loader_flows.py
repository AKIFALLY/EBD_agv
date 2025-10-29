#!/usr/bin/env python3
"""測試 Loader AGV 6 個流程的完整集成測試

修復策略:
1. 不創建測試設備，假設生產環境設備已存在
2. 使用生產環境 Port IDs (Equipment 201, 203, 204, 205)
3. ⚠️ **關鍵差異**: Loader 與 Unloader 的 Carrier 設計不同
   - Unloader: Carrier 使用 agv_id (4個載具直接在 AGV 上)
   - Loader: Carrier 使用 port_id (2101-2104 代表 AGV 上的4個位置)

4. PUT 測試的 Carrier 必須包含 port_id 和正確的 status_id:
   - PUT 清洗機: port_id=2101 (Port 1), status_id=101 (從入口箱)
   - PUT 泡藥機: port_id=2101 (Port 1), status_id=303 (清洗完成)
   - PUT 預烘機: port_id=2102 (Port 2), status_id=403 (泡藥完成)
"""

import sys
import asyncio

sys.path.insert(0, '/app/tafl_wcs_ws/src')
sys.path.insert(0, '/app/db_proxy_ws/src')
sys.path.insert(0, '/app/tafl_ws/src')

from tafl_wcs.tafl_executor_wrapper import TAFLExecutorWrapper
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import AGV, Carrier, Task, Room
from sqlmodel import select, delete

# 測試用 ID 範圍
TEST_ROOM_ID = 991
TEST_AGV_ID = 9001
TEST_CARRIER_BASE = 90000

# 生產環境設備 IDs (假設已存在)
BOXIN_TRANSFER_EQUIPMENT = 201  # 入口傳送箱
CLEANER_EQUIPMENT = 203         # 清洗機
SOAKER_EQUIPMENT = 204          # 泡藥機
PRE_DRYER_EQUIPMENT = 205       # 預烘機


async def test_1_loader_take_boxin_transfer():
    """測試 1: Loader AGV 從入口傳送箱取料

    使用生產環境 Equipment 201 (入口傳送箱), Ports 2011, 2012
    """
    print("\n" + "="*60)
    print("測試 1: Loader AGV 從入口傳送箱取料")
    print("="*60)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    test_carrier_ids = [TEST_CARRIER_BASE + 1, TEST_CARRIER_BASE + 2]
    production_port_ids = [2011, 2012]  # Equipment 201 Ports

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

        # 創建測試資料
        print("\n📝 創建測試資料...")
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

            # 創建 Carriers (在入口傳送箱生產環境 Ports)
            for i, carrier_id in enumerate(test_carrier_ids):
                carrier = Carrier(
                    id=carrier_id,
                    room_id=TEST_ROOM_ID,
                    port_id=production_port_ids[i],
                    status_id=201  # 在入口傳送箱
                )
                session.add(carrier)

            session.commit()
            print("✅ 測試資料創建完成")
            print(f"   Room: {TEST_ROOM_ID} (enabled=1)")
            print(f"   AGV: TEST_LOADER01 (車上載具: 0個)")
            print(f"   Equipment {BOXIN_TRANSFER_EQUIPMENT} Ports {production_port_ids}: 2個 Carriers (status=201)")
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
                select(Task).where(
                    Task.room_id == TEST_ROOM_ID,
                    Task.type == "loader_take"
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

                # 驗證 work_id
                expected_work_ids = [2010101, 2010301]
                if task.work_id in expected_work_ids:
                    print(f"  - 目的地驗證: ✅ 正確 (入口傳送箱 Station)")
                    return True
                else:
                    print(f"  - 目的地驗證: ❌ 錯誤 (work_id 不正確)")
                    return False
            else:
                print("  - ❌ 未創建任務")
                return False

    finally:
        # 清理
        print("\n🧹 清理測試資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
            session.exec(delete(Carrier).where(Carrier.id.in_(test_carrier_ids)))
            session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
            session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
            session.commit()
        print("✅ 清理完成")


async def test_2_loader_put_cleaner():
    """測試 2: Loader AGV 放料到清洗機

    使用生產環境 Equipment 203 (清洗機), Ports 2033, 2034 (下層)
    **關鍵修復**: Carrier 必須包含 agv_id
    """
    print("\n" + "="*60)
    print("測試 2: Loader AGV 放料到清洗機")
    print("="*60)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    test_carrier_id = TEST_CARRIER_BASE + 3

    try:
        # 清理舊資料
        print("\n🧹 清理舊資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
            session.exec(delete(Carrier).where(Carrier.id == test_carrier_id))
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

            # 創建 Carrier (在 AGV Port 1 上) - 關鍵修復：使用 port_id 而非 agv_id
            carrier = Carrier(
                id=test_carrier_id,
                room_id=TEST_ROOM_ID,
                port_id=2101,  # ✅ 修復：Loader AGV Port 1 (2101)
                status_id=101  # 從入口箱取出的 carrier
            )
            session.add(carrier)

            session.commit()
            print("✅ 測試資料創建完成")
            print(f"   Room: {TEST_ROOM_ID} (enabled=1)")
            print(f"   AGV: TEST_LOADER01 (model=LOADER)")
            print(f"   Carrier: {test_carrier_id} (port_id=2101 (Port 1), status_id=101)")
            print(f"   Equipment {CLEANER_EQUIPMENT} Ports [2033,2034]: 假設有空位")
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
                select(Task).where(
                    Task.room_id == TEST_ROOM_ID,
                    Task.type == "loader_put"
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
                    print(f"    * model: {task.parameters.get('model')}")

                # 驗證 work_id
                if task.work_id == 2030302:
                    print(f"  - 目的地驗證: ✅ 正確 (清洗機 Station 03 下層)")
                    return True
                else:
                    print(f"  - 目的地驗證: ❌ 錯誤 (work_id 應為 2030302)")
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


async def test_3_loader_take_cleaner():
    """測試 3: Loader AGV 從清洗機取料

    使用生產環境 Equipment 203 (清洗機), Ports 2031, 2032 (上層)
    """
    print("\n" + "="*60)
    print("測試 3: Loader AGV 從清洗機取料")
    print("="*60)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    test_carrier_ids = [TEST_CARRIER_BASE + 5, TEST_CARRIER_BASE + 6]
    production_port_ids = [2031, 2032]  # Equipment 203 上層

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

        # 創建測試資料
        print("\n📝 創建測試資料...")
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

            # 創建 Carriers (清洗完成，在生產環境 Ports)
            for i, carrier_id in enumerate(test_carrier_ids):
                carrier = Carrier(
                    id=carrier_id,
                    room_id=TEST_ROOM_ID,
                    port_id=production_port_ids[i],
                    status_id=303  # 清洗完成
                )
                session.add(carrier)

            session.commit()
            print("✅ 測試資料創建完成")
            print(f"   Room: {TEST_ROOM_ID} (enabled=1)")
            print(f"   AGV: TEST_LOADER01 (車上載具: 0個, 有空位)")
            print(f"   Equipment {CLEANER_EQUIPMENT} Ports {production_port_ids}: 2個 Carriers (status=303)")
            print(f"   現有 Tasks: 0個")

        # 執行流程
        print("\n🚀 執行流程: loader_take_cleaner.yaml")
        with open('/app/config/tafl/flows/loader_take_cleaner.yaml', 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        result = await flow_executor.execute_flow(flow_yaml)
        print(f"📊 執行結果: {result.get('status')}")

        # 驗證任務創建
        print("\n✅ 測試後狀態:")
        with pool_manager.get_session() as session:
            tasks = session.exec(
                select(Task).where(
                    Task.room_id == TEST_ROOM_ID,
                    Task.type == "loader_take"
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
                    print(f"    * model: {task.parameters.get('model')}")

                # 驗證 work_id
                if task.work_id == 2030101:
                    print(f"  - 目的地驗證: ✅ 正確 (清洗機 Station 01 上層)")
                    return True
                else:
                    print(f"  - 目的地驗證: ❌ 錯誤 (work_id 應為 2030101)")
                    return False
            else:
                print("  - ❌ 未創建任務")
                return False

    finally:
        # 清理
        print("\n🧹 清理測試資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
            session.exec(delete(Carrier).where(Carrier.id.in_(test_carrier_ids)))
            session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
            session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
            session.commit()
        print("✅ 清理完成")


async def test_4_loader_put_soaker():
    """測試 4: Loader AGV 放料到泡藥機 (特殊設備)

    使用生產環境 Equipment 204 (泡藥機), Ports 2041-2046 (6個獨立 ports)
    **關鍵修復**: Carrier 必須包含 agv_id
    """
    print("\n" + "="*60)
    print("測試 4: Loader AGV 放料到泡藥機 (特殊設備)")
    print("="*60)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    test_carrier_id = TEST_CARRIER_BASE + 7

    try:
        # 清理舊資料
        print("\n🧹 清理舊資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
            session.exec(delete(Carrier).where(Carrier.id == test_carrier_id))
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

            # 創建 Carrier (在 AGV Port 1 上) - 關鍵修復：使用 port_id 而非 agv_id
            carrier = Carrier(
                id=test_carrier_id,
                room_id=TEST_ROOM_ID,
                port_id=2101,  # ✅ 修復：Loader AGV Port 1 (2101)
                status_id=303  # 清洗完成 (放入泡藥機前的狀態)
            )
            session.add(carrier)

            session.commit()
            print("✅ 測試資料創建完成")
            print(f"   Room: {TEST_ROOM_ID} (enabled=1)")
            print(f"   AGV: TEST_LOADER01 (model=LOADER)")
            print(f"   Carrier: {test_carrier_id} (port_id=2101 (Port 1), status_id=303)")
            print(f"   Equipment {SOAKER_EQUIPMENT} Ports [2041-2046]: 假設有空位")
            print(f"   現有 Tasks: 0個")

        # 執行流程
        print("\n🚀 執行流程: loader_put_soaker.yaml")
        with open('/app/config/tafl/flows/loader_put_soaker.yaml', 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        result = await flow_executor.execute_flow(flow_yaml)
        print(f"📊 執行結果: {result.get('status')}")

        # 驗證任務創建
        print("\n✅ 測試後狀態:")
        with pool_manager.get_session() as session:
            tasks = session.exec(
                select(Task).where(
                    Task.room_id == TEST_ROOM_ID,
                    Task.type == "loader_put"
                )
            ).all()

            if tasks:
                task = tasks[0]
                print(f"  - 創建 Task: {len(tasks)}個 (work_id={task.work_id})")
                print(f"  - 任務參數:")
                if task.parameters:
                    print(f"    * equipment_id: {task.parameters.get('equipment_id')}")
                    print(f"    * station: {task.parameters.get('station')}")
                    print(f"    * port: {task.parameters.get('port')}")  # 單一 port
                    print(f"    * batch_size: {task.parameters.get('batch_size')}")
                    print(f"    * model: {task.parameters.get('model')}")

                # 驗證 work_id (特殊設備: 6個 stations)
                expected_work_ids = [2040102, 2040202, 2040302, 2040402, 2040502, 2040602]
                if task.work_id in expected_work_ids:
                    print(f"  - 目的地驗證: ✅ 正確 (泡藥機特殊設備)")
                    return True
                else:
                    print(f"  - 目的地驗證: ❌ 錯誤 (work_id 不在預期範圍)")
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


async def test_5_loader_take_soaker():
    """測試 5: Loader AGV 從泡藥機取料

    使用生產環境 Equipment 204 (泡藥機), Ports 2041-2046 (6個獨立 ports)
    """
    print("\n" + "="*60)
    print("測試 5: Loader AGV 從泡藥機取料")
    print("="*60)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    test_carrier_ids = list(range(TEST_CARRIER_BASE + 20, TEST_CARRIER_BASE + 26))  # 6 carriers
    production_port_ids = list(range(2041, 2047))  # Equipment 204 Ports 2041-2046

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

        # 創建測試資料
        print("\n📝 創建測試資料...")
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

            # 創建 Carriers (泡藥完成，在生產環境 Ports)
            for i, carrier_id in enumerate(test_carrier_ids):
                carrier = Carrier(
                    id=carrier_id,
                    room_id=TEST_ROOM_ID,
                    port_id=production_port_ids[i],
                    status_id=403  # 泡藥完成
                )
                session.add(carrier)

            session.commit()
            print("✅ 測試資料創建完成")
            print(f"   Room: {TEST_ROOM_ID} (enabled=1)")
            print(f"   AGV: TEST_LOADER01 (車上載具: 0個, 有空位)")
            print(f"   Equipment {SOAKER_EQUIPMENT} Ports {production_port_ids}: 6個 Carriers (status=403)")
            print(f"   現有 Tasks: 0個")

        # 執行流程
        print("\n🚀 執行流程: loader_take_soaker.yaml")
        with open('/app/config/tafl/flows/loader_take_soaker.yaml', 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        result = await flow_executor.execute_flow(flow_yaml)
        print(f"📊 執行結果: {result.get('status')}")

        # 驗證任務創建
        print("\n✅ 測試後狀態:")
        with pool_manager.get_session() as session:
            tasks = session.exec(
                select(Task).where(
                    Task.room_id == TEST_ROOM_ID,
                    Task.type == "loader_take"
                )
            ).all()

            if tasks:
                task = tasks[0]
                print(f"  - 創建 Task: {len(tasks)}個 (work_id={task.work_id})")
                print(f"  - 任務參數:")
                if task.parameters:
                    print(f"    * equipment_id: {task.parameters.get('equipment_id')}")
                    print(f"    * station: {task.parameters.get('station')}")
                    print(f"    * port: {task.parameters.get('port')}")
                    print(f"    * batch_size: {task.parameters.get('batch_size')}")
                    print(f"    * model: {task.parameters.get('model')}")

                # 驗證 work_id
                expected_work_ids = [2040101, 2040201, 2040301, 2040401, 2040501, 2040601]
                if task.work_id in expected_work_ids:
                    print(f"  - 目的地驗證: ✅ 正確 (泡藥機特殊設備)")
                    return True
                else:
                    print(f"  - 目的地驗證: ❌ 錯誤 (work_id 不在預期範圍)")
                    return False
            else:
                print("  - ❌ 未創建任務")
                return False

    finally:
        # 清理
        print("\n🧹 清理測試資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
            session.exec(delete(Carrier).where(Carrier.id.in_(test_carrier_ids)))
            session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
            session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
            session.commit()
        print("✅ 清理完成")


async def test_6_loader_put_pre_dryer():
    """測試 6: Loader AGV 放料到預烘機 (銜接 Unloader)

    使用生產環境 Equipment 205 (預烘機), Ports 2051-2058 (4個 stations, 每個2 ports)
    **關鍵修復**: Carrier 必須包含 agv_id
    """
    print("\n" + "="*60)
    print("測試 6: Loader AGV 放料到預烘機 (銜接 Unloader)")
    print("="*60)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    test_carrier_id = TEST_CARRIER_BASE + 30

    try:
        # 清理舊資料
        print("\n🧹 清理舊資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
            session.exec(delete(Carrier).where(Carrier.id == test_carrier_id))
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

            # 創建 Carrier (在 AGV Port 2 上) - 關鍵修復：使用 port_id 而非 agv_id
            carrier = Carrier(
                id=test_carrier_id,
                room_id=TEST_ROOM_ID,
                port_id=2102,  # ✅ 修復：Loader AGV Port 2 (2102)
                status_id=403  # 泡藥完成 (放入預烘機前的狀態)
            )
            session.add(carrier)

            session.commit()
            print("✅ 測試資料創建完成")
            print(f"   Room: {TEST_ROOM_ID} (enabled=1)")
            print(f"   AGV: TEST_LOADER01 (model=LOADER)")
            print(f"   Carrier: {test_carrier_id} (port_id=2102 (Port 2), status_id=403)")
            print(f"   Equipment {PRE_DRYER_EQUIPMENT} Ports [2051-2058]: 假設有空位 (4 Stations)")
            print(f"   現有 Tasks: 0個")

        # 執行流程
        print("\n🚀 執行流程: loader_put_pre_dryer.yaml")
        with open('/app/config/tafl/flows/loader_put_pre_dryer.yaml', 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        result = await flow_executor.execute_flow(flow_yaml)
        print(f"📊 執行結果: {result.get('status')}")

        # 驗證任務創建
        print("\n✅ 測試後狀態:")
        with pool_manager.get_session() as session:
            tasks = session.exec(
                select(Task).where(
                    Task.room_id == TEST_ROOM_ID,
                    Task.type == "loader_put"
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

                # 驗證 work_id (關鍵銜接點)
                expected_work_ids = [2050102, 2050302, 2050502, 2050702]
                if task.work_id in expected_work_ids:
                    print(f"  - 目的地驗證: ✅ 正確 (預烘機，銜接 Unloader AGV)")
                    return True
                else:
                    print(f"  - 目的地驗證: ❌ 錯誤 (work_id 不在預期範圍)")
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


async def test_7_loader_take_boxin_duplicate():
    """測試 7: Loader 入口傳送箱取料 - 重複任務防護

    驗證連續執行流程 3 次，只創建 1 個任務
    """
    print("\n" + "="*60)
    print("測試 7: Loader 入口傳送箱取料 - 重複任務防護")
    print("="*60)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    test_carrier_ids = [TEST_CARRIER_BASE + 1, TEST_CARRIER_BASE + 2]
    production_port_ids = [2011, 2012]

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

        # 創建測試資料
        print("\n📝 創建測試資料...")
        with pool_manager.get_session() as session:
            room = Room(
                id=TEST_ROOM_ID,
                name="TEST_ROOM_LOADER",
                process_settings_id=1,
                enable=1
            )
            session.add(room)

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

            for i, carrier_id in enumerate(test_carrier_ids):
                carrier = Carrier(
                    id=carrier_id,
                    room_id=TEST_ROOM_ID,
                    port_id=production_port_ids[i],
                    status_id=201
                )
                session.add(carrier)

            session.commit()
            print("✅ 測試資料創建完成")

        # 載入流程
        with open('/app/config/tafl/flows/loader_take_boxin_transfer.yaml', 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        # 連續執行流程 3 次
        for i in range(3):
            print(f"\n🚀 第{i+1}次執行流程...")
            result = await flow_executor.execute_flow(flow_yaml)
            print(f"   結果: {result.get('status')}")

        # 驗證任務數量
        print("\n📊 檢查任務數量...")
        with pool_manager.get_session() as session:
            tasks = session.exec(
                select(Task).where(
                    Task.room_id == TEST_ROOM_ID,
                    Task.type == "loader_take"
                )
            ).all()

            print(f"\n創建的任務總數: {len(tasks)}")

            if len(tasks) == 1:
                task = tasks[0]
                print(f"✅ 防護機制正常！只創建了 1 個任務")
                print(f"   任務 ID: {task.id}")
                print(f"   任務名稱: {task.name}")
                print(f"   work_id: {task.work_id}")

                expected_work_ids = [2010101, 2010301]
                if task.work_id in expected_work_ids:
                    print(f"   目的地驗證: ✅ 正確 (入口傳送箱 Station)")
                    return True
                else:
                    print(f"   目的地驗證: ❌ 錯誤 (work_id 不正確)")
                    return False
            else:
                print(f"❌ 防護機制失敗！創建了 {len(tasks)} 個任務")
                for i, task in enumerate(tasks, 1):
                    print(f"   任務 {i}: ID={task.id}, 名稱={task.name}")
                return False

    finally:
        print("\n🧹 清理測試資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
            session.exec(delete(Carrier).where(Carrier.id.in_(test_carrier_ids)))
            session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
            session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
            session.commit()
        print("✅ 清理完成")


async def test_8_loader_put_cleaner_duplicate():
    """測試 8: Loader 放料到清洗機 - 重複任務防護

    驗證連續執行流程 3 次，只創建 1 個任務
    """
    print("\n" + "="*60)
    print("測試 8: Loader 放料到清洗機 - 重複任務防護")
    print("="*60)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    test_carrier_id = TEST_CARRIER_BASE + 3

    try:
        print("\n🧹 清理舊資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
            session.exec(delete(Carrier).where(Carrier.id == test_carrier_id))
            session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
            session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
            session.commit()
        print("✅ 清理完成")

        print("\n📝 創建測試資料...")
        with pool_manager.get_session() as session:
            room = Room(id=TEST_ROOM_ID, name="TEST_ROOM_LOADER", process_settings_id=1, enable=1)
            session.add(room)
            agv = AGV(id=TEST_AGV_ID, name="TEST_LOADER01", model="Loader", x=0.0, y=0.0, heading=0.0, enable=1)
            session.add(agv)
            carrier = Carrier(id=test_carrier_id, room_id=TEST_ROOM_ID, port_id=2101, status_id=101)
            session.add(carrier)
            session.commit()
            print("✅ 測試資料創建完成")

        with open('/app/config/tafl/flows/loader_put_cleaner.yaml', 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        for i in range(3):
            print(f"\n🚀 第{i+1}次執行流程...")
            result = await flow_executor.execute_flow(flow_yaml)
            print(f"   結果: {result.get('status')}")

        print("\n📊 檢查任務數量...")
        with pool_manager.get_session() as session:
            tasks = session.exec(select(Task).where(Task.room_id == TEST_ROOM_ID, Task.type == "loader_put")).all()
            print(f"\n創建的任務總數: {len(tasks)}")

            if len(tasks) == 1:
                task = tasks[0]
                print(f"✅ 防護機制正常！只創建了 1 個任務")
                print(f"   任務 ID: {task.id}, work_id: {task.work_id}")
                if task.work_id == 2030302:
                    print(f"   目的地驗證: ✅ 正確")
                    return True
                else:
                    print(f"   目的地驗證: ❌ 錯誤")
                    return False
            else:
                print(f"❌ 防護機制失敗！創建了 {len(tasks)} 個任務")
                return False

    finally:
        print("\n🧹 清理測試資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
            session.exec(delete(Carrier).where(Carrier.id == test_carrier_id))
            session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
            session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
            session.commit()
        print("✅ 清理完成")


async def test_9_loader_take_cleaner_duplicate():
    """測試 9: Loader 從清洗機取料 - 重複任務防護

    驗證連續執行流程 3 次，只創建 1 個任務
    """
    print("\n" + "="*60)
    print("測試 9: Loader 從清洗機取料 - 重複任務防護")
    print("="*60)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    test_carrier_ids = [TEST_CARRIER_BASE + 5, TEST_CARRIER_BASE + 6]
    production_port_ids = [2031, 2032]

    try:
        print("\n🧹 清理舊資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
            session.exec(delete(Carrier).where(Carrier.id.in_(test_carrier_ids)))
            session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
            session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
            session.commit()
        print("✅ 清理完成")

        print("\n📝 創建測試資料...")
        with pool_manager.get_session() as session:
            room = Room(id=TEST_ROOM_ID, name="TEST_ROOM_LOADER", process_settings_id=1, enable=1)
            session.add(room)
            agv = AGV(id=TEST_AGV_ID, name="TEST_LOADER01", model="Loader", x=0.0, y=0.0, heading=0.0, enable=1)
            session.add(agv)
            for i, carrier_id in enumerate(test_carrier_ids):
                carrier = Carrier(id=carrier_id, room_id=TEST_ROOM_ID, port_id=production_port_ids[i], status_id=303)
                session.add(carrier)
            session.commit()
            print("✅ 測試資料創建完成")

        with open('/app/config/tafl/flows/loader_take_cleaner.yaml', 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        for i in range(3):
            print(f"\n🚀 第{i+1}次執行流程...")
            result = await flow_executor.execute_flow(flow_yaml)
            print(f"   結果: {result.get('status')}")

        print("\n📊 檢查任務數量...")
        with pool_manager.get_session() as session:
            tasks = session.exec(select(Task).where(Task.room_id == TEST_ROOM_ID, Task.type == "loader_take")).all()
            print(f"\n創建的任務總數: {len(tasks)}")

            if len(tasks) == 1:
                task = tasks[0]
                print(f"✅ 防護機制正常！只創建了 1 個任務")
                print(f"   任務 ID: {task.id}, work_id: {task.work_id}")
                if task.work_id == 2030101:
                    print(f"   目的地驗證: ✅ 正確")
                    return True
                else:
                    print(f"   目的地驗證: ❌ 錯誤")
                    return False
            else:
                print(f"❌ 防護機制失敗！創建了 {len(tasks)} 個任務")
                return False

    finally:
        print("\n🧹 清理測試資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
            session.exec(delete(Carrier).where(Carrier.id.in_(test_carrier_ids)))
            session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
            session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
            session.commit()
        print("✅ 清理完成")


async def test_10_loader_put_soaker_duplicate():
    """測試 10: Loader 放料到泡藥機 - 重複任務防護

    驗證連續執行流程 3 次，只創建 1 個任務
    """
    print("\n" + "="*60)
    print("測試 10: Loader 放料到泡藥機 - 重複任務防護")
    print("="*60)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    test_carrier_id = TEST_CARRIER_BASE + 7

    try:
        print("\n🧹 清理舊資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
            session.exec(delete(Carrier).where(Carrier.id == test_carrier_id))
            session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
            session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
            session.commit()
        print("✅ 清理完成")

        print("\n📝 創建測試資料...")
        with pool_manager.get_session() as session:
            room = Room(id=TEST_ROOM_ID, name="TEST_ROOM_LOADER", process_settings_id=1, enable=1)
            session.add(room)
            agv = AGV(id=TEST_AGV_ID, name="TEST_LOADER01", model="Loader", x=0.0, y=0.0, heading=0.0, enable=1)
            session.add(agv)
            carrier = Carrier(id=test_carrier_id, room_id=TEST_ROOM_ID, port_id=2101, status_id=303)
            session.add(carrier)
            session.commit()
            print("✅ 測試資料創建完成")

        with open('/app/config/tafl/flows/loader_put_soaker.yaml', 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        for i in range(3):
            print(f"\n🚀 第{i+1}次執行流程...")
            result = await flow_executor.execute_flow(flow_yaml)
            print(f"   結果: {result.get('status')}")

        print("\n📊 檢查任務數量...")
        with pool_manager.get_session() as session:
            tasks = session.exec(select(Task).where(Task.room_id == TEST_ROOM_ID, Task.type == "loader_put")).all()
            print(f"\n創建的任務總數: {len(tasks)}")

            # 泡藥機有 6 個 Station，每個 Station 應該只創建 1 個任務
            # 第1次執行: 創建 6 個任務（每個 Station 1個）
            # 第2-3次執行: 不再創建新任務（重複防護）
            if len(tasks) == 6:
                print(f"✅ 防護機制正常！泡藥機 6 個 Station 各創建了 1 個任務")
                expected_work_ids = [2040102, 2040202, 2040302, 2040402, 2040502, 2040602]
                work_ids = [task.work_id for task in tasks]
                if all(wid in expected_work_ids for wid in work_ids):
                    print(f"   目的地驗證: ✅ 正確 (work_ids: {sorted(work_ids)})")
                    return True
                else:
                    print(f"   目的地驗證: ❌ 錯誤 (work_ids: {sorted(work_ids)})")
                    return False
            else:
                print(f"❌ 防護機制失敗！預期 6 個任務，實際 {len(tasks)} 個")
                return False

    finally:
        print("\n🧹 清理測試資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
            session.exec(delete(Carrier).where(Carrier.id == test_carrier_id))
            session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
            session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
            session.commit()
        print("✅ 清理完成")


async def test_11_loader_take_soaker_duplicate():
    """測試 11: Loader 從泡藥機取料 - 重複任務防護

    驗證連續執行流程 3 次，只創建 1 個任務
    """
    print("\n" + "="*60)
    print("測試 11: Loader 從泡藥機取料 - 重複任務防護")
    print("="*60)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    test_carrier_ids = list(range(TEST_CARRIER_BASE + 20, TEST_CARRIER_BASE + 26))
    production_port_ids = list(range(2041, 2047))

    try:
        print("\n🧹 清理舊資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
            session.exec(delete(Carrier).where(Carrier.id.in_(test_carrier_ids)))
            session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
            session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
            session.commit()
        print("✅ 清理完成")

        print("\n📝 創建測試資料...")
        with pool_manager.get_session() as session:
            room = Room(id=TEST_ROOM_ID, name="TEST_ROOM_LOADER", process_settings_id=1, enable=1)
            session.add(room)
            agv = AGV(id=TEST_AGV_ID, name="TEST_LOADER01", model="Loader", x=0.0, y=0.0, heading=0.0, enable=1)
            session.add(agv)
            for i, carrier_id in enumerate(test_carrier_ids):
                carrier = Carrier(id=carrier_id, room_id=TEST_ROOM_ID, port_id=production_port_ids[i], status_id=403)
                session.add(carrier)
            session.commit()
            print("✅ 測試資料創建完成")

        with open('/app/config/tafl/flows/loader_take_soaker.yaml', 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        for i in range(3):
            print(f"\n🚀 第{i+1}次執行流程...")
            result = await flow_executor.execute_flow(flow_yaml)
            print(f"   結果: {result.get('status')}")

        print("\n📊 檢查任務數量...")
        with pool_manager.get_session() as session:
            tasks = session.exec(select(Task).where(Task.room_id == TEST_ROOM_ID, Task.type == "loader_take")).all()
            print(f"\n創建的任務總數: {len(tasks)}")

            # 泡藥機有 6 個 Station，每個 Station 應該只創建 1 個任務
            # 第1次執行: 創建 6 個任務（每個 Station 1個）
            # 第2-3次執行: 不再創建新任務（重複防護）
            if len(tasks) == 6:
                print(f"✅ 防護機制正常！泡藥機 6 個 Station 各創建了 1 個任務")
                expected_work_ids = [2040101, 2040201, 2040301, 2040401, 2040501, 2040601]
                work_ids = [task.work_id for task in tasks]
                if all(wid in expected_work_ids for wid in work_ids):
                    print(f"   目的地驗證: ✅ 正確 (work_ids: {sorted(work_ids)})")
                    return True
                else:
                    print(f"   目的地驗證: ❌ 錯誤 (work_ids: {sorted(work_ids)})")
                    return False
            else:
                print(f"❌ 防護機制失敗！預期 6 個任務，實際 {len(tasks)} 個")
                return False

    finally:
        print("\n🧹 清理測試資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
            session.exec(delete(Carrier).where(Carrier.id.in_(test_carrier_ids)))
            session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
            session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
            session.commit()
        print("✅ 清理完成")


async def test_12_loader_put_pre_dryer_duplicate():
    """測試 12: Loader 放料到預烘機 - 重複任務防護

    驗證連續執行流程 3 次，只創建 1 個任務
    """
    print("\n" + "="*60)
    print("測試 12: Loader 放料到預烘機 - 重複任務防護")
    print("="*60)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    test_carrier_id = TEST_CARRIER_BASE + 30

    try:
        print("\n🧹 清理舊資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
            session.exec(delete(Carrier).where(Carrier.id == test_carrier_id))
            session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
            session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
            session.commit()
        print("✅ 清理完成")

        print("\n📝 創建測試資料...")
        with pool_manager.get_session() as session:
            room = Room(id=TEST_ROOM_ID, name="TEST_ROOM_LOADER", process_settings_id=1, enable=1)
            session.add(room)
            agv = AGV(id=TEST_AGV_ID, name="TEST_LOADER01", model="Loader", x=0.0, y=0.0, heading=0.0, enable=1)
            session.add(agv)
            carrier = Carrier(id=test_carrier_id, room_id=TEST_ROOM_ID, port_id=2102, status_id=403)
            session.add(carrier)
            session.commit()
            print("✅ 測試資料創建完成")

        with open('/app/config/tafl/flows/loader_put_pre_dryer.yaml', 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        for i in range(3):
            print(f"\n🚀 第{i+1}次執行流程...")
            result = await flow_executor.execute_flow(flow_yaml)
            print(f"   結果: {result.get('status')}")

        print("\n📊 檢查任務數量...")
        with pool_manager.get_session() as session:
            tasks = session.exec(select(Task).where(Task.room_id == TEST_ROOM_ID, Task.type == "loader_put")).all()
            print(f"\n創建的任務總數: {len(tasks)}")

            # 預烘機有 4 個 Station (1, 3, 5, 7)，每個 Station 應該只創建 1 個任務
            # 第1次執行: 創建 4 個任務（每個 Station 1個）
            # 第2-3次執行: 不再創建新任務（重複防護）
            if len(tasks) == 4:
                print(f"✅ 防護機制正常！預烘機 4 個 Station 各創建了 1 個任務")
                expected_work_ids = [2050102, 2050302, 2050502, 2050702]
                work_ids = [task.work_id for task in tasks]
                if all(wid in expected_work_ids for wid in work_ids):
                    print(f"   目的地驗證: ✅ 正確 (work_ids: {sorted(work_ids)})")
                    return True
                else:
                    print(f"   目的地驗證: ❌ 錯誤 (work_ids: {sorted(work_ids)})")
                    return False
            else:
                print(f"❌ 防護機制失敗！預期 4 個任務，實際 {len(tasks)} 個")
                return False

    finally:
        print("\n🧹 清理測試資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.room_id == TEST_ROOM_ID))
            session.exec(delete(Carrier).where(Carrier.id == test_carrier_id))
            session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
            session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
            session.commit()
        print("✅ 清理完成")


async def main():
    """執行所有 Loader AGV 測試"""
    print("\n" + "="*60)
    print("Loader AGV 完整測試 (6個基本功能 + 6個重複防護)")
    print("="*60)

    results = []

    # 測試 1: 入口傳送箱取料
    results.append(("Loader 入口傳送箱取料", await test_1_loader_take_boxin_transfer()))

    # 測試 2: 放料到清洗機
    results.append(("Loader 放料到清洗機", await test_2_loader_put_cleaner()))

    # 測試 3: 從清洗機取料
    results.append(("Loader 從清洗機取料", await test_3_loader_take_cleaner()))

    # 測試 4: 放料到泡藥機
    results.append(("Loader 放料到泡藥機", await test_4_loader_put_soaker()))

    # 測試 5: 從泡藥機取料
    results.append(("Loader 從泡藥機取料", await test_5_loader_take_soaker()))

    # 測試 6: 放料到預烘機
    results.append(("Loader 放料到預烘機", await test_6_loader_put_pre_dryer()))

    # 測試 7: 入口傳送箱取料 - 重複防護
    results.append(("Loader 取料重複防護 (入口箱)", await test_7_loader_take_boxin_duplicate()))

    # 測試 8: 放料到清洗機 - 重複防護
    results.append(("Loader 放料重複防護 (清洗機)", await test_8_loader_put_cleaner_duplicate()))

    # 測試 9: 從清洗機取料 - 重複防護
    results.append(("Loader 取料重複防護 (清洗機)", await test_9_loader_take_cleaner_duplicate()))

    # 測試 10: 放料到泡藥機 - 重複防護
    results.append(("Loader 放料重複防護 (泡藥機)", await test_10_loader_put_soaker_duplicate()))

    # 測試 11: 從泡藥機取料 - 重複防護
    results.append(("Loader 取料重複防護 (泡藥機)", await test_11_loader_take_soaker_duplicate()))

    # 測試 12: 放料到預烘機 - 重複防護
    results.append(("Loader 放料重複防護 (預烘機)", await test_12_loader_put_pre_dryer_duplicate()))

    # 總結
    print("\n" + "="*60)
    print("測試總結")
    print("="*60)
    for name, result in results:
        status = "✅ 通過" if result else "❌ 失敗"
        print(f"{status} - {name}")

    all_passed = all(r for _, r in results)
    total_tests = len(results)
    passed_tests = sum(1 for _, r in results if r)

    if all_passed:
        print(f"\n🎉 所有 Loader AGV 測試通過！({passed_tests}/{total_tests})")
        return 0
    else:
        print(f"\n⚠️ 部分測試失敗 ({passed_tests}/{total_tests})")
        return 1


if __name__ == '__main__':
    exit_code = asyncio.run(main())
    sys.exit(exit_code)
