#!/usr/bin/env python3
"""最簡化的 TAFL 流程測試"""
import sys
import asyncio

sys.path.insert(0, '/app/tafl_wcs_ws/src')
sys.path.insert(0, '/app/db_proxy_ws/src')
sys.path.insert(0, '/app/tafl_ws/src')

from tafl_wcs.tafl_executor_wrapper import TAFLExecutorWrapper
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import AGV, Carrier, Task, Room, Work
from sqlmodel import select, delete

# 測試用 ID
TEST_ROOM_ID = 991
TEST_AGV_ID = 9001
TEST_CARRIER_ID = 90001
TEST_WORK_ID = 9999901

async def test_minimal_flow():
    """測試最簡化的 TAFL 流程"""
    print("\n" + "="*60)
    print("最簡化 TAFL 流程測試")
    print("="*60)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    try:
        # === 清理舊資料 ===
        print("\n🧹 清理舊測試資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.work_id == TEST_WORK_ID))
            session.exec(delete(Carrier).where(Carrier.id == TEST_CARRIER_ID))
            session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
            session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
            session.exec(delete(Work).where(Work.id == TEST_WORK_ID))
            session.commit()
        print("✅ 清理完成")

        # === 創建測試資料 ===
        print("\n📝 創建測試資料...")
        with pool_manager.get_session() as session:
            # 0. Work
            test_work = Work(
                id=TEST_WORK_ID,
                name="測試 Work",
                description="用於 TAFL 最簡化測試",
                work_code="TEST_MINIMAL"
            )
            session.add(test_work)
            print(f"   ✅ Work: id={TEST_WORK_ID}, name=測試 Work")

            # 1. Room
            room = Room(
                id=TEST_ROOM_ID,
                name="TEST_ROOM",
                process_settings_id=1,
                enable=1
            )
            session.add(room)
            print(f"   ✅ Room: id={TEST_ROOM_ID}, name=TEST_ROOM, enable=1")

            # 2. AGV
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
            print(f"   ✅ AGV: id={TEST_AGV_ID}, model=LOADER, enable=1")

            # 3. Carrier
            carrier = Carrier(
                id=TEST_CARRIER_ID,
                room_id=TEST_ROOM_ID,
                port_id=2011,  # 隨意一個 port
                status_id=201
            )
            session.add(carrier)
            print(f"   ✅ Carrier: id={TEST_CARRIER_ID}, room_id={TEST_ROOM_ID}")

            session.commit()
        print("✅ 測試資料創建完成")

        # === 執行 TAFL 流程 ===
        print("\n🚀 執行最簡化 TAFL 流程...")
        with open('/app/config/tafl/flows/test_minimal_simple.yaml', 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        result = await flow_executor.execute_flow(flow_yaml)
        print(f"📊 執行結果: {result.get('status')}")

        if 'error' in result:
            print(f"❌ 錯誤: {result['error']}")
            return False

        # === 驗證任務創建 ===
        print("\n✅ 驗證任務創建...")
        with pool_manager.get_session() as session:
            tasks = session.exec(
                select(Task).where(Task.work_id == TEST_WORK_ID)
            ).all()

            if tasks:
                print(f"✅ 成功創建 {len(tasks)} 個任務:")
                for task in tasks:
                    print(f"   - Task ID: {task.id}")
                    print(f"     work_id: {task.work_id}")
                    print(f"     name: {task.name}")
                    print(f"     room_id: {task.room_id}")
                    if task.parameters:
                        print(f"     參數:")
                        print(f"       rooms_found: {task.parameters.get('rooms_found')}")
                        print(f"       agvs_found: {task.parameters.get('agvs_found')}")
                        print(f"       carriers_found: {task.parameters.get('carriers_found')}")
                print("\n🎉 測試通過！")
                return True
            else:
                print("❌ 未創建任務")
                print("\n可能原因：")
                print("  1. Room 查詢失敗")
                print("  2. AGV 查詢失敗")
                print("  3. Carrier 查詢失敗")
                print("  4. 任務創建邏輯有問題")
                return False

    except Exception as e:
        print(f"\n❌ 測試過程發生錯誤: {e}")
        import traceback
        traceback.print_exc()
        return False

    finally:
        # === 清理測試資料 ===
        print("\n🧹 清理測試資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.work_id == TEST_WORK_ID))
            session.exec(delete(Carrier).where(Carrier.id == TEST_CARRIER_ID))
            session.exec(delete(AGV).where(AGV.id == TEST_AGV_ID))
            session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
            session.exec(delete(Work).where(Work.id == TEST_WORK_ID))
            session.commit()
        print("✅ 清理完成")
        flow_executor.shutdown()
        pool_manager.shutdown()

if __name__ == '__main__':
    result = asyncio.run(test_minimal_flow())
    sys.exit(0 if result else 1)
