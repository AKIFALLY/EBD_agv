#!/usr/bin/env python3
"""完全靜態的任務創建測試"""
import sys
import asyncio

sys.path.insert(0, '/app/tafl_wcs_ws/src')
sys.path.insert(0, '/app/db_proxy_ws/src')
sys.path.insert(0, '/app/tafl_ws/src')

from tafl_wcs.tafl_executor_wrapper import TAFLExecutorWrapper
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import Task, Work, Room
from sqlmodel import select, delete

TEST_WORK_ID = 9999902
TEST_ROOM_ID = 991

async def test_static_task():
    """測試靜態任務創建"""
    print("\n" + "="*60)
    print("靜態任務創建測試（無查詢，無條件，無變數）")
    print("="*60)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    try:
        # 清理舊資料
        print("\n🧹 清理舊測試資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.work_id == TEST_WORK_ID))
            session.exec(delete(Work).where(Work.id == TEST_WORK_ID))
            session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
            session.commit()
        print("✅ 清理完成")

        # 創建測試用記錄
        print("\n📝 創建測試資料...")
        with pool_manager.get_session() as session:
            # Work 記錄
            test_work = Work(
                id=TEST_WORK_ID,
                name="測試 Work",
                description="用於 TAFL 靜態任務測試",
                work_code="TEST_STATIC"
            )
            session.add(test_work)
            print("   ✅ Work 記錄")

            # Room 記錄
            test_room = Room(
                id=TEST_ROOM_ID,
                name="TEST_ROOM",
                process_settings_id=1,
                enable=1
            )
            session.add(test_room)
            print("   ✅ Room 記錄")

            session.commit()
        print("✅ 測試資料創建完成")

        # 執行流程
        print("\n🚀 執行靜態 TAFL 流程...")
        with open('/app/config/tafl/flows/test_static_task.yaml', 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        result = await flow_executor.execute_flow(flow_yaml)
        print(f"📊 執行結果: {result.get('status')}")

        if 'error' in result:
            print(f"❌ 錯誤: {result['error']}")
            return False

        # 驗證
        print("\n✅ 驗證任務創建...")
        with pool_manager.get_session() as session:
            tasks = session.exec(
                select(Task).where(Task.work_id == TEST_WORK_ID)
            ).all()

            if tasks:
                print(f"🎉 成功！創建了 {len(tasks)} 個任務:")
                for task in tasks:
                    print(f"   - Task ID: {task.id}")
                    print(f"     work_id: {task.work_id}")
                    print(f"     name: {task.name}")
                    print(f"     room_id: {task.room_id}")
                    print(f"     priority: {task.priority}")
                    print(f"     status_id: {task.status_id}")
                print("\n✅ 測試通過！TAFL 任務創建功能正常！")
                return True
            else:
                print("❌ 未創建任務")
                print("\n這表示 TAFL 流程的 create 動詞可能有問題")
                return False

    except Exception as e:
        print(f"\n❌ 測試過程發生錯誤: {e}")
        import traceback
        traceback.print_exc()
        return False

    finally:
        # 清理測試資料
        print("\n🧹 清理測試資料...")
        with pool_manager.get_session() as session:
            session.exec(delete(Task).where(Task.work_id == TEST_WORK_ID))
            session.exec(delete(Work).where(Work.id == TEST_WORK_ID))
            session.exec(delete(Room).where(Room.id == TEST_ROOM_ID))
            session.commit()
        print("✅ 清理完成")
        flow_executor.shutdown()
        pool_manager.shutdown()

if __name__ == '__main__':
    result = asyncio.run(test_static_task())
    sys.exit(0 if result else 1)
