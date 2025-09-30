#!/usr/bin/env python3
"""測試重複執行防護機制"""

import sys
import asyncio

sys.path.insert(0, '/app/tafl_wcs_ws/src')
sys.path.insert(0, '/app/db_proxy_ws/src')
sys.path.insert(0, '/app/tafl_ws/src')

from tafl_wcs.tafl_executor_wrapper import TAFLExecutorWrapper
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import Rack, Location, Task
from sqlmodel import select, delete
from sqlalchemy import update

async def test_duplicate_prevention():
    """測試：重複執行流程不會產生重複任務"""
    print("\n" + "="*60)
    print("測試：重複執行防護機制")
    print("="*60)

    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    rack_id = 501
    inlet_id = 10001
    outlet_id = 10002

    try:
        # 清理任何舊資料
        print("\n🧹 清理舊資料...")
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

        # 創建測試資料
        print("\n📝 創建測試資料...")
        with pool_manager.get_session() as session:
            rack = Rack(
                id=rack_id,
                name="TEST_DUPLICATE_RACK",
                location_id=inlet_id,
                status_id=1,
                product_id=1,
                direction=0,
                is_carry=0
            )
            session.add(rack)

            # 入口佔用
            session.exec(
                update(Location)
                .where(Location.id == inlet_id)
                .values(location_status_id=3, rack_id=rack_id)
            )

            # 出口空閒
            session.exec(
                update(Location)
                .where(Location.id == outlet_id)
                .values(location_status_id=2, rack_id=None)
            )
            session.commit()
            print("✅ 測試資料創建完成（rack 501 在入口，出口空閒）")

        # 載入流程
        with open('/app/config/tafl/flows/empty_rack_inlet_to_outlet.yaml', 'r', encoding='utf-8') as f:
            flow_yaml = f.read()

        # 第1次執行
        print("\n🚀 第1次執行流程...")
        result1 = await flow_executor.execute_flow(flow_yaml)
        print(f"   結果: {result1.get('status')}")

        # 第2次執行
        print("\n🚀 第2次執行流程...")
        result2 = await flow_executor.execute_flow(flow_yaml)
        print(f"   結果: {result2.get('status')}")

        # 第3次執行
        print("\n🚀 第3次執行流程...")
        result3 = await flow_executor.execute_flow(flow_yaml)
        print(f"   結果: {result3.get('status')}")

        # 檢查結果
        print("\n📊 檢查任務數量...")
        with pool_manager.get_session() as session:
            tasks = session.exec(
                select(Task).where(
                    Task.rack_id == rack_id,
                    Task.work_id == 220001
                )
            ).all()

            print(f"\n創建的任務總數: {len(tasks)}")

            if len(tasks) == 1:
                task = tasks[0]
                print(f"✅ 防護機制正常！只創建了 1 個任務")
                print(f"   任務 ID: {task.id}")
                print(f"   任務名稱: {task.name}")
                print(f"   狀態: {task.status_id}")
                if task.parameters:
                    print(f"   目的地: {task.parameters.get('to_location_name')}")
                return True
            else:
                print(f"❌ 防護機制失敗！創建了 {len(tasks)} 個任務")
                for i, task in enumerate(tasks, 1):
                    print(f"   任務 {i}: ID={task.id}, 名稱={task.name}")
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

if __name__ == '__main__':
    result = asyncio.run(test_duplicate_prevention())
    sys.exit(0 if result else 1)