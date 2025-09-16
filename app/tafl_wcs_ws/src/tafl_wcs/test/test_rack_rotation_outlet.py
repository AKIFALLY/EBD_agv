#!/usr/bin/env python3
"""
測試房間出口架台翻轉流程
測試條件：A面滿、B面空、房間內有載具待處理
"""

import sys
import os
import logging
from datetime import datetime

# Add paths for imports
sys.path.insert(0, '/app/tafl_wcs_ws/src/tafl_wcs')
sys.path.insert(0, '/app/db_proxy_ws/install/db_proxy/lib/python3.12/site-packages')
sys.path.insert(0, '/app/tafl_ws/src')

from tafl_wcs.tafl_db_bridge import TAFLDatabaseBridge
from tafl_wcs.tafl_executor_wrapper import TAFLExecutorWrapper
from db_proxy.models import Rack, Carrier, Location, Task

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Database connection
DATABASE_URL = "postgresql://agvc:password@192.168.100.254:5432/agvc"


def setup_test_data(db_bridge):
    """設置測試資料：在房間出口建立架台並填充載具"""
    logger.info("📦 設置測試資料...")

    with db_bridge.pool_manager.get_session() as session:
        # 1. 查詢房間1出口位置 (ID: 10002)
        location = session.get(Location, 10002)
        if location:
            logger.info(f"找到房間1出口位置: {location.name} (ID: {location.id})")

        # 2. 查詢或創建架台在該位置
        rack = session.query(Rack).filter(Rack.location_id == 10002).first()
        if not rack:
            # 創建測試架台
            rack = Rack(
                name="TestRack_Outlet_01",
                location_id=10002,
                room_id=1,
                status_id=1,  # Available
                is_carry=1,
                is_docked=1
            )
            session.add(rack)
            session.commit()
            logger.info(f"✅ 創建測試架台: {rack.name} at location 10002")
        else:
            logger.info(f"使用現有架台: {rack.name} (ID: {rack.id})")

        # 3. 清除該架台上的現有載具
        existing_carriers = session.query(Carrier).filter(Carrier.rack_id == rack.id).all()
        for carrier in existing_carriers:
            session.delete(carrier)
        session.commit()
        logger.info(f"清除了 {len(existing_carriers)} 個現有載具")

        # 4. 在A面填滿載具 (rack_index 1-16)
        for i in range(1, 17):  # 1-16 for A-side
            carrier = Carrier(
                rack_id=rack.id,
                room_id=1,
                rack_index=i,
                status_id=1  # Available
            )
            session.add(carrier)
        session.commit()
        logger.info("✅ A面已填滿16個載具 (rack_index 1-16)")

        # 5. B面保持為空 (不創建任何載具在17-32)
        logger.info("✅ B面保持為空 (rack_index 17-32 無載具)")

        # 6. 在房間內創建一些載具（不在架台上）
        # 注意：Carrier模型沒有type字段，我們創建在房間內但不在架台上的載具
        for i in range(1, 6):  # Create 5 carriers in the room
            carrier = Carrier(
                room_id=1,
                rack_id=None,  # Not on rack
                rack_index=None,
                status_id=1  # Available
            )
            session.add(carrier)
        session.commit()
        logger.info("✅ 在房間1內創建了5個載具（不在架台上）")

        # 7. 清除該架台的未完成任務
        existing_tasks = session.query(Task).filter(
            Task.rack_id == rack.id,
            Task.status_id.in_([0, 1, 2, 3])
        ).all()
        for task in existing_tasks:
            task.status_id = 4  # Set to completed
        session.commit()
        logger.info(f"標記了 {len(existing_tasks)} 個現有任務為已完成")

        return rack.id


def test_rack_side_check(db_bridge, rack_id):
    """測試架台側面檢查功能"""
    logger.info("\n🔍 測試架台側面檢查...")

    # 檢查架台兩側狀態
    result = db_bridge.check_rack_side_status(rack_id, check_type='both')

    logger.info(f"架台 ID {rack_id} 狀態:")
    logger.info(f"  - A面載具數: {result.get('a_side_count', 0)}")
    logger.info(f"  - B面載具數: {result.get('b_side_count', 0)}")
    logger.info(f"  - A面已滿: {result.get('a_side_full', False)}")
    logger.info(f"  - B面為空: {result.get('b_side_empty', False)}")
    logger.info(f"  - 需要翻轉: {result.get('rotation_needed', False)}")

    return result


def test_room_carriers_check(db_bridge, room_id=1):
    """測試房間載具檢查功能"""
    logger.info("\n🏠 測試房間載具檢查...")

    result = db_bridge.check_carriers_in_room(room_id)

    logger.info(f"房間 {room_id} 載具狀態:")
    logger.info(f"  - 總載具數: {result.get('total_carriers', 0)}")
    logger.info(f"  - 有載具: {result.get('has_carriers', False)}")
    logger.info(f"  - 狀態分布: {result.get('status_counts', {})}")

    return result


def test_tafl_flow_execution(db_bridge):
    """測試執行TAFL流程"""
    logger.info("\n🚀 測試執行TAFL流程...")

    # 載入並執行TAFL流程
    flow_file = "/app/config/tafl/flows/rack_rotation_room_outlet_afull_bempty.yaml"

    if not os.path.exists(flow_file):
        logger.error(f"TAFL流程檔案不存在: {flow_file}")
        return

    # 創建執行器
    executor = TAFLExecutorWrapper(DATABASE_URL)

    # 載入流程內容
    with open(flow_file, 'r', encoding='utf-8') as f:
        flow_content = f.read()

    # 執行流程（execute_flow 是異步的，需要用 asyncio 執行）
    logger.info("開始執行TAFL流程...")
    import asyncio
    result = asyncio.run(executor.execute_flow(flow_content))

    if result.get('status') == 'completed':
        logger.info("✅ TAFL流程執行成功")
        logger.info(f"執行時間: {result.get('execution_time', 0):.2f}秒")
    else:
        logger.error(f"❌ TAFL流程執行失敗: {result.get('error')}")

    return result


def check_created_tasks(db_bridge):
    """檢查創建的任務"""
    logger.info("\n📋 檢查創建的任務...")

    # 查詢最近創建的翻轉任務
    result = db_bridge.query_tasks(
        work_id=220001,  # Rack rotation work ID
        sort_by='created_at',
        sort_order='desc',
        limit=5
    )

    tasks = result.get('data', [])
    logger.info(f"找到 {len(tasks)} 個翻轉任務")

    for task in tasks:
        logger.info(f"\n任務 ID: {task['id']}")
        logger.info(f"  名稱: {task.get('name', 'N/A')}")
        logger.info(f"  描述: {task.get('description', 'N/A')}")
        logger.info(f"  狀態: {task.get('status', 'N/A')}")
        logger.info(f"  架台ID: {task.get('rack_id', 'N/A')}")
        logger.info(f"  參數: {task.get('parameters', {})}")
        logger.info(f"  創建時間: {task.get('created_at', 'N/A')}")


def main():
    """主測試函數"""
    logger.info("=" * 60)
    logger.info("房間出口架台翻轉測試")
    logger.info("測試條件：")
    logger.info("  1. A面滿 (rack_index 1-16 全部有載具)")
    logger.info("  2. B面空 (rack_index 17-32 沒有載具)")
    logger.info("  3. 房間有載具待處理")
    logger.info("=" * 60)

    try:
        # 初始化資料庫橋接
        db_bridge = TAFLDatabaseBridge(DATABASE_URL)
        logger.info("✅ 資料庫連接成功")

        # 設置測試資料
        rack_id = setup_test_data(db_bridge)

        # 測試架台側面檢查
        side_result = test_rack_side_check(db_bridge, rack_id)

        # 測試房間載具檢查
        room_result = test_room_carriers_check(db_bridge, room_id=1)

        # 檢查是否滿足翻轉條件
        if (side_result.get('a_side_full') and
            side_result.get('b_side_empty') and
            room_result.get('has_carriers')):
            logger.info("\n✅ 滿足翻轉條件！")

            # 執行TAFL流程
            flow_result = test_tafl_flow_execution(db_bridge)

            # 檢查創建的任務
            check_created_tasks(db_bridge)
        else:
            logger.warning("\n⚠️ 不滿足翻轉條件")
            logger.info(f"  - A面已滿: {side_result.get('a_side_full')}")
            logger.info(f"  - B面為空: {side_result.get('b_side_empty')}")
            logger.info(f"  - 房間有載具: {room_result.get('has_carriers')}")

        logger.info("\n" + "=" * 60)
        logger.info("測試完成")
        logger.info("=" * 60)

    except Exception as e:
        logger.error(f"測試過程中發生錯誤: {e}", exc_info=True)
    finally:
        if 'db_bridge' in locals():
            db_bridge.close()


if __name__ == '__main__':
    main()