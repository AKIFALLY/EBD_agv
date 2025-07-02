#!/usr/bin/env python3
"""
創建測試任務的腳本，用於測試 Kuka API
"""
from sqlmodel import select
from db_proxy.crud.task_crud import task_crud
from db_proxy.models import Task, Work, TaskStatus
from db_proxy.connection_pool_manager import ConnectionPoolManager
from datetime import datetime, timezone
import sys
import os
sys.path.append('/app/db_proxy_ws/src')


def create_test_task():
    """創建測試任務"""

    # 資料庫連接
    db_url = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'
    db_pool = ConnectionPoolManager(db_url)

    try:
        with db_pool.get_session() as session:
            # 檢查是否已存在測試任務
            existing_task = session.exec(
                select(Task).where(Task.mission_code == "mission202309250005")
            ).first()

            if existing_task:
                print(
                    f"✅ 測試任務已存在: ID={existing_task.id}, Name={existing_task.name}, Mission Code={existing_task.mission_code}")
                return existing_task.id

            # 查找第一個可用的 Work 和 TaskStatus
            work = session.exec(select(Work)).first()
            task_status = session.exec(
                select(TaskStatus).where(TaskStatus.name == "待執行")
            ).first()

            if not work:
                print("❌ 找不到 Work 資料，請先初始化資料庫")
                return None

            if not task_status:
                print("❌ 找不到 TaskStatus 資料，請先初始化資料庫")
                return None

            # 創建測試任務
            test_task_data = {
                "work_id": work.id,
                "status_id": task_status.id,
                "name": "Kuka API 測試任務 - mission202309250005",
                "description": "Kuka API 測試任務",
                "mission_code": "mission202309250005",  # 對應 API 測試中的 missionCode
                "priority": 5,
                "parameters": {
                    "test_task": True,
                    "created_for": "kuka_api_testing"
                }
            }

            # 使用 CRUD 創建任務
            new_task = Task(**test_task_data)
            created_task = task_crud.create(session, new_task)

            print(f"✅ 測試任務創建成功:")
            print(f"   ID: {created_task.id}")
            print(f"   Name: {created_task.name}")
            print(f"   Description: {created_task.description}")
            print(f"   Mission Code: {created_task.mission_code}")
            print(f"   Work ID: {created_task.work_id}")
            print(f"   Status ID: {created_task.status_id}")

            return created_task.id

    except Exception as e:
        print(f"❌ 創建測試任務時發生錯誤: {e}")
        return None


def create_additional_test_tasks():
    """創建額外的測試任務"""

    db_url = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'
    db_pool = ConnectionPoolManager(db_url)

    test_missions = [
        "1357",              # 對應您之前測試的 missionCode
        "test_mission_001",
        "test_mission_002",
        "test_mission_003"
    ]

    try:
        with db_pool.get_session() as session:
            work = session.exec(select(Work)).first()
            task_status = session.exec(
                select(TaskStatus).where(TaskStatus.name == "待執行")
            ).first()

            if not work or not task_status:
                print("❌ 找不到必要的基礎資料")
                return

            created_count = 0
            for mission_name in test_missions:
                # 檢查是否已存在
                existing = session.exec(
                    select(Task).where(Task.mission_code == mission_name)
                ).first()

                if existing:
                    print(f"⚠️  任務 {mission_name} 已存在，跳過")
                    continue

                # 創建任務
                test_task_data = {
                    "work_id": work.id,
                    "status_id": task_status.id,
                    "name": f"Kuka API 測試任務 - {mission_name}",
                    "description": f"Kuka API 測試任務 - {mission_name}",
                    "mission_code": mission_name,
                    "priority": 3,
                    "parameters": {
                        "test_task": True,
                        "created_for": "kuka_api_testing"
                    }
                }

                new_task = Task(**test_task_data)
                created_task = task_crud.create(session, new_task)
                created_count += 1

                print(
                    f"✅ 創建任務: {created_task.name} (ID: {created_task.id}, Mission Code: {created_task.mission_code})")

            print(f"\n✅ 總共創建了 {created_count} 個測試任務")

    except Exception as e:
        print(f"❌ 創建額外測試任務時發生錯誤: {e}")


def list_test_tasks():
    """列出所有測試任務"""

    db_url = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'
    db_pool = ConnectionPoolManager(db_url)

    try:
        with db_pool.get_session() as session:
            # 查找所有測試任務
            test_tasks = session.exec(
                select(Task).where(Task.mission_code.like("%mission%"))
            ).all()

            if not test_tasks:
                print("❌ 找不到任何測試任務")
                return

            print(f"\n📋 找到 {len(test_tasks)} 個測試任務:")
            print("-" * 100)
            for task in test_tasks:
                print(
                    f"ID: {task.id:3d} | Name: {task.name:30s} | Mission Code: {task.mission_code:20s} | Status: {task.status_id}")
            print("-" * 100)

    except Exception as e:
        print(f"❌ 列出測試任務時發生錯誤: {e}")


if __name__ == "__main__":
    print("=== Kuka API 測試任務創建工具 ===\n")

    # 創建主要測試任務
    task_id = create_test_task()

    if task_id:
        print(f"\n🎯 主要測試任務創建完成，可以使用以下 missionCode 測試 API:")
        print(f"   missionCode: mission202309250005")
        print(f"   對應的 Task ID: {task_id}")

    # 詢問是否創建額外測試任務
    user_input = input("\n是否要創建額外的測試任務? (y/n): ")
    if user_input.lower() == 'y':
        create_additional_test_tasks()

    # 列出所有測試任務
    list_test_tasks()

    print("\n✅ 完成！現在可以使用 test_kuka_api.py 來測試 API 了。")
