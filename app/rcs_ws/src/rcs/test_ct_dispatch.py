#!/usr/bin/env python3
"""
CT Manager 任務派發測試腳本
用於測試基於房間和車型的智能任務分派機制
"""

import sys
import os
import json
from datetime import datetime, timezone
from sqlmodel import select

# 添加必要的路徑到 Python 路徑中
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..'))

from db_proxy.models import Task, AGV, TaskStatus, AgvStatus
from db_proxy.connection_pool_manager import ConnectionPoolManager
from rcs.ct_manager import CtManager


class MockRcsNode:
    """模擬 RCS Core 節點"""
    def __init__(self):
        self.db_pool = ConnectionPoolManager()
        
    def get_logger(self):
        return MockLogger()


class MockLogger:
    """模擬日誌記錄器"""
    def debug(self, msg):
        print(f"[DEBUG] {msg}")
        
    def info(self, msg):
        print(f"[INFO] {msg}")
        
    def warning(self, msg):
        print(f"[WARNING] {msg}")
        
    def error(self, msg, exc_info=False):
        print(f"[ERROR] {msg}")


def create_test_tasks(session):
    """創建測試任務"""
    print("🔧 創建測試任務...")
    
    test_tasks = [
        {
            "name": "房間2 Loader 任務",
            "description": "房間2內的裝載任務",
            "status_id": 1,  # 待執行
            "room_id": 2,
            "priority": 10,
            "parameters": {"model": "Loader", "operation": "load"}
        },
        {
            "name": "房間1 Unloader 任務", 
            "description": "房間1內的卸載任務",
            "status_id": 1,  # 待執行
            "room_id": 1,
            "priority": 8,
            "parameters": {"model": "Unloader", "operation": "unload"}
        },
        {
            "name": "房外 Cargo 任務",
            "description": "房外的運輸任務", 
            "status_id": 1,  # 待執行
            "room_id": None,  # 房外任務
            "priority": 15,
            "parameters": {"model": "Cargo", "operation": "transport"}
        }
    ]
    
    for task_data in test_tasks:
        task = Task(**task_data)
        session.add(task)
    
    session.commit()
    print("✅ 測試任務創建完成")


def check_agv_status(session):
    """檢查 AGV 狀態"""
    print("🔍 檢查 AGV 狀態...")
    
    agvs = session.exec(
        select(AGV).where(
            (AGV.model == "Cargo") |
            (AGV.model == "Loader") |
            (AGV.model == "Unloader")
        )
    ).all()
    
    for agv in agvs:
        print(f"  AGV: {agv.name}, 車型: {agv.model}, 狀態: {agv.status_id}")
        
        # 確保測試 AGV 為閒置狀態
        if agv.name in ['Cargo02', 'Loader02', 'Unloader01']:
            agv.status_id = 3  # 設為閒置
    
    session.commit()
    print("✅ AGV 狀態檢查完成")


def test_dispatch_logic():
    """測試派發邏輯"""
    print("🚀 開始測試 CT Manager 任務派發邏輯")
    print("=" * 50)
    
    # 創建模擬節點和管理器
    mock_node = MockRcsNode()
    ct_manager = CtManager(mock_node)
    
    try:
        with mock_node.db_pool.get_session() as session:
            # 1. 檢查 AGV 狀態
            check_agv_status(session)
            
            # 2. 創建測試任務
            create_test_tasks(session)
            
            # 3. 顯示派發前狀態
            print("\n📋 派發前任務狀態:")
            tasks = session.exec(
                select(Task).where(
                    Task.status_id == 1,
                    # 明確指定支援的 CT 車型
                    (Task.parameters["model"].as_string() == "Cargo") |
                    (Task.parameters["model"].as_string() == "Loader") |
                    (Task.parameters["model"].as_string() == "Unloader")
                )
            ).all()
            
            for task in tasks:
                print(f"  任務 {task.id}: {task.name}, 優先級: {task.priority}, "
                      f"車型: {task.parameters.get('model')}, 房間: {task.room_id}")
            
            # 4. 執行派發
            print("\n🎯 執行任務派發...")
            ct_manager.dispatch()
            
            # 5. 顯示派發後狀態
            print("\n📊 派發後任務狀態:")
            session.refresh_all()  # 重新載入資料
            
            all_tasks = session.exec(select(Task)).all()
            for task in all_tasks:
                if task.parameters and task.parameters.get('model') in ['Cargo', 'Loader', 'Unloader']:
                    agv_name = "未分派"
                    if task.agv_id:
                        agv = session.exec(select(AGV).where(AGV.id == task.agv_id)).first()
                        agv_name = agv.name if agv else f"AGV ID {task.agv_id}"
                    
                    print(f"  任務 {task.id}: {task.name}")
                    print(f"    狀態: {task.status_id}, AGV: {agv_name}")
                    print(f"    任務代碼: {task.mission_code}")
                    print()
            
            # 6. 顯示 AGV 狀態
            print("🚗 AGV 狀態:")
            agvs = session.exec(
                select(AGV).where(
                    (AGV.model == "Cargo") |
                    (AGV.model == "Loader") |
                    (AGV.model == "Unloader")
                )
            ).all()
            
            for agv in agvs:
                status_name = "未知"
                if agv.status_id == 3:
                    status_name = "閒置"
                elif agv.status_id == 4:
                    status_name = "任務中"
                    
                print(f"  {agv.name}: {status_name} (status_id={agv.status_id})")
            
            # 7. 統計資訊
            print("\n📈 任務統計:")
            stats = ct_manager.get_task_statistics()
            print(f"  待執行: {stats.get('pending', 0)}")
            print(f"  執行中: {stats.get('running', 0)}")
            print(f"  已完成: {stats.get('completed', 0)}")
            
    except Exception as e:
        print(f"❌ 測試過程中發生錯誤: {e}")
        import traceback
        traceback.print_exc()
    
    print("\n✅ 測試完成")


if __name__ == "__main__":
    test_dispatch_logic()
