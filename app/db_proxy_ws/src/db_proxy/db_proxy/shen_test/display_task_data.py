#!/usr/bin/env python3
"""
Task 資料顯示程式
功能：存取並顯示 AGVCDatabaseNode 中的 Task 相關資料結構
"""

import sys
import json
import traceback
from typing import List, Optional
from datetime import datetime

# ROS2 相關匯入
import rclpy
from rclpy.node import Node

# 資料庫相關匯入
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import Task
from db_proxy.ros_converter import model_to_msg
from sqlmodel import select

# ROS2 訊息匯入
from db_proxy_interfaces.msg import Tasks, Task as TaskMsg

# 匯入 AGVCDatabaseNode
from db_proxy.agvc_database_node import AGVCDatabaseNode


class TaskDataDisplayer:
    """Task 資料顯示器類別"""
    
    def __init__(self, db_url: str = None):
        """初始化 Task 資料顯示器
        
        Args:
            db_url: 資料庫連線 URL，如果為 None 則使用預設值
        """
        if db_url is None:
            db_url = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'
        
        self.db_url = db_url
        self.pool_agvc = None
        self._initialize_database()
    
    def _initialize_database(self):
        """初始化資料庫連線"""
        try:
            self.pool_agvc = ConnectionPoolManager(self.db_url)
            print(f"✅ 資料庫連線成功: {self.db_url}")
        except Exception as e:
            print(f"❌ 資料庫連線失敗: {e}")
            raise
    
    def get_all_tasks(self) -> List[Task]:
        """從資料庫取得所有 Task 資料
        
        Returns:
            List[Task]: Task 物件列表
        """
        try:
            with self.pool_agvc.get_session() as session:
                tasks = session.exec(select(Task)).all()
                return list(tasks)
        except Exception as e:
            print(f"❌ 查詢 Task 資料失敗: {e}")
            return []
    
    def convert_tasks_to_ros_messages(self, tasks: List[Task]) -> List[TaskMsg]:
        """將 Task 物件轉換為 ROS2 TaskMsg 訊息
        
        Args:
            tasks: Task 物件列表
            
        Returns:
            List[TaskMsg]: TaskMsg 訊息列表
        """
        task_messages = []
        for task in tasks:
            try:
                task_msg = model_to_msg(task, TaskMsg)
                task_messages.append(task_msg)
            except Exception as e:
                print(f"❌ 轉換 Task {task.id} 為 ROS 訊息失敗: {e}")
        
        return task_messages
    
    def create_tasks_message(self, task_messages: List[TaskMsg]) -> Tasks:
        """建立 Tasks 集合訊息
        
        Args:
            task_messages: TaskMsg 訊息列表
            
        Returns:
            Tasks: Tasks 集合訊息
        """
        tasks_msg = Tasks()
        tasks_msg.datas = task_messages
        return tasks_msg
    
    def display_task_object(self, task: Task, index: int = None):
        """顯示單個 Task 物件的詳細資訊
        
        Args:
            task: Task 物件
            index: 索引編號（可選）
        """
        prefix = f"Task[{index}]" if index is not None else "Task"
        
        print(f"\n{'='*60}")
        print(f"📋 {prefix} 物件詳細資訊")
        print(f"{'='*60}")
        print(f"🆔 ID: {task.id}")
        print(f"📝 名稱: {task.name}")
        print(f"📄 描述: {task.description or '無'}")
        print(f"🏢 Work ID: {task.work_id}")
        print(f"📊 狀態 ID: {task.status_id}")
        print(f"🏠 房間 ID: {task.room_id}")
        print(f"🔗 節點 ID: {task.node_id}")
        print(f"🤖 AGV ID: {task.agv_id}")
        print(f"⭐ 優先級: {task.priority}")
        print(f"🔧 任務代碼: {task.mission_code or '無'}")
        print(f"👨‍👩‍👧‍👦 父任務 ID: {task.parent_task_id or '無'}")
        
        # 顯示參數（JSON 格式）
        if task.parameters:
            print(f"⚙️ 參數:")
            try:
                formatted_params = json.dumps(task.parameters, indent=2, ensure_ascii=False)
                print(f"   {formatted_params}")
            except Exception as e:
                print(f"   {task.parameters} (JSON 格式化失敗: {e})")
        else:
            print(f"⚙️ 參數: 無")
        
        # 顯示時間戳
        print(f"🕐 建立時間: {task.created_at}")
        print(f"🕑 更新時間: {task.updated_at or '無'}")
    
    def display_task_message(self, task_msg: TaskMsg, index: int = None):
        """顯示單個 TaskMsg 訊息的詳細資訊
        
        Args:
            task_msg: TaskMsg 訊息
            index: 索引編號（可選）
        """
        prefix = f"TaskMsg[{index}]" if index is not None else "TaskMsg"
        
        print(f"\n{'='*60}")
        print(f"📨 {prefix} ROS2 訊息詳細資訊")
        print(f"{'='*60}")
        print(f"🆔 ID: {task_msg.id}")
        print(f"📝 名稱: {task_msg.name}")
        print(f"📄 描述: {task_msg.description}")
        print(f"🏢 Work ID: {task_msg.work_id}")
        print(f"📊 狀態 ID: {task_msg.status_id}")
        print(f"🏠 房間 ID: {task_msg.room_id}")
        print(f"🔗 節點 ID: {task_msg.node_id}")
        print(f"🤖 AGV ID: {task_msg.agv_id}")
        print(f"⭐ 優先級: {task_msg.priority}")
        print(f"⚙️ 參數: {task_msg.parameters}")
        print(f"🕐 建立時間: {task_msg.created_at}")
        print(f"🕑 更新時間: {task_msg.updated_at}")
    
    def display_tasks_collection(self, tasks_msg: Tasks):
        """顯示 Tasks 集合訊息的詳細資訊
        
        Args:
            tasks_msg: Tasks 集合訊息
        """
        print(f"\n{'='*80}")
        print(f"📦 Tasks 集合訊息詳細資訊")
        print(f"{'='*80}")
        print(f"📊 總任務數量: {len(tasks_msg.datas)}")
        
        if tasks_msg.datas:
            print(f"📋 任務列表:")
            for i, task_msg in enumerate(tasks_msg.datas):
                print(f"   [{i+1}] ID: {task_msg.id}, 名稱: {task_msg.name}, 狀態: {task_msg.status_id}")
        else:
            print(f"📋 任務列表: 空")
    
    def display_all_task_data(self):
        """顯示所有 Task 相關資料"""
        print(f"\n{'🚀'*20}")
        print(f"🚀 開始顯示 Task 資料")
        print(f"{'🚀'*20}")
        
        try:
            # 1. 取得所有 Task 物件
            print(f"\n📥 正在從資料庫取得 Task 資料...")
            tasks = self.get_all_tasks()
            print(f"✅ 成功取得 {len(tasks)} 筆 Task 資料")
            
            # 2. 顯示每個 Task 物件
            if tasks:
                print(f"\n📋 顯示 Task 物件詳細資訊:")
                for i, task in enumerate(tasks):
                    self.display_task_object(task, i + 1)
            else:
                print(f"\n⚠️ 資料庫中沒有 Task 資料")
                return
            
            # 3. 轉換為 ROS2 訊息
            print(f"\n🔄 正在轉換為 ROS2 訊息...")
            task_messages = self.convert_tasks_to_ros_messages(tasks)
            print(f"✅ 成功轉換 {len(task_messages)} 筆 TaskMsg 訊息")
            
            # 4. 顯示每個 TaskMsg 訊息
            if task_messages:
                print(f"\n📨 顯示 TaskMsg 訊息詳細資訊:")
                for i, task_msg in enumerate(task_messages):
                    self.display_task_message(task_msg, i + 1)
            
            # 5. 建立並顯示 Tasks 集合
            print(f"\n📦 正在建立 Tasks 集合訊息...")
            tasks_collection = self.create_tasks_message(task_messages)
            self.display_tasks_collection(tasks_collection)
            
            print(f"\n{'✅'*20}")
            print(f"✅ Task 資料顯示完成")
            print(f"{'✅'*20}")
            
        except Exception as e:
            print(f"\n❌ 顯示 Task 資料時發生錯誤: {e}")
            print(f"錯誤詳情:")
            traceback.print_exc()
    
    def cleanup(self):
        """清理資源"""
        if self.pool_agvc:
            try:
                self.pool_agvc.shutdown()
                print(f"🔻 資料庫連線已關閉")
            except Exception as e:
                print(f"⚠️ 關閉資料庫連線時發生錯誤: {e}")


def main():
    """主程式入口"""
    print(f"🎯 Task 資料顯示程式啟動")
    print(f"時間: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    
    displayer = None
    try:
        # 建立顯示器
        displayer = TaskDataDisplayer()
        
        # 顯示所有 Task 資料
        displayer.display_all_task_data()
        
    except KeyboardInterrupt:
        print(f"\n⚠️ 程式被使用者中斷")
    except Exception as e:
        print(f"\n❌ 程式執行時發生錯誤: {e}")
        traceback.print_exc()
    finally:
        # 清理資源
        if displayer:
            displayer.cleanup()
        print(f"\n👋 程式結束")


if __name__ == "__main__":
    main()
