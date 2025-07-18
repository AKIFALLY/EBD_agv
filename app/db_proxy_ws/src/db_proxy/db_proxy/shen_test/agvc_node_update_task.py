#!/usr/bin/env python3
"""
透過 AGVCDatabaseNode ROS2 服務更新 Task 的程式
功能：使用 ROS2 UpdateTask 服務來更新任務，而不是直接操作資料庫
"""

from update_task_data import TaskDataUpdater
from db_proxy_interfaces.msg import Task as TaskMsg
from db_proxy_interfaces.srv import UpdateTask
from rclpy.client import Client
from rclpy.node import Node
import rclpy
import sys
import os
import json
import traceback
from typing import Dict, Any, Optional
from datetime import datetime

# 添加上一層目錄到 Python 路徑
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

# ROS2 相關匯入

# ROS2 服務和訊息匯入

# 匯入資料庫相關（用於查詢現有任務）


class AGVCNodeTaskUpdater(Node):
    """透過 AGVCDatabaseNode ROS2 服務更新 Task 的類別"""

    def __init__(self):
        """初始化 ROS2 節點和服務客戶端"""
        super().__init__('agvc_node_task_updater')

        # 建立 UpdateTask 服務客戶端
        self.update_task_client: Client = self.create_client(
            UpdateTask,
            '/agvc/update_task'
        )

        # 建立資料庫查詢器（用於查詢現有任務）
        self.db_updater = TaskDataUpdater()

        self.get_logger().info("🚀 AGVCNodeTaskUpdater 節點已啟動")
        self.get_logger().info("📡 等待 /agvc/update_task 服務...")

        # 等待服務可用
        if not self.wait_for_service():
            self.get_logger().error("❌ 無法連接到 /agvc/update_task 服務")
            raise RuntimeError("UpdateTask 服務不可用")

        self.get_logger().info("✅ 已連接到 /agvc/update_task 服務")

    def wait_for_service(self, timeout_sec: float = 10.0) -> bool:
        """等待 UpdateTask 服務可用

        Args:
            timeout_sec: 等待超時時間（秒）

        Returns:
            bool: 服務是否可用
        """
        return self.update_task_client.wait_for_service(timeout_sec=timeout_sec)

    def create_task_msg(self, task_data: Dict[str, Any]) -> TaskMsg:
        """建立 TaskMsg 訊息

        Args:
            task_data: Task 資料字典

        Returns:
            TaskMsg: ROS2 Task 訊息
        """
        task_msg = TaskMsg()

        # 安全地設定數值欄位，確保都是整數
        def safe_int(value, default=0):
            if value is None:
                return default
            try:
                return int(value)
            except (ValueError, TypeError):
                return default

        # 設定基本欄位
        task_msg.id = safe_int(task_data.get('id'))
        task_msg.work_id = safe_int(task_data.get('work_id'))
        task_msg.status_id = safe_int(task_data.get('status_id'))
        task_msg.room_id = safe_int(task_data.get('room_id'))
        task_msg.node_id = safe_int(task_data.get('node_id'))
        task_msg.agv_id = safe_int(task_data.get('agv_id'))
        task_msg.priority = safe_int(task_data.get('priority'))

        # 設定字串欄位
        task_msg.name = str(task_data.get('name', ''))
        task_msg.description = str(task_data.get('description', ''))

        # 處理參數欄位（轉換為 JSON 字串）
        parameters = task_data.get('parameters', {})
        if isinstance(parameters, dict):
            task_msg.parameters = json.dumps(parameters)
        elif isinstance(parameters, str):
            task_msg.parameters = parameters
        else:
            task_msg.parameters = ''

        # 處理時間欄位
        created_at = task_data.get('created_at', '')
        updated_at = task_data.get('updated_at', '')

        task_msg.created_at = str(created_at) if created_at else ''
        task_msg.updated_at = str(updated_at) if updated_at else ''

        return task_msg

    def update_task_via_service(self, task_data: Dict[str, Any]) -> Optional[TaskMsg]:
        """透過 ROS2 服務更新任務

        Args:
            task_data: 要更新的任務資料

        Returns:
            Optional[TaskMsg]: 更新後的任務訊息，失敗則返回 None
        """
        try:
            # 建立服務請求
            request = UpdateTask.Request()
            request.task = self.create_task_msg(task_data)

            self.get_logger().info(f"📤 發送更新請求: Task ID {task_data.get('id', 'N/A')}")

            # 呼叫服務
            future = self.update_task_client.call_async(request)

            # 等待回應
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

            if future.result() is not None:
                response = future.result()

                if response.success:
                    self.get_logger().info(f"✅ 任務更新成功: {response.message}")
                    return response.task
                else:
                    self.get_logger().error(f"❌ 任務更新失敗: {response.message}")
                    return None
            else:
                self.get_logger().error("❌ 服務呼叫超時或失敗")
                return None

        except Exception as e:
            self.get_logger().error(f"❌ 服務呼叫異常: {e}")
            traceback.print_exc()
            return None

    def get_existing_task(self, task_id: int) -> Optional[Dict[str, Any]]:
        """取得現有任務資料

        Args:
            task_id: 任務 ID

        Returns:
            Optional[Dict[str, Any]]: 任務資料字典，不存在則返回 None
        """
        try:
            task = self.db_updater.get_task_by_id(task_id)
            if task:
                # 轉換為字典格式
                task_dict = task.model_dump()

                # 處理時間欄位
                if task_dict.get('created_at'):
                    task_dict['created_at'] = task_dict['created_at'].isoformat()
                if task_dict.get('updated_at'):
                    task_dict['updated_at'] = task_dict['updated_at'].isoformat()

                return task_dict
            return None
        except Exception as e:
            self.get_logger().error(f"❌ 查詢任務失敗: {e}")
            return None

    def display_task_info(self, task_msg: TaskMsg):
        """顯示任務資訊

        Args:
            task_msg: 任務訊息
        """
        print(f"\n📋 任務資訊:")
        print(f"   🆔 ID: {task_msg.id}")
        print(f"   📝 名稱: {task_msg.name}")
        print(f"   📄 描述: {task_msg.description}")
        print(f"   🏢 Work ID: {task_msg.work_id}")
        print(f"   📊 狀態 ID: {task_msg.status_id}")
        print(f"   🏠 房間 ID: {task_msg.room_id}")
        print(f"   🔗 節點 ID: {task_msg.node_id}")
        print(f"   🤖 AGV ID: {task_msg.agv_id}")
        print(f"   ⭐ 優先級: {task_msg.priority}")
        print(f"   ⚙️ 參數: {task_msg.parameters}")
        print(f"   🕐 建立時間: {task_msg.created_at}")
        print(f"   🕑 更新時間: {task_msg.updated_at}")

    def cleanup(self):
        """清理資源"""
        if hasattr(self, 'db_updater') and self.db_updater:
            self.db_updater.cleanup()
        self.get_logger().info("🔻 AGVCNodeTaskUpdater 節點已關閉")


def test_update_task_via_service():
    """測試透過 ROS2 服務更新任務"""
    print(f"\n{'🧪'*20}")
    print(f"🧪 開始測試透過 ROS2 服務更新任務")
    print(f"{'🧪'*20}")

    # 初始化 ROS2
    rclpy.init()

    updater = None
    try:
        # 建立更新器節點
        updater = AGVCNodeTaskUpdater()

        # 測試參數
        task_id = 1  # 修改為您想要測試的任務 ID

        print(f"\n📋 查詢現有任務 ID {task_id}...")
        existing_task = updater.get_existing_task(task_id)

        if not existing_task:
            print(f"❌ 任務 ID {task_id} 不存在")
            return

        print(f"✅ 找到任務: {existing_task['name']}")

        # 準備更新資料
        update_data = existing_task.copy()
        update_data.update({
            'priority': 15,
            'description': f'透過 ROS2 服務更新 - {datetime.now().strftime("%H:%M:%S")}',
            'parameters': {
                'update_method': 'ros2_service',
                'test_timestamp': datetime.now().isoformat(),
                'original_priority': existing_task['priority']
            }
        })

        print(f"\n🔄 透過 ROS2 服務更新任務...")
        print(f"📝 更新內容: 優先級 {existing_task['priority']} → {update_data['priority']}")

        # 執行更新
        result = updater.update_task_via_service(update_data)

        if result:
            print(f"✅ 更新成功！")
            updater.display_task_info(result)
        else:
            print(f"❌ 更新失敗")

        print(f"\n{'✅'*20}")
        print(f"✅ 測試完成")
        print(f"{'✅'*20}")

    except Exception as e:
        print(f"\n❌ 測試過程中發生錯誤: {e}")
        traceback.print_exc()
    finally:
        if updater:
            updater.cleanup()
        rclpy.shutdown()


def main():
    """主程式入口"""
    print(f"🎯 AGVCNode Task 更新程式啟動")
    print(f"時間: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

    # 檢查命令列參數
    if len(sys.argv) > 1 and sys.argv[1] == 'test':
        test_update_task_via_service()
    else:
        print(f"\n💡 使用說明:")
        print(f"   python3 agvc_node_update_task.py test  # 執行測試功能")
        print(f"   或者匯入此模組使用 AGVCNodeTaskUpdater 類別")

        print(f"\n📚 AGVCNodeTaskUpdater 主要方法:")
        print(f"   - update_task_via_service(task_data)  # 透過 ROS2 服務更新任務")
        print(f"   - get_existing_task(task_id)          # 查詢現有任務")
        print(f"   - create_task_msg(task_data)          # 建立 TaskMsg 訊息")

        print(f"\n📝 使用範例:")
        print(f"   rclpy.init()")
        print(f"   updater = AGVCNodeTaskUpdater()")
        print(f"   ")
        print(f"   # 更新任務")
        print(f"   task_data = {{'id': 1, 'priority': 10, 'name': '任務名稱'}}")
        print(f"   result = updater.update_task_via_service(task_data)")
        print(f"   ")
        print(f"   # 清理資源")
        print(f"   updater.cleanup()")
        print(f"   rclpy.shutdown()")


if __name__ == "__main__":
    main()
