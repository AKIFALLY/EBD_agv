#!/usr/bin/env python3
"""
Task 資料更新程式
功能：提供完整的 Task 資料 CRUD 操作功能
"""

import sys
import json
import traceback
from typing import List, Optional, Dict, Any, Union
from datetime import datetime, timezone

# ROS2 相關匯入
import rclpy
from rclpy.node import Node

# 資料庫相關匯入
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import Task
from db_proxy.crud.task_crud import task_crud
from db_proxy.ros_converter import model_to_msg, msg_to_model
from sqlmodel import select

# ROS2 訊息匯入
from db_proxy_interfaces.msg import Tasks, Task as TaskMsg

# 匯入 AGVCDatabaseNode（可選，用於參考）
from db_proxy.agvc_database_node import AGVCDatabaseNode


class TaskDataUpdater:
    """Task 資料更新器類別"""

    def __init__(self, db_url: str = None):
        """初始化 Task 資料更新器

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

    def _validate_task_data(self, task_data: Dict[str, Any]) -> Dict[str, str]:
        """驗證 Task 資料的有效性

        Args:
            task_data: Task 資料字典

        Returns:
            Dict[str, str]: 驗證錯誤訊息字典，空字典表示驗證通過
        """
        errors = {}

        # 檢查必要欄位
        if not task_data.get('name'):
            errors['name'] = '任務名稱不能為空'

        # 檢查數值欄位的有效性
        numeric_fields = ['work_id', 'status_id', 'room_id',
                          'node_id', 'agv_id', 'priority', 'parent_task_id']
        for field in numeric_fields:
            value = task_data.get(field)
            if value is not None and not isinstance(value, int):
                try:
                    int(value)
                except (ValueError, TypeError):
                    errors[field] = f'{field} 必須是有效的整數'

        # 檢查優先級範圍
        priority = task_data.get('priority', 0)
        if isinstance(priority, (int, str)):
            try:
                priority_int = int(priority)
                if priority_int < 0 or priority_int > 100:
                    errors['priority'] = '優先級必須在 0-100 之間'
            except (ValueError, TypeError):
                errors['priority'] = '優先級必須是有效的整數'

        # 檢查 JSON 參數格式
        parameters = task_data.get('parameters')
        if parameters is not None:
            if isinstance(parameters, str):
                try:
                    json.loads(parameters)
                except json.JSONDecodeError:
                    errors['parameters'] = '參數必須是有效的 JSON 格式'
            elif not isinstance(parameters, dict):
                errors['parameters'] = '參數必須是字典或 JSON 字串格式'

        return errors

    def get_task_by_id(self, task_id: int) -> Optional[Task]:
        """根據 ID 取得 Task 資料

        Args:
            task_id: Task ID

        Returns:
            Optional[Task]: Task 物件，如果不存在則返回 None
        """
        try:
            with self.pool_agvc.get_session() as session:
                task = task_crud.get_by_id(session, task_id)
                return task
        except Exception as e:
            print(f"❌ 查詢 Task ID {task_id} 失敗: {e}")
            return None

    def create_task(self, task_data: Union[Dict[str, Any], Task, TaskMsg]) -> Optional[Task]:
        """新增 Task 記錄

        Args:
            task_data: Task 資料（字典、Task 物件或 TaskMsg 訊息）

        Returns:
            Optional[Task]: 新建的 Task 物件，失敗則返回 None
        """
        try:
            # 處理不同類型的輸入
            if isinstance(task_data, TaskMsg):
                task_obj = msg_to_model(task_data, Task)
            elif isinstance(task_data, dict):
                # 驗證資料
                errors = self._validate_task_data(task_data)
                if errors:
                    print(f"❌ 資料驗證失敗:")
                    for field, error in errors.items():
                        print(f"   {field}: {error}")
                    return None

                # 處理參數欄位
                if 'parameters' in task_data and isinstance(task_data['parameters'], str):
                    try:
                        task_data['parameters'] = json.loads(task_data['parameters'])
                    except json.JSONDecodeError:
                        print(f"❌ 參數 JSON 格式錯誤")
                        return None

                # 設定時間戳
                current_time = datetime.now(timezone.utc)
                task_data['created_at'] = current_time
                task_data['updated_at'] = current_time

                # 確保 ID 為 None（新建記錄）
                task_data['id'] = None

                task_obj = Task(**task_data)
            elif isinstance(task_data, Task):
                task_obj = task_data
                # 確保是新建記錄
                task_obj.id = None
                current_time = datetime.now(timezone.utc)
                task_obj.created_at = current_time
                task_obj.updated_at = current_time
            else:
                print(f"❌ 不支援的資料類型: {type(task_data)}")
                return None

            # 執行新增操作
            with self.pool_agvc.get_session() as session:
                result = task_crud.create(session, task_obj)
                print(f"✅ 成功新增 Task，ID: {result.id}")
                return result

        except Exception as e:
            print(f"❌ 新增 Task 失敗: {e}")
            traceback.print_exc()
            return None

    def update_task(self, task_id: int, task_data: Union[Dict[str, Any], Task, TaskMsg]) -> Optional[Task]:
        """更新 Task 記錄

        Args:
            task_id: 要更新的 Task ID
            task_data: Task 資料（字典、Task 物件或 TaskMsg 訊息）

        Returns:
            Optional[Task]: 更新後的 Task 物件，失敗則返回 None
        """
        try:
            # 檢查 Task 是否存在
            existing_task = self.get_task_by_id(task_id)
            if not existing_task:
                print(f"❌ Task ID {task_id} 不存在")
                return None

            # 處理不同類型的輸入
            if isinstance(task_data, TaskMsg):
                task_obj = msg_to_model(task_data, Task)
            elif isinstance(task_data, dict):
                # 驗證資料
                errors = self._validate_task_data(task_data)
                if errors:
                    print(f"❌ 資料驗證失敗:")
                    for field, error in errors.items():
                        print(f"   {field}: {error}")
                    return None

                # 處理參數欄位
                if 'parameters' in task_data and isinstance(task_data['parameters'], str):
                    try:
                        task_data['parameters'] = json.loads(task_data['parameters'])
                    except json.JSONDecodeError:
                        print(f"❌ 參數 JSON 格式錯誤")
                        return None

                # 設定 ID 和更新時間
                task_data['id'] = task_id
                task_data['updated_at'] = datetime.now(timezone.utc)

                task_obj = Task(**task_data)
            elif isinstance(task_data, Task):
                task_obj = task_data
                task_obj.id = task_id
                task_obj.updated_at = datetime.now(timezone.utc)
            else:
                print(f"❌ 不支援的資料類型: {type(task_data)}")
                return None

            # 執行更新操作
            with self.pool_agvc.get_session() as session:
                result = task_crud.update(session, task_id, task_obj)
                print(f"✅ 成功更新 Task ID {task_id}")
                return result

        except Exception as e:
            print(f"❌ 更新 Task ID {task_id} 失敗: {e}")
            traceback.print_exc()
            return None

    def delete_task(self, task_id: int) -> bool:
        """刪除 Task 記錄

        Args:
            task_id: 要刪除的 Task ID

        Returns:
            bool: 刪除成功返回 True，失敗返回 False
        """
        try:
            # 檢查 Task 是否存在
            existing_task = self.get_task_by_id(task_id)
            if not existing_task:
                print(f"❌ Task ID {task_id} 不存在")
                return False

            # 執行刪除操作
            with self.pool_agvc.get_session() as session:
                success = task_crud.delete(session, task_id)
                if success:
                    print(f"✅ 成功刪除 Task ID {task_id}")
                else:
                    print(f"❌ 刪除 Task ID {task_id} 失敗")
                return success

        except Exception as e:
            print(f"❌ 刪除 Task ID {task_id} 失敗: {e}")
            traceback.print_exc()
            return False

    def create_or_update_task(self, task_data: Union[Dict[str, Any], Task, TaskMsg]) -> Optional[Task]:
        """新增或更新 Task 記錄（根據是否有 ID 自動判斷）

        Args:
            task_data: Task 資料（字典、Task 物件或 TaskMsg 訊息）

        Returns:
            Optional[Task]: 操作後的 Task 物件，失敗則返回 None
        """
        try:
            # 取得 ID
            task_id = None
            if isinstance(task_data, TaskMsg):
                task_id = task_data.id if task_data.id > 0 else None
            elif isinstance(task_data, dict):
                task_id = task_data.get('id')
                if task_id == 0:
                    task_id = None
            elif isinstance(task_data, Task):
                task_id = task_data.id

            # 根據是否有 ID 決定操作
            if task_id is None:
                print(f"📝 執行新增操作...")
                return self.create_task(task_data)
            else:
                print(f"📝 執行更新操作 (ID: {task_id})...")
                return self.update_task(task_id, task_data)

        except Exception as e:
            print(f"❌ 新增或更新 Task 失敗: {e}")
            traceback.print_exc()
            return None

    def batch_update_tasks(self, tasks_data: List[Union[Dict[str, Any], Task, TaskMsg]]) -> List[Optional[Task]]:
        """批量更新多個 Task 記錄

        Args:
            tasks_data: Task 資料列表

        Returns:
            List[Optional[Task]]: 操作結果列表，成功的項目為 Task 物件，失敗的為 None
        """
        results = []
        print(f"📦 開始批量處理 {len(tasks_data)} 個 Task...")

        for i, task_data in enumerate(tasks_data):
            print(f"\n📝 處理第 {i+1}/{len(tasks_data)} 個 Task...")
            try:
                result = self.create_or_update_task(task_data)
                results.append(result)
                if result:
                    print(f"✅ 第 {i+1} 個 Task 處理成功")
                else:
                    print(f"❌ 第 {i+1} 個 Task 處理失敗")
            except Exception as e:
                print(f"❌ 第 {i+1} 個 Task 處理異常: {e}")
                results.append(None)

        success_count = sum(1 for r in results if r is not None)
        print(f"\n📊 批量處理完成: 成功 {success_count}/{len(tasks_data)} 個")
        return results

    def get_all_tasks(self) -> List[Task]:
        """取得所有 Task 記錄

        Returns:
            List[Task]: Task 物件列表
        """
        try:
            with self.pool_agvc.get_session() as session:
                tasks = session.exec(select(Task)).all()
                return list(tasks)
        except Exception as e:
            print(f"❌ 查詢所有 Task 失敗: {e}")
            return []

    def display_task_summary(self, task: Task):
        """顯示 Task 摘要資訊

        Args:
            task: Task 物件
        """
        print(
            f"   🆔 ID: {task.id} | 📝 名稱: {task.name} | 📊 狀態: {task.status_id} | ⭐ 優先級: {task.priority}")

    def cleanup(self):
        """清理資源"""
        if self.pool_agvc:
            try:
                self.pool_agvc.shutdown()
                print(f"🔻 資料庫連線已關閉")
            except Exception as e:
                print(f"⚠️ 關閉資料庫連線時發生錯誤: {e}")


def run_tests():
    """執行測試功能"""
    print(f"\n{'🧪'*20}")
    print(f"🧪 開始執行 Task 更新測試")
    print(f"{'🧪'*20}")

    updater = None
    try:
        # 初始化更新器
        updater = TaskDataUpdater()

        # 測試 1: 新增 Task
        print(f"\n🧪 測試 1: 新增新的 Task")
        new_task_data = {
            'name': '測試任務-自動新增',
            'description': '這是一個測試用的自動新增任務',
            'work_id': 100001,
            'status_id': 1,
            'room_id': 2,
            'agv_id': 1,
            'priority': 5,
            'parameters': {
                'test_mode': True,
                'created_by': 'update_task_data.py',
                'test_timestamp': datetime.now().isoformat()
            }
        }

        created_task = updater.create_task(new_task_data)
        if created_task:
            print(f"✅ 測試 1 通過: 成功新增 Task")
            updater.display_task_summary(created_task)

            # 測試 2: 更新剛新增的 Task
            print(f"\n🧪 測試 2: 更新剛新增的 Task")
            update_data = {
                'name': '測試任務-已更新',
                'description': '這個任務已經被更新過了',
                'priority': 8,
                'parameters': {
                    'test_mode': True,
                    'updated_by': 'update_task_data.py',
                    'update_timestamp': datetime.now().isoformat()
                }
            }

            updated_task = updater.update_task(created_task.id, update_data)
            if updated_task:
                print(f"✅ 測試 2 通過: 成功更新 Task")
                updater.display_task_summary(updated_task)
            else:
                print(f"❌ 測試 2 失敗: 更新 Task 失敗")

            # 測試 3: 使用 create_or_update 方法
            print(f"\n🧪 測試 3: 使用 create_or_update 方法")
            update_data_2 = {
                'id': created_task.id,
                'name': '測試任務-create_or_update',
                'description': '使用 create_or_update 方法更新',
                'priority': 10
            }

            result_task = updater.create_or_update_task(update_data_2)
            if result_task:
                print(f"✅ 測試 3 通過: create_or_update 成功")
                updater.display_task_summary(result_task)
            else:
                print(f"❌ 測試 3 失敗: create_or_update 失敗")

            # 測試 4: 刪除測試 Task
            print(f"\n🧪 測試 4: 刪除測試 Task")
            delete_success = updater.delete_task(created_task.id)
            if delete_success:
                print(f"✅ 測試 4 通過: 成功刪除測試 Task")
            else:
                print(f"❌ 測試 4 失敗: 刪除測試 Task 失敗")
        else:
            print(f"❌ 測試 1 失敗: 新增 Task 失敗")

        # 測試 5: 批量操作測試
        print(f"\n🧪 測試 5: 批量操作測試")
        batch_data = [
            {
                'name': '批量測試任務 1',
                'description': '批量操作測試 1',
                'work_id': 100001,
                'status_id': 1,
                'priority': 1
            },
            {
                'name': '批量測試任務 2',
                'description': '批量操作測試 2',
                'work_id': 100001,
                'status_id': 1,
                'priority': 2
            }
        ]

        batch_results = updater.batch_update_tasks(batch_data)
        success_count = sum(1 for r in batch_results if r is not None)
        if success_count == len(batch_data):
            print(f"✅ 測試 5 通過: 批量操作全部成功")

            # 清理批量測試資料
            print(f"🧹 清理批量測試資料...")
            for result in batch_results:
                if result:
                    updater.delete_task(result.id)
        else:
            print(f"⚠️ 測試 5 部分成功: {success_count}/{len(batch_data)} 個成功")

        print(f"\n{'✅'*20}")
        print(f"✅ 測試完成")
        print(f"{'✅'*20}")

    except Exception as e:
        print(f"\n❌ 測試過程中發生錯誤: {e}")
        traceback.print_exc()
    finally:
        if updater:
            updater.cleanup()


def test_update_only():
    """只測試更新任務功能"""
    print(f"\n{'🔄'*20}")
    print(f"🔄 開始測試更新任務功能")
    print(f"{'🔄'*20}")

    updater = None
    try:
        # 初始化更新器
        updater = TaskDataUpdater()

        # 先查看現有任務
        print(f"\n📋 查看現有任務...")
        all_tasks = updater.get_all_tasks()
        if not all_tasks:
            print(f"❌ 資料庫中沒有任務，無法測試更新功能")
            print(f"💡 建議先執行完整測試來建立一些測試資料")
            return

        print(f"✅ 找到 {len(all_tasks)} 個現有任務:")
        for i, task in enumerate(all_tasks[:5]):  # 只顯示前5個
            print(f"   [{i+1}] ID: {task.id}, 名稱: {task.name}, 優先級: {task.priority}")

        # 選擇第一個任務進行更新測試
        test_task = all_tasks[0]
        original_name = test_task.name
        original_priority = test_task.priority

        print(f"\n🎯 選擇任務 ID {test_task.id} 進行更新測試")
        print(f"📝 原始資料: 名稱='{original_name}', 優先級={original_priority}")

        # 測試 1: 基本更新
        print(f"\n🧪 測試 1: 基本更新（修改名稱和優先級）")
        update_data_1 = {
            'name': f'{original_name} [已更新-測試1]',
            'priority': (original_priority + 1) % 10,
            'description': f'更新測試 - {datetime.now().strftime("%H:%M:%S")}'
        }

        updated_task_1 = updater.update_task(test_task.id, update_data_1)
        if updated_task_1:
            print(f"✅ 測試 1 成功")
            updater.display_task_summary(updated_task_1)
        else:
            print(f"❌ 測試 1 失敗")

        # 測試 2: 更新參數
        print(f"\n🧪 測試 2: 更新 JSON 參數")
        update_data_2 = {
            'parameters': {
                'update_test': True,
                'test_number': 2,
                'timestamp': datetime.now().isoformat(),
                'original_name': original_name
            }
        }

        updated_task_2 = updater.update_task(test_task.id, update_data_2)
        if updated_task_2:
            print(f"✅ 測試 2 成功")
            print(f"📄 參數內容: {updated_task_2.parameters}")
        else:
            print(f"❌ 測試 2 失敗")

        # 測試 3: 使用 create_or_update 方法
        print(f"\n🧪 測試 3: 使用 create_or_update 方法更新")
        update_data_3 = {
            'id': test_task.id,
            'name': f'{original_name} [create_or_update測試]',
            'priority': (original_priority + 2) % 10
        }

        updated_task_3 = updater.create_or_update_task(update_data_3)
        if updated_task_3:
            print(f"✅ 測試 3 成功")
            updater.display_task_summary(updated_task_3)
        else:
            print(f"❌ 測試 3 失敗")

        # 恢復原始資料
        print(f"\n🔄 恢復原始資料...")
        restore_data = {
            'name': original_name,
            'priority': original_priority,
            'description': test_task.description
        }

        restored_task = updater.update_task(test_task.id, restore_data)
        if restored_task:
            print(f"✅ 成功恢復原始資料")
            updater.display_task_summary(restored_task)
        else:
            print(f"⚠️ 恢復原始資料失敗")

        print(f"\n{'✅'*20}")
        print(f"✅ 更新測試完成")
        print(f"{'✅'*20}")

    except Exception as e:
        print(f"\n❌ 更新測試過程中發生錯誤: {e}")
        traceback.print_exc()
    finally:
        if updater:
            updater.cleanup()


def main():
    """主程式入口"""
    print(f"🎯 Task 資料更新程式啟動")
    print(f"時間: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

    # 檢查命令列參數
    if len(sys.argv) > 1:
        if sys.argv[1] == 'test':
            run_tests()
        elif sys.argv[1] == 'update':
            test_update_only()
        else:
            print(f"❌ 未知的參數: {sys.argv[1]}")
            print(f"💡 可用參數: test, update")
    else:
        print(f"\n💡 使用說明:")
        print(f"   python3 update_task_data.py test  # 執行測試功能")
        print(f"   或者匯入此模組使用 TaskDataUpdater 類別")

        print(f"\n📚 TaskDataUpdater 主要方法:")
        print(f"   - create_task(task_data)           # 新增 Task")
        print(f"   - update_task(task_id, task_data)  # 更新 Task")
        print(f"   - delete_task(task_id)             # 刪除 Task")
        print(f"   - create_or_update_task(task_data) # 自動新增或更新")
        print(f"   - batch_update_tasks(tasks_data)   # 批量操作")
        print(f"   - get_task_by_id(task_id)          # 查詢 Task")
        print(f"   - get_all_tasks()                  # 查詢所有 Task")

        print(f"\n📝 使用範例:")
        print(f"   from db_proxy.update_task_data import TaskDataUpdater")
        print(f"   updater = TaskDataUpdater()")
        print(f"   ")
        print(f"   # 新增任務")
        print(f"   task_data = {{'name': '新任務', 'priority': 5}}")
        print(f"   new_task = updater.create_task(task_data)")
        print(f"   ")
        print(f"   # 更新任務")
        print(f"   update_data = {{'priority': 10}}")
        print(f"   updated_task = updater.update_task(task_id, update_data)")
        print(f"   ")
        print(f"   # 清理資源")
        print(f"   updater.cleanup()")


if __name__ == "__main__":
    main()
