#!/usr/bin/env python3

from update_task_data import TaskDataUpdater
import sys
import os

# 添加上一層目錄到 Python 路徑，以便匯入 update_task_data
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

# 現在可以匯入 update_task_data


def main():
    updater = TaskDataUpdater()

    try:
        # 查看現有任務
        tasks = updater.get_all_tasks()
        print(f"📋 找到 {len(tasks)} 個任務:")
        for task in tasks:
            print(f"   ID: {task.id}, 名稱: {task.name}, 優先級: {task.priority}")

        # 選擇要更新的任務 ID
        task_id = 8  # 修改為您想要的任務 ID

        # 檢查任務是否存在
        target_task = updater.get_task_by_id(task_id)
        if not target_task:
            print(f"❌ 任務 ID {task_id} 不存在")
            return

        print(f"\n🎯 選擇更新任務: ID={task_id}, 名稱='{target_task.name}'")

        # 定義更新資料（保留原始名稱，只更新其他欄位）
        update_data = {
            'name': target_task.name,  # 保留原始名稱
            'priority': 12,
            'description': '手動更新測試123'
        }

        # 執行更新
        print(f"🔄 正在更新任務...")
        result = updater.update_task(task_id, update_data)

        if result:
            print("✅ 更新成功！")
            updater.display_task_summary(result)
        else:
            print("❌ 更新失敗")

    finally:
        updater.cleanup()


if __name__ == "__main__":
    main()
