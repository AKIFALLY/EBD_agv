#!/usr/bin/env python3
"""
測試任務排序邏輯的腳本
驗證 _load_task_from_database() 方法中的 created_at 排序功能
"""

from datetime import datetime, timezone
from dateutil.parser import isoparse


class MockTask:
    """模擬 Task 物件"""

    def __init__(self, id, agv_id, work_id, name, created_at):
        self.id = id
        self.agv_id = agv_id
        self.work_id = work_id
        self.name = name
        self.created_at = created_at


def get_created_at_time(task):
    """取得任務的 created_at 時間，處理 None 值和時區問題（與 idle_state.py 中相同的邏輯）"""
    if hasattr(task, 'created_at') and task.created_at:
        # 如果 created_at 是字串格式，嘗試解析為 datetime
        if isinstance(task.created_at, str):
            try:
                parsed_time = isoparse(task.created_at)
                # 確保時間有時區資訊
                if parsed_time.tzinfo is None:
                    parsed_time = parsed_time.replace(tzinfo=timezone.utc)
                return parsed_time
            except (ImportError, ValueError):
                # 如果無法解析，使用當前時間作為預設值
                return datetime.now(timezone.utc)
        elif isinstance(task.created_at, datetime):
            # 確保 datetime 物件有時區資訊
            if task.created_at.tzinfo is None:
                return task.created_at.replace(tzinfo=timezone.utc)
            return task.created_at
    # 如果 created_at 為 None 或無效，使用最大時間值（排到最後）
    return datetime.max.replace(tzinfo=timezone.utc)


def test_task_sorting():
    """測試任務排序邏輯"""
    print("🧪 開始測試任務排序邏輯...")

    # 建立測試任務資料
    test_tasks = [
        MockTask(
            id=1,
            agv_id=1,
            work_id=2000102,
            name="任務1",
            created_at="2024-01-15T10:30:00Z"
        ),
        MockTask(
            id=2,
            agv_id=1,
            work_id=2000201,
            name="任務2",
            created_at="2024-01-15T09:15:00Z"  # 更早的時間
        ),
        MockTask(
            id=3,
            agv_id=1,
            work_id=2000102,
            name="任務3",
            created_at="2024-01-15T11:45:00Z"  # 更晚的時間
        ),
        MockTask(
            id=4,
            agv_id=1,
            work_id=2000201,
            name="任務4",
            created_at=None  # None 值測試
        ),
        MockTask(
            id=5,
            agv_id=2,  # 不同的 agv_id，應該被過濾掉
            work_id=2000102,
            name="任務5",
            created_at="2024-01-15T08:00:00Z"
        ),
    ]

    print(f"📋 原始任務列表：")
    for task in test_tasks:
        print(f"  ID={task.id}, agv_id={task.agv_id}, created_at={task.created_at}")

    # 模擬 idle_state.py 中的邏輯
    target_agv_id = 1
    matching_tasks = [task for task in test_tasks if task.agv_id == target_agv_id]

    print(f"\n🔍 符合 agv_id={target_agv_id} 的任務：")
    for task in matching_tasks:
        print(f"  ID={task.id}, created_at={task.created_at}")

    # 按 created_at 排序
    sorted_tasks = sorted(matching_tasks, key=get_created_at_time)

    print(f"\n📊 排序後的任務（按 created_at 升序）：")
    for i, task in enumerate(sorted_tasks):
        created_at_time = get_created_at_time(task)
        print(f"  [{i+1}] ID={task.id}, created_at={task.created_at}, 解析時間={created_at_time}")

    # 選擇第一個任務
    if sorted_tasks:
        selected_task = sorted_tasks[0]
        print(
            f"\n✅ 選中的任務：ID={selected_task.id}, name={selected_task.name}, created_at={selected_task.created_at}")

        # 驗證結果
        expected_task_id = 2  # 任務2 有最早的 created_at 時間
        if selected_task.id == expected_task_id:
            print("🎉 測試通過！正確選擇了最早建立的任務。")
        else:
            print(f"❌ 測試失敗！預期選擇任務 {expected_task_id}，實際選擇任務 {selected_task.id}")
    else:
        print("❌ 沒有找到符合條件的任務")


def test_edge_cases():
    """測試邊界情況"""
    print("\n🧪 測試邊界情況...")

    # 測試所有任務都有 None created_at
    tasks_with_none = [
        MockTask(id=1, agv_id=1, work_id=2000102, name="任務1", created_at=None),
        MockTask(id=2, agv_id=1, work_id=2000201, name="任務2", created_at=None),
    ]

    matching_tasks = [task for task in tasks_with_none if task.agv_id == 1]
    sorted_tasks = sorted(matching_tasks, key=get_created_at_time)

    print("📋 所有 created_at 都為 None 的情況：")
    for task in sorted_tasks:
        print(f"  ID={task.id}, created_at={task.created_at}")

    # 測試混合格式
    mixed_format_tasks = [
        MockTask(id=1, agv_id=1, work_id=2000102, name="任務1",
                 created_at=datetime(2024, 1, 15, 10, 30, 0, tzinfo=timezone.utc)),
        MockTask(id=2, agv_id=1, work_id=2000201, name="任務2",
                 created_at="2024-01-15T09:15:00Z"),
        MockTask(id=3, agv_id=1, work_id=2000102, name="任務3",
                 created_at="invalid_date_string"),
    ]

    matching_tasks = [task for task in mixed_format_tasks if task.agv_id == 1]
    sorted_tasks = sorted(matching_tasks, key=get_created_at_time)

    print("\n📋 混合格式的 created_at：")
    for task in sorted_tasks:
        created_at_time = get_created_at_time(task)
        print(f"  ID={task.id}, created_at={task.created_at}, 解析時間={created_at_time}")


if __name__ == "__main__":
    test_task_sorting()
    test_edge_cases()
    print("\n🏁 測試完成！")
