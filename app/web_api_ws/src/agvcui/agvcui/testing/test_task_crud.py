#!/usr/bin/env python3
"""
Task CRUD 完整功能測試腳本 (ROS2 版本)

測試 Task 的創建、讀取、更新、刪除功能
可以通過 ros2 run agvcui test_task_crud_full 運行
"""

import sys
import os


def test_task_crud():
    """測試 Task CRUD 功能"""
    print("🧪 開始測試 Task CRUD 功能...")

    # 1. 測試獲取選項數據
    print("\n1. 測試獲取選項數據...")
    try:
        from agvcui.db import (
            work_all, task_status_all, room_all, agv_all, node_all
        )

        works = work_all()
        print(f"   ✅ 工作類型數量: {len(works)}")

        task_statuses = task_status_all()
        print(f"   ✅ 任務狀態數量: {len(task_statuses)}")

        rooms = room_all()
        print(f"   ✅ 房間數量: {len(rooms)}")

        agvs = agv_all()
        print(f"   ✅ AGV 數量: {len(agvs)}")

        nodes = node_all()
        print(f"   ✅ 節點數量: {len(nodes)}")

    except Exception as e:
        print(f"   ❌ 獲取選項數據失敗: {e}")
        return False

    # 2. 測試獲取任務列表
    print("\n2. 測試獲取任務列表...")
    try:
        from agvcui.db import get_tasks, count_tasks

        tasks = get_tasks(offset=0, limit=5)
        total = count_tasks()
        print(f"   ✅ 任務總數: {total}")
        print(f"   ✅ 獲取前5個任務: {len(tasks)}")

        if tasks:
            first_task = tasks[0]
            print(f"   ✅ 第一個任務: ID={first_task.id}, 名稱={first_task.name}")

    except Exception as e:
        print(f"   ❌ 獲取任務列表失敗: {e}")
        return False

    # 3. 測試創建任務
    print("\n3. 測試創建任務...")
    try:
        from agvcui.db import create_task

        test_task_data = {
            "name": "ROS2測試任務",
            "description": "這是一個通過ROS2測試創建的任務",
            "priority": 5
        }

        # 如果有可用的選項，使用它們
        if works:
            test_task_data["work_id"] = works[0]["id"]
        if task_statuses:
            test_task_data["status_id"] = task_statuses[0]["id"]
        if rooms:
            test_task_data["room_id"] = rooms[0]["id"]

        new_task = create_task(test_task_data)
        print(f"   ✅ 創建任務成功: ID={new_task.id}, 名稱={new_task.name}")
        test_task_id = new_task.id

    except Exception as e:
        print(f"   ❌ 創建任務失敗: {e}")
        return False

    # 4. 測試獲取單個任務
    print("\n4. 測試獲取單個任務...")
    try:
        from agvcui.db import get_task_by_id

        task = get_task_by_id(test_task_id)
        if task:
            print(f"   ✅ 獲取任務成功: ID={task.id}, 名稱={task.name}")
            print(f"   ✅ 任務描述: {task.description}")
            print(f"   ✅ 優先級: {task.priority}")
        else:
            print("   ❌ 任務不存在")
            return False

    except Exception as e:
        print(f"   ❌ 獲取任務失敗: {e}")
        return False

    # 5. 測試更新任務
    print("\n5. 測試更新任務...")
    try:
        from agvcui.db import update_task

        update_data = {
            "name": "ROS2更新後的測試任務",
            "description": "這是更新後的描述",
            "priority": 10
        }

        updated_task = update_task(test_task_id, update_data)
        if updated_task:
            print(f"   ✅ 更新任務成功: 名稱={updated_task.name}")
            print(f"   ✅ 新描述: {updated_task.description}")
            print(f"   ✅ 新優先級: {updated_task.priority}")
        else:
            print("   ❌ 更新任務失敗")
            return False

    except Exception as e:
        print(f"   ❌ 更新任務失敗: {e}")
        return False

    # 6. 測試刪除任務
    print("\n6. 測試刪除任務...")
    try:
        from agvcui.db import delete_task

        success = delete_task(test_task_id)
        if success:
            print("   ✅ 刪除任務成功")

            # 驗證任務已被刪除
            deleted_task = get_task_by_id(test_task_id)
            if deleted_task is None:
                print("   ✅ 確認任務已被刪除")
            else:
                print("   ❌ 任務仍然存在")
                return False
        else:
            print("   ❌ 刪除任務失敗")
            return False

    except Exception as e:
        print(f"   ❌ 刪除任務失敗: {e}")
        return False

    print("\n🎉 所有 Task CRUD 測試通過！")
    return True


def test_task_form_data():
    """測試任務表單所需的數據"""
    print("\n🧪 測試任務表單數據...")

    try:
        from agvcui.db import (
            work_all, task_status_all, room_all, agv_all, node_all
        )

        # 測試所有下拉選項
        works = work_all()
        task_statuses = task_status_all()
        rooms = room_all()
        agvs = agv_all()
        nodes = node_all()

        print("📋 表單選項數據:")
        print(f"   工作類型: {len(works)} 個")
        if works:
            print(f"   - 示例: {works[0]['name']}")

        print(f"   任務狀態: {len(task_statuses)} 個")
        if task_statuses:
            print(f"   - 示例: {task_statuses[0]['name']}")

        print(f"   房間: {len(rooms)} 個")
        if rooms:
            print(f"   - 示例: {rooms[0]['name']}")

        print(f"   AGV: {len(agvs)} 個")
        if agvs:
            print(f"   - 示例: {agvs[0]['name']}")

        print(f"   節點: {len(nodes)} 個")
        if nodes:
            node_name = nodes[0].get('name', f'Node {nodes[0]["id"]}')
            print(f"   - 示例: {node_name} (ID: {nodes[0]['id']})")

        return True

    except Exception as e:
        print(f"❌ 測試表單數據失敗: {e}")
        return False


def run_tests():
    """運行所有測試"""
    print("🚀 Task CRUD 完整功能測試 (ROS2 版本)")
    print("=" * 50)

    # 測試表單數據
    form_test_passed = test_task_form_data()

    # 測試 CRUD 功能
    crud_test_passed = test_task_crud()

    print("\n" + "=" * 50)
    if form_test_passed and crud_test_passed:
        print("🎉 所有測試通過！Task CRUD 功能正常工作。")
        print("\n📝 現在可以測試以下功能:")
        print("   1. 啟動服務器: ros2 run agvcui agvc_ui_server")
        print("   2. 訪問 /tasks 查看任務列表")
        print("   3. 點擊「新增任務」創建新任務")
        print("   4. 點擊「編輯」修改現有任務")
        print("   5. 點擊「刪除」刪除任務（需要管理員權限）")
        return True
    else:
        print("❌ 部分測試失敗，請檢查配置和數據庫連接。")
        return False


def main():
    """ROS2 入口點"""
    try:
        success = run_tests()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n⚠️  測試被用戶中斷")
        sys.exit(1)
    except Exception as e:
        print(f"❌ 測試運行失敗: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
