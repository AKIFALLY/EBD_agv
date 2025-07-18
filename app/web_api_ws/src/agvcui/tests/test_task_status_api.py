#!/usr/bin/env python3
"""
任務狀態 API 測試腳本
驗證任務狀態相關的 API 端點和資料一致性
"""

import sys
import os
import asyncio
import json
from pathlib import Path

# 添加專案路徑
sys.path.append(str(Path(__file__).parent.parent))

from agvcui.database.task_ops import task_all, get_tasks
from agvcui.database.connection import connection_pool
from db_proxy.models import TaskStatus
from sqlmodel import select


def test_task_status_consistency():
    """測試任務狀態定義的一致性"""
    print("🔍 測試任務狀態定義一致性...")
    
    # 預期的狀態定義（來自 13_works_tasks.py）
    expected_statuses = [
        {"id": 0, "name": "請求中", "description": "UI-請求執行任務"},
        {"id": 1, "name": "待處理", "description": "WCS-任務已接受，待處理"},
        {"id": 2, "name": "待執行", "description": "RCS-任務已派發，待執行"},
        {"id": 3, "name": "執行中", "description": "AGV-任務正在執行"},
        {"id": 4, "name": "已完成", "description": "AGV-任務已完成"},
        {"id": 5, "name": "取消中", "description": "任務取消"},
        {"id": 51, "name": "WCS-取消中", "description": "WCS-任務取消中，待處理"},
        {"id": 52, "name": "RCS-取消中", "description": "RCS-任務取消中，取消中"},
        {"id": 53, "name": "AGV-取消中", "description": "AGV-取消完成"},
        {"id": 54, "name": "已取消", "description": "任務已取消"},
        {"id": 6, "name": "錯誤", "description": "錯誤"},
    ]
    
    try:
        with connection_pool.get_session() as session:
            # 獲取資料庫中的狀態定義
            statement = select(TaskStatus).order_by(TaskStatus.id)
            db_statuses = session.exec(statement).all()
            
            print(f"📊 資料庫中找到 {len(db_statuses)} 個狀態定義")
            print(f"📋 預期 {len(expected_statuses)} 個狀態定義")
            
            # 檢查每個預期狀態
            missing_statuses = []
            inconsistent_statuses = []
            
            for expected in expected_statuses:
                db_status = next((s for s in db_statuses if s.id == expected["id"]), None)
                
                if not db_status:
                    missing_statuses.append(expected)
                elif db_status.name != expected["name"]:
                    inconsistent_statuses.append({
                        "id": expected["id"],
                        "expected_name": expected["name"],
                        "actual_name": db_status.name,
                        "expected_desc": expected["description"],
                        "actual_desc": db_status.description
                    })
            
            # 報告結果
            if not missing_statuses and not inconsistent_statuses:
                print("✅ 所有狀態定義與預期一致")
                return True
            else:
                if missing_statuses:
                    print("❌ 缺少的狀態定義：")
                    for status in missing_statuses:
                        print(f"   - ID {status['id']}: {status['name']}")
                
                if inconsistent_statuses:
                    print("❌ 不一致的狀態定義：")
                    for status in inconsistent_statuses:
                        print(f"   - ID {status['id']}: 預期='{status['expected_name']}', 實際='{status['actual_name']}'")
                
                return False
                
    except Exception as e:
        print(f"❌ 測試失敗: {e}")
        return False


def test_task_data_structure():
    """測試任務資料結構"""
    print("\n🔍 測試任務資料結構...")
    
    try:
        # 獲取任務資料
        tasks = task_all()
        print(f"📊 找到 {len(tasks)} 個任務")
        
        if not tasks:
            print("⚠️  沒有任務資料可供測試")
            return True
        
        # 檢查第一個任務的結構
        sample_task = tasks[0]
        required_fields = ['id', 'status_id', 'name']
        
        missing_fields = []
        for field in required_fields:
            if field not in sample_task:
                missing_fields.append(field)
        
        if missing_fields:
            print(f"❌ 任務資料缺少必要欄位: {missing_fields}")
            return False
        
        # 檢查狀態 ID 的有效性
        invalid_status_tasks = []
        for task in tasks[:10]:  # 只檢查前10個任務
            status_id = task.get('status_id')
            if status_id is None:
                invalid_status_tasks.append(f"任務 {task['id']}: status_id 為 None")
            elif not isinstance(status_id, int):
                invalid_status_tasks.append(f"任務 {task['id']}: status_id 不是整數 ({type(status_id)})")
        
        if invalid_status_tasks:
            print("❌ 發現無效的狀態 ID：")
            for msg in invalid_status_tasks:
                print(f"   - {msg}")
            return False
        
        print("✅ 任務資料結構正確")
        return True
        
    except Exception as e:
        print(f"❌ 測試失敗: {e}")
        return False


def test_task_status_distribution():
    """測試任務狀態分佈"""
    print("\n🔍 測試任務狀態分佈...")
    
    try:
        tasks = task_all()
        
        if not tasks:
            print("⚠️  沒有任務資料可供測試")
            return True
        
        # 統計狀態分佈
        status_counts = {}
        for task in tasks:
            status_id = task.get('status_id')
            status_counts[status_id] = status_counts.get(status_id, 0) + 1
        
        print("📊 任務狀態分佈：")
        for status_id, count in sorted(status_counts.items()):
            print(f"   - 狀態 {status_id}: {count} 個任務")
        
        # 檢查是否有未知狀態
        with connection_pool.get_session() as session:
            statement = select(TaskStatus.id)
            valid_status_ids = set(session.exec(statement).all())
        
        unknown_statuses = set(status_counts.keys()) - valid_status_ids
        if unknown_statuses:
            print(f"❌ 發現未知狀態 ID: {unknown_statuses}")
            return False
        
        print("✅ 所有狀態 ID 都是有效的")
        return True
        
    except Exception as e:
        print(f"❌ 測試失敗: {e}")
        return False


def main():
    """主測試函數"""
    print("🚀 開始任務狀態 API 測試")
    print("=" * 50)
    
    tests = [
        ("任務狀態定義一致性", test_task_status_consistency),
        ("任務資料結構", test_task_data_structure),
        ("任務狀態分佈", test_task_status_distribution),
    ]
    
    results = []
    for test_name, test_func in tests:
        try:
            result = test_func()
            results.append((test_name, result))
        except Exception as e:
            print(f"❌ 測試 '{test_name}' 發生異常: {e}")
            results.append((test_name, False))
    
    # 總結報告
    print("\n" + "=" * 50)
    print("📋 測試結果總結")
    print("=" * 50)
    
    passed = 0
    total = len(results)
    
    for test_name, result in results:
        status = "✅ 通過" if result else "❌ 失敗"
        print(f"{status} {test_name}")
        if result:
            passed += 1
    
    print(f"\n🎯 總計: {passed}/{total} 個測試通過")
    
    if passed == total:
        print("🎉 所有測試都通過了！")
        return 0
    else:
        print("⚠️  有測試失敗，請檢查上述錯誤訊息")
        return 1


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
