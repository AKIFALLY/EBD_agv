#!/usr/bin/env python3
"""
測試 Kuka API parameters 更新功能
"""
import sys
import os
sys.path.append('/app/db_proxy_ws/src')

import requests
import json
from datetime import datetime
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import Task
from sqlmodel import select

def check_task_parameters_before_and_after():
    """檢查任務 parameters 在 API 調用前後的變化"""
    
    # 資料庫連接
    db_url = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'
    db_pool = ConnectionPoolManager(db_url)
    
    # 測試的 mission_code
    test_mission_code = "1357"
    
    print(f"🔍 檢查 mission_code '{test_mission_code}' 的任務 parameters 更新...")
    
    try:
        # 1. 檢查任務調用 API 前的狀態
        with db_pool.get_session() as session:
            task = session.exec(
                select(Task).where(Task.mission_code == test_mission_code)
            ).first()
            
            if not task:
                print(f"❌ 找不到 mission_code 為 '{test_mission_code}' 的任務")
                print("請先執行: python tests/create_test_task.py")
                return
            
            print(f"\n📋 API 調用前的任務狀態:")
            print(f"   Task ID: {task.id}")
            print(f"   Mission Code: {task.mission_code}")
            print(f"   Status ID: {task.status_id}")
            print(f"   Parameters: {json.dumps(task.parameters or {}, indent=2, ensure_ascii=False)}")
            print(f"   Updated At: {task.updated_at}")
            
            original_params = task.parameters or {}
            original_status = task.status_id
            original_updated_at = task.updated_at
        
        # 2. 調用 API
        url = "http://localhost:8000/interfaces/api/amr/missionStateCallback"
        test_data = {
            "missionCode": test_mission_code,
            "missionStatus": "MOVE_BEGIN",
            "robotId": "robot_test_123",
            "containerCode": "container_test_456",
            "currentPosition": "position_test_789",
            "slotCode": "slot_A1",
            "viewBoardType": "test_board",
            "message": "測試 parameters 更新",
            "missionData": {
                "test_key": "test_value",
                "timestamp": datetime.now().isoformat()
            }
        }
        
        print(f"\n🚀 調用 API...")
        print(f"URL: {url}")
        print(f"測試資料: {json.dumps(test_data, indent=2, ensure_ascii=False)}")
        
        response = requests.post(url, json=test_data, headers={"Content-Type": "application/json"})
        
        print(f"\n📊 API 回應:")
        print(f"狀態碼: {response.status_code}")
        print(f"回應內容: {response.text}")
        
        if response.status_code != 200:
            print("❌ API 調用失敗，無法繼續檢查")
            return
        
        # 3. 檢查任務調用 API 後的狀態
        with db_pool.get_session() as session:
            updated_task = session.exec(
                select(Task).where(Task.mission_code == test_mission_code)
            ).first()
            
            print(f"\n📋 API 調用後的任務狀態:")
            print(f"   Task ID: {updated_task.id}")
            print(f"   Mission Code: {updated_task.mission_code}")
            print(f"   Status ID: {updated_task.status_id}")
            print(f"   Parameters: {json.dumps(updated_task.parameters or {}, indent=2, ensure_ascii=False)}")
            print(f"   Updated At: {updated_task.updated_at}")
        
        # 4. 比較變化
        print(f"\n🔄 變化分析:")
        
        # 檢查 status_id 變化
        if updated_task.status_id != original_status:
            print(f"   ✅ Status ID 已更新: {original_status} -> {updated_task.status_id}")
        else:
            print(f"   ⚠️  Status ID 未變化: {original_status}")
        
        # 檢查 updated_at 變化
        if updated_task.updated_at != original_updated_at:
            print(f"   ✅ Updated At 已更新: {original_updated_at} -> {updated_task.updated_at}")
        else:
            print(f"   ⚠️  Updated At 未變化: {original_updated_at}")
        
        # 檢查 parameters 變化
        new_params = updated_task.parameters or {}
        
        print(f"\n📝 Parameters 詳細比較:")
        print(f"   原始 parameters 數量: {len(original_params)}")
        print(f"   更新後 parameters 數量: {len(new_params)}")
        
        # 檢查新增的 Kuka 相關參數
        kuka_keys = [
            "kuka_mission_status", "kuka_robot_id", "kuka_container_code",
            "kuka_current_position", "kuka_slot_code", "kuka_view_board_type",
            "kuka_message", "kuka_mission_data", "kuka_last_update"
        ]
        
        for key in kuka_keys:
            if key in new_params:
                print(f"   ✅ {key}: {new_params[key]}")
            else:
                print(f"   ❌ 缺少 {key}")
        
        # 檢查是否保留了原有參數
        for key, value in original_params.items():
            if key in new_params and new_params[key] == value:
                print(f"   ✅ 保留原有參數 {key}: {value}")
            elif key in new_params:
                print(f"   🔄 參數 {key} 已變更: {value} -> {new_params[key]}")
            else:
                print(f"   ❌ 遺失原有參數 {key}: {value}")
        
        # 總結
        if len(new_params) > len(original_params) and "kuka_mission_status" in new_params:
            print(f"\n✅ Parameters 更新成功！")
        else:
            print(f"\n❌ Parameters 更新可能有問題！")
            
    except requests.exceptions.ConnectionError:
        print("❌ 無法連接到 API 伺服器")
        print("請確認 web_api 伺服器是否正在運行:")
        print("ros2 run web_api api_server")
    except Exception as e:
        print(f"❌ 測試過程中發生錯誤: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    check_task_parameters_before_and_after()
