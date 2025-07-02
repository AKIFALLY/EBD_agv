#!/usr/bin/env python3
"""
測試 API 並立即檢查資料庫更新結果
"""
import sys
import os
sys.path.append('/app/db_proxy_ws/src')

import requests
import json
import time
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import Task
from sqlmodel import select

def test_api_and_check_db(mission_code="1357"):
    """測試 API 並檢查資料庫更新"""
    
    db_url = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'
    db_pool = ConnectionPoolManager(db_url)
    
    # 1. 檢查任務是否存在
    print(f"🔍 檢查任務 '{mission_code}' 是否存在...")
    try:
        with db_pool.get_session() as session:
            task = session.exec(
                select(Task).where(Task.mission_code == mission_code)
            ).first()
            
            if not task:
                print(f"❌ 找不到 mission_code 為 '{mission_code}' 的任務")
                print("請先執行: python tests/create_test_task.py")
                return
            
            print(f"✅ 找到任務: ID={task.id}, Name={task.name}")
            original_params = task.parameters or {}
            print(f"📝 API 調用前的 parameters: {json.dumps(original_params, ensure_ascii=False)}")
    
    except Exception as e:
        print(f"❌ 檢查任務時發生錯誤: {e}")
        return
    
    # 2. 調用 API
    print(f"\n🚀 調用 Kuka API...")
    url = "http://localhost:8000/interfaces/api/amr/missionStateCallback"
    
    test_data = {
        "missionCode": mission_code,
        "missionStatus": "MOVE_BEGIN",
        "robotId": f"robot_{int(time.time())}",  # 使用時間戳確保每次不同
        "containerCode": f"container_{int(time.time())}",
        "currentPosition": f"position_{int(time.time())}",
        "slotCode": "slot_A1",
        "viewBoardType": "test_board",
        "message": f"測試時間: {time.strftime('%Y-%m-%d %H:%M:%S')}",
        "missionData": {
            "test_timestamp": int(time.time()),
            "test_message": "parameters 更新測試"
        }
    }
    
    print(f"📤 發送資料: {json.dumps(test_data, indent=2, ensure_ascii=False)}")
    
    try:
        response = requests.post(url, json=test_data, headers={"Content-Type": "application/json"})
        
        print(f"\n📊 API 回應:")
        print(f"   狀態碼: {response.status_code}")
        print(f"   回應內容: {response.text}")
        
        if response.status_code != 200:
            print("❌ API 調用失敗")
            return
            
    except requests.exceptions.ConnectionError:
        print("❌ 無法連接到 API 伺服器")
        print("請確認 web_api 伺服器是否正在運行: ros2 run web_api api_server")
        return
    except Exception as e:
        print(f"❌ API 調用時發生錯誤: {e}")
        return
    
    # 3. 等待一下確保資料庫更新完成
    print(f"\n⏳ 等待資料庫更新...")
    time.sleep(1)
    
    # 4. 檢查資料庫更新結果
    print(f"🔍 檢查資料庫更新結果...")
    try:
        with db_pool.get_session() as session:
            updated_task = session.exec(
                select(Task).where(Task.mission_code == mission_code)
            ).first()
            
            print(f"\n📋 更新後的任務資訊:")
            print(f"   Task ID: {updated_task.id}")
            print(f"   Status ID: {updated_task.status_id}")
            print(f"   Updated At: {updated_task.updated_at}")
            
            new_params = updated_task.parameters or {}
            print(f"\n📝 更新後的 parameters:")
            print(json.dumps(new_params, indent=2, ensure_ascii=False))
            
            # 5. 比較更新前後的差異
            print(f"\n🔄 更新分析:")
            
            # 檢查是否有新的 Kuka 參數
            kuka_keys = [
                "kuka_mission_status", "kuka_robot_id", "kuka_container_code",
                "kuka_current_position", "kuka_slot_code", "kuka_view_board_type",
                "kuka_message", "kuka_mission_data", "kuka_last_update"
            ]
            
            found_kuka_params = 0
            for key in kuka_keys:
                if key in new_params:
                    print(f"   ✅ {key}: {new_params[key]}")
                    found_kuka_params += 1
                else:
                    print(f"   ❌ 缺少 {key}")
            
            # 總結
            if found_kuka_params > 0:
                print(f"\n✅ Parameters 更新成功！找到 {found_kuka_params}/{len(kuka_keys)} 個 Kuka 參數")
                
                # 檢查具體的測試資料是否正確
                if new_params.get("kuka_robot_id") == test_data["robotId"]:
                    print(f"✅ robotId 更新正確: {new_params.get('kuka_robot_id')}")
                else:
                    print(f"❌ robotId 更新錯誤: 期望 {test_data['robotId']}, 實際 {new_params.get('kuka_robot_id')}")
                    
                if new_params.get("kuka_mission_status") == test_data["missionStatus"]:
                    print(f"✅ missionStatus 更新正確: {new_params.get('kuka_mission_status')}")
                else:
                    print(f"❌ missionStatus 更新錯誤: 期望 {test_data['missionStatus']}, 實際 {new_params.get('kuka_mission_status')}")
                    
            else:
                print(f"\n❌ Parameters 更新失敗！沒有找到任何 Kuka 參數")
                
    except Exception as e:
        print(f"❌ 檢查資料庫時發生錯誤: {e}")

if __name__ == "__main__":
    import sys
    mission_code = sys.argv[1] if len(sys.argv) > 1 else "1357"
    test_api_and_check_db(mission_code)
