#!/usr/bin/env python3
"""
測試 Kuka API 的簡單腳本
"""
import requests
import json

def test_kuka_mission_state_callback():
    """測試 Kuka missionStateCallback API"""
    
    # API 端點
    url = "http://localhost:8000/interfaces/api/amr/missionStateCallback"
    
    # 測試資料 (根據 API 規格的範例)
    test_data = {
        "missionCode": "mission202309250005",
        "viewBoardType": "",
        "slotCode": "",
        "robotId": "14",
        "containerCode": "1000002",
        "currentPosition": "M001-A001-31",
        "missionStatus": "MOVE_BEGIN",
        "message": "",
        "missionData": {}
    }
    
    headers = {
        "Content-Type": "application/json"
    }
    
    try:
        print("發送測試請求到 Kuka API...")
        print(f"URL: {url}")
        print(f"Data: {json.dumps(test_data, indent=2, ensure_ascii=False)}")
        
        response = requests.post(url, json=test_data, headers=headers)
        
        print(f"\n回應狀態碼: {response.status_code}")
        print(f"回應內容: {response.text}")
        
        if response.status_code == 200:
            print("✅ API 測試成功!")
            response_data = response.json()
            print(f"任務 ID: {response_data.get('task_id')}")
            print(f"任務代碼: {response_data.get('mission_code')}")
            print(f"任務狀態: {response_data.get('mission_status')}")
        else:
            print("❌ API 測試失敗!")
            
    except requests.exceptions.ConnectionError:
        print("❌ 無法連接到 API 伺服器，請確認伺服器是否正在運行")
    except Exception as e:
        print(f"❌ 測試過程中發生錯誤: {e}")

def test_different_mission_statuses():
    """測試不同的任務狀態"""
    
    url = "http://localhost:8000/interfaces/api/amr/missionStateCallback"
    
    # 測試不同的任務狀態
    test_statuses = [
        "MOVE_BEGIN",
        "ARRIVED", 
        "UP_CONTAINER",
        "DOWN_CONTAINER",
        "ROLLER_RECEIVE",
        "ROLLER_SEND",
        "PICKER_RECEIVE",
        "PICKER_SEND",
        "FORK_UP",
        "FORK_DOWN",
        "COMPLETED",
        "CANCELED"
    ]
    
    base_data = {
        "missionCode": "test_mission_001",
        "robotId": "robot_01",
        "containerCode": "container_001",
        "currentPosition": "position_001",
        "message": "測試訊息",
        "missionData": {"test": "data"}
    }
    
    for status in test_statuses:
        test_data = base_data.copy()
        test_data["missionStatus"] = status
        
        try:
            print(f"\n測試狀態: {status}")
            response = requests.post(url, json=test_data)
            print(f"狀態碼: {response.status_code}")
            
            if response.status_code == 200:
                print("✅ 成功")
            else:
                print(f"❌ 失敗: {response.text}")
                
        except Exception as e:
            print(f"❌ 錯誤: {e}")

if __name__ == "__main__":
    print("=== Kuka API 測試 ===")
    
    # 基本測試
    test_kuka_mission_state_callback()
    
    # 詢問是否要測試所有狀態
    user_input = input("\n是否要測試所有任務狀態? (y/n): ")
    if user_input.lower() == 'y':
        test_different_mission_statuses()
    
    print("\n測試完成!")
