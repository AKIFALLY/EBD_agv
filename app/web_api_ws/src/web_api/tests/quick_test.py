#!/usr/bin/env python3
"""
快速測試 Kuka API 修正後的功能
"""
import requests
import json

def quick_test():
    """快速測試 API"""
    
    url = "http://localhost:8000/interfaces/api/amr/missionStateCallback"
    
    # 使用您之前測試的資料格式
    test_data = {
        "missionCode": "1357",  # 假設這是您創建的測試任務的 mission_code
        "missionStatus": "MOVE_BEGIN",  # 使用有效的狀態
        "robotId": "robot_01",
        "containerCode": "container_001",
        "currentPosition": "position_001",
        "message": "測試訊息",
        "missionData": {"test": "data"}
    }
    
    headers = {
        "Content-Type": "application/json"
    }
    
    try:
        print("🧪 快速測試 Kuka API...")
        print(f"URL: {url}")
        print(f"測試資料: {json.dumps(test_data, indent=2, ensure_ascii=False)}")
        
        response = requests.post(url, json=test_data, headers=headers)
        
        print(f"\n📊 回應結果:")
        print(f"狀態碼: {response.status_code}")
        print(f"回應內容: {response.text}")
        
        if response.status_code == 200:
            print("\n✅ API 測試成功！修正有效！")
            response_data = response.json()
            print(f"任務 ID: {response_data.get('task_id')}")
            print(f"任務代碼: {response_data.get('mission_code')}")
            print(f"任務狀態: {response_data.get('mission_status')}")
        elif response.status_code == 404:
            print(f"\n⚠️  任務不存在，請先創建 mission_code 為 '{test_data['missionCode']}' 的測試任務")
            print("執行以下命令創建測試任務:")
            print("python tests/create_test_task.py")
        else:
            print(f"\n❌ API 測試失敗，狀態碼: {response.status_code}")
            
    except requests.exceptions.ConnectionError:
        print("❌ 無法連接到 API 伺服器")
        print("請確認 web_api 伺服器是否正在運行:")
        print("ros2 run web_api api_server")
    except Exception as e:
        print(f"❌ 測試過程中發生錯誤: {e}")

if __name__ == "__main__":
    quick_test()
