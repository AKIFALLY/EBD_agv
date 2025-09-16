#\!/usr/bin/env python3
"""
測試 Task name 欄位的 None 值處理修復
"""

import sys
sys.path.append('/app/db_proxy_ws/src')

from db_proxy.db_proxy.ros_converter import msg_to_model
from db_proxy.db_proxy.models.agvc_task import Task
from db_proxy_interfaces.msg import Task as TaskMsg

def test_empty_name_conversion():
    """測試空字串 name 的轉換處理"""
    print("測試 Task name 空字串轉換...")
    
    # 建立一個有空字串 name 的 TaskMsg
    task_msg = TaskMsg()
    task_msg.id = 150
    task_msg.work_id = 0
    task_msg.status_id = 3
    task_msg.room_id = 0
    task_msg.node_id = 0
    task_msg.name = ""  # 空字串，應該被轉換為預設值
    task_msg.description = ""
    task_msg.agv_id = 1
    task_msg.priority = 0
    task_msg.parameters = "null"
    task_msg.created_at = ""
    task_msg.updated_at = ""
    
    print(f"原始 TaskMsg.name: '{task_msg.name}'")
    
    try:
        # 轉換為 Task 模型
        task_model = msg_to_model(task_msg, Task)
        print(f"轉換後 Task.name: '{task_model.name}'")
        print("✅ 轉換成功！空字串已正確處理")
        return True
    except Exception as e:
        print(f"❌ 轉換失敗: {e}")
        return False

def test_normal_name_conversion():
    """測試正常 name 的轉換處理"""
    print("\n測試 Task name 正常值轉換...")
    
    task_msg = TaskMsg()
    task_msg.id = 151
    task_msg.name = "測試任務"  # 正常名稱
    task_msg.work_id = 1
    task_msg.status_id = 1
    task_msg.room_id = 1
    task_msg.node_id = 1
    task_msg.description = "測試描述"
    task_msg.agv_id = 1
    task_msg.priority = 1
    task_msg.parameters = "{}"
    task_msg.created_at = ""
    task_msg.updated_at = ""
    
    print(f"原始 TaskMsg.name: '{task_msg.name}'")
    
    try:
        task_model = msg_to_model(task_msg, Task)
        print(f"轉換後 Task.name: '{task_model.name}'")
        print("✅ 轉換成功！正常名稱保持不變")
        return True
    except Exception as e:
        print(f"❌ 轉換失敗: {e}")
        return False

if __name__ == "__main__":
    print("🔍 Task Name 修復測試")
    print("=" * 50)
    
    success1 = test_empty_name_conversion()
    success2 = test_normal_name_conversion()
    
    print("\n" + "=" * 50)
    if success1 and success2:
        print("🎉 所有測試通過！修復成功")
    else:
        print("❌ 部分測試失敗")
