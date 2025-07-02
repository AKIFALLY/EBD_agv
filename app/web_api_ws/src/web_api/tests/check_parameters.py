#!/usr/bin/env python3
"""
檢查任務 parameters 是否正確更新
"""
import sys
import os
sys.path.append('/app/db_proxy_ws/src')

import json
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import Task
from sqlmodel import select

def check_task_parameters(mission_code="1357"):
    """檢查指定任務的 parameters"""
    
    db_url = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'
    db_pool = ConnectionPoolManager(db_url)
    
    try:
        with db_pool.get_session() as session:
            task = session.exec(
                select(Task).where(Task.mission_code == mission_code)
            ).first()
            
            if not task:
                print(f"❌ 找不到 mission_code 為 '{mission_code}' 的任務")
                return
            
            print(f"📋 任務資訊:")
            print(f"   Task ID: {task.id}")
            print(f"   Mission Code: {task.mission_code}")
            print(f"   Name: {task.name}")
            print(f"   Status ID: {task.status_id}")
            print(f"   Updated At: {task.updated_at}")
            
            print(f"\n📝 Parameters 內容:")
            if task.parameters:
                print(json.dumps(task.parameters, indent=2, ensure_ascii=False))
                
                # 檢查是否包含 Kuka 相關參數
                kuka_keys = [
                    "kuka_mission_status", "kuka_robot_id", "kuka_container_code",
                    "kuka_current_position", "kuka_slot_code", "kuka_view_board_type",
                    "kuka_message", "kuka_mission_data", "kuka_last_update"
                ]
                
                print(f"\n🔍 Kuka 參數檢查:")
                has_kuka_data = False
                for key in kuka_keys:
                    if key in task.parameters:
                        print(f"   ✅ {key}: {task.parameters[key]}")
                        has_kuka_data = True
                    else:
                        print(f"   ❌ 缺少 {key}")
                
                if has_kuka_data:
                    print(f"\n✅ 任務包含 Kuka 狀態資訊")
                else:
                    print(f"\n⚠️  任務不包含 Kuka 狀態資訊")
            else:
                print("   (空的 parameters)")
                
    except Exception as e:
        print(f"❌ 檢查過程中發生錯誤: {e}")

if __name__ == "__main__":
    import sys
    mission_code = sys.argv[1] if len(sys.argv) > 1 else "1357"
    print(f"🔍 檢查 mission_code '{mission_code}' 的任務...")
    check_task_parameters(mission_code)
