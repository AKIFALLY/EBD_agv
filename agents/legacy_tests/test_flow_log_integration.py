#!/usr/bin/env python3
"""
測試 FlowLog 整合功能 - 確認 flow_wcs 和 db_proxy 的統一模型正確運作
"""

import sys
sys.path.append('/app/db_proxy_ws/src/db_proxy')
sys.path.append('/app/flow_wcs_ws/src/flow_wcs')

from flow_wcs.database_unified import db_manager
import json
from datetime import datetime

def test_flow_log():
    """測試 FlowLog 記錄功能"""
    print("=== 測試 FlowLog 整合 ===\n")
    
    # 測試參數
    flow_id = f"FLOW_TEST_{datetime.now().strftime('%Y%m%d%H%M%S')}"
    flow_name = "Test Flow for Integration"
    work_id = "TEST_WORK_001"
    
    # 測試日誌記錄
    print("1. 測試 Flow 執行日誌記錄:")
    try:
        db_manager.log_flow_execution(
            flow_id=flow_id,
            flow_name=flow_name,
            work_id=work_id,
            section="initialization",
            step_id="step_1",
            function="check_rack_status",
            params={"rack_id": 1, "side": "a"},
            result={"status": "empty"},
            status="success",
            error_message=None,
            duration=0.125
        )
        print(f"   ✅ 成功記錄日誌 - flow_id: {flow_id}")
    except Exception as e:
        print(f"   ❌ 錯誤: {e}")
    
    # 記錄另一個步驟
    print("\n2. 測試記錄多個步驟:")
    try:
        db_manager.log_flow_execution(
            flow_id=flow_id,
            flow_name=flow_name,
            work_id=work_id,
            section="execution",
            step_id="step_2",
            function="create_task",
            params={"type": "pickup", "location_id": 1},
            result={"task_id": "TASK_TEST_001"},
            status="success",
            error_message=None,
            duration=0.235
        )
        print("   ✅ 成功記錄第二個步驟")
    except Exception as e:
        print(f"   ❌ 錯誤: {e}")
    
    # 記錄錯誤情況
    print("\n3. 測試記錄錯誤狀態:")
    try:
        db_manager.log_flow_execution(
            flow_id=flow_id,
            flow_name=flow_name,
            work_id=work_id,
            section="validation",
            step_id="step_3",
            function="validate_agv",
            params={"agv_id": "AGV_999"},
            result=None,
            status="failed",
            error_message="AGV not found",
            duration=0.050
        )
        print("   ✅ 成功記錄錯誤狀態")
    except Exception as e:
        print(f"   ❌ 錯誤: {e}")
    
    # 查詢記錄的日誌
    print("\n4. 查詢記錄的日誌:")
    try:
        with db_manager.get_session() as session:
            from db_proxy.models.agvc_task import FlowLog
            logs = session.query(FlowLog).filter(FlowLog.flow_id == flow_id).all()
            
            print(f"   ✅ 找到 {len(logs)} 條日誌記錄")
            
            for log in logs:
                print(f"   - Section: {log.section}, Step: {log.step_id}, "
                      f"Function: {log.function}, Status: {log.status}")
                if log.error_message:
                    print(f"     錯誤: {log.error_message}")
                if log.duration:
                    print(f"     執行時間: {log.duration:.3f}s")
    except Exception as e:
        print(f"   ❌ 錯誤: {e}")
    
    print("\n=== FlowLog 整合測試完成 ===")

if __name__ == "__main__":
    test_flow_log()