#!/usr/bin/env python3
"""
整合測試腳本：驗證 flow_wcs 和 db_proxy 統一模型的功能
Integration test: Verify unified model functionality
"""

import sys
import os
from datetime import datetime

# 添加路徑以導入模組
sys.path.append('/app/db_proxy_ws/src/db_proxy')

from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker

# 導入統一模型
from db_proxy.models.agvc_location import Location, LocationStatus
from db_proxy.models.rack import Rack
from db_proxy.models.agvc_task import Task, Work, FlowLog
from db_proxy.models.agvc_rcs import AGV
from db_proxy.models.carrier import Carrier

# 資料庫連線設定
DB_URL = os.getenv(
    'DATABASE_URL',
    'postgresql://agvc:password@192.168.100.254:5432/agvc'
)

class IntegrationTest:
    """整合測試類別"""
    
    def __init__(self, db_url: str = DB_URL):
        """初始化測試"""
        self.engine = create_engine(db_url)
        self.SessionLocal = sessionmaker(bind=self.engine)
        self.test_results = []
    
    def test_location_integration(self):
        """測試 Location 整合功能"""
        print("\n測試 Location 整合功能...")
        
        with self.SessionLocal() as session:
            try:
                # 創建測試 Location
                location = Location(
                    name="Test_Location_001",
                    type="room_inlet",
                    room_id=1,
                    x=100.5,
                    y=200.5,
                    location_status_id=LocationStatus.UNOCCUPIED
                )
                session.add(location)
                session.flush()
                
                # 測試 flow_wcs 相容性方法
                assert location.has_rack == False, "has_rack 應該為 False"
                
                # 設定 rack_id
                location.rack_id = 1
                assert location.has_rack == True, "設定 rack_id 後 has_rack 應該為 True"
                
                # 測試 to_flow_wcs_dict 方法
                flow_dict = location.to_flow_wcs_dict()
                assert flow_dict['name'] == "Test_Location_001"
                assert flow_dict['has_rack'] == True
                assert flow_dict['x'] == 100.5
                assert flow_dict['y'] == 200.5
                
                print("✅ Location 整合測試通過")
                self.test_results.append(("Location", "PASSED"))
                
                # 清理測試資料
                session.rollback()
                
            except Exception as e:
                print(f"❌ Location 整合測試失敗: {e}")
                self.test_results.append(("Location", f"FAILED: {e}"))
                session.rollback()
    
    def test_rack_integration(self):
        """測試 Rack 整合功能"""
        print("\n測試 Rack 整合功能...")
        
        with self.SessionLocal() as session:
            try:
                # 創建測試 Rack
                rack = Rack(
                    name="Test_Rack_001",
                    location_id=1,
                    direction=0,
                    side_a_status="empty",
                    side_b_status="has_carrier"
                )
                session.add(rack)
                session.flush()
                
                # 測試側面狀態管理
                assert rack.get_side_status('a') == "empty"
                assert rack.get_side_status('b') == "has_carrier"
                
                # 測試設定側面狀態
                rack.set_side_status('a', 'completed')
                assert rack.side_a_status == "completed"
                
                # 測試旋轉功能
                rack.rotate(180.0)
                assert rack.rotation_angle == 180.0
                assert rack.side_a_status == "has_carrier", "180度旋轉應該交換兩面狀態"
                assert rack.side_b_status == "completed", "180度旋轉應該交換兩面狀態"
                
                # 測試 to_flow_wcs_dict 方法
                flow_dict = rack.to_flow_wcs_dict()
                assert flow_dict['rotation_angle'] == 180.0
                assert flow_dict['side_a_status'] == "has_carrier"
                
                print("✅ Rack 整合測試通過")
                self.test_results.append(("Rack", "PASSED"))
                
                # 清理測試資料
                session.rollback()
                
            except Exception as e:
                print(f"❌ Rack 整合測試失敗: {e}")
                self.test_results.append(("Rack", f"FAILED: {e}"))
                session.rollback()
    
    def test_task_integration(self):
        """測試 Task 整合功能"""
        print("\n測試 Task 整合功能...")
        
        with self.SessionLocal() as session:
            try:
                # 創建測試 Work
                work = Work(
                    name="Test_Work",
                    work_code="W001",
                    work_type="transport",
                    flow_definition={"flow": "test"}
                )
                session.add(work)
                session.flush()
                
                # 創建測試 Task
                task = Task(
                    task_id="TASK_W001_20240101120000",
                    name="Test_Task",
                    type="transport",
                    work_id=work.id,
                    priority=100,
                    parameters={"test": "data"}
                )
                session.add(task)
                session.flush()
                
                # 測試 flow_wcs 相容性方法
                assert task.metadata == {"test": "data"}
                assert task.task_metadata == {"test": "data"}
                
                # 測試任務 ID 生成
                new_task_id = task.generate_task_id("W002")
                assert new_task_id.startswith("TASK_W002_")
                
                # 測試 to_flow_wcs_dict 方法
                flow_dict = task.to_flow_wcs_dict()
                assert flow_dict['task_id'] == "TASK_W001_20240101120000"
                assert flow_dict['type'] == "transport"
                assert flow_dict['priority'] == 100
                assert flow_dict['metadata'] == {"test": "data"}
                
                print("✅ Task 整合測試通過")
                self.test_results.append(("Task", "PASSED"))
                
                # 清理測試資料
                session.rollback()
                
            except Exception as e:
                print(f"❌ Task 整合測試失敗: {e}")
                self.test_results.append(("Task", f"FAILED: {e}"))
                session.rollback()
    
    def test_agv_integration(self):
        """測試 AGV 整合功能"""
        print("\n測試 AGV 整合功能...")
        
        with self.SessionLocal() as session:
            try:
                # 創建測試 AGV
                agv = AGV(
                    agv_id="AGV_001",
                    name="Test_AGV",
                    model="Cargo",
                    x=100.0,
                    y=200.0,
                    heading=90.0,
                    battery=85.5,
                    current_location="Location_A",
                    current_task_id="TASK_001",
                    status_id=1  # idle
                )
                session.add(agv)
                session.flush()
                
                # 測試 flow_wcs 相容性方法
                assert agv.type == "cargo"
                assert agv.theta == 90.0
                assert agv.battery_level == 85.5
                assert agv.status == "idle"
                
                # 測試 setter
                agv.theta = 180.0
                assert agv.heading == 180.0
                
                agv.battery_level = 90.0
                assert agv.battery == 90.0
                
                # 測試 to_flow_wcs_dict 方法
                flow_dict = agv.to_flow_wcs_dict()
                assert flow_dict['agv_id'] == "AGV_001"
                assert flow_dict['type'] == "cargo"
                assert flow_dict['theta'] == 180.0
                assert flow_dict['battery_level'] == 90.0
                assert flow_dict['current_location'] == "Location_A"
                
                print("✅ AGV 整合測試通過")
                self.test_results.append(("AGV", "PASSED"))
                
                # 清理測試資料
                session.rollback()
                
            except Exception as e:
                print(f"❌ AGV 整合測試失敗: {e}")
                self.test_results.append(("AGV", f"FAILED: {e}"))
                session.rollback()
    
    def test_flow_log_integration(self):
        """測試 FlowLog 整合功能"""
        print("\n測試 FlowLog 整合功能...")
        
        with self.SessionLocal() as session:
            try:
                # 創建測試 Work
                work = Work(
                    name="Test_Work_For_Log",
                    work_code="W_LOG_001"
                )
                session.add(work)
                session.flush()
                
                # 創建測試 FlowLog
                flow_log = FlowLog(
                    flow_id="FLOW_001",
                    flow_name="Test Flow",
                    work_id=work.id,
                    section="init",
                    step_id="step_1",
                    function="query_locations",
                    params={"type": "room_inlet"},
                    result={"count": 5},
                    status="success",
                    duration=0.123
                )
                session.add(flow_log)
                session.flush()
                
                # 驗證資料
                assert flow_log.flow_id == "FLOW_001"
                assert flow_log.status == "success"
                assert flow_log.duration == 0.123
                assert flow_log.params == {"type": "room_inlet"}
                
                print("✅ FlowLog 整合測試通過")
                self.test_results.append(("FlowLog", "PASSED"))
                
                # 清理測試資料
                session.rollback()
                
            except Exception as e:
                print(f"❌ FlowLog 整合測試失敗: {e}")
                self.test_results.append(("FlowLog", f"FAILED: {e}"))
                session.rollback()
    
    def test_carrier_integration(self):
        """測試 Carrier 整合功能"""
        print("\n測試 Carrier 整合功能...")
        
        with self.SessionLocal() as session:
            try:
                # 創建測試 Carrier
                carrier = Carrier(
                    room_id=1,
                    rack_id=1,
                    rack_side='a',
                    status_id=1
                )
                session.add(carrier)
                session.flush()
                
                # 驗證資料
                assert carrier.rack_id == 1
                assert carrier.rack_side == 'a'
                
                # 更新側面
                carrier.rack_side = 'b'
                assert carrier.rack_side == 'b'
                
                print("✅ Carrier 整合測試通過")
                self.test_results.append(("Carrier", "PASSED"))
                
                # 清理測試資料
                session.rollback()
                
            except Exception as e:
                print(f"❌ Carrier 整合測試失敗: {e}")
                self.test_results.append(("Carrier", f"FAILED: {e}"))
                session.rollback()
    
    def run_all_tests(self):
        """執行所有測試"""
        print("=" * 50)
        print("開始整合測試...")
        print("=" * 50)
        
        # 執行各項測試
        self.test_location_integration()
        self.test_rack_integration()
        self.test_task_integration()
        self.test_agv_integration()
        self.test_flow_log_integration()
        self.test_carrier_integration()
        
        # 顯示測試結果
        print("\n" + "=" * 50)
        print("測試結果總結:")
        print("=" * 50)
        
        passed = 0
        failed = 0
        
        for test_name, result in self.test_results:
            if result == "PASSED":
                print(f"✅ {test_name}: PASSED")
                passed += 1
            else:
                print(f"❌ {test_name}: {result}")
                failed += 1
        
        print("-" * 50)
        print(f"總計: {passed} 通過, {failed} 失敗")
        
        return failed == 0


def main():
    """主程式"""
    import argparse
    
    parser = argparse.ArgumentParser(description='測試 flow_wcs 和 db_proxy 統一模型整合')
    parser.add_argument('--db-url', default=DB_URL, help='資料庫連線 URL')
    
    args = parser.parse_args()
    
    tester = IntegrationTest(args.db_url)
    success = tester.run_all_tests()
    
    # 返回適當的退出碼
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()