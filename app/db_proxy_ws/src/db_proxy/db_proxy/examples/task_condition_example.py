"""
TaskCondition 使用範例
展示如何在實際應用中使用 TaskCondition 資料表
"""

import json
from datetime import datetime, timezone
from sqlmodel import Session
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import TaskCondition
from db_proxy.crud.task_condition_crud import task_condition_crud


class TaskConditionManager:
    """任務條件管理器"""
    
    def __init__(self, db_url: str):
        """
        初始化任務條件管理器
        
        Args:
            db_url: 資料庫連接字串
        """
        self.pool = ConnectionPoolManager(db_url)
    
    def add_agv_condition(self, agv_id: str, status: str, location: str) -> TaskCondition:
        """
        新增 AGV 狀態條件
        
        Args:
            agv_id: AGV ID
            status: AGV 狀態
            location: AGV 位置
            
        Returns:
            TaskCondition: 建立的條件記錄
        """
        with self.pool.get_session() as session:
            conditions = f"AGV_ID = '{agv_id}' AND STATUS = '{status}' AND LOCATION = '{location}'"
            results = {
                "agv_id": agv_id,
                "status": status,
                "location": location,
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "type": "agv_status"
            }
            
            return task_condition_crud.create_condition(session, conditions, results)
    
    def add_task_condition(self, task_type: str, priority: int, requirements: dict) -> TaskCondition:
        """
        新增任務條件
        
        Args:
            task_type: 任務類型
            priority: 優先級
            requirements: 任務需求
            
        Returns:
            TaskCondition: 建立的條件記錄
        """
        with self.pool.get_session() as session:
            conditions = f"TASK_TYPE = '{task_type}' AND PRIORITY = {priority}"
            results = {
                "task_type": task_type,
                "priority": priority,
                "requirements": requirements,
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "type": "task_requirement"
            }
            
            return task_condition_crud.create_condition(session, conditions, results, f"AGV {agv_id} 狀態條件")
    
    def check_carrier_condition(self, carrier_id: str, status: str, cargo_info: dict) -> TaskCondition:
        """
        檢查載具條件
        
        Args:
            carrier_id: 載具 ID
            status: 載具狀態
            cargo_info: 貨物資訊
            
        Returns:
            TaskCondition: 建立的條件記錄
        """
        with self.pool.get_session() as session:
            conditions = f"CARRIER_ID = '{carrier_id}' AND STATUS = '{status}'"
            results = {
                "carrier_id": carrier_id,
                "status": status,
                "cargo_info": cargo_info,
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "type": "carrier_status"
            }
            
            return task_condition_crud.create_condition(session, conditions, results, f"任務類型 {task_type} 條件")
    
    def search_conditions_by_type(self, condition_type: str) -> list:
        """
        根據條件類型搜尋條件
        
        Args:
            condition_type: 條件類型
            
        Returns:
            list: 符合條件的記錄列表
        """
        with self.pool.get_session() as session:
            conditions = task_condition_crud.search_conditions(session, condition_type.upper())
            return [
                {
                    "id": condition.id,
                    "conditions": condition.conditions,
                    "results": condition.results
                }
                for condition in conditions
            ]
    
    def update_condition_result(self, condition_id: int, new_results: dict) -> bool:
        """
        更新條件結果
        
        Args:
            condition_id: 條件 ID
            new_results: 新的結果資料
            
        Returns:
            bool: 是否更新成功
        """
        with self.pool.get_session() as session:
            # 加入更新時間戳
            new_results["updated_at"] = datetime.now(timezone.utc).isoformat()
            
            updated_condition = task_condition_crud.update_results(
                session, condition_id, new_results
            )
            return updated_condition is not None
    
    def get_all_conditions_summary(self) -> dict:
        """
        取得所有條件的摘要資訊
        
        Returns:
            dict: 條件摘要資訊
        """
        with self.pool.get_session() as session:
            all_conditions = task_condition_crud.get_all(session)
            conditions_with_results = task_condition_crud.get_all_with_results(session)
            
            # 統計不同類型的條件
            type_counts = {}
            for condition in conditions_with_results:
                if condition.results and "type" in condition.results:
                    condition_type = condition.results["type"]
                    type_counts[condition_type] = type_counts.get(condition_type, 0) + 1
            
            return {
                "total_conditions": len(all_conditions),
                "conditions_with_results": len(conditions_with_results),
                "type_distribution": type_counts,
                "summary_timestamp": datetime.now(timezone.utc).isoformat()
            }


def main():
    """主函式 - 展示使用範例"""
    print("🚀 TaskCondition 使用範例")
    print("=" * 50)
    
    # 資料庫連接字串（請根據實際環境修改）
    db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
    
    try:
        # 建立任務條件管理器
        manager = TaskConditionManager(db_url)
        
        print("📋 1. 新增 AGV 狀態條件")
        agv_condition = manager.add_agv_condition("AGV001", "IDLE", "STATION_A")
        print(f"   ✅ 建立條件 ID: {agv_condition.id}")
        
        print("\n📋 2. 新增任務條件")
        task_condition = manager.add_task_condition(
            "TRANSPORT", 
            8, 
            {"from": "STATION_A", "to": "STATION_B", "cargo_type": "PARTS"}
        )
        print(f"   ✅ 建立條件 ID: {task_condition.id}")
        
        print("\n📋 3. 檢查載具條件")
        carrier_condition = manager.check_carrier_condition(
            "CARRIER001", 
            "LOADED", 
            {"weight": 150.5, "type": "ELECTRONICS"}
        )
        print(f"   ✅ 建立條件 ID: {carrier_condition.id}")
        
        print("\n📋 4. 搜尋 AGV 相關條件")
        agv_conditions = manager.search_conditions_by_type("AGV")
        print(f"   📊 找到 {len(agv_conditions)} 個 AGV 相關條件")
        
        print("\n📋 5. 更新條件結果")
        updated = manager.update_condition_result(
            agv_condition.id, 
            {
                "status": "BUSY",
                "current_task": "TRANSPORT_001",
                "estimated_completion": "2024-01-01T12:30:00Z"
            }
        )
        print(f"   ✅ 條件更新{'成功' if updated else '失敗'}")
        
        print("\n📋 6. 取得條件摘要")
        summary = manager.get_all_conditions_summary()
        print(f"   📊 總條件數: {summary['total_conditions']}")
        print(f"   📊 有結果的條件數: {summary['conditions_with_results']}")
        print(f"   📊 類型分布: {json.dumps(summary['type_distribution'], indent=2, ensure_ascii=False)}")
        
        print("\n" + "=" * 50)
        print("✅ 範例執行完成！")
        
    except Exception as e:
        print(f"❌ 執行失敗: {e}")


if __name__ == "__main__":
    main()
