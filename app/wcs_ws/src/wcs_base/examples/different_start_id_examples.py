"""
不同任務處理器設定不同初始 ID 的範例
展示如何為每個任務類型指定專屬的條件檢查起始點
"""

import config.config as CONFIG
from db_proxy.models import Task
from datetime import datetime, timezone
from wcs_base.base_task_handler import BaseTaskHandler
from wcs_base.task_condition_checker import TaskConditionChecker


class EmptyRackToBoxoutHandler(BaseTaskHandler):
    """
    空架到出口傳送箱任務處理器
    使用 ID 1 作為條件檢查起始點
    """

    def __init__(self, node):
        super().__init__(node)
        self.collected_data = {}
        self.condition_checker = TaskConditionChecker(
            db_manager=self.db_manager,
            logger=self.node.get_logger(),
            real_time_mode=True
        )

    def check_condition(self):
        """條件檢查 - 從 ID 1 開始"""
        try:
            # 空架到出口傳送箱任務從 ID 1 開始檢查
            success, collected_data = self.condition_checker.check_conditions_from_id(start_id=1)
            
            if success:
                self.collected_data = collected_data
                self.find_task = True
                return True
            return False
            
        except Exception as e:
            self.node.get_logger().error(f"❌ 空架到出口傳送箱條件檢查失敗: {e}")
            return False


class FullRackToStorageHandler(BaseTaskHandler):
    """
    滿架到儲存區任務處理器
    使用 ID 10 作為條件檢查起始點
    """

    def __init__(self, node):
        super().__init__(node)
        self.collected_data = {}
        self.condition_checker = TaskConditionChecker(
            db_manager=self.db_manager,
            logger=self.node.get_logger(),
            real_time_mode=True
        )

    def check_condition(self):
        """條件檢查 - 從 ID 10 開始"""
        try:
            # 滿架到儲存區任務從 ID 10 開始檢查
            success, collected_data = self.condition_checker.check_conditions_from_id(start_id=10)
            
            if success:
                self.collected_data = collected_data
                self.find_task = True
                return True
            return False
            
        except Exception as e:
            self.node.get_logger().error(f"❌ 滿架到儲存區條件檢查失敗: {e}")
            return False


class AGVMaintenanceHandler(BaseTaskHandler):
    """
    AGV 維護任務處理器
    使用 ID 20 作為條件檢查起始點
    """

    def __init__(self, node):
        super().__init__(node)
        self.collected_data = {}
        self.condition_checker = TaskConditionChecker(
            db_manager=self.db_manager,
            logger=self.node.get_logger(),
            real_time_mode=True
        )

    def check_condition(self):
        """條件檢查 - 從 ID 20 開始"""
        try:
            # AGV 維護任務從 ID 20 開始檢查
            success, collected_data = self.condition_checker.check_conditions_from_id(start_id=20)
            
            if success:
                self.collected_data = collected_data
                self.find_task = True
                return True
            return False
            
        except Exception as e:
            self.node.get_logger().error(f"❌ AGV 維護條件檢查失敗: {e}")
            return False


class ProductionLineHandler(BaseTaskHandler):
    """
    生產線任務處理器
    使用 ID 30 作為條件檢查起始點
    """

    def __init__(self, node):
        super().__init__(node)
        self.collected_data = {}
        self.condition_checker = TaskConditionChecker(
            db_manager=self.db_manager,
            logger=self.node.get_logger(),
            real_time_mode=True
        )

    def check_condition(self):
        """條件檢查 - 從 ID 30 開始"""
        try:
            # 生產線任務從 ID 30 開始檢查
            success, collected_data = self.condition_checker.check_conditions_from_id(start_id=30)
            
            if success:
                self.collected_data = collected_data
                self.find_task = True
                return True
            return False
            
        except Exception as e:
            self.node.get_logger().error(f"❌ 生產線條件檢查失敗: {e}")
            return False


class EmergencyResponseHandler(BaseTaskHandler):
    """
    緊急回應任務處理器
    使用 ID 100 作為條件檢查起始點
    """

    def __init__(self, node):
        super().__init__(node)
        self.collected_data = {}
        self.condition_checker = TaskConditionChecker(
            db_manager=self.db_manager,
            logger=self.node.get_logger(),
            real_time_mode=True
        )

    def check_condition(self):
        """條件檢查 - 從 ID 100 開始"""
        try:
            # 緊急回應任務從 ID 100 開始檢查
            success, collected_data = self.condition_checker.check_conditions_from_id(start_id=100)
            
            if success:
                self.collected_data = collected_data
                self.find_task = True
                return True
            return False
            
        except Exception as e:
            self.node.get_logger().error(f"❌ 緊急回應條件檢查失敗: {e}")
            return False


# 任務處理器與起始 ID 的對應表
TASK_HANDLER_START_IDS = {
    "EmptyRackToBoxoutHandler": 1,      # 空架到出口傳送箱
    "FullRackToStorageHandler": 10,     # 滿架到儲存區
    "AGVMaintenanceHandler": 20,        # AGV 維護
    "ProductionLineHandler": 30,        # 生產線任務
    "EmergencyResponseHandler": 100,    # 緊急回應
}


def print_start_id_mapping():
    """列印任務處理器與起始 ID 的對應關係"""
    print("📋 任務處理器與起始 ID 對應表:")
    print("=" * 60)
    
    for handler_name, start_id in TASK_HANDLER_START_IDS.items():
        print(f"   {handler_name:<30} → 起始 ID: {start_id}")
    
    print("=" * 60)


def demonstrate_usage():
    """示範不同任務處理器的使用方式"""
    print("\n🎯 使用方式示範:")
    print("=" * 60)
    
    print("\n1. 空架到出口傳送箱任務:")
    print("   success, data = self.condition_checker.check_conditions_from_id(start_id=1)")
    
    print("\n2. 滿架到儲存區任務:")
    print("   success, data = self.condition_checker.check_conditions_from_id(start_id=10)")
    
    print("\n3. AGV 維護任務:")
    print("   success, data = self.condition_checker.check_conditions_from_id(start_id=20)")
    
    print("\n4. 生產線任務:")
    print("   success, data = self.condition_checker.check_conditions_from_id(start_id=30)")
    
    print("\n5. 緊急回應任務:")
    print("   success, data = self.condition_checker.check_conditions_from_id(start_id=100)")


def show_database_structure():
    """展示資料庫中條件記錄的建議結構"""
    print("\n🗄️ 建議的 task_condition 表格結構:")
    print("=" * 60)
    
    conditions_structure = [
        {"id_range": "1-9", "purpose": "空架到出口傳送箱相關條件"},
        {"id_range": "10-19", "purpose": "滿架到儲存區相關條件"},
        {"id_range": "20-29", "purpose": "AGV 維護相關條件"},
        {"id_range": "30-39", "purpose": "生產線任務相關條件"},
        {"id_range": "40-49", "purpose": "庫存管理相關條件"},
        {"id_range": "50-59", "purpose": "品質檢查相關條件"},
        {"id_range": "60-69", "purpose": "設備監控相關條件"},
        {"id_range": "70-79", "purpose": "能源管理相關條件"},
        {"id_range": "80-89", "purpose": "安全檢查相關條件"},
        {"id_range": "90-99", "purpose": "系統維護相關條件"},
        {"id_range": "100+", "purpose": "緊急回應相關條件"},
    ]
    
    for item in conditions_structure:
        print(f"   ID {item['id_range']:<8} → {item['purpose']}")


def create_sample_conditions():
    """創建範例條件記錄的 SQL"""
    print("\n📝 範例條件記錄 SQL:")
    print("=" * 60)
    
    sample_sqls = [
        {
            "id": 1,
            "description": "空架到出口傳送箱 - 檢查空料架區狀態",
            "sql": "SELECT 'True' as result, '2,3,4' as next_id, location_id as location FROM racks WHERE status = 'empty' LIMIT 1"
        },
        {
            "id": 10,
            "description": "滿架到儲存區 - 檢查滿料架狀態",
            "sql": "SELECT 'True' as result, '11,12,13' as next_id, storage_area as location FROM racks WHERE status = 'full' LIMIT 1"
        },
        {
            "id": 20,
            "description": "AGV 維護 - 檢查 AGV 狀態",
            "sql": "SELECT 'True' as result, '21,22' as next_id, agv_id FROM agv_status WHERE maintenance_required = true LIMIT 1"
        },
        {
            "id": 100,
            "description": "緊急回應 - 檢查緊急事件",
            "sql": "SELECT 'True' as result, 'end' as next_id, event_type FROM emergency_events WHERE status = 'active' LIMIT 1"
        }
    ]
    
    for sample in sample_sqls:
        print(f"\nINSERT INTO task_condition (id, description, conditions) VALUES (")
        print(f"  {sample['id']},")
        print(f"  '{sample['description']}',")
        print(f"  '{sample['sql']}'")
        print(f");")


if __name__ == "__main__":
    print("🚀 不同任務處理器的初始 ID 設定示範")
    
    # 顯示對應表
    print_start_id_mapping()
    
    # 示範使用方式
    demonstrate_usage()
    
    # 展示資料庫結構建議
    show_database_structure()
    
    # 創建範例條件記錄
    create_sample_conditions()
    
    print("\n🎉 示範完成！")
    print("\n💡 重點總結:")
    print("   1. 每個任務處理器在 check_condition() 方法中設定自己的起始 ID")
    print("   2. 不同任務類型使用不同的 ID 範圍，避免衝突")
    print("   3. 建議按功能分組規劃 ID 範圍")
    print("   4. 緊急任務使用較大的 ID 號碼以示重要性")
