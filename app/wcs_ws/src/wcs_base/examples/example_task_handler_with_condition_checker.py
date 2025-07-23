"""
範例：使用 TaskConditionChecker 的任務處理器

展示如何在其他任務處理器中使用通用的條件檢查功能
"""

import config.config as CONFIG
from db_proxy.models import Task
from datetime import datetime, timezone
from wcs_base.base_task_handler import BaseTaskHandler
from wcs_base.task_condition_checker import TaskConditionChecker


class ExampleTaskHandler(BaseTaskHandler):
    """
    範例任務處理器
    展示如何使用 TaskConditionChecker 進行條件檢查
    """

    def __init__(self, node):
        super().__init__(node)

        # 任務相關數據
        self.task_work = None
        self.task_node_id = None
        self.task_room_id = None
        self.collected_data = {}
        
        # 初始化條件檢查器
        self.condition_checker = TaskConditionChecker(self.db_manager, self.node.get_logger())
        
        # 可選：設定最大迭代次數
        self.condition_checker.set_max_iterations(50)

    def check_condition(self):
        """
        使用 TaskConditionChecker 進行條件檢查
        """
        try:
            # 方法 1: 從預設起始 ID (1) 開始檢查
            success, collected_data = self.condition_checker.check_conditions_from_id()
            
            if success:
                self.collected_data = collected_data
                self._extract_task_data_from_collected()
                
                self.node.get_logger().info(f"✅ 條件檢查成功")
                self.find_task = True
                return True
            else:
                self.node.get_logger().info(f"📋 條件檢查未滿足")
                return False
                
        except Exception as e:
            self.node.get_logger().error(f"❌ 條件檢查失敗: {e}")
            return False

    def check_condition_from_specific_id(self, start_id: int):
        """
        從特定 ID 開始進行條件檢查
        
        Args:
            start_id: 起始條件 ID
        """
        try:
            # 方法 2: 從指定 ID 開始檢查
            success, collected_data = self.condition_checker.check_conditions_from_id(start_id=start_id)
            
            if success:
                self.collected_data = collected_data
                self._extract_task_data_from_collected()
                
                self.node.get_logger().info(f"✅ 從 ID {start_id} 開始的條件檢查成功")
                self.find_task = True
                return True
            else:
                self.node.get_logger().info(f"📋 從 ID {start_id} 開始的條件檢查未滿足")
                return False
                
        except Exception as e:
            self.node.get_logger().error(f"❌ 從 ID {start_id} 開始的條件檢查失敗: {e}")
            return False

    def check_single_condition(self, condition_id: int):
        """
        檢查單一條件是否滿足
        
        Args:
            condition_id: 條件 ID
            
        Returns:
            bool: 條件是否滿足
        """
        try:
            # 方法 3: 檢查單一條件
            return self.condition_checker.check_single_id_condition(condition_id)
            
        except Exception as e:
            self.node.get_logger().error(f"❌ 檢查單一條件 {condition_id} 失敗: {e}")
            return False

    def get_condition_data(self, condition_id: int):
        """
        取得特定條件的資料
        
        Args:
            condition_id: 條件 ID
            
        Returns:
            dict: 條件資料
        """
        try:
            # 方法 4: 直接取得條件資料
            results = self.condition_checker.get_task_condition_results(condition_id)
            if results:
                parse_result = self.condition_checker.parse_condition_results(condition_id, results)
                if parse_result:
                    success, data_list = parse_result
                    return {
                        "success": success,
                        "data": data_list,
                        "raw_results": results
                    }
            return None
            
        except Exception as e:
            self.node.get_logger().error(f"❌ 取得條件 {condition_id} 資料失敗: {e}")
            return None

    def _extract_task_data_from_collected(self):
        """
        從收集的資料中提取任務所需的資訊
        根據實際需求調整此方法
        """
        try:
            # 範例：提取位置資訊
            if "location" in self.collected_data:
                location_value = self.collected_data["location"]
                if isinstance(location_value, str) and location_value.isdigit():
                    self.task_node_id = int(location_value)
                    self.node.get_logger().info(f"📝 提取 task_node_id: {self.task_node_id}")
            
            # 範例：提取房間資訊
            if "room_id" in self.collected_data:
                self.task_room_id = self.collected_data["room_id"]
                self.node.get_logger().info(f"📝 提取 task_room_id: {self.task_room_id}")
            
            # 記錄所有收集到的資料
            self.node.get_logger().info(f"📊 收集到的所有資料: {self.collected_data}")
            
        except Exception as e:
            self.node.get_logger().error(f"❌ 提取任務資料失敗: {e}")

    def insert_task(self):
        """
        插入任務
        根據實際需求實作此方法
        """
        try:
            # 這裡實作具體的任務插入邏輯
            # 使用 self.collected_data 中的資料
            
            self.node.get_logger().info(f"🔄 準備插入任務，使用資料: {self.collected_data}")
            
            # 範例任務插入邏輯
            if self.task_node_id and self.task_room_id:
                # 實際的任務創建邏輯
                self.node.get_logger().info(f"✅ 任務插入成功")
                self.task_inserted = True
                return True
            else:
                self.node.get_logger().warning(f"⚠️ 缺少必要的任務資料")
                return False
                
        except Exception as e:
            self.node.get_logger().error(f"❌ 插入任務失敗: {e}")
            return False

    def check_insert_done(self):
        """
        檢查任務插入是否成功
        """
        try:
            # 實作任務插入檢查邏輯
            if self.task_inserted:
                self.node.get_logger().info(f"✅ 任務插入完成")
                self.find_task = False
                self.task_inserted = False
                return True
            return False
            
        except Exception as e:
            self.node.get_logger().error(f"❌ 檢查任務插入狀態失敗: {e}")
            return False


# 使用範例
def example_usage():
    """
    使用範例
    """
    # 假設有一個 node 實例
    # handler = ExampleTaskHandler(node)
    
    # 範例 1: 標準條件檢查流程
    # if handler.check_condition():
    #     print("條件滿足，準備插入任務")
    #     if handler.insert_task():
    #         print("任務插入成功")
    
    # 範例 2: 從特定 ID 開始檢查
    # if handler.check_condition_from_specific_id(start_id=5):
    #     print("從 ID 5 開始的條件檢查成功")
    
    # 範例 3: 檢查單一條件
    # if handler.check_single_condition(condition_id=10):
    #     print("條件 10 滿足")
    
    # 範例 4: 取得條件資料
    # data = handler.get_condition_data(condition_id=1)
    # if data:
    #     print(f"條件 1 的資料: {data}")
    
    pass


if __name__ == "__main__":
    print("這是一個範例檔案，展示如何使用 TaskConditionChecker")
    example_usage()
