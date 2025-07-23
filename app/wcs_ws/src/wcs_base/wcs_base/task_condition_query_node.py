"""
任務條件查詢節點
提供定時執行和手動執行任務條件查詢的 ROS 2 節點
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import json
from datetime import datetime, timezone
from typing import Dict, Any

from wcs_base.database_manager import DatabaseManager
from wcs_base.task_condition_query_service import TaskConditionQueryService


class TaskConditionQueryNode(Node):
    """任務條件查詢節點"""
    
    def __init__(self):
        super().__init__('task_condition_query_node')
        self.get_logger().info("🚀 任務條件查詢節點正在啟動...")
        
        # 宣告參數
        self.declare_parameter('db_url_agvc', 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc')
        self.declare_parameter('auto_execution_interval', 1.0)  # 預設 1 秒鐘執行一次
        self.declare_parameter('enable_auto_execution', True)     # 是否啟用自動執行
        
        # 取得參數值
        db_url = self.get_parameter('db_url_agvc').value
        self.auto_interval = self.get_parameter('auto_execution_interval').value
        self.auto_enabled = self.get_parameter('enable_auto_execution').value
        
        # 初始化資料庫管理器
        try:
            self.db_manager = DatabaseManager(self.get_logger(), db_url)
            self.get_logger().info("✅ 資料庫管理器初始化成功")
        except Exception as e:
            self.get_logger().error(f"❌ 資料庫管理器初始化失敗: {e}")
            raise
        
        # 初始化查詢服務
        try:
            self.query_service = TaskConditionQueryService(self.db_manager, self.get_logger())
            self.get_logger().info("✅ 任務條件查詢服務初始化成功")
        except Exception as e:
            self.get_logger().error(f"❌ 任務條件查詢服務初始化失敗: {e}")
            raise
        
        # 建立服務
        self.manual_trigger_service = self.create_service(
            Trigger,
            'task_condition_query/manual_execute',
            self.manual_execute_callback
        )
        
        # 建立發布者
        self.result_publisher = self.create_publisher(
            String,
            'task_condition_query/results',
            10
        )
        
        # 建立狀態發布者
        self.status_publisher = self.create_publisher(
            String,
            'task_condition_query/status',
            10
        )
        
        # 執行狀態追蹤
        self.is_executing = False
        self.last_execution_time = None
        self.last_execution_result = None
        
        # 建立定時器
        if self.auto_enabled and self.auto_interval > 0:
            self.auto_timer = self.create_timer(
                self.auto_interval,
                self.auto_execute_callback
            )
            self.get_logger().info(f"✅ 自動執行定時器已啟動，間隔: {self.auto_interval} 秒")
        else:
            self.auto_timer = None
            self.get_logger().info("📋 自動執行已停用，僅支援手動觸發")
        
        # 建立狀態發布定時器（每 30 秒發布一次狀態）
        self.status_timer = self.create_timer(30.0, self.publish_status)
        
        self.get_logger().info("🎉 任務條件查詢節點啟動完成")
    
    def manual_execute_callback(self, request, response):
        """
        手動執行服務回調函式
        
        Args:
            request: 服務請求
            response: 服務回應
            
        Returns:
            response: 執行結果
        """
        self.get_logger().info("📞 收到手動執行請求")
        
        if self.is_executing:
            response.success = False
            response.message = "任務條件查詢正在執行中，請稍後再試"
            self.get_logger().warning("⚠️ 任務正在執行中，拒絕新的執行請求")
            return response
        
        try:
            # 執行查詢
            result = self.execute_query_process()
            
            # 設定回應
            response.success = result.get("successful", 0) > 0 or result.get("failed", 0) == 0
            response.message = self.format_execution_summary(result)
            
            self.get_logger().info(f"✅ 手動執行完成: {response.message}")
            
        except Exception as e:
            response.success = False
            response.message = f"執行失敗: {str(e)}"
            self.get_logger().error(f"❌ 手動執行失敗: {e}")
        
        return response
    
    def auto_execute_callback(self):
        """自動執行定時器回調函式"""
        if self.is_executing:
            self.get_logger().warning("⚠️ 上次自動執行尚未完成，跳過本次執行")
            return
        
        self.get_logger().info("⏰ 開始自動執行任務條件查詢")
        
        try:
            result = self.execute_query_process()
            self.get_logger().info(f"✅ 自動執行完成: {self.format_execution_summary(result)}")
        except Exception as e:
            self.get_logger().error(f"❌ 自動執行失敗: {e}")
    
    def execute_query_process(self) -> Dict[str, Any]:
        """
        執行查詢處理流程
        
        Returns:
            Dict: 執行結果
        """
        self.is_executing = True
        execution_start = datetime.now(timezone.utc)
        
        try:
            # 執行所有條件查詢
            result = self.query_service.process_all_conditions()
            
            # 更新執行狀態
            self.last_execution_time = execution_start
            self.last_execution_result = result
            
            # 發布結果
            self.publish_execution_result(result)
            
            return result
            
        finally:
            self.is_executing = False
    
    def publish_execution_result(self, result: Dict[str, Any]):
        """
        發布執行結果
        
        Args:
            result: 執行結果
        """
        try:
            result_msg = String()
            result_msg.data = json.dumps(result, ensure_ascii=False, indent=2)
            self.result_publisher.publish(result_msg)
            
            self.get_logger().debug("📤 執行結果已發布到 topic")
            
        except Exception as e:
            self.get_logger().error(f"❌ 發布執行結果失敗: {e}")
    
    def publish_status(self):
        """發布節點狀態"""
        try:
            status = {
                "node_name": self.get_name(),
                "is_executing": self.is_executing,
                "auto_execution_enabled": self.auto_enabled,
                "auto_execution_interval": self.auto_interval,
                "last_execution_time": self.last_execution_time.isoformat() if self.last_execution_time else None,
                "last_execution_result": self.last_execution_result,
                "current_time": datetime.now(timezone.utc).isoformat()
            }
            
            status_msg = String()
            status_msg.data = json.dumps(status, ensure_ascii=False, indent=2)
            self.status_publisher.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f"❌ 發布狀態失敗: {e}")
    
    def format_execution_summary(self, result: Dict[str, Any]) -> str:
        """
        格式化執行摘要
        
        Args:
            result: 執行結果
            
        Returns:
            str: 格式化的摘要字串
        """
        if "error" in result:
            return f"執行失敗: {result['error']}"
        
        total = result.get("total_conditions", 0)
        successful = result.get("successful", 0)
        failed = result.get("failed", 0)
        duration = result.get("duration_seconds", 0)
        
        return f"處理 {total} 個條件，成功 {successful} 個，失敗 {failed} 個，耗時 {duration:.2f} 秒"


def main(args=None):
    """主函式"""
    rclpy.init(args=args)
    
    try:
        node = TaskConditionQueryNode()
        
        # 執行節點
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("🛑 收到中斷信號，正在關閉節點...")
    except Exception as e:
        print(f"❌ 節點執行失敗: {e}")
    finally:
        # 清理資源
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
