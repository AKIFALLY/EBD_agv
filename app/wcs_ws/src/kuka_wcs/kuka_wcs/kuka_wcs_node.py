"""
KUKA WCS (Warehouse Control System) 主節點
整合 KukaFleetAdapter 和任務判斷引擎，提供完整的 AGV 車隊管理功能
"""

import rclpy
from rclpy.node import Node
import sys
import os

# 添加 kuka_fleet_adapter 到 Python 路徑
sys.path.append('/app/kuka_fleet_ws/src/kuka_fleet_adapter')
sys.path.append('/app/db_proxy_ws/src/db_proxy')

try:
    from kuka_fleet_adapter.kuka_fleet_adapter import KukaFleetAdapter
except ImportError as e:
    print(f"Warning: Could not import KukaFleetAdapter: {e}")
    KukaFleetAdapter = None

try:
    from db_proxy.agvc_database_client import AGVCDatabaseClient
    from db_proxy_interfaces.msg import Tasks, Task
except ImportError as e:
    print(f"Warning: Could not import AGVCDatabaseClient or Tasks: {e}")
    AGVCDatabaseClient = None
    Tasks = None
    Task = None

from .task_decision_engine import TaskDecisionEngine


class KukaWCSNode(Node):
    """KUKA WCS 主節點"""
    
    def __init__(self):
        super().__init__('kuka_wcs_node')

        self.get_logger().info("🚀 KUKA WCS Node 正在啟動...")

        # 初始化任務決策引擎
        self.decision_engine = TaskDecisionEngine(self.get_logger())

        # 初始化 KUKA Fleet Adapter
        self.kuka_adapter = None
        if KukaFleetAdapter:
            try:
                self.kuka_adapter = KukaFleetAdapter(self)
                self.get_logger().info("✅ KUKA Fleet Adapter 初始化成功")
            except Exception as e:
                self.get_logger().error(f"❌ KUKA Fleet Adapter 初始化失敗: {e}")
        else:
            self.get_logger().warn("⚠️ KUKA Fleet Adapter 不可用")

        # 初始化資料庫客戶端
        self.db_client = None
        if AGVCDatabaseClient:
            try:
                self.db_client = AGVCDatabaseClient(self)
                self.get_logger().info("✅ Database Client 初始化成功")
            except Exception as e:
                self.get_logger().error(f"❌ Database Client 初始化失敗: {e}")
        else:
            self.get_logger().warn("⚠️ Database Client 不可用")

        # 初始化任務相關變數
        self.task_list = []
        self.current_tasks = []

        # 訂閱 tasks topic (參照 mission_select_state)
        if Tasks:
            self.tasks_subscription = self.create_subscription(
                Tasks,
                '/agvc/tasks',
                self.tasks_callback,
                10
            )
            self.get_logger().info("✅ 已訂閱 /agvc/tasks topic")
        else:
            self.get_logger().warn("⚠️ Tasks 消息類型不可用")

        # 創建定時器，每1秒執行一次任務處理 (參照 mission_select_state)
        self.task_timer = self.create_timer(1.0, self.process_tasks)

        self.get_logger().info("✅ KUKA WCS Node 啟動完成")

    def tasks_callback(self, msg):
        """任務回調函數 - 參照 mission_select_state.py"""
        tasks = msg.datas

        self.get_logger().info(f"📋 收到 {len(tasks)} 個任務")

        # 更新任務列表
        self.task_list = tasks

        # 篩選待處理的任務 (status_id == 0: 未執行)
        pending_tasks = [
            t for t in tasks
            if t.status_id == 0  # 未執行的任務
        ]

        # 篩選執行中的任務 (status_id == 1: 已選擇, status_id == 2: 執行中)
        running_tasks = [
            t for t in tasks
            if t.status_id == 1 or t.status_id == 2
        ]

        self.get_logger().info(f"📊 待處理任務: {len(pending_tasks)}, 執行中任務: {len(running_tasks)}")

        # 更新決策引擎的任務資料
        if pending_tasks:
            self.decision_engine.update_pending_tasks(pending_tasks)

    def process_tasks(self):
        """定時處理任務 - 每1秒執行一次"""
        try:
            # 這裡可以添加定期的任務處理邏輯
            # 例如：檢查任務狀態、進行任務分配決策等

            if self.task_list:
                # 獲取系統狀態
                status = self.decision_engine.get_system_status()

                # 每30秒輸出一次狀態摘要 (參照 mission_select_state 的 count 機制)
                if hasattr(self, 'process_count'):
                    self.process_count += 1
                else:
                    self.process_count = 1

                if self.process_count >= 30:  # 30秒
                    self.process_count = 0
                    self.get_logger().info(
                        f"📈 系統狀態 - 總任務: {len(self.task_list)}, "
                        f"待處理: {status.get('pending_tasks', 0)}, "
                        f"執行中: {status.get('active_tasks', 0)}"
                    )

        except Exception as e:
            self.get_logger().error(f"❌ 處理任務時發生錯誤: {e}")


def main(args=None):
    """主函數"""
    rclpy.init(args=args)
    
    try:
        node = KukaWCSNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"節點運行時發生錯誤: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
