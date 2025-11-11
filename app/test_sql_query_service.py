#!/usr/bin/env python3
"""
測試 SQL Query Service 連接和查詢

測試項目：
1. 檢查 /agvc/sql_query service 是否可用
2. 查詢 task 資料表
3. 驗證資料格式轉換
"""

import rclpy
from rclpy.node import Node
from db_proxy_interfaces.srv import SqlQuery
import json
import time


class TestSqlQueryClient(Node):
    def __init__(self):
        super().__init__('test_sql_query_client')

        # 建立 service client
        self.sql_query_client = self.create_client(SqlQuery, '/agvc/sql_query')

        self.get_logger().info("🔍 測試 SQL Query Service")
        self.get_logger().info("=" * 60)

    def test_service_availability(self):
        """測試 1: 檢查 service 是否可用"""
        self.get_logger().info("\n【測試 1】檢查 /agvc/sql_query service 可用性")

        # 等待 service（10 秒超時）
        self.get_logger().info("⏳ 等待 service 可用（10 秒超時）...")
        if self.sql_query_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info("✅ Service 可用！")
            return True
        else:
            self.get_logger().error("❌ Service 不可用！")
            return False

    def test_simple_query(self):
        """測試 2: 執行簡單查詢"""
        self.get_logger().info("\n【測試 2】執行簡單查詢")

        request = SqlQuery.Request()
        request.query_string = "SELECT 1 as test"

        self.get_logger().info(f"📤 查詢 SQL: {request.query_string}")

        future = self.sql_query_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.done():
            response = future.result()
            if response.success:
                self.get_logger().info(f"✅ 查詢成功！")
                self.get_logger().info(f"📊 結果: {response.json_result}")
                return True
            else:
                self.get_logger().error(f"❌ 查詢失敗: {response.message}")
                return False
        else:
            self.get_logger().error("❌ 查詢超時！")
            return False

    def test_task_query(self, agv_id=None):
        """測試 3: 查詢 task 資料表"""
        self.get_logger().info("\n【測試 3】查詢 task 資料表")

        request = SqlQuery.Request()

        if agv_id is not None:
            request.query_string = (
                f"SELECT id, work_id, status_id, room_id, node_id, "
                f"name, description, agv_id, priority, parameters, "
                f"created_at, updated_at "
                f"FROM task "
                f"WHERE agv_id = {agv_id} AND status_id IN (1, 2, 3) "
                f"ORDER BY priority DESC, created_at ASC"
            )
            self.get_logger().info(f"📤 查詢 AGV ID={agv_id} 的任務")
        else:
            request.query_string = (
                f"SELECT id, work_id, status_id, room_id, node_id, "
                f"name, description, agv_id, priority, parameters, "
                f"created_at, updated_at "
                f"FROM task "
                f"WHERE status_id IN (1, 2, 3) "
                f"ORDER BY priority DESC, created_at ASC LIMIT 5"
            )
            self.get_logger().info(f"📤 查詢前 5 筆進行中的任務")

        self.get_logger().info(f"SQL: {request.query_string}")

        future = self.sql_query_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.done():
            response = future.result()
            if response.success:
                self.get_logger().info(f"✅ 查詢成功！")

                # 解析結果
                result = json.loads(response.json_result)
                self.get_logger().info(f"📊 查詢到 {len(result)} 筆任務")

                # 顯示詳細資料
                for i, task_data in enumerate(result, 1):
                    self.get_logger().info(f"\n--- 任務 {i} ---")
                    self.get_logger().info(f"  ID: {task_data.get('id')}")
                    self.get_logger().info(f"  Name: {task_data.get('name')}")
                    self.get_logger().info(f"  Work ID: {task_data.get('work_id')}")
                    self.get_logger().info(f"  Status ID: {task_data.get('status_id')}")
                    self.get_logger().info(f"  AGV ID: {task_data.get('agv_id')}")
                    self.get_logger().info(f"  Priority: {task_data.get('priority')}")
                    self.get_logger().info(f"  Node ID: {task_data.get('node_id')}")
                    self.get_logger().info(f"  Parameters type: {type(task_data.get('parameters'))}")
                    self.get_logger().info(f"  Parameters: {task_data.get('parameters')}")
                    self.get_logger().info(f"  Created at: {task_data.get('created_at')}")
                    self.get_logger().info(f"  Updated at: {task_data.get('updated_at')}")

                return True, result
            else:
                self.get_logger().error(f"❌ 查詢失敗: {response.message}")
                return False, None
        else:
            self.get_logger().error("❌ 查詢超時！")
            return False, None

    def test_data_type_conversion(self, task_data):
        """測試 4: 驗證資料類型轉換"""
        self.get_logger().info("\n【測試 4】驗證資料類型轉換")

        from db_proxy_interfaces.msg import Task as TaskMsg

        try:
            task_msg = TaskMsg()

            # 數值欄位
            task_msg.id = int(task_data.get('id', 0))
            task_msg.work_id = int(task_data.get('work_id', 0))
            task_msg.status_id = int(task_data.get('status_id', 0))
            task_msg.room_id = int(task_data.get('room_id', 0))
            task_msg.node_id = int(task_data.get('node_id', 0))
            task_msg.agv_id = int(task_data.get('agv_id', 0))

            # priority 是 uint8
            priority = task_data.get('priority', 0)
            task_msg.priority = max(0, min(255, int(priority)))

            # 字串欄位
            task_msg.name = str(task_data.get('name', ''))
            task_msg.description = str(task_data.get('description', ''))

            # parameters 轉換
            parameters = task_data.get('parameters')
            if parameters is None:
                task_msg.parameters = ''
            elif isinstance(parameters, str):
                task_msg.parameters = parameters
            elif isinstance(parameters, dict):
                task_msg.parameters = json.dumps(parameters)
            else:
                task_msg.parameters = str(parameters)

            # 時間戳轉換
            created_at = task_data.get('created_at', '')
            updated_at = task_data.get('updated_at', '')
            task_msg.created_at = str(created_at) if created_at else ''
            task_msg.updated_at = str(updated_at) if updated_at else ''

            self.get_logger().info("✅ 資料類型轉換成功！")
            self.get_logger().info(f"   TaskMsg.id: {task_msg.id}")
            self.get_logger().info(f"   TaskMsg.name: {task_msg.name}")
            self.get_logger().info(f"   TaskMsg.priority: {task_msg.priority}")
            self.get_logger().info(f"   TaskMsg.parameters: {task_msg.parameters[:50]}...")

            return True
        except Exception as e:
            self.get_logger().error(f"❌ 資料類型轉換失敗: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return False


def main():
    rclpy.init()

    test_node = TestSqlQueryClient()

    try:
        # 測試 1: Service 可用性
        if not test_node.test_service_availability():
            test_node.get_logger().error("\n❌ Service 不可用，測試中止")
            test_node.get_logger().error("請確認以下事項：")
            test_node.get_logger().error("  1. AGVC 容器是否正常運行")
            test_node.get_logger().error("  2. agvc_database_node 是否已啟動")
            test_node.get_logger().error("  3. Zenoh Router 是否正常運行")
            return

        # 測試 2: 簡單查詢
        if not test_node.test_simple_query():
            test_node.get_logger().error("\n❌ 簡單查詢失敗，測試中止")
            return

        # 測試 3: 查詢 task 資料表（全部）
        success, tasks = test_node.test_task_query(agv_id=None)
        if not success:
            test_node.get_logger().warn("\n⚠️ 查詢 task 失敗或沒有任務資料")

        # 測試 4: 查詢特定 AGV 的任務
        test_node.get_logger().info("\n" + "=" * 60)
        agv_id = input("請輸入要查詢的 AGV ID（留空跳過）: ").strip()
        if agv_id:
            success, tasks = test_node.test_task_query(agv_id=int(agv_id))

        # 測試 5: 資料類型轉換（如果有資料）
        if tasks and len(tasks) > 0:
            test_node.test_data_type_conversion(tasks[0])

        test_node.get_logger().info("\n" + "=" * 60)
        test_node.get_logger().info("✅ 所有測試完成！")

    except KeyboardInterrupt:
        test_node.get_logger().info("\n🛑 測試中斷")
    except Exception as e:
        test_node.get_logger().error(f"\n❌ 測試異常: {e}")
        import traceback
        test_node.get_logger().error(traceback.format_exc())
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
