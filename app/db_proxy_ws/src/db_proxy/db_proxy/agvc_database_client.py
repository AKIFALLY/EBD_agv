import json
from db_proxy.crud.task_crud import task_crud  # ✅ 正確：匯入那個實例
import rclpy
from rclpy.node import Node
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.sql.db_install import initialize_default_data
# 只需匯入 SQLModel 與 model
from db_proxy.models import Task, Work, TaskStatus, ProcessSettings, Product, TrafficZone
from rclpy.service import Service
from db_proxy_interfaces.srv import SqlQuery
from db_proxy_interfaces.srv import UpdateTask
from db_proxy_interfaces.srv import UpdateRack
from db_proxy_interfaces.srv import UpdateCarrier
from db_proxy_interfaces.srv import GenericQuery  # 確保有 import
from db_proxy_interfaces.msg import Tasks
from db_proxy_interfaces.msg import Task as TaskMsg
from db_proxy_interfaces.msg import Rack as RackMsg
from db_proxy_interfaces.msg import Carrier as CarrierMsg
from sqlmodel import SQLModel, select, text


class AGVCDatabaseClient:
    def __init__(self, node: Node):
        self.node = node
        self.task_client = self.node.create_client(
            UpdateTask, '/agvc/update_task')
        self.rack_client = self.node.create_client(
            UpdateRack, '/agvc/update_rack')
        self.carrier_client = self.node.create_client(
            UpdateCarrier, '/agvc/update_carrier')
        self.node.get_logger().info("🚀 AGVC Database Client 已啟動")
        self.generic_query_client = self.node.create_client(GenericQuery, '/agvc/generic_query')

    def update_rack(self, rack: RackMsg, timeout_sec=2.0):
        """同步更新 Rack（會阻塞直到有回應或逾時）"""
        if not self.rack_client.wait_for_service(timeout_sec=timeout_sec):
            self.node.get_logger().warn('❌ Service /agvc/update_rack 不可用')
            return None

        request = UpdateRack.Request(rack=rack)
        future = self.rack_client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=timeout_sec)
        if future.done():
            return future.result()
        else:
            self.node.get_logger().warn('⚠️ update_rack 同步請求逾時')
            return None

    def update_task(self, task: TaskMsg, timeout_sec=2.0):
        """同步更新 Task（會阻塞直到有回應或逾時）"""
        if not self.task_client.wait_for_service(timeout_sec=timeout_sec):
            self.node.get_logger().warn('❌ Service /agvc/update_task 不可用')
            return None

        request = UpdateTask.Request(task=task)
        future = self.task_client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=timeout_sec)
        if future.done():
            return future.result()
        else:
            self.node.get_logger().warn('⚠️ update_task 同步請求逾時')
            return None

    def update_carrier(self, carrier: CarrierMsg, timeout_sec=2.0):
        """同步更新 Carrier（會阻塞直到有回應或逾時）"""
        if not self.carrier_client.wait_for_service(timeout_sec=timeout_sec):
            self.node.get_logger().warn('❌ Service /agvc/update_carrier 不可用')
            return None

        request = UpdateCarrier.Request(carrier=carrier)
        future = self.carrier_client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=timeout_sec)
        if future.done():
            return future.result()
        else:
            self.node.get_logger().warn('⚠️ update_carrier 同步請求逾時')
            return None

    from db_proxy_interfaces.srv import GenericQuery  # 確保有 import

    def generic_query(self, table_name: str, columns: list[str], data: list[str], condition: str, mode: str, timeout_sec=3.0):
        """同步呼叫 generic SQL 查詢/寫入服務"""
        if not self.generic_query_client.wait_for_service(timeout_sec=timeout_sec):
            self.node.get_logger().warn('❌ Service /agvc/generic_query 不可用')
            return None

        request = GenericQuery.Request()
        request.table_name = table_name
        request.columns = columns
        request.data = data
        request.condition = condition
        request.mode = mode

        future = self.generic_query_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)

        if future.done():
            return future.result()
        else:
            self.node.get_logger().warn('⚠️ generic_query 同步請求逾時')
            return None




    def async_update_task(self, task: TaskMsg, callback):
        """非同步更新 Task（需提供 callback）"""
        if not self.task_client.service_is_ready():
            self.node.get_logger().warn('❌ Service /agvc/update_task 尚未就緒')
            return None

        request = UpdateTask.Request(task=task)
        future = self.task_client.call_async(request)

        def _internal_callback(_future):
            try:
                if _future.done():
                    result = _future.result()
                    callback(result)
                else:
                    self.node.get_logger().warn("⚠️ async_update_task 未完成")
                    callback(None)
            except Exception as e:
                self.node.get_logger().error(
                    f"❌ async_update_task callback 發生錯誤: {e}")
                callback(None)

        future.add_done_callback(_internal_callback)
        return future

    def async_update_carrier(self, carrier: CarrierMsg, callback):
        """非同步更新 Carrier（需提供 callback）"""
        if not self.carrier_client.service_is_ready():
            self.node.get_logger().warn('❌ Service /agvc/update_carrier 尚未就緒')
            return None

        request = UpdateCarrier.Request(carrier=carrier)
        future = self.carrier_client.call_async(request)

        def _internal_callback(_future):
            try:
                if _future.done():
                    result = _future.result()
                    callback(result)
                else:
                    self.node.get_logger().warn("⚠️ async_update_carrier 未完成")
                    callback(None)
            except Exception as e:
                self.node.get_logger().error(
                    f"❌ async_update_carrier callback 發生錯誤: {e}")
                callback(None)

        future.add_done_callback(_internal_callback)
        return future


    def async_generic_query(self, table_name: str, columns: list[str], data: list[str], condition: str, mode: str, callback=None):
        """非同步呼叫 GenericQuery 服務，支援 callback"""
        if not self.generic_query_client.service_is_ready():
            self.node.get_logger().warn('❌ Service /agvc/generic_query 尚未就緒')
            if callback:
                callback(None)
            return None

        request = GenericQuery.Request()
        request.table_name = table_name
        request.columns = columns
        request.data = data
        request.condition = condition
        request.mode = mode

        future = self.generic_query_client.call_async(request)

        def _internal_callback(_future):
            try:
                if _future.done():
                    result = _future.result()
                    callback(result)
                else:
                    self.node.get_logger().warn("⚠️ async_generic_query 未完成")
                    callback(None)
            except Exception as e:
                self.node.get_logger().error(f"❌ async_generic_query 發生錯誤: {e}")
                callback(None)

        future.add_done_callback(_internal_callback)
        return future




    def destroy(self):
        if self.task_client:
            self.node.destroy_client(self.task_client)
            self.task_client = None


def test_task(node, client: AGVCDatabaseClient):

    # 模擬 TaskMsg
    test_task = TaskMsg()
    test_task.id = 4
    test_task.name = "test_task"
    test_task.agv_id = 1
    test_task.priority = 1
    test_task.work_id = 1
    test_task.room_id = 1
    test_task.status_id = 1
    test_task.description = "這是一個測試任務"
    test_task.parameters = "{}"

    print(f"TaskMsg type: {type(test_task)}")
    print(
        f"TaskMsg class path: {test_task.__class__.__module__}.{test_task.__class__.__name__}")

    # ✅ 測試同步呼叫
    response = client.update_task(test_task)
    if response:
        node.get_logger().info(f"✅ 同步呼叫結果: {response.success}")
        node.get_logger().info(f"✅ 同步呼叫訊息: {response.message}")
        node.get_logger().info(f"✅ 同步呼叫任務: {response.task}")
    else:
        node.get_logger().warn("❌ 同步呼叫失敗或逾時")

    # ✅ 測試非同步呼叫

    def async_callback(result):
        if result:
            node.get_logger().info(f"✅ 非同步呼叫結果: {result.success}")
            node.get_logger().info(f"✅ 非同步呼叫訊息: {result.message}")
            node.get_logger().info(f"✅ 非同步呼叫訊息: {result.task}")
        else:
            node.get_logger().warn("❌ 非同步呼叫失敗")
#
    client.async_update_task(test_task, async_callback)


def test_rack(node, client: AGVCDatabaseClient):
    # 模擬 RackMsg
    test_rack_msg = RackMsg()
    test_rack_msg.id = 1
    test_rack_msg.location_id = 1
    test_rack_msg.product_id = 1
    test_rack_msg.status_id = 1

    # ✅ 測試同步呼叫
    response = client.update_rack(test_rack_msg)
    if response:
        node.get_logger().info(f"✅ 同步呼叫結果: {response.success}")
        node.get_logger().info(f"✅ 同步呼叫訊息: {response.message}")
        node.get_logger().info(f"✅ 同步呼叫任務: {response.rack}")
    else:
        node.get_logger().warn("❌ 同步呼叫失敗或逾時")


def main(args=None):
    rclpy.init(args=args)
    node = Node("agvc_database_client")
    client = AGVCDatabaseClient(node)

    test_task(node, client)
    # test_rack(node, client)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
