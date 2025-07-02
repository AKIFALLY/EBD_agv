from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.crud.rosout_log_crud import rosout_log_crud
from db_proxy.crud.runtime_log_crud import runtime_log_crud
from db_proxy.models import RosoutLog, RuntimeLog
from rclpy.node import Node
from rcl_interfaces.msg import Log
from datetime import datetime, timezone


class AgvcLogger():
    def __init__(self, node: Node, pool_agvc: ConnectionPoolManager):
        self.pool_agvc = pool_agvc
        self.node = node

        # 訂閱 ROSOUT 的訊息
        node.create_subscription(Log, "/rosout", self.rosout_callback, 30)
        node.get_logger().info("✅ ROSOUT Logger 已啟動，開始訂閱 /rosout")

        # 訂閱 Running 的訊息
        node.create_subscription(
            Log, "/runtime_log", self.runtime_log_callback, 30)
        node.get_logger().info("✅ Runtime Logger 已啟動，開始訂閱 /runtime_log")

    def rosout_callback(self, msg: Log):
        """處理 ROSOUT 的訊息"""
        # 轉換 ROS2 時間為 datetime
        timestamp = datetime.fromtimestamp(
            msg.stamp.sec + msg.stamp.nanosec / 1e9,
            tz=timezone.utc
        )
        try:
            with self.pool_agvc.get_session() as session:
                rosout_log_crud.create(session, RosoutLog(
                    timestamp=timestamp, level=msg.level, name=msg.name, message=msg.msg, file=msg.file, function=msg.function, line=msg.line))
                # 將 ORM 物件轉換為 dict
        except Exception as e:
            #self.node.get_logger().error(f"❌ 儲存資料庫時發生錯誤：{e}")
            pass

    def runtime_log_callback(self, msg: Log):
        """處理 Runtime Logger 的訊息"""
        # 轉換 ROS2 時間為 datetime
        timestamp = datetime.fromtimestamp(
            msg.stamp.sec + msg.stamp.nanosec / 1e9,
            tz=timezone.utc
        )
        try:
            with self.pool_agvc.get_session() as session:
                runtime_log_crud.create(session, RuntimeLog(
                    timestamp=timestamp, level=msg.level, name=msg.name, message=msg.msg, file=msg.file, function=msg.function, line=msg.line))
        except Exception as e:
            self.node.get_logger().error(f"❌ 儲存資料庫時發生錯誤：{e}")
