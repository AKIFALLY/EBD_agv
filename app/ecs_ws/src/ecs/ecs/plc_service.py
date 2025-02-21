import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.service import Service
from interfaces.srv import (
    ForceOn,
    ForceOff,
    ReadData,
    WriteData,
    ReadContinuousData,
    WriteContinuousData,
)
from keyence_plc.keyence_plc_pool import KeyencePlcPool  # 連線池,管理實際PLC連線
from keyence_plc.keyence_plc_command import KeyencePlcCommand


class PlcService(Node):
    def __init__(self):
        super().__init__("plc_service")

        self.declare_parameter("plc_ip", "127.0.0.1")
        para_ip = self.get_parameter("plc_ip").get_parameter_value().string_value
        self.get_logger().info(f"Hello {para_ip}!")

        self.ip = para_ip
        self.port = 8501
        self.pool = KeyencePlcPool(self.ip, self.port)

        # ROS2 服務可重覆進入設定 (設為可重覆且多執行緒模式時, 非同步執行加快反應速度)
        self.callback_group = ReentrantCallbackGroup()

        # 註冊服務 強制 on/off 不可重覆進入 ,其他讀寫功能可重覆進入
        self.create_service(ForceOn, "force_on", self.force_on_callback)
        self.create_service(ForceOff, "force_off", self.force_off_callback)
        self.create_service(ReadData, "read_data", self.read_data_callback,
            qos_profile=rclpy.qos.QoSProfile(
                depth=100, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
            ),
            callback_group=self.callback_group,)
        self.create_service(WriteData, "write_data", self.write_data_callback,
            qos_profile=rclpy.qos.QoSProfile(
                depth=100, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
            ),
            callback_group=self.callback_group,)
        self.create_service(
            ReadContinuousData,
            "read_continuous_data",
            self.read_continuous_data_callback,
            qos_profile=rclpy.qos.QoSProfile(
                depth=100, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
            ),
            callback_group=self.callback_group,
        )
        self.create_service(
            WriteContinuousData,
            "write_continuous_data",
            self.write_continuous_data_callback,
            qos_profile=rclpy.qos.QoSProfile(
                depth=100, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
            ),
            callback_group=self.callback_group,
        )

        rclpy.get_default_context().on_shutdown(self.shutdown_callback)

    def shutdown_callback(self):
        self.get_logger().info("關閉 PLC 連線...")
        try:
            self.pool.close_connection()  # 中斷 PLC 連線
            self.get_logger().info("PLC 中斷連線")
        except Exception as e:
            self.get_logger().info(f"PLC 中斷連線 例外 {e}")
        finally:
            self.get_logger().info("PLC disconnect()")

    def force_on_callback(self, request, response):
        self.get_logger().info(f"PLC 強制置位 request: {request}")
        command = KeyencePlcCommand.force_on(request.device_type, request.address)
        response.success = False
        response.message = ""
        try:
            self.pool.execute(command)
        except Exception as e:
            self.get_logger().info(f"PLC 強制置位失敗 {e}")
            response.message = f"PLC 強制置位失敗 {e}"
            return response
        response.success = True
        self.get_logger().info("PLC 強制置位完成")
        return response

    def force_off_callback(self, request, response):
        self.get_logger().info(f"PLC 強制復位 request: {request}")
        command = KeyencePlcCommand.force_off(request.device_type, request.address)
        response.success = False
        response.message = ""
        try:
            self.pool.execute(command)
        except Exception as e:
            self.get_logger().info(f"PLC 強制復位失敗 {e}")
            response.message = f"PLC 強制復位失敗 {e}"
            return response
        response.success = True
        self.get_logger().info("PLC 強制復位完成")
        return response

    def read_data_callback(self, request, response):
        self.get_logger().info(f"PLC 讀取數據 request: {request}")
        command = KeyencePlcCommand.read_data(request.device_type, request.address)
        response.success = False
        response.value = ""
        response.message = ""
        try:
            response.value = self.pool.execute(command)
        except Exception as e:
            self.get_logger().info(f"PLC 讀取數據失敗 {e}")
            response.message = f"PLC 讀取數據失敗 {e}"
            return response
        response.success = True
        self.get_logger().info("PLC 讀取數據完成")
        return response

    def write_data_callback(self, request, response):
        self.get_logger().info(f"PLC 寫入數據 request: {request}")
        command = KeyencePlcCommand.write_data(
            request.device_type, request.address, request.value
        )
        response.success = False
        response.message = ""
        try:
            self.pool.execute(command)
        except Exception as e:
            self.get_logger().info(f"PLC 寫入數據失敗 {e}")
            response.message = f"PLC 寫入數據失敗 {e}"
            return response
        response.success = True
        self.get_logger().info("PLC 寫入數據完成")
        return response

    def read_continuous_data_callback(self, request, response):
        #self.get_logger().info(f"PLC 連續讀取數據 request: {request}")
        command = KeyencePlcCommand.read_continuous_data(
            request.device_type, request.start_address, request.count
        )
        response.success = False
        response.values = []
        response.message = ""
        try:
            values = self.pool.execute(command)
            response.values = values.split(" ")
        except Exception as e:
            self.get_logger().info(f"PLC 連續讀取數據失敗 {e} ,連線池數量:{len(self.pool.connections)}")
            response.message = f"PLC 連續讀取數據失敗 {e}"
            return response
        response.success = True
        #self.get_logger().info("PLC 連續讀取數據完成")
        return response

    def write_continuous_data_callback(self, request, response):
        self.get_logger().info(f"PLC 連續寫入數據 request: {request}")
        command = KeyencePlcCommand.write_continuous_data(
            request.device_type, request.start_address, request.values
        )
        response.success = False
        response.message = ""
        try:
            self.pool.execute(command)
        except Exception as e:
            self.get_logger().info(f"PLC 連續寫入數據失敗 {e}")
            response.message = f"PLC 連續寫入數據失敗 {e}"
            return response
        response.success = True
        self.get_logger().info("PLC 連續寫入數據完成")
        return response


def main():
    rclpy.init()
    node = PlcService()

    # 使用 MultiThreadedExecutor 支援多執行緒服務處理
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # 使用 spin 來啟動 ROS 事件循環
    executor.spin()

    # 當 spin 結束後，銷毀節點並進行清理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
