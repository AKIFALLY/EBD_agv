import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.clock import Clock
from rclpy.callback_groups import ReentrantCallbackGroup
import logging
from plc_interfaces.srv import (
    ForceOn,
    ForceOff,
    ReadData,
    WriteData,
    ReadContinuousData,
    WriteContinuousData,
    ReadContinuousByte,
    WriteContinuousByte,
)
from keyence_plc.keyence_plc_pool import KeyencePlcPool  # 連線池,管理實際PLC連線
from keyence_plc.keyence_plc_command import KeyencePlcCommand
from keyence_plc.keyence_plc_memory import PlcMemory
from keyence_plc.keyence_plc_bytes import PlcBytes
import threading
import time


class PlcService(Node):
    def __init__(self):
        super().__init__("plc_service")
#        self.get_logger().set_level(rclpy.logging.LoggingSeverity.ERROR)

        self.declare_parameter("plc_ip", "192.168.2.100")
        para_ip = self.get_parameter(
            "plc_ip").get_parameter_value().string_value
        self.get_logger().info(f"🚀PlcService Start {para_ip} !")

        self.get_logger().info(
            f"🧩PlcService Namespace {self.get_namespace()} !")
        # 定義自動讀取plc記憶體位置
        self.declare_parameter("read_ranges", ["DM,7600,200", "DM,5000,200"])
        raw_ranges = (
            self.get_parameter(
                "read_ranges").get_parameter_value().string_array_value
        )
        self.read_ranges = [tuple(r.split(",")) for r in raw_ranges]  # 解析參數

        self.ip = para_ip
        self.port = 8501
        self.pool = KeyencePlcPool(self.ip, self.port)
        # plc記憶體大小 65536 word (1 word = 2 byte)
        self.memory = PlcMemory(65536 * 2)
        self.clock = Clock()
        self.last_time = self.clock.now()
        self.responsed = True  # 記錄是否已經回應

        # ROS2 服務可重覆進入設定 (設為可重覆且多執行緒模式時, 非同步執行加快反應速度)
        self.callback_group = ReentrantCallbackGroup()

        # 註冊服務 強制 on/off 不可重覆進入 ,其他讀寫功能可重覆進入
        self.create_service(ForceOn, "force_on", self.force_on_callback,
                            qos_profile=rclpy.qos.QoSProfile(
                                depth=100, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
                            ),
                            callback_group=self.callback_group,)
        self.create_service(ForceOff, "force_off", self.force_off_callback,
                            qos_profile=rclpy.qos.QoSProfile(
                                depth=100, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
                            ),
                            callback_group=self.callback_group,)
        self.create_service(ReadData,
                            "read_data",
                            self.read_data_callback,
                            qos_profile=rclpy.qos.QoSProfile(
                                depth=100, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
                            ),
                            callback_group=self.callback_group,
                            )
        self.create_service(WriteData,
                            "write_data",
                            self.write_data_callback,
                            qos_profile=rclpy.qos.QoSProfile(
                                depth=100, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
                            ),
                            callback_group=self.callback_group,
                            )
        self.create_service(ReadContinuousData,
                            "read_continuous_data",
                            self.read_continuous_data_callback,
                            qos_profile=rclpy.qos.QoSProfile(
                                depth=100, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
                            ),
                            callback_group=self.callback_group,
                            )
        self.create_service(WriteContinuousData,
                            "write_continuous_data",
                            self.write_continuous_data_callback,
                            qos_profile=rclpy.qos.QoSProfile(
                                depth=100, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
                            ),
                            callback_group=self.callback_group,
                            )
        self.create_service(ReadContinuousByte,
                            "read_continuous_byte",
                            self.read_continuous_byte_callback,
                            qos_profile=rclpy.qos.QoSProfile(
                                depth=100, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
                            ),
                            callback_group=self.callback_group,
                            )

        self.timer = self.create_timer(
            0.1, self.read_plc_timer_callback)  # 100ms 執行一次

        rclpy.get_default_context().on_shutdown(self.shutdown_callback)

        # Replace ROS 2 timer with a separate thread
        # self.read_plc_thread = threading.Thread(target=self.read_plc_loop, daemon=True)
        # self.read_plc_thread.start()

    # def read_plc_loop(self):
    #    """Thread-based periodic PLC data reading."""
    #    while rclpy.ok():
    #        #self.get_logger().warn("Thread-based periodic PLC data reading")
    #        self.read_plc_timer_callback()
    #        time.sleep(0.2)  # 200ms interval

    def read_plc_timer_callback(self):
        """每秒主動讀取 PLC 資料"""
        now = self.clock.now()
        interval_ms = (now - self.last_time).nanoseconds / 1e6  # 轉換為毫秒
        self.last_time = now  # 更新時間
#
        #self.get_logger().info(f"Timer interval: {interval_ms:.3f} ms")
#
        for area, start, length in self.read_ranges:
            #
            # DM 7600 ,200
            command = KeyencePlcCommand.read_continuous_data(
                area, start, length)
            try:
                values = self.pool.execute(command)
#
                data = values.split()  # 拆分字串
                try:
                    # **轉換成 PlcBytes，每個 16-bit 轉成 2 bytes（小端序）**
                    data_bytes = PlcBytes()  # 使用 PlcBytes 來儲存資料
                    for x in data:
                        word = int(x) & 0xFFFF  # 確保是 16-bit 整數
                        data_bytes.extend(PlcBytes.from_int(word, 2))
                        # 使用 PlcBytes.from_int 轉換
                    # 儲存到記憶體
                    self.memory.set_memory(int(start), data_bytes)
                    # self.get_logger().info(
                    #    f"PLC Data Updated: {area} {start}-{int(start) + int(length)}"
                    # )
                    # 測試讀取字串
                    # get_string,長度是幾個byte(幾個字元)
                    # agvid = self.memory.get_string(7600, 20).replace("\x00", "")
                    # self.get_logger().info(f"AGV_ID {agvid}")
                except Exception as e:
                    self.get_logger().error(f"PLC Read Failed: {e}")
            except Exception as e:
                self.get_logger().error(f"PLC Read Command Failed: {e}")

    def shutdown_callback(self):
        self.get_logger().info("關閉 PLC 連線...")
        try:
            self.pool.stop()  # <- 加上這行通知 background thread 停止
            self.pool.close_connection()  # 中斷 PLC 連線
            self.get_logger().info("PLC 中斷連線")
        except Exception as e:
            self.get_logger().warn(f"PLC 中斷連線 例外 {e}")
        finally:
            self.get_logger().info("PLC disconnect()")

    def force_on_callback(self, request, response):
        # response.success = True
        # response.message = ""
        # return response
        self.get_logger().info(f"PLC 強制置位 request: {request}")
        command = KeyencePlcCommand.force_on(
            request.device_type, request.address)
        response.success = False
        response.message = ""
        try:
            self.pool.execute(command)
        except Exception as e:
            error_msg = f"PLC 強制置位失敗 {e},pool conn:{len(self.pool.connections)}"
            self.get_logger().warn(error_msg)
            response.message = error_msg
            return response
        response.success = True
        self.get_logger().info("PLC 強制置位完成")
        return response

    def force_off_callback(self, request, response):
        # response.success = True
        # response.message = ""
        # return response
        self.get_logger().info(f"PLC 強制復位 request: {request}")
        command = KeyencePlcCommand.force_off(
            request.device_type, request.address)
        response.success = False
        response.message = ""
        try:
            self.pool.execute(command)
        except Exception as e:
            error_msg = f"PLC 強制復位失敗 {e},pool conn:{len(self.pool.connections)}"
            self.get_logger().warn(error_msg)
            response.message = error_msg
            return response
        response.success = True
        self.get_logger().info("PLC 強制復位完成")
        return response

    def read_data_callback(self, request, response):
        # response.success = True
        # response.message = ""
        # return response
        # self.get_logger().info(f"PLC 讀取數據 request: {request}")
        command = KeyencePlcCommand.read_data(
            request.device_type, request.address)
        response.success = False
        response.value = ""
        response.message = ""
        try:
            response.value = self.pool.execute(command)
        except Exception as e:
            error_msg = f"PLC 讀取數據失敗 {e},pool conn:{len(self.pool.connections)}"
            self.get_logger().warn(error_msg)
            response.message = error_msg
            return response
        response.success = True
        # self.get_logger().info("PLC 讀取數據完成")
        return response

    def write_data_callback(self, request, response):
        # response.success = True
        # response.message = ""
        # return response
        self.get_logger().info(f"PLC 寫入數據 request: {request}")
        command = KeyencePlcCommand.write_data(
            request.device_type, request.address, request.value
        )
        response.success = False
        response.message = ""
        try:
            self.pool.execute(command)
        except Exception as e:
            error_msg = f"PLC 寫入數據失敗 {e},pool conn:{len(self.pool.connections)}"
            self.get_logger().warn(error_msg)
            response.message = error_msg
            return response
        response.success = True
        self.get_logger().info("PLC 寫入數據完成")
        return response

    def read_continuous_data_callback(self, request, response):
        # response.success = True
        # response.message = ""
        # return response
        # self.get_logger().debug(f"PLC 連續讀取數據 request: {request}")
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
            error_msg = (
                f"PLC 連續讀取數據失敗 {e},pool conn:{len(self.pool.connections)}"
            )
            self.get_logger().warn(error_msg)
            response.message = error_msg
            return response
        response.success = True
        # self.get_logger().debug("PLC 連續讀取數據完成")
        return response

    def write_continuous_data_callback(self, request, response):
        # response.success = True
        # response.message = ""
        # return response
        #self.get_logger().info(f"PLC 連續寫入數據 request: {request}")
        command = KeyencePlcCommand.write_continuous_data(
            request.device_type, request.start_address, request.values
        )
        response.success = False
        response.message = ""
        try:
            self.pool.execute(command)
        except Exception as e:
            error_msg = (
                f"PLC 連續寫入數據失敗 {e},pool conn:{len(self.pool.connections)}"
            )
            self.get_logger().warn(error_msg)
            response.message = error_msg
            return response
        response.success = True
        #self.get_logger().info("PLC 連續寫入數據完成")
        return response

    def read_continuous_byte_callback(self, request, response):
        # response.success = True
        # response.values = []
        # response.message = ""
        # return response
        # self.get_logger().debug(f"PLC 連續讀取數據 request: {request}")
        response.success = False
        response.values = []
        response.message = ""
        try:
            response.values = self.memory.get_bytes(
                int(request.start_address), request.count
            )
        except Exception as e:
            error_msg = f"PLC 連續讀取數據Byte失敗 {e}"
            self.get_logger().warn(error_msg)
            response.message = error_msg
            return response
        response.success = True
        self.get_logger().debug("PLC 連續讀取數據完成")
        return response


def main():
    rclpy.init()
    node = PlcService()

    # 使用 MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("🛑 偵測到 Ctrl+C，正在關閉節點...")
    finally:
        node.destroy_node()
        node.get_logger().info("🛑 節點已關閉，ROS 2 即將關閉。")
        executor.shutdown()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
