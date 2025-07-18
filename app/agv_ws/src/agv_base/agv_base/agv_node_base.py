import time
from agv_base.base_context import BaseContext
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import SingleThreadedExecutor
import threading
from rclpy.clock import Clock
from plc_interfaces.srv import ReadContinuousByte  # 替換成你的實際 service 名稱
from keyence_plc.keyence_plc_memory import PlcMemory
from plc_proxy.plc_client import PlcClient
from agv_base.states.idle_state import IdleState
from agv_base.agv_status import AgvStatus
from agv_interfaces.msg import AgvStatus as AgvStatusMsg


class AgvNodebase(Node):
    def __init__(self, node_name='agv_node_base', **kwargs):
        super().__init__(node_name=node_name, **kwargs)

        # 初始化讀取PLC狀態
        self.dMmemory = PlcMemory(
            65536 * 2
        )  # PLC 記憶體大小 65536 word (1 word = 2 byte)
        # 創建服務客戶端讀--讀取AGV_PLC狀態
        self.plc_client = PlcClient(self)

        # self.client = self.create_client(ReadContinuousByte, 'read_continuous_byte')
        # 輸出日誌信息
        self.get_logger().info("ROS 2  AGV狀態機啟動 None_State")
        self.agv_status = AgvStatus()  # 初始化 AGV 狀態
        # 等待服務可用

        # 創建 BaseContext 並傳入初始狀態 (IdleState)
        self.base_context = BaseContext(IdleState(self))  # 初始狀態為 Idle
        # 50ms 執行一次主迴圈(read plc data , context.handle)
        self.timer = self.create_timer(0.05, self.main_loop_timer)
        self.timer_to_write_status = self.create_timer(1.5, self.toWriteStatus)
        self._status_publisher = self.create_publisher(AgvStatusMsg, "/agv/status", 10)
        self.clock = Clock()  # 建立 ROS 2 時鐘
        self.start_time = self.clock.now()  # 記錄請求開始時間
        self.requesting = False  # 重置請求狀態
        self.read_cycle_time_ms = 200  # 設定循環時間為 200ms
        self.robot_finished = False  # 機器人是否完成動作
        self._running = False
        self._thread = None
        self.plc_heartbeat = 0  # PLC 心跳計數器
        self.BaseState = 0  # 狀態機狀態
        self.writing_status = False  # 狀態寫入 PLC 標誌
        self.count = 0  # 計數器，用於執行次數

        # self.start(one_cycle_ms=50)
        self.last_one_sec = int(time.time() * 1000)  # 取得現在時間（ms）

    def start(self, one_cycle_ms=50):
        """啟動搖桿監聽 (獨立執行緒)"""
        if self._running:
            return

        self._running = True
        self._thread = threading.Thread(
            target=self.main_loop, args=(one_cycle_ms,))
        self._thread.daemon = True
        self._thread.start()

    def stop(self):
        """停止"""
        print("[STOP] Called stop()...")
        self._running = False
        if self._thread and self._thread.is_alive():
            if threading.current_thread() != self._thread:
                self._thread.join(timeout=1.0)
            else:
                print("⚠️ 無法在 agv_node_base 執行緒內 join 自己，略過 join()")

    def main_loop(self, one_cycle_ms=50):
        """主迴圈"""
        last_cycle_time = int(time.time() * 1000)  # 取得現在時間（ms）
        while self._running and rclpy.ok():
            now = int(time.time() * 1000)  # 取得現在時間（ms）
            elapsed = now - last_cycle_time  # 經過的毫秒數
            if elapsed >= one_cycle_ms:
                last_cycle_time = now  # 更新上次執行時間
                try:
                    self.get_logger().info(
                        f"✅ main_loop fired! elapsed = {elapsed} ms")
                    self.read_plc_data()
                    self.context_handle()
                except Exception as e:
                    msg = str(e)
                    if not rclpy.ok() or "context is invalid" in msg:
                        self.get_logger().warn("🛑 ROS context 無效或已關閉，將退出執行迴圈。")
                        break
                    self.get_logger().error(f"⚠️ AGV 狀態機異常: {msg}")

            else:
                time.sleep(0.001)  # 避免 CPU 空轉
        self._running = False

    def main_loop_timer(self):
        """主迴圈"""

        now = int(time.time() * 1000)  # 取得現在時間（ms）
        elapsed = now - self.last_one_sec  # 經過的毫秒數
        if elapsed >= 1000:
            self.last_one_sec = now  # 更新上次執行時間
            # 1000 # 1秒發佈一次狀態
            self.publish_agv_status()

        try:
            # self.get_logger().info(f"requesting!{self.requesting}")
            if self.requesting:
                self.count += 1
            self.read_plc_data()
            self.context_handle()

            if self.count > 30:
                self.requesting = False  # 重置請求狀態
                self.count = 0  # 重置計數器
        except Exception as e:
            self.get_logger().error(f"❌ 主迴圈異常: {e}")

    def context_handle(self):
        try:
            # 處理當前的 AGV 狀態邏輯
            self.base_context.handle()
            pass
        except Exception as e:
            self.get_logger().error(f"未處理的 AGV 狀態機異常: {e}")

    def read_plc_data(self):  # 讀取AGV_PLC資料
        # 請求中不再請求
        if self.requesting:
            return
        # 計算時間差 到達循環時間200ms 才讀取PLC
        elapsed_time_ms = (self.clock.now() -
                           self.start_time).nanoseconds / 1e6
        if elapsed_time_ms < self.read_cycle_time_ms:
            return

        self.requesting = True
        self.start_time = self.clock.now()  # 使用 ROS 時鐘記錄時間
        start_address = "7600"
        self.plc_client.async_read_continuous_byte(
            device_type="DM",
            start_address="7600",
            count=200,
            callback=lambda res, sa=start_address: self.response_callback(
                res, sa),
        )
        # self.get_logger().info(f"📡 發送請求到 PLC: DM {start_address}，請求數量: 200")

    def publish_agv_status(self):
        try:
            msg = AgvStatusMsg()
            msg.agv_id = self.agv_status.AGV_ID or ""
            msg.slam_x = self.agv_status.AGV_SLAM_X or 0
            msg.slam_y = self.agv_status.AGV_SLAM_Y or 0
            msg.slam_theta = self.agv_status.AGV_SLAM_THETA or 0
            msg.power = self.agv_status.POWER or 0.0
            msg.x_speed = self.agv_status.AGV_X_SPEED or 0
            msg.y_speed = self.agv_status.AGV_Y_SPEED or 0
            msg.theta_speed = self.agv_status.AGV_THETA_SPEED or 0
            msg.front_pgv = self.agv_status.AGV_FPGV or 0
            msg.back_pgv = self.agv_status.AGV_BPGV or 0
            msg.start_point = self.agv_status.AGV_START_POINT or 0
            msg.end_point = self.agv_status.AGV_END_POINT or 0
            msg.action = self.agv_status.AGV_ACTION or 0
            msg.zone = self.agv_status.AGV_ZONE or 0
            msg.status1 = self.agv_status.AGV_STATUS1 or 0
            msg.status2 = self.agv_status.AGV_STATUS2 or 0
            msg.status3 = self.agv_status.AGV_STATUS3 or 0
            msg.alarm1 = self.agv_status.AGV_ALARM1 or 0
            msg.alarm2 = self.agv_status.AGV_ALARM2 or 0
            msg.alarm3 = self.agv_status.AGV_ALARM3 or 0
            msg.alarm4 = self.agv_status.AGV_ALARM4 or 0
            msg.alarm5 = self.agv_status.AGV_ALARM5 or 0
            msg.alarm6 = self.agv_status.AGV_ALARM6 or 0
            msg.magic = self.agv_status.MAGIC or 0
            msg.layer = self.agv_status.AGV_LAYER or 0

            self._status_publisher.publish(msg)
        except:
            self.get_logger().error("Error publishing AGV_PLC data")
            pass
    # 讀取AGV_PLC資料回傳

    def response_callback(self, response, start_address):
        try:
            end_time = self.clock.now()  # 取得回應時間
            elapsed_time = (
                end_time - self.start_time
            ).nanoseconds / 1e6  # 轉換為毫秒 (ms)
            # self.get_logger().info(
            #    f"Service call duration: {elapsed_time:.3f} ms"
            # )  # 顯示執行時間
            if response.success:
                self.dMmemory.set_memory(int(start_address), response.values)
                self.agv_status.get_agv_status(self.dMmemory)
                self.agv_status.get_agv_bitstatus(self.dMmemory)
                self.agv_status.get_agv_door_open_status(self.dMmemory)
                self.agv_status.get_agv_inputs(self.dMmemory)
                self.agv_status.get_agv_outputs(self.dMmemory)
                self.agv_status.get_alarm_status(self.dMmemory)

                # self.get_logger().info(f"Received string: {response.values}")
                # self.get_logger().info(f"Service call duration: {elapsed_time:.3f} ms")  # 顯示執行時間
            else:
                self.get_logger().error(
                    f"Failed to read data: {response.message}")
                pass
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")
        finally:
            self.requesting = False  # 重置請求狀態

    def toWriteStatus(self):
        """寫入AGV狀態到PLC"""
        self.plc_heartbeat += 1

        # write_countinuous_byte
        if self.writing_status:  # 寫入狀態標誌為 True，則不執行寫入
            return
        # 根據當前狀態設定 BaseState
        if self.base_context.state.__class__.__name__ == "IdleState":
            self.BaseState = 1
        elif self.base_context.state.__class__.__name__ == "ManualState":
            self.BaseState = 2
        elif self.base_context.state.__class__.__name__ == "AutoState":
            self.BaseState = 3
        elif self.base_context.state.__class__.__name__ == "ErrorState":
            self.BaseState = 4
        # 寫入資料到DM7800
        valuedata = [str(self.plc_heartbeat), str(self.BaseState)]
        try:
            self.plc_client.async_write_continuous_data(
                device_type='DM',
                start_address='7800',
                values=valuedata,
                callback=self.plc_write_status_callback
            )
            self.writing_status = True  # 設置寫入狀態標誌
            # self.get_logger().info("✅ AGV 狀態已寫入 PLC (DM7800)")
        except Exception as e:
            self.get_logger().error(f"寫入 PLC 狀態失敗: {str(e)}")
            return

    def plc_write_status_callback(self, response):
        self.writing_status = False  # 重置寫入狀態標誌
        if response.success:
            # self.get_logger().info("✅ 狀態寫入PLC成功")
            pass
        else:
            self.get_logger().warn("⚠️ 狀態寫入PLC失敗")

    def destroy_node(self):
        self.stop()
        self.plc_client.destroy()
        super().destroy_node()


def main():
    rclpy.init()
    node = AgvNodebase()

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
