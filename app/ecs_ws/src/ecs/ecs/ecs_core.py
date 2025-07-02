import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from keyence_plc.keyence_plc_memory import PlcMemory

from rclpy.executors import MultiThreadedExecutor
from plc_proxy.plc_client import PlcClient
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.crud.eqp_crud import eqp_crud, eqp_port_crud, eqp_signal_crud
import re
import time

class EcsCore(Node):
    def __init__(self):
        super().__init__('ecs_core')

        namespace = self.get_namespace()
        self.get_logger().info(f"🔧 服務命名空間: {namespace}")

        db_url_agvc = self.declare_parameter(
            'db_url_agvc',
            'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'
        ).value
        self.get_logger().info(f"使用資料庫 URL: {db_url_agvc}")

        # 使用 SQLModel metadata 建立資料表
        self.pool_agvc = ConnectionPoolManager(
            db_url_agvc)  # SQLModel.metadata.create_all

        # eqp_signal 中dm有做設定的資料 抓出來,要更新時使用
        self.available_signals = []

        self.read_signals_from_db()

        self.plc_client = PlcClient(self)
        self.memory = PlcMemory(65536 * 2)  # word-based，實際是 byte array
        self.plc_loaded = []
        self.clock = Clock()

        self.declare_parameter('read_ranges', ["DM,7600,20", "DM,5000,200"])
        raw_ranges = self.get_parameter(
            'read_ranges').get_parameter_value().string_array_value
        self.read_ranges = [
            (device_type.strip(), int(start.strip()), int(count.strip()))
            for device_type, start, count in (r.split(",") for r in raw_ranges)
        ]
        # 初始化所有地址都未完成讀取
        self.plc_loaded = {
            int(start.strip()): False
            for device_type, start, count in (r.split(",") for r in raw_ranges)
        }

        # 先讀1次
        self.read_plc_data()
        # 0.1 秒的時間周期，用於讀取 Agvc 主PLC 的資料
        self.timer = self.create_timer(0.1, self.main_loop_timer)

    def main_loop_timer(self):

        self.read_plc_data()

        self.write_signals_to_db()
        
        #self.read_signals_from_db()

    def read_plc_data(self):
        for device_type, start_address, count in self.read_ranges:
            self.plc_client.async_read_continuous_byte(
                device_type=device_type,
                start_address=str(start_address),
                count=count,
                callback=lambda res, sa=start_address: self.handle_plc_response(
                    res, sa)
            )
    def read_signals_from_db(self):        
        self.available_signals.clear()
        with self.pool_agvc.get_session() as session:
            all_signals = eqp_signal_crud.get_signals_with_dm(session)
            for signal in all_signals:
                self.available_signals.append(signal[0])
        
    def write_signals_to_db(self):
        if not all(self.plc_loaded.values()):
            return  # 尚未全部完成

        with self.pool_agvc.get_session() as session:
            for signal in self.available_signals:
#                    print( signal.dm_address, signal.type_of_value)
                try:
                    # 只保留 數字和小數點的字元
                    address = re.sub(r'[^0-9\.]', '', signal.dm_address)                        
                    value = self.memory.get_value(address, signal.type_of_value)              
                    #value = self.memory.get_value(signal.dm_address, signal.type_of_value)
                    #數j至有變動時才寫入更新
                    if signal.value != str(value) :
                        self.get_logger().info(f"變動[{signal.value}] to {str(value)} for {signal.name}")
                        signal.value = str(value)
                        session.merge(signal)  # or session.add(signal) if new
                    #eqp_signal_crud.update(session,signal.id,signal)

                except Exception as e:
                    self.get_logger().warning(f"⚠️ Failed to get value for {signal.name} at {signal.dm_address}: {e}")

            session.commit()  # ✅ 最後才做一次提交 中間用merge ,提高效率

    def handle_plc_response(self, response, start_address):

        start = time.perf_counter()
        if response and response.success:
            try:
                self.memory.set_memory(start_address, response.values)
                # 測試印出部分記憶體內容
                # self.get_logger().info(f"Memory[{start_address}] updated: {response.values[:10]}")
                # self.get_logger().info(f"Memory: {self.memory.get_string(7600, 20)}")
                # self.get_logger().info(f"Memory: {self.memory.get_bool("5000.0")}")
                self.plc_loaded[start_address] = True #設為已讀取

            except Exception as e:
                self.get_logger().error(
                    f"❌ Failed to update memory at {start_address}: {e}")
        else:
            msg = response.message if response else "No response"
            self.get_logger().error(f"🚨 Read failed at {start_address}: {msg}")

        end = time.perf_counter()
        #print(f"🔧 read_plc_data 耗時: {end - start:.6f} 秒")

def main(args=None):
    rclpy.init(args=args)
    ecs_core = EcsCore()
    executor = MultiThreadedExecutor()
    executor.add_node(ecs_core)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass  # Ctrl+C was prese
    finally:
        executor.shutdown()
        ecs_core.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
