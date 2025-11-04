import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from keyence_plc.keyence_plc_memory import PlcMemory

from rclpy.executors import MultiThreadedExecutor
from plc_proxy.plc_client import PlcClient
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.crud.eqp_crud import eqp_crud, eqp_port_crud, eqp_signal_crud
from db_proxy.crud.carrier_crud import carrier_crud
from db_proxy.models import EqpSignal
from sqlmodel import select
from ecs.door_controller_config import DoorControllerConfig
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

        # 用於追蹤預烘機 carrier 狀態（避免重複寫入 PLC）
        # Port ID 2051-2058 對應 DM2000-2007
        self.dryer_carrier_status = {
            2000: None,  # Port 2051
            2001: None,  # Port 2052
            2002: None,  # Port 2053
            2003: None,  # Port 2054
            2004: None,  # Port 2055
            2005: None,  # Port 2056
            2006: None,  # Port 2057
            2007: None,  # Port 2058
        }

        self.read_signals_from_db()

        # 🆕 自動同步 door_config.yaml 中的門信號到資料庫
        self.sync_door_signals_from_config()

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
        # 1.0 秒的時間周期，用於更新預烘機 Carrier 狀態到 PLC
        self.carrier_status_timer = self.create_timer(1.0, self.carrier_status_timer_callback)

    def main_loop_timer(self):

        self.read_plc_data()

        self.write_signals_to_db()

        #self.read_signals_from_db()

    def carrier_status_timer_callback(self):
        """1 秒週期的回調函數，用於更新預烘機 Carrier 狀態到 PLC"""
        self.read_carrier_in_dryer_write_to_main()

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

    def sync_door_signals_from_config(self):
        """從 door_config.yaml 同步自動門信號到資料庫

        此方法在 ECS Core 啟動時執行，確保 door_config.yaml 中定義的
        所有門都有對應的資料庫信號記錄，實現配置文件到資料庫的自動同步。

        流程：
        1. 讀取 /app/config/door_config.yaml
        2. 檢查每個門的 DM 地址是否存在於 eqp_signal 表
        3. 如果不存在，自動創建對應的信號記錄
        4. 重新載入 available_signals 列表
        """
        try:
            # 載入門配置
            door_config = DoorControllerConfig()
            door_config.load_config_yaml("/app/config/door_config.yaml")

            self.get_logger().info("🚪 開始同步 door_config.yaml 到資料庫...")

            created_count = 0
            with self.pool_agvc.get_session() as session:
                for door_id, door_cfg in door_config.doors.items():
                    # 檢查信號是否已存在 (根據 DM 地址)
                    stmt = select(EqpSignal).where(
                        EqpSignal.dm_address == door_cfg.dm_address
                    )
                    existing_signal = session.exec(stmt).first()

                    if not existing_signal:
                        # 自動創建缺失的門信號
                        new_signal = EqpSignal(
                            eqp_id=999,  # 自動門系統設備 ID
                            name=f"Door_{door_id}_Status",
                            description=f"自動門{door_id}狀態 (door_config.yaml 自動創建)",
                            value="0",  # 預設值：0=關閉
                            type_of_value="int",
                            dm_address=door_cfg.dm_address
                        )
                        session.add(new_signal)
                        created_count += 1
                        self.get_logger().info(
                            f"✅ 自動創建門信號: Door {door_id} → DM {door_cfg.dm_address} "
                            f"(MR {door_cfg.mr_address} 控制)"
                        )
                    else:
                        self.get_logger().debug(
                            f"✓ 門信號已存在: Door {door_id} → DM {door_cfg.dm_address}"
                        )

                # 提交所有變更
                if created_count > 0:
                    session.commit()
                    self.get_logger().info(
                        f"✅ 門信號同步完成: 新增 {created_count} 個門信號"
                    )
                else:
                    self.get_logger().info(
                        "✅ 門信號同步完成: 所有門信號已存在，無需創建"
                    )

                # 🔑 關鍵修復：無論是否創建新記錄，都重新載入信號列表
                # 這確保 available_signals 包含所有門信號
                self.read_signals_from_db()

                # 🔍 診斷日誌：確認門信號已載入
                door_signals_in_memory = [
                    s for s in self.available_signals
                    if "Door" in s.name or (s.dm_address and s.dm_address in ['5000', '5001', '5002', '5003', '5004'])
                ]
                self.get_logger().info(
                    f"🔄 已重新載入信號列表，當前監控 {len(self.available_signals)} 個信號 "
                    f"（其中門信號: {len(door_signals_in_memory)} 個）"
                )

                # 🔍 詳細列出門信號（用於診斷）
                if door_signals_in_memory:
                    for door_signal in door_signals_in_memory:
                        self.get_logger().debug(
                            f"  📍 {door_signal.name}: DM{door_signal.dm_address} (當前值: {door_signal.value})"
                        )
                else:
                    self.get_logger().warning(
                        "⚠️ 未找到門信號！請檢查數據庫中的 eqp_signal 記錄"
                    )

        except FileNotFoundError:
            self.get_logger().warning(
                "⚠️ door_config.yaml 未找到，跳過門信號同步"
            )
        except Exception as e:
            self.get_logger().error(
                f"❌ 門信號同步失敗: {e}",
                exc_info=True
            )

    def write_signals_to_db(self):
        # 檢查 PLC 數據是否已載入完成
        if not all(self.plc_loaded.values()):
            # 只在初次幾次循環記錄，避免日誌過多
            if not hasattr(self, '_plc_load_warn_count'):
                self._plc_load_warn_count = 0
            if self._plc_load_warn_count < 3:
                self.get_logger().warning(
                    f"⚠️ PLC 數據尚未完全載入，跳過信號更新。當前狀態: {self.plc_loaded}"
                )
                self._plc_load_warn_count += 1
            return  # 尚未全部完成

        with self.pool_agvc.get_session() as session:
            for signal in self.available_signals:
                try:
                    # 只保留 數字和小數點的字元
                    address = re.sub(r'[^0-9\.]', '', signal.dm_address)
                    value = self.memory.get_value(address, signal.type_of_value)

                    # 數據有變動時才寫入更新
                    if signal.value != str(value):
                        # 門信號用 🚪 圖標區分，便於日誌查看
                        is_door_signal = "Door" in signal.name
                        log_msg = f"變動[{signal.value}] to {str(value)} for {signal.name}"
                        if is_door_signal:
                            self.get_logger().info(f"🚪 {log_msg}")
                        else:
                            self.get_logger().info(log_msg)
                        signal.value = str(value)
                        session.merge(signal)

                except Exception as e:
                    # 門信號錯誤用 error 級別，其他用 warning
                    if "Door" in signal.name:
                        self.get_logger().error(
                            f"❌ 門信號讀取失敗: {signal.name} at DM{signal.dm_address}: {e}"
                        )
                    else:
                        self.get_logger().warning(
                            f"⚠️ Failed to get value for {signal.name} at {signal.dm_address}: {e}"
                        )

            session.commit()  # ✅ 最後才做一次提交 中間用merge ,提高效率

    def read_carrier_in_dryer_write_to_main(self):
        """
        讀取 carrier 資料表，若有 carrier 在預烘機的 port id (2051~2058)，
        則寫入 1 到主 PLC 的 DM2000~DM2007，無 carrier 則寫入 0。
        使用批次寫入，只在整體狀態變化時才寫入 PLC。

        Port ID 對應 DM 地址：
        - Port 2051 → DM2000
        - Port 2052 → DM2001
        - Port 2053 → DM2002
        - Port 2054 → DM2003
        - Port 2055 → DM2004
        - Port 2056 → DM2005
        - Port 2057 → DM2006
        - Port 2058 → DM2007
        """
        try:
            # 建立當前狀態陣列（按 DM 地址順序：2000-2007）
            current_status = [0] * 8  # 預設都是 0

            # 查詢 carrier 資料表中 port_id 在 2051-2058 的記錄
            with self.pool_agvc.get_session() as session:
                from sqlmodel import select
                from db_proxy.models import Carrier

                # 查詢預烘機 port 中的 carrier
                stmt = select(Carrier).where(
                    Carrier.port_id.in_([2051, 2052, 2053, 2054, 2055, 2056, 2057, 2058])
                )
                carriers = session.exec(stmt).all()

                # 根據查詢結果，設定對應的陣列索引為 1
                for carrier in carriers:
                    if carrier.port_id is not None:
                        # Port ID 2051-2058 對應陣列索引 0-7
                        index = carrier.port_id - 2051
                        current_status[index] = 1

                # 檢查整體狀態是否有變化
                old_status = [
                    self.dryer_carrier_status[2000 + i] for i in range(8)
                ]

                if current_status != old_status:
                    # 記錄變化的 port
                    changes = []
                    for i in range(8):
                        if current_status[i] != old_status[i]:
                            port_id = 2051 + i
                            dm_addr = 2000 + i
                            changes.append(
                                f"Port{port_id}(DM{dm_addr}): {old_status[i]}→{current_status[i]}"
                            )

                    self.get_logger().info(
                        f"🔄 預烘機 Carrier 狀態變化: {', '.join(changes)}"
                    )

                    # 使用批次寫入一次寫入所有 8 個 DM
                    self.plc_client.async_write_continuous_data(
                        device_type="DM",
                        start_address="2000",
                        values=[str(v) for v in current_status],
                        callback=lambda res, status=current_status: self._handle_write_dryer_status_response(res, status)
                    )

        except Exception as e:
            self.get_logger().error(
                f"❌ read_carrier_in_dryer_write_to_main 執行失敗: {e}"
            )

    def handle_plc_response(self, response, start_address):

        start = time.perf_counter()
        if response and response.success:
            try:
                # 📊 診斷日誌：記錄實際收到的數據量
                #actual_bytes = len(response.values)
                #actual_words = actual_bytes // 2
                #self.get_logger().info(
                #    f"📊 PLC 讀取診斷 | 起始地址: DM{start_address} | "
                #    f"收到: {actual_bytes} bytes ({actual_words} words)"
                #)
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

    def _handle_write_dryer_status_response(self, response, current_status):
        """處理異步寫入預烘機狀態的回調函數

        Args:
            response: PLC 寫入回應對象
            current_status: 寫入的狀態列表 [0-7]
        """
        if response and response.success:
            # 更新快取狀態
            for i in range(8):
                self.dryer_carrier_status[2000 + i] = current_status[i]
            self.get_logger().info(
                "✅ 成功批次寫入 DM2000-2007"
            )
        else:
            msg = response.message if response else "無回應"
            self.get_logger().error(
                f"❌ 批次寫入 DM2000-2007 失敗: {msg}"
            )

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
