"""
PLC DM 監控節點
監控 PLC DM2500-2509 範圍，根據讀取的 work_id 自動建立 Task
"""

import rclpy
from rclpy.node import Node
from plc_proxy.plc_client import PlcClient
from alan_room_task_build.database_helper import DatabaseHelper
from alan_room_task_build import config


class PlcDmMonitorNode(Node):
    """PLC DM2500 範圍監控節點"""

    def __init__(self):
        super().__init__('plc_dm_monitor_node')

        # 初始化 PLC 客戶端
        self.plc_client = PlcClient(self)
        self.get_logger().info("✅ PLC Client 初始化完成")

        # 初始化資料庫助手（不依賴 TAFL）
        self.db_helper = DatabaseHelper(config.DATABASE_URL, self.get_logger())

        # 記錄已處理的 work_id（避免重複）
        self.processed_work_ids = set()

        # 定期檢查 Timer（每 1 秒）
        self.timer = self.create_timer(
            config.MONITOR_INTERVAL,
            self.check_plc_dm_callback
        )

        self.get_logger().info(
            f"🚀 PLC DM Monitor Node 已啟動 "
            f"(監控頻率: {config.MONITOR_INTERVAL} 秒)"
        )

    def check_plc_dm_callback(self):
        """定期檢查 PLC DM2500-2509（每 1 秒執行）"""

        # 讀取 DM2500 開始的 10 個 word
        response = self.plc_client.read_continuous_data(
            device_type="DM",
            start_address=config.DM_START_ADDRESS,
            count=config.DM_READ_COUNT
        )

        if not response or not response.success:
            self.get_logger().warn("⚠️ 讀取 PLC DM 失敗")
            return

        # 轉換為整數陣列
        try:
            values = [int(v) for v in response.values]
        except (ValueError, TypeError) as e:
            self.get_logger().error(f"❌ 解析 PLC DM 值失敗: {e}")
            return

        # 處理 Loader AGV (DM2500-2501)
        self._process_agv_dm(
            values,
            config.LOADER_DM_LOW,
            config.LOADER_DM_HIGH,
            config.LOADER_DM_START_ADDRESS,
            config.AGV_TYPE_LOADER
        )

        # 處理 Unloader AGV (DM2502-2503)
        self._process_agv_dm(
            values,
            config.UNLOADER_DM_LOW,
            config.UNLOADER_DM_HIGH,
            config.UNLOADER_DM_START_ADDRESS,
            config.AGV_TYPE_UNLOADER
        )

    def _process_agv_dm(
        self,
        values: list,
        low_index: int,
        high_index: int,
        dm_start_address: str,
        agv_type: str
    ):
        """
        處理單一 AGV 的 DM 值

        Args:
            values: DM 值陣列
            low_index: 低位 word 索引
            high_index: 高位 word 索引
            dm_start_address: DM 起始位址（用於清除）
            agv_type: AGV 類型
        """
        # 組合 32-bit work_id
        work_id = self._combine_32bit(values[low_index], values[high_index])

        # 若 work_id > 0 且未處理過，則處理
        if work_id > 0 and work_id not in self.processed_work_ids:
            self.get_logger().info(f"📥 檢測到 {agv_type} AGV work_id: {work_id}")

            # 處理 work_id（查詢 Work → 檢查重複 → 建立 Task）
            if self._process_work_id(work_id, agv_type):
                # 處理成功，清除 DM
                self._clear_dm_range(dm_start_address, 2)
                # 記錄已處理的 work_id
                self.processed_work_ids.add(work_id)
            else:
                self.get_logger().warn(f"⚠️ 處理 work_id {work_id} 失敗，不清除 DM")

    def _combine_32bit(self, low_word: int, high_word: int) -> int:
        """
        組合 32-bit 整數（低位 + 高位）

        Args:
            low_word: 低位 word（16-bit）
            high_word: 高位 word（16-bit）

        Returns:
            32-bit 整數
        """
        return low_word + (high_word << 16)

    def _extract_room_id(self, work_id: int) -> int:
        """
        提取 room_id（取第一位數字）

        Args:
            work_id: Work ID

        Returns:
            room_id（work_id 的第一位數字）
        """
        return int(str(work_id)[0])

    def _process_work_id(self, work_id: int, agv_type: str) -> bool:
        """
        處理 work_id：查詢 Work → 檢查重複 → 建立 Task

        Args:
            work_id: Work ID
            agv_type: AGV 類型（LOADER 或 UNLOADER）

        Returns:
            True 表示成功建立 Task，False 表示失敗
        """

        # 1. 查詢 Work 是否存在
        work = self.db_helper.get_work_by_id(work_id)
        if not work:
            self.get_logger().warn(f"⚠️ Work ID {work_id} 不存在，跳過")
            return False

        # 2. 提取 room_id（取第一位數字）
        room_id = self._extract_room_id(work_id)
        self.get_logger().info(
            f"📍 解析 room_id: {room_id} (來自 work_id {work_id})"
        )

        # 3. 檢查是否已有未完成的 Task（避免重複）
        if self.db_helper.check_duplicate_task(work_id, room_id):
            self.get_logger().info(
                f"⚠️ Work {work_id} 已有未完成的 Task，跳過"
            )
            return False

        # 4. 建立新 Task
        task = self.db_helper.create_task(
            work_id=work_id,
            room_id=room_id,
            agv_type=agv_type,
            work_name=work.name,
            status_id=config.DEFAULT_STATUS_ID,
            priority=config.DEFAULT_PRIORITY,
            agv_id=config.DEFAULT_AGV_ID,
            node_id=config.DEFAULT_NODE_ID
        )

        return task is not None

    def _clear_dm_range(self, start_address: str, count: int):
        """
        清除 DM 範圍

        Args:
            start_address: 起始位址
            count: 清除數量（word）
        """
        zeros = ["0"] * count
        response = self.plc_client.write_continuous_data(
            device_type="DM",
            start_address=start_address,
            values=zeros
        )

        if response and response.success:
            end_address = int(start_address) + count - 1
            self.get_logger().info(
                f"✅ 已清除 DM{start_address}-DM{end_address}"
            )
        else:
            self.get_logger().error(
                f"❌ 清除 DM{start_address} 失敗"
            )

    def destroy_node(self):
        """節點銷毀時清理資源"""
        self.db_helper.shutdown()
        super().destroy_node()


def main(args=None):
    """主函數"""
    rclpy.init(args=args)
    node = PlcDmMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
