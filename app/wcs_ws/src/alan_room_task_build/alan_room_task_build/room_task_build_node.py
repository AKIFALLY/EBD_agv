"""
房間任務建立節點
主要功能：
1. 監控 PLC DM3000-3009 範圍，根據讀取的 work_id 自動建立 Task
2. 自動清理已完成（status=4）和已取消（status=54）的 Task

觸發模式：邊緣觸發（只在 work_id 值改變時觸發任務建立）
清理頻率：每 2 秒自動清理一次
注意：系統只負責讀取和建立任務，不會清除或修改 PLC DM 資料
"""

import rclpy
from rclpy.node import Node
from plc_proxy.plc_client import PlcClient
from alan_room_task_build.database_helper import DatabaseHelper
from alan_room_task_build import config


class RoomTaskBuildNode(Node):
    """房間任務建立節點 - 監控 PLC DM3000 範圍（只讀模式）"""

    def __init__(self):
        super().__init__('room_task_build_node')

        # 初始化 PLC 客戶端
        self.plc_client = PlcClient(self)
        self.get_logger().info("✅ PLC Client 初始化完成")

        # 初始化資料庫助手（不依賴 TAFL）
        self.db_helper = DatabaseHelper(config.DATABASE_URL, self.get_logger())

        # 記錄上一次的 work_id 值（用於邊緣觸發檢測）
        self.last_loader_work_id = 0
        self.last_unloader_work_id = 0

        # 定期檢查 PLC DM Timer（每 1 秒）
        self.monitor_timer = self.create_timer(
            config.MONITOR_INTERVAL,
            self.check_plc_dm_callback
        )

        # 定期清理已完成/已取消 Task Timer（每 2 秒）
        self.cleanup_timer = self.create_timer(
            config.AUTO_CLEANUP_INTERVAL,
            self.cleanup_completed_tasks_callback
        )

        self.get_logger().info(
            f"🚀 Room Task Build Node 已啟動 "
            f"(監控頻率: {config.MONITOR_INTERVAL} 秒, "
            f"觸發模式: 邊緣觸發, "
            f"自動清理: {config.AUTO_CLEANUP_INTERVAL} 秒)"
        )

    def check_plc_dm_callback(self):
        """定期檢查 PLC DM3000-3009（每 1 秒執行）- 使用異步調用"""

        # 使用異步調用讀取 PLC DM
        self.plc_client.async_read_continuous_data(
            device_type="DM",
            start_address=config.DM_START_ADDRESS,
            count=config.DM_READ_COUNT,
            callback=self._handle_plc_response
        )

    def cleanup_completed_tasks_callback(self):
        """定期清理已完成/已取消的 Task（每 2 秒執行）"""

        try:
            # 刪除已完成或已取消的 Task
            deleted_count = self.db_helper.delete_completed_tasks(
                config.CLEANUP_STATUS_IDS
            )

            # 只在有刪除時才記錄 INFO 日誌
            if deleted_count > 0:
                self.get_logger().info(
                    f"🗑️ 自動清理完成：已刪除 {deleted_count} 個 Task "
                    f"(status in {config.CLEANUP_STATUS_IDS})"
                )

        except Exception as e:
            self.get_logger().error(f"❌ 自動清理 Task 失敗: {e}")

    def _handle_plc_response(self, response):
        """處理 PLC 異步回應"""
        if not response or not response.success:
            self.get_logger().warn("⚠️ 讀取 PLC DM 失敗")
            return

        # 轉換為整數陣列
        try:
            values = [int(v) for v in response.values]
        except (ValueError, TypeError) as e:
            self.get_logger().error(f"❌ 解析 PLC DM 值失敗: {e}")
            return

        # 處理 Loader AGV (DM3000-3001)
        self.last_loader_work_id = self._process_agv_dm(
            values,
            config.LOADER_DM_LOW,
            config.LOADER_DM_HIGH,
            config.AGV_TYPE_LOADER,
            self.last_loader_work_id
        )

        # 處理 Unloader AGV (DM3002-3003)
        self.last_unloader_work_id = self._process_agv_dm(
            values,
            config.UNLOADER_DM_LOW,
            config.UNLOADER_DM_HIGH,
            config.AGV_TYPE_UNLOADER,
            self.last_unloader_work_id
        )

    def _process_agv_dm(
        self,
        values: list,
        low_index: int,
        high_index: int,
        agv_type: str,
        last_work_id: int
    ) -> int:
        """
        處理單一 AGV 的 DM 值（邊緣觸發模式）

        觸發機制：
        - 邊緣觸發：值改變時立即處理

        Args:
            values: DM 值陣列
            low_index: 低位 word 索引
            high_index: 高位 word 索引
            agv_type: AGV 類型
            last_work_id: 上一次的 work_id 值

        Returns:
            當前的 work_id 值
        """
        # 組合 32-bit work_id
        current_work_id = self._combine_32bit(values[low_index], values[high_index])

        # 判斷是否需要處理
        should_process = False

        # 觸發條件: 邊緣觸發（值改變）
        if current_work_id != last_work_id:
            should_process = True
            self.get_logger().info(
                f"📥 檢測到 {agv_type} AGV work_id 變化: {last_work_id} → {current_work_id}"
            )

            # 如果變成 0，記錄日誌但不處理
            if current_work_id == 0:
                self.get_logger().info(f"🔄 {agv_type} AGV work_id 已清除")
                should_process = False

        # 執行處理
        if should_process and current_work_id > 0:
            # 先檢查 work_id 是否存在
            work = self.db_helper.get_work_by_id(current_work_id)
            if not work:
                self.get_logger().warn(
                    f"⚠️ Work ID {current_work_id} 不存在，跳過處理"
                )
                return current_work_id

            # 處理 work_id（檢查重複 → 建立 Task）
            if not self._process_work_id(current_work_id, agv_type, work):
                self.get_logger().warn(
                    f"⚠️ [邊緣觸發] 處理 work_id {current_work_id} 失敗或已跳過"
                )
            else:
                self.get_logger().info(
                    f"✅ [邊緣觸發] 成功處理 work_id {current_work_id}"
                )

        # 返回當前值，用於下次比較
        return current_work_id

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

    def _process_work_id(self, work_id: int, agv_type: str, work) -> bool:
        """
        處理 work_id：檢查重複 → 建立 Task

        Args:
            work_id: Work ID
            agv_type: AGV 類型（LOADER 或 UNLOADER）
            work: Work 物件（已查詢）

        Returns:
            True 表示成功建立 Task，False 表示失敗
        """

        # 1. 提取 room_id（取第一位數字）
        room_id = self._extract_room_id(work_id)
        self.get_logger().info(
            f"📍 解析 room_id: {room_id} (來自 work_id {work_id})"
        )

        # 2. 檢查是否已有未完成的 Task（避免重複）
        if self.db_helper.check_duplicate_task(work_id, room_id):
            self.get_logger().info(
                f"⚠️ Work {work_id} 已有未完成的 Task，跳過"
            )
            return False

        # 3. 查詢對應的 AGV
        agv_name = f"{agv_type.lower()}{room_id:02d}"
        agv = self.db_helper.get_agv_by_name(agv_name)

        if not agv:
            self.get_logger().warn(
                f"⚠️ 找不到對應的 AGV: {agv_name} (enable=1)，跳過創建任務"
            )
            return False

        agv_id = agv.id
        self.get_logger().info(
            f"✅ 找到對應的 AGV: {agv_name} (ID={agv_id}, Model={agv.model})"
        )

        # 4. 建立新 Task（使用查詢到的 agv_id，並從 work.parameters 提取 node_id）
        task = self.db_helper.create_task(
            work_id=work_id,
            room_id=room_id,
            agv_type=agv_type,
            work=work,  # 傳入 work 物件以提取 parameters.nodes
            work_name=work.name,
            status_id=config.DEFAULT_STATUS_ID,
            priority=config.DEFAULT_PRIORITY,
            agv_id=agv_id,
            node_id=config.DEFAULT_NODE_ID  # 預設值，會被 work.parameters.nodes 覆蓋
        )

        return task is not None

    def destroy_node(self):
        """節點銷毀時清理資源"""
        self.db_helper.shutdown()
        super().destroy_node()


def main(args=None):
    """主函數"""
    rclpy.init(args=args)
    node = RoomTaskBuildNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
