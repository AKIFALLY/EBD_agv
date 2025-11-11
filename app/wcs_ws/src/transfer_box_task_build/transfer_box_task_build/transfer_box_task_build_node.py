"""
Conveyor Task Build Node - 通用傳送箱任務建立節點

功能：
1. 遍歷所有傳送箱，監控 Rack carrier_bitmap 並寫入 PLC
2. 統一監控 PLC DM3010-3011 (work_id)，根據 work_id 分發建立 Task
3. 遍歷所有傳送箱，讀取 PLC 回饋在席值並更新 Rack
4. 自動清理已完成的 Task
"""

import rclpy
from rclpy.node import Node
from plc_proxy.plc_client import PlcClient
from transfer_box_task_build.database_helper import DatabaseHelper
from transfer_box_task_build.transfer_box_manager import TransferBoxManager
from transfer_box_task_build import config


class TransferBoxTaskBuildNode(Node):
    """通用傳送箱任務自動建立節點"""

    def __init__(self):
        super().__init__('transfer_box_task_build_node')

        # PLC 客戶端
        self.plc_client = PlcClient(self)
        self.get_logger().info("✅ PLC Client 初始化完成")

        # 資料庫助手
        self.db_helper = DatabaseHelper(config.DATABASE_URL, self.get_logger())

        # 傳送箱管理器
        self.transfer_box_manager = TransferBoxManager()
        self.get_logger().info(
            f"✅ 傳送箱管理器初始化完成 (共 {self.transfer_box_manager.get_transfer_box_count()} 個傳送箱)"
        )

        # 邊緣觸發記錄（PLC work_id）
        self.last_work_id = 0

        # PLC Rack 資訊快取（用於異步讀取）
        self.plc_rack_cache = {}

        # Timer 1: 每 3 秒遍歷所有傳送箱 → 寫入 PLC
        self.rack_monitor_timer = self.create_timer(
            config.RACK_CHECK_INTERVAL,
            self.check_racks_and_write_plc_callback
        )

        # Timer 2: 每 1 秒監控 PLC DM3010-3011 → 建立 Task
        self.plc_monitor_timer = self.create_timer(
            config.PLC_MONITOR_INTERVAL,
            self.check_plc_dm_callback
        )

        # Timer 3: 每 2 秒清理已完成的 Task
        self.cleanup_timer = self.create_timer(
            config.AUTO_CLEANUP_INTERVAL,
            self.cleanup_completed_tasks_callback
        )

        # Timer 4: 每 10 秒遍歷所有傳送箱 → 讀取 PLC 回饋並更新 Rack
        self.plc_feedback_timer = self.create_timer(
            config.FEEDBACK_UPDATE_INTERVAL,
            self.update_racks_from_plc_callback
        )

        self.get_logger().info("✅ TransferBoxTaskBuildNode 初始化完成")
        self.get_logger().info(
            f"📋 監控設定: "
            f"Rack Check={config.RACK_CHECK_INTERVAL}s, "
            f"PLC Monitor={config.PLC_MONITOR_INTERVAL}s, "
            f"Cleanup={config.AUTO_CLEANUP_INTERVAL}s, "
            f"Feedback Update={config.FEEDBACK_UPDATE_INTERVAL}s"
        )

        # 顯示傳送箱配置
        for transfer_box in self.transfer_box_manager.get_all_transfer_boxs():
            self.get_logger().info(
                f"📦 {transfer_box['name']}: "
                f"Location={transfer_box['location_id']}, "
                f"WorkIDs={transfer_box['work_ids']}, "
                f"DM Write={transfer_box['dm_write_start']}, "
                f"DM Feedback={transfer_box['dm_feedback_start']}"
            )

    def check_racks_and_write_plc_callback(self):
        """
        Timer 1 回調: 遍歷所有傳送箱並寫入 PLC

        流程:
        1. 遍歷所有傳送箱配置
        2. 查詢對應的 Rack
        3. 解析 carrier_bitmap
        4. 檢查是否有料且無 Task
        5. 若條件滿足 → 寫入對應的 DM
        """
        try:
            for transfer_box in self.transfer_box_manager.get_all_transfer_boxs():
                self._check_and_write_single_transfer_box(transfer_box)

        except Exception as e:
            self.get_logger().error(f"❌ 檢查 Racks 並寫入 PLC 失敗: {e}")

    def _check_and_write_single_transfer_box(self, transfer_box: dict):
        """
        檢查單個傳送箱並寫入 PLC（從資料庫讀取 Rack 資訊）

        Args:
            transfer_box: 傳送箱配置字典
        """
        try:
            # 1. 從資料庫查詢 Rack
            rack = self.db_helper.get_rack_by_location(transfer_box["location_id"])

            if not rack:
                self.get_logger().debug(
                    f"{transfer_box['name']} Location {transfer_box['location_id']} "
                    f"沒有 Rack，跳過"
                )
                return

            # 2. 從資料庫 Rack 解析 carrier_bitmap
            a_side, b_side = self._parse_carrier_bitmap(rack.carrier_bitmap)

            # 3. 檢查是否有料
            has_material = (a_side > 0) or (b_side > 0)

            # 4. 根據傳送箱類型判斷是否寫入
            transfer_box_type = transfer_box.get("type", "entrance")

            if transfer_box_type == "entrance":
                # 入口傳送箱：有料才寫入
                if not has_material:
                    self.get_logger().debug(
                        f"{transfer_box['name']} (入口) Rack {rack.id} 無料，跳過"
                    )
                    return
            elif transfer_box_type == "exit":
                # 出口傳送箱：無料才寫入
                if has_material:
                    self.get_logger().debug(
                        f"{transfer_box['name']} (出口) Rack {rack.id} 有料，跳過"
                    )
                    return

            # 5. 檢查 task 表是否已有對應任務
            has_task = self.db_helper.check_any_cargo_task_exists(transfer_box["work_ids"])

            if has_task:
                self.get_logger().debug(
                    f"{transfer_box['name']} 已有 Task，停止寫入 PLC"
                )
                return

            # 6. 檢查 rack.is_carry 是否為 0
            if rack.is_carry != 0:
                self.get_logger().debug(
                    f"{transfer_box['name']} Rack {rack.id} is_carry={rack.is_carry}，"
                    f"不為 0，停止寫入 PLC"
                )
                return

            # 7. 解析 carrier_enable_bitmap
            a_enable, b_enable = self._parse_carrier_bitmap(rack.carrier_enable_bitmap)

            # 8. 轉換 direction（根據傳送箱類型）
            direction_converted = self._convert_direction_value(rack.direction, transfer_box_type)

            # 9. 寫入 PLC DM (包含 carrier_bitmap, enable_bitmap, direction)
            self._write_rack_info_to_plc(
                a_side, b_side, a_enable, b_enable, direction_converted,
                rack.id, transfer_box["dm_write_start"], transfer_box["name"]
            )

        except Exception as e:
            self.get_logger().error(
                f"❌ {transfer_box['name']} 檢查或寫入失敗: {e}"
            )

    def _convert_direction_value(self, direction: int, transfer_box_type: str) -> int:
        """
        轉換資料庫 direction 值為寫入 PLC 的確認值

        Args:
            direction: 資料庫 Rack direction
            transfer_box_type: 傳送箱類型 ("entrance" 或 "exit")

        Returns:
            confirm_value: 寫入 PLC 的值

        轉換規則：
            入口 (entrance):
                direction > 0 → 1
                direction < 0 → 2
                direction = 0 → 0

            出口 (exit):
                direction < 0 → 1
                direction > 0 → 2
                direction = 0 → 0
        """
        try:
            if direction == 0:
                self.get_logger().debug(f"🔄 Direction轉換: direction=0, type={transfer_box_type} → 0")
                return 0

            if transfer_box_type == "entrance":
                # 入口：direction>0→1, direction<0→2
                result = 1 if direction > 0 else 2
                self.get_logger().info(
                    f"🔄 入口Direction轉換: direction={direction} → {result}"
                )
                return result
            elif transfer_box_type == "exit":
                # 出口：direction<0→1, direction>0→2
                result = 1 if direction < 0 else 2
                self.get_logger().info(
                    f"🔄 出口Direction轉換: direction={direction} → {result}"
                )
                return result
            else:
                # 預設使用入口邏輯
                result = 1 if direction > 0 else 2
                self.get_logger().warn(
                    f"⚠️ 未知type={transfer_box_type}, direction={direction} → {result}"
                )
                return result

        except Exception as e:
            self.get_logger().error(f"❌ 轉換 direction 失敗: {e}")
            return 0

    def _write_rack_info_to_plc(
        self, a_side: int, b_side: int, a_enable: int, b_enable: int,
        direction_value: int, rack_id: int, dm_start: str, transfer_box_name: str
    ):
        """
        寫入完整 Rack 資訊到 PLC DM（異步）

        寫入格式：
        DM[0~1]: carrier_bitmap (32-bit, 小端序)
        DM[2~3]: carrier_enable_bitmap (32-bit, 小端序)
        DM[4]: direction 確認值 (16-bit)

        Args:
            a_side: A面 carrier_bitmap (16-bit)
            b_side: B面 carrier_bitmap (16-bit)
            a_enable: A面 carrier_enable_bitmap (16-bit)
            b_enable: B面 carrier_enable_bitmap (16-bit)
            direction_value: Direction 確認值 (1 或 2)
            rack_id: Rack ID
            dm_start: DM 起始位址
            transfer_box_name: 傳送箱名稱
        """
        try:
            # PLC 使用小端序（低位在前），需要反轉 A面/B面順序
            values = [
                str(b_side),     # DM[0] = B面 carrier_bitmap（低16位）
                str(a_side),     # DM[1] = A面 carrier_bitmap（高16位）
                str(b_enable),   # DM[2] = B面 enable_bitmap（低16位）
                str(a_enable),   # DM[3] = A面 enable_bitmap（高16位）
                str(direction_value)  # DM[4] = direction 確認值
            ]

            self.plc_client.async_write_continuous_data(
                device_type="DM",
                start_address=dm_start,
                values=values,
                callback=lambda response: self._handle_write_rack_response(
                    response, a_side, b_side, a_enable, b_enable,
                    direction_value, rack_id, transfer_box_name
                )
            )

        except Exception as e:
            self.get_logger().error(
                f"❌ {transfer_box_name} 寫入 PLC 失敗: {e}"
            )

    def _handle_write_rack_response(
        self, response, a_side: int, b_side: int, a_enable: int, b_enable: int,
        direction_value: int, rack_id: int, transfer_box_name: str
    ):
        """處理 Rack 資訊寫入 PLC 的回應"""
        try:
            if response and response.success:
                self.get_logger().info(
                    f"✅ {transfer_box_name} 寫入 PLC 成功: "
                    f"Rack ID={rack_id}, "
                    f"Carrier=[A={a_side:#06x}, B={b_side:#06x}], "
                    f"Enable=[A={a_enable:#06x}, B={b_enable:#06x}], "
                    f"Direction={direction_value}"
                )
            else:
                self.get_logger().error(
                    f"❌ {transfer_box_name} 寫入 PLC 失敗 (Rack ID={rack_id})"
                )

        except Exception as e:
            self.get_logger().error(f"❌ 處理 PLC 寫入回應失敗: {e}")

    def _parse_carrier_bitmap(self, bitmap_hex: str) -> tuple:
        """
        解析 carrier_bitmap 為 A面/B面

        Args:
            bitmap_hex: 8位16進制字串 (例如 "FFFF0000")

        Returns:
            (a_side, b_side): A面和B面的 16-bit 整數
        """
        try:
            bitmap_hex = bitmap_hex.replace("0x", "").replace("0X", "").upper()
            bitmap_hex = bitmap_hex.zfill(8)
            full_value = int(bitmap_hex, 16)

            b_side = full_value & 0xFFFF
            a_side = (full_value >> 16) & 0xFFFF

            return a_side, b_side

        except Exception as e:
            self.get_logger().error(f"❌ 解析 carrier_bitmap 失敗: {e}")
            return 0, 0

    def check_plc_dm_callback(self):
        """
        Timer 2 回調: 統一監控 PLC DM3010-3011 並建立 Task

        流程:
        1. 讀取 DM3010-3011 (32-bit work_id)
        2. 邊緣觸發檢測
        3. 若 work_id 改變且 > 0 → 根據 work_id 查找傳送箱配置
        4. 建立對應的 Task
        """
        try:
            self.plc_client.async_read_continuous_data(
                device_type="DM",
                start_address=config.DM_READ_WORK_ID_START,
                count=config.DM_READ_WORK_ID_COUNT,
                callback=self._handle_plc_response
            )

        except Exception as e:
            self.get_logger().error(f"❌ 讀取 PLC DM 失敗: {e}")

    def _handle_plc_response(self, response):
        """處理 PLC 讀取回應"""
        try:
            if not response or not response.success:
                self.get_logger().debug("PLC 讀取失敗或無資料")
                return

            values = [int(v) for v in response.values]

            if len(values) < 2:
                self.get_logger().error(f"❌ PLC 返回值不足: {len(values)} < 2")
                return

            # 組合 32-bit work_id
            current_work_id = self._combine_32bit(values[0], values[1])

            # 邊緣觸發檢測
            if current_work_id != self.last_work_id:
                if current_work_id > 0:
                    # 根據 work_id 查找傳送箱配置
                    transfer_box = self.transfer_box_manager.get_transfer_box_by_work_id(
                        current_work_id
                    )

                    if transfer_box:
                        self.get_logger().info(
                            f"🔔 PLC work_id 變化: {self.last_work_id} → {current_work_id} "
                            f"({transfer_box['name']})"
                        )
                        self._process_transfer_box_task(current_work_id, transfer_box)
                    else:
                        self.get_logger().warning(
                            f"⚠️ work_id {current_work_id} 無對應的傳送箱配置"
                        )

                self.last_work_id = current_work_id

        except Exception as e:
            self.get_logger().error(f"❌ 處理 PLC 回應失敗: {e}")

    def _combine_32bit(self, low_word: int, high_word: int) -> int:
        """組合 32-bit 整數"""
        return low_word + (high_word << 16)

    def _process_transfer_box_task(self, work_id: int, transfer_box: dict):
        """
        處理傳送箱任務建立

        Args:
            work_id: Work ID
            transfer_box: 傳送箱配置字典
        """
        try:
            # 1. 檢查 work_id 是否存在
            work = self.db_helper.get_work_by_id(work_id)
            if not work:
                self.get_logger().error(f"❌ Work ID {work_id} 不存在")
                return

            # 2. 提取 room_id
            room_id = self._extract_room_id(work_id)

            # 3. 查詢對應的 Rack
            rack = self.db_helper.get_rack_by_location(transfer_box["location_id"])

            # 4. 檢查重複
            rack_id = rack.id if rack else None
            if self.db_helper.check_duplicate_task(work_id, room_id, rack_id):
                self.get_logger().warning(
                    f"⚠️ {transfer_box['name']} Work {work_id} 已有未完成的 Task"
                )
                return

            # 5. 建立 Task
            task = self.db_helper.create_task(
                work_id=work_id,
                room_id=room_id,
                rack_id=rack_id,
                agv_type=config.AGV_TYPE_CARGO,
                work=work,
                rack=rack,
                work_name=work.name if work else "",
                status_id=config.DEFAULT_STATUS_ID,
                priority=config.DEFAULT_PRIORITY,
                agv_id=None
            )

            if task:
                self.get_logger().info(
                    f"✅ {transfer_box['name']} Task 建立成功: "
                    f"Task ID={task.id}, Work ID={work_id}, "
                    f"Room ID={room_id}, Rack ID={rack_id}"
                )

        except Exception as e:
            self.get_logger().error(
                f"❌ {transfer_box['name']} 處理任務失敗: {e}"
            )

    def _extract_room_id(self, work_id: int) -> int:
        """從 work_id 提取 room_id（取第一位數字）"""
        return int(str(work_id)[0])

    def cleanup_completed_tasks_callback(self):
        """Timer 3 回調: 清理已完成/已取消的 Task"""
        try:
            deleted_count = self.db_helper.delete_completed_tasks(
                config.CLEANUP_STATUS_IDS
            )

            if deleted_count > 0:
                self.get_logger().info(f"🗑️ 已清理 {deleted_count} 個已完成的 Task")

        except Exception as e:
            self.get_logger().error(f"❌ 清理已完成 Task 失敗: {e}")

    def update_racks_from_plc_callback(self):
        """
        Timer 4 回調: 遍歷所有傳送箱讀取 PLC 回饋並更新 Rack

        流程:
        1. 遍歷所有傳送箱配置
        2. 總是讀取對應的 DM_FEEDBACK（不檢查任務）
        3. 比較新舊 carrier_bitmap
        4. 只有值變化時才更新資料庫
        """
        try:
            for transfer_box in self.transfer_box_manager.get_all_transfer_boxs():
                self._update_single_transfer_box_from_plc(transfer_box)

        except Exception as e:
            self.get_logger().error(f"❌ 讀取 PLC 回饋失敗: {e}")

    def _update_single_transfer_box_from_plc(self, transfer_box: dict):
        """
        更新單個傳送箱的 Rack（從 PLC 回饋）

        總是讀取 PLC，當值有變化時才更新資料庫

        Args:
            transfer_box: 傳送箱配置字典
        """
        try:
            # 異步讀取 PLC DM 回饋（總是讀取，不檢查任務）
            self.plc_client.async_read_continuous_data(
                device_type="DM",
                start_address=transfer_box["dm_feedback_start"],
                count=config.DM_FEEDBACK_COUNT,
                callback=lambda response: self._handle_feedback_response(
                    response, transfer_box
                )
            )

        except Exception as e:
            self.get_logger().error(
                f"❌ {transfer_box['name']} 讀取回饋失敗: {e}"
            )

    def _handle_feedback_response(self, response, transfer_box: dict):
        """
        處理 PLC 回饋

        只有當 PLC 回饋值與資料庫不同時才更新

        Args:
            response: ReadContinuousData.Response
            transfer_box: 傳送箱配置字典
        """
        try:
            if not response or not response.success:
                self.get_logger().debug(
                    f"{transfer_box['name']} PLC 回饋讀取失敗"
                )
                return

            values = [int(v) for v in response.values]

            if len(values) < 2:
                self.get_logger().error(
                    f"❌ {transfer_box['name']} PLC 回饋值不足: {len(values)} < 2"
                )
                return

            # PLC 使用小端序（低位在前），需要反轉順序
            # DM[0]=B面（低16位）, DM[1]=A面（高16位）
            b_side = values[0]
            a_side = values[1]

            # 組合為 8位16進制字串
            new_carrier_bitmap = self._combine_to_bitmap_hex(a_side, b_side)

            # 1. 從資料庫讀取當前 Rack
            rack = self.db_helper.get_rack_by_location(transfer_box["location_id"])

            if not rack:
                self.get_logger().debug(
                    f"{transfer_box['name']} Location {transfer_box['location_id']} "
                    f"沒有 Rack，跳過"
                )
                return

            # 2. 比較新舊值
            current_carrier_bitmap = rack.carrier_bitmap if rack.carrier_bitmap else "00000000"

            if current_carrier_bitmap == new_carrier_bitmap:
                self.get_logger().debug(
                    f"{transfer_box['name']} PLC 回饋值未變化: {current_carrier_bitmap}"
                )
                return

            # 3. 只有變化時才更新資料庫
            success = self.db_helper.update_rack_carrier_bitmap(
                location_id=transfer_box["location_id"],
                carrier_bitmap=new_carrier_bitmap
            )

            if success:
                self.get_logger().info(
                    f"📥 {transfer_box['name']} PLC 回饋在席值已更新: "
                    f"{current_carrier_bitmap} → {new_carrier_bitmap} "
                    f"[A={a_side:#06x}, B={b_side:#06x}]"
                )

        except Exception as e:
            self.get_logger().error(
                f"❌ {transfer_box['name']} 處理 PLC 回饋失敗: {e}"
            )

    def _combine_to_bitmap_hex(self, a_side: int, b_side: int) -> str:
        """組合 A面/B面為 8位16進制字串"""
        try:
            full_value = (a_side << 16) | b_side
            bitmap_hex = f"{full_value:08X}"
            return bitmap_hex

        except Exception as e:
            self.get_logger().error(f"❌ 組合 carrier_bitmap 失敗: {e}")
            return "00000000"

    def destroy_node(self):
        """節點關閉時清理資源"""
        self.get_logger().info("🛑 TransferBoxTaskBuildNode 正在關閉...")
        self.db_helper.shutdown()
        super().destroy_node()


def main(args=None):
    """主函數"""
    rclpy.init(args=args)

    node = None
    try:
        node = TransferBoxTaskBuildNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().error(f"❌ 節點執行錯誤: {e}")
    finally:
        if node:
            try:
                node.destroy_node()
            except Exception as e:
                print(f"⚠️ 節點銷毀時發生錯誤: {e}")

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"⚠️ rclpy shutdown 時發生錯誤: {e}")


if __name__ == '__main__':
    main()
