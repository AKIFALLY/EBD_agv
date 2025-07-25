from agv_base.states.state import State
from rclpy.node import Node
from cargo_mover_agv.robot_context import RobotContext
import time


class CompleteState(State):
    def __init__(self, node: Node):
        super().__init__(node)
        self.node = node
        # Hokuyo 寫入相關變數
        self.hokuyo_write_completed = False
        # 延遲重置相關變數
        self.reset_timer_started = False
        self.reset_start_time = None
        self.reset_completed = False

    def enter(self):
        self.node.get_logger().info("🤖Robot 目前狀態: Complete")

    def leave(self):
        self.node.get_logger().info("Robot 離開 Complete 狀態")

    def _reset_hokuyo_parameters(self):
        """同時重置兩個 Hokuyo 物件的參數"""
        if not self.hokuyo_write_completed:
            self.node.get_logger().info("🔄 開始 Hokuyo 參數重置流程")

            # 同時對兩個 Hokuyo 物件進行參數設定
            hokuyo_1 = self.node.hokuyo_dms_8bit_1
            hokuyo_2 = self.node.hokuyo_dms_8bit_2

            try:
                # 並行設定 hokuyo_dms_8bit_1 的參數
                hokuyo_1.write_valid("0")
                hokuyo_1.write_tr_req("0")
                hokuyo_1.write_busy("0")
                hokuyo_1.write_complete("1")
                self.node.get_logger().info("✅ Hokuyo_1 參數設定完成: valid=0, tr_req=0, busy=0, complete=1")

                # 並行設定 hokuyo_dms_8bit_2 的參數
                hokuyo_2.write_valid("0")
                hokuyo_2.write_tr_req("0")
                hokuyo_2.write_busy("0")
                hokuyo_2.write_complete("1")
                self.node.get_logger().info("✅ Hokuyo_2 參數設定完成: valid=0, tr_req=0, busy=0, complete=1")

                # 標記完成並啟動延遲重置計時器
                self.hokuyo_write_completed = True
                self.reset_timer_started = True
                self.reset_start_time = time.time()
                self.node.get_logger().info("✅ 所有 Hokuyo 參數重置完成，5秒後將執行延遲重置")

            except Exception as e:
                self.node.get_logger().error(f"❌ Hokuyo 參數設定失敗: {e}")
                # 即使失敗也標記為完成，避免無限重試
                self.hokuyo_write_completed = True

    def _handle_delayed_reset(self):
        """處理 5 秒延遲後的 Hokuyo 參數重置"""
        if self.reset_timer_started and not self.reset_completed:
            current_time = time.time()
            elapsed_time = current_time - self.reset_start_time

            if elapsed_time >= 5.0:  # 5 秒後執行重置
                self.node.get_logger().info("⏰ 5秒延遲時間到達，開始執行 Hokuyo 參數延遲重置")

                # 同時對兩個 Hokuyo 物件進行延遲重置
                hokuyo_1 = self.node.hokuyo_dms_8bit_1
                hokuyo_2 = self.node.hokuyo_dms_8bit_2

                try:
                    # 並行重置 hokuyo_dms_8bit_1 的參數
                    hokuyo_1.write_valid("0")
                    hokuyo_1.write_complete("0")
                    self.node.get_logger().info("✅ Hokuyo_1 延遲重置完成: valid=0, complete=0")

                    # 並行重置 hokuyo_dms_8bit_2 的參數
                    hokuyo_2.write_valid("0")
                    hokuyo_2.write_complete("0")
                    self.node.get_logger().info("✅ Hokuyo_2 延遲重置完成: valid=0, complete=0")

                    # 標記延遲重置完成
                    self.reset_completed = True
                    self.node.get_logger().info("✅ 所有 Hokuyo 參數延遲重置完成")

                except Exception as e:
                    self.node.get_logger().error(f"❌ Hokuyo 延遲重置失敗: {e}")
                    # 即使失敗也標記為完成，避免無限重試
                    self.reset_completed = True

    def handle(self, context: RobotContext):
        self.node.get_logger().info("Robot Complete 狀態")

        # 執行 Hokuyo 參數重置（同時對兩個 Hokuyo 物件進行設定）
        self._reset_hokuyo_parameters()

        # 處理延遲重置邏輯
        self._handle_delayed_reset()

        # 清理共同的狀態變數
        context.rack_photo_up_or_down_buffer = None

        # 清理 buffer 變數 - 同時處理 entrance 和 exit 的情況
        if hasattr(context, 'boxin_buffer'):
            context.boxin_buffer = None
        if hasattr(context, 'boxout_buffer'):
            context.boxout_buffer = None

        self.node.get_logger().info("✅ Complete 狀態處理完成，所有相關狀態已重置")
