from agv_base.states.state import State
from rclpy.node import Node
from unloader_agv.robot_context import RobotContext  # 新增的匯入


class IdleState(State):
    # 位置類型常數
    BOX_OUT_TRANSFER = "02"
    PRE_DRYER = "05"
    OVEN = "06"

    # 動作類型常數
    TAKE = "01"
    PUT = "02"

    def __init__(self, node: Node):
        super().__init__(node)
        self.node = node

        # Hokuyo 初始化狀態
        self.hokuyo_write_completed = False

        # 動態計算工作 ID 範圍
        self.room_id_str = str(self.node.room_id)

        # 計算各流程的工作 ID 範圍
        # take_pre_dryer: 4個工作站 (01-04)，每個工作站包含2個PORT，總計8個PORT
        self.take_pre_dryer_start = int(self.room_id_str + self.PRE_DRYER + "01" + self.TAKE)
        self.take_pre_dryer_end = int(self.room_id_str + self.PRE_DRYER + "04" + self.TAKE)

        # take_oven: 2個工作站 (01-02)
        self.take_oven_start = int(self.room_id_str + self.OVEN + "01" + self.TAKE)
        self.take_oven_end = int(self.room_id_str + self.OVEN + "02" + self.TAKE)

        # put_boxout_transfer: 2個工作站 (01-02)
        self.put_boxout_transfer_start = int(
            self.room_id_str + self.BOX_OUT_TRANSFER + "01" + self.PUT)
        self.put_boxout_transfer_end = int(
            self.room_id_str + self.BOX_OUT_TRANSFER + "02" + self.PUT)

        # put_oven: 2個工作站 (01-02)
        self.put_oven_start = int(self.room_id_str + self.OVEN + "01" + self.PUT)
        self.put_oven_end = int(self.room_id_str + self.OVEN + "02" + self.PUT)

    def enter(self):
        self.node.get_logger().info("Unloader Robot 目前狀態: Idle")

    def leave(self):
        self.node.get_logger().info("Unloader Robot 離開 Idle 狀態")

    def _initialize_hokuyo_parameters(self):
        """初始化單一 Hokuyo 物件的參數"""
        if not self.hokuyo_write_completed:
            self.node.get_logger().info("🔄 開始 Hokuyo 參數初始化流程")

            # 對單一 Hokuyo 物件進行參數設定
            hokuyo_1 = self.node.hokuyo_dms_8bit_1

            try:
                # 設定 hokuyo_dms_8bit_1 的參數
                hokuyo_1.write_valid("0")
                hokuyo_1.write_tr_req("0")
                hokuyo_1.write_busy("0")
                hokuyo_1.write_complete("0")
                self.node.get_logger().info("✅ Hokuyo_1 參數設定完成: valid=0, tr_req=0, busy=0, complete=0")

                # 標記完成
                self.hokuyo_write_completed = True
                self.node.get_logger().info("✅ Hokuyo 參數初始化完成")

            except Exception as e:
                self.node.get_logger().error(f"❌ Hokuyo 參數設定失敗: {e}")
                # 即使失敗也標記為完成，避免無限重試
                self.hokuyo_write_completed = True

    def handle(self, context: RobotContext):
        self.node.get_logger().info("Unloader Robot Idle 狀態")

        # 執行 Hokuyo 參數初始化
        self._initialize_hokuyo_parameters()

        # 只有在 Hokuyo 參數初始化完成後，才進行任務處理和狀態切換
        if self.hokuyo_write_completed:
            self.node.get_logger().info("✅ Hokuyo 初始化完成，開始處理任務")

            # 檢查是否有任務
            if not hasattr(self.node, 'task') or self.node.task is None:
                self.node.get_logger().debug("等待任務分配...")
                return

            # 解析 work_id
            work_id = self.node.work_id
            self.node.get_logger().info(f"處理工作 ID: {work_id}")

            # 根據 work_id 範圍決定流程
            if self.take_pre_dryer_start <= work_id <= self.take_pre_dryer_end:
                self.node.get_logger().info(f"切換到 TAKE_PRE_DRYER 流程 (work_id: {work_id})")
                from unloader_agv.robot_states.take_pre_dryer.pre_dryer_vision_position_state import PreDryerVisionPositionState
                context.set_state(PreDryerVisionPositionState(self.node))
            elif self.take_oven_start <= work_id <= self.take_oven_end:
                self.node.get_logger().info(f"切換到 TAKE_OVEN 流程 (work_id: {work_id})")
                from unloader_agv.robot_states.take_oven.oven_vision_position_state import OvenVisionPositionState
                context.set_state(OvenVisionPositionState(self.node))
            elif self.put_boxout_transfer_start <= work_id <= self.put_boxout_transfer_end:
                self.node.get_logger().info(f"切換到 PUT_BOXOUT_TRANSFER 流程 (work_id: {work_id})")
                from unloader_agv.robot_states.put_boxout_transfer.boxout_transfer_vision_position_state import BoxoutTransferVisionPositionState
                context.set_state(BoxoutTransferVisionPositionState(self.node))
            elif self.put_oven_start <= work_id <= self.put_oven_end:
                self.node.get_logger().info(f"切換到 PUT_OVEN 流程 (work_id: {work_id})")
                from unloader_agv.robot_states.put_oven.oven_vision_position_state import OvenVisionPositionState
                context.set_state(OvenVisionPositionState(self.node))
            else:
                self.node.get_logger().warn(f"未知的工作 ID: {work_id}")
                # 提供詳細的範圍資訊以便調試
                self.node.get_logger().debug(f"工作 ID 範圍資訊:")
                self.node.get_logger().debug(
                    f"  TAKE_PRE_DRYER: {self.take_pre_dryer_start}-{self.take_pre_dryer_end}")
                self.node.get_logger().debug(
                    f"  TAKE_OVEN: {self.take_oven_start}-{self.take_oven_end}")
                self.node.get_logger().debug(
                    f"  PUT_BOXOUT_TRANSFER: {self.put_boxout_transfer_start}-{self.put_boxout_transfer_end}")
                self.node.get_logger().debug(
                    f"  PUT_OVEN: {self.put_oven_start}-{self.put_oven_end}")

                # 轉換到完成狀態
                from unloader_agv.robot_states.complete_state import CompleteState
                context.set_state(CompleteState(self.node))
        else:
            self.node.get_logger().debug("⏳ 等待 Hokuyo 參數初始化完成...")
