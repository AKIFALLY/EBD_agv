from agv_base.states.state import State
from rclpy.node import Node
from loader_agv.robot_context import RobotContext  # 新增的匯入


class IdleState(State):
    # 位置類型常數
    TRANSFER = "01"
    CLEANER = "03"
    SOAKER = "04"
    PRE_DRYER = "05"

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
        # take_transfer: 1個工作站 (01)
        self.take_transfer_start = int(self.room_id_str + self.TRANSFER + "01" + self.TAKE)
        self.take_transfer_end = int(self.room_id_str + self.TRANSFER + "01" + self.TAKE)

        # take_soaker: 6個工作站 (01-06)
        self.take_soaker_start = int(self.room_id_str + self.SOAKER + "01" + self.TAKE)
        self.take_soaker_end = int(self.room_id_str + self.SOAKER + "06" + self.TAKE)

        # take_cleaner: 2個工作站 (01-02)
        self.take_cleaner_start = int(self.room_id_str + self.CLEANER + "01" + self.TAKE)
        self.take_cleaner_end = int(self.room_id_str + self.CLEANER + "02" + self.TAKE)

        # take_pre_dryer: 8個工作站 (01-08)
        self.take_pre_dryer_start = int(self.room_id_str + self.PRE_DRYER + "01" + self.TAKE)
        self.take_pre_dryer_end = int(self.room_id_str + self.PRE_DRYER + "08" + self.TAKE)

        # put_soaker: 6個工作站 (01-06)
        self.put_soaker_start = int(self.room_id_str + self.SOAKER + "01" + self.PUT)
        self.put_soaker_end = int(self.room_id_str + self.SOAKER + "06" + self.PUT)

        # put_cleaner: 2個工作站 (01-02)
        self.put_cleaner_start = int(self.room_id_str + self.CLEANER + "01" + self.PUT)
        self.put_cleaner_end = int(self.room_id_str + self.CLEANER + "02" + self.PUT)

        # put_pre_dryer: 8個工作站 (01-08)
        self.put_pre_dryer_start = int(self.room_id_str + self.PRE_DRYER + "01" + self.PUT)
        self.put_pre_dryer_end = int(self.room_id_str + self.PRE_DRYER + "08" + self.PUT)

    def enter(self):
        self.node.get_logger().info("robot 目前狀態: Idle")

    def leave(self):
        self.node.get_logger().info("robot 離開 Idle 狀態")

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
        self.node.get_logger().info("robot Idle 狀態")

        # 執行 Hokuyo 參數初始化
        self._initialize_hokuyo_parameters()

        # 只有在 Hokuyo 參數初始化完成後，才進行工作 ID 檢查和狀態切換
        if self.hokuyo_write_completed:
            self.node.get_logger().info("✅ Hokuyo 初始化完成，開始檢查工作 ID")

            # 簡化的 work_id 取得方式
            work_id = self.node.work_id
            self.node.get_logger().info(f"檢查工作 ID: {work_id}")

            # 使用預計算的範圍變數進行流程判斷
            if self.take_transfer_start <= work_id <= self.take_transfer_end:
                self.node.get_logger().info(f"切換到 TAKE_TRANSFER 流程 (work_id: {work_id})")
                from loader_agv.robot_states.take_transfer.transfer_vision_position_state import TransferVisionPositionState
                context.set_state(TransferVisionPositionState(self.node))
            elif self.take_soaker_start <= work_id <= self.take_soaker_end:
                self.node.get_logger().info(f"切換到 TAKE_SOAKER 流程 (work_id: {work_id})")
                from loader_agv.robot_states.take_soaker.soaker_vision_position_state import SoakerVisionPositionState
                context.set_state(SoakerVisionPositionState(self.node))
            elif self.take_cleaner_start <= work_id <= self.take_cleaner_end:
                self.node.get_logger().info(f"切換到 TAKE_CLEANER 流程 (work_id: {work_id})")
                from loader_agv.robot_states.take_cleaner.cleaner_vision_position_state import CleanerVisionPositionState
                context.set_state(CleanerVisionPositionState(self.node))
            elif self.take_pre_dryer_start <= work_id <= self.take_pre_dryer_end:
                self.node.get_logger().info(f"切換到 TAKE_PRE_DRYER 流程 (work_id: {work_id})")
                from loader_agv.robot_states.take_pre_dryer.pre_dryer_vision_position_state import PreDryerVisionPositionState
                context.set_state(PreDryerVisionPositionState(self.node))
            elif self.put_soaker_start <= work_id <= self.put_soaker_end:
                self.node.get_logger().info(f"切換到 PUT_SOAKER 流程 (work_id: {work_id})")
                from loader_agv.robot_states.put_soaker.soaker_vision_position_state import SoakerVisionPositionState
                context.set_state(SoakerVisionPositionState(self.node))
            elif self.put_cleaner_start <= work_id <= self.put_cleaner_end:
                self.node.get_logger().info(f"切換到 PUT_CLEANER 流程 (work_id: {work_id})")
                from loader_agv.robot_states.put_cleaner.cleaner_vision_position_state import CleanerVisionPositionState
                context.set_state(CleanerVisionPositionState(self.node))
            elif self.put_pre_dryer_start <= work_id <= self.put_pre_dryer_end:
                self.node.get_logger().info(f"切換到 PUT_PRE_DRYER 流程 (work_id: {work_id})")
                from loader_agv.robot_states.put_pre_dryer.pre_dryer_vision_position_state import PreDryerVisionPositionState
                context.set_state(PreDryerVisionPositionState(self.node))
            else:
                self.node.get_logger().warn(f"未知的工作 ID: {work_id}")
                # 提供詳細的範圍資訊以便調試
                self.node.get_logger().debug(f"工作 ID 範圍資訊:")
                self.node.get_logger().debug(
                    f"  TAKE_TRANSFER: {self.take_transfer_start}-{self.take_transfer_end}")
                self.node.get_logger().debug(
                    f"  TAKE_SOAKER: {self.take_soaker_start}-{self.take_soaker_end}")
                self.node.get_logger().debug(
                    f"  TAKE_CLEANER: {self.take_cleaner_start}-{self.take_cleaner_end}")
                self.node.get_logger().debug(
                    f"  TAKE_PRE_DRYER: {self.take_pre_dryer_start}-{self.take_pre_dryer_end}")
                self.node.get_logger().debug(
                    f"  PUT_SOAKER: {self.put_soaker_start}-{self.put_soaker_end}")
                self.node.get_logger().debug(
                    f"  PUT_CLEANER: {self.put_cleaner_start}-{self.put_cleaner_end}")
                self.node.get_logger().debug(
                    f"  PUT_PRE_DRYER: {self.put_pre_dryer_start}-{self.put_pre_dryer_end}")
        else:
            self.node.get_logger().debug("⏳ 等待 Hokuyo 參數初始化完成...")
