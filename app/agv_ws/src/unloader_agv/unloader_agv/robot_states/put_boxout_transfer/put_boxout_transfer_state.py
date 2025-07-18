from db_proxy_interfaces.msg import Carrier as CarrierMsg
from rclpy.node import Node
from unloader_agv.robot_context import RobotContext
from agv_base.robot import Robot
from agv_base.hokuyo_dms_8bit import HokuyoDMS8Bit
from db_proxy.agvc_database_client import AGVCDatabaseClient
from unloader_agv.robot_states.base_robot_state import BaseRobotState


class PutBoxoutTransferState(BaseRobotState):
    def __init__(self, node: Node):
        super().__init__(node)
        self.port_id_address = self.node.room_id * 1000 + 20
        self.step = RobotContext.IDLE
        self.agvc_client = AGVCDatabaseClient(self.node)
        self.update_carrier_success = False
        self.update_carrier_min_success = False  # 第一個 carrier 更新狀態
        self.update_carrier_max_success = False  # 第二個 carrier 更新狀態
        self.carrier_updates_pending = 0  # 待更新的 carrier 數量
        self.sent = False

    def enter(self):
        self.node.get_logger().info("Unloader Robot Put BoxoutTransfer 目前狀態: PutBoxoutTransfer")
        self.update_carrier_success = False
        self.update_carrier_min_success = False
        self.update_carrier_max_success = False
        self.carrier_updates_pending = 0
        self.sent = False

    def leave(self):
        self.node.get_logger().info("Unloader Robot Put BoxoutTransfer 離開 PutBoxoutTransfer 狀態")
        self.update_carrier_success = False
        self.update_carrier_min_success = False
        self.update_carrier_max_success = False
        self.carrier_updates_pending = 0
        self.sent = False

    def update_carrier_database(self, context: RobotContext):
        """更新雙 carrier 資料庫"""
        self.carrier_updates_pending = 0
        self.update_carrier_min_success = False
        self.update_carrier_max_success = False

        # 檢查並更新第一個 carrier (carrier_id_min)
        if context.carrier_id[0] is not None:
            carrier_min = CarrierMsg()
            carrier_min.id = context.carrier_id[0]
            carrier_min.room_id = self.node.room_id
            carrier_min.rack_id = 0
            carrier_min.port_id = self.port_id_address+context.get_boxout_port  # 使用 boxout port
            carrier_min.rack_index = 0
            carrier_min.status_id = Robot.CARRIER_STATUS_ENTER_BOXOUT_TRANSFER  # 進入出入口傳送箱

            self.agvc_client.async_update_carrier(
                carrier_min, lambda result: self.update_carrier_min_callback(result))
            self.carrier_updates_pending += 1
            self.node.get_logger().info(f"🔄 開始更新第一個 Carrier: {context.carrier_id[0]}")
        else:
            self.update_carrier_min_success = True  # 沒有需要更新的，視為成功

        # 檢查並更新第二個 carrier (carrier_id_max)
        if context.carrier_id[1] is not None:
            carrier_max = CarrierMsg()
            carrier_max.id = context.carrier_id[1]
            carrier_max.room_id = self.node.room_id
            carrier_max.rack_id = 0
            carrier_max.port_id = self.port_id_address+context.get_boxout_port + 1  # 第二個 port
            carrier_max.rack_index = 0
            carrier_max.status_id = Robot.CARRIER_STATUS_ENTER_BOXOUT_TRANSFER  # 進入出入口傳送箱

            self.agvc_client.async_update_carrier(
                carrier_max, lambda result: self.update_carrier_max_callback(result))
            self.carrier_updates_pending += 1
            self.node.get_logger().info(f"🔄 開始更新第二個 Carrier: {context.carrier_id[1]}")
        else:
            self.update_carrier_max_success = True  # 沒有需要更新的，視為成功

        # 如果沒有需要更新的 carrier，直接設為成功
        if self.carrier_updates_pending == 0:
            self.update_carrier_success = True
            self.node.get_logger().info("ℹ️ 沒有需要更新的 Carrier")

    def update_carrier_min_callback(self, result):
        """第一個 carrier 更新回調"""
        if result is not None and result.success:
            self.node.get_logger().info(f"✅ 第一個 Carrier 更新成功: {result.message}")
            self.update_carrier_min_success = True
        else:
            error_msg = result.message if result is not None else "無回應"
            self.node.get_logger().error(f"❌ 第一個 Carrier 更新失敗: {error_msg}")
            self.update_carrier_min_success = False

        self._check_all_carriers_updated()

    def update_carrier_max_callback(self, result):
        """第二個 carrier 更新回調"""
        if result is not None and result.success:
            self.node.get_logger().info(f"✅ 第二個 Carrier 更新成功: {result.message}")
            self.update_carrier_max_success = True
        else:
            error_msg = result.message if result is not None else "無回應"
            self.node.get_logger().error(f"❌ 第二個 Carrier 更新失敗: {error_msg}")
            self.update_carrier_max_success = False

        self._check_all_carriers_updated()

    def _check_all_carriers_updated(self):
        """檢查所有 carrier 是否都已更新完成"""
        if self.update_carrier_min_success and self.update_carrier_max_success:
            self.update_carrier_success = True
            self.node.get_logger().info("✅ 所有 Carrier 更新完成")

    def handle(self, context: RobotContext):
        self.node.get_logger().info("Unloader Robot Put BoxoutTransfer PutBoxoutTransfer 狀態")

        # 並行執行：Hokuyo write_busy 設定
        self._set_hokuyo_busy()

        # 並行執行：其他操作（不需等待 Hokuyo 完成）
        PUT_BOXOUT_TRANSFER_PGNO = context.robot.ACTION_TO + \
            context.robot.NONE_POSITION + context.robot.BOX_OUT_POSITION
        read_pgno = context.robot.read_pgno_response
        context.robot.read_robot_status()

        # 更新 Hokuyo Input - 使用統一方法
        self._handle_hokuyo_input()

        # 條件執行：只有機器人邏輯需要等待 Hokuyo 完成
        if self.hokuyo_busy_write_completed:
            self._execute_robot_logic(context, PUT_BOXOUT_TRANSFER_PGNO, read_pgno)

    def _execute_robot_logic(self, context: RobotContext, PUT_BOXOUT_TRANSFER_PGNO, read_pgno):
        """執行機器人邏輯"""
        match self.step:
            case RobotContext.IDLE:
                self.node.get_logger().info("Unloader Robot Put BoxoutTransfer PUT BOXOUT TRANSFER IDLE")
                self.step = RobotContext.CHECK_IDLE
            case RobotContext.CHECK_IDLE:
                self.node.get_logger().info("Unloader Robot Put BoxoutTransfer PUT BOXOUT TRANSFER CHECK_IDLE")
                if read_pgno is None:
                    self.node.get_logger().debug("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅Robot狀態為IDLE")
                    self.step = RobotContext.WRITE_CHG_PARAMTER
                else:
                    self.node.get_logger().info("❌Robot狀態不為IDLE")

            case RobotContext.WRITE_CHG_PARAMTER:
                if not self.sent:
                    context.update_port_parameters()
                    self.sent = True
                if context.robot.update_parameter_success:
                    self.node.get_logger().info("✅更新參數成功")
                    self.sent = False
                    context.robot.update_parameter_success = False
                    self.step = RobotContext.WRITE_CHG_PARA
                elif context.robot.update_parameter_failed:
                    self.node.get_logger().info("❌更新參數失敗")
                    self.sent = False
                    context.robot.update_parameter_failed = False
                else:
                    self.node.get_logger().info("🕒更新參數中")

            case RobotContext.WRITE_CHG_PARA:
                self.node.get_logger().info("Unloader Robot Put BoxoutTransfer PUT BOXOUT TRANSFER WRITE CHG PARA")
                if not self.sent:
                    context.robot.update_pgno(Robot.CHG_PARA)
                    self.sent = True
                if context.robot.update_pgno_success:
                    self.node.get_logger().info("✅傳送預執行成功")
                    self.sent = False
                    context.robot.update_pgno_success = False
                    self.step = RobotContext.CHECK_CHG_PARA

                elif context.robot.update_pgno_failed:
                    self.node.get_logger().info("❌傳送預執行失敗")
                    self.sent = False
                    context.robot.update_pgno_failed = False
                else:
                    self.node.get_logger().info("🕒傳送預執行中")

            case RobotContext.CHECK_CHG_PARA:
                self.node.get_logger().info("Unloader Robot Put BoxoutTransfer PUT BOXOUT TRANSFER CHECK CHG PARA")
                if read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("✅讀取預執行成功")
                    self.step = RobotContext.WRITE_PGNO
                elif read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("🕒讀取預執行中...")
                else:
                    self.node.get_logger().info("❌讀取預執行失敗")

            case RobotContext.WRITE_PGNO:
                self.node.get_logger().info("Unloader Robot Put BoxoutTransfer PUT BOXOUT TRANSFER WRITE_PGNO")
                if not self.sent:
                    context.robot.update_pgno(PUT_BOXOUT_TRANSFER_PGNO)
                    self.sent = True
                if context.robot.update_pgno_success:
                    self.node.get_logger().info("✅傳送PGNO成功")
                    self.sent = False
                    context.robot.update_pgno_success = False
                    self.step = RobotContext.CHECK_PGNO
                elif context.robot.update_pgno_failed:
                    self.node.get_logger().info("❌傳送PGNO失敗")
                    self.sent = False
                    context.robot.update_pgno_failed = False
                else:
                    self.node.get_logger().info("🕒傳送PGNO中...")

            case RobotContext.CHECK_PGNO:
                self.node.get_logger().info("Unloader Robot Put BoxoutTransfer PUT BOXOUT TRANSFER CHECK_PGNO")
                if read_pgno is None:
                    self.node.get_logger().debug("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == (PUT_BOXOUT_TRANSFER_PGNO):
                    self.node.get_logger().info("✅讀取PGNO成功")
                    self.step = RobotContext.ACTING
                elif read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("🕒讀取PGNO中...")
                else:
                    self.node.get_logger().info("❌讀取PGNO失敗")

            case RobotContext.ACTING:
                self.node.get_logger().info("Unloader Robot Put BoxoutTransfer PUT BOXOUT TRANSFER ACTING")
                if read_pgno is None:
                    self.node.get_logger().debug("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == (PUT_BOXOUT_TRANSFER_PGNO):
                    self.node.get_logger().debug("🤖手臂動作中")
                elif read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅手臂動作完成")
                    self.step = RobotContext.FINISH
                else:
                    self.node.get_logger().info("❌手臂動作失敗")
            case RobotContext.FINISH:
                self.node.get_logger().info("Unloader Robot Put BoxoutTransfer PUT BOXOUT TRANSFER Finish")
                if read_pgno is None:
                    self.node.get_logger().debug("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅放出料傳送箱完成")
                    self.step = RobotContext.UPDATE_DATABASE
                else:
                    self.node.get_logger().info("❌放出料傳送箱失敗")
            case RobotContext.UPDATE_DATABASE:
                self.node.get_logger().info("Unloader Robot Put BoxoutTransfer PUT BOXOUT TRANSFER UPDATE_DATABASE")
                if not self.sent:
                    self.update_carrier_database(context)
                    self.sent = True
                elif self.sent and self.update_carrier_success:
                    self.node.get_logger().info("✅更新 Carrier 資料庫成功")
                    self.sent = False

                    # 完成 PUT_BOXOUT_TRANSFER 流程，進入完成狀態
                    self.node.get_logger().info("✅ Put BoxoutTransfer 完成: 進入 CompleteState")
                    from unloader_agv.robot_states.complete_state import CompleteState
                    context.set_state(CompleteState(self.node))

                    self.step = RobotContext.IDLE
