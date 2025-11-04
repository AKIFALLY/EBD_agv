from rclpy.node import Node
from unloader_agv.robot_context import RobotContext
from agv_base.robot import Robot
# 移除不必要的 HokuyoDMS8Bit import - 此 state 不需要直接操作 Hokuyo 設備
from unloader_agv.robot_states.base_robot_state import BaseRobotState


class TakeAgvState(BaseRobotState):

    def __init__(self, node: Node):
        super().__init__(node)
        self.node = node
        self.port_id_address = self.node.room_id * 1000 + 110

    def enter(self):
        self.node.get_logger().info("Unloader Robot Put BoxoutTransfer 目前狀態: TakeAgv")
        self._reset_state()

    def leave(self):
        self.node.get_logger().info("Unloader Robot Put BoxoutTransfer 離開 TakeAgv 狀態")
        self._reset_state()

    def handle(self, context: RobotContext):
        # 添加循環日誌
        if hasattr(context, 'take_put_cycle_count') and hasattr(context, 'take_put_current_batch'):
            cycle_num = context.take_put_cycle_count + 1
            current_agv_ports = context.take_put_current_batch
            self.node.get_logger().info(
                f"Unloader Robot Take AGV (第{cycle_num}次) - "
                f"從 AGV ports {current_agv_ports} 取料")
        else:
            self.node.get_logger().info("Unloader Robot Put BoxoutTransfer TakeAgv 狀態")

        # 並行執行：Hokuyo write_busy 設定
        self._set_hokuyo_busy()

        # 並行執行：其他操作（不需等待 Hokuyo 完成）
        TAKE_UNLOADER_AGV_PGNO = context.robot.ACTION_FROM + \
            context.robot.AGV_POSITION + context.robot.NONE_POSITION
        read_pgno = context.robot.read_pgno_response
        context.robot.read_robot_status()

        # 更新 Hokuyo Input - 使用統一方法
        self._handle_hokuyo_input()

        # 條件執行：只有機器人邏輯需要等待 Hokuyo 完成
        if self.hokuyo_busy_write_completed:
            self._execute_robot_logic(context, TAKE_UNLOADER_AGV_PGNO, read_pgno)

    def _execute_robot_logic(self, context: RobotContext, TAKE_UNLOADER_AGV_PGNO, read_pgno):
        """執行機器人邏輯"""
        match self.step:
            case RobotContext.IDLE:
                self.node.get_logger().info("Unloader Robot Put BoxoutTransfer TAKE UNLOADER AGV IDLE")
                self.step = RobotContext.CHECK_IDLE
            case RobotContext.CHECK_IDLE:
                self.node.get_logger().info("Unloader Robot Put BoxoutTransfer TAKE UNLOADER AGV CHECK_IDLE")
                if read_pgno is None:
                    self.node.get_logger().info("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅Robot狀態為IDLE")
                    self.step = RobotContext.WRITE_CHG_PARAMTER
                else:
                    self.node.get_logger().error("❌Robot狀態不為IDLE")

            case RobotContext.WRITE_CHG_PARAMTER:
                if not self.sent:
                    context.update_port_parameters()
                    self.sent = True
                if context.robot.update_parameter_success:
                    self.node.get_logger().info("✅更新參數成功")
                    self.sent = False
                    context.robot.update_parameter_success = False
                    self.step = RobotContext.CHECK_CHG_PARAMETER
                elif context.robot.update_parameter_failed:
                    self.node.get_logger().info("❌更新參數失敗")
                    self.sent = False
                    context.robot.update_parameter_failed = False
                else:
                    self.node.get_logger().info("🕒更新參數中")

            case RobotContext.CHECK_CHG_PARAMETER:
                self.node.get_logger().info("Unloader Robot Put BoxoutTransfer TAKE AGV CHECK CHG PARAMETER")

                # 構建預期參數字典
                expected_params = {}

                # 檢查 unloader_agv_port_back → W110
                # layer_z = ((port-1) // 2) + 1, layer_y = 0
                layer_z_back = ((context.get_unloader_agv_port_back - 1) // 2) + 1
                layer_y_back = 0
                expected_params['w110'] = (layer_z_back | (layer_y_back << 16))

                self.node.get_logger().info(
                    f"預期檢查: agv_port_back={context.get_unloader_agv_port_back} → "
                    f"W110 (z={layer_z_back}, y={layer_y_back})")

                # 執行檢查
                if self._handle_check_chg_parameter(context, expected_params):
                    self.step = RobotContext.WRITE_CHG_PARA

            case RobotContext.WRITE_CHG_PARA:
                self.node.get_logger().info("Unloader Robot Put BoxoutTransfer TAKE UNLOADER AGV WRITE CHG PARA")
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
                self.node.get_logger().info("Unloader Robot Put BoxoutTransfer TAKE UNLOADER AGV CHECK CHG PARA")
                if read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("✅讀取預執行成功")
                    self.step = RobotContext.WRITE_PGNO
                elif read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("🕒讀取預執行中...")
                else:
                    self.node.get_logger().info("❌讀取預執行失敗")

            case RobotContext.WRITE_PGNO:
                self.node.get_logger().info("Unloader Robot Put BoxoutTransfer TAKE UNLOADER AGV WRITE_PGNO")
                if not self.sent:
                    context.robot.update_pgno(TAKE_UNLOADER_AGV_PGNO)
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
                self.node.get_logger().info("Unloader Robot Put BoxoutTransfer TAKE UNLOADER AGV CHECK_PGNO")
                if read_pgno is None:
                    self.node.get_logger().debug("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == (TAKE_UNLOADER_AGV_PGNO):
                    self.node.get_logger().info("✅讀取PGNO成功")
                    self.step = RobotContext.ACTING
                elif read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("🕒讀取PGNO中...")
                else:
                    self.node.get_logger().error("❌讀取PGNO失敗")

            case RobotContext.ACTING:
                self.node.get_logger().info("Unloader Robot Put BoxoutTransfer TAKE UNLOADER AGV ACTING")
                if read_pgno is None:
                    self.node.get_logger().debug("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == (TAKE_UNLOADER_AGV_PGNO):
                    self.node.get_logger().debug("🤖手臂動作中")
                elif read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅手臂動作完成")
                    self.step = RobotContext.FINISH
                else:
                    self.node.get_logger().error("❌手臂動作失敗")
            case RobotContext.FINISH:
                self.node.get_logger().info("Unloader Robot Put BoxoutTransfer TAKE UNLOADER AGV Finish")
                if read_pgno is None:
                    self.node.get_logger().debug("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅取AGV箱完成")

                    # 直接進入下一個狀態，不需要更新資料庫
                    self.node.get_logger().info("✅ Take AGV 完成: 進入下一個狀態")
                    from unloader_agv.robot_states.put_boxout_transfer.put_boxout_transfer_state import PutBoxoutTransferState
                    context.set_state(PutBoxoutTransferState(self.node))

                    self.step = RobotContext.IDLE
                else:
                    self.node.get_logger().info("❌取AGV箱失敗")
