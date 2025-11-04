from db_proxy_interfaces.msg import Carrier as CarrierMsg
from rclpy.node import Node
from loader_agv.robot_context import RobotContext  # 新增的匯入
from agv_base.robot import Robot
from agv_base.hokuyo_dms_8bit import HokuyoDMS8Bit
from db_proxy.agvc_database_client import AGVCDatabaseClient


from loader_agv.robot_states.base_robot_state import BaseRobotState


class TakeTransferState(BaseRobotState):
    def __init__(self, node: Node):
        super().__init__(node)
        self.step = RobotContext.IDLE
        self.agvc_client = AGVCDatabaseClient(self.node)
        self.update_carrier_success = False
        self.sent = False

    def enter(self):
        self.node.get_logger().info("Robot Take Transfer 目前狀態: TakeTransfer")
        self.update_carrier_success = False
        self.sent = False

    def leave(self):
        self.node.get_logger().info("Robot Take Transfer 離開 TakeTransfer 狀態")
        self.update_carrier_success = False
        self.sent = False

    def handle(self, context: RobotContext):
        # 批量取料 port 映射：根據計數器決定來源 port
        # Station-based 設計：
        # - Station 01: Port [1, 2] → 第1次取 port1, 第2次取 port2
        # - Station 03: Port [3, 4] → 第1次取 port3, 第2次取 port4
        source_port = context.transfer_ports[context.transfer_take_count]
        context.get_boxin_port = source_port

        # 更新當前使用的 carrier_id
        context.carrier_id = context.transfer_carrier_ids[context.transfer_take_count]

        self.node.get_logger().info(
            f"[Station-based 批量] 取料第 {context.transfer_take_count + 1}/2 次 "
            f"(Work ID {context.work_id})")
        self.node.get_logger().info(
            f"來源: 傳送箱 Port {source_port} → 目標: 機械臂, carrier_id={context.carrier_id}")

        # 並行執行：Hokuyo write_busy 設定
        self._set_hokuyo_busy()

        # 並行執行：其他操作（不需等待 Hokuyo 完成）
        TAKE_TRANSFER_PGNO = context.robot.ACTION_FROM + \
            context.robot.BOX_IN_POSITION + context.robot.NONE_POSITION
        read_pgno = context.robot.read_pgno_response
        context.robot.read_robot_status()

        # 更新 Hokuyo Input - 使用統一方法
        self._handle_hokuyo_input()

        # 條件執行：只有機器人邏輯需要等待 Hokuyo 完成
        if self.hokuyo_busy_write_completed:
            self._execute_robot_logic(context, TAKE_TRANSFER_PGNO, read_pgno)

    def _execute_robot_logic(self, context: RobotContext, TAKE_TRANSFER_PGNO, read_pgno):
        """執行機器人邏輯"""

        match self.step:
            case RobotContext.IDLE:
                self.node.get_logger().info("Robot Take Transfer TAKE TRANSFER IDLE")
                self.step = RobotContext.CHECK_IDLE
            case RobotContext.CHECK_IDLE:
                self.node.get_logger().info("Robot Take Transfer TAKE TRANSFER CHECK_IDLE")
                if read_pgno is None:
                    self.node.get_logger().info("⏳等待讀取PGNO回應...")
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
                    self.step = RobotContext.CHECK_CHG_PARAMETER
                elif context.robot.update_parameter_failed:
                    self.node.get_logger().info("❌更新參數失敗")
                    self.sent = False
                    context.robot.update_parameter_failed = False
                else:
                    self.node.get_logger().info("🕒更新參數中")

            case RobotContext.CHECK_CHG_PARAMETER:
                self.node.get_logger().info("Robot Take Transfer CHECK CHG PARAMETER")

                # 導入計算方法
                from loader_agv.robot_states.loader_robot_parameter import LoaderRobotParameter

                # 構建預期參數字典
                expected_params = {}

                # 檢查 boxin_port → W114(layer_z_boxin), W115(layer_y_boxin)
                layer_z_boxin, layer_y_boxin = LoaderRobotParameter.calculate_layer_from_port(
                    context.get_boxin_port
                )
                expected_params['w114'] = layer_z_boxin
                expected_params['w115'] = layer_y_boxin

                self.node.get_logger().info(
                    f"預期檢查: boxin_port={context.get_boxin_port} → "
                    f"W114={layer_z_boxin}, W115={layer_y_boxin}"
                )

                # 執行檢查
                if self._handle_check_chg_parameter(context, expected_params):
                    # 檢查通過，進入下一步驟
                    self.step = RobotContext.WRITE_CHG_PARA
                # 否則繼續停留在此步驟，_handle_check_chg_parameter 會處理重試邏輯

            case RobotContext.WRITE_CHG_PARA:
                self.node.get_logger().info("Robot Take Transfer TAKE TRANSFER WRITE CHG PARA")
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
                self.node.get_logger().info("Robot Take Transfer TAKE TRANSFER CHECK CHG PARA")
                if read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("✅讀取預執行成功")
                    self.step = RobotContext.WRITE_PGNO
                elif read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("🕒讀取預執行中...")
                else:
                    self.node.get_logger().info("❌讀取預執行失敗")

            case RobotContext.WRITE_PGNO:
                self.node.get_logger().info("Robot Take Transfer TAKE TRANSFER WRITE_PGNO")
                if not self.sent:
                    context.robot.update_pgno(TAKE_TRANSFER_PGNO)
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
                self.node.get_logger().info("Robot Take Transfer TAKE TRANSFER CHECK_PGNO")
                if read_pgno is None:
                    self.node.get_logger().info("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == (TAKE_TRANSFER_PGNO):
                    self.node.get_logger().info("✅讀取PGNO成功")
                    self.step = RobotContext.ACTING
                elif read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("🕒讀取PGNO中...")
                else:
                    self.node.get_logger().info("❌讀取PGNO失敗")

            case RobotContext.ACTING:
                self.node.get_logger().info("Robot Take Transfer TAKE TRANSFER ACTING")
                if read_pgno is None:
                    self.node.get_logger().info("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == (TAKE_TRANSFER_PGNO):
                    self.node.get_logger().info("🤖手臂動作中")
                elif read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅手臂動作完成")
                    self.step = RobotContext.FINISH
                else:
                    self.node.get_logger().info("❌手臂動作失敗")
            case RobotContext.FINISH:
                self.node.get_logger().info("Robot Take Transfer TAKE TRANSFER BOX Finish")
                if read_pgno is None:
                    self.node.get_logger().info("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅取傳送箱完成")
                    self.step = 0
                    from loader_agv.robot_states.take_transfer.put_agv_state import PutAgvState
                    context.set_state(PutAgvState(self.node))
                else:
                    self.node.get_logger().info("❌取傳送箱失敗")
