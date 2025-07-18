from db_proxy_interfaces.msg import Carrier as CarrierMsg
from rclpy.node import Node
from cargo_mover_agv.robot_context import RobotContext
from agv_base.robot import Robot
from agv_base.hokuyo_dms_8bit import HokuyoDMS8Bit
from db_proxy.agvc_database_client import AGVCDatabaseClient
from cargo_mover_agv.robot_states.base_robot_state import BaseRobotState


class PutTransferState(BaseRobotState):
    def __init__(self, node: Node):
        super().__init__(node)

        self.hokuyo_dms_8bit_1: HokuyoDMS8Bit = self.node.hokuyo_dms_8bit_1
        self.step = RobotContext.IDLE
        self.agvc_client = AGVCDatabaseClient(self.node)
        self.update_carrier_success = False
        self.sent = False

        # 動態計算 port_id_address
        self.port_id_address = self.node.room_id * 1000 + 10

    def enter(self):
        self.node.get_logger().info("Robot Entrance 目前狀態: PutTransfer")
        self.update_carrier_success = False
        self.sent = False

    def leave(self):
        self.node.get_logger().info("Robot Entrance 離開 PutTransfer 狀態")
        self.update_carrier_success = False
        self.sent = False

    def update_carrier_database(self, context: RobotContext):
        carrier = CarrierMsg()
        carrier.id = context.carrier_id
        carrier.room_id = self.node.room_id
        carrier.rack_id = 0
        carrier.port_id = self.port_id_address+context.get_boxin_port
        carrier.rack_index = 0
        carrier.status_id = Robot.CARRIER_STATUS_ENTER_BOXIN_TRANSFER  # 進入入口傳送箱
        self.agvc_client.async_update_carrier(
            carrier, self.update_carrier_database_callback)

    def update_carrier_database_callback(self, result):
        if result is not None:
            self.node.get_logger().info(
                f"✅ Carrier 更新成功: {result.success}, {result.message}")
            self.update_carrier_success = True
        else:
            self.node.get_logger().error("❌ Carrier 更新失敗")
            self.update_carrier_success = False

    def handle(self, context: RobotContext):
        self.node.get_logger().info("Robot Entrance PutTransfer 狀態")

        # 並行執行：Hokuyo write_busy 設定
        self._set_hokuyo_busy_entrance()

        # 並行執行：其他操作（不需等待 Hokuyo 完成）
        PUT_TRANSFER_PGNO = context.robot.ACTION_TO + \
            context.robot.NONE_POSITION + context.robot.BOX_IN_POSITION
        read_pgno = context.robot.read_pgno_response
        context.robot.read_robot_status()

        # 更新 Hokuyo Input - 使用統一方法
        self._handle_hokuyo_input_entrance()
        # 條件執行：只有機器人邏輯需要等待 Hokuyo 完成
        if self.hokuyo_busy_write_completed:
            self._execute_robot_logic(context, PUT_TRANSFER_PGNO, read_pgno)

    def _execute_robot_logic(self, context: RobotContext, PUT_TRANSFER_PGNO, read_pgno):
        """執行機器人邏輯"""
        match self.step:
            case RobotContext.IDLE:
                self.node.get_logger().info("Robot Entrance PUT TRANSFER IDLE")
                self.step = RobotContext.CHECK_IDLE
            case RobotContext.CHECK_IDLE:
                self.node.get_logger().info("Robot Entrance PUT TRANSFER CHECK_IDLE")
                if read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅Robot狀態為IDLE")
                    self.step = RobotContext.WRITE_CHG_PARAMTER
                else:
                    self.node.get_logger().info("❌Robot狀態不為IDLE")

            case RobotContext.WRITE_CHG_PARAMTER:
                if not self.sent:
                    context.update_rack_box_port()
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
                self.node.get_logger().info("Robot Entrance PUT TRANSFER WRITE CHG PARA")
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
                self.node.get_logger().info("Robot Entrance PUT TRANSFER CHECK CHG PARA")
                if read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("✅讀取預執行成功")
                    self.step = RobotContext.WRITE_PGNO
                elif read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("🕒讀取預執行中...")
                else:
                    self.node.get_logger().info("❌讀取預執行失敗")

            case RobotContext.WRITE_PGNO:
                self.node.get_logger().info("Robot Entrance PUT TRANSFER WRITE_PGNO")
                if not self.sent:
                    context.robot.update_pgno(PUT_TRANSFER_PGNO)
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
                self.node.get_logger().info("Robot Entrance PUT TRANSFER CHECK_PGNO")
                if read_pgno.value == (PUT_TRANSFER_PGNO):
                    self.node.get_logger().info("✅讀取PGNO成功")
                    self.step = RobotContext.ACTING
                elif read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("🕒讀取PGNO中...")
                else:
                    self.node.get_logger().info("❌讀取PGNO失敗")

            case RobotContext.ACTING:
                self.node.get_logger().info("Robot Entrance PUT TRANSFER ACTING")
                if read_pgno.value == (PUT_TRANSFER_PGNO):
                    self.node.get_logger().debug("🤖手臂動作中")
                elif read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅手臂動作完成")
                    # 這裡可以添加完成後的邏輯
                    # 例如：context.set_state(AgvIdleState(self.node))
                    self.step = RobotContext.FINISH
                else:
                    self.node.get_logger().info("❌手臂動作失敗")
            case RobotContext.FINISH:
                self.node.get_logger().info("Robot Entrance PUT TRANSFER BOX Finish")
                if read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅放傳送箱完成")
                    context.boxin_buffer = context.get_boxin_port
                    self.step = RobotContext.UPDATE_DATABASE
                else:
                    self.node.get_logger().info("❌放傳送箱失敗")
            case RobotContext.UPDATE_DATABASE:
                self.node.get_logger().info("Robot Entrance PUT TRANSFER UPDATE_DATABASE")
                if not self.sent:
                    self.update_carrier_database(context)
                    self.sent = True
                elif self.sent and self.update_carrier_success:
                    self.node.get_logger().info("✅更新 Carrier 資料庫成功")
                    self.sent = False
                    from cargo_mover_agv.robot_states.entrance.check_rack_side_state import CheckRackSideState
                    context.set_state(CheckRackSideState(self.node))
                    self.step = RobotContext.IDLE
