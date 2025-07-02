from agv_base.states.state import State
from db_proxy_interfaces.msg import Carrier as CarrierMsg
from rclpy.node import Node
from cargo_mover_agv.robot_context import RobotContext  # 新增的匯入
from agv_base.robot import Robot
from agv_base.hokuyo_dms_8bit import HokuyoDMS8Bit
from db_proxy.agvc_database_client import AGVCDatabaseClient


class TakeTransferState(State):
    def __init__(self, node: Node):
        super().__init__(node)

        self.hokuyo_dms_8bit_1: HokuyoDMS8Bit = self.node.hokuyo_dms_8bit_1
        self.step = RobotContext.IDLE
        self.agvc_client = AGVCDatabaseClient(self.node)
        self.update_carrier_success = False
        self.sent = False
        self.hokuyo_input_updated = False  # 用於判斷是否已經更新過 Hokuyo Input

    def enter(self):
        self.node.get_logger().info("Robot Exit 目前狀態: TakeTransfer")
        self.update_carrier_success = False
        self.sent = False
        self.hokuyo_input_updated = False  # 用於判斷是否已經更新過 Hokuyo Input

    def leave(self):
        self.node.get_logger().info("Robot Exit 離開 TakeTransfer 狀態")
        self.update_carrier_success = False
        self.sent = False
        self.hokuyo_input_updated = False  # 用於判斷是否已經更新過 Hokuyo Input

    def handle(self, context: RobotContext):
        self.node.get_logger().info("Robot Exit TakeTransfer 狀態")
        TAKE_TRANSFER_PGNO = context.robot.ACTION_FROM + \
            context.robot.BOX_OUT_POSITION + context.robot.NONE_POSITION
        read_pgno = context.robot.read_pgno_response
        context.robot.read_pgno()

        # 更新 Hokuyo Input
        if not self.hokuyo_input_updated:
            self.hokuyo_dms_8bit_1.update_hokuyo_input()
            self.hokuyo_input_updated = True
        if self.hokuyo_dms_8bit_1.hokuyo_input_success:
            self.node.get_logger().info("Hokuyo Input 更新成功")
            self.hokuyo_dms_8bit_1.hokuyo_input_success = False
            self.hokuyo_input_updated = False
        elif self.hokuyo_dms_8bit_1.hokuyo_input_failed:
            self.node.get_logger().info("Hokuyo Input 更新失敗")
            self.hokuyo_dms_8bit_1.hokuyo_input_failed = False
            self.hokuyo_input_updated = False
        else:
            self.node.get_logger().info("等待 Hokuyo Input 更新")

        print("🔶=========================================================================🔶")

        match self.step:
            case RobotContext.IDLE:
                self.node.get_logger().info("Robot Exit TAKE TRANSFER IDLE")
                self.step = RobotContext.CHECK_IDLE
            case RobotContext.CHECK_IDLE:
                self.node.get_logger().info("Robot Exit TAKE TRANSFER CHECK_IDLE")
                if read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅Robot狀態為IDLE")
                    self.step = RobotContext.WRITE_CHG_PARAMTER
                else:
                    self.node.get_logger().info("❌Robot狀態不為IDLE")

            case RobotContext.WRITE_CHG_PARAMTER:
                if not self.sent:
                    context.update_rack_box_port()
                    context.robot.update_parameter()
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
                self.node.get_logger().info("Robot Exit TAKE TRANSFER WRITE CHG PARA")
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
                self.node.get_logger().info("Robot Exit TAKE TRANSFER CHECK CHG PARA")
                if read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("✅讀取預執行成功")
                    self.step = RobotContext.WRITE_PGNO
                elif read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("🕒讀取預執行中...")
                else:
                    self.node.get_logger().info("❌讀取預執行失敗")

            case RobotContext.WRITE_PGNO:
                self.node.get_logger().info("Robot Exit TAKE TRANSFER WRITE_PGNO")
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
                self.node.get_logger().info("Robot Exit TAKE TRANSFER CHECK_PGNO")
                if read_pgno.value == (TAKE_TRANSFER_PGNO):
                    self.node.get_logger().info("✅讀取PGNO成功")
                    self.step = RobotContext.ACTING
                elif read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("🕒讀取PGNO中...")
                else:
                    self.node.get_logger().info("❌讀取PGNO失敗")

            case RobotContext.ACTING:
                self.node.get_logger().info("Robot Exit TAKE TRANSFER ACTING")
                if read_pgno.value == (TAKE_TRANSFER_PGNO):
                    self.node.get_logger().info("🤖手臂動作中")
                elif read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅手臂動作完成")
                    # 這裡可以添加完成後的邏輯
                    # 例如：context.set_state(AgvIdleState(self.node))
                    self.step = RobotContext.FINISH
                else:
                    self.node.get_logger().info("❌手臂動作失敗")
            case RobotContext.FINISH:
                self.node.get_logger().info("Robot Exit TAKE TRANSFER BOX Finish")
                if read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅取傳送箱完成")
                    self.step = 0
                    from cargo_mover_agv.robot_states.exit.put_rack_port_state import PutRackPortState
                    context.set_state(PutRackPortState(self.node))
                else:
                    self.node.get_logger().info("❌取傳送箱失敗")
