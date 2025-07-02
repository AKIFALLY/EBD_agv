from agv_base.states.state import State
from db_proxy_interfaces.msg import Carrier as CarrierMsg
from rclpy.node import Node
from loader_agv.robot_context import RobotContext  # 新增的匯入
from agv_base.robot import Robot
from agv_base.hokuyo_dms_8bit import HokuyoDMS8Bit
from db_proxy.agvc_database_client import AGVCDatabaseClient


class PutAgvState(State):
    PORT_ID_ADDRESS = 2100

    def __init__(self, node: Node):
        super().__init__(node)

        self.hokuyo_dms_8bit_1: HokuyoDMS8Bit = self.node.hokuyo_dms_8bit_1
        self.step = RobotContext.IDLE
        self.agvc_client = AGVCDatabaseClient(self.node)
        self.update_carrier_success = False
        self.sent = False
        self.hokuyo_input_updated = False  # 用於判斷是否已經更新過 Hokuyo Input

    def enter(self):
        self.node.get_logger().info("Robot Take Transfer 目前狀態: PutAgv")
        self.update_carrier_success = False
        self.sent = False
        self.hokuyo_input_updated = False  # 用於判斷是否已經更新過 Hokuyo Input

    def leave(self):
        self.node.get_logger().info("Robot Take Transfer 離開 PutAgv 狀態")
        self.update_carrier_success = False
        self.sent = False
        self.hokuyo_input_updated = False  # 用於判斷是否已經更新過 Hokuyo Input

    def update_carrier_database(self, context: RobotContext):
        carrier = CarrierMsg()
        carrier.id = context.carrier_id
        context.get_room_id = 2  # 假設這是傳送箱所在的房間 ID
        carrier.room_id = context.get_room_id
        carrier.rack_id = 0
        carrier.port_id = self.PORT_ID_ADDRESS+context.get_loader_agv_port_front
        carrier.rack_index = 0
        carrier.status_id = 2  # 假設 2 是表示傳送箱的狀態
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
        self.node.get_logger().info("Robot Take Transfer PutAgv 狀態")
        PUT_LOADER_AGV_PGNO = context.robot.ACTION_TO + \
            context.robot.NONE_POSITION + context.robot.AGV_POSITION
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
                self.node.get_logger().info("Robot Take Transfer PUT LOADER AGV IDLE")
                self.step = RobotContext.CHECK_IDLE
            case RobotContext.CHECK_IDLE:
                self.node.get_logger().info("Robot Take Transfer PUT LOADER AGV CHECK_IDLE")
                if read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅Robot狀態為IDLE")
                    self.step = RobotContext.WRITE_CHG_PARAMTER
                else:
                    self.node.get_logger().info("❌Robot狀態不為IDLE")

            case RobotContext.WRITE_CHG_PARAMTER:
                if not self.sent:
                    context.update_port_parameters()
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
                self.node.get_logger().info("Robot Take Transfer PUT LOADER AGV WRITE CHG PARA")
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
                self.node.get_logger().info("Robot Take Transfer PUT LOADER AGV CHECK CHG PARA")
                if read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("✅讀取預執行成功")
                    self.step = RobotContext.WRITE_PGNO
                elif read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("🕒讀取預執行中...")
                else:
                    self.node.get_logger().info("❌讀取預執行失敗")

            case RobotContext.WRITE_PGNO:
                self.node.get_logger().info("Robot Take Transfer PUT LOADER AGV WRITE_PGNO")
                if not self.sent:
                    context.robot.update_pgno(PUT_LOADER_AGV_PGNO)
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
                self.node.get_logger().info("Robot Take Transfer PUT LOADER AGV CHECK_PGNO")
                if read_pgno.value == (PUT_LOADER_AGV_PGNO):
                    self.node.get_logger().info("✅讀取PGNO成功")
                    self.step = RobotContext.ACTING
                elif read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("🕒讀取PGNO中...")
                else:
                    self.node.get_logger().info("❌讀取PGNO失敗")

            case RobotContext.ACTING:
                self.node.get_logger().info("Robot Take Transfer PUT LOADER AGV ACTING")
                if read_pgno.value == (PUT_LOADER_AGV_PGNO):
                    self.node.get_logger().info("🤖手臂動作中")
                elif read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅手臂動作完成")
                    # 這裡可以添加完成後的邏輯
                    # 例如：context.set_state(AgvIdleState(self.node))
                    self.step = RobotContext.FINISH
                else:
                    self.node.get_logger().info("❌手臂動作失敗")
            case RobotContext.FINISH:
                self.node.get_logger().info("Robot Take Transfer PUT LOADER AGV BOX Finish")
                if read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅放AGV箱完成")
                    context.boxin_buffer = context.get_loader_agv_port_front
                    self.step = RobotContext.UPDATE_DATABASE
                else:
                    self.node.get_logger().info("❌放AGV箱失敗")
            case RobotContext.UPDATE_DATABASE:
                self.node.get_logger().info("Robot Take Transfer PUT LOADER AGV UPDATE_DATABASE")
                if not self.sent:
                    self.update_carrier_database(context)
                    self.sent = True
                elif self.sent and self.update_carrier_success:
                    self.node.get_logger().info("✅更新 Carrier 資料庫成功")
                    self.sent = False

                    # 根據take_transfer_continue決定下一個狀態
                    if context.take_transfer_continue:
                        self.node.get_logger().info("🔄 Take Transfer 繼續: 進入 TransferCheckHaveState")
                        from loader_agv.robot_states.take_transfer.transfer_check_have_state import TransferCheckHaveState
                        context.set_state(TransferCheckHaveState(self.node))
                    else:
                        self.node.get_logger().info("✅ Take Transfer 完成: 進入 CompleteState")
                        from loader_agv.robot_states.complete_state import CompleteState
                        context.set_state(CompleteState(self.node))

                    self.step = RobotContext.IDLE
