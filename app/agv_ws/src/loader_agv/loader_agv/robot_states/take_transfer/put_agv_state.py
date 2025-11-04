from db_proxy_interfaces.msg import Carrier as CarrierMsg
from rclpy.node import Node
from loader_agv.robot_context import RobotContext
from agv_base.robot import Robot
from db_proxy.agvc_database_client import AGVCDatabaseClient
from loader_agv.robot_states.base_robot_state import BaseRobotState


class PutAgvState(BaseRobotState):

    def __init__(self, node: Node):
        super().__init__(node)
        self.node = node
        self.port_id_address = self.node.room_id * 1000 + 100

        self.agvc_client = AGVCDatabaseClient(self.node)
        self.update_carrier_success = False

    def enter(self):
        self.node.get_logger().info("Robot Take Transfer 目前狀態: PutAgv")
        self.update_carrier_success = False
        self._reset_state()

    def leave(self):
        self.node.get_logger().info("Robot Take Transfer 離開 PutAgv 狀態")
        self.update_carrier_success = False
        self._reset_state()

    def update_carrier_database(self, context: RobotContext):
        carrier = CarrierMsg()
        carrier.id = context.carrier_id
        carrier.room_id = self.node.room_id
        carrier.rack_id = 0
        carrier.port_id = self.port_id_address + context.get_loader_agv_port_front
        carrier.rack_index = 0
        carrier.status_id = Robot.CARRIER_STATUS_PREPARE_ENTER_CLEANER  # 使用中

        self.node.get_logger().info(
            f"更新 Carrier 資料庫: carrier_id={carrier.id}, "
            f"port_id={carrier.port_id}, status_id={carrier.status_id}")

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
        # 批量放料 AGV port 映射：根據計數器決定目標 AGV port
        # Station-based 設計 + L尺寸配置：
        # - 第1次從傳送箱取料 → 放到 AGV port1 (第1層)
        # - 第2次從傳送箱取料 → 放到 AGV port3 (第3層)
        # L尺寸產品配置固定使用 port1 和 port3（第2層和第4層掛勾已解下）
        agv_port_mapping = [1, 3]  # 第1次→port1, 第2次→port3
        target_agv_port = agv_port_mapping[context.transfer_take_count]
        context.get_loader_agv_port_front = target_agv_port

        self.node.get_logger().info(
            f"[Station-based 批量] 放料第 {context.transfer_take_count + 1}/2 次 "
            f"(Work ID {context.work_id})")
        self.node.get_logger().info(
            f"來源: 機械臂 → 目標: AGV Port {target_agv_port} (L尺寸第{target_agv_port}層), "
            f"carrier_id={context.carrier_id}")

        # 並行執行：Hokuyo write_busy 設定
        self._set_hokuyo_busy()

        # 並行執行：其他操作（不需等待 Hokuyo 完成）
        PUT_LOADER_AGV_PGNO = context.robot.ACTION_TO + \
            context.robot.NONE_POSITION + context.robot.AGV_POSITION
        read_pgno = context.robot.read_pgno_response
        context.robot.read_robot_status()

        # 更新 Hokuyo Input - 使用統一方法
        self._handle_hokuyo_input()

        # 條件執行：只有機器人邏輯需要等待 Hokuyo 完成
        if self.hokuyo_busy_write_completed:
            self._execute_robot_logic(context, PUT_LOADER_AGV_PGNO, read_pgno)

    def _execute_robot_logic(self, context: RobotContext, PUT_LOADER_AGV_PGNO, read_pgno):
        """執行機器人邏輯"""
        match self.step:
            case RobotContext.IDLE:
                self.node.get_logger().info("Robot Take Transfer PUT LOADER AGV IDLE")
                self.step = RobotContext.CHECK_IDLE
            case RobotContext.CHECK_IDLE:
                self.node.get_logger().info("Robot Take Transfer PUT LOADER AGV CHECK_IDLE")
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
                self.node.get_logger().info("Robot Put AGV CHECK CHG PARAMETER")

                # 構建預期參數字典
                expected_params = {}

                # 只檢查 W112（Layer Front 的 Z 軸）
                # W112 應該等於 get_loader_agv_port_front
                expected_params['w112'] = context.get_loader_agv_port_front

                self.node.get_logger().info(
                    f"預期檢查: loader_agv_port_front={context.get_loader_agv_port_front} → "
                    f"W112={context.get_loader_agv_port_front}"
                )

                # 執行檢查
                if self._handle_check_chg_parameter(context, expected_params):
                    # 檢查通過，進入下一步驟
                    self.step = RobotContext.WRITE_CHG_PARA
                # 否則繼續停留在此步驟

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
                if read_pgno is None:
                    self.node.get_logger().info("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == (PUT_LOADER_AGV_PGNO):
                    self.node.get_logger().info("✅讀取PGNO成功")
                    self.step = RobotContext.ACTING
                elif read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("🕒讀取PGNO中...")
                else:
                    self.node.get_logger().info("❌讀取PGNO失敗")

            case RobotContext.ACTING:
                self.node.get_logger().info("Robot Take Transfer PUT LOADER AGV ACTING")
                if read_pgno is None:
                    self.node.get_logger().info("⏳等待讀取PGNO回應...")
                    return
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
                if read_pgno is None:
                    self.node.get_logger().info("⏳等待讀取PGNO回應...")
                    return
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

                    # 批量取料邏輯：檢查是否完成 2 次取料
                    # Station-based 設計：每個 Station 包含2個 port，批量處理2格
                    if context.transfer_take_count == 0:
                        # 第 1 次完成 → 繼續第 2 次
                        context.transfer_take_count = 1
                        self.node.get_logger().info(
                            f"🔄 [Station-based 批量] 第 1/2 次完成，繼續第 2 次取料 "
                            f"(Work ID {context.work_id})")
                        self.node.get_logger().info(
                            f"下一次取料: 傳送箱 Port {context.transfer_ports[1]} → AGV Port 3")
                        from loader_agv.robot_states.take_transfer.take_transfer_state import TakeTransferState
                        context.set_state(TakeTransferState(self.node))
                    else:
                        # 第 2 次完成 → 任務完成
                        self.node.get_logger().info(
                            f"✅ [Station-based 批量] 批量取料完成 (2/2 次): 進入 CompleteState "
                            f"(Work ID {context.work_id})")
                        from loader_agv.robot_states.complete_state import CompleteState
                        context.set_state(CompleteState(self.node))

                    self.step = RobotContext.IDLE
