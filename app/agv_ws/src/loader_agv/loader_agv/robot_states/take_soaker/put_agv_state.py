from db_proxy_interfaces.msg import Carrier as CarrierMsg
from rclpy.node import Node
from loader_agv.robot_context import RobotContext
from agv_base.robot import Robot
from agv_base.hokuyo_dms_8bit import HokuyoDMS8Bit
from db_proxy.agvc_database_client import AGVCDatabaseClient
from loader_agv.robot_states.base_robot_state import BaseRobotState


class PutAgvState(BaseRobotState):

    def __init__(self, node: Node):
        super().__init__(node)
        self.node = node
        # 動態參數設定：AGV 參數
        self.port_id_address = self.node.room_id * 1000 + 100
        self.agvc_client = AGVCDatabaseClient(self.node)
        self.update_carrier_success = False

    def enter(self):
        self.node.get_logger().info(
            "[Station-based 1格] Loader Robot Take Soaker 目前狀態: PutAgv")
        self.update_carrier_success = False
        self._reset_state()

    def leave(self):
        self.node.get_logger().info(
            "[Station-based 1格] Loader Robot Take Soaker 離開 PutAgv 狀態")
        self.update_carrier_success = False
        self._reset_state()

    def update_carrier_database(self, context: RobotContext):
        """更新 carrier 的最終位置資訊 - 單格處理

        說明：Take Soaker 使用偶數層策略（Port 2, 4）
        - 從泡藥機取出後，放到 AGV Port 2 或 4（偶數層）
        - 與 Put Soaker 的奇數層（Port 1, 3）區隔，避免衝突
        """
        carrier = CarrierMsg()
        carrier.id = context.carrier_id
        carrier.room_id = self.node.room_id
        carrier.rack_id = 0
        carrier.port_id = self.port_id_address + context.get_loader_agv_port_side
        carrier.rack_index = 0
        carrier.status_id = Robot.CARRIER_STATUS_PREPARE_ENTER_PRE_DRYER  # 準備進入烘箱

        self.agvc_client.async_update_carrier(
            carrier, self.update_carrier_database_callback)
        self.node.get_logger().info(
            f"🔄 [Station-based 1格] 開始更新 Carrier: {context.carrier_id} "
            f"(Work ID {context.work_id})")
        self.node.get_logger().info(
            f"完整路徑: 泡藥機 Port {context.get_soaker_port} → 機械臂 → "
            f"AGV Port {context.get_loader_agv_port_side} (偶數層策略)")
        self.node.get_logger().info(
            f"最終位置: port_id={carrier.port_id}, "
            f"狀態: PREPARE_ENTER_PRE_DRYER")

    def update_carrier_database_callback(self, result):
        """處理 carrier 資料庫更新回應"""
        if result is not None and result.success:
            self.node.get_logger().info(
                f"✅ [Station-based 1格] Carrier 更新成功: {result.success}, {result.message}")
            self.update_carrier_success = True
        else:
            self.node.get_logger().error(
                "❌ [Station-based 1格] Carrier 更新失敗")
            self.update_carrier_success = False

    def handle(self, context: RobotContext):
        # 單格處理：機械臂 → AGV Port（偶數層策略）
        source_soaker_port = context.get_soaker_port
        target_agv_port = context.get_loader_agv_port_side

        self.node.get_logger().info(
            f"[Station-based 1格] 放料到 AGV (Work ID {context.work_id})")
        self.node.get_logger().info(
            f"來源: 機械臂 → 目標: AGV Port {target_agv_port} (偶數層策略)")
        self.node.get_logger().info(
            f"完整路徑: 泡藥機 Port {source_soaker_port} → 機械臂 → "
            f"AGV Port {target_agv_port}")

        # 並行執行：Hokuyo write_busy 設定
        self._set_hokuyo_busy()

        # 並行執行：其他操作（不需等待 Hokuyo 完成）
        # PGNO 常數調整：PUT_LOADER_AGV_PGNO
        PUT_LOADER_AGV_PGNO = context.robot.ACTION_TO + \
            context.robot.NONE_POSITION + context.robot.AGV_POSITION_SIDE
        read_pgno = context.robot.read_pgno_response
        context.robot.read_robot_status()

        # 更新 Hokuyo Input - 使用統一方法
        self._handle_hokuyo_input()

        # 條件執行：只有機器人邏輯需要等待 Hokuyo 完成
        if self.hokuyo_busy_write_completed:
            self._execute_robot_logic(context, PUT_LOADER_AGV_PGNO, read_pgno)

    def _execute_robot_logic(self, context: RobotContext, PUT_LOADER_AGV_PGNO, read_pgno):
        """執行機器人邏輯 - 單格處理"""
        match self.step:
            case RobotContext.IDLE:
                self.node.get_logger().info(
                    "[Station-based 1格] Loader Robot Take Soaker PUT LOADER AGV IDLE")
                self.step = RobotContext.CHECK_IDLE
            case RobotContext.CHECK_IDLE:
                self.node.get_logger().info(
                    "[Station-based 1格] Loader Robot Take Soaker PUT LOADER AGV CHECK_IDLE")
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
                self.node.get_logger().info("Robot Take Soaker PUT AGV CHECK CHG PARAMETER")

                # 構建預期參數字典
                expected_params = {}

                # 只檢查 W110（Layer Side 的 Z 軸）
                # W110 應該等於 get_loader_agv_port_side
                expected_params['w110'] = context.get_loader_agv_port_side

                self.node.get_logger().info(
                    f"預期檢查: loader_agv_port_side={context.get_loader_agv_port_side} → "
                    f"W110={context.get_loader_agv_port_side}"
                )

                # 執行檢查
                if self._handle_check_chg_parameter(context, expected_params):
                    # 檢查通過，進入下一步驟
                    self.step = RobotContext.WRITE_CHG_PARA
                # 否則繼續停留在此步驟

            case RobotContext.WRITE_CHG_PARA:
                self.node.get_logger().info(
                    "[Station-based 1格] Loader Robot Take Soaker PUT LOADER AGV WRITE CHG PARA")
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
                self.node.get_logger().info(
                    "[Station-based 1格] Loader Robot Take Soaker PUT LOADER AGV CHECK CHG PARA")
                if read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("✅讀取預執行成功")
                    self.step = RobotContext.WRITE_PGNO
                elif read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("🕒讀取預執行中...")
                else:
                    self.node.get_logger().info("❌讀取預執行失敗")

            case RobotContext.WRITE_PGNO:
                self.node.get_logger().info(
                    "[Station-based 1格] Loader Robot Take Soaker PUT LOADER AGV WRITE_PGNO")
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
                self.node.get_logger().info(
                    "[Station-based 1格] Loader Robot Take Soaker PUT LOADER AGV CHECK_PGNO")
                if read_pgno is None:
                    self.node.get_logger().debug("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == (PUT_LOADER_AGV_PGNO):
                    self.node.get_logger().info("✅讀取PGNO成功")
                    self.step = RobotContext.ACTING
                elif read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("🕒讀取PGNO中...")
                else:
                    self.node.get_logger().error("❌讀取PGNO失敗")

            case RobotContext.ACTING:
                self.node.get_logger().info(
                    "[Station-based 1格] Loader Robot Take Soaker PUT LOADER AGV ACTING")
                if read_pgno is None:
                    self.node.get_logger().debug("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == (PUT_LOADER_AGV_PGNO):
                    self.node.get_logger().debug("🤖手臂動作中")
                elif read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅手臂動作完成")
                    self.step = RobotContext.FINISH
                else:
                    self.node.get_logger().info("❌手臂動作失敗")
            case RobotContext.FINISH:
                self.node.get_logger().info(
                    "[Station-based 1格] Loader Robot Take Soaker PUT LOADER AGV Finish")
                if read_pgno is None:
                    self.node.get_logger().info("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅放AGV箱完成")
                    self.node.get_logger().info(
                        f"完整路徑: 泡藥機 Port {context.get_soaker_port} → 機械臂 → "
                        f"AGV Port {context.get_loader_agv_port_side} (偶數層策略)")
                    context.boxin_buffer = context.get_loader_agv_port_side
                    self.step = RobotContext.UPDATE_DATABASE
                else:
                    self.node.get_logger().info("❌放AGV箱失敗")
            case RobotContext.UPDATE_DATABASE:
                self.node.get_logger().info(
                    "[Station-based 1格] Loader Robot Take Soaker PUT LOADER AGV UPDATE_DATABASE")
                if not self.sent:
                    self.update_carrier_database(context)
                    self.sent = True
                elif self.sent and self.update_carrier_success:
                    self.node.get_logger().info(
                        "✅ [Station-based 1格] 更新 Carrier 資料庫成功")
                    self.sent = False

                    # 單格處理：put_agv_state.py 完成後直接轉換到 CompleteState 結束流程
                    self.node.get_logger().info(
                        f"✅ [Station-based 1格] Take Soaker 完成: 進入 CompleteState "
                        f"(Work ID {context.work_id})")
                    self.node.get_logger().info(
                        f"完整路徑: 泡藥機 Port {context.get_soaker_port} → 機械臂 → "
                        f"AGV Port {context.get_loader_agv_port_side} (偶數層策略)")
                    from loader_agv.robot_states.complete_state import CompleteState
                    context.set_state(CompleteState(self.node))

                    self.step = RobotContext.IDLE
