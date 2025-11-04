from agv_base.states.state import State
from rclpy.node import Node
from loader_agv.robot_context import RobotContext
from shared_constants.equipment_stations import EquipmentStations


class IdleState(State):
    """Loader AGV Idle 狀態 - 使用新 Work ID 系統進行自動路由"""

    def __init__(self, node: Node):
        super().__init__(node)
        self.node = node

        # Hokuyo 初始化狀態
        self.hokuyo_write_completed = False

        # 從 task 獲取 room_id 和 work_id
        self.node.room_id = self.node.task.room_id
        self.node.work_id = self.node.task.work_id

    def enter(self):
        self.node.get_logger().info("🤖 Robot 目前狀態: Idle")

    def leave(self):
        self.node.get_logger().info("Robot 離開 Idle 狀態")

    def _initialize_hokuyo_parameters(self):
        """初始化 Hokuyo 8-bit 參數"""
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
                self.node.get_logger().info(
                    "✅ Hokuyo_1 參數設定完成: valid=0, tr_req=0, busy=0, complete=0")

                # 標記完成
                self.hokuyo_write_completed = True
                self.node.get_logger().info("✅ Hokuyo 參數初始化完成")

            except Exception as e:
                self.node.get_logger().error(f"❌ Hokuyo 參數設定失敗: {e}")
                # 即使失敗也標記為完成，避免無限重試
                self.hokuyo_write_completed = True

    def handle(self, context: RobotContext):
        """處理 Idle 狀態並根據 work_id 路由到對應的流程"""
        self.node.get_logger().info("Robot Idle 狀態")

        # 執行 Hokuyo 參數初始化
        self._initialize_hokuyo_parameters()

        # 只有在 Hokuyo 參數初始化完成後，才進行工作 ID 檢查和狀態切換
        if not self.hokuyo_write_completed:
            self.node.get_logger().debug("⏳ 等待 Hokuyo 參數初始化完成...")
            return

        self.node.get_logger().info("✅ Hokuyo 初始化完成，開始檢查工作 ID")

        # 獲取 work_id
        work_id = self.node.work_id
        self.node.get_logger().info(f"檢查工作 ID: {work_id}")

        # 使用 EquipmentStations 解析 work_id
        try:
            room_id, eqp_id, station, action_type = \
                EquipmentStations.extract_station_from_work_id(work_id)
            ports = EquipmentStations.station_to_ports(eqp_id, station)

            # 輸出解析結果
            self.node.get_logger().info(
                f"✅ Work ID {work_id} 解析成功: "
                f"Room={room_id}, Equipment={eqp_id}, Station={station:02d}, "
                f"Action={action_type}, Ports={ports}")

        except ValueError as e:
            self.node.get_logger().error(f"❌ Work ID 解析失敗: {e}")
            return

        # 根據設備類型路由到對應狀態
        equipment_type = eqp_id % 100

        if equipment_type == 1:  # Transfer (201)
            if action_type == 1:  # TAKE
                from loader_agv.robot_states.take_transfer.transfer_vision_position_state import TransferVisionPositionState
                self.node.get_logger().info("🎯 路由到: TAKE_TRANSFER 流程")
                context.set_state(TransferVisionPositionState(self.node))
            else:
                self.node.get_logger().error(
                    f"❌ Transfer 不支援 action_type={action_type}")

        elif equipment_type == 3:  # Cleaner (203)
            if action_type == 1:  # TAKE
                from loader_agv.robot_states.take_cleaner.cleaner_vision_position_state import CleanerVisionPositionState
                self.node.get_logger().info("🎯 路由到: TAKE_CLEANER 流程")
                context.set_state(CleanerVisionPositionState(self.node))
            elif action_type == 2:  # PUT
                from loader_agv.robot_states.put_cleaner.cleaner_vision_position_state import CleanerVisionPositionState
                self.node.get_logger().info("🎯 路由到: PUT_CLEANER 流程")
                context.set_state(CleanerVisionPositionState(self.node))
            else:
                self.node.get_logger().error(
                    f"❌ Cleaner 不支援 action_type={action_type}")

        elif equipment_type == 4:  # Soaker (204)
            if action_type == 1:  # TAKE
                from loader_agv.robot_states.take_soaker.soaker_vision_position_state import SoakerVisionPositionState
                self.node.get_logger().info("🎯 路由到: TAKE_SOAKER 流程")
                context.set_state(SoakerVisionPositionState(self.node))
            elif action_type == 2:  # PUT
                from loader_agv.robot_states.put_soaker.soaker_vision_position_state import SoakerVisionPositionState
                self.node.get_logger().info("🎯 路由到: PUT_SOAKER 流程")
                context.set_state(SoakerVisionPositionState(self.node))
            else:
                self.node.get_logger().error(
                    f"❌ Soaker 不支援 action_type={action_type}")

        elif equipment_type == 5:  # Pre-Dryer (205)
            if action_type == 1:  # TAKE
                from loader_agv.robot_states.take_pre_dryer.pre_dryer_vision_position_state import PreDryerVisionPositionState
                self.node.get_logger().info("🎯 路由到: TAKE_PRE_DRYER 流程")
                context.set_state(PreDryerVisionPositionState(self.node))
            elif action_type == 2:  # PUT
                from loader_agv.robot_states.put_pre_dryer.pre_dryer_vision_position_state import PreDryerVisionPositionState
                self.node.get_logger().info("🎯 路由到: PUT_PRE_DRYER 流程")
                context.set_state(PreDryerVisionPositionState(self.node))
            else:
                self.node.get_logger().error(
                    f"❌ Pre-Dryer 不支援 action_type={action_type}")

        else:
            self.node.get_logger().error(
                f"❌ 未知設備類型: {equipment_type} (eqp_id={eqp_id})")
