from agv_base.states.state import State
from rclpy.node import Node
from cargo_mover_agv.robot_context import RobotContext  # 新增的匯入
import json


class IdleState(State):
    ENTRANCE = "01"
    EXIT = "02"
    TAKE = "01"
    PUT = "02"

    def __init__(self, node: Node):
        super().__init__(node)
        self.node = node
        # Hokuyo 寫入相關變數
        self.hokuyo_write_completed = False
        
        # 工作 ID 將在 enter 方法中動態計算
        self.entrance_work = None
        self.exit_work = None

    def enter(self):
        self.node.get_logger().info("🤖robot 目前狀態: Idle")

        
    def leave(self):
        self.node.get_logger().info("robot 離開 Idle 狀態")

    def update_rack_id(self, context: RobotContext):
        """
        從 task.parameters 中解析 rack_id 並設定到 context 中

        此方法會在每次執行時重新解析 task.parameters，支援 JSON 字串和字典格式，
        並包含完整的錯誤處理和日誌記錄功能。

        Args:
            context (RobotContext): 機器人上下文物件，用於存儲解析得到的 rack_id
        """
        # 解析 task.parameters 並設定 rack_id（每次執行時都重新解析）
        try:
            if self.node.task.parameters:
                # 如果 parameters 是字串，需要解析 JSON
                if isinstance(self.node.task.parameters, str):
                    parameters_dict = json.loads(self.node.task.parameters)
                else:
                    # 如果已經是字典，直接使用
                    parameters_dict = self.node.task.parameters

                # 從 parameters 中取得 rack_id
                rack_id = parameters_dict.get("rack_id")
                if rack_id is not None:
                    try:
                        # 確保 rack_id 是整數類型，符合 CarrierQuery 服務的 uint64 要求
                        context.rack_id = int(rack_id)
                        self.node.get_logger().info(
                            f"✅ 從 task.parameters 取得 rack_id: {context.rack_id} (type: {type(context.rack_id).__name__})")
                    except (ValueError, TypeError) as e:
                        context.rack_id = None
                        self.node.get_logger().error(f"❌ 無法將 rack_id 轉換為整數: {rack_id}, 錯誤: {e}")
                else:
                    context.rack_id = None
                    self.node.get_logger().warn("⚠️ task.parameters 中沒有找到 rack_id，設為 None")
            else:
                context.rack_id = None
                self.node.get_logger().warn("⚠️ task.parameters 為空，rack_id 設為 None")
        except (json.JSONDecodeError, TypeError, AttributeError) as e:
            context.rack_id = None
            self.node.get_logger().error(f"❌ 解析 task.parameters 時發生錯誤: {e}，rack_id 設為 None")

    def _initialize_hokuyo_parameters(self):
        """同時初始化兩個 Hokuyo 物件的參數"""
        if not self.hokuyo_write_completed:
            self.node.get_logger().info("🔄 開始 Hokuyo 參數初始化流程")

            # 同時對兩個 Hokuyo 物件進行參數設定
            hokuyo_1 = self.node.hokuyo_dms_8bit_1
            hokuyo_2 = self.node.hokuyo_dms_8bit_2

            try:
                # 並行設定 hokuyo_dms_8bit_1 的參數
                hokuyo_1.write_valid("0")
                hokuyo_1.write_tr_req("0")
                hokuyo_1.write_busy("0")
                hokuyo_1.write_complete("0")
                self.node.get_logger().info("✅ Hokuyo_1 參數設定完成: valid=0, tr_req=0, busy=0, complete=0")

                # 並行設定 hokuyo_dms_8bit_2 的參數
                hokuyo_2.write_valid("0")
                hokuyo_2.write_tr_req("0")
                hokuyo_2.write_busy("0")
                hokuyo_2.write_complete("0")
                self.node.get_logger().info("✅ Hokuyo_2 參數設定完成: valid=0, tr_req=0, busy=0, complete=0")

                # 標記完成
                self.hokuyo_write_completed = True
                self.node.get_logger().info("✅ 所有 Hokuyo 參數初始化完成")

            except Exception as e:
                self.node.get_logger().error(f"❌ Hokuyo 參數設定失敗: {e}")
                # 即使失敗也標記為完成，避免無限重試
                self.hokuyo_write_completed = True

    def handle(self, context: RobotContext):
        # 檢查 task 物件是否存在
        if not hasattr(self.node, 'task') or self.node.task is None:
            self.node.get_logger().info("⏳ robot Idle 狀態 - 等待任務分配")
            return
        
        work_id = self.node.task.work_id
        self.node.get_logger().info("robot Idle 狀態")

        # 執行 Hokuyo 參數初始化（同時對兩個 Hokuyo 物件進行設定）
        self._initialize_hokuyo_parameters()

        # 顯示 Hokuyo 初始化狀態
        self.node.get_logger().info(f"🔍 Hokuyo 初始化狀態: {self.hokuyo_write_completed}")

        # 只有在 Hokuyo 參數初始化完成後，才進行 rack_id 解析和狀態切換
        if self.hokuyo_write_completed:
            self.node.get_logger().info("✅ Hokuyo 初始化完成，開始解析 rack_id")

            # 解析 rack_id（在工作 ID 檢查之前）
            self.update_rack_id(context)

            # 驗證 rack_id 解析結果
            if context.rack_id is not None:
                self.node.get_logger().info(f"✅ rack_id 解析成功: {context.rack_id}")
            else:
                self.node.get_logger().warn("⚠️ rack_id 解析失敗，將使用 fallback 值 123")

            # 簡化的 work_id 取得方式
            self.node.get_logger().info(f"檢查task: {self.node.task}")
            self.node.get_logger().info(f"檢查工作 ID: {work_id}")
            self.node.room_id = self.node.task.room_id
            self.node.work_id = self.node.task.work_id
        
            # 在 enter 時動態計算工作 ID（確保 room_id 已正確設定）
            self.entrance_work = int(str(self.node.room_id) + "00" + self.ENTRANCE + self.PUT)
            self.exit_work = int(str(self.node.room_id) + "00" + self.EXIT + self.TAKE)
            self.node.get_logger().info(f"🔢 動態計算工作 ID - entrance_work: {self.entrance_work}, exit_work: {self.exit_work}")
            # 使用預計算的動態工作ID進行比較
            if work_id == self.entrance_work:
                self.node.get_logger().info("切換到 ENTRANCE 流程")
                try:
                    from cargo_mover_agv.robot_states.entrance.transfer_vision_position_state import TransferVisionPositionState
                    self.node.get_logger().info("✅ TransferVisionPositionState 導入成功")
                    context.set_state(TransferVisionPositionState(self.node))
                    self.node.get_logger().info("✅ 狀態切換完成")
                    return  # 重要：狀態切換後立即返回，避免重複執行
                except Exception as e:
                    self.node.get_logger().error(f"❌ ENTRANCE 流程狀態切換失敗: {e}")
            elif work_id == self.exit_work:
                self.node.get_logger().info("切換到 EXIT 流程")
                try:
                    from cargo_mover_agv.robot_states.exit.transfer_vision_position_state import TransferVisionPositionState
                    self.node.get_logger().info("✅ TransferVisionPositionState 導入成功")
                    context.set_state(TransferVisionPositionState(self.node))
                    self.node.get_logger().info("✅ 狀態切換完成")
                    return  # 重要：狀態切換後立即返回，避免重複執行
                except Exception as e:
                    self.node.get_logger().error(f"❌ EXIT 流程狀態切換失敗: {e}")
            else:
                self.node.get_logger().warn(f"未知的工作 ID: {work_id}")
        else:
            self.node.get_logger().info("⏳ 等待 Hokuyo 參數初始化完成...")
