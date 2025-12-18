from agv_base.states.state import State
from rclpy.node import Node


class RunningState(State):
    def __init__(self, node: Node):
        super().__init__(node)
        self.count = 0  # 計數器
        self.next_node = 0 #下一個點位
        self.ask_traffic_area = [] #詢問AGVC交管區域
        self.traffic_area_registed = [] #註冊的交管區域
        # 假設有一個 RobotContext 類別來管理機器人狀態

        # 【新增】任務訂閱相關變數已移至 node (全局共享)
        self.subscription = None
        # self.latest_tasks 已移至 node (全局共享)

        # 狀態一致性檢查相關變數
        self.status_check_counter = 0  # 計數器（每 5 秒檢查）

    def enter(self):
        self.node.get_logger().info("🏃 AGV 進入: Running 狀態")
        self.node.get_logger().info("📡 使用全局 /agvc/tasks 訂閱（已在 agv_node_base 建立）")

    def leave(self):
        self.node.get_logger().info("🚪 AGV 離開 Running 狀態")

        # 離開前將訂閱到的task表利用agv_name搜尋到當前執行的整個任務欄位丟進self.node.task
        # 使用 is_task_executing_status() 檢查所有執行中狀態 (2,4,12,14,22)
        # 注意：tasks 現在是 dict 列表
        if self.node.latest_tasks:
            from shared_constants.task_status import TaskStatus
            for task in self.node.latest_tasks:
                task_agv_name = task.get('agv_name') if isinstance(task, dict) else getattr(task, 'agv_name', '')
                task_status_id = task.get('status_id') if isinstance(task, dict) else getattr(task, 'status_id', 0)
                if (task_agv_name == self.node.agv_name and
                    TaskStatus.is_task_executing_status(task_status_id)):
                    self.node.task = task
                    task_id = task.get('id') if isinstance(task, dict) else getattr(task, 'id', 0)
                    self.node.get_logger().info(f"⚠️ 離開前印出當前任務 (status_id={task_status_id}): task_id={task_id}")
                    break

        # 全局訂閱不需移除，由 agv_node_base 管理

    def handle(self, context):
        # self.node.get_logger().info("AGV Running 狀態")
        #self.node.get_logger().info(f"work_id={self.node.task.work_id}")
        #self.node.get_logger().info(f"path={self.node.agv_status.AGV_PATH}")
        #self.node.get_logger().info(f"Complete={self.node.agv_status.AGV_LD_COMPLETE}")

        # 🔍 每 5 秒檢查狀態一致性（100 個循環 ≈ 5 秒）
        if self.status_check_counter > 100:
            self.status_check_counter = 0
            self._check_status_consistency()
        self.status_check_counter += 1

        # 統一處理：沒有路徑資料時進入 WaitRobot 統一判斷
        if not self.node.agv_status.AGV_PATH:
            self.node.get_logger().info("⚠️ AGV 沒有路徑資料，進入 WaitRobot 狀態統一判斷")

            # 在跳轉前先抓取 task，避免因為直接跳出而沒抓到
            # 使用 is_task_executing_status() 檢查所有執行中狀態 (2,4,12,14,22)
            # 注意：tasks 現在是 dict 列表
            if self.node.latest_tasks:
                from shared_constants.task_status import TaskStatus
                for task in self.node.latest_tasks:
                    task_agv_name = task.get('agv_name') if isinstance(task, dict) else getattr(task, 'agv_name', '')
                    task_status_id = task.get('status_id') if isinstance(task, dict) else getattr(task, 'status_id', 0)
                    if (task_agv_name == self.node.agv_name and
                        TaskStatus.is_task_executing_status(task_status_id)):
                        self.node.task = task
                        task_id = task.get('id') if isinstance(task, dict) else getattr(task, 'id', 0)
                        self.node.get_logger().info(f"✅ 進入 WaitRobot 前抓取任務 (status_id={task_status_id}): task_id={task_id}")
                        break

            # 重置機器人完成狀態，進入 WaitRobot 統一判斷
            self.node.robot_finished = False
            context.set_state(context.WaitRobotState(self.node))
        # 如果有路徑資料，則持續運行狀態
        if self.node.agv_status.AGV_PATH:

            if self.count > 100:
                self.count = 0
                self.node.get_logger().info(f"🏃 AGV RunningState... ")

            self.count += 1

        if self.node.agv_status.AGV_2POSITION:
            self.node.get_logger().info("✅ AGV 到達目標位置")

            # 在跳轉前先抓取 task，避免因為直接跳出而沒抓到
            # 使用 is_task_executing_status() 檢查所有執行中狀態 (2,4,12,14,22)
            # 注意：tasks 現在是 dict 列表
            task_found = False
            found_task_id = 0
            if self.node.latest_tasks:
                from shared_constants.task_status import TaskStatus
                for task in self.node.latest_tasks:
                    task_agv_name = task.get('agv_name') if isinstance(task, dict) else getattr(task, 'agv_name', '')
                    task_status_id = task.get('status_id') if isinstance(task, dict) else getattr(task, 'status_id', 0)
                    if (task_agv_name == self.node.agv_name and
                        TaskStatus.is_task_executing_status(task_status_id)):
                        self.node.task = task
                        found_task_id = task.get('id') if isinstance(task, dict) else getattr(task, 'id', 0)
                        self.node.get_logger().info(f"✅ 跳轉前抓取任務 (status_id={task_status_id}): task_id={found_task_id}")
                        task_found = True
                        break

            # 檢查 task id，如果為 0 則不進入 waitrobot
            # 注意：task 現在是 dict 格式
            current_task_id = self.node.task.get('id', 0) if isinstance(self.node.task, dict) else getattr(self.node.task, 'id', 0)
            if task_found and hasattr(self.node, 'task') and self.node.task and current_task_id != 0:
                self.node.robot_finished = False  # 重置機器人完成狀態
                context.set_state(context.WaitRobotState(self.node))
            else:
                # 沒有找到有效任務，回到任務選擇狀態重新評估
                self.node.get_logger().warn(
                    f"⚠️ 任務 ID 為 0 或沒有找到有效任務，回到任務選擇狀態"
                    f"\n  - latest_tasks 數量: {len(self.node.latest_tasks)}"
                    f"\n  - 當前 AGV Name: {self.node.agv_name}"
                    f"\n  - task_found: {task_found}"
                )
                # 增加診斷資訊：顯示前3個任務狀態
                if self.node.latest_tasks:
                    for idx, t in enumerate(self.node.latest_tasks[:3]):
                        t_id = t.get('id') if isinstance(t, dict) else getattr(t, 'id', 0)
                        t_agv_name = t.get('agv_name') if isinstance(t, dict) else getattr(t, 'agv_name', '')
                        t_status_id = t.get('status_id') if isinstance(t, dict) else getattr(t, 'status_id', 0)
                        self.node.get_logger().warn(
                            f"  - Task[{idx}] id={t_id}, agv_name={t_agv_name}, status_id={t_status_id}"
                        )

                # 跳轉回任務選擇狀態
                context.set_state(context.MissionSelectState(self.node))

         



    # tasks 改用 agv_node_base 的 Web API 輪詢取得

    def _find_current_task(self):
        """從全局 latest_tasks 找到當前 AGV 的任務"""
        if not self.node.latest_tasks or not hasattr(self.node, 'task') or not self.node.task:
            return None

        # 取得當前任務 ID（支援 dict 格式）
        current_task_id = self.node.task.get('id') if isinstance(self.node.task, dict) else getattr(self.node.task, 'id', 0)

        for task in self.node.latest_tasks:
            task_id = task.get('id') if isinstance(task, dict) else getattr(task, 'id', 0)
            if task_id == current_task_id:
                return task
        return None

    def _check_status_consistency(self):
        """檢查狀態一致性（Running 狀態應該是執行中狀態）

        執行中狀態包含（根據任務類型）:
        - From->To: 2 (From執行中), 4 (To執行中)
        - From Only: 12
        - To Only: 14
        - Path: 22

        注意: status=3 (FROM_COMPLETE) 是過渡狀態，不屬於執行中
        """
        from shared_constants.task_status import TaskStatus

        current_task = self._find_current_task()

        if not current_task:
            return

        # 取得任務屬性（支援 dict 格式）
        task_status_id = current_task.get('status_id') if isinstance(current_task, dict) else getattr(current_task, 'status_id', 0)
        task_id = current_task.get('id') if isinstance(current_task, dict) else getattr(current_task, 'id', 0)

        # 使用 is_task_executing_status() 檢查是否為任一執行中狀態
        if not TaskStatus.is_task_executing_status(task_status_id):
            status_desc = TaskStatus.get_description(task_status_id)
            self.node.get_logger().warn(
                f"⚠️ 狀態不一致！Running 狀態但 task.status={task_status_id} ({status_desc})，應為執行中狀態\n"
                f"  - task_id: {task_id}\n"
                f"  - 不自動修正，等待系統自然流程處理"
            )
            # 注意：不再自動修正狀態，因為狀態流程由 WritePathState 控制

    # 注意：已移除 _correct_task_status()、_status_correction_callback()、task_update_callback()
    # 狀態更新由 WritePathState 透過 Web API 處理，不再在 RunningState 自動修正