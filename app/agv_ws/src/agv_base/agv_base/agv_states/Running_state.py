from agv_base.states.state import State
from rclpy.node import Node
from db_proxy_interfaces.msg._tasks import Tasks
from db_proxy.agvc_database_client import AGVCDatabaseClient



class RunningState(State):
    def __init__(self, node: Node):
        super().__init__(node)
        self.agvdbclient = AGVCDatabaseClient(node)
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

        # 離開前將訂閱到的task表利用agv_id搜尋到當前執行的整個任務欄位丟進self.node.task
        # 新增 status_id 為 2 (READY_TO_EXECUTE) 或 3 (EXECUTING) 的判斷條件
        if self.node.latest_tasks:
            from shared_constants.task_status import TaskStatus
            for task in self.node.latest_tasks:
                if (task.agv_id == self.node.agv_id and
                    (task.status_id == TaskStatus.READY_TO_EXECUTE or
                     task.status_id == TaskStatus.EXECUTING)):
                    self.node.task = task
                    self.node.get_logger().info(f"⚠️ 離開前印出當前任務 (status_id={task.status_id}): " + str(self.node.task))
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
            # 新增 status_id 為 2 (READY_TO_EXECUTE) 或 3 (EXECUTING) 的判斷條件
            if self.node.latest_tasks:
                from shared_constants.task_status import TaskStatus
                for task in self.node.latest_tasks:
                    if (task.agv_id == self.node.agv_id and
                        (task.status_id == TaskStatus.READY_TO_EXECUTE or
                         task.status_id == TaskStatus.EXECUTING)):
                        self.node.task = task
                        self.node.get_logger().info(f"✅ 進入 WaitRobot 前抓取任務 (status_id={task.status_id}): " + str(task.id))
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
            # 新增 status_id 為 2 (READY_TO_EXECUTE) 或 3 (EXECUTING) 的判斷條件
            task_found = False
            if self.node.latest_tasks:
                from shared_constants.task_status import TaskStatus
                for task in self.node.latest_tasks:
                    if (task.agv_id == self.node.agv_id and
                        (task.status_id == TaskStatus.READY_TO_EXECUTE or
                         task.status_id == TaskStatus.EXECUTING)):
                        self.node.task = task
                        self.node.get_logger().info(f"✅ 跳轉前抓取任務 (status_id={task.status_id}): " + str(task.id))
                        task_found = True
                        break
            
            # 檢查 task id，如果為 0 則不進入 waitrobot
            if task_found and hasattr(self.node, 'task') and self.node.task and self.node.task.id != 0:
                self.node.robot_finished = False  # 重置機器人完成狀態
                context.set_state(context.WaitRobotState(self.node))
            else:
                # 沒有找到有效任務，回到任務選擇狀態重新評估
                self.node.get_logger().warn(
                    f"⚠️ 任務 ID 為 0 或沒有找到有效任務，回到任務選擇狀態"
                    f"\n  - latest_tasks 數量: {len(self.node.latest_tasks)}"
                    f"\n  - 當前 AGV ID: {self.node.agv_id}"
                    f"\n  - task_found: {task_found}"
                )
                # 增加診斷資訊：顯示前3個任務狀態
                if self.node.latest_tasks:
                    for idx, t in enumerate(self.node.latest_tasks[:3]):
                        self.node.get_logger().warn(
                            f"  - Task[{idx}] id={t.id}, agv_id={t.agv_id}, status_id={t.status_id}"
                        )

                # 跳轉回任務選擇狀態
                context.set_state(context.MissionSelectState(self.node))

         



    # tasks_callback 已移除，改用 agv_node_base 的全局訂閱
    # 全局回調會自動更新 self.node.latest_tasks 和 self.node.last_tasks_callback_time

    def _find_current_task(self):
        """從全局 latest_tasks 找到當前 AGV 的任務"""
        if not self.node.latest_tasks or not hasattr(self.node, 'task') or not self.node.task:
            return None

        for task in self.node.latest_tasks:
            if task.id == self.node.task.id:
                return task
        return None

    def _check_status_consistency(self):
        """檢查狀態一致性（Running 狀態應該是 EXECUTING=3）"""
        from shared_constants.task_status import TaskStatus

        current_task = self._find_current_task()

        if not current_task:
            return

        # 檢查狀態是否一致，不一致則直接修正
        if current_task.status_id != TaskStatus.EXECUTING:
            self.node.get_logger().warn(
                f"⚠️ 狀態不一致！Running 狀態但 task.status={current_task.status_id}，應為 3 (EXECUTING)\n"
                f"  - task_id: {current_task.id}\n"
                f"  - 立即執行修正"
            )
            self._correct_task_status()

    def _correct_task_status(self):
        """修正任務狀態為 EXECUTING (3)"""
        from shared_constants.task_status import TaskStatus

        self.node.task.status_id = TaskStatus.EXECUTING
        self.agvdbclient.async_update_task(
            self.node.task,
            self._status_correction_callback
        )
        self.node.get_logger().info(
            f"🔧 自動修正任務狀態為 EXECUTING (task_id={self.node.task.id})"
        )

    def _status_correction_callback(self, response):
        """狀態修正回調"""
        if response is None:
            self.node.get_logger().error("❌ 狀態修正未收到回應")
            return

        if response.success:
            self.node.get_logger().info(f"✅ 狀態修正成功: {response.message}")
        else:
            self.node.get_logger().error(f"❌ 狀態修正失敗: {response.message}，將在下次檢查時重試")


    def task_update_callback(self,response):
        if response is None:
            self.node.get_logger().error("❌ 未收到任務更新的回應（可能逾時或錯誤）", throttle_duration_sec=1.0)
            return

        if response.success:
            self.node.get_logger().info(f"✅ 任務更新成功，訊息: {response.message}")
        else:
            self.node.get_logger().error(f"⚠️ 任務更新失敗，訊息: {response.message}")