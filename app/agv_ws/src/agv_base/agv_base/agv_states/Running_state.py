from agv_base.states.state import State
from rclpy.node import Node
from db_proxy_interfaces.msg._tasks import Tasks



class RunningState(State):
    def __init__(self, node: Node):
        super().__init__(node)
        self.count = 0  # 計數器
        self.next_node = 0 #下一個點位
        self.ask_traffic_area = [] #詢問AGVC交管區域
        self.traffic_area_registed = [] #註冊的交管區域
        # 假設有一個 RobotContext 類別來管理機器人狀態
        
        # 【新增】任務訂閱相關變數，參考 mission_select_state
        self.subscription = None
        self.latest_tasks = []  # 儲存最新的任務資料

    def enter(self):
        self.node.get_logger().info("🏃 AGV 進入: Running 狀態")
        
        # 【新增】訂閱任務資料，參考 mission_select_state - QoS 必須與 Publisher 匹配
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

        # ⚠️ 重要：QoS 必須與 agvc_database_node 的 Publisher 匹配
        # Publisher 使用：RELIABLE + VOLATILE + depth=10
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,  # 修正：改為 VOLATILE 以匹配 Publisher
            depth=10  # 匹配 Publisher 的 depth
        )

        # 直接使用 node.create_subscription() 並手動管理訂閱
        subscription = self.node.create_subscription(Tasks, '/agvc/tasks', self.tasks_callback, qos_profile)
        self.subscriptions.append(subscription)
        self.node.get_logger().info("📡 已訂閱 /agvc/tasks (QoS: RELIABLE + VOLATILE)")

    def leave(self):
        self.node.get_logger().info("🚪 AGV 離開 Running 狀態")
        
        # 離開前將訂閱到的task表利用agv_id搜尋到當前執行的整個任務欄位丟進self.node.task
        # 新增 status_id 為 2 (READY_TO_EXECUTE) 或 3 (EXECUTING) 的判斷條件
        if self.latest_tasks:
            from shared_constants.task_status import TaskStatus
            for task in self.latest_tasks:
                if (task.agv_id == self.node.agv_id and
                    (task.status_id == TaskStatus.READY_TO_EXECUTE or
                     task.status_id == TaskStatus.EXECUTING)):
                    self.node.task = task
                    self.node.get_logger().info(f"⚠️ 離開前印出當前任務 (status_id={task.status_id}): " + str(self.node.task))
                    break
        
        # 移除訂閱
        self.remove_subscription()

    def handle(self, context):
        # self.node.get_logger().info("AGV Running 狀態")
        if not self.node.agv_status.AGV_PATH:
            self.node.get_logger().info("⚠️ AGV 沒有路徑資料，回到任務選擇狀態")
            
            # 在跳轉前先抓取 task，避免因為直接跳出而沒抓到
            # 新增 status_id 為 2 (READY_TO_EXECUTE) 或 3 (EXECUTING) 的判斷條件
            if self.latest_tasks:
                from shared_constants.task_status import TaskStatus
                for task in self.latest_tasks:
                    if (task.agv_id == self.node.agv_id and
                        (task.status_id == TaskStatus.READY_TO_EXECUTE or
                         task.status_id == TaskStatus.EXECUTING)):
                        self.node.task = task
                        self.node.get_logger().info(f"✅ 回到任務選擇前抓取任務 (status_id={task.status_id}): " + str(task.id))
                        break
            
            from agv_base.agv_states.mission_select_state import MissionSelectState
            context.set_state(MissionSelectState(self.node))
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
            if self.latest_tasks:
                from shared_constants.task_status import TaskStatus
                for task in self.latest_tasks:
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
                from agv_base.agv_states.wait_robot_state import WaitRobotState
                context.set_state(WaitRobotState(self.node))
            else:
                # 沒有找到有效任務，回到任務選擇狀態重新評估
                self.node.get_logger().warn(
                    f"⚠️ 任務 ID 為 0 或沒有找到有效任務，回到任務選擇狀態"
                    f"\n  - latest_tasks 數量: {len(self.latest_tasks)}"
                    f"\n  - 當前 AGV ID: {self.node.agv_id}"
                    f"\n  - task_found: {task_found}"
                )
                # 增加診斷資訊：顯示前3個任務狀態
                if self.latest_tasks:
                    for idx, t in enumerate(self.latest_tasks[:3]):
                        self.node.get_logger().warn(
                            f"  - Task[{idx}] id={t.id}, agv_id={t.agv_id}, status_id={t.status_id}"
                        )

                # 跳轉回任務選擇狀態
                from agv_base.agv_states.mission_select_state import MissionSelectState
                context.set_state(MissionSelectState(self.node))

    def tasks_callback(self, msg: Tasks):
        """任務資料回調函數"""
        tasks = msg.datas
        self.latest_tasks = tasks  # 儲存最新的任務資料
