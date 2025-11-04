from agv_base.states.state import State
from db_proxy_interfaces.msg._tasks import Tasks
from agv_base.agv_states.write_path_state import WritePathState
import json
from std_msgs.msg import String
from rclpy.node import Node
import time


class MissionSelectState(State):
    def __init__(self, node: Node):
        super().__init__(node)
        self.node = node
        self.startpoint = None  # 起點
        self.endpoint = None  # 終點
        self.subscription = None
        self.task_list = []
        self.subscribed = False
        self.highest_priority_task = []
        self.count = 0  # 計數器，用於執行次數
        self.localMission = False  # 觸發Local端任務旗標
        self.latest_tasks = []  # 儲存最新的任務資料
        self.status_log_count = 0  # 狀態日誌計數器（每5秒輸出一次）

    def enter(self):
        self.node.get_logger().info("🎯 AGV 進入: Mission Select")

        # 訂閱 task_table topic
        # self.create_subscription(String,'/task_table',self.task_table_callback)
        # 訂閱 task_table topic - QoS 必須與 Publisher 匹配
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
        self.locamissiontimer = self.node.create_timer(1.0, self.local_mission)

    def leave(self):
        self.node.get_logger().info("🚪 AGV 離開 Mission Select 狀態")
        self.remove_subscription()  # 移除訂閱
        self.locamissiontimer.cancel()  # 取消timer

    def handle(self, context):
        #self.node.get_logger().info("AGV Mission Select 狀態")
        #self.node.get_logger().info(f"Task列表:{self.latest_tasks}")
        
        

        if self.count > 30:
            self.count = 0

            # 🔍 【新增】在檢查離開條件之前，先確保從 task table 中搜尋該 AGV 的任務資料
            if self.latest_tasks and len(self.latest_tasks) > 0:
                has_task = self._process_tasks(self.latest_tasks)
                if not has_task and not hasattr(self.node, 'task'):
                    # 沒有找到任務且 node.task 也不存在，記錄警告
                    self.node.get_logger().debug("🔍 未找到屬於該 AGV 的任務")
                elif has_task:
                    self.node.get_logger().debug(f"🔍 確認任務資料: task_id={getattr(self.node.task, 'id', 'None')}")

            # 如果已經有路徑
            if self.node.agv_status.AGV_PATH:
                # ⚠️ 【改善】檢查是否有任務資料，避免路徑有資料但任務無資料的情況
                if hasattr(self.node, 'task') and self.node.task:
                    self.node.get_logger().info(f"✅ AGV 已有路徑資料且有任務資料 (task_id={self.node.task.id})，離開 Mission Select 狀態")
                    self.node.get_logger().info(f"✅ AGV 已有路徑資料且有任務資料 (task_id={self.node.task.id})，離開 Mission Select 狀態")
                    from agv_base.agv_states.Running_state import RunningState
                    context.set_state(RunningState(self.node))  # 切換狀態
                else:
                    self.node.get_logger().warn("⚠️ AGV 有路徑但無任務資料，等待任務分配")

            # 當已取得任務後，可選擇自動取消訂閱（或等 leave() 處理）
            elif self.highest_priority_task:
                task = self.highest_priority_task
                self.node.get_logger().info(f"✅ 選擇任務: {task}")
                context.set_state(WritePathState(self.node))  # 切換狀態

            # 如果HMI有設定Magic跟終點設定
            elif self.localMission and not self.node.agv_status.AGV_PATH:
                self.node.get_logger().info(
                    f"✅ HMI任務下達---  Magic:{self.node.agv_status.MAGIC}  Dest.:{self.node.agv_status.AGV_END_POINT}")
                context.set_state(WritePathState(self.node))  # 切換狀態

        self.count += 1

    def tasks_callback(self, msg: Tasks):
        tasks = msg.datas
        self.latest_tasks = tasks  # 儲存最新的任務資料

        self.node.get_logger().info(f"📦 收到 {len(tasks)} 個任務")

        # 處理任務篩選
        self._process_tasks(tasks)
        
    def _process_tasks(self, tasks):
        """處理任務篩選邏輯"""
        # 篩選已執行卻未完成的任務 或是未執行但AGV已選擇
        # 新增 status_id 為 2 (READY_TO_EXECUTE) 或 3 (EXECUTING) 的判斷條件
        from shared_constants.task_status import TaskStatus
        running_tasks = [
            t for t in tasks
            if (t.status_id == TaskStatus.READY_TO_EXECUTE or
                t.status_id == TaskStatus.EXECUTING or
                t.status_id == TaskStatus.PENDING) and t.agv_id == self.node.agv_id
        ]

        if len(running_tasks) > 0:
            self.node.get_logger().info("⚠️ 有正在執行的任務")
            self.node.mission_id = running_tasks[0].id
            self.node.node_id = running_tasks[0].node_id
            self.highest_priority_task = running_tasks[0]
            self.node.task = running_tasks[0]
            self.node.get_logger().info(f"✅ 任務ID: {running_tasks[0].id}, "
                                        f"WORK ID: {running_tasks[0].work_id}, "
                                        f"Status: {running_tasks[0].status_id}, "
                                        f"優先級: {running_tasks[0].priority}, "
                                        f"名稱: {running_tasks[0].name}, "
                                        f"目標節點: {running_tasks[0].node_id}")
            return True  # 找到任務

        else:
            # ⚠️ 沒有找到已分配給此 AGV 的任務
            # 任務分配應由 RCS 負責，AGV 只接收已分配的任務
            self.node.get_logger().debug("📭 未查詢到已分配給此 AGV 的任務，等待 RCS 派發")
            return False  # 沒找到任務

    def local_mission(self):
        # self.node.get_logger().info(f"(magic={self.node.agv_status.MAGIC}) dest.={self.node.agv_status.AGV_END_POINT}")
        # 判斷是否AGV_HMI路徑任務,判斷nMagci,From,To是否值大於0
        # 安全檢查：確保 MAGIC 和 AGV_END_POINT 不是 None
        magic = self.node.agv_status.MAGIC if self.node.agv_status.MAGIC is not None else 0
        end_point = self.node.agv_status.AGV_END_POINT if self.node.agv_status.AGV_END_POINT is not None else 0
        auto = self.node.agv_status.AGV_Auto if self.node.agv_status.AGV_Auto is not None else 0
        local = self.node.agv_status.AGV_LOCAL if self.node.agv_status.AGV_LOCAL is not None else 0
        if magic > 0 and end_point > 0 and auto == 1 and local == 1:
            self.node.node_id = end_point
            self.localMission = True


"""
uint64 id
uint64 work_id
uint64 status_id
uint64 room_id
string name
string description
uint64 agv_id
string agv_name
uint8 priority
string parameters  # JSON string
string created_at
string updated_at
"""




"""
task_status
id	name	description
0	請求中	UI-請求執行任務
1	待處理	WCS-任務已接受，待處理
2	待執行	RCS-任務已派發，待執行
3	執行中	AGV-任務正在執行
4	已完成	AGV-任務已完成
5	取消中	任務取消中
6	錯誤	錯誤
"""