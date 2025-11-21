from agv_base.states.state import State
from db_proxy_interfaces.msg._tasks import Tasks
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
        # self.latest_tasks 已移至 node (全局共享)
        self.status_log_count = 0  # 狀態日誌計數器（每5秒輸出一次）
        # self.last_callback_time 已移至 node (全局共享)
        self.callback_timeout_s = 10.0  # tasks_callback 超時時間（秒）
        self.timeout_warning_shown = False  # 是否已顯示超時警告（避免重複輸出）
        self.enter_state_time = None  # 進入狀態的時間

    def enter(self):
        self.node.get_logger().info("🎯 AGV 進入: Mission Select")

        # 初始化超時檢測機制（使用全局時間戳）
        self.timeout_warning_shown = False
        self.enter_state_time = time.time()  # 記錄進入狀態時間
        self.node.get_logger().info("📡 使用全局 /agvc/tasks 訂閱（已在 agv_node_base 建立）")

        # 建立 local mission timer
        self.locamissiontimer = self.node.create_timer(1.0, self.local_mission)

    def leave(self):
        self.node.get_logger().info("🚪 AGV 離開 Mission Select 狀態")
        # 全局訂閱不需移除，由 agv_node_base 管理
        self.locamissiontimer.cancel()  # 取消timer

    def handle(self, context):
        #self.node.get_logger().info("AGV Mission Select 狀態")
        #self.node.get_logger().info(f"Task列表:{self.latest_tasks}")

        # ⏱️ 檢查 tasks_callback 超時（使用全局時間戳）
        current_time = time.time()

        if self.node.last_tasks_callback_time is not None:
            # 情況 1: 曾經收到過訂閱，檢查是否超時
            elapsed_time = current_time - self.node.last_tasks_callback_time
            if elapsed_time > self.callback_timeout_s:
                if not self.timeout_warning_shown:
                    self.node.get_logger().error(
                        f"❌ tasks_callback 超時！已經 {elapsed_time:.1f} 秒未收到任務資料回應\n"
                        f"  - 超時閾值: {self.callback_timeout_s} 秒\n"
                        f"  - 訂閱 Topic: /agvc/tasks (全局訂閱)\n"
                        f"  - 建議檢查: agvc_database_node 是否正常運行"
                    )
                    self.timeout_warning_shown = True  # 設置標記，避免重複輸出
        elif self.enter_state_time is not None:
            # 情況 2: 從未收到過訂閱，檢查進入狀態後是否超時
            elapsed_since_enter = current_time - self.enter_state_time
            if elapsed_since_enter > self.callback_timeout_s:
                if not self.timeout_warning_shown:
                    self.node.get_logger().error(
                        f"❌ tasks_callback 超時！進入 MissionSelect 後 {elapsed_since_enter:.1f} 秒從未收到任務資料\n"
                        f"  - 超時閾值: {self.callback_timeout_s} 秒\n"
                        f"  - 訂閱 Topic: /agvc/tasks (全局訂閱)\n"
                        f"  - 建議檢查: agvc_database_node 是否正常運行"
                    )
                    self.timeout_warning_shown = True  # 設置標記，避免重複輸出


        if self.count > 30:
            self.count = 0

            # 🔍 【新增】在檢查離開條件之前，先確保從 task table 中搜尋該 AGV 的任務資料
            if self.node.latest_tasks and len(self.node.latest_tasks) > 0:
                has_task = self._process_tasks(self.node.latest_tasks)
                if not has_task and not hasattr(self.node, 'task'):
                    # 沒有找到任務且 node.task 也不存在，記錄警告
                    self.node.get_logger().debug("🔍 未找到屬於該 AGV 的任務")
                elif has_task:
                    self.node.get_logger().debug(f"🔍 確認任務資料: task_id={getattr(self.node.task, 'id', 'None')}")

            # 🔒 【狀態轉換守衛】檢查 Base 層狀態，只有在 Auto 狀態時才允許 AGV 層狀態轉換
            if not self._is_base_auto_state():
                base_state_name = self.node.base_context.state.__class__.__name__
                self.node.get_logger().debug(
                    f"🔒 AGV 層狀態轉換被阻止: Base 層不在 Auto 狀態 (當前: {base_state_name})")
                self.count += 1
                return



            # 如果已經有路徑
            if self.node.agv_status.AGV_PATH:
                # ⚠️ 【改善】檢查是否有有效任務資料（task_id 不能為 0）
                if (hasattr(self.node, 'task') and self.node.task and
                    hasattr(self.node.task, 'id') and self.node.task.id != 0):
                    self.node.get_logger().info(f"✅ AGV 已有路徑資料且有任務資料 (task_id={self.node.task.id})，離開 Mission Select 狀態")

                    context.set_state(context.RunningState(self.node))  # 切換狀態
                else:
                    self.node.get_logger().warn("⚠️ AGV 有路徑但無有效任務資料（task_id=0 或無任務），等待任務分配")

            # 當已取得任務後，可選擇自動取消訂閱（或等 leave() 處理）
            elif self.highest_priority_task:
                task = self.highest_priority_task
                from shared_constants.task_status import TaskStatus

                # 檢查任務狀態：status=3 且無路徑 → 根據 MISSION_CANCEL 決定行為
                if task.status_id == TaskStatus.EXECUTING and not self.node.agv_status.AGV_PATH:
                    # 如果 MISSION_CANCEL=1，進入 WritePathState 重新規劃路徑
                    if self.node.agv_status.MISSION_CANCEL == 1:
                        self.node.get_logger().info(
                            f"🔄 任務取消標記啟動 (task_id={task.id}, MISSION_CANCEL=1)，進入 WritePathState 重新規劃路徑"
                        )
                        context.set_state(context.WritePathState(self.node))
                    else:
                        # MISSION_CANCEL≠1，進入 WaitRobot 統一判斷
                        self.node.get_logger().info(
                            f"⚠️ 任務執行中但無路徑 (task_id={task.id}, status=3)，進入 WaitRobot 統一判斷"
                        )
                        self.node.robot_finished = False  # 重置機器人完成狀態
                        context.set_state(context.WaitRobotState(self.node))
                else:
                    # status=1,2 或其他情況 → 正常寫路徑
                    self.node.get_logger().info(f"✅ 選擇任務 (status={task.status_id}): {task}")
                    context.set_state(context.WritePathState(self.node))  # 切換狀態

            # 如果HMI有設定Magic跟終點設定
            elif self.localMission and not self.node.agv_status.AGV_PATH:
                self.node.get_logger().info(
                    f"✅ HMI任務下達---  Magic:{self.node.agv_status.MAGIC}  Dest.:{self.node.agv_status.AGV_END_POINT}")
                context.set_state(context.WritePathState(self.node))  # 切換狀態

        self.count += 1

    def _is_base_auto_state(self) -> bool:
        """
        檢查 Base 層是否在 Auto 狀態

        Returns:
            bool: Base 層在 Auto 狀態時返回 True，否則返回 False
        """
        from agv_base.states.auto_state import AutoState
        return isinstance(self.node.base_context.state, AutoState)



    # tasks_callback 已移除，改用 agv_node_base 的全局訂閱
    # 全局回調會自動更新 self.node.latest_tasks 和 self.node.last_tasks_callback_time
        
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