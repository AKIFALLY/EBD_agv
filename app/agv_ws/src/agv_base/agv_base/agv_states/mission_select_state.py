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

    def enter(self):
        self.node.get_logger().info("AGV 進入: Mission Select")

        # 訂閱 task_table topic
        # self.create_subscription(String,'/task_table',self.task_table_callback)
        # 訂閱 task_table topic
        self.create_subscription(Tasks, '/agvc/tasks', self.tasks_callback)
        self.locamissiontimer = self.node.create_timer(1.0, self.local_mission)

    def leave(self):
        self.node.get_logger().info("AGV 離開 Mission Select 狀態")
        self.remove_subscription()  # 移除訂閱
        self.locamissiontimer.cancel()  # 取消timer

    def handle(self, context):
        # self.node.get_logger().info("AGV Mission Select 狀態")

        if self.count > 30:
            self.count = 0

            # 如果已經有路徑
            if self.node.agv_status.AGV_PATH:
                self.node.get_logger().info("AGV 已有路徑資料，離開 Mission Select 狀態")
                # 跳過任務選擇狀態，直接切換到下一個狀態
                from agv_base.agv_states.Running_state import RunningState
                context.set_state(RunningState(self.node))  # 切換狀態

            # 當已取得任務後，可選擇自動取消訂閱（或等 leave() 處理）
            if self.highest_priority_task:
                task = self.highest_priority_task
                self.node.get_logger().info(f"✅ 選擇任務: {task}")
                context.set_state(WritePathState(self.node))  # 切換狀態

            # 如果HMI有設定Magic跟終點設定
            if self.localMission and not self.node.agv_status.AGV_PATH:
                self.node.get_logger().info(
                    f"✅ HMI任務下達---  Magic:{self.node.agv_status.MAGIC}  Dest.:{self.node.agv_status.AGV_END_POINT}")
                context.set_state(WritePathState(self.node))  # 切換狀態

        self.count += 1

    def tasks_callback(self, msg: Tasks):
        tasks = msg.datas

        self.node.get_logger().info(f"收到 {len(tasks)} 個任務")

        # 篩選已執行卻未完成的任務 或是未執行但AGV已選擇
        running_tasks = [
            t for t in tasks
            if (t.status_id == 2 or t.status_id == 1) and t.agv_id == self.node.AGV_id
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
            return

        else:
            # ✅ 篩選符合未執行條件的任務
            filtered_tasks = [
                t for t in tasks
                if t.status_id == 1 and t.work_id >= 2000 and t.work_id < 3000 and t.agv_id == 0 and t.room_id == self.node.room_id
            ]

            if not filtered_tasks:
                # self.node.get_logger().warn("⚠️ 沒有符合條件的任務 (status_id=1 且 work_id<100)")
                return

            # ✅ 找出 priority 最大的那一筆
            self.highest_priority_task = max(filtered_tasks, key=lambda t: t.priority)
            self.node.mission_id = self.highest_priority_task.id
            self.node.node_id = self.highest_priority_task.node_id
            self.node.task = self.highest_priority_task
            self.node.get_logger().info(
                f"✅ 優先級最高任務: ID={self.highest_priority_task.id}, "
                f"WORK ID={self.highest_priority_task.work_id}, "
                f"Status={self.highest_priority_task.status_id}, "
                f"優先級={self.highest_priority_task.priority}, "
                f"名稱={self.highest_priority_task.name}"
                f"目標節點={self.highest_priority_task.node_id}"
            )

    def local_mission(self):
        # self.node.get_logger().info(f"(magic={self.node.agv_status.MAGIC}) dest.={self.node.agv_status.AGV_END_POINT}")
        # 判斷是否AGV_HMI路徑任務,判斷nMagci,From,To是否值大於0
        if self.node.agv_status.MAGIC > 0:
            if self.node.agv_status.AGV_END_POINT > 0:
                self.node.node_id = self.node.agv_status.AGV_END_POINT
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
2001	"LoaderAGV取入口傳送箱"	"從LoaderAGV取入口傳送箱拿到車上放"	"{}"
2011	"LoaderAGV放清洗機"	"從LoaderAGV車上放到清洗機"	"{}"
2021	"LoaderAGV取清洗機"	"從LoaderAGV取清洗機到車上放"	"{}"
2031	"LoaderAGV放泡藥機A"	"從LoaderAGV車上放到泡藥機A"	"{}"
2032	"LoaderAGV放泡藥機B"	"從LoaderAGV車上放到泡藥機B"	"{}"
2033	"LoaderAGV放泡藥機C"	"從LoaderAGV車上放到泡藥機C"	"{}"
2034	"LoaderAGV放泡藥機D"	"從LoaderAGV車上放到泡藥機D"	"{}"
2035	"LoaderAGV放泡藥機E"	"從LoaderAGV車上放到泡藥機E"	"{}"
2036	"LoaderAGV放泡藥機F"	"從LoaderAGV車上放到泡藥機F"	"{}"
2041	"LoaderAGV拿泡藥機A"	"從泡藥機A拿到LoaderAGV車上放"	"{}"
2042	"LoaderAGV拿泡藥機B"	"從泡藥機B拿到LoaderAGV車上放"	"{}"
2043	"LoaderAGV拿泡藥機C"	"從泡藥機C拿到LoaderAGV車上放"	"{}"
2044	"LoaderAGV拿泡藥機D"	"從泡藥機D拿到LoaderAGV車上放"	"{}"
2045	"LoaderAGV拿泡藥機E"	"從泡藥機E拿到LoaderAGV車上放"	"{}"
2046	"LoaderAGV拿泡藥機F"	"從泡藥機F拿到LoaderAGV車上放"	"{}"
2051	"LoaderAGV放預烘機1"	"從LoaderAGV車上放到預烘機1"	"{}"
2052	"LoaderAGV放預烘機2"	"從LoaderAGV車上放到預烘機2"	"{}"
2053	"LoaderAGV放預烘機3"	"從LoaderAGV車上放到預烘機3"	"{}"
2054	"LoaderAGV放預烘機4"	"從LoaderAGV車上放到預烘機4"	"{}"
2055	"LoaderAGV放預烘機5"	"從LoaderAGV車上放到預烘機5"	"{}"
2056	"LoaderAGV放預烘機6"	"從LoaderAGV車上放到預烘機6"	"{}"
2057	"LoaderAGV放預烘機7"	"從LoaderAGV車上放到預烘機7"	"{}"
2058	"LoaderAGV放預烘機8"	"從LoaderAGV車上放到預烘機8"	"{}"
"""
