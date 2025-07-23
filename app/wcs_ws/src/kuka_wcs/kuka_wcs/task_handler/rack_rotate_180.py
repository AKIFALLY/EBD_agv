from db_proxy.models import Task
from datetime import datetime, timezone
from wcs_base.base_task_handler import BaseTaskHandler
from wcs_base.task_condition_checker import TaskConditionChecker
import config.config as CONFIG
import json

class RackRotate180Handler(BaseTaskHandler):
    def __init__(self, node):
        super().__init__(node)  # 調用父類初始化，這會設置 db_manager

        self.find_task = False  # 找到任任務標記
        self.task_inserted = False  # 插入任務中

        self.agv_rotate_context = None  # 有旋轉需求的Context
        self.task = None  # 需發送旋轉貨架任務
        self.parent_task_id = None  # 父任務id
        self.rotate_task = None  # 旋轉任務
        self.kuka_rack_move_work_parameter = None  # 對應的work的parameter

        self.system_empty_rack_count = 0  # 系統空車區空位數量
        self.system_empty_rack_place_count = 0  # 系統空車區佔用數量
        self.system_empty_rack_list = []  # 系統空車區空位列表
        self.system_empty_rack_place_list = []  # 系統空車區佔用列表

        # 條件檢查相關數據
        self.collected_data = {}  # 收集的條件檢查資料

        # 初始化條件檢查器（使用即時查詢模式）
        self.condition_checker = TaskConditionChecker(
            db_manager=self.db_manager,
            logger=self.node.get_logger(),
            real_time_mode=True,  # 啟用即時查詢模式
            query_timeout=30,     # 設定查詢超時時間
            max_iterations=10     # 設定最大迭代次數
        )


    
    """
    =====================判斷是否有rack180度轉向需求===========================   
    1.檢查AGV的context中,是否有state為wait_rotation_state的AGV
    再去task表中
    2.找出跟context中相同的agv_id且id 沒有在parent_task_id_list的任務
    """

    def check_condition(self):
        """
        基於 task_condition 表格的條件檢查

        使用 TaskConditionChecker 進行條件檢查，
        從 id=1 開始進行條件判斷流程。
        """
        try:
            # 使用條件檢查器進行檢查 (料架180度旋轉任務從 ID 3 開始)
            success, collected_data = self.condition_checker.check_conditions_from_id(start_id=3)

            if success:
                self.collected_data = collected_data
                self._extract_task_data_from_collected()
                self.find_task = True
                return True
            return False

        except Exception as e:
            self.node.get_logger().error(f"❌ 條件檢查失敗: {e}")
            return False

    def _extract_task_data_from_collected(self):
        """
        從收集的資料中提取任務相關資訊
        """
        try:
            # 從收集的資料中提取相關資訊
            if 'agv_id' in self.collected_data:
                # 根據 agv_id 找到對應的 context 和 task
                agv_id = self.collected_data['agv_id']
                for nAgvContext in self.db_manager.agv_contexts:
                    if nAgvContext.agv_id == agv_id:
                        self.agv_rotate_context = nAgvContext
                        break

                for nTask in self.db_manager.tasks:
                    if nTask.agv_id == agv_id and nTask.id not in self.db_manager.parent_task_ids:
                        self.task = nTask
                        self.parent_task_id = nTask.id
                        break

            # 收集的資料
            self.node.get_logger().info(f"📊 收集到的所有資料: {self.collected_data}")

        except Exception as e:
            self.node.get_logger().error(f"❌ 提取任務資料失敗: {e}")

    def insert_task(self):
        """插入旋轉任務"""
        try:
            if self.find_task and not self.task_inserted:

                # 設定parameter
                # roate不需要設定nodes,因為從父任務直接複製task裡面的parameter
                self.kuka_rack_move_work_parameter = self.task.parameters
                self.kuka_rack_move_work_parameter["model"] = CONFIG.KUKA_MODLE_NAME  # 設定機器名稱

                # 創建任務
                
                with self.db_manager.get_session() as session:
                    task_data = Task(
                        parent_task_id=self.parent_task_id,
                        work_id=CONFIG.KUKA_RACK_MOVE,
                        status_id=1,
                        room_id=self.task.room_id,
                        node_id=self.task.node_id,
                        name=f"KUKA AGV 貨架轉向任務--room_id:{self.task.room_id}",
                        description="KUKA AGV 貨架轉向任務",
                        mission_code="null",
                        priority=CONFIG.PRIORITY_FOR_RACK_ROTATE,
                        parameters=self.kuka_rack_move_work_parameter,
                        created_at=datetime.now(timezone.utc),
                        updated_at=None
                    )
                    #任務創建結果取出
                    self.create_task_result = self.create_task(session, task_data)
                    if self.create_task_result:
                        self.task_inserted = True
                        self.node.get_logger().info(f"🔄KUKA 180度轉向任務插入中......任務id:{self.create_task_result.id}")
                        return True

        except Exception as e:
            self.node.get_logger().error(f"❌ 插入旋轉任務失敗: {e}")




    def check_insert_done(self):
        """
        檢查任務插入是否成功
        """
        try:
            with self.db_manager.get_session() as session:
                task = self.get_task_by_id(session, self.create_task_result.id)
                if task:
                    self.node.get_logger().info(f"✅KUKA 180度旋轉任務插入檢查完成-任務id:{self.task.id}")
                    self.find_task = False
                    self.task_inserted = False
                    return True
                return False
        except Exception as e:
            self.node.get_logger().error(f"	❌KUKA 180度旋轉任務插入檢查失敗: {e}")
            return False