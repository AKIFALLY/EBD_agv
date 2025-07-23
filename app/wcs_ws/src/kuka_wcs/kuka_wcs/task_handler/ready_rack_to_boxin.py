import config.config as CONFIG
from db_proxy.models import Task
from datetime import datetime, timezone
from wcs_base.base_task_handler import BaseTaskHandler
from wcs_base.task_condition_checker import TaskConditionChecker
import json


class ReadyRackToBoxinHandler(BaseTaskHandler):
    """
    準備區料架送往入口傳送箱任務處理器

    功能：將系統準備區的料架搬運到入口傳送箱

    判斷條件：
    1. 系統準備區需要有料架 (ready_rack_place_count > 0)
    2. boxin 位置不可以有其他料架
    3. 不可以有重複的任務
    """

    def __init__(self, node):
        super().__init__(node)  # 調用父類初始化，這會設置 db_manager

        # 任務相關變數
        self.kuka_rack_move_work = None  # 對應的workID資料
        self.task_node_id = None  # 搬運料架到入口傳送箱的node_id
        self.task_room_id = None  # 搬運料架到入口傳送箱的room_id

        # 系統準備區狀態
        self.ready_rack_empty_count = 0  # 系統準備區空位數量
        self.ready_rack_place_count = 0  # 系統準備區佔用數量
        self.ready_rack_empty_list = []  # 系統準備區空位列表
        self.ready_rack_place_list = []  # 系統準備區佔用列表

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

    def check_condition(self) -> bool:
        """
        基於 task_condition 表格的條件檢查

        使用 TaskConditionChecker 進行條件檢查，
        從 id=1 開始進行條件判斷流程。
        """
        try:
            # 使用條件檢查器進行檢查 (準備區料架到入口傳送箱任務從 ID 4 開始)
            success, collected_data = self.condition_checker.check_conditions_from_id(start_id=4)

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
            if 'task_node_id' in self.collected_data:
                self.task_node_id = self.collected_data['task_node_id']
            if 'task_room_id' in self.collected_data:
                self.task_room_id = self.collected_data['task_room_id']

            # 收集的資料
            self.node.get_logger().info(f"📊 收集到的所有資料: {self.collected_data}")

        except Exception as e:
            self.node.get_logger().error(f"❌ 提取任務資料失敗: {e}")

    def insert_task(self) -> bool:
        """
        插入準備區料架送往入口傳送箱任務

        Returns:
            bool: 是否成功插入
        """
        try:
            # 獲取對應的work
            for work in self.db_manager.works:
                if work.id == CONFIG.KUKA_RACK_MOVE:
                    self.kuka_rack_move_work = work
                    break
            # 取得取(系統準備區)放(入口傳送箱)的uuid
            self.boxin_node_id = self.convert_to_node_id(self.task_room_id, "出口傳送箱")#將room轉換成node_id
            uuid_nodes = [self.get_uuid(self.ready_rack_place_list[0]),self.get_uuid(self.boxin_node_id)]#取得取放的uuid
            self.node.get_logger().info(f"🔄uuid_nodes:{uuid_nodes}")

            if not self.kuka_rack_move_work:
                self.node.get_logger().error(f"❌ 找不到 KUKA_RACK_MOVE work: {CONFIG.KUKA_RACK_MOVE}")
                return False

            #設定parameter
            self.kuka_rack_move_work.parameters["nodes"] = uuid_nodes#設定nodes
            self.kuka_rack_move_work.parameters["model"] = CONFIG.KUKA_MODLE_NAME#設定機器名稱


            # 創建任務
            with self.db_manager.get_session() as session:
                task_data = Task(
                    work_id=self.kuka_rack_move_work.id,
                    node_id=self.task_node_id,
                    name=f"系統準備區搬運道入口傳送區 系統準備區:{self.ready_rack_place_list[0]}-->房間:{self.task_room_id}",
                    room_id=self.task_room_id,
                    priority=CONFIG.PRIORITY_FOR_KUKA_FROM_READY_TO_BOXIN,  # 使用適當的優先級
                    status_id=CONFIG.WCS_STATUS,
                    created_at=datetime.now(timezone.utc),
                    updated_at=datetime.now(timezone.utc),
                    parameters=self.kuka_rack_move_work.parameters
                )

                # 任務創建結果取出
                self.create_task_result = self.create_task(session, task_data)
                if self.create_task_result:
                    self.task_inserted = True
                    self.node.get_logger().info(f"✅ 準備區料架送往入口傳送箱任務插入成功 - 任務ID: {self.create_task_result.id}")
                    return True
                else:
                    self.node.get_logger().error("❌ 準備區料架送往入口傳送箱任務插入失敗")
                    return False

        except Exception as e:
            self.node.get_logger().error(f"❌ 插入準備區料架送往入口傳送箱任務失敗: {e}")
            return False

    def check_insert_done(self) -> bool:
        """
        檢查任務插入是否成功

        Returns:
            bool: 是否插入成功
        """
        try:
            # 檢查任務是否已經在資料庫中
            with self.db_manager.get_session() as session:
                task = self.get_task_by_id(session, self.create_task_result.id)
                if task:
                    self.node.get_logger().info(f"✅ 準備區料架送往入口傳送箱任務確認存在 - 任務ID: {self.create_task_result.id}")
                    # 重置狀態，準備下一次檢查
                    self.find_task = False
                    self.task_inserted = False
                    return True
                else:
                    self.node.get_logger().warn(f"⚠️ 準備區料架送往入口傳送箱任務不存在 - 任務ID: {self.create_task_result.id}")
                    # 重置狀態，重新嘗試
                    self.find_task = False
                    self.task_inserted = False
                    return False

        except Exception as e:
            self.node.get_logger().error(f"❌ 檢查準備區料架送往入口傳送箱任務插入狀態失敗: {e}")
            # 重置狀態
            self.find_task = False
            self.task_inserted = False
            return False
    
