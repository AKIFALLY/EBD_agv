import config.config as CONFIG
from db_proxy.models import Task
from datetime import datetime, timezone
from wcs_base.base_task_handler import BaseTaskHandler
from wcs_base.task_condition_checker import TaskConditionChecker
import json


class ManualEmptyRackRecyclingHandler(BaseTaskHandler):
    """
    人工空料架回收任務處理器

    功能：將人工回收空料架區的空料架搬運到系統空料架區

    判斷條件：
    1. 檢查人工回收空料架區是否有料架需要搬運
    2. 檢查空料架回收區是否有空位
    3. 檢查是否有重複的執行中任務
    """

    def __init__(self, node):
        super().__init__(node)  # 調用父類初始化，這會設置 db_manager

        # 任務相關變數
        self.kuka_rack_move_work = None  # 對應的workID資料
        self.task_node_id = None  # 目標位置的node_id（空料架回收區）
        self.source_location_id = None  # 來源位置（人工回收空料架區位置ID）
        self.target_location_id = None  # 目標位置（空料架回收區位置ID）

        # 人工回收空料架區狀態
        self.manual_recycle_empty_count = 0  # 人工回收區空位數量
        self.manual_recycle_place_count = 0  # 人工回收區佔用數量
        self.manual_recycle_empty_list = []  # 人工回收區空位列表
        self.manual_recycle_place_list = []  # 人工回收區佔用列表

        # 空料架回收區狀態
        self.empty_rack_recycle_empty_count = 0  # 空料架回收區空位數量
        self.empty_rack_recycle_place_count = 0  # 空料架回收區佔用數量
        self.empty_rack_recycle_empty_list = []  # 空料架回收區空位列表
        self.empty_rack_recycle_place_list = []  # 空料架回收區佔用列表

        # 條件檢查相關數據
        self.collected_data = {}  # 收集的條件檢查資料

        # 初始化條件檢查器（使用即時查詢模式）
        self.condition_checker = TaskConditionChecker(
            db_manager=self.db_manager,
            logger=self.node.get_logger(),
            real_time_mode=True,  # 啟用即時查詢模式
            query_timeout=30,     # 設定查詢超時時間
            max_iterations=50     # 設定最大迭代次數
        )

    def check_condition(self) -> bool:
        """
        基於 task_condition 表格的條件檢查

        使用 TaskConditionChecker 進行條件檢查，
        從 id=7 開始進行條件判斷流程。
        """
        try:
            # 使用條件檢查器進行檢查（人工空料架回收任務從 ID 7 開始）
            success, collected_data = self.condition_checker.check_conditions_from_id(start_id=7)

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
            # 條件7：人工回收空料架區的來源位置
            if '7_location' in self.collected_data:
                self.source_location_id = self.collected_data['7_location']
            elif 'location' in self.collected_data:
                # 如果是第一個條件的結果，通常是來源位置
                self.source_location_id = self.collected_data['location']

            # 條件8：空料架回收區的目標位置
            if '8_location' in self.collected_data:
                self.target_location_id = self.collected_data['8_location']
                self.task_node_id = self.target_location_id
            elif 'target_location' in self.collected_data:
                self.target_location_id = self.collected_data['target_location']
                self.task_node_id = self.target_location_id

            # 如果沒有明確的分離，嘗試從條件流程中推斷
            if not self.target_location_id and 'condition_results' in self.collected_data:
                condition_results = self.collected_data['condition_results']
                if '8' in condition_results and 'location' in condition_results['8']:
                    self.target_location_id = condition_results['8']['location']
                    self.task_node_id = self.target_location_id

            # 收集的資料
            self.node.get_logger().info(f"📊 收集到的所有資料: {self.collected_data}")
            self.node.get_logger().info(f"📍 來源位置: {self.source_location_id}, 目標位置: {self.target_location_id}")

        except Exception as e:
            self.node.get_logger().error(f"❌ 提取任務資料失敗: {e}")

    def insert_task(self) -> bool:
        """
        插入KUKA搬運人工回收空料架到系統空料架區的任務

        Returns:
            bool: 是否成功插入
        """
        try:
            # 獲取對應的work
            for work in self.db_manager.works:
                if work.id == CONFIG.KUKA_RACK_MOVE:
                    self.kuka_rack_move_work = work
                    break

            if not self.kuka_rack_move_work:
                self.node.get_logger().error(f"❌ 找不到 KUKA_RACK_MOVE work: {CONFIG.KUKA_RACK_MOVE}")
                return False

            # 確保必要資料都有
            if not self.source_location_id or not self.target_location_id:
                self.node.get_logger().error(f"❌ 缺少必要資料: source_location={self.source_location_id}, target_location={self.target_location_id}")
                return False

            # 取得取(人工回收空料架區)放(空料架回收區)的uuid
            uuid_nodes = [self.get_uuid(self.source_location_id), self.get_uuid(self.target_location_id)]
            self.node.get_logger().info(f"🔄 uuid_nodes: {uuid_nodes}")

            # 設定parameter
            self.kuka_rack_move_work.parameters["nodes"] = uuid_nodes
            self.kuka_rack_move_work.parameters["model"] = CONFIG.KUKA_MODLE_NAME

            # 創建任務
            with self.db_manager.get_session() as session:
                task_data = Task(
                    work_id=self.kuka_rack_move_work.id,
                    node_id=self.task_node_id,
                    name=f"KUKA搬運人工回收空料架到系統空料架區 人工回收區:{self.source_location_id}-->空料架回收區:{self.target_location_id}",
                    room_id=None,  # 這個任務不特定於某個房間
                    priority=CONFIG.PRIORITY_FOR_KUKA_MANUAL_EMPTY_RACK_RECYCLING,
                    status_id=CONFIG.WCS_STATUS,
                    created_at=datetime.now(timezone.utc),
                    updated_at=datetime.now(timezone.utc),
                    parameters=self.kuka_rack_move_work.parameters
                )

                # 任務創建結果取出
                self.create_task_result = self.create_task(session, task_data)
                if self.create_task_result:
                    self.task_inserted = True
                    self.node.get_logger().info(f"✅ KUKA搬運人工回收空料架到系統空料架區任務插入成功 - 任務ID: {self.create_task_result.id}")
                    return True
                else:
                    self.node.get_logger().error("❌ KUKA搬運人工回收空料架到系統空料架區任務插入失敗")
                    return False

        except Exception as e:
            self.node.get_logger().error(f"❌ 插入KUKA搬運人工回收空料架到系統空料架區任務失敗: {e}")
            return False

    def check_insert_done(self) -> bool:
        """
        檢查任務插入是否成功

        Returns:
            bool: 是否插入成功
        """
        try:
            with self.db_manager.get_session() as session:
                task = self.get_task_by_id(session, self.create_task_result.id)
                if task:
                    self.node.get_logger().info(f"✅ KUKA搬運人工回收空料架到系統空料架區任務確認存在 - 任務ID: {self.create_task_result.id}")
                    # 重置狀態，準備下一次檢查
                    self.find_task = False
                    self.task_inserted = False
                    return True
                else:
                    self.node.get_logger().warn(f"⚠️ KUKA搬運人工回收空料架到系統空料架區任務不存在 - 任務ID: {self.create_task_result.id}")
                    # 重置狀態，重新嘗試
                    self.find_task = False
                    self.task_inserted = False
                    return False

        except Exception as e:
            self.node.get_logger().error(f"❌ 檢查KUKA搬運人工回收空料架到系統空料架區任務插入狀態失敗: {e}")
            # 重置狀態
            self.find_task = False
            self.task_inserted = False
            return False