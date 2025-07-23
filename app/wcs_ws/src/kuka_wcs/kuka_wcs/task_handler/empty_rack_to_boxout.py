import config.config as CONFIG
from db_proxy.models import Task
from datetime import datetime, timezone
from wcs_base.base_task_handler import BaseTaskHandler
from wcs_base.task_condition_checker import TaskConditionChecker
import json

class EmptyRackToBoxoutHandler(BaseTaskHandler):
    """
    空架到出口傳送箱任務處理器
    處理將空料架搬運到出口傳送箱的任務
    """

    def __init__(self, node):
        super().__init__(node)

        # 任務相關數據
        self.kuka_rack_move_work = None  # 對應的workID資料
        self.empty_to_outbox_task = None  # 創建的任務
        self.task_node_id = None  # 目標節點ID
        self.task_room_id = None  # 目標房間ID

        # 系統空車區狀態
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
            max_iterations=50     # 設定最大迭代次數
        )

    def check_condition(self):
        """
        基於 task_condition 表格的條件檢查

        使用 TaskConditionChecker 進行條件檢查，
        從 id=1 開始進行條件判斷流程。
        """
        try:
            # 使用條件檢查器進行檢查
            success, collected_data = self.condition_checker.check_conditions_from_id(start_id=1)

            if success:
                # 儲存收集到的資料
                self.collected_data = collected_data

                # 從收集的資料中提取任務所需資訊
                self._extract_task_data_from_collected()

                self.node.get_logger().info(f"✅ 條件檢查成功，準備插入任務")
                self.find_task = True
                return True
            else:
                self.node.get_logger().info(f"📋 條件檢查未滿足")
                return False

        except Exception as e:
            self.node.get_logger().error(f"❌ 條件檢查失敗: {e}")
            return False

    def set_condition_start_id(self, start_id: int):
        """
        設定條件檢查的起始 ID

        Args:
            start_id: 起始條件 ID
        """
        self.condition_start_id = start_id
        self.node.get_logger().info(f"🔧 設定條件檢查起始 ID: {start_id}")

    def get_condition_start_id(self) -> int:
        """
        取得當前的條件檢查起始 ID

        Returns:
            int: 當前的起始 ID
        """
        return self.condition_start_id



    def insert_task(self):
        """
        插入KUKA搬運空架到出口傳送箱的任務
        基於收集的條件檢查資料來創建任務
        """
        # 收集的資料
        self.node.get_logger().info(f"📊 收集到的所有資料: {self.collected_data}")
        try:
            # 從收集的資料中提取必要資訊
            self._extract_task_data_from_collected()

            # 獲取對應的work
            for nWork in self.db_manager.works:
                if nWork.id == CONFIG.KUKA_RACK_MOVE:
                    self.kuka_rack_move_work = nWork
                    break

            if not self.kuka_rack_move_work:
                self.node.get_logger().error(f"❌ 找不到 KUKA_RACK_MOVE work (ID: {CONFIG.KUKA_RACK_MOVE})")
                return False

            # 取得取(空料架區)放(出口傳送箱)的uuid
            if not self.system_empty_rack_place_list or not self.task_node_id:
                self.node.get_logger().error(f"❌ 缺少必要的任務資料: empty_rack_place_list={self.system_empty_rack_place_list}, task_node_id={self.task_node_id}")
                return False

            uuid_nodes = [self.get_uuid(self.system_empty_rack_place_list[0]), self.get_uuid(self.task_node_id)]
            self.node.get_logger().info(f"🔄 uuid_nodes: {uuid_nodes}")

            # 設定parameter
            self.kuka_rack_move_work.parameters["nodes"] = uuid_nodes  # 設定nodes
            self.kuka_rack_move_work.parameters["model"] = CONFIG.KUKA_MODLE_NAME  # 設定機器名稱

            # 創建任務
            with self.db_manager.get_session() as session:
                task_data = Task(
                    work_id=CONFIG.KUKA_RACK_MOVE,
                    status_id=1,
                    room_id=self.task_room_id,
                    node_id=self.task_node_id,
                    name=f"KUKA搬運空架到出口傳送箱{self.task_room_id}",
                    description=self.kuka_rack_move_work.description,
                    mission_code="null",
                    priority=CONFIG.PRIORITY_FOR_KUKA_FROM_EMPTY_TO_BOXOUT,
                    parameters=self.kuka_rack_move_work.parameters,
                    created_at=datetime.now(timezone.utc),
                    updated_at=None
                )

                # 任務創建結果取出
                self.create_task_result = self.create_task(session, task_data)
                if self.create_task_result:
                    self.task_inserted = True
                    self.node.get_logger().info(f"🔄 KUKA搬運空架到出口傳送箱任務插入中......任務id: {self.create_task_result.id}")
                    return True

            return False

        except Exception as e:
            self.node.get_logger().error(f"❌ 插入KUKA搬運空架到出口傳送箱任務失敗: {e}")
            return False

    def _extract_task_data_from_collected(self):
        """
        從收集的資料中提取任務所需的資訊
        """
        try:
            # 從收集的資料中提取相關資訊
            # 這裡需要根據實際的 task_condition 資料結構來調整

            # 範例：從收集的資料中提取位置資訊
            if "location" in self.collected_data:
                location_value = self.collected_data["location"]
                if isinstance(location_value, str) and location_value.isdigit():
                    self.task_node_id = int(location_value)
                    self.node.get_logger().info(f"📝 從收集資料中取得 task_node_id: {self.task_node_id}")

            # 範例：從收集的資料中提取房間資訊
            if "room_id" in self.collected_data:
                self.task_room_id = self.collected_data["room_id"]
                self.node.get_logger().info(f"📝 從收集資料中取得 task_room_id: {self.task_room_id}")

            # 範例：從收集的資料中提取空料架區資訊
            if "empty_rack_place_list" in self.collected_data:
                self.system_empty_rack_place_list = self.collected_data["empty_rack_place_list"]
                if isinstance(self.system_empty_rack_place_list, str):
                    # 如果是字串，嘗試解析為列表
                    try:
                        self.system_empty_rack_place_list = json.loads(self.system_empty_rack_place_list)
                    except:
                        # 如果解析失敗，假設是單一值
                        self.system_empty_rack_place_list = [self.system_empty_rack_place_list]
                self.node.get_logger().info(f"📝 從收集資料中取得 empty_rack_place_list: {self.system_empty_rack_place_list}")

            # 記錄所有收集到的資料
            self.node.get_logger().info(f"📊 收集到的所有資料: {self.collected_data}")

        except Exception as e:
            self.node.get_logger().error(f"❌ 提取任務資料失敗: {e}")

    def check_insert_done(self):
        """
        檢查任務插入是否成功
        """
        try:
            with self.db_manager.get_session() as session:
                task = self.get_task_by_id(session, self.create_task_result.id)
                if task:
                    self.node.get_logger().info(f"✅ KUKA搬運空架到出口傳送箱任務插入完成-任務id: {self.create_task_result.id}")
                    self.find_task = False
                    self.task_inserted = False
                    return True
                return False
        except Exception as e:
            self.node.get_logger().error(f"❌ 檢查KUKA搬運空架到出口傳送箱任務插入是否成功失敗: {e}")
            return False
