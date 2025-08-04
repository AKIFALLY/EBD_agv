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
            success, collected_data = self.condition_checker.check_conditions_from_id(start_id=5)

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
        從收集的資料中提取任務所需的資訊
        
        收集資料格式: {'source_location': 11, '_condition_id': 211, 'target_location': 20001}
        提取:
        - source_location -> 來源位置ID（系統準備區）
        - target_location -> 目標位置ID（入口傳送箱）
        - 從 target_location 計算 room_id (20001 // 10000 = 2)
        """
        try:
            # 提取來源位置 (source_location)
            if "source_location" in self.collected_data:
                source_location = self.collected_data["source_location"]
                self.ready_rack_place_list = [source_location]  # 設定為列表格式以保持相容性
                self.ready_rack_place_count = 1
                self.node.get_logger().info(f"📝 從收集資料中取得來源位置: {source_location}")
            else:
                self.ready_rack_place_list = []
                self.ready_rack_place_count = 0
                self.node.get_logger().warning(f"⚠️ 收集資料中缺少來源位置 (source_location)")

            # 提取目標位置 (target_location) 
            if "target_location" in self.collected_data:
                target_location = self.collected_data["target_location"]
                self.task_node_id = target_location  # 設定 node_id
                self.node.get_logger().info(f"📝 從收集資料中取得目標位置: {target_location}")
            else:
                self.task_node_id = None
                self.node.get_logger().warning(f"⚠️ 收集資料中缺少目標位置 (target_location)")

            # 從目標位置計算房間ID (使用硬編碼規則: room_id = location_id // 10000)
            if self.task_node_id is not None:
                self.task_room_id = self.task_node_id // 10000
                self.node.get_logger().info(f"📝 從目標位置計算任務房間ID: {self.task_node_id} // 10000 = {self.task_room_id}")
            else:
                self.task_room_id = None
                self.node.get_logger().warning(f"⚠️ 無法計算房間ID，目標位置為空")

            # 記錄提取結果
            self.node.get_logger().info(f"📊 資料提取完成:")
            self.node.get_logger().info(f"   來源位置列表: {self.ready_rack_place_list}")
            self.node.get_logger().info(f"   目標位置: {self.task_node_id}") 
            self.node.get_logger().info(f"   房間ID: {self.task_room_id}")
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
            self.node.get_logger().info(f"📊 收集到的所有資料: {self.collected_data}")
            
            # 從收集的資料中提取必要資訊
            self._extract_task_data_from_collected()

            # 驗證必要的任務資料
            if self.task_room_id is None:
                self.node.get_logger().error(f"❌ 缺少必要的任務資料: task_room_id={self.task_room_id}")
                return False

            # 目標位置已經在 _extract_task_data_from_collected() 中設定
            self.boxin_node_id = self.task_node_id
            self.node.get_logger().info(f"📝 boxin_node_id: {self.boxin_node_id}")
            
            # 檢查 ready_rack_place_list 是否為空
            try:
                if not self.ready_rack_place_list or len(self.ready_rack_place_list) == 0:
                    self.node.get_logger().error(f"❌ ready_rack_place_list 為空或不存在: {self.ready_rack_place_list}")
                    return False

                # 透過 kuka_node 資料表取得來源位置和目標位置的 UUID
                self.node.get_logger().info(f"🔍 查詢 kuka_node 資料表取得 UUID:")
                self.node.get_logger().info(f"   來源位置 ID: {self.ready_rack_place_list[0]}")
                self.node.get_logger().info(f"   目標位置 ID: {self.boxin_node_id}")
                
                source_uuid = self.get_uuid(self.ready_rack_place_list[0])
                target_uuid = self.get_uuid(self.boxin_node_id)
                
                self.node.get_logger().info(f"🔍 UUID 查詢結果:")
                self.node.get_logger().info(f"   來源位置 UUID: {source_uuid}")
                self.node.get_logger().info(f"   目標位置 UUID: {target_uuid}")
                
                if not source_uuid or not target_uuid:
                    self.node.get_logger().error(f"❌ 無法取得位置的 UUID，請檢查 kuka_node 資料表:")
                    self.node.get_logger().error(f"   來源位置 {self.ready_rack_place_list[0]} -> UUID: {source_uuid}")
                    self.node.get_logger().error(f"   目標位置 {self.boxin_node_id} -> UUID: {target_uuid}")
                    return False

                # 建立 nodes 陣列 [source_uuid, target_uuid]
                uuid_nodes = [source_uuid, target_uuid]
                self.node.get_logger().info(f"✅ nodes 陣列建立完成: {uuid_nodes}")
                self.node.get_logger().info(f"   nodes[0] (來源): {uuid_nodes[0]}")
                self.node.get_logger().info(f"   nodes[1] (目標): {uuid_nodes[1]}")
                
            except IndexError as e:
                self.node.get_logger().error(f"❌ ready_rack_place_list 索引錯誤: {e}")
                self.node.get_logger().error(f"❌ ready_rack_place_list 內容: {self.ready_rack_place_list}")
                return False
            except Exception as e:
                self.node.get_logger().error(f"❌ 取得 UUID 失敗: {e}")
                return False

            if not self.kuka_rack_move_work:
                self.node.get_logger().error(f"❌ 找不到 KUKA_RACK_MOVE work: {CONFIG.KUKA_RACK_MOVE}")
                return False

            # 複製工作參數並設定 nodes
            task_parameters = self.kuka_rack_move_work.parameters.copy() if self.kuka_rack_move_work.parameters else {}
            task_parameters["nodes"] = uuid_nodes
            task_parameters["model"] = CONFIG.KUKA_MODLE_NAME

            # 記錄任務創建參數
            self.node.get_logger().info(f"📋 準備創建任務:")
            self.node.get_logger().info(f"   work_id: {self.kuka_rack_move_work.id}")
            self.node.get_logger().info(f"   status_id: {CONFIG.WCS_STATUS}")
            self.node.get_logger().info(f"   room_id: {self.task_room_id}")
            self.node.get_logger().info(f"   node_id: {self.task_node_id}")
            self.node.get_logger().info(f"   name: {self.kuka_rack_move_work.name or '系統準備區搬運道入口傳送區'}")
            self.node.get_logger().info(f"   priority: {CONFIG.PRIORITY_FOR_KUKA_FROM_READY_TO_BOXIN}")
            self.node.get_logger().info(f"   parameters: {task_parameters}")

            # 創建任務
            with self.db_manager.get_session() as session:
                try:
                    # 安全地取得第一個元素
                    source_location = self.ready_rack_place_list[0] if self.ready_rack_place_list else "未知"
                    
                    task_data = Task(
                        work_id=self.kuka_rack_move_work.id,
                        node_id=self.boxin_node_id,
                        name=self.kuka_rack_move_work.name or "系統準備區搬運道入口傳送區",
                        description=f"系統準備區搬運道入口傳送區 系統準備區:{source_location}-->房間:{self.task_room_id}",
                        room_id=self.task_room_id,
                        priority=CONFIG.PRIORITY_FOR_KUKA_FROM_READY_TO_BOXIN,  # 使用適當的優先級
                        status_id=CONFIG.WCS_STATUS,
                        created_at=datetime.now(timezone.utc),
                        updated_at=datetime.now(timezone.utc),
                        parameters=task_parameters
                    )
                    self.node.get_logger().info(f"✅ Task 物件創建成功")
                except Exception as task_create_error:
                    self.node.get_logger().error(f"❌ 創建 Task 物件失敗: {task_create_error}")
                    return False

                # 任務創建結果取出
                self.create_task_result = self.create_task(session, task_data)
                if self.create_task_result:
                    self.task_inserted = True
                    self.node.get_logger().info(f"✅ 準備區料架送往入口傳送箱任務插入成功，任務ID: {self.create_task_result.id}")
                    self.node.get_logger().info(f"📝 任務詳情: work_id={self.kuka_rack_move_work.id}, room_id={self.task_room_id}, nodes={uuid_nodes}")
                    return True
                else:
                    self.node.get_logger().error("❌ 準備區料架送往入口傳送箱任務插入失敗")
                    return False
                return False

        except Exception as e:
            import traceback
            self.node.get_logger().error(f"❌ 插入準備區料架送往入口傳送箱任務失敗: {e}")
            self.node.get_logger().error(f"❌ 錯誤詳細資訊: {traceback.format_exc()}")
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
    
