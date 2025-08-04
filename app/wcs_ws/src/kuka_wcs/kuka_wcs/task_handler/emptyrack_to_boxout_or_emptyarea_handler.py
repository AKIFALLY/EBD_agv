import config.config as CONFIG
from db_proxy.models import Task
from datetime import datetime, timezone
from wcs_base.base_task_handler import BaseTaskHandler
from wcs_base.task_condition_checker import TaskConditionChecker
import json


class EmptyRackToBoxoutOrEmptyareaHandler(BaseTaskHandler):
    """
    空料架從入口傳送箱搬運到出口傳送箱或系統空料架區任務處理器
    處理將入口傳送箱的空料架搬運到出口傳送箱或系統空料架區的任務
    支援條件分支邏輯：優先選擇出口傳送箱，若佔用則選擇系統空料架區
    """

    def __init__(self, node):
        super().__init__(node)

        # 任務相關數據
        self.kuka_rack_move_work = None  # 對應的workID資料
        self.empty_to_boxout_or_emptyarea_task = None  # 創建的任務
        self.task_node_id = None  # 目標節點ID (舊版，保留相容性)
        self.task_room_id = None  # 目標房間ID
        self.source_location = None  # 來源位置 (location)
        self.target_location = None  # 目標位置 (location_id)
        self.target_type = None  # 目標類型 ('outlet' 或 'emptyarea')

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
        從 id=5 開始進行條件判斷流程。
        """
        try:
            # 使用條件檢查器進行檢查
            success, collected_data = self.condition_checker.check_conditions_from_id(start_id=5)

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

    def _extract_task_data_from_collected(self):
        """
        從收集的資料中提取任務所需的資訊
        
        收集資料格式: 
        - 從條件 5: {'source_location': 20001, 'rack_id': 123, 'room_id': 2}
        - 從條件 215/216: {'target_location': 20002, 'target_type': 'outlet'} 或 {'target_location': 31, 'target_type': 'emptyarea'}
        提取:
        - source_location: 來源位置ID (入口傳送箱)
        - target_location: 目標位置ID (出口傳送箱或空料架區)
        - target_type: 目標類型 ('outlet' 或 'emptyarea')
        - room_id: 房間ID
        """
        try:
            # 提取來源位置 (source_location)
            if "source_location" in self.collected_data:
                self.source_location = self.collected_data["source_location"]
                self.node.get_logger().info(f"📝 從收集資料中取得來源位置: {self.source_location}")
            else:
                self.source_location = None
                self.node.get_logger().warning(f"⚠️ 收集資料中缺少來源位置 (source_location)")

            # 提取目標位置 (target_location) 
            if "target_location" in self.collected_data:
                self.target_location = self.collected_data["target_location"]
                self.node.get_logger().info(f"📝 從收集資料中取得目標位置: {self.target_location}")
            else:
                self.target_location = None
                self.node.get_logger().warning(f"⚠️ 收集資料中缺少目標位置 (target_location)")

            # 提取目標類型 (target_type)
            if "target_type" in self.collected_data:
                self.target_type = self.collected_data["target_type"]
                self.node.get_logger().info(f"📝 從收集資料中取得目標類型: {self.target_type}")
            else:
                # 預設為 outlet 類型
                self.target_type = "outlet" if self.target_location and self.target_location >= 10000 else "emptyarea"
                self.node.get_logger().info(f"📝 根據目標位置推定目標類型: {self.target_type}")

            # 提取或計算房間ID
            if "room_id" in self.collected_data:
                self.task_room_id = self.collected_data["room_id"]
                self.node.get_logger().info(f"📝 從收集資料中取得房間ID: {self.task_room_id}")
            elif self.target_type == "outlet" and self.target_location is not None:
                self.task_room_id = self.target_location // 10000
                self.node.get_logger().info(f"📝 從目標位置計算任務房間ID: {self.target_location} // 10000 = {self.task_room_id}")
            elif self.source_location is not None:
                self.task_room_id = self.source_location // 10000
                self.node.get_logger().info(f"📝 從來源位置計算任務房間ID: {self.source_location} // 10000 = {self.task_room_id}")
            else:
                self.task_room_id = None
                self.node.get_logger().warning(f"⚠️ 無法計算房間ID")

            # 記錄提取結果
            self.node.get_logger().info(f"📊 資料提取完成:")
            self.node.get_logger().info(f"   來源位置: {self.source_location}")
            self.node.get_logger().info(f"   目標位置: {self.target_location}")
            self.node.get_logger().info(f"   目標類型: {self.target_type}") 
            self.node.get_logger().info(f"   房間ID: {self.task_room_id}")

        except Exception as e:
            self.node.get_logger().error(f"❌ 提取任務資料失敗: {e}")

    def insert_task(self):
        """
        插入KUKA搬運空料架從入口傳送箱到出口傳送箱或系統空料架區的任務
        基於收集的條件檢查資料來創建任務，支援條件分支邏輯
        
        收集資料格式: 
        - 基本資料: {'source_location': 20001, 'room_id': 2}
        - 目標資料: {'target_location': 20002, 'target_type': 'outlet'} 或 {'target_location': 31, 'target_type': 'emptyarea'}
        任務參數:
        - work_id: 220001
        - status_id: 1  
        - room_id: 從 source_location 或 target_location 計算
        - parameters.nodes: [source_uuid, target_uuid] 透過 kuka_node 資料表轉換
        """
        try:
            self.node.get_logger().info(f"📊 收集到的所有資料: {self.collected_data}")
            
            # 從收集的資料中提取必要資訊
            self._extract_task_data_from_collected()

            # 獲取對應的 work (work_id = 220001)
            target_work = None
            for nWork in self.db_manager.works:
                if nWork.id == 220001:
                    target_work = nWork
                    break

            if not target_work:
                self.node.get_logger().error(f"❌ 找不到 work_id=220001 的工作定義")
                return False

            # 驗證必要的任務資料
            if self.source_location is None or self.target_location is None:
                self.node.get_logger().error(f"❌ 缺少必要的任務資料: source_location={self.source_location}, target_location={self.target_location}, target_type={self.target_type}")
                return False

            # 透過 kuka_node 資料表取得來源位置和目標位置的 UUID
            self.node.get_logger().info(f"🔍 查詢 kuka_node 資料表取得 UUID:")
            self.node.get_logger().info(f"   來源位置 ID: {self.source_location}")
            self.node.get_logger().info(f"   目標位置 ID: {self.target_location}")
            
            source_uuid = self.get_uuid(self.source_location)
            target_uuid = self.get_uuid(self.target_location)
            
            self.node.get_logger().info(f"🔍 UUID 查詢結果:")
            self.node.get_logger().info(f"   來源位置 UUID: {source_uuid}")
            self.node.get_logger().info(f"   目標位置 UUID: {target_uuid}")
            
            if not source_uuid or not target_uuid:
                self.node.get_logger().error(f"❌ 無法取得位置的 UUID，請檢查 kuka_node 資料表:")
                self.node.get_logger().error(f"   來源位置 {self.source_location} -> UUID: {source_uuid}")
                self.node.get_logger().error(f"   目標位置 {self.target_location} -> UUID: {target_uuid}")
                return False

            # 建立 nodes 陣列 [source_uuid, target_uuid]
            uuid_nodes = [source_uuid, target_uuid]
            self.node.get_logger().info(f"✅ nodes 陣列建立完成: {uuid_nodes}")
            self.node.get_logger().info(f"   nodes[0] (來源): {uuid_nodes[0]}")
            self.node.get_logger().info(f"   nodes[1] (目標): {uuid_nodes[1]}")

            # 重複任務檢查：檢查是否有執行中任務使用相同的 nodes（不限制 work_id）
            with self.db_manager.get_session() as check_session:
                try:
                    # 檢查是否有執行中任務的 nodes 參數與當前任務重疊
                    from sqlalchemy import text
                    duplicate_check_query = text("""
                        SELECT COUNT(*) as count
                        FROM task 
                        WHERE status_id IN (0, 1, 2)
                        AND (
                            (parameters->>'nodes')::jsonb ? :source_uuid 
                            OR (parameters->>'nodes')::jsonb ? :target_uuid
                        )
                    """)
                    result = check_session.execute(duplicate_check_query, {
                        'source_uuid': source_uuid, 
                        'target_uuid': target_uuid
                    })
                    duplicate_count = result.fetchone()[0]
                    
                    if duplicate_count > 0:
                        self.node.get_logger().warning(f"⚠️ 發現重複任務：已有執行中任務使用相同的 nodes")
                        self.node.get_logger().warning(f"   檢查的 nodes: {uuid_nodes}")
                        self.node.get_logger().warning(f"   重複任務數量: {duplicate_count}")
                        return False
                    else:
                        self.node.get_logger().info(f"✅ 重複任務檢查通過：無相同 nodes 的執行中任務")
                        
                except Exception as duplicate_check_error:
                    self.node.get_logger().error(f"❌ 重複任務檢查失敗: {duplicate_check_error}")
                    return False

            # 複製工作參數並設定 nodes 和 model
            task_parameters = target_work.parameters.copy() if target_work.parameters else {}
            task_parameters["nodes"] = uuid_nodes
            task_parameters["model"] = CONFIG.KUKA_MODLE_NAME

            # 記錄任務創建參數
            self.node.get_logger().info(f"📋 準備創建任務:")
            self.node.get_logger().info(f"   work_id: 220001")
            self.node.get_logger().info(f"   status_id: 1")
            self.node.get_logger().info(f"   room_id: {self.task_room_id}")
            self.node.get_logger().info(f"   node_id: {self.target_location}")
            self.node.get_logger().info(f"   name: {target_work.name}")
            self.node.get_logger().info(f"   priority: {CONFIG.PRIORITY_FOR_KUKA_EMPTY_RACK_TRANSFER}")
            self.node.get_logger().info(f"   parameters: {task_parameters}")

            # 創建任務
            with self.db_manager.get_session() as session:
                try:
                    # 根據目標類型設定任務名稱和描述
                    if self.target_type == "outlet":
                        task_name = target_work.name or "KUKA搬運空料架從入口傳送箱到出口傳送箱"
                        task_description = f"空料架從入口傳送箱到出口傳送箱 來源:{self.source_location}->目標:{self.target_location} 房間:{self.task_room_id}"
                    else:  # emptyarea
                        task_name = target_work.name or "KUKA搬運空料架從入口傳送箱到系統空料架區"
                        task_description = f"空料架從入口傳送箱到系統空料架區 來源:{self.source_location}->目標:{self.target_location} 房間:{self.task_room_id}"
                    
                    task_data = Task(
                        work_id=target_work.id,
                        node_id=self.target_location,
                        name=task_name,
                        description=task_description,
                        room_id=self.task_room_id,
                        priority=CONFIG.PRIORITY_FOR_KUKA_EMPTY_RACK_TRANSFER,
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
                    target_desc = "出口傳送箱" if self.target_type == "outlet" else "系統空料架區"
                    self.node.get_logger().info(f"✅ KUKA搬運空料架從入口傳送箱到{target_desc}任務插入成功，任務ID: {self.create_task_result.id}")
                    self.node.get_logger().info(f"📝 任務詳情: work_id=220001, room_id={self.task_room_id}, target_type={self.target_type}, nodes={uuid_nodes}")
                    return True

            return False

        except Exception as e:
            import traceback
            self.node.get_logger().error(f"❌ 插入KUKA搬運空料架從入口傳送箱到目標位置任務失敗: {e}")
            self.node.get_logger().error(f"❌ 錯誤詳細資訊: {traceback.format_exc()}")
            return False

    def check_insert_done(self):
        """
        檢查任務插入是否成功
        """
        try:
            with self.db_manager.get_session() as session:
                task = self.get_task_by_id(session, self.create_task_result.id)
                if task:
                    target_desc = "出口傳送箱" if getattr(self, 'target_type', 'outlet') == 'outlet' else "系統空料架區"
                    self.node.get_logger().info(f"✅ KUKA搬運空料架從入口傳送箱到{target_desc}任務插入完成-任務id: {self.create_task_result.id}")
                    self.find_task = False
                    self.task_inserted = False
                    return True
                return False
        except Exception as e:
            self.node.get_logger().error(f"❌ 檢查KUKA搬運空料架從入口傳送箱到目標位置任務插入是否成功失敗: {e}")
            return False