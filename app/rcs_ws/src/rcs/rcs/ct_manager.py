"""
CT (Custom Transport) 車隊管理器
處理您自己的 AGV 車隊相關功能
"""
import rclpy
from rclpy.node import Node
from agv_interfaces.msg import AgvStateChange
from agv_interfaces.msg import AgvStatus
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import AGV, AGVContext
from sqlmodel import select


class CtManager:
    """CT 車隊管理器類別"""

    def __init__(self, rcs_core_node):
        """
        初始化 CT 車隊管理器

        Args:
            rcs_core_node: RCS Core 節點實例
        """
        self.rcs_core = rcs_core_node
        self.logger = rcs_core_node.get_logger()
        self.db_pool: ConnectionPoolManager = rcs_core_node.db_pool

        # 設定 AGV 狀態監控訂閱者
        self._setup_agv_monitoring()

        # 載入您自己的 AGV 資料
        self.ct_agvs = {}
        self._load_ct_agvs()

    def _setup_agv_monitoring(self):
        """設定 AGV 狀態監控訂閱者"""
        # AGV 狀態變更監控
        self.agv_state_monitor_sub = self.rcs_core.create_subscription(
            AgvStateChange,
            "/agv/state_change",
            self.handle_state_change,
            10
        )
        self.logger.info("CT AGV Monitor 訂閱 /agv/state_change 啟動")

        # AGV 狀態監控
        self.agv_status_monitor_sub = self.rcs_core.create_subscription(
            AgvStateChange,
            "/agv/status",
            self.agv_status_monitor_callback,
            10
        )
        self.logger.info("CT AGV Monitor 訂閱 /agv/status 啟動")

    def agv_status_monitor_callback(self, msg: AgvStatus):
        """處理 AGV 狀態監控回調"""
        try:
            self.logger.info(
                f"[CT AGV監控] AGV: {msg.agv_id}, Power: {msg.power}, "
                f"slam_x: {msg.slam_x}, slam_y: {msg.slam_y}, slam_theta: {msg.slam_theta}, "
                f"x_speed: {msg.x_speed}, y_speed: {msg.y_speed}, theta_speed: {msg.theta_speed}, "
                f"front_pgv: {msg.front_pgv}, back_pgv: {msg.back_pgv}, "
                f"start_point: {msg.start_point}, end_point: {msg.end_point}, "
                f"action: {msg.action}, zone: {msg.zone}, "
                f"status1: {msg.status1}, status2: {msg.status2}, status3: {msg.status3}, "
                f"alarm1: {msg.alarm1}, alarm2: {msg.alarm2}, alarm3: {msg.alarm3}, "
                f"alarm4: {msg.alarm4}, alarm5: {msg.alarm5}, alarm6: {msg.alarm6}, "
                f"layer: {msg.layer}, magic: {msg.magic}"
            )
            # 更新資料進資料庫

        except Exception as e:
            self.get_logger().error(f"處理 AGV 狀態監控失敗: {e}")

    def handle_state_change(self, msg: AgvStateChange):
        """處理 AGV 狀態變更並更新資料庫"""
        self.logger.info(
            f"CT AGV 狀態變更: {msg.agv_id} 狀態從 {msg.from_state} 變更為 {msg.to_state}")

        if not self.db_pool:
            self.logger.error("資料庫連線池不可用，無法更新 AGV context。")
            return

        try:
            with self.db_pool.get_session() as session:
                agv = session.exec(select(AGV).where(
                    AGV.name == msg.agv_id)).first()

                if not agv:
                    self.logger.warning(
                        f"找不到 AGV 名稱為 {msg.agv_id} 的資料，無法更新 context")
                    return

                agv_context = next(
                    (ctx for ctx in agv.contexts if ctx.context == msg.context_name), None)

                if agv_context:
                    agv_context.last_state = msg.from_state
                    agv_context.current_state = msg.to_state
                else:
                    agv_context = AGVContext(
                        agv_id=agv.id,
                        context=msg.context_name,
                        last_state=msg.from_state,
                        current_state=msg.to_state
                    )
                    session.add(agv_context)

                session.commit()
                self.logger.info(f"成功更新 CT AGV {msg.agv_id} 的 context。")
        except Exception as e:
            self.logger.error(
                f"更新 CT AGV context 時發生錯誤: {e}", exc_info=True)

    def _load_ct_agvs(self):
        """從 AGV 資料表取得所有 enable 且非 KUKA400i 的 AGV，儲存於 self.ct_agvs。"""
        if not self.db_pool:
            self.logger.error("資料庫連線池不可用，無法載入 CT AGV。")
            return
        try:
            with self.db_pool.get_session() as session:
                from db_proxy.models import AGV
                from sqlmodel import select
                # 載入 CT 車隊（明確指定支援的車型）
                agvs = session.exec(
                    select(AGV).where(
                        AGV.enable == 1,
                        # 明確指定支援的 CT 車型
                        (AGV.model == "Cargo") |
                        (AGV.model == "Loader") |
                        (AGV.model == "Unloader")
                    )
                ).all()
                self.ct_agvs = {agv.id: agv for agv in agvs}
                self.logger.info(
                    f"已載入 {len(self.ct_agvs)} 台 CT AGV。{self.ct_agvs}")
        except Exception as e:
            self.logger.error(f"載入 CT AGV 時發生錯誤: {e}")

    def dispatch(self):
        """
        CT AGV 智能任務派發邏輯
        基於房間和車型的任務分派機制
        """
        self.logger.debug("執行 CT AGV 任務派發")

        if not self.db_pool:
            self.logger.error("資料庫連線池不可用，無法執行任務派發。")
            return

        try:
            with self.db_pool.get_session() as session:
                from db_proxy.models import Task
                from sqlmodel import select

                # 1. 查詢待執行的 CT 任務（按優先級降序排列）
                ct_tasks = session.exec(
                    select(Task).where(
                        Task.status_id == 1,  # 待執行
                        Task.mission_code == None,  # 尚未指定任務代碼
                        # 明確指定支援的 CT 車型
                        (Task.parameters["model"].as_string() == "Cargo") |
                        (Task.parameters["model"].as_string() == "Loader") |
                        (Task.parameters["model"].as_string() == "Unloader")
                    ).order_by(Task.priority.desc())  # 優先級高的任務優先派遣
                ).all()

                if not ct_tasks:
                    self.logger.debug("目前沒有 CT 任務需要處理")
                    return

                self.logger.info(f"找到 {len(ct_tasks)} 個 CT 任務待處理")

                # 2. 逐一處理每個任務
                for task in ct_tasks:
                    try:
                        # 驗證任務參數
                        if not self._validate_task_parameters(task):
                            continue

                        # 根據房間和車型選擇合適的 AGV
                        target_agv = self._select_agv_for_task(session, task)

                        if not target_agv:
                            self.logger.warning(
                                f"任務 {task.id} 找不到合適的 AGV，跳過此任務")
                            continue

                        # 檢查 AGV 是否為閒置狀態
                        if target_agv.status_id != 3:
                            self.logger.debug(
                                f"AGV {target_agv.name} 非閒置狀態 (status_id={target_agv.status_id})，跳過")
                            continue

                        # 執行任務派發
                        success = self._assign_task_to_agv(session, task, target_agv)

                        if success:
                            self.logger.info(
                                f"✅ 任務 {task.id} 成功派發給 AGV {target_agv.name}")
                        else:
                            self.logger.error(
                                f"❌ 任務 {task.id} 派發給 AGV {target_agv.name} 失敗")

                    except Exception as task_error:
                        self.logger.error(
                            f"處理任務 {task.id} 時發生錯誤: {task_error}", exc_info=True)
                        continue

                # 提交所有變更
                session.commit()

        except Exception as e:
            self.logger.error(f"CT 任務派發時發生錯誤: {e}", exc_info=True)

    def _validate_task_parameters(self, task):
        """
        驗證任務參數是否完整

        Args:
            task: 任務物件

        Returns:
            bool: 參數是否有效
        """
        try:
            if not task.parameters:
                self.logger.warning(f"任務 {task.id} 缺少 parameters 欄位")
                return False

            model = task.parameters.get('model')
            if not model:
                self.logger.warning(f"任務 {task.id} 缺少 model 參數")
                return False

            if model not in ['Cargo', 'Loader', 'Unloader']:
                self.logger.warning(f"任務 {task.id} 的 model 參數無效: {model}")
                return False

            return True

        except Exception as e:
            self.logger.error(f"驗證任務 {task.id} 參數時發生錯誤: {e}")
            return False

    def _select_agv_for_task(self, session, task):
        """
        根據房間和車型選擇合適的 AGV

        Args:
            session: 資料庫會話
            task: 任務物件

        Returns:
            AGV: 選中的 AGV 物件，如果沒有找到則返回 None
        """
        try:
            from db_proxy.models import AGV
            from sqlmodel import select

            model = task.parameters.get('model')
            room_id = task.room_id

            # 根據分派邏輯規則決定目標 AGV 名稱
            target_agv_name = self._determine_target_agv_name(model, room_id)

            if not target_agv_name:
                self.logger.warning(
                    f"無法為任務 {task.id} (model={model}, room_id={room_id}) 決定目標 AGV")
                return None

            # 查詢目標 AGV
            agv = session.exec(
                select(AGV).where(
                    AGV.name == target_agv_name,
                    AGV.enable == 1  # 確保 AGV 已啟用
                )
            ).first()

            if not agv:
                self.logger.warning(f"找不到名稱為 {target_agv_name} 的 AGV")
                return None

            return agv

        except Exception as e:
            self.logger.error(f"選擇 AGV 時發生錯誤: {e}")
            return None

    def _determine_target_agv_name(self, model, room_id):
        """
        根據車型和房間 ID 決定目標 AGV 名稱

        Args:
            model: 車型 (Cargo, Loader, Unloader)
            room_id: 房間 ID

        Returns:
            str: 目標 AGV 名稱，如果無法決定則返回 None
        """
        try:
            if room_id is None:
                # 房外任務 - Cargo 車輛
                if model == 'Cargo':
                    # 目前規劃 1 台 Cargo 負責 1 個房間的房外任務
                    # 暫時規劃 Cargo02 負責房間 2 的房外任務
                    return 'Cargo02'
                else:
                    self.logger.warning(f"房外任務不應使用 {model} 車型")
                    return None
            else:
                # 房內任務 - Loader 或 Unloader
                if model == 'Loader':
                    return f'Loader{room_id:02d}'  # 格式化為兩位數，如 Loader02
                elif model == 'Unloader':
                    return f'Unloader{room_id:02d}'  # 格式化為兩位數，如 Unloader01
                elif model == 'Cargo':
                    self.logger.warning(f"房內任務不應使用 Cargo 車型")
                    return None
                else:
                    self.logger.warning(f"未知的車型: {model}")
                    return None

        except Exception as e:
            self.logger.error(f"決定目標 AGV 名稱時發生錯誤: {e}")
            return None

    def _assign_task_to_agv(self, session, task, agv):
        """
        將任務指派給指定的 AGV

        Args:
            session: 資料庫會話
            task: 任務物件
            agv: AGV 物件

        Returns:
            bool: 是否成功派發
        """
        try:
            import uuid
            from datetime import datetime, timezone

            # 生成任務代碼
            mission_code = str(uuid.uuid4())

            # 更新任務資訊
            task.agv_id = agv.id
            task.mission_code = mission_code
            task.status_id = 2  # 更新為執行中
            task.updated_at = datetime.now(timezone.utc)

            # 更新 AGV 狀態為任務中
            agv.status_id = 4  # 任務中

            # 記錄派發資訊
            self.logger.info(
                f"任務派發詳情 - 任務ID: {task.id}, AGV: {agv.name}, "
                f"車型: {task.parameters.get('model')}, 房間: {task.room_id}, "
                f"任務代碼: {mission_code}")

            return True

        except Exception as e:
            self.logger.error(f"派發任務時發生錯誤: {e}")
            return False

    def get_available_agvs(self):
        """
        取得可用的 CT AGV 列表

        Returns:
            list: 可用的 AGV 列表
        """
        try:
            if not self.db_pool:
                self.logger.error("資料庫連線池不可用")
                return []

            with self.db_pool.get_session() as session:
                from db_proxy.models import AGV
                from sqlmodel import select

                # 查詢閒置且啟用的 CT AGV
                available_agvs = session.exec(
                    select(AGV).where(
                        AGV.enable == 1,
                        # 明確指定支援的 CT 車型
                        (AGV.model == "Cargo") |
                        (AGV.model == "Loader") |
                        (AGV.model == "Unloader"),
                        AGV.status_id == 3  # 閒置狀態
                    )
                ).all()

                return list(available_agvs)

        except Exception as e:
            self.logger.error(f"取得可用 AGV 時發生錯誤: {e}")
            return []

    def assign_task_to_agv(self, task_id, agv_id):
        """
        將任務指派給指定的 AGV（向後相容的方法）

        Args:
            task_id: 任務 ID
            agv_id: AGV ID
        """
        self.logger.info(f"將任務 {task_id} 指派給 CT AGV {agv_id}")
        # 注意：這個方法保留是為了向後相容，實際派發邏輯在 dispatch() 方法中

    def update_agv_status(self, agv_id, status):
        """
        更新 AGV 狀態

        Args:
            agv_id: AGV ID
            status: 新狀態 ID
        """
        try:
            if not self.db_pool:
                self.logger.error("資料庫連線池不可用，無法更新 AGV 狀態")
                return False

            with self.db_pool.get_session() as session:
                from db_proxy.models import AGV
                from sqlmodel import select

                agv = session.exec(
                    select(AGV).where(AGV.id == agv_id)
                ).first()

                if not agv:
                    self.logger.warning(f"找不到 ID 為 {agv_id} 的 AGV")
                    return False

                old_status = agv.status_id
                agv.status_id = status
                session.commit()

                self.logger.info(
                    f"✅ 成功更新 CT AGV {agv.name} 狀態：{old_status} → {status}")
                return True

        except Exception as e:
            self.logger.error(f"更新 CT AGV {agv_id} 狀態時發生錯誤: {e}")
            return False

    def get_task_statistics(self):
        """
        取得任務統計資訊

        Returns:
            dict: 任務統計資訊
        """
        try:
            if not self.db_pool:
                return {"error": "資料庫連線池不可用"}

            with self.db_pool.get_session() as session:
                from db_proxy.models import Task
                from sqlmodel import select, func

                # 統計各狀態的任務數量
                stats = {}

                # 待執行任務
                pending_count = session.exec(
                    select(func.count(Task.id)).where(
                        Task.status_id == 1,
                        # 明確指定支援的 CT 車型
                        (Task.parameters["model"].as_string() == "Cargo") |
                        (Task.parameters["model"].as_string() == "Loader") |
                        (Task.parameters["model"].as_string() == "Unloader")
                    )
                ).first()
                stats["pending"] = pending_count or 0

                # 執行中任務
                running_count = session.exec(
                    select(func.count(Task.id)).where(
                        Task.status_id == 2,
                        # 明確指定支援的 CT 車型
                        (Task.parameters["model"].as_string() == "Cargo") |
                        (Task.parameters["model"].as_string() == "Loader") |
                        (Task.parameters["model"].as_string() == "Unloader")
                    )
                ).first()
                stats["running"] = running_count or 0

                # 已完成任務
                completed_count = session.exec(
                    select(func.count(Task.id)).where(
                        Task.status_id == 3,
                        # 明確指定支援的 CT 車型
                        (Task.parameters["model"].as_string() == "Cargo") |
                        (Task.parameters["model"].as_string() == "Loader") |
                        (Task.parameters["model"].as_string() == "Unloader")
                    )
                ).first()
                stats["completed"] = completed_count or 0

                return stats

        except Exception as e:
            self.logger.error(f"取得任務統計時發生錯誤: {e}")
            return {"error": str(e)}
