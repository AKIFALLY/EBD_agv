"""
簡化的 CT (Custom Transport) 車隊管理器
基於原始 186行版本的清晰設計，移除過度複雜的功能

更新記錄:
- 2025-10-21: 新增基於 YAML 配置的 work_id → AGV 任務分配系統
"""
import rclpy
from rclpy.node import Node
from agv_interfaces.msg import AgvStateChange
from agv_interfaces.msg import AgvStatus
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import AGV, AGVContext
from sqlmodel import select
from rcs.ct_task_allocator import CtTaskAllocator


class CtManager:
    """簡化的 CT 車隊管理器"""
    
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

        # 初始化任務分配器 (基於 YAML 配置)
        config_path = '/app/config/rcs/ct_task_allocation.yaml'
        self.task_allocator = CtTaskAllocator(config_path, self.logger)
        self.logger.info("CT 任務分配器已初始化")
        
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
            AgvStatus,
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
            # 更新資料進資料庫 (可根據需要實作)
            
        except Exception as e:
            self.logger.error(f"處理 AGV 狀態監控失敗: {e}")

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
                f"更新 CT AGV context 時發生錯誤: {e}")

    def _load_ct_agvs(self):
        """從 AGV 資料表取得所有 enable 且非 KUKA400i 的 AGV，儲存於 self.ct_agvs。"""
        if not self.db_pool:
            self.logger.error("資料庫連線池不可用，無法載入 CT AGV。")
            return
        try:
            with self.db_pool.get_session() as session:
                from db_proxy.models import AGV
                from sqlmodel import select
                # 載入非 KUKA400i 的 AGV（您自己的車隊）
                agvs = session.exec(
                    select(AGV).where(
                        AGV.enable == 1, 
                        AGV.model != "KUKA400i"
                    )
                ).all()
                self.ct_agvs = {agv.id: agv for agv in agvs}
                self.logger.info(
                    f"已載入 {len(self.ct_agvs)} 台 CT AGV。{self.ct_agvs}")
        except Exception as e:
            self.logger.error(f"載入 CT AGV 時發生錯誤: {e}")

    def shutdown(self):
        """關閉 CT 管理器，清理資源"""
        self.logger.info("正在關閉 CT Manager...")
        # CT Manager 沒有定時器，但有訂閱者
        # 訂閱者會在節點銷毀時自動清理，這裡只記錄日誌
        self.logger.info("CT Manager 已關閉")
    
    def dispatch(self):
        """
        CT AGV 任務派發邏輯（基於 YAML 配置）

        流程:
        1. 檢查配置文件變更並熱重載
        2. 獲取可用的 CT AGV 列表
        3. 查詢待處理的 CT 任務
        4. 使用任務分配器根據 work_id 分配任務
        5. 執行任務分配並更新狀態
        """
        self.logger.debug("執行 CT AGV 任務派發")

        if not self.db_pool:
            self.logger.error("資料庫連線池不可用，無法執行任務派發。")
            return

        # 檢查配置文件變更並熱重載
        self.task_allocator.check_and_reload()

        try:
            with self.db_pool.get_session() as session:
                from db_proxy.models import Task
                from shared_constants.task_status import TaskStatus
                from sqlmodel import select

                # 獲取可用的 AGV 列表
                available_agvs = self.get_available_agvs(session)

                if not available_agvs:
                    self.logger.debug("目前沒有可用的 CT AGV")
                    return

                # 查詢待執行的 CT 任務 (使用小寫 model，排除 KUKA400i)
                ct_tasks = session.exec(
                    select(Task).where(
                        Task.status_id == TaskStatus.PENDING,  # 待處理
                        Task.mission_code == None,  # 尚未指定任務代碼
                        Task.parameters["model"].as_string() != "KUKA400i"  # 排除 KUKA
                    ).order_by(Task.priority.asc())  # 優先級低的數字先執行
                ).all()

                if not ct_tasks:
                    return

                self.logger.info(f"找到 {len(ct_tasks)} 個 CT 任務待處理")

                # 遍歷任務並分配
                for task in ct_tasks:
                    try:
                        # 使用任務分配器分配任務
                        agv_name, priority_override = self.task_allocator.allocate_task(
                            task, available_agvs
                        )

                        if agv_name is None:
                            # 無法分配，跳過此任務
                            self.logger.debug(
                                f"任務 {task.id} (work_id={task.work_id}) "
                                f"暫時無法分配，跳過"
                            )
                            continue

                        # 檢查該 AGV 是否已經有其他活動任務
                        agv = session.exec(
                            select(AGV).where(AGV.name == agv_name)
                        ).first()

                        if not agv:
                            self.logger.error(f"找不到 AGV: {agv_name}")
                            continue

                        # 查詢該 AGV 是否有待執行或執行中的任務
                        existing_task = session.exec(
                            select(Task).where(
                                Task.agv_id == agv.id,
                                Task.status_id.in_([
                                    TaskStatus.READY_TO_EXECUTE,  # 待執行
                                    TaskStatus.EXECUTING           # 執行中
                                ])
                            )
                        ).first()

                        if existing_task:
                            self.logger.info(
                                f"⏸️  跳過任務 {task.id} (work_id={task.work_id})："
                                f"AGV {agv_name} 忙碌中（正在執行任務 {existing_task.id}）"
                            )
                            continue

                        # 執行任務分配
                        success = self.assign_task_to_agv(
                            session, task, agv_name, priority_override
                        )

                        if success:
                            # 提交變更
                            session.commit()
                            self.logger.info(
                                f"成功分配任務 {task.id} (work_id={task.work_id}) "
                                f"給 AGV {agv_name}"
                            )
                        else:
                            # 回滾變更
                            session.rollback()

                    except Exception as e:
                        self.logger.error(
                            f"處理 CT 任務 {task.id} 時發生錯誤: {e}"
                        )
                        session.rollback()
                        continue

        except Exception as e:
            self.logger.error(f"CT 任務派發時發生錯誤: {e}")

    def get_available_agvs(self, session):
        """
        取得可用的 CT AGV 列表

        檢查條件:
        - AGV enable = 1 (啟用)
        - AGV model != "KUKA400i" (排除 KUKA)
        - 可以擴展: AGV 狀態、電量、當前任務數等

        Args:
            session: 資料庫 session

        Returns:
            List[AGV]: 可用的 AGV 列表
        """
        try:
            from db_proxy.models import AGV
            from sqlmodel import select

            # 查詢啟用的 CT AGV
            available_agvs = session.exec(
                select(AGV).where(
                    AGV.enable == 1,
                    AGV.model != "KUKA400i"
                )
            ).all()

            # TODO: 可以進一步過濾
            # 例如：
            # - 檢查 AGV 狀態（idle, busy, charging 等）
            # - 檢查電量（battery_level > threshold）
            # - 檢查當前任務數（避免超過 max_concurrent_tasks）

            return list(available_agvs)

        except Exception as e:
            self.logger.error(f"取得可用 AGV 列表時發生錯誤: {e}")
            return []

    def assign_task_to_agv(self, session, task, agv_name: str, priority_override=None):
        """
        將任務分配給指定的 AGV

        執行步驟:
        1. 根據 agv_name 查找 AGV
        2. 更新 task.agv_id
        3. 應用優先級覆蓋（如果有）
        4. 更新任務狀態為 "已分配"
        5. 生成 mission_code (可選)

        Args:
            session: 資料庫 session
            task: Task 實例
            agv_name: AGV 名稱
            priority_override: 優先級覆蓋值（None = 不覆蓋）

        Returns:
            bool: 分配成功返回 True
        """
        try:
            from db_proxy.models import AGV
            from shared_constants.task_status import TaskStatus
            from sqlmodel import select

            # 查找 AGV
            agv = session.exec(
                select(AGV).where(AGV.name == agv_name)
            ).first()

            if not agv:
                self.logger.error(f"找不到 AGV: {agv_name}")
                return False

            # 更新任務的 AGV
            task.agv_id = agv.id

            # 應用優先級覆蓋
            if priority_override is not None:
                old_priority = task.priority
                task.priority = priority_override
                self.logger.info(
                    f"任務 {task.id} 優先級從 {old_priority} 覆蓋為 {priority_override}"
                )

            # 更新任務狀態為 "待執行" (READY_TO_EXECUTE = 2)
            task.status_id = TaskStatus.READY_TO_EXECUTE

            # TODO: 生成 mission_code
            # task.mission_code = self._generate_mission_code(task, agv)

            # 記錄分配詳情
            self.logger.info(
                f"分配任務詳情: "
                f"Task ID={task.id}, Work ID={task.work_id}, "
                f"AGV={agv_name} (id={agv.id}), "
                f"Priority={task.priority}, Room={task.room_id}"
            )

            return True

        except Exception as e:
            self.logger.error(f"分配任務時發生錯誤: {e}")
            return False
        
    def update_agv_status(self, agv_id, status):
        """更新 AGV 狀態"""
        # TODO: 實作 AGV 狀態更新邏輯
        self.logger.info(f"更新 CT AGV {agv_id} 狀態為 {status}")