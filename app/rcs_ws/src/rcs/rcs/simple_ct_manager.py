"""
簡化的 CT (Custom Transport) 車隊管理器
基於原始 186行版本的清晰設計，移除過度複雜的功能
"""
import rclpy
from rclpy.node import Node
from agv_interfaces.msg import AgvStateChange
from agv_interfaces.msg import AgvStatus
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import AGV, AGVContext
from sqlmodel import select


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

    def dispatch(self):
        """CT AGV 簡單任務派發邏輯"""
        self.logger.debug("執行 CT AGV 任務派發")
        
        if not self.db_pool:
            self.logger.error("資料庫連線池不可用，無法執行任務派發。")
            return
            
        try:
            with self.db_pool.get_session() as session:
                from db_proxy.models import Task
                from sqlmodel import select
                
                # 查詢待執行的 CT 任務 (使用大寫 Model，排除 KUKA400i)
                ct_tasks = session.exec(
                    select(Task).where(
                        Task.status_id == 1,  # 待執行
                        Task.mission_code == None,  # 尚未指定任務代碼
                        Task.parameters["model"].as_string() != "KUKA400i"  # 排除 KUKA 任務
                    ).order_by(Task.priority.asc())  # 優先級低的數字先執行
                ).all()
                
                if not ct_tasks:
                    # self.logger.debug("目前沒有 CT 任務需要處理")
                    return
                    
                self.logger.info(f"找到 {len(ct_tasks)} 個 CT 任務待處理")
                
                # 簡單的任務派發邏輯 (可根據需要擴展)
                for task in ct_tasks:
                    try:
                        # 基本的任務資訊記錄
                        model = task.parameters.get('model', 'Unknown')
                        self.logger.info(
                            f"CT 任務 {task.id}: 車型={model}, "
                            f"工作類型={task.work_id}, 優先級={task.priority}")
                        
                        # TODO: 實作具體的 CT 任務派發邏輯
                        # 例如：
                        # 1. 根據車型 (Cargo, Loader, Unloader) 選擇合適的 AGV
                        # 2. 檢查 AGV 是否在正確的房間
                        # 3. 檢查 AGV 狀態是否為閒置
                        # 4. 派發任務並更新狀態
                        
                        # 暫時只記錄任務資訊，不實際派發
                        self.logger.info(f"CT 任務 {task.id} 等待實作派發邏輯")
                        
                    except Exception as e:
                        self.logger.error(f"處理 CT 任務 {task.id} 時發生錯誤: {e}")
                        continue
                    
        except Exception as e:
            self.logger.error(f"CT 任務派發時發生錯誤: {e}", exc_info=True)

    def get_available_agvs(self):
        """取得可用的 CT AGV 列表"""
        # TODO: 實作取得可用 AGV 的邏輯
        # 例如：檢查 AGV 狀態、電量、位置等
        return list(self.ct_agvs.values())

    def assign_task_to_agv(self, task_id, agv_id):
        """將任務指派給指定的 AGV"""
        # TODO: 實作任務指派邏輯
        self.logger.info(f"將任務 {task_id} 指派給 CT AGV {agv_id}")
        
    def update_agv_status(self, agv_id, status):
        """更新 AGV 狀態"""
        # TODO: 實作 AGV 狀態更新邏輯
        self.logger.info(f"更新 CT AGV {agv_id} 狀態為 {status}")