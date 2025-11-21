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
from db_proxy.models import AGV, AGVContext, ModifyLog
from sqlmodel import select
from rcs.ct_task_allocator import CtTaskAllocator
from datetime import datetime, timezone


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
        """處理 AGV 狀態監控回調並更新資料庫"""
        try:
            # 記錄接收到的狀態 (改為 debug 級別避免刷屏)
            self.logger.debug(
                f"[CT AGV監控] AGV: {msg.agv_id}, "
                f"Position: ({msg.slam_x:.2f}, {msg.slam_y:.2f}, {msg.slam_theta:.2f}), "
                f"Power: {msg.power:.1f}%"
            )

            # 更新資料庫
            self._update_agv_position(msg)

        except Exception as e:
            self.logger.error(f"處理 AGV 狀態監控失敗: {e}")

    def ct_unit_2_px(self, y, x):
        """
        將 CT AGV 單位轉換為像素座標
        CT AGV 使用 mm 單位，地圖使用像素，轉換比例: 12.5mm = 1px

        Args:
            y: CT AGV y 座標 (mm)
            x: CT AGV x 座標 (mm)

        Returns:
            tuple: (px_y, px_x) 像素座標
        """
        return y / 12.5, x / 12.5

    def ct_angle_2_map_angle(self, angle):
        """
        將 CT AGV 角度轉換為地圖角度

        轉換邏輯參考 simple_kuka_manager.py 的 kuka_angle_2_map_angle 方法：
        - 反向旋轉：-1 * angle（如果方向相反）
        - 座標系偏移：- 90（匹配地圖東方(css px 座標)）
        - 範圍歸一化：-180 到 180 度

        Args:
            angle: CT AGV 角度（已除以10，0-360度）

        Returns:
            float: 地圖角度（-180 到 180 度）
        """
        # 座標系轉換公式（與 KUKA 一致）
        #angle = ((-1 * angle + 90) + 540 % 360) - 180
        angle =  (-1 * (angle - 90) + 540 % 360) - 180
        return angle

    def _update_agv_position(self, msg: AgvStatus):
        """
        更新 CT AGV 位置到資料庫

        根據 AgvStatus 訊息更新 AGV 表中的位置、航向角和電量資訊。
        參考 simple_kuka_manager.py 的實現模式。

        Args:
            msg: AgvStatus 訊息，包含 AGV 的即時狀態資訊
        """
        if not self.db_pool:
            self.logger.error("資料庫連線池不可用，無法更新 CT AGV 位置")
            return

        try:
            with self.db_pool.get_session() as session:
                # 根據 agv_id (name) 查詢 AGV
                agv = session.exec(
                    select(AGV).where(AGV.name == msg.agv_id)
                ).first()

                if not agv:
                    self.logger.warning(
                        f"找不到 AGV 名稱為 {msg.agv_id} 的資料，無法更新位置"
                    )
                    return

                # 更新 AGV 位置和狀態
                # 使用 ct_unit_2_px 轉換座標 (mm → px)
                px_y, px_x = self.ct_unit_2_px(msg.slam_y, msg.slam_x)
                agv.x = px_x
                agv.y = px_y
                # 使用 ct_angle_2_map_angle 轉換角度（單位轉換 + 座標系轉換）
                agv.heading = self.ct_angle_2_map_angle(msg.slam_theta / 10)
                agv.battery = msg.power

                # 將完整 AgvStatus 訊息序列化為 JSON
                agv.agv_status_json = {
                    "agv_id": msg.agv_id,
                    "slam_x": msg.slam_x,
                    "slam_y": msg.slam_y,
                    "slam_theta": msg.slam_theta,
                    "power": msg.power,
                    "x_speed": msg.x_speed,
                    "y_speed": msg.y_speed,
                    "theta_speed": msg.theta_speed,
                    "front_pgv": msg.front_pgv,
                    "back_pgv": msg.back_pgv,
                    "start_point": msg.start_point,
                    "end_point": msg.end_point,
                    "action": msg.action,
                    "zone": msg.zone,
                    "status1": msg.status1,
                    "status2": msg.status2,
                    "status3": msg.status3,
                    "alarm1": msg.alarm1,
                    "alarm2": msg.alarm2,
                    "alarm3": msg.alarm3,
                    "alarm4": msg.alarm4,
                    "alarm5": msg.alarm5,
                    "alarm6": msg.alarm6,
                    "magic": msg.magic,
                    "layer": msg.layer,
                    "timestamp": datetime.now(timezone.utc).isoformat()
                }

                # 🆕 根據 status1 和 alarm 字段計算並更新 status_id
                old_status_id = agv.status_id
                new_status_id = self._calculate_ct_agv_status_id(msg)

                if old_status_id != new_status_id:
                    agv.status_id = new_status_id
                    self.logger.info(
                        f"[CT AGV狀態] {msg.agv_id} 狀態變更: "
                        f"{old_status_id} → {new_status_id}"
                    )

                # 🔴 關鍵：標記 AGV 資料已更新，觸發前端更新
                # 前端 agvc_ui_socket.py 監聽此事件進行即時更新
                # 絕對不可移除！(參考 rcs_ws/CLAUDE.md 警告)
                ModifyLog.mark(session, "agv")

                # 提交變更
                session.commit()

                self.logger.debug(
                    f"已更新 CT AGV {msg.agv_id} 位置: "
                    f"({agv.x:.2f}, {agv.y:.2f}, {agv.heading:.2f}°), "
                    f"電量: {agv.battery:.1f}%"
                )

        except Exception as e:
            self.logger.error(f"更新 CT AGV {msg.agv_id} 位置時發生錯誤: {e}")

    def _calculate_ct_agv_status_id(self, msg: AgvStatus) -> int:
        """
        根據 CT AGV 的 status1 位和 alarm 字段計算 status_id

        映射規則：
        - 異常 (7): status1.bit2=1 或 alarm1~4 任一 > 0
        - 待機 (9): status1.bit11=1 或 alarm5~6 任一 > 0
        - 任務中 (4): status1.bit0=1 且 status1.bit3=1
        - 空閒 (3): status1.bit0=1 且 status1.bit3=0
        - 默認 (8): 其他情況返回維護中

        優先級：異常 > 待機 > 任務中 > 空閒 > 維護中

        Args:
            msg: AgvStatus 消息

        Returns:
            int: 對應的 status_id (3=空閒, 4=任務中, 7=異常, 8=維護中, 9=待機)
        """
        status1 = msg.status1

        # 優先級 1: 檢查異常狀態
        # status1.bit2=1 或 alarm1~4 任一 > 0
        if (status1 & (1 << 2)) or msg.alarm1 or msg.alarm2 or msg.alarm3 or msg.alarm4:
            return 7  # 異常

        # 優先級 2: 檢查待機狀態
        # status1.bit11=1 或 alarm5~6 任一 > 0
        if (status1 & (1 << 11)) or msg.alarm5 or msg.alarm6:
            return 9  # 待機

        # 優先級 3: 檢查任務中狀態
        # status1.bit0=1 且 status1.bit3=1
        if (status1 & (1 << 0)) and (status1 & (1 << 3)):
            return 4  # 任務中

        # 優先級 4: 檢查空閒狀態
        # status1.bit0=1 且 status1.bit3=0
        if (status1 & (1 << 0)) and not (status1 & (1 << 3)):
            return 3  # 空閒

        # 默認：維護中（無法判斷狀態）
        # 如果不符合任何已知的 PLC 狀態模式，表示 AGV 處於維護或初始化狀態
        return 8

    def handle_state_change(self, msg: AgvStateChange):
        """處理 AGV 狀態變更並更新資料庫"""
        self.logger.info(
            f"🔄 CT AGV 狀態變更: {msg.agv_id} 狀態從 {msg.from_state} 變更為 {msg.to_state}")

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
                self.logger.info(f"✅ 成功更新 CT AGV {msg.agv_id} 的 context。")
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

    def update_ct_agv_status(self):
        """
        從資料庫讀取 CT AGV 狀態並更新 status_id

        流程:
        1. 查詢所有啟用的 CT AGV (model != 'KUKA400i')
        2. 讀取每個 AGV 的 agv_status_json
        3. 使用 _calculate_ct_agv_status_id() 計算新的 status_id
        4. 如果有變化，更新資料庫並觸發前端更新
        """
        self.logger.debug("[CT AGV定時更新] 開始執行狀態檢查")

        if not self.db_pool:
            self.logger.error("資料庫連線池不可用，無法更新 CT AGV 狀態")
            return

        try:
            with self.db_pool.get_session() as session:
                from db_proxy.models import AGV, ModifyLog
                from sqlmodel import select

                # 查詢所有啟用的 CT AGV
                ct_agvs = session.exec(
                    select(AGV).where(
                        AGV.enable == 1,
                        AGV.model != "KUKA400i"
                    )
                ).all()

                if not ct_agvs:
                    return

                # 追蹤是否有任何變更
                has_changes = False

                for agv in ct_agvs:
                    try:
                        # 檢查 agv_status_json 是否存在
                        if not agv.agv_status_json:
                            continue

                        # 從 JSON 讀取狀態欄位
                        status_json = agv.agv_status_json

                        # 🔴 優先級 0: 檢查資料新鮮度（離線判斷）
                        # 這是最高優先級，因為如果資料很舊，即使舊資料顯示異常
                        # 也不如通訊中斷來得嚴重
                        timestamp_str = status_json.get("timestamp")
                        if timestamp_str:
                            from datetime import datetime, timezone
                            try:
                                # 解析 ISO 格式時間戳（支援多種格式）
                                if timestamp_str.endswith('Z'):
                                    timestamp_str = timestamp_str.replace('Z', '+00:00')
                                timestamp = datetime.fromisoformat(timestamp_str)

                                # 計算資料年齡（使用 UTC 時間）
                                if timestamp.tzinfo:
                                    now = datetime.now(timezone.utc)
                                else:
                                    timestamp = timestamp.replace(tzinfo=timezone.utc)
                                    now = datetime.now(timezone.utc)
                                age_seconds = (now - timestamp).total_seconds()

                                self.logger.debug(
                                    f"[CT AGV時間戳檢查] {agv.name} 資料年齡: {age_seconds:.1f} 秒"
                                )

                                # 資料超過 30 秒 → 強制離線
                                if age_seconds > 30:
                                    old_status_id = agv.status_id
                                    if old_status_id != 2:
                                        agv.status_id = 2  # 離線
                                        has_changes = True
                                        self.logger.warning(
                                            f"[CT AGV通訊中斷] {agv.name} 資料已 {age_seconds:.1f} 秒未更新 → 離線"
                                        )
                                    continue  # 跳過後續的 PLC 狀態計算
                            except Exception as e:
                                self.logger.error(f"解析 {agv.name} 時間戳失敗: {e}")

                        # 資料新鮮，繼續正常的 PLC 狀態計算
                        status1 = status_json.get("status1", 0)
                        alarm1 = status_json.get("alarm1", 0)
                        alarm2 = status_json.get("alarm2", 0)
                        alarm3 = status_json.get("alarm3", 0)
                        alarm4 = status_json.get("alarm4", 0)
                        alarm5 = status_json.get("alarm5", 0)
                        alarm6 = status_json.get("alarm6", 0)

                        # 創建臨時 AgvStatus 物件用於計算
                        class TempAgvStatus:
                            def __init__(self, status1, alarm1, alarm2, alarm3,
                                         alarm4, alarm5, alarm6):
                                self.status1 = status1
                                self.alarm1 = alarm1
                                self.alarm2 = alarm2
                                self.alarm3 = alarm3
                                self.alarm4 = alarm4
                                self.alarm5 = alarm5
                                self.alarm6 = alarm6

                        temp_msg = TempAgvStatus(
                            status1, alarm1, alarm2, alarm3,
                            alarm4, alarm5, alarm6
                        )

                        # 計算新的 status_id
                        old_status_id = agv.status_id
                        new_status_id = self._calculate_ct_agv_status_id(temp_msg)

                        # 檢查是否有變化
                        if old_status_id != new_status_id:
                            agv.status_id = new_status_id
                            has_changes = True

                            self.logger.info(
                                f"[CT AGV狀態更新] {agv.name} 狀態變更: "
                                f"{old_status_id} → {new_status_id} "
                                f"(status1={status1}, alarms=[{alarm1},{alarm2},"
                                f"{alarm3},{alarm4},{alarm5},{alarm6}])"
                            )

                    except KeyError as e:
                        self.logger.warning(
                            f"AGV {agv.name} 的 agv_status_json 缺少欄位: {e}"
                        )
                        continue
                    except Exception as e:
                        self.logger.error(
                            f"更新 AGV {agv.name} 狀態時發生錯誤: {e}"
                        )
                        continue

                # 如果有任何變更，觸發前端更新
                if has_changes:
                    ModifyLog.mark(session, "agv")
                    session.commit()

        except Exception as e:
            self.logger.error(f"更新 CT AGV 狀態時發生錯誤: {e}")

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

                # 查詢待執行的 CT 任務
                # 注意：不在 SQL 中過濾 model，因為 CT 任務的 parameters 中沒有 model 字段
                all_pending_tasks = session.exec(
                    select(Task).where(
                        Task.status_id == TaskStatus.PENDING,  # 待處理
                        Task.mission_code == None,  # 尚未指定任務代碼
                        Task.agv_id == None  # ✅ 只處理未分配 AGV 的任務
                    ).order_by(Task.priority.asc())  # 優先級低的數字先執行
                ).all()

                # 在 Python 中過濾：排除 model="KUKA400i" 的任務
                ct_tasks = [
                    task for task in all_pending_tasks
                    if not (task.parameters and task.parameters.get("model") == "KUKA400i")
                ]

                if not ct_tasks:
                    self.logger.debug(f"沒有 CT 任務待處理 (總共 {len(all_pending_tasks)} 個待處理任務)")
                    return

                self.logger.info(f"🔍 找到 {len(ct_tasks)} 個 CT 任務待處理 (已排除 {len(all_pending_tasks) - len(ct_tasks)} 個 KUKA 任務)")

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

                        # 查詢該 AGV 是否有活動任務（包含取消中的狀態）
                        # 取消中的任務仍可能占用 AGV（執行清理動作）
                        existing_task = session.exec(
                            select(Task).where(
                                Task.agv_id == agv.id,
                                Task.status_id.in_([
                                    TaskStatus.READY_TO_EXECUTE,  # 2 - 待執行
                                    TaskStatus.EXECUTING,          # 3 - 執行中
                                    TaskStatus.CANCELLING,         # 5 - 取消中
                                    TaskStatus.WCS_CANCELLING,     # 51 - WCS取消中
                                    TaskStatus.RCS_CANCELLING,     # 52 - RCS取消中
                                    TaskStatus.AGV_CANCELLING      # 53 - AGV取消中
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
                                f"✅ 成功分配任務 {task.id} (work_id={task.work_id}) "
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
                f"📋 分配任務詳情: "
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