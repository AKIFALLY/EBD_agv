"""
簡化的 KUKA 車隊管理器
基於原始 kuka_dispatcher 的簡潔設計，移除複雜的 WCS 適配和優先度調度

⚠️ 重構歷史與警告 (2025-07-29 重構事件):
=====================================
原始系統包含多個模組 (kuka_manager.py 1517行, kuka_robot.py, kuka_container.py 等)
簡化後合併為單一檔案，但曾誤刪關鍵功能，已於 2025-09-18 (commit d77f8275) 恢復

🔴 絕對不可刪除的功能:
1. on_robot_update() - 機器人位置即時更新 (前端地圖顯示依賴)
2. on_container_update() - 容器狀態管理 (Rack 狀態同步)
3. kuka_unit_2_px() - 座標轉換 (KUKA mm → 地圖像素)
4. kuka_angle_2_map_angle() - 角度轉換 (KUKA → 地圖角度)
5. ModifyLog.mark() - 觸發前端即時更新機制

這些功能是前後端即時同步的核心，移除會導致前端無法顯示最新狀態
"""
from rclpy.logging import RcutilsLogger
from db_proxy.connection_pool_manager import ConnectionPoolManager
import uuid
from kuka_fleet_adapter.kuka_fleet_adapter import KukaFleetAdapter
from db_proxy.models import AGV, ModifyLog, Rack, Task
from db_proxy.models.agvc_kuka import KukaNode
from db_proxy.models.agvc_location import Location
from db_proxy.crud.agv_crud import agv_crud
from sqlmodel import select
import traceback
from typing import Optional
from shared_constants.task_status import TaskStatus


class KukaManager:
    """簡化的 KUKA 車隊管理器"""
    
    def __init__(self, rcs_core):
        """
        初始化 KUKA 管理器

        Args:
            rcs_core: RCS Core 節點實例
        """
        self.rcs_core = rcs_core
        self.kuka_fleet: KukaFleetAdapter = KukaFleetAdapter(rcs_core)
        self.db_pool: ConnectionPoolManager = rcs_core.db_pool
        self.get_logger: RcutilsLogger = rcs_core.get_logger

        # 日誌頻率控制（降低刷屏）
        self._container_log_counter = 0
        self._container_log_interval = 10  # 每10次查詢輸出一次匯總（約1秒）

        # 設置機器人位置更新回調
        self.kuka_fleet.on_robot_query_complete = self.on_robot_update

        # 設置容器狀態更新回調
        self.kuka_fleet.on_container_query_complete = self.on_container_update

        # 啟動監控
        self.kuka_fleet.start_monitoring()
        self.get_logger().info("=" * 60)
        self.get_logger().info("🚀 KUKA Fleet 監控已啟動")
        self.get_logger().info("✅ 機器人位置更新功能已啟用")
        self.get_logger().info("✅ 容器狀態更新功能已啟用 (Rack 資料同步)")
        self.get_logger().info("=" * 60)

    def kuka_unit_2_px(self, y, x):
        """
        將 KUKA 單位轉換為像素座標
        KUKA 使用 mm 單位，地圖使用像素，轉換比例: 12.5mm = 1px

        Args:
            y: KUKA y 座標 (mm)
            x: KUKA x 座標 (mm)

        Returns:
            tuple: (px_y, px_x) 像素座標
        """
        return y / 12.5, x / 12.5

    def kuka_angle_2_map_angle(self, angle):
        """
        將 KUKA 角度轉換為地圖角度

        Args:
            angle: KUKA 機器人角度

        Returns:
            float: 地圖角度
        """
        # map angle to kuka angle
        angle = ((-1 * (angle - 90)) + 540 % 360) - 180
        return angle

    def on_robot_update(self, robots: list):
        """
        🔴 關鍵功能 - 絕對不可刪除！
        當 KukaFleetAdapter 查詢到機器人狀態時，更新資料庫

        此功能負責：
        - 將 KUKA 機器人即時位置同步到資料庫
        - 轉換座標系統 (KUKA mm → 像素)
        - 觸發 ModifyLog 讓前端即時更新地圖顯示
        - 前端 agvc_ui_socket.py 依賴此更新機制

        Args:
            robots: 機器人狀態列表，每個元素包含:
                - robotId: 機器人 ID
                - x, y: 位置座標 (mm)
                - robotOrientation: 角度
                - batteryLevel: 電池電量
                - status: 狀態碼 (3=空閒, 4=任務中, 等)
                - nodeNumber: 當前節點編號
        """
        if not self.db_pool:
            self.get_logger().error("資料庫連線池不可用，無法更新機器人狀態")
            return

        try:
            with self.db_pool.get_session() as session:
                updated_count = 0
                for robot in robots:
                    if self._update_single_robot(session, robot):
                        updated_count += 1

                # 🆕 同步 robot→rack 關聯（在 callback 無法工作時的備用機制）
                self._sync_rack_agv_mapping(session, robots)

                if updated_count > 0:
                    # 🔴 關鍵：標記 AGV 資料已更新，觸發前端更新
                    # 絕對不可移除！前端 agvc_ui_socket.py 監聽此事件
                    ModifyLog.mark(session, "agv")
                    session.commit()
                    self.get_logger().debug(f"已更新 {updated_count} 台 KUKA 機器人狀態")

        except Exception as e:
            self.get_logger().error(f"更新 KUKA 機器人狀態時發生錯誤: {e}")

    def _update_single_robot(self, session, robot_data: dict) -> bool:
        """
        更新單個機器人的資料

        Args:
            session: 資料庫 session
            robot_data: 機器人資料字典

        Returns:
            bool: 是否成功更新
        """
        robot_id = robot_data.get("robotId")
        if not robot_id:
            return False

        try:
            # 驗證機器人資料完整性
            if not self._validate_robot_data(robot_data):
                return False

            # 從資料庫取得 AGV 記錄
            agv: AGV = agv_crud.get_by_id(session, int(robot_id))
            if not agv:
                # AGV 不在資料庫中，可能是新機器人或未註冊
                self.get_logger().debug(f"AGV {robot_id} 不存在於資料庫中")
                return False

            # 座標轉換
            pos_px_y, pos_px_x = self.kuka_unit_2_px(
                float(robot_data.get("y", 0)),
                float(robot_data.get("x", 0))
            )

            # 角度轉換
            heading = self.kuka_angle_2_map_angle(
                float(robot_data.get("robotOrientation", 0))
            )

            # 取得節點編號
            node_number_str = robot_data.get("nodeNumber", "")
            node_number = None
            if node_number_str and node_number_str != '':
                try:
                    node_number = int(node_number_str)
                except (ValueError, TypeError):
                    pass

            # 更新 AGV 屬性
            agv.x = pos_px_x
            agv.y = pos_px_y
            agv.heading = heading
            agv.battery = robot_data.get("batteryLevel")
            agv.status_id = robot_data.get("status")
            if node_number is not None:
                agv.last_node_id = node_number

            # 填充 agv_status_json (為前端狀態顯示和未來錯誤分級做準備)
            from datetime import datetime, timezone
            agv.agv_status_json = {
                "robotId": robot_data.get("robotId"),
                "status": robot_data.get("status"),
                "batteryLevel": robot_data.get("batteryLevel"),
                "x": robot_data.get("x"),
                "y": robot_data.get("y"),
                "robotOrientation": robot_data.get("robotOrientation"),
                "nodeNumber": robot_data.get("nodeNumber"),
                "containerCode": robot_data.get("containerCode"),
                "errorCode": robot_data.get("errorCode", ""),
                "timestamp": datetime.now(timezone.utc).isoformat()
            }

            return True

        except Exception as e:
            self.get_logger().error(f"更新 AGV {robot_id} 狀態時發生錯誤: {e}")
            if self.get_logger().isEnabledFor(10):  # DEBUG level
                tb_str = traceback.format_exc()
                self.get_logger().debug(f"堆疊訊息:\n{tb_str}")
            return False

    def _sync_rack_agv_mapping(self, session, robots: list):
        """
        同步 robotId → containerCode 到 rack.agv_id

        此功能用於在 KUKA Fleet Callback 無法正常工作時，
        透過輪詢 robotQuery 來維持 robot-rack 關聯關係

        Args:
            session: 資料庫 session
            robots: 機器人狀態列表
        """
        try:
            # 1. 構建 robotId → containerCode 映射
            robot_containers = {}
            for robot in robots:
                robot_id = robot.get("robotId")
                container_code = robot.get("containerCode")
                if robot_id and container_code:
                    try:
                        robot_containers[container_code] = int(robot_id)
                    except (ValueError, TypeError):
                        self.get_logger().warning(
                            f"無效的 robotId: {robot_id}，跳過容器 {container_code}")
                        continue

            if not robot_containers:
                # 沒有任何 robot 正在搬運容器，執行安全清除
                self._safe_clear_rack_agv_mapping(session)
                return

            # 2. 更新有映射的 rack.agv_id（增量更新）
            updated_count = 0
            for container_code, robot_id in robot_containers.items():
                rack = session.exec(
                    select(Rack).where(Rack.name == container_code)
                ).first()

                if rack:
                    # 查詢 AGV 是否存在
                    agv = agv_crud.get_by_id(session, robot_id)
                    if agv:
                        # 只在 agv_id 有變化時更新
                        if rack.agv_id != agv.id:
                            old_agv_id = rack.agv_id
                            rack.agv_id = agv.id
                            updated_count += 1
                            self.get_logger().info(
                                f"✅ Rack-AGV 同步: {rack.name} agv_id {old_agv_id} → {agv.id}")
                    else:
                        self.get_logger().warning(
                            f"⚠️ Robot {robot_id} 不存在於 AGV 表，無法同步 Rack {container_code}")
                else:
                    self.get_logger().debug(
                        f"Rack {container_code} 不存在於資料庫中")

            # 3. 安全清除：只清除 is_carry=0 且不在映射中的 rack
            cleared_count = self._safe_clear_rack_agv_mapping(session, robot_containers)

            # 4. 如果有更新，觸發前端更新
            if updated_count > 0 or cleared_count > 0:
                ModifyLog.mark(session, "rack")
                if updated_count > 0:
                    self.get_logger().info(f"✅ 同步了 {updated_count} 個 rack.agv_id")
                if cleared_count > 0:
                    self.get_logger().info(f"✅ 安全清除了 {cleared_count} 個 rack.agv_id")

        except Exception as e:
            self.get_logger().error(f"同步 Rack-AGV 映射時發生錯誤: {e}")
            if self.get_logger().isEnabledFor(10):  # DEBUG level
                tb_str = traceback.format_exc()
                self.get_logger().debug(f"堆疊訊息:\n{tb_str}")

    def _safe_clear_rack_agv_mapping(self, session, robot_containers: dict = None):
        """
        安全清除 rack.agv_id
        只清除 is_carry=0（未被搬運）且不在 robot_containers 映射中的 rack

        Args:
            session: 資料庫 session
            robot_containers: robotId → containerCode 映射（可選）

        Returns:
            int: 清除的數量
        """
        try:
            if robot_containers is None:
                robot_containers = {}

            # 查詢需要清除的 rack：
            # 1. is_carry = 0（未被搬運）
            # 2. agv_id 不為 None
            # 3. name 不在當前的 robot_containers 映射中
            query = select(Rack).where(
                Rack.is_carry == 0,
                Rack.agv_id != None
            )

            racks_to_check = session.exec(query).all()

            cleared_count = 0
            for rack in racks_to_check:
                # 檢查是否在當前映射中
                if rack.name not in robot_containers:
                    old_agv_id = rack.agv_id
                    rack.agv_id = None
                    cleared_count += 1
                    self.get_logger().info(
                        f"🧹 安全清除: Rack {rack.name} agv_id {old_agv_id} → None (is_carry=0)")

            return cleared_count

        except Exception as e:
            self.get_logger().error(f"安全清除 Rack-AGV 映射時發生錯誤: {e}")
            return 0

    def _get_location_id_from_node_code(self, node_code: str, session) -> Optional[int]:
        """
        將 KUKA nodeCode 映射到 location_id

        映射逻辑：
        1. 查询 kuka_node 表，匹配 kuka_node.uuid = nodeCode
        2. 获取 kuka_node.id
        3. 查询 location 表，匹配 location.node_id = kuka_node.id
        4. 返回 location.id

        Args:
            node_code: KUKA 节点代码（如 "AlanACT-AlanSec1-26"）
            session: 数据库 session

        Returns:
            Optional[int]: location_id 或 None（映射失败时）
        """
        if not node_code:
            return None

        try:
            # 步骤1: 查询 KukaNode（通过 uuid 匹配，而不是 name）
            kuka_node = session.exec(
                select(KukaNode).where(KukaNode.uuid == node_code)
            ).first()

            if not kuka_node:
                # 可能是新节点，记录 debug 日志
                self.get_logger().debug(f"找不到 KukaNode: {node_code}")
                return None

            # 步骤2: 使用 kuka_node.id 查询 Location
            location = session.exec(
                select(Location).where(Location.node_id == kuka_node.id)
            ).first()

            if location:
                return location.id
            else:
                self.get_logger().debug(
                    f"KukaNode {node_code} (id={kuka_node.id}) 沒有對應的 Location")
                return None

        except Exception as e:
            self.get_logger().error(f"映射 nodeCode 到 location_id 時發生錯誤: {e}")
            if self.get_logger().isEnabledFor(10):  # DEBUG level
                tb_str = traceback.format_exc()
                self.get_logger().debug(f"堆疊訊息:\n{tb_str}")
            return None

    def _normalize_direction(self, orientation: float) -> int:
        """将 KUKA orientation 规范化到 10 的倍数（保持原始正負號）"""
        return round(orientation / 10) * 10

    def _validate_robot_data(self, robot_data: dict) -> bool:
        """
        驗證機器人資料的完整性

        Args:
            robot_data: 機器人資料字典

        Returns:
            bool: 資料是否有效
        """
        required_fields = ["robotId", "x", "y", "robotOrientation"]
        for field in required_fields:
            if field not in robot_data:
                self.get_logger().warning(f"機器人資料缺少必要欄位: {field}")
                return False

        # 驗證數值欄位
        try:
            float(robot_data.get("x"))
            float(robot_data.get("y"))
            float(robot_data.get("robotOrientation"))
        except (ValueError, TypeError) as e:
            self.get_logger().warning(f"機器人資料包含無效的數值: {e}")
            return False

        return True

    def dispatch(self):
        """KUKA400i AGV 簡單任務派發"""
        try:
            # 🆕 優先同步已派發任務的狀態和 AGV ID（無論是否有新任務要派發）
            self.sync_task_status_and_agv_ids()

            # 1. 查詢閒置的 KUKA400i AGV
            idle_kuka400i_agvs = self.kuka_fleet.select_agv(KukaFleetAdapter.STATUS_IDLE)
            idle_kuka400i_agv_ids = [int(agv["id"]) for agv in idle_kuka400i_agvs]

            if not idle_kuka400i_agv_ids:
                # self.get_logger().debug("目前沒有閒置的 KUKA400i AGV")
                return

            self.get_logger().info(f"API 查詢到閒置 KUKA400i AGV: {idle_kuka400i_agv_ids}")

            with self.db_pool.get_session() as session:
                # 2. 確認資料庫中的 AGV 狀態
                enable_kuka400i_agvs = session.exec(
                    select(AGV).where(
                        AGV.enable == 1,
                        AGV.model == "KUKA400i",
                        AGV.id.in_(idle_kuka400i_agv_ids)
                    )
                ).all()
                available_kuka400i_agv_ids = [agv.id for agv in enable_kuka400i_agvs]

                if not available_kuka400i_agv_ids:
                    # self.get_logger().debug("資料庫中沒有可用的 KUKA400i AGV")
                    return

                self.get_logger().info(f"閒置且可用 KUKA400i AGV: {available_kuka400i_agv_ids}")

                # 3. 查詢待執行的 KUKA 任務 (只選擇支援的 work_id)
                from shared_constants.work_ids import WorkIds
                kuka400i_tasks = session.exec(
                    select(Task).where(
                        Task.status_id == TaskStatus.PENDING,  # 待處理 (WCS-任務已接受，待處理)
                        Task.mission_code == None,  # 尚未指定任務代碼
                        Task.parameters["model"].as_string() == "KUKA400i",  # 使用小寫 model
                        Task.work_id.in_(WorkIds.KUKA_SUPPORTED_WORK_IDS)  # 只選擇支援的 work_id
                    ).order_by(Task.priority.desc())  # 優先級高的數字先執行
                ).all()
                
                if not kuka400i_tasks:
                    # self.get_logger().debug("目前沒有 KUKA400i 任務需要處理")
                    return

                self.get_logger().info(f"KUKA400i 任務: {[task.id for task in kuka400i_tasks]}")

                # 4. 簡化的任務派發邏輯 - 不指定 robotId，由 KUKA Fleet Manager 自動分配
                for task in kuka400i_tasks:
                    success = self._dispatch_task_to_agv(session, task)
                    if success:
                        self.get_logger().info(f"✅ 任務 {task.id} 成功派發 (由 KUKA Fleet Manager 自動分配車輛)")
                    else:
                        self.get_logger().warning(f"❌ 任務 {task.id} 派發失敗")

                # 🆕 已改為每個任務立即提交，不再需要批量提交
                # 每個任務在 _dispatch_task_to_agv 中已經 commit，確保狀態即時更新

        except Exception as e:
            self.get_logger().error(f"KUKA 任務派發時發生錯誤: {e}")

    def sync_task_status_and_agv_ids(self):
        """
        同步 KUKA 任務的狀態和 AGV ID（批量查詢優化版）
        一次性查詢所有 KUKA Fleet 的 jobs（所有狀態），然後反向匹配資料庫中的任務

        功能：
        1. 同步 AGV ID (task.agv_id) - 由 KUKA Fleet 自動分配的機器人
        2. 同步任務狀態 (task.status_id) - 根據 KUKA API 返回的狀態

        ⚠️ KUKA Fleet API 狀態碼（官方文檔）
        狀態映射：
        - KUKA Status 10 (待執行) → status_id 2 (READY_TO_EXECUTE)
        - KUKA Status 20 (執行中) → status_id 3 (EXECUTING)
        - KUKA Status 25 (等待放行) → status_id 3 (EXECUTING)
        - KUKA Status 28 (取消中) → status_id 5 (CANCELLING)
        - KUKA Status 30 (已完成) → status_id 4 (COMPLETED)
        - KUKA Status 31 (已取消) → status_id 54 (CANCELLED)
        - KUKA Status 35 (手動完成) → status_id 4 (COMPLETED)
        - KUKA Status 50 (告警) → status_id 6 (ERROR)
        - KUKA Status 60 (流程啟動異常) → status_id 6 (ERROR)
        """
        try:
            # 1. 查詢 KUKA Fleet Manager 中所有狀態的 jobs
            result = self.kuka_fleet.api_client.job_query({})

            if not result.get("success"):
                self.get_logger().warning("KUKA Fleet API 調用失敗")
                return

            # 處理兩種 data 格式：
            # 格式 1: {"data": {"jobs": [...]}}
            # 格式 2: {"data": [...]}
            data = result.get("data")
            if data is None:
                return

            if isinstance(data, dict):
                jobs = data.get("jobs", [])
            elif isinstance(data, list):
                jobs = data
            else:
                self.get_logger().warning(f"未知的 data 格式: {type(data)}")
                return

            if not jobs:
                return

            # 2. 提取所有 mission_codes 並批量查詢資料庫（優化：1次查詢）
            mission_codes = [job.get("jobCode") for job in jobs if job.get("jobCode")]

            if not mission_codes:
                self.get_logger().debug("所有 jobs 都沒有 jobCode，跳過同步")
                return

            updated_count = 0
            with self.db_pool.get_session() as session:
                # 3. 批量查詢資料庫（一次查詢所有匹配的任務）
                tasks = session.exec(
                    select(Task).where(
                        Task.mission_code.in_(mission_codes),
                        Task.parameters["model"].as_string() == "KUKA400i"
                    )
                ).all()

                # 4. 建立 mission_code → task 映射（記憶體查找）
                task_map = {task.mission_code: task for task in tasks}

                # 5. 遍歷每個 job，從映射中查找並更新
                for job in jobs:
                    try:
                        job_code = job.get("jobCode")
                        robot_id = job.get("robotId")

                        if not job_code:
                            continue

                        # 從映射中查找任務（O(1) 查找）
                        task = task_map.get(job_code)

                        if not task:
                            # 資料庫中沒有對應的任務，直接跳過（不記錄）
                            continue

                        # 記錄舊狀態
                        old_agv_id = task.agv_id
                        old_status_id = task.status_id
                        task_updated = False

                        # 6. 同步 AGV ID（如果 robot_id 存在且 task.agv_id 為空）
                        if robot_id and not task.agv_id:
                            task.agv_id = int(robot_id)
                            if task.parameters is None:
                                task.parameters = {}
                            task.parameters["agvId"] = int(robot_id)
                            task_updated = True
                            self.get_logger().info(
                                f"✅ 任務 {task.id} 同步 AGV ID: {old_agv_id} → {task.agv_id}")

                        # 7. 同步任務狀態（根據 KUKA status）
                        kuka_status = job.get("status")
                        if kuka_status is not None:
                            status_changed = self._sync_task_status_from_kuka(
                                task, kuka_status, old_status_id)
                            if status_changed:
                                task_updated = True

                        # 8. 提交變更
                        if task_updated:
                            from datetime import datetime
                            task.updated_at = datetime.utcnow()
                            session.commit()
                            updated_count += 1

                    except Exception as e:
                        self.get_logger().error(
                            f"處理 job {job.get('jobCode', 'unknown')} 時發生錯誤: {e}")
                        import traceback
                        self.get_logger().error(f"堆棧跟踪: {traceback.format_exc()}")
                        continue

            # 9. 最終統計（簡化日誌）
            self.get_logger().info(
                f"查詢到 {len(jobs)} 個 KUKA jobs，成功同步 {updated_count} 個任務")

        except Exception as e:
            self.get_logger().error(f"同步任務 AGV ID 時發生錯誤: {e}")
            import traceback
            self.get_logger().error(f"🔍 [DEBUG] 完整堆棧: {traceback.format_exc()}")

    def _dispatch_task_to_agv(self, session, task) -> bool:
        """
        派發任務 (不指定 AGV，由 KUKA Fleet Manager 自動分配)

        Args:
            session: 資料庫會話
            task: 任務物件

        Returns:
            bool: 是否成功派發
        """
        try:
            # 🆕 立即標記為 READY_TO_EXECUTE，防止重複派發
            task.status_id = TaskStatus.READY_TO_EXECUTE
            session.commit()
            self.get_logger().info(f"任務 {task.id} 狀態已更新為 READY_TO_EXECUTE，防止重複選中")

            # 生成任務代碼
            kuka_mission_code = str(uuid.uuid4())

            from shared_constants.work_ids import WorkIds

            # 🆕 檢查是否指定了 AGV
            robot_id = task.agv_id or (task.parameters.get('agvId') if task.parameters else None)
            if robot_id:
                self.get_logger().info(
                    f"預派發任務 {task.id} (指定 AGV {robot_id}) kuka_mission_code:{kuka_mission_code}")
            else:
                self.get_logger().info(
                    f"預派發任務 {task.id} (KUKA Fleet Manager 自動分配車輛) kuka_mission_code:{kuka_mission_code}")

            self.get_logger().info(f"work_id: {task.work_id} ({WorkIds.get_description(task.work_id)})")

            # 根據 work_id 執行對應的 KUKA API
            result = self._execute_kuka_api(task, kuka_mission_code)

            if result["success"]:
                # 更新任務狀態 (agv_id 由 KUKA Fleet callback 更新)
                task.mission_code = kuka_mission_code
                task.status_id = TaskStatus.EXECUTING
                session.commit()
                self.get_logger().info(
                    f"✅ 任務 {task.id} 已派發並執行，mission_code: {kuka_mission_code}")
                return True
            else:
                # 🆕 增強的錯誤日誌輸出（傳入實際的 robot_id）
                self._log_detailed_dispatch_failure(session, task, robot_id, kuka_mission_code, result)

                # 記錄失敗信息到 task.parameters 中的 rcs_kuka_response
                if task.parameters is None:
                    task.parameters = {}
                task.parameters["rcs_kuka_response"] = {
                    "success": False,
                    "error_code": result.get("code", "UNKNOWN"),
                    "error_message": result.get("message", "Unknown error"),
                    "full_response": result,
                    "failed_mission_code": kuka_mission_code,
                    "failure_timestamp": str(uuid.uuid4())  # 使用 uuid 作為唯一時間戳
                }
                # 更新任務狀態為錯誤狀態，防止重複選擇
                task.status_id = TaskStatus.ERROR  # 設置為錯誤狀態 (6)
                session.commit()
                self.get_logger().warning(
                    f"❌ 任務 {task.id} 派發失敗，已記錄錯誤信息並更新狀態為錯誤 (status_id: {TaskStatus.ERROR})")
                return False

        except Exception as e:
            self.get_logger().error(f"派發任務 {task.id} 時發生錯誤: {e}")
            # 記錄異常信息到 task.parameters 中的 rcs_kuka_response
            if task.parameters is None:
                task.parameters = {}
            task.parameters["rcs_kuka_response"] = {
                "success": False,
                "error_code": "EXCEPTION",
                "error_message": str(e),
                "full_response": {"error": "Exception occurred", "details": str(e)},
                "failed_mission_code": kuka_mission_code if 'kuka_mission_code' in locals() else "NOT_GENERATED",
                "failure_timestamp": str(uuid.uuid4())  # 使用 uuid 作為唯一時間戳
            }
            # 更新任務狀態為錯誤狀態，防止重複選擇
            task.status_id = TaskStatus.ERROR  # 設置為錯誤狀態 (6)
            session.commit()
            self.get_logger().warning(
                f"❌ 任務 {task.id} 派發異常，已記錄錯誤信息並更新狀態為錯誤 (status_id: {TaskStatus.ERROR})")
            return False

    def _execute_kuka_api(self, task, mission_code: str) -> dict:
        """
        執行 KUKA API 調用（支持可選的 robotId 指定）

        Args:
            task: 任務物件
            mission_code: 任務代碼

        Returns:
            dict: API 調用結果
        """
        try:
            from shared_constants.work_ids import WorkIds
            result = {"success": False}

            # 🆕 提取可選的 robot_id 參數
            # 優先使用資料庫字段 task.agv_id，如果沒有則從 parameters 中讀取
            robot_id = None
            if task.agv_id:
                robot_id = task.agv_id
                self.get_logger().info(f"任務 {task.id} 指定 AGV: {robot_id} (來源: task.agv_id)")
            elif task.parameters and task.parameters.get('agvId'):
                robot_id = task.parameters.get('agvId')
                self.get_logger().info(f"任務 {task.id} 指定 AGV: {robot_id} (來源: parameters.agvId)")
            else:
                self.get_logger().info(f"任務 {task.id} 未指定 AGV，由 KUKA Fleet Manager 自動分配")

            # 基於 work_id 的簡單派發邏輯
            if task.work_id == WorkIds.KUKA_MOVE:  # KUKA 移動
                self.get_logger().info(f"parameters['nodes']: {task.parameters['nodes']}")
                if task.parameters['nodes']:
                    result = self.kuka_fleet.move(
                        task.parameters['nodes'], mission_code, robot_id=robot_id)
                else:
                    self.get_logger().warn(
                        f"缺少參數 parameters['nodes'] 無法執行任務 {task.id}")

            elif task.work_id == WorkIds.KUKA_RACK_MOVE:  # KUKA 移動貨架
                self.get_logger().info(f"parameters['nodes']: {task.parameters['nodes']}")
                if task.parameters['nodes']:
                    result = self.kuka_fleet.rack_move(
                        task.parameters['nodes'], mission_code, robot_id=robot_id)
                else:
                    self.get_logger().warn(
                        f"缺少參數 parameters['nodes'] 無法執行任務 {task.id}")

            elif task.work_id == WorkIds.KUKA_WORKFLOW:  # KUKA template 流程任務
                template_code = task.parameters.get('templateCode')
                self.get_logger().info(f"templateCode: {template_code}")
                if template_code:
                    result = self.kuka_fleet.workflow(
                        template_code, mission_code, robot_id=robot_id)
                else:
                    self.get_logger().warn(
                        f"缺少參數 parameters['templateCode'] 無法執行任務 {task.id}")

            else:  # 理論上不會到達這裡，因為查詢時已過濾
                self.get_logger().error(f"意外的 task.work_id: {task.work_id} ({WorkIds.get_description(task.work_id)}) (應該已在查詢時過濾)")
            return result

        except Exception as e:
            self.get_logger().error(f"執行 KUKA API 時發生錯誤: {e}")
            return {"success": False, "error": str(e)}

    def on_container_update(self, containers: list):
        """
        🔴 關鍵功能 - 絕對不可刪除！
        當 KukaFleetAdapter 查詢到容器狀態時，更新資料庫中的 Rack 資料
        KUKA 的 container 對應到資料庫中的 Rack 表

        此功能負責：
        - 同步 KUKA 容器狀態到 Rack 表
        - 更新 is_carry 和 is_in_map 狀態
        - 觸發 ModifyLog 讓前端即時更新 Rack 顯示
        - 前端依賴此機制顯示 Rack 的搬運和入場狀態

        Args:
            containers: 容器狀態列表，每個元素包含:
                - containerCode: 容器代碼 (對應 Rack.name)
                - isCarry: 是否正在被搬運
                - inMapStatus: 是否在地圖中（入場狀態）
        """
        if not self.db_pool:
            self.get_logger().error("資料庫連線池不可用，無法更新容器狀態")
            return

        # 日誌頻率控制
        self._container_log_counter += 1
        should_log_summary = (self._container_log_counter % self._container_log_interval == 0)

        if not containers:
            if should_log_summary:
                self.get_logger().debug("KUKA Fleet 查詢結果：沒有容器")
            return

        # 只在每 N 次查詢時輸出匯總信息
        if should_log_summary:
            container_codes = [c.get("containerCode", "Unknown") for c in containers]
            self.get_logger().info(
                f"🔄 KUKA Fleet 查詢到 {len(containers)} 個容器: {', '.join(container_codes)}"
            )

        try:
            with self.db_pool.get_session() as session:
                updated_count = 0
                changed_racks = []  # 記錄有狀態變化的 Rack

                for container in containers:
                    result = self._update_single_container(session, container)
                    if result['updated']:
                        updated_count += 1
                    if result['changed']:
                        changed_racks.append(result['rack_name'])

                if updated_count > 0:
                    # 🔴 關鍵：標記 Rack 資料已更新，觸發前端更新
                    # 絕對不可移除！前端 agvc_ui_socket.py 監聽此事件
                    ModifyLog.mark(session, "rack")
                    session.commit()

                    # 根據是否有變化輸出不同日誌
                    if changed_racks:
                        self.get_logger().info(f"✅ Rack 狀態變化: {', '.join(changed_racks)}")
                    elif should_log_summary:
                        # 只在匯總時輸出無變化訊息（降低刷屏）
                        self.get_logger().debug(f"已同步 {updated_count} 個容器（無狀態變化）")

        except Exception as e:
            self.get_logger().error(f"更新 KUKA 容器狀態時發生錯誤: {e}")

    def _update_single_container(self, session, container_data: dict) -> dict:
        """
        更新單個容器的資料

        Args:
            session: 資料庫 session
            container_data: 容器資料字典

        Returns:
            dict: {
                'updated': bool,     # 是否成功更新
                'changed': bool,     # 狀態是否有變化
                'rack_name': str     # Rack 名稱
            }
        """
        container_code = container_data.get("containerCode")
        if not container_code:
            self.get_logger().warning("容器資料缺少 containerCode")
            return {'updated': False, 'changed': False, 'rack_name': None}

        try:
            # 根據 containerCode (name) 查找 Rack
            rack = session.exec(
                select(Rack).where(Rack.name == container_code)
            ).first()

            if not rack:
                # Rack 不在資料庫中，可能是新容器或未註冊
                self.get_logger().debug(f"Rack {container_code} 不存在於資料庫中")
                return {'updated': False, 'changed': False, 'rack_name': container_code}

            # 記錄舊狀態（用於檢測變化）
            old_is_carry = rack.is_carry
            old_is_in_map = rack.is_in_map
            old_location_id = rack.location_id
            old_direction = rack.direction

            # 更新 is_carry 狀態 (是否被搬運)
            is_carry = container_data.get("isCarry")
            if is_carry is not None:
                rack.is_carry = 1 if is_carry else 0

            # 更新 is_in_map 狀態 (是否入場)
            is_in_map = container_data.get("inMapStatus")
            if is_in_map is not None:
                rack.is_in_map = 1 if is_in_map else 0

            # 🆕 當容器在地圖中時，根據 orientation 更新 direction
            if rack.is_in_map == 1:
                orientation = container_data.get("orientation")
                if orientation is not None:
                    try:
                        orientation_float = float(orientation)
                        rack.direction = self._normalize_direction(orientation_float)
                    except (ValueError, TypeError) as e:
                        self.get_logger().warning(
                            f"⚠️ 无法解析容器 {container_code} 的 orientation: {orientation}, error: {e}"
                        )

            # ✅ 更新：location_id 更新邏輯調整
            # 1. 當 is_carry = 1 且 is_in_map = 1 時：容器被搬運中，持續更新位置（實時追蹤）
            # 2. 當 is_carry 從 1 變 0 時：容器剛放下，更新最後位置
            # 3. 當 is_carry = 0 且之前就是 0：容器已放下，不再更新位置
            if rack.is_in_map == 1:
                # 判斷是否需要更新：
                # - 正在被搬運 (is_carry == 1)：持續更新
                # - 剛放下 (is_carry == 0 and old_is_carry == 1)：更新最後一次
                should_update = (rack.is_carry == 1) or (rack.is_carry == 0 and old_is_carry == 1)

                if should_update:
                    node_code = container_data.get("nodeCode")
                    if node_code:
                        location_id = self._get_location_id_from_node_code(node_code, session)
                        if location_id is not None and location_id != rack.location_id:
                            rack.location_id = location_id
                            # 根據狀態顯示不同的日誌
                            if rack.is_carry == 1:
                                self.get_logger().info(
                                    f"📍 Rack {rack.name} 搬運中位置更新: "
                                    f"location_id {old_location_id} → {location_id} "
                                    f"(nodeCode: {node_code})")
                            else:
                                self.get_logger().info(
                                    f"📍 Rack {rack.name} 放下位置記錄: "
                                    f"location_id {old_location_id} → {location_id} "
                                    f"(nodeCode: {node_code})")
                        elif location_id is None:
                            self.get_logger().warning(
                                f"⚠️ 無法映射 nodeCode: {node_code} → location_id (Rack: {rack.name})")
                    else:
                        self.get_logger().debug(f"Rack {rack.name} 的 nodeCode 為空")
                else:
                    # is_carry == 0 且 old_is_carry == 0：容器已放下，不更新位置
                    self.get_logger().debug(
                        f"Rack {rack.name} 已放下 (is_carry=0)，跳過位置更新")
            elif rack.is_in_map != 1:
                # 容器不在地圖中，跳過位置更新
                self.get_logger().debug(
                    f"Rack {rack.name} 不在地圖中 (is_in_map={rack.is_in_map})，"
                    f"跳過 location 更新")

            # ✅ rack.direction 更新機制：
            # 現在使用 _normalize_direction() 自動將 KUKA orientation 規範化到 10 的倍數
            # 這樣可以避免旋轉過程中的中間角度值，並保持角度的一致性

            # 檢查狀態是否有變化（包含位置和方向變化）
            changed = (
                old_is_carry != rack.is_carry or
                old_is_in_map != rack.is_in_map or
                old_location_id != rack.location_id or
                old_direction != rack.direction
            )

            # 只在狀態變化時輸出詳細日誌
            if changed:
                log_parts = [f"🔄 Rack {rack.name} 狀態變化:"]
                if old_is_carry != rack.is_carry:
                    log_parts.append(f"is_carry {old_is_carry}→{rack.is_carry}")
                if old_is_in_map != rack.is_in_map:
                    log_parts.append(f"is_in_map {old_is_in_map}→{rack.is_in_map}")
                if old_location_id != rack.location_id:
                    log_parts.append(f"location_id {old_location_id}→{rack.location_id}")
                if old_direction != rack.direction:
                    log_parts.append(f"direction {old_direction}°→{rack.direction}°")

                self.get_logger().info(", ".join(log_parts))

            return {
                'updated': True,
                'changed': changed,
                'rack_name': rack.name
            }

        except Exception as e:
            self.get_logger().error(f"更新容器 {container_code} 狀態時發生錯誤: {e}")
            if self.get_logger().isEnabledFor(10):  # DEBUG level
                tb_str = traceback.format_exc()
                self.get_logger().debug(f"堆疊訊息:\n{tb_str}")
            return {'updated': False, 'changed': False, 'rack_name': container_code}

    def _log_detailed_dispatch_failure(self, session, task, agv_id: Optional[int], mission_code: str, result: dict):
        """
        記錄詳細的任務派發失敗診斷資訊

        提供完整的錯誤上下文，包括：
        - KUKA API 錯誤碼解析
        - AGV 當前狀態
        - Rack/Container 當前狀態
        - 故障排除建議

        Args:
            session: 資料庫會話
            task: 任務物件
            agv_id: AGV ID（可選，None 表示由 KUKA Fleet Manager 自動分配）
            mission_code: 任務代碼
            result: KUKA API 返回結果
        """
        error_code = result.get("code", "UNKNOWN")
        error_message = result.get("message", "Unknown error")

        # 🔴 錯誤概覽
        self.get_logger().error("=" * 80)
        self.get_logger().error(f"❌ KUKA 任務派發失敗 - 任務 ID: {task.id}")
        self.get_logger().error("=" * 80)

        # 📋 基本錯誤資訊
        self.get_logger().error(f"📌 錯誤代碼: {error_code}")
        self.get_logger().error(f"📌 錯誤訊息: {error_message}")
        self.get_logger().error(f"📌 AGV ID: {agv_id}")
        self.get_logger().error(f"📌 Mission Code: {mission_code}")

        # 🔍 錯誤碼解析與說明
        error_explanation = self._explain_kuka_error(error_code, error_message)
        if error_explanation:
            self.get_logger().error("")
            self.get_logger().error("🔍 錯誤說明:")
            for line in error_explanation.split('\n'):
                self.get_logger().error(f"   {line}")

        # 🤖 AGV 當前狀態
        self.get_logger().error("")
        self.get_logger().error("🤖 AGV 當前狀態:")
        agv_status = None  # 初始化變數，避免未定義錯誤
        if agv_id is not None:
            agv_status = self._get_agv_status_info(session, agv_id)
            for line in agv_status.split('\n'):
                self.get_logger().error(f"   {line}")
        else:
            self.get_logger().error("   ⚠️ 任務未指定 AGV，由 KUKA Fleet Manager 自動分配")

        # 📦 Rack/Container 狀態
        if task.rack_id:
            self.get_logger().error("")
            self.get_logger().error("📦 Rack/Container 狀態:")
            rack_status = self._get_rack_status_info(session, task.rack_id)
            for line in rack_status.split('\n'):
                self.get_logger().error(f"   {line}")

        # 📋 任務詳情
        self.get_logger().error("")
        self.get_logger().error("📋 任務詳情:")
        self.get_logger().error(f"   Task ID: {task.id}")
        self.get_logger().error(f"   Work ID: {task.work_id}")
        self.get_logger().error(f"   Priority: {task.priority}")
        self.get_logger().error(f"   Nodes: {task.parameters.get('nodes', 'N/A')}")
        self.get_logger().error(f"   Rack ID: {task.rack_id or 'N/A'}")

        # 💡 故障排除建議
        suggestions = self._get_troubleshooting_suggestions(error_code, error_message, agv_status, rack_status if task.rack_id else None)
        if suggestions:
            self.get_logger().error("")
            self.get_logger().error("💡 故障排除建議:")
            for i, suggestion in enumerate(suggestions, 1):
                self.get_logger().error(f"   {i}. {suggestion}")

        self.get_logger().error("=" * 80)

    def _explain_kuka_error(self, error_code: str, error_message: str) -> str:
        """
        解釋 KUKA API 錯誤碼含義

        Args:
            error_code: 錯誤代碼
            error_message: 錯誤訊息

        Returns:
            str: 錯誤說明文字
        """
        # KUKA Fleet Manager 常見錯誤碼映射
        error_map = {
            "100001": {
                "title": "Container 狀態錯誤",
                "desc": "Container (Rack) 當前狀態不是 idle (空閒)，可能正在被使用或處於非空閒狀態"
            },
            "100002": {
                "title": "Robot 狀態錯誤",
                "desc": "Robot (AGV) 當前狀態不允許接受新任務"
            },
            "100003": {
                "title": "任務衝突",
                "desc": "目標位置或路徑上存在衝突"
            },
            "VALIDATION_ERROR": {
                "title": "參數驗證失敗",
                "desc": "請求參數不符合 KUKA API 要求"
            },
            "RESOURCE_NOT_FOUND": {
                "title": "資源不存在",
                "desc": "指定的 Robot、Container 或 Node 不存在於 KUKA Fleet Manager"
            },
            "RESOURCE_CONFLICT": {
                "title": "資源衝突",
                "desc": "資源已被其他任務佔用"
            }
        }

        error_info = error_map.get(error_code, {
            "title": "未知錯誤",
            "desc": f"錯誤代碼 {error_code} 未在映射表中，請參考 KUKA Fleet Manager 文檔"
        })

        return f"{error_info['title']}\n{error_info['desc']}\n原始訊息: {error_message}"

    def _get_agv_status_info(self, session, agv_id: Optional[int]) -> str:
        """
        獲取 AGV 當前狀態資訊

        Args:
            session: 資料庫會話
            agv_id: AGV ID（可選，None 表示未指定 AGV）

        Returns:
            str: AGV 狀態資訊
        """
        try:
            if agv_id is None:
                return "⚠️ 未指定 AGV（由 KUKA Fleet Manager 自動分配）"

            agv = agv_crud.get_by_id(session, agv_id)
            if not agv:
                return f"❌ AGV {agv_id} 不存在於資料庫"

            status_map = {
                1: "離場 (REMOVED)",
                2: "離線 (OFFLINE)",
                3: "空閒 (IDLE)",
                4: "任務中 (RUNNING)",
                5: "充電中 (CHARGING)",
                6: "更新中 (UPDATING)",
                7: "錯誤 (ERROR)"
            }

            status_name = status_map.get(agv.status_id, f"未知狀態 ({agv.status_id})")

            info = [
                f"ID: {agv.id}, Name: {agv.name}",
                f"Status: {status_name}",
                f"Battery: {agv.battery}%",
                f"Enable: {'是' if agv.enable else '否'}",
                f"Last Node: {agv.last_node_id or 'N/A'}",
                f"Position: ({agv.x:.1f}, {agv.y:.1f}), Heading: {agv.heading:.1f}°"
            ]

            # 🆕 檢查 AGV 是否正在執行其他任務
            from db_proxy.models import Task
            from shared_constants.task_status import TaskStatus
            running_tasks = session.exec(
                select(Task).where(
                    Task.agv_id == agv_id,
                    Task.status_id.in_([TaskStatus.READY_TO_EXECUTE, TaskStatus.EXECUTING])
                )
            ).all()

            if running_tasks:
                info.append(f"⚠️ 警告: AGV 當前正在執行 {len(running_tasks)} 個任務: {[t.id for t in running_tasks]}")

            return '\n'.join(info)

        except Exception as e:
            return f"❌ 獲取 AGV 狀態失敗: {e}"

    def _get_rack_status_info(self, session, rack_id: int) -> str:
        """
        獲取 Rack/Container 當前狀態資訊

        Args:
            session: 資料庫會話
            rack_id: Rack ID

        Returns:
            str: Rack 狀態資訊
        """
        try:
            rack = session.exec(
                select(Rack).where(Rack.id == rack_id)
            ).first()

            if not rack:
                return f"❌ Rack {rack_id} 不存在於資料庫"

            info = [
                f"ID: {rack.id}, Name: {rack.name}",
                f"Is Carry: {'是' if rack.is_carry else '否'}",
                f"Is In Map: {'是' if rack.is_in_map else '否'}",
                f"Location ID: {rack.location_id or 'N/A'}",
                f"AGV ID: {rack.agv_id or 'N/A'}",
                f"Direction: {rack.direction}°" if rack.direction is not None else "Direction: N/A",
                f"Carrier Bitmap: {rack.carrier_bitmap or '00000000'}"
            ]

            # 🆕 警告標記
            warnings = []
            if rack.is_carry == 1:
                warnings.append("⚠️ Rack 正在被搬運中 (is_carry=1)")
            if rack.agv_id and rack.agv_id != 0:
                warnings.append(f"⚠️ Rack 已綁定到 AGV {rack.agv_id}")

            if warnings:
                info.extend(warnings)

            return '\n'.join(info)

        except Exception as e:
            return f"❌ 獲取 Rack 狀態失敗: {e}"

    def _get_troubleshooting_suggestions(self, error_code: str, error_message: str,
                                        agv_status: str, rack_status: Optional[str]) -> list:
        """
        根據錯誤情況提供故障排除建議

        Args:
            error_code: 錯誤代碼
            error_message: 錯誤訊息
            agv_status: AGV 狀態資訊
            rack_status: Rack 狀態資訊 (可選)

        Returns:
            list: 建議列表
        """
        suggestions = []

        # Container is not idle 特定建議
        if error_code == "100001" or "Container is not idle" in error_message:
            suggestions.append("檢查 Rack 是否正在被其他 AGV 搬運 (is_carry=1)")
            suggestions.append("確認 Rack 是否已綁定到其他 AGV (agv_id != None)")
            suggestions.append("檢查 KUKA Fleet Manager 中的 Container 狀態是否與資料庫一致")
            suggestions.append("嘗試在 KUKA Fleet Manager UI 中手動查看該 Container 的詳細狀態")

            # 根據 rack_status 提供更具體的建議
            if rack_status:
                if "is_carry=1" in rack_status or "Is Carry: 是" in rack_status:
                    suggestions.append("⚠️ Rack 確實處於搬運狀態，需等待當前任務完成")
                if "AGV ID:" in rack_status and "N/A" not in rack_status:
                    suggestions.append("⚠️ Rack 已綁定到 AGV，可能存在狀態同步問題")

        # AGV 狀態相關建議
        if agv_status and "任務中 (RUNNING)" in agv_status:
            suggestions.append("AGV 當前正在執行任務，無法接受新任務")
        elif agv_status and "錯誤 (ERROR)" in agv_status:
            suggestions.append("AGV 處於錯誤狀態，需先處理 AGV 錯誤")
        elif agv_status and "離線 (OFFLINE)" in agv_status:
            suggestions.append("AGV 離線，檢查 AGV 與 Fleet Manager 的連接")

        # 通用建議
        if not suggestions:
            suggestions.append("檢查 KUKA Fleet Manager 系統日誌")
            suggestions.append("確認網路連接和 API 通訊正常")
            suggestions.append("嘗試在 KUKA Fleet Manager UI 中手動執行相同操作")

        return suggestions

    def _sync_task_status_from_kuka(self, task: Task, kuka_status: int, old_status_id: int) -> bool:
        """
        根據 KUKA API 狀態同步任務狀態

        ⚠️ KUKA Fleet API 狀態碼（官方文檔）

        狀態映射規則（簡化條件：避免更新終態任務，保證幂等性）：
        - KUKA Status 10 (待執行) → status_id 2 (READY_TO_EXECUTE)
        - KUKA Status 20 (執行中) → status_id 3 (EXECUTING)
        - KUKA Status 25 (等待放行) → status_id 3 (EXECUTING)
        - KUKA Status 28 (取消中) → status_id 5 (CANCELLING)
        - KUKA Status 30 (已完成) → status_id 4 (COMPLETED)
        - KUKA Status 31 (已取消) → status_id 54 (CANCELLED)
        - KUKA Status 35 (手動完成) → status_id 4 (COMPLETED)
        - KUKA Status 50 (告警) → status_id 6 (ERROR)
        - KUKA Status 60 (流程啟動異常) → status_id 6 (ERROR)

        Args:
            task: 任務物件
            kuka_status: KUKA API 返回的狀態碼（10 的倍數格式）
            old_status_id: 任務原狀態 ID

        Returns:
            bool: 狀態是否有變更
        """
        # KUKA 狀態名稱映射（用於日誌）- 官方狀態碼: 10, 20, 25, 28, 30, 31, 35, 50, 60
        kuka_status_names = {
            10: "待執行",
            20: "執行中",
            25: "等待放行",
            28: "取消中",
            30: "已完成",
            31: "已取消",
            35: "手動完成",
            50: "告警",
            60: "流程啟動異常"
        }

        # 系統狀態名稱映射（用於日誌）
        system_status_names = {
            1: "PENDING",
            2: "READY_TO_EXECUTE",
            3: "EXECUTING",
            4: "COMPLETED",
            5: "CANCELLING",
            6: "ERROR"
        }

        kuka_status_name = kuka_status_names.get(kuka_status, f"Unknown({kuka_status})")
        old_status_name = system_status_names.get(old_status_id, f"Unknown({old_status_id})")

        # 狀態映射邏輯（簡化條件：避免更新終態任務，保證幂等性）
        # ⚠️ KUKA API 官方狀態碼
        # 狀態碼值: 10=待執行, 20=執行中, 25=等待放行, 28=取消中, 30=已完成, 31=已取消, 35=手動完成, 50=告警, 60=流程啟動異常
        new_status_id = None

        if kuka_status == 10:  # 待執行
            # 更新為 READY_TO_EXECUTE（避免覆蓋已開始執行的任務）
            if task.status_id not in [TaskStatus.READY_TO_EXECUTE, TaskStatus.EXECUTING, TaskStatus.COMPLETED, TaskStatus.ERROR, TaskStatus.CANCELLED]:
                new_status_id = TaskStatus.READY_TO_EXECUTE
        elif kuka_status == 20:  # 執行中
            # 更新為 EXECUTING（避免覆蓋終態）
            if task.status_id not in [TaskStatus.EXECUTING, TaskStatus.COMPLETED, TaskStatus.ERROR, TaskStatus.CANCELLED]:
                new_status_id = TaskStatus.EXECUTING
        elif kuka_status == 25:  # 等待放行
            # 視為執行中的一種狀態
            if task.status_id not in [TaskStatus.EXECUTING, TaskStatus.COMPLETED, TaskStatus.ERROR, TaskStatus.CANCELLED]:
                new_status_id = TaskStatus.EXECUTING
        elif kuka_status == 28:  # 取消中
            # 更新為 CANCELLING（避免覆蓋已完成/已取消）
            if task.status_id not in [TaskStatus.CANCELLING, TaskStatus.COMPLETED, TaskStatus.ERROR, TaskStatus.CANCELLED]:
                new_status_id = TaskStatus.CANCELLING
        elif kuka_status == 30:  # 已完成
            # 更新為 COMPLETED（避免重複更新已完成任務）
            if task.status_id not in [TaskStatus.COMPLETED, TaskStatus.ERROR, TaskStatus.CANCELLED]:
                new_status_id = TaskStatus.COMPLETED
        elif kuka_status == 31:  # 已取消
            # 更新為 CANCELLED（終態）
            if task.status_id not in [TaskStatus.COMPLETED, TaskStatus.ERROR, TaskStatus.CANCELLED]:
                new_status_id = TaskStatus.CANCELLED
        elif kuka_status == 35:  # 手動完成
            # 視為已完成
            if task.status_id not in [TaskStatus.COMPLETED, TaskStatus.ERROR, TaskStatus.CANCELLED]:
                new_status_id = TaskStatus.COMPLETED
        elif kuka_status == 50:  # 告警
            # 🆕 優化：告警時不立即標記 ERROR，保持 EXECUTING 繼續監控
            if task.status_id == TaskStatus.EXECUTING:
                # 任務正在執行中，告警可能是臨時狀態，繼續監控
                self.get_logger().warning(
                    f"⚠️ 任務 {task.id} 處於 KUKA 告警狀態 (50)，繼續監控（不立即標記 ERROR）"
                )
                # 不修改狀態，返回 False 表示無變更
                return False
            elif task.status_id not in [TaskStatus.COMPLETED, TaskStatus.ERROR, TaskStatus.CANCELLED]:
                # 非執行中的任務遇到告警，標記為 ERROR
                new_status_id = TaskStatus.ERROR
        elif kuka_status == 60:  # 流程啟動異常
            # 更新為 ERROR（流程異常視為錯誤）
            if task.status_id not in [TaskStatus.COMPLETED, TaskStatus.ERROR, TaskStatus.CANCELLED]:
                new_status_id = TaskStatus.ERROR

        # 執行狀態更新
        if new_status_id is not None and new_status_id != task.status_id:
            task.status_id = new_status_id
            new_status_name = system_status_names.get(new_status_id, f"Unknown({new_status_id})")

            self.get_logger().info(
                f"🔄 任務 {task.id} 狀態同步: "
                f"KUKA Status={kuka_status_name} → "
                f"System Status: {old_status_name} ({old_status_id}) → {new_status_name} ({new_status_id})")
            return True
        elif new_status_id is None:
            # KUKA 狀態不需要同步（如 Pending 或未知狀態）
            self.get_logger().debug(
                f"任務 {task.id} KUKA 狀態 {kuka_status_name} 不需要同步")
            return False
        else:
            # 狀態無變化或不符合更新條件
            self.get_logger().debug(
                f"任務 {task.id} 狀態無變化或不符合更新條件: "
                f"KUKA Status={kuka_status_name}, Current Status={old_status_name}")
            return False

    def stop_monitoring(self):
        """停止 KUKA Fleet 監控"""
        if hasattr(self, 'kuka_fleet') and self.kuka_fleet:
            self.kuka_fleet.stop_monitoring()
            self.get_logger().info("KUKA Fleet 監控已停止")