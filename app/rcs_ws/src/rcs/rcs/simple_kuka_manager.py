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
from itertools import zip_longest
from kuka_fleet_adapter.kuka_fleet_adapter import KukaFleetAdapter
from db_proxy.models import AGV, ModifyLog, Rack
from db_proxy.crud.agv_crud import agv_crud
from sqlmodel import select
import traceback


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

            return True

        except Exception as e:
            self.get_logger().error(f"更新 AGV {robot_id} 狀態時發生錯誤: {e}")
            if self.get_logger().isEnabledFor(10):  # DEBUG level
                tb_str = traceback.format_exc()
                self.get_logger().debug(f"堆疊訊息:\n{tb_str}")
            return False

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
            # 1. 查詢閒置的 KUKA400i AGV
            idle_kuka400i_agvs = self.kuka_fleet.select_agv(KukaFleetAdapter.STATUS_IDLE)
            idle_kuka400i_agv_ids = [int(agv["id"]) for agv in idle_kuka400i_agvs]

            if not idle_kuka400i_agv_ids:
                # self.get_logger().debug("目前沒有閒置的 KUKA400i AGV")
                return

            self.get_logger().info(f"API 查詢到閒置 KUKA400i AGV: {idle_kuka400i_agv_ids}")
            
            with self.db_pool.get_session() as session:
                from db_proxy.models import AGV, Task
                from shared_constants.task_status import TaskStatus
                from sqlmodel import select
                
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

                # 4. 簡單的任務派發邏輯
                for agv, task in zip_longest(available_kuka400i_agv_ids, kuka400i_tasks):
                    if agv and task:
                        success = self._dispatch_task_to_agv(session, task, agv)
                        if success:
                            self.get_logger().info(f"✅ 任務 {task.id} 成功派發給 AGV {agv}")
                        else:
                            self.get_logger().warning(f"❌ 任務 {task.id} 派發給 AGV {agv} 失敗")
                    elif agv and not task:
                        self.get_logger().debug(f"AGV {agv} 目前無任務可派發")
                    elif task and not agv:
                        self.get_logger().debug(f"任務 {task.id} 目前無可用 AGV")

                session.commit()

        except Exception as e:
            self.get_logger().error(f"KUKA 任務派發時發生錯誤: {e}")

    def _dispatch_task_to_agv(self, session, task, agv_id: int) -> bool:
        """
        派發任務給指定 AGV
        
        Args:
            session: 資料庫會話
            task: 任務物件
            agv_id: AGV ID
            
        Returns:
            bool: 是否成功派發
        """
        try:
            # 生成任務代碼
            kuka_mission_code = str(uuid.uuid4())
            
            from shared_constants.work_ids import WorkIds
            self.get_logger().info(
                f"預派發任務 {task.id} 給 AGV {agv_id} kuka_mission_code:{kuka_mission_code}")
            self.get_logger().info(f"work_id: {task.work_id} ({WorkIds.get_description(task.work_id)})")
            
            # 根據 work_id 執行對應的 KUKA API
            result = self._execute_kuka_api(task, agv_id, kuka_mission_code)
            
            if result["success"]:
                # 更新任務狀態
                task.agv_id = agv_id
                task.mission_code = kuka_mission_code
                task.parameters["agvId"] = agv_id
                self.get_logger().info(
                    f"任務 {task.id} 已派發，mission_code: {kuka_mission_code}")
                return True
            else:
                self.get_logger().error(f"KUKA API 調用失敗: {result}")
                # 記錄失敗信息到 task.parameters 中的 rcs_kuka_response
                if task.parameters is None:
                    task.parameters = {}
                task.parameters["rcs_kuka_response"] = {
                    "success": False,
                    "error_code": result.get("code", "UNKNOWN"),
                    "error_message": result.get("message", "Unknown error"),
                    "full_response": result,
                    "failed_agv_id": agv_id,
                    "failed_mission_code": kuka_mission_code,
                    "failure_timestamp": str(uuid.uuid4())  # 使用 uuid 作為唯一時間戳
                }
                # 更新任務狀態為錯誤狀態，防止重複選擇
                from shared_constants.task_status import TaskStatus as const_task_status
                task.status_id = const_task_status.ERROR  # 設置為錯誤狀態 (6)
                self.get_logger().warning(
                    f"任務 {task.id} 派發失敗，已記錄錯誤信息並更新狀態為錯誤 (status_id: {const_task_status.ERROR})")
                return False
                
        except Exception as e:
            self.get_logger().error(f"派發任務給 AGV {agv_id} 時發生錯誤: {e}")
            # 記錄異常信息到 task.parameters 中的 rcs_kuka_response
            if task.parameters is None:
                task.parameters = {}
            task.parameters["rcs_kuka_response"] = {
                "success": False,
                "error_code": "EXCEPTION",
                "error_message": str(e),
                "full_response": {"error": "Exception occurred", "details": str(e)},
                "failed_agv_id": agv_id,
                "failed_mission_code": kuka_mission_code,
                "failure_timestamp": str(uuid.uuid4())  # 使用 uuid 作為唯一時間戳
            }
            # 更新任務狀態為錯誤狀態，防止重複選擇
            from shared_constants.task_status import TaskStatus
            task.status_id = TaskStatus.ERROR  # 設置為錯誤狀態 (6)
            self.get_logger().warning(
                f"任務 {task.id} 派發異常，已記錄錯誤信息並更新狀態為錯誤 (status_id: {TaskStatus.ERROR})")
            return False

    def _execute_kuka_api(self, task, agv_id: int, mission_code: str) -> dict:
        """
        執行 KUKA API 調用（簡化版本）
        
        Args:
            task: 任務物件
            agv_id: AGV ID
            mission_code: 任務代碼
            
        Returns:
            dict: API 調用結果
        """
        try:
            from shared_constants.work_ids import WorkIds
            result = {"success": False}
            
            # 基於 work_id 的簡單派發邏輯
            if task.work_id == WorkIds.KUKA_MOVE:  # KUKA 移動
                self.get_logger().info(f"parameters['nodes']: {task.parameters['nodes']}")
                if task.parameters['nodes']:
                    result = self.kuka_fleet.move(
                        task.parameters['nodes'], agv_id, mission_code)
                else:
                    self.get_logger().warn(
                        f"缺少參數 parameters['nodes'] 無法執行任務 {task.id}")
                        
            elif task.work_id == WorkIds.KUKA_RACK_MOVE:  # KUKA 移動貨架
                self.get_logger().info(f"parameters['nodes']: {task.parameters['nodes']}")
                if task.parameters['nodes']:
                    result = self.kuka_fleet.rack_move(
                        task.parameters['nodes'], agv_id, mission_code)
                else:
                    self.get_logger().warn(
                        f"缺少參數 parameters['nodes'] 無法執行任務 {task.id}")

            elif task.work_id == WorkIds.KUKA_WORKFLOW:  # KUKA template 流程任務
                template_code = task.parameters.get('templateCode')
                self.get_logger().info(f"templateCode: {template_code}")
                if template_code:
                    result = self.kuka_fleet.workflow(
                        template_code, agv_id, mission_code)
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

            # 更新 is_carry 狀態 (是否被搬運)
            is_carry = container_data.get("isCarry")
            if is_carry is not None:
                rack.is_carry = 1 if is_carry else 0

            # 更新 is_in_map 狀態 (是否入場)
            is_in_map = container_data.get("inMapStatus")
            if is_in_map is not None:
                rack.is_in_map = 1 if is_in_map else 0

            # ⚠️ 注意：不要從 KUKA orientation 更新 rack.direction
            # 原因：
            # 1. KUKA orientation 是連續角度值（0-360°），反映容器實時物理方向
            # 2. rack.direction 是業務邏輯欄位，只能是 0 或 180
            #    - 0° = A面朝外（Port 1-16 可存取）
            #    - 180° = B面朝外（Port 17-32 可存取）
            # 3. 如果直接賦值，會在旋轉過程中產生 45°、90°、135° 等中間值
            # 4. 這會導致 AGV 狀態機（check_rack_side_state.py）的 port 驗證失敗
            #
            # 正確做法：rack.direction 應該由旋轉任務完成回調來更新
            # （在 web_api/routers/kuka.py 的 missionStateCallback 中處理）

            # 檢查狀態是否有變化（不再檢查 direction，因為它不應該在這裡更新）
            changed = (
                old_is_carry != rack.is_carry or
                old_is_in_map != rack.is_in_map
            )

            # 只在狀態變化時輸出詳細日誌
            if changed:
                log_parts = [f"🔄 Rack {rack.name} 狀態變化:"]
                if old_is_carry != rack.is_carry:
                    log_parts.append(f"is_carry {old_is_carry}→{rack.is_carry}")
                if old_is_in_map != rack.is_in_map:
                    log_parts.append(f"is_in_map {old_is_in_map}→{rack.is_in_map}")

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

    def stop_monitoring(self):
        """停止 KUKA Fleet 監控"""
        if hasattr(self, 'kuka_fleet') and self.kuka_fleet:
            self.kuka_fleet.stop_monitoring()
            self.get_logger().info("KUKA Fleet 監控已停止")