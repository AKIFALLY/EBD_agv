"""
KUKA 統一管理器
整合所有 KUKA 相關功能：機器人狀態更新、容器管理、任務派發
"""
import uuid
from itertools import zip_longest
from datetime import datetime, timezone
import asyncio
import time
from typing import Callable, Any, Optional, Dict, List
from db_proxy.connection_pool_manager import ConnectionPoolManager
from kuka_fleet_adapter.kuka_fleet_adapter import KukaFleetAdapter
from rcs.kuka_container import KukaContainer
from rcs.kuka_robot import KukaRobot
from rcs.wcs_task_adapter import WCSTaskAdapter, WCSTask, WCSTaskType, WCSTaskPriority
from rcs.wcs_priority_scheduler import WCSPriorityScheduler, WCSPriorityLevel, TaskScheduleInfo
from rcs.rack_state_manager import RackStateManager, RackAnalysisResult, CarrierInfo


# KUKA 工作類型常數 (參考 init_work.py)
class KukaWorkType:
    """KUKA 工作類型常數定義"""
    KUKA_MOVE = 210001          # kuka-移動
    KUKA_RACK_MOVE = 220001     # kuka-移動貨架
    KUKA_WORKFLOW = 230001      # kuka-workflow

    @classmethod
    def get_work_name(cls, work_id: int) -> str:
        """取得工作類型名稱"""
        work_names = {
            cls.KUKA_MOVE: "KUKA移動",
            cls.KUKA_RACK_MOVE: "KUKA移動貨架",
            cls.KUKA_WORKFLOW: "KUKA工作流程"
        }
        return work_names.get(work_id, f"未知工作類型({work_id})")

    @classmethod
    def is_kuka_work(cls, work_id: int) -> bool:
        """檢查是否為 KUKA 工作類型"""
        return work_id in [cls.KUKA_MOVE, cls.KUKA_RACK_MOVE, cls.KUKA_WORKFLOW]

    @classmethod
    def get_work_type_by_function(cls, function: str) -> int:
        """根據 function 參數取得對應的工作類型"""
        function_map = {
            "move": cls.KUKA_MOVE,
            "rack_move": cls.KUKA_RACK_MOVE,
            "workflow": cls.KUKA_WORKFLOW
        }
        return function_map.get(function, cls.KUKA_WORKFLOW)  # 預設為 workflow


class KukaManager:
    """KUKA 統一管理器類別"""

    def __init__(self, rcs_core_node):
        """
        初始化 KUKA 管理器

        Args:
            rcs_core_node: RCS Core 節點實例
        """
        self.rcs_core = rcs_core_node
        self.logger = rcs_core_node.get_logger()
        self.db_pool: ConnectionPoolManager = rcs_core_node.db_pool

        # 創建並管理 KukaFleetAdapter
        self.kuka_fleet = KukaFleetAdapter(rcs_core_node)

        # 初始化 KUKA 相關管理器
        self.kuka_robot = KukaRobot(rcs_core_node)
        self.kuka_container = KukaContainer(rcs_core_node)

        # 設定回調函數
        self.kuka_fleet.on_robot_query_complete = self.kuka_robot.on_robot_update
        self.kuka_fleet.on_container_query_complete = self.kuka_container.on_container_update

        # 載入 KUKA AGV 資料
        self.kuka_agvs = {}
        self._load_kuka_agvs()

        # 初始化 WCS 任務適配器
        self.wcs_adapter = WCSTaskAdapter(self.logger)
        
        # 初始化 WCS 優先度調度器
        self.priority_scheduler = WCSPriorityScheduler(self.logger)
        
        # 初始化 Rack 狀態管理器
        self.rack_state_manager = RackStateManager(self.logger)
        
        # API 重試配置
        self.api_retry_config = {
            'max_attempts': 3,
            'base_delay': 1.0,  # 基礎延遲(秒)
            'backoff_factor': 2.0,  # 指數退避因子
            'max_delay': 30.0,  # 最大延遲(秒)
            'timeout': 60.0  # API 超時(秒)
        }
        
        # API 統計
        self.api_stats = {
            'total_calls': 0,
            'successful_calls': 0,
            'failed_calls': 0,
            'retry_calls': 0,
            'average_response_time': 0.0
        }

        # 啟動監控
        self.kuka_fleet.start_monitoring()

        # 測試：查詢閒置的 KUKA400i AGV
        idle_kuka400i_agvs = self.kuka_fleet.select_agv(
            KukaFleetAdapter.STATUS_IDLE)
        idle_kuka400i_agv_ids = [agv["id"] for agv in idle_kuka400i_agvs]
        self.logger.info(f"閒置的 KUKA400i AGV: {idle_kuka400i_agv_ids}")

    def _load_kuka_agvs(self):
        """從 AGV 資料表取得所有 enable 且 model 為 KUKA400i 的 AGV，儲存於 self.kuka_agvs。"""
        if not self.db_pool:
            self.logger.error("資料庫連線池不可用，無法載入 KUKA AGV。")
            return
        try:
            with self.db_pool.get_session() as session:
                from db_proxy.models import AGV
                from sqlmodel import select
                agvs = session.exec(
                    select(AGV).where(AGV.enable == 1, AGV.model == "KUKA400i")
                ).all()
                self.kuka_agvs = {agv.id: agv for agv in agvs}
                self.logger.info(
                    f"已載入 {len(self.kuka_agvs)} 台 KUKA400i AGV。{self.kuka_agvs}")
        except Exception as e:
            self.logger.error(f"載入 KUKA AGV 時發生錯誤: {e}")

    def dispatch(self):
        """KUKA400i AGV 智能任務派發 - 支援 WCS 四級優先度"""
        try:
            # 1. 查詢閒置的 KUKA400i AGV
            idle_kuka400i_agvs = self.kuka_fleet.select_agv(KukaFleetAdapter.STATUS_IDLE)
            idle_kuka400i_agv_ids = [int(agv["id"]) for agv in idle_kuka400i_agvs]

            with self.db_pool.get_session() as session:
                from db_proxy.models import AGV, Task
                from sqlmodel import select

                # 2. 檢查資料庫中的 AGV 狀態，確保只選擇真正閒置的 AGV
                available_agvs = session.exec(
                    select(AGV).where(
                        AGV.enable == 1,
                        AGV.model == "KUKA400i",
                        AGV.id.in_(idle_kuka400i_agv_ids),
                        AGV.status_id == 3  # 確保 AGV 在資料庫中也是閒置狀態
                    )
                ).all()
                available_agv_ids = [agv.id for agv in available_agvs]

                if not available_agv_ids:
                    # self.logger.debug("目前沒有可用的 KUKA400i AGV")
                    return

                # 3. 查詢 KUKA 任務 - 支援 WCS 和傳統任務格式
                kuka_tasks = self._get_pending_kuka_tasks(session)
                
                if not kuka_tasks:
                    # self.logger.debug("目前沒有 KUKA 任務需要處理")
                    return

                # 4. 使用 WCS 優先度調度器進行智能排程
                system_context = self._build_system_context(session, len(available_agv_ids))
                schedule_infos = self.priority_scheduler.schedule_tasks(kuka_tasks, system_context)
                
                if not schedule_infos:
                    self.logger.warning("WCS 優先度調度失敗，使用基本排序")
                    sorted_tasks = sorted(kuka_tasks, key=lambda t: getattr(t, 'priority', 40), reverse=True)
                else:
                    # 重新排序任務基於調度資訊
                    task_priority_map = {info.task_id: info.calculated_priority for info in schedule_infos}
                    sorted_tasks = sorted(
                        kuka_tasks, 
                        key=lambda t: task_priority_map.get(t.id, getattr(t, 'priority', 40)), 
                        reverse=True
                    )
                
                self.logger.info(
                    f"找到 {len(sorted_tasks)} 個 KUKA 任務待處理，"
                    f"可用 AGV: {len(available_agv_ids)} 台"
                )

                # 5. 派發任務給可用 AGV
                dispatch_count = 0
                for i, task in enumerate(sorted_tasks):
                    if i >= len(available_agv_ids):
                        break  # 沒有更多可用 AGV
                    
                    agv_id = available_agv_ids[i]
                    success = self._dispatch_task_to_agv(session, task, agv_id)
                    
                    if success:
                        dispatch_count += 1

                session.commit()
                
                if dispatch_count > 0:
                    self.logger.info(f"成功派發 {dispatch_count} 個 KUKA 任務")

        except Exception as e:
            self.logger.error(f"KUKA 任務派發時發生錯誤: {e}", exc_info=True)

    def _build_system_context(self, session, available_agv_count: int) -> dict:
        """
        構建系統上下文資訊供優先度調度器使用
        
        Args:
            session: 資料庫會話
            available_agv_count: 可用 AGV 數量
            
        Returns:
            dict: 系統上下文資訊
        """
        try:
            from db_proxy.models import Task, AGV
            from sqlmodel import select, func

            # 統計待處理任務數量
            pending_tasks_count = session.exec(
                select(func.count(Task.id)).where(
                    Task.status_id == 1,  # 待執行
                    Task.parameters["model"].as_string() == "KUKA400i"
                )
            ).first() or 0

            # 統計總 KUKA AGV 數量
            total_agvs_count = session.exec(
                select(func.count(AGV.id)).where(
                    AGV.enable == 1,
                    AGV.model == "KUKA400i"
                )
            ).first() or 1

            # 統計執行中任務數量
            executing_tasks_count = session.exec(
                select(func.count(Task.id)).where(
                    Task.status_id.in_([2, 3]),  # 執行中狀態
                    Task.parameters["model"].as_string() == "KUKA400i"
                )
            ).first() or 0

            system_context = {
                'available_agvs': available_agv_count,
                'total_agvs': total_agvs_count,
                'pending_tasks_count': pending_tasks_count,
                'executing_tasks_count': executing_tasks_count,
                'system_load_ratio': executing_tasks_count / max(total_agvs_count, 1),
                'agv_utilization_ratio': (total_agvs_count - available_agv_count) / max(total_agvs_count, 1),
                'timestamp': datetime.now(timezone.utc)
            }

            return system_context

        except Exception as e:
            self.logger.error(f"構建系統上下文時發生錯誤: {e}")
            return {
                'available_agvs': available_agv_count,
                'total_agvs': 1,
                'pending_tasks_count': 0,
                'executing_tasks_count': 0,
                'system_load_ratio': 0.0,
                'agv_utilization_ratio': 0.0,
                'timestamp': datetime.now(timezone.utc)
            }

    def _get_pending_kuka_tasks(self, session):
        """
        獲取待處理的 KUKA 任務 - 支援 WCS 和傳統任務格式
        
        Args:
            session: 資料庫會話
            
        Returns:
            List: 待處理的 KUKA 任務列表
        """
        try:
            from db_proxy.models import Task
            from sqlmodel import select

            # 查詢所有待執行的 KUKA400i 任務
            all_tasks = session.exec(
                select(Task).where(
                    Task.status_id == 1,  # 待執行
                    Task.mission_code == None,  # 尚未指定任務代碼
                    Task.parameters["model"].as_string() == "KUKA400i"
                )
            ).all()

            kuka_tasks = []
            
            for task in all_tasks:
                # 檢查是否為 KUKA 工作類型 (傳統任務)
                if KukaWorkType.is_kuka_work(task.work_id):
                    kuka_tasks.append(task)
                    continue
                
                # 檢查是否為 WCS 任務格式
                if self._is_wcs_format_task(task):
                    kuka_tasks.append(task)
                    continue
                    
            return kuka_tasks
            
        except Exception as e:
            self.logger.error(f"獲取 KUKA 任務時發生錯誤: {e}")
            return []

    def _is_wcs_format_task(self, task) -> bool:
        """
        檢查任務是否為 WCS 格式
        
        Args:
            task: 任務物件
            
        Returns:
            bool: 是否為 WCS 格式
        """
        try:
            params = task.parameters or {}
            
            # 檢查 WCS 任務標識
            if 'wcs_task_type' in params or 'wcs_task_id' in params:
                return True
                
            # 檢查 function 為 rack_move (WCS 標準格式)
            if params.get('function') == 'rack_move':
                return True
                
            return False
            
        except Exception as e:
            self.logger.error(f"檢查 WCS 任務格式時發生錯誤: {e}")
            return False

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
            from db_proxy.models import AGV
            from sqlmodel import select
            from datetime import datetime, timezone

            # 獲取 AGV 物件
            agv = session.exec(
                select(AGV).where(AGV.id == agv_id)
            ).first()
            
            if not agv:
                self.logger.error(f"找不到 AGV {agv_id}")
                return False

            # 生成任務代碼
            mission_code = str(uuid.uuid4())
            
            # 獲取任務描述
            task_description = self._get_task_description(task)
            
            self.logger.info(
                f"派發任務 {task.id} ({task_description}) 給 AGV {agv_id}, "
                f"priority: {task.priority}, mission_code: {mission_code}"
            )

            # 執行任務
            execution_result = self._execute_kuka_task(task, agv_id, mission_code)

            # 處理執行結果
            if execution_result["success"]:
                # 更新任務狀態
                task.agv_id = agv_id
                task.mission_code = mission_code
                task.status_id = 2  # 更新為執行中
                task.updated_at = datetime.now(timezone.utc)
                
                # 更新任務參數
                if not task.parameters:
                    task.parameters = {}
                task.parameters["agvId"] = agv_id
                
                # 更新 AGV 狀態
                agv.status_id = 4  # 任務中
                
                session.add(task)
                session.add(agv)
                
                self.logger.info(
                    f"✅ 任務 {task.id} ({task_description}) 成功派發給 AGV {agv_id}"
                )
                return True
            else:
                self.logger.error(
                    f"❌ 任務 {task.id} ({task_description}) 派發失敗: {execution_result.get('error', '未知錯誤')}"
                )
                return False

        except Exception as e:
            self.logger.error(f"派發任務給 AGV {agv_id} 時發生錯誤: {e}")
            return False

    def _get_task_description(self, task) -> str:
        """
        獲取任務描述
        
        Args:
            task: 任務物件
            
        Returns:
            str: 任務描述
        """
        try:
            params = task.parameters or {}
            
            # WCS 任務格式
            if 'wcs_task_type' in params:
                wcs_type = params['wcs_task_type']
                priority = params.get('wcs_priority', task.priority)
                return f"WCS-{wcs_type} (Priority: {priority})"
            
            # 傳統 KUKA 任務格式
            if KukaWorkType.is_kuka_work(task.work_id):
                work_name = KukaWorkType.get_work_name(task.work_id)
                return f"{work_name} (Work ID: {task.work_id})"
            
            # 基於 function 的描述
            function = params.get('function', 'unknown')
            return f"Function: {function}"
            
        except Exception as e:
            self.logger.error(f"獲取任務描述時發生錯誤: {e}")
            return f"Task ID: {task.id}"

    def _execute_kuka_task(self, task, agv_id: int, mission_code: str):
        """
        執行 KUKA 任務 - 支援 WCS 和傳統任務格式

        Args:
            task: 任務物件
            agv_id: AGV ID
            mission_code: 任務代碼

        Returns:
            dict: 執行結果
        """
        try:
            # 檢查是否為 WCS 格式任務
            if self._is_wcs_format_task(task):
                return self._execute_wcs_format_task(task, agv_id, mission_code)
            
            # 傳統 KUKA 任務格式處理
            work_id = task.work_id

            if work_id == KukaWorkType.KUKA_MOVE:
                return self._execute_move_task(task, agv_id, mission_code)
            elif work_id == KukaWorkType.KUKA_RACK_MOVE:
                return self._execute_rack_move_task(task, agv_id, mission_code)
            elif work_id == KukaWorkType.KUKA_WORKFLOW:
                return self._execute_workflow_task(task, agv_id, mission_code)
            else:
                # 未知的工作類型，嘗試作為 workflow 處理
                self.logger.warning(f"未知的 KUKA 工作類型 {work_id}，嘗試作為 workflow 處理")
                return self._execute_workflow_task(task, agv_id, mission_code)
                
        except Exception as e:
            self.logger.error(f"執行 KUKA 任務時發生錯誤: {e}")
            return {"success": False, "error": str(e)}

    def _execute_wcs_format_task(self, task, agv_id: int, mission_code: str):
        """
        執行 WCS 格式任務
        
        Args:
            task: 任務物件
            agv_id: AGV ID
            mission_code: 任務代碼
            
        Returns:
            dict: 執行結果
        """
        try:
            params = task.parameters or {}
            
            # 提取 WCS 任務參數
            function = params.get('function', 'rack_move')
            nodes = params.get('nodes', [])
            wcs_task_type = params.get('wcs_task_type', 'unknown')
            task_subtype = params.get('task_subtype', 'normal')
            
            # 驗證必要參數
            if not nodes or len(nodes) < 2:
                return {"success": False, "error": "WCS 任務缺少有效的節點路徑"}
            
            # 記錄 WCS 任務執行資訊
            self.logger.info(
                f"執行 WCS 任務: type={wcs_task_type}, subtype={task_subtype}, "
                f"function={function}, nodes={nodes}, rack_id={params.get('rack_id')}"
            )
            
            # 根據 function 選擇執行方式
            if function == 'rack_move':
                return self._execute_wcs_rack_move(task, agv_id, mission_code, nodes)
            elif function == 'move':
                return self._execute_wcs_move(task, agv_id, mission_code, nodes)
            elif function == 'workflow':
                return self._execute_wcs_workflow(task, agv_id, mission_code)
            else:
                # 預設使用 rack_move
                self.logger.warning(f"未知的 WCS function: {function}，使用預設 rack_move")
                return self._execute_wcs_rack_move(task, agv_id, mission_code, nodes)
                
        except Exception as e:
            self.logger.error(f"執行 WCS 格式任務時發生錯誤: {e}")
            return {"success": False, "error": str(e)}

    def _execute_wcs_rack_move(self, task, agv_id: int, mission_code: str, nodes: list):
        """執行 WCS Rack 搬運任務 - 整合 Rack 狀態管理"""
        try:
            params = task.parameters or {}
            rack_id = params.get('rack_id')
            task_subtype = params.get('task_subtype', 'normal')
            
            # 獲取 Rack 狀態資訊
            rack_state = None
            if rack_id:
                rack_state = self.rack_state_manager.get_rack_state(rack_id)
                if rack_state:
                    self.logger.info(
                        f"Rack {rack_id} 狀態: 載貨={rack_state.total_carriers}, "
                        f"方向={rack_state.current_direction}, 需旋轉={rack_state.needs_rotation}, "
                        f"有NG={rack_state.has_ng}"
                    )
                else:
                    self.logger.warning(f"找不到 Rack {rack_id} 的狀態資訊")
            
            # 特殊處理旋轉任務
            if task_subtype == 'rotation':
                return self._execute_rotation_task_with_validation(
                    task, agv_id, mission_code, nodes, rack_state
                )
            
            # 一般搬運任務
            return self._execute_normal_rack_move_task(
                task, agv_id, mission_code, nodes, rack_state
            )
            
        except Exception as e:
            self.logger.error(f"執行 WCS Rack 搬運時發生錯誤: {e}")
            return {"success": False, "error": str(e)}

    def _execute_rotation_task_with_validation(self, task, agv_id: int, mission_code: str, 
                                             nodes: list, rack_state: Optional[RackAnalysisResult]):
        """執行旋轉任務並進行驗證 - 增強錯誤處理和重試機制"""
        try:
            params = task.parameters or {}
            rack_id = params.get('rack_id')

            self.logger.info(f"執行 WCS 旋轉任務 - Rack {rack_id}, nodes: {nodes}")
            
            # 驗證旋轉路徑格式
            path_validation = self._validate_rotation_path(nodes)
            if not path_validation['valid']:
                self.logger.error(f"旋轉路徑驗證失敗: {path_validation['error']}")
                return {"success": False, "error": path_validation['error']}
            
            # 驗證 Rack 狀態是否適合旋轉
            if rack_state:
                feasibility_result = self._validate_rotation_feasibility(rack_state)
                if not feasibility_result['feasible']:
                    self.logger.warning(f"Rack {rack_id} 旋轉可行性警告: {feasibility_result['reason']}")
                    # 根據信心度決定是否繼續執行
                    if feasibility_result.get('confidence') == 'low' and feasibility_result.get('block_execution', False):
                        return {"success": False, "error": f"旋轉任務被阻止: {feasibility_result['reason']}"}
            
            self.logger.info(
                f"旋轉任務路徑驗證通過: 起點({nodes[0]}) -> 移出點({nodes[1]}) -> 回到起點({nodes[2]})"
            )
            
            # 執行帶重試的 KUKA rack_move (旋轉)
            result = self._execute_kuka_api_with_retry(
                'rack_move', 
                [nodes, agv_id, mission_code],
                task_type='rotation',
                rack_id=rack_id
            )
            
            if result.get("success"):
                self.logger.info(f"✅ WCS 旋轉任務執行成功 - Rack {rack_id}")
                
                # 更新 Rack 狀態 (如果有狀態資訊)
                if rack_state:
                    self._update_rack_state_after_rotation(rack_state)
                    
            else:
                self.logger.error(f"❌ WCS 旋轉任務執行失敗 - {result.get('error', '未知錯誤')}")
            
            return result
            
        except Exception as e:
            self.logger.error(f"執行旋轉任務時發生錯誤: {e}")
            return {"success": False, "error": str(e)}

    def _execute_normal_rack_move_task(self, task, agv_id: int, mission_code: str, 
                                     nodes: list, rack_state: Optional[RackAnalysisResult]):
        """執行一般 Rack 搬運任務 - 增強錯誤處理和重試機制"""
        try:
            params = task.parameters or {}
            rack_id = params.get('rack_id')
            wcs_task_type = params.get('wcs_task_type', 'rack_move')

            # 驗證路徑格式
            path_validation = self._validate_move_path(nodes)
            if not path_validation['valid']:
                self.logger.error(f"搬運路徑驗證失敗: {path_validation['error']}")
                return {"success": False, "error": path_validation['error']}

            # 驗證搬運可行性
            if rack_state:
                validation_result = self._validate_move_feasibility(rack_state, params)
                if not validation_result['feasible']:
                    self.logger.warning(f"Rack {rack_id} 搬運可行性警告: {validation_result['reason']}")
                    # 檢查是否應該阻止執行
                    if validation_result.get('confidence') == 'low' and validation_result.get('block_execution', False):
                        return {"success": False, "error": f"搬運任務被阻止: {validation_result['reason']}"}
            
            self.logger.info(f"執行 WCS {wcs_task_type} 任務 - Rack {rack_id}, nodes: {nodes}")
            
            # 執行帶重試的 KUKA rack_move
            result = self._execute_kuka_api_with_retry(
                'rack_move', 
                [nodes, agv_id, mission_code],
                task_type=wcs_task_type,
                rack_id=rack_id
            )
            
            if result.get("success"):
                self.logger.info(
                    f"✅ WCS {wcs_task_type} 任務執行成功 - "
                    f"Rack {rack_id}, 載貨: {rack_state.total_carriers if rack_state else 'unknown'}"
                )
                
                # 更新 Rack 位置狀態 (如果有狀態資訊)
                if rack_state:
                    self._update_rack_state_after_move(rack_state, params)
                    
            else:
                self.logger.error(
                    f"❌ WCS {wcs_task_type} 任務執行失敗 - {result.get('error', '未知錯誤')}"
                )
            
            return result
            
        except Exception as e:
            self.logger.error(f"執行一般搬運任務時發生錯誤: {e}")
            return {"success": False, "error": str(e)}

    def _validate_rotation_feasibility(self, rack_state: RackAnalysisResult) -> Dict[str, Any]:
        """驗證旋轉任務可行性"""
        try:
            # 檢查是否真的需要旋轉
            if not rack_state.needs_rotation:
                return {
                    'feasible': True,
                    'reason': 'Rack 可能不需要旋轉，但仍可執行',
                    'confidence': 'medium'
                }
            
            # 檢查 NG 狀態
            if rack_state.has_ng:
                return {
                    'feasible': True,
                    'reason': f'Rack 含有 {rack_state.ng_count} 個 NG Carrier，旋轉後需特別處理',
                    'confidence': 'low'
                }
            
            # 檢查載貨狀態
            if rack_state.is_empty:
                return {
                    'feasible': True,
                    'reason': 'Rack 為空，旋轉操作安全',
                    'confidence': 'high'
                }
            
            return {
                'feasible': True,
                'reason': f'Rack 載貨 {rack_state.total_carriers} 個，可安全旋轉',
                'confidence': 'high'
            }
            
        except Exception as e:
            self.logger.error(f"驗證旋轉可行性時發生錯誤: {e}")
            return {
                'feasible': True,
                'reason': '驗證過程發生錯誤，預設允許執行',
                'confidence': 'low'
            }

    def _validate_move_feasibility(self, rack_state: RackAnalysisResult, task_params: Dict[str, Any]) -> Dict[str, Any]:
        """驗證搬運任務可行性"""
        try:
            target_room = task_params.get('target_room')
            
            # 檢查製程相符性
            if not rack_state.process_compatible and target_room:
                return {
                    'feasible': True,
                    'reason': f'製程不相符: {rack_state.process_error_message}，可能需要調整',
                    'confidence': 'low'
                }
            
            # 檢查 NG 狀態
            if rack_state.has_ng:
                return {
                    'feasible': True,
                    'reason': f'Rack 含有 {rack_state.ng_count} 個 NG Carrier，建議送往 NG 區處理',
                    'confidence': 'medium'
                }
            
            return {
                'feasible': True,
                'reason': 'Rack 狀態正常，可安全搬運',
                'confidence': 'high'
            }
            
        except Exception as e:
            self.logger.error(f"驗證搬運可行性時發生錯誤: {e}")
            return {
                'feasible': True,
                'reason': '驗證過程發生錯誤，預設允許執行',
                'confidence': 'low'
            }

    def _update_rack_state_after_rotation(self, rack_state: RackAnalysisResult):
        """旋轉後更新 Rack 狀態"""
        try:
            # 更新方向
            if rack_state.current_direction == 0:  # A面朝前 -> B面朝前
                rack_state.current_direction = 180
            else:  # B面朝前 -> A面朝前
                rack_state.current_direction = 0
            
            # 重新評估是否需要旋轉
            rack_state.needs_rotation = False  # 剛旋轉完，暫時不需要
            
            # 更新時間戳記
            rack_state.analysis_timestamp = datetime.now(timezone.utc)
            
            self.logger.info(
                f"Rack {rack_state.rack_id} 旋轉後狀態已更新: "
                f"新方向={rack_state.current_direction}"
            )
            
        except Exception as e:
            self.logger.error(f"更新旋轉後狀態時發生錯誤: {e}")

    def _update_rack_state_after_move(self, rack_state: RackAnalysisResult, task_params: Dict[str, Any]):
        """搬運後更新 Rack 狀態"""
        try:
            # 更新位置
            target_location = task_params.get('target_location')
            if target_location:
                rack_state.location_id = target_location
            
            # 更新房間
            target_room = task_params.get('target_room')
            if target_room:
                rack_state.room_id = target_room
            
            # 更新時間戳記
            rack_state.analysis_timestamp = datetime.now(timezone.utc)
            
            self.logger.info(
                f"Rack {rack_state.rack_id} 搬運後狀態已更新: "
                f"位置={rack_state.location_id}, 房間={rack_state.room_id}"
            )
            
        except Exception as e:
            self.logger.error(f"更新搬運後狀態時發生錯誤: {e}")

    def _execute_wcs_move(self, task, agv_id: int, mission_code: str, nodes: list):
        """執行 WCS 移動任務"""
        try:
            self.logger.info(f"執行 WCS 移動任務 - nodes: {nodes}")
            return self.kuka_fleet.move(nodes, agv_id, mission_code)
            
        except Exception as e:
            self.logger.error(f"執行 WCS 移動任務時發生錯誤: {e}")
            return {"success": False, "error": str(e)}

    def _execute_wcs_workflow(self, task, agv_id: int, mission_code: str):
        """執行 WCS 工作流程任務"""
        try:
            params = task.parameters or {}
            template_code = params.get('templateCode')
            
            if not template_code:
                return {"success": False, "error": "WCS 工作流程任務缺少 templateCode 參數"}
            
            self.logger.info(f"執行 WCS 工作流程任務 - templateCode: {template_code}")
            return self.kuka_fleet.workflow(template_code, agv_id, mission_code)
            
        except Exception as e:
            self.logger.error(f"執行 WCS 工作流程任務時發生錯誤: {e}")
            return {"success": False, "error": str(e)}

    def _execute_move_task(self, task, agv_id: int, mission_code: str):
        """執行 KUKA 移動任務"""
        nodes = task.parameters.get('nodes')
        if not nodes:
            self.logger.warn(f"KUKA 移動任務 {task.id} 缺少參數 'nodes'")
            return {"success": False, "error": "缺少參數 'nodes'"}

        self.logger.info(f"執行 KUKA 移動任務 - nodes: {nodes}")
        return self.kuka_fleet.move(nodes, agv_id, mission_code)

    def _execute_rack_move_task(self, task, agv_id: int, mission_code: str):
        """執行 KUKA 移動貨架任務"""
        nodes = task.parameters.get('nodes')
        if not nodes:
            self.logger.warn(f"KUKA 移動貨架任務 {task.id} 缺少參數 'nodes'")
            return {"success": False, "error": "缺少參數 'nodes'"}

        self.logger.info(f"執行 KUKA 移動貨架任務 - nodes: {nodes}")
        return self.kuka_fleet.rack_move(nodes, agv_id, mission_code)

    def _execute_workflow_task(self, task, agv_id: int, mission_code: str):
        """執行 KUKA workflow 任務"""
        template_code = task.parameters.get('templateCode')
        if not template_code:
            self.logger.warn(f"KUKA workflow 任務 {task.id} 缺少參數 'templateCode'")
            return {"success": False, "error": "缺少參數 'templateCode'"}

        self.logger.info(f"執行 KUKA workflow 任務 - templateCode: {template_code}")
        return self.kuka_fleet.workflow(template_code, agv_id, mission_code)

    def get_pending_wcs_tasks_count(self) -> dict:
        """
        獲取待處理的 WCS 任務統計
        
        Returns:
            dict: WCS 任務統計資訊
        """
        try:
            with self.db_pool.get_session() as session:
                from db_proxy.models import Task
                from sqlmodel import select

                # 查詢所有待執行的 KUKA400i 任務
                all_tasks = session.exec(
                    select(Task).where(
                        Task.status_id == 1,  # 待執行
                        Task.mission_code == None,  # 尚未指定任務代碼
                        Task.parameters["model"].as_string() == "KUKA400i"
                    )
                ).all()

                stats = {
                    'total_tasks': len(all_tasks),
                    'wcs_tasks': 0,
                    'traditional_tasks': 0,
                    'priority_breakdown': {
                        'high': 0,    # Priority >= 80
                        'medium': 0,  # Priority 60-79
                        'low': 0      # Priority < 60
                    }
                }

                for task in all_tasks:
                    if self._is_wcs_format_task(task):
                        stats['wcs_tasks'] += 1
                    elif KukaWorkType.is_kuka_work(task.work_id):
                        stats['traditional_tasks'] += 1
                    
                    # 統計優先度分布
                    priority = getattr(task, 'priority', 40)
                    if priority >= 80:
                        stats['priority_breakdown']['high'] += 1
                    elif priority >= 60:
                        stats['priority_breakdown']['medium'] += 1
                    else:
                        stats['priority_breakdown']['low'] += 1

                return stats

        except Exception as e:
            self.logger.error(f"獲取 WCS 任務統計時發生錯誤: {e}")
            return {'error': str(e)}

    def get_wcs_adapter_statistics(self) -> dict:
        """
        獲取 WCS 適配器統計資訊
        
        Returns:
            dict: WCS 適配器統計
        """
        try:
            return self.wcs_adapter.get_conversion_statistics()
        except Exception as e:
            self.logger.error(f"獲取 WCS 適配器統計時發生錯誤: {e}")
            return {'error': str(e)}

    def create_wcs_task_from_decision(self, wcs_task: WCSTask) -> bool:
        """
        從 WCS 決策創建 KUKA 任務
        
        Args:
            wcs_task: WCS 任務物件
            
        Returns:
            bool: 是否成功創建任務
        """
        try:
            # 驗證 WCS 任務
            is_valid, error_msg = self.wcs_adapter.validate_wcs_task(wcs_task)
            if not is_valid:
                self.logger.error(f"WCS 任務驗證失敗: {error_msg}")
                return False

            # 轉換為 KUKA 任務
            kuka_task = self.wcs_adapter.convert_wcs_task_to_kuka(wcs_task)
            if not kuka_task:
                self.logger.error("WCS 任務轉換失敗")
                return False

            # 寫入資料庫
            with self.db_pool.get_session() as session:
                from db_proxy.models import Task
                from datetime import datetime, timezone

                db_task = Task(
                    **kuka_task.to_db_format(),
                    name=f"WCS-{wcs_task.task_type.value}",
                    description=f"WCS 任務: {wcs_task.task_type.value} - Rack {wcs_task.rack_id}",
                    created_at=datetime.now(timezone.utc)
                )

                session.add(db_task)
                session.commit()

                self.logger.info(
                    f"✅ 成功創建 WCS 任務: {wcs_task.task_id} -> DB Task ID: {db_task.id}"
                )
                return True

        except Exception as e:
            self.logger.error(f"創建 WCS 任務時發生錯誤: {e}")
            return False

    def batch_create_wcs_tasks(self, wcs_tasks: list) -> dict:
        """
        批量創建 WCS 任務
        
        Args:
            wcs_tasks: WCS 任務列表
            
        Returns:
            dict: 創建結果統計
        """
        try:
            results = {
                'total': len(wcs_tasks),
                'success': 0,
                'failed': 0,
                'errors': []
            }

            for wcs_task in wcs_tasks:
                if self.create_wcs_task_from_decision(wcs_task):
                    results['success'] += 1
                else:
                    results['failed'] += 1
                    results['errors'].append(f"任務 {wcs_task.task_id} 創建失敗")

            self.logger.info(
                f"批量創建 WCS 任務完成: "
                f"總計 {results['total']}, 成功 {results['success']}, 失敗 {results['failed']}"
            )

            return results

        except Exception as e:
            self.logger.error(f"批量創建 WCS 任務時發生錯誤: {e}")
            return {'error': str(e)}

    def get_priority_scheduler_statistics(self) -> dict:
        """
        獲取優先度調度器統計資訊
        
        Returns:
            dict: 調度器統計
        """
        try:
            return self.priority_scheduler.get_schedule_statistics()
        except Exception as e:
            self.logger.error(f"獲取調度器統計時發生錯誤: {e}")
            return {'error': str(e)}

    def explain_task_priority(self, task_id: int) -> dict:
        """
        解釋特定任務的優先度計算
        
        Args:
            task_id: 任務 ID
            
        Returns:
            dict: 優先度解釋
        """
        try:
            with self.db_pool.get_session() as session:
                from db_proxy.models import Task
                from sqlmodel import select

                task = session.exec(
                    select(Task).where(Task.id == task_id)
                ).first()

                if not task:
                    return {'error': f'找不到任務 ID: {task_id}'}

                system_context = self._build_system_context(session, 1)
                return self.priority_scheduler.explain_task_priority(task, system_context)

        except Exception as e:
            self.logger.error(f"解釋任務優先度時發生錯誤: {e}")
            return {'error': str(e)}

    def get_comprehensive_system_status(self) -> dict:
        """
        獲取綜合系統狀態報告
        
        Returns:
            dict: 綜合系統狀態
        """
        try:
            with self.db_pool.get_session() as session:
                # 獲取各種統計資訊
                wcs_task_stats = self.get_pending_wcs_tasks_count()
                adapter_stats = self.get_wcs_adapter_statistics()
                scheduler_stats = self.get_priority_scheduler_statistics()
                system_context = self._build_system_context(session, 0)  # 不考慮可用AGV，只獲取系統概況

                comprehensive_status = {
                    'timestamp': datetime.now(timezone.utc).isoformat(),
                    'wcs_integration': {
                        'task_statistics': wcs_task_stats,
                        'adapter_statistics': adapter_stats,
                        'scheduler_statistics': scheduler_stats
                    },
                    'system_performance': {
                        'total_agvs': system_context.get('total_agvs', 0),
                        'agv_utilization_ratio': system_context.get('agv_utilization_ratio', 0),
                        'system_load_ratio': system_context.get('system_load_ratio', 0),
                        'pending_tasks': system_context.get('pending_tasks_count', 0),
                        'executing_tasks': system_context.get('executing_tasks_count', 0)
                    },
                    'health_indicators': {
                        'kuka_fleet_connected': hasattr(self, 'kuka_fleet') and self.kuka_fleet is not None,
                        'database_connected': self.db_pool is not None,
                        'wcs_adapter_ready': hasattr(self, 'wcs_adapter') and self.wcs_adapter is not None,
                        'priority_scheduler_ready': hasattr(self, 'priority_scheduler') and self.priority_scheduler is not None
                    }
                }

                return comprehensive_status

        except Exception as e:
            self.logger.error(f"獲取綜合系統狀態時發生錯誤: {e}")
            return {'error': str(e)}

    def reset_all_statistics(self):
        """重置所有統計資訊"""
        try:
            if hasattr(self, 'wcs_adapter'):
                self.wcs_adapter.reset_statistics()
            
            if hasattr(self, 'priority_scheduler'):
                self.priority_scheduler.reset_statistics()
            
            if hasattr(self, 'rack_state_manager'):
                self.rack_state_manager.clear_all_states()
            
            # 重置 API 統計
            if hasattr(self, 'api_stats'):
                self.api_stats = {
                    'total_calls': 0,
                    'successful_calls': 0,
                    'failed_calls': 0,
                    'retry_calls': 0,
                    'average_response_time': 0.0
                }
            
            self.logger.info("所有統計資訊已重置 (包含 API 統計)")
            
        except Exception as e:
            self.logger.error(f"重置統計資訊時發生錯誤: {e}")

    def update_rack_analysis_result(self, rack_analysis: RackAnalysisResult) -> bool:
        """
        更新來自 WCS 的 Rack 分析結果
        
        Args:
            rack_analysis: WCS Rack 分析結果
            
        Returns:
            bool: 是否更新成功
        """
        try:
            return self.rack_state_manager.update_rack_state(rack_analysis)
        except Exception as e:
            self.logger.error(f"更新 Rack 分析結果時發生錯誤: {e}")
            return False

    def get_rack_state_statistics(self) -> dict:
        """
        獲取 Rack 狀態統計資訊
        
        Returns:
            dict: Rack 狀態統計
        """
        try:
            return self.rack_state_manager.get_state_statistics()
        except Exception as e:
            self.logger.error(f"獲取 Rack 狀態統計時發生錯誤: {e}")
            return {'error': str(e)}

    def get_racks_needing_attention(self) -> dict:
        """
        獲取需要關注的 Rack 列表
        
        Returns:
            dict: 各類需要關注的 Rack 列表
        """
        try:
            return {
                'rotation_needed': [
                    {'rack_id': r.rack_id, 'location_id': r.location_id, 'current_direction': r.current_direction}
                    for r in self.rack_state_manager.get_racks_needing_rotation()
                ],
                'with_ng': [
                    {'rack_id': r.rack_id, 'location_id': r.location_id, 'ng_count': r.ng_count}
                    for r in self.rack_state_manager.get_racks_with_ng()
                ],
                'empty_racks': [
                    {'rack_id': r.rack_id, 'location_id': r.location_id}
                    for r in self.rack_state_manager.get_empty_racks()
                ],
                'full_racks': [
                    {'rack_id': r.rack_id, 'location_id': r.location_id, 'total_carriers': r.total_carriers}
                    for r in self.rack_state_manager.get_full_racks()
                ]
            }
        except Exception as e:
            self.logger.error(f"獲取需要關注的 Rack 時發生錯誤: {e}")
            return {'error': str(e)}

    def find_suitable_rack_for_wcs_task(self, wcs_task: WCSTask) -> Optional[RackAnalysisResult]:
        """
        為 WCS 任務尋找合適的 Rack
        
        Args:
            wcs_task: WCS 任務物件
            
        Returns:
            RackAnalysisResult: 合適的 Rack，沒找到時返回 None
        """
        try:
            # 基於 WCS 任務類型建立需求
            task_requirements = {
                'room_id': wcs_task.room_id,
            }
            
            # 根據任務類型設定需求
            if wcs_task.task_type == WCSTaskType.ROTATION:
                task_requirements.update({
                    'state': 'any',  # 旋轉任務對載貨狀態無特殊要求
                    'exclude_ng': False  # NG Rack 也可能需要旋轉
                })
            elif wcs_task.task_type == WCSTaskType.EMPTY_DELIVERY:
                task_requirements.update({
                    'state': 'empty',
                    'exclude_ng': True
                })
            elif wcs_task.task_type == WCSTaskType.FULL_COLLECTION:
                task_requirements.update({
                    'state': 'full',
                    'exclude_ng': False  # 滿載可能包含 NG
                })
            
            return self.rack_state_manager.find_suitable_rack_for_task(task_requirements)
            
        except Exception as e:
            self.logger.error(f"為 WCS 任務尋找合適 Rack 時發生錯誤: {e}")
            return None

    def generate_rack_task_recommendations(self) -> List[Dict[str, Any]]:
        """
        基於 Rack 狀態生成任務建議
        
        Returns:
            List[Dict]: 任務建議列表
        """
        try:
            return self.rack_state_manager.generate_task_recommendations()
        except Exception as e:
            self.logger.error(f"生成 Rack 任務建議時發生錯誤: {e}")
            return []

    def export_rack_states_report(self) -> Dict[str, Any]:
        """
        匯出 Rack 狀態報告
        
        Returns:
            Dict: Rack 狀態報告
        """
        try:
            return self.rack_state_manager.export_rack_states()
        except Exception as e:
            self.logger.error(f"匯出 Rack 狀態報告時發生錯誤: {e}")
            return {'error': str(e)}

    def get_enhanced_system_status(self) -> dict:
        """
        獲取增強版系統狀態報告 (包含 Rack 狀態資訊)
        
        Returns:
            dict: 增強版系統狀態
        """
        try:
            # 獲取基本系統狀態
            base_status = self.get_comprehensive_system_status()
            
            # 添加 Rack 狀態資訊
            rack_statistics = self.get_rack_state_statistics()
            rack_attention = self.get_racks_needing_attention()
            task_recommendations = self.generate_rack_task_recommendations()
            
            base_status.update({
                'rack_management': {
                    'statistics': rack_statistics,
                    'attention_needed': rack_attention,
                    'task_recommendations': task_recommendations[:5],  # 只顯示前5個建議
                    'total_recommendations': len(task_recommendations)
                }
            })
            
            return base_status
            
        except Exception as e:
            self.logger.error(f"獲取增強版系統狀態時發生錯誤: {e}")
            return {'error': str(e)}

    def _validate_rotation_path(self, nodes: list) -> dict:
        """
        驗證旋轉任務路徑格式
        
        Args:
            nodes: 路徑節點列表
            
        Returns:
            dict: 驗證結果
        """
        try:
            # 基本格式檢查
            if not nodes or not isinstance(nodes, list):
                return {
                    'valid': False,
                    'error': '路徑節點列表不能為空或格式錯誤'
                }
            
            # 旋轉任務必須有3個節點
            if len(nodes) != 3:
                return {
                    'valid': False,
                    'error': f'旋轉任務需要3個節點，當前: {len(nodes)} 個'
                }
            
            # 起點和終點必須相同 (旋轉後回到原位)
            if nodes[0] != nodes[2]:
                return {
                    'valid': False,
                    'error': f'旋轉任務起點({nodes[0]})和終點({nodes[2]})必須相同'
                }
            
            # 檢查節點值有效性
            for i, node in enumerate(nodes):
                if not isinstance(node, (int, str)) or (isinstance(node, str) and not node.strip()):
                    return {
                        'valid': False,
                        'error': f'節點 {i} 格式無效: {node}'
                    }
            
            # 移出點不能與起點相同
            if nodes[1] == nodes[0]:
                return {
                    'valid': False,
                    'error': f'移出點({nodes[1]})不能與起點({nodes[0]})相同'
                }
            
            return {
                'valid': True,
                'path_type': 'rotation',
                'start_node': nodes[0],
                'intermediate_node': nodes[1],
                'end_node': nodes[2]
            }
            
        except Exception as e:
            return {
                'valid': False,
                'error': f'路徑驗證過程發生錯誤: {str(e)}'
            }

    def _validate_move_path(self, nodes: list) -> dict:
        """
        驗證搬運任務路徑格式
        
        Args:
            nodes: 路徑節點列表
            
        Returns:
            dict: 驗證結果
        """
        try:
            # 基本格式檢查
            if not nodes or not isinstance(nodes, list):
                return {
                    'valid': False,
                    'error': '路徑節點列表不能為空或格式錯誤'
                }
            
            # 搬運任務至少需要2個節點
            if len(nodes) < 2:
                return {
                    'valid': False,
                    'error': f'搬運任務至少需要2個節點，當前: {len(nodes)} 個'
                }
            
            # 檢查節點值有效性
            for i, node in enumerate(nodes):
                if not isinstance(node, (int, str)) or (isinstance(node, str) and not node.strip()):
                    return {
                        'valid': False,
                        'error': f'節點 {i} 格式無效: {node}'
                    }
            
            # 檢查路徑長度合理性 (不應超過20個節點)
            if len(nodes) > 20:
                return {
                    'valid': False,
                    'error': f'路徑過長: {len(nodes)} 個節點，建議不超過20個'
                }
            
            return {
                'valid': True,
                'path_type': 'move',
                'start_node': nodes[0],
                'end_node': nodes[-1],
                'intermediate_count': len(nodes) - 2
            }
            
        except Exception as e:
            return {
                'valid': False,
                'error': f'路徑驗證過程發生錯誤: {str(e)}'
            }

    def _execute_kuka_api_with_retry(self, api_method: str, args: list, 
                                   task_type: str = 'unknown', rack_id: int = None) -> dict:
        """
        執行 KUKA API 調用並實現重試機制
        
        Args:
            api_method: API 方法名 ('rack_move', 'move', 'workflow')
            args: API 方法參數列表
            task_type: 任務類型 (用於日誌)
            rack_id: Rack ID (用於日誌)
            
        Returns:
            dict: API 調用結果
        """
        start_time = time.time()
        last_error = None
        
        # 更新統計
        self.api_stats['total_calls'] += 1
        
        for attempt in range(self.api_retry_config['max_attempts']):
            try:
                # 計算延遲時間
                if attempt > 0:
                    delay = min(
                        self.api_retry_config['base_delay'] * (self.api_retry_config['backoff_factor'] ** (attempt - 1)),
                        self.api_retry_config['max_delay']
                    )
                    self.logger.info(f"重試 {attempt}/{self.api_retry_config['max_attempts']-1} - 延遲 {delay:.1f}秒")
                    time.sleep(delay)
                    self.api_stats['retry_calls'] += 1
                
                # 執行 API 調用
                if api_method == 'rack_move':
                    result = self.kuka_fleet.rack_move(*args)
                elif api_method == 'move':
                    result = self.kuka_fleet.move(*args)
                elif api_method == 'workflow':
                    result = self.kuka_fleet.workflow(*args)
                else:
                    return {'success': False, 'error': f'不支援的 API 方法: {api_method}'}
                
                # 檢查結果
                if result and result.get('success'):
                    # 成功
                    response_time = time.time() - start_time
                    self._update_api_stats(True, response_time)
                    
                    self.logger.info(
                        f"KUKA API 調用成功 - 方法: {api_method}, "
                        f"任務類型: {task_type}, Rack: {rack_id}, "
                        f"嘗試: {attempt + 1}, 響應時間: {response_time:.2f}秒"
                    )
                    return result
                else:
                    # API 返回失敗
                    error_msg = result.get('error', '未知錯誤') if result else 'API 返回空結果'
                    last_error = f"API 調用失敗: {error_msg}"
                    
                    if attempt < self.api_retry_config['max_attempts'] - 1:
                        self.logger.warning(
                            f"KUKA API 調用失敗 (嘗試 {attempt + 1}) - {last_error}, 準備重試"
                        )
                    
            except Exception as e:
                last_error = f"API 調用異常: {str(e)}"
                
                if attempt < self.api_retry_config['max_attempts'] - 1:
                    self.logger.warning(
                        f"KUKA API 調用異常 (嘗試 {attempt + 1}) - {last_error}, 準備重試"
                    )
                else:
                    self.logger.error(f"KUKA API 調用最終失敗 - {last_error}")
        
        # 所有重試都失敗了
        response_time = time.time() - start_time
        self._update_api_stats(False, response_time)
        
        self.logger.error(
            f"KUKA API 調用最終失敗 - 方法: {api_method}, "
            f"任務類型: {task_type}, Rack: {rack_id}, "
            f"總嘗試: {self.api_retry_config['max_attempts']}, "
            f"最後錯誤: {last_error}"
        )
        
        return {'success': False, 'error': last_error}

    def _update_api_stats(self, success: bool, response_time: float):
        """更新 API 統計資訊"""
        try:
            if success:
                self.api_stats['successful_calls'] += 1
            else:
                self.api_stats['failed_calls'] += 1
            
            # 更新平均響應時間
            total_calls = self.api_stats['total_calls']
            current_avg = self.api_stats['average_response_time']
            self.api_stats['average_response_time'] = (
                (current_avg * (total_calls - 1) + response_time) / total_calls
            )
            
        except Exception as e:
            self.logger.error(f"更新 API 統計時發生錯誤: {e}")
    
    def get_api_statistics(self) -> dict:
        """
        獲取 API 調用統計資訊
        
        Returns:
            dict: API 統計資訊
        """
        try:
            stats = self.api_stats.copy()
            
            # 計算成功率
            if stats['total_calls'] > 0:
                stats['success_rate'] = (stats['successful_calls'] / stats['total_calls']) * 100
            else:
                stats['success_rate'] = 0.0
            
            # 計算重試率
            if stats['total_calls'] > 0:
                stats['retry_rate'] = (stats['retry_calls'] / stats['total_calls']) * 100
            else:
                stats['retry_rate'] = 0.0
            
            return stats
            
        except Exception as e:
            self.logger.error(f"獲取 API 統計時發生錯誤: {e}")
            return {'error': str(e)}

    def stop_monitoring(self):
        """停止 KUKA Fleet 監控"""
        if hasattr(self, 'kuka_fleet') and self.kuka_fleet:
            self.kuka_fleet.stop_monitoring()
            self.logger.info("KUKA Fleet 監控已停止")
