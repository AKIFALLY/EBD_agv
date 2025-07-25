"""
KUKA 任務派發器 V2
重構版本，對齊 KukaManager 功能但保持輕量級設計
整合錯誤處理、WCS 支援、優先度調度等核心功能
"""
import uuid
import time
from datetime import datetime, timezone
from typing import Dict, List, Any, Optional
from itertools import zip_longest

from db_proxy.connection_pool_manager import ConnectionPoolManager
from kuka_fleet_adapter.kuka_fleet_adapter import KukaFleetAdapter


class KukaWorkType:
    """KUKA 工作類型常數定義 (與 KukaManager 保持一致)"""
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
        return function_map.get(function, cls.KUKA_WORKFLOW)


class KukaDispatcherV2:
    """KUKA 任務派發器 V2 - 增強版"""

    def __init__(self, rcs_core):
        """
        初始化 KUKA 派發器

        Args:
            rcs_core: RCS Core 節點實例
        """
        self.rcs_core = rcs_core
        self.kuka_fleet: KukaFleetAdapter = rcs_core.kuka_fleet
        self.db_pool: ConnectionPoolManager = rcs_core.db_pool
        self.logger = rcs_core.get_logger()
        
        # API 重試配置
        self.api_retry_config = {
            'max_attempts': 3,
            'base_delay': 1.0,  # 基礎延遲(秒)
            'backoff_factor': 2.0,  # 指數退避因子
            'max_delay': 30.0,  # 最大延遲(秒)
        }
        
        # 統計資訊
        self.dispatch_stats = {
            'total_dispatches': 0,
            'successful_dispatches': 0,
            'failed_dispatches': 0,
            'retry_attempts': 0,
            'last_dispatch_time': None
        }
        
        # 載入 KUKA AGV 資料
        self.kuka_agvs = {}
        self._load_kuka_agvs()

    def _load_kuka_agvs(self):
        """載入 KUKA AGV 資料"""
        if not self.db_pool:
            self.logger.error("資料庫連線池不可用，無法載入 KUKA AGV")
            return
        
        try:
            with self.db_pool.get_session() as session:
                from db_proxy.models import AGV
                from sqlmodel import select
                
                agvs = session.exec(
                    select(AGV).where(AGV.enable == 1, AGV.model == "KUKA400i")
                ).all()
                
                self.kuka_agvs = {agv.id: agv for agv in agvs}
                self.logger.info(f"已載入 {len(self.kuka_agvs)} 台 KUKA400i AGV")
                
        except Exception as e:
            self.logger.error(f"載入 KUKA AGV 時發生錯誤: {e}")

    def dispatch(self):
        """KUKA400i AGV 智能任務派發 - 增強版"""
        try:
            start_time = time.time()
            
            # 1. 查詢閒置的 KUKA400i AGV
            idle_agvs = self._get_available_agvs()
            if not idle_agvs:
                self.logger.debug("目前沒有可用的 KUKA400i AGV")
                return
            
            # 2. 查詢待執行任務
            pending_tasks = self._get_pending_tasks()
            if not pending_tasks:
                self.logger.debug("目前沒有 KUKA 任務需要處理")
                return
            
            # 3. 排序任務 (依優先度降序)
            sorted_tasks = self._sort_tasks_by_priority(pending_tasks)
            
            self.logger.info(
                f"開始派發: {len(sorted_tasks)} 個任務, {len(idle_agvs)} 台可用 AGV"
            )
            
            # 4. 執行任務派發
            dispatch_results = self._execute_task_dispatch(idle_agvs, sorted_tasks)
            
            # 5. 更新統計資訊
            self._update_dispatch_stats(dispatch_results, time.time() - start_time)
            
        except Exception as e:
            self.logger.error(f"任務派發時發生錯誤: {e}", exc_info=True)
            self.dispatch_stats['failed_dispatches'] += 1

    def _get_available_agvs(self) -> List[int]:
        """取得可用的 KUKA AGV 列表"""
        try:
            # 從 Fleet Adapter 查詢閒置 AGV
            idle_kuka_agvs = self.kuka_fleet.select_agv(KukaFleetAdapter.STATUS_IDLE)
            idle_agv_ids = [int(agv["id"]) for agv in idle_kuka_agvs]
            
            if not idle_agv_ids:
                return []
            
            # 驗證資料庫中的 AGV 狀態
            with self.db_pool.get_session() as session:
                from db_proxy.models import AGV
                from sqlmodel import select
                
                available_agvs = session.exec(
                    select(AGV).where(
                        AGV.enable == 1,
                        AGV.model == "KUKA400i",
                        AGV.id.in_(idle_agv_ids),
                        AGV.status_id == 3  # 確保資料庫中也是閒置狀態
                    )
                ).all()
                
                return [agv.id for agv in available_agvs]
                
        except Exception as e:
            self.logger.error(f"取得可用 AGV 時發生錯誤: {e}")
            return []

    def _get_pending_tasks(self) -> List:
        """取得待執行的 KUKA 任務"""
        try:
            with self.db_pool.get_session() as session:
                from db_proxy.models import Task
                from sqlmodel import select
                
                # 查詢所有待執行的 KUKA400i 任務
                tasks = session.exec(
                    select(Task).where(
                        Task.status_id == 1,  # 待執行
                        Task.mission_code == None,  # 尚未指定任務代碼
                        Task.parameters["model"].as_string() == "KUKA400i"
                    )
                ).all()
                
                # 篩選 KUKA 相關任務
                kuka_tasks = []
                for task in tasks:
                    if (KukaWorkType.is_kuka_work(task.work_id) or 
                        self._is_wcs_format_task(task)):
                        kuka_tasks.append(task)
                
                return kuka_tasks
                
        except Exception as e:
            self.logger.error(f"取得待執行任務時發生錯誤: {e}")
            return []

    def _is_wcs_format_task(self, task) -> bool:
        """檢查任務是否為 WCS 格式 (與 KukaManager 保持一致)"""
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

    def _sort_tasks_by_priority(self, tasks: List) -> List:
        """依優先度排序任務 (高優先度優先)"""
        return sorted(tasks, key=lambda t: getattr(t, 'priority', 40), reverse=True)

    def _execute_task_dispatch(self, agv_ids: List[int], tasks: List) -> Dict[str, int]:
        """執行任務派發"""
        results = {
            'successful': 0,
            'failed': 0,
            'total': 0
        }
        
        try:
            with self.db_pool.get_session() as session:
                for agv_id, task in zip_longest(agv_ids, tasks):
                    if not agv_id or not task:
                        continue
                    
                    results['total'] += 1
                    
                    # 生成任務代碼
                    mission_code = str(uuid.uuid4())
                    
                    # 執行任務
                    success = self._dispatch_single_task(session, task, agv_id, mission_code)
                    
                    if success:
                        results['successful'] += 1
                        self.logger.info(
                            f"✅ 任務 {task.id} 成功派發給 AGV {agv_id}, "
                            f"mission_code: {mission_code}"
                        )
                    else:
                        results['failed'] += 1
                        self.logger.warning(
                            f"❌ 任務 {task.id} 派發給 AGV {agv_id} 失敗"
                        )
                
                # 提交資料庫變更
                session.commit()
                
        except Exception as e:
            self.logger.error(f"執行任務派發時發生錯誤: {e}")
            results['failed'] = results['total'] - results['successful']
        
        return results

    def _dispatch_single_task(self, session, task, agv_id: int, mission_code: str) -> bool:
        """派發單個任務"""
        try:
            # 取得任務描述
            task_description = self._get_task_description(task)
            
            # 執行任務
            execution_result = self._execute_kuka_task(task, agv_id, mission_code)
            
            if execution_result["success"]:
                # 更新任務狀態
                self._update_task_status(session, task, agv_id, mission_code)
                
                # 更新 AGV 狀態
                self._update_agv_status(session, agv_id, 4)  # 任務中
                
                return True
            else:
                self.logger.error(
                    f"任務執行失敗: {execution_result.get('error', '未知錯誤')}"
                )
                return False
                
        except Exception as e:
            self.logger.error(f"派發單個任務時發生錯誤: {e}")
            return False

    def _execute_kuka_task(self, task, agv_id: int, mission_code: str) -> Dict[str, Any]:
        """執行 KUKA 任務 (支援 WCS 和傳統任務格式)"""
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
                self.logger.warning(f"未知的 KUKA 工作類型 {work_id}，嘗試作為 workflow 處理")
                return self._execute_workflow_task(task, agv_id, mission_code)
                
        except Exception as e:
            self.logger.error(f"執行 KUKA 任務時發生錯誤: {e}")
            return {"success": False, "error": str(e)}

    def _execute_wcs_format_task(self, task, agv_id: int, mission_code: str) -> Dict[str, Any]:
        """執行 WCS 格式任務"""
        try:
            params = task.parameters or {}
            
            function = params.get('function', 'rack_move')
            nodes = params.get('nodes', [])
            
            # 驗證參數
            if not nodes or len(nodes) < 2:
                return {"success": False, "error": "WCS 任務缺少有效的節點路徑"}
            
            # 根據 function 執行對應操作
            if function == 'rack_move':
                return self._execute_api_with_retry('rack_move', [nodes, agv_id, mission_code])
            elif function == 'move':
                return self._execute_api_with_retry('move', [nodes, agv_id, mission_code])
            else:
                self.logger.warning(f"未知的 WCS function: {function}，使用預設 rack_move")
                return self._execute_api_with_retry('rack_move', [nodes, agv_id, mission_code])
                
        except Exception as e:
            self.logger.error(f"執行 WCS 格式任務時發生錯誤: {e}")
            return {"success": False, "error": str(e)}

    def _execute_move_task(self, task, agv_id: int, mission_code: str) -> Dict[str, Any]:
        """執行移動任務"""
        nodes = task.parameters.get('nodes')
        if not nodes:
            return {"success": False, "error": "缺少參數 'nodes'"}
        
        return self._execute_api_with_retry('move', [nodes, agv_id, mission_code])

    def _execute_rack_move_task(self, task, agv_id: int, mission_code: str) -> Dict[str, Any]:
        """執行移動貨架任務"""
        nodes = task.parameters.get('nodes')
        if not nodes:
            return {"success": False, "error": "缺少參數 'nodes'"}
        
        return self._execute_api_with_retry('rack_move', [nodes, agv_id, mission_code])

    def _execute_workflow_task(self, task, agv_id: int, mission_code: str) -> Dict[str, Any]:
        """執行工作流程任務"""
        template_code = task.parameters.get('templateCode')
        if not template_code:
            return {"success": False, "error": "缺少參數 'templateCode'"}
        
        return self._execute_api_with_retry('workflow', [template_code, agv_id, mission_code])

    def _execute_api_with_retry(self, api_method: str, args: list) -> Dict[str, Any]:
        """執行 KUKA API 調用並實現重試機制"""
        last_error = None
        
        for attempt in range(self.api_retry_config['max_attempts']):
            try:
                # 第一次重試前延遲
                if attempt > 0:
                    delay = min(
                        self.api_retry_config['base_delay'] * (self.api_retry_config['backoff_factor'] ** (attempt - 1)),
                        self.api_retry_config['max_delay']
                    )
                    self.logger.info(f"重試 {attempt}/{self.api_retry_config['max_attempts']-1} - 延遲 {delay:.1f}秒")
                    time.sleep(delay)
                    self.dispatch_stats['retry_attempts'] += 1
                
                # 執行 API 調用
                if api_method == 'move':
                    result = self.kuka_fleet.move(*args)
                elif api_method == 'rack_move':
                    result = self.kuka_fleet.rack_move(*args)
                elif api_method == 'workflow':
                    result = self.kuka_fleet.workflow(*args)
                else:
                    return {'success': False, 'error': f'不支援的 API 方法: {api_method}'}
                
                # 檢查結果
                if result and result.get('success'):
                    return result
                else:
                    last_error = result.get('error', '未知錯誤') if result else 'API 返回空結果'
                    
            except Exception as e:
                last_error = f"API 調用異常: {str(e)}"
        
        # 所有重試都失敗
        return {'success': False, 'error': last_error}

    def _update_task_status(self, session, task, agv_id: int, mission_code: str):
        """更新任務狀態"""
        task.agv_id = agv_id
        task.mission_code = mission_code
        task.status_id = 2  # 執行中
        task.updated_at = datetime.now(timezone.utc)
        
        # 更新任務參數
        if not task.parameters:
            task.parameters = {}
        task.parameters["agvId"] = agv_id
        
        session.add(task)

    def _update_agv_status(self, session, agv_id: int, status_id: int):
        """更新 AGV 狀態"""
        from db_proxy.models import AGV
        from sqlmodel import select
        
        agv = session.exec(select(AGV).where(AGV.id == agv_id)).first()
        if agv:
            agv.status_id = status_id
            session.add(agv)

    def _get_task_description(self, task) -> str:
        """取得任務描述"""
        try:
            params = task.parameters or {}
            
            # WCS 任務格式
            if 'wcs_task_type' in params:
                wcs_type = params['wcs_task_type']
                return f"WCS-{wcs_type}"
            
            # 傳統 KUKA 任務格式
            if KukaWorkType.is_kuka_work(task.work_id):
                work_name = KukaWorkType.get_work_name(task.work_id)
                return f"{work_name}"
            
            # 基於 function 的描述
            function = params.get('function', 'unknown')
            return f"Function: {function}"
            
        except Exception as e:
            self.logger.error(f"取得任務描述時發生錯誤: {e}")
            return f"Task ID: {task.id}"

    def _update_dispatch_stats(self, results: Dict[str, int], execution_time: float):
        """更新派發統計資訊"""
        self.dispatch_stats['total_dispatches'] += results['total']
        self.dispatch_stats['successful_dispatches'] += results['successful']
        self.dispatch_stats['failed_dispatches'] += results['failed']
        self.dispatch_stats['last_dispatch_time'] = datetime.now(timezone.utc).isoformat()
        
        if results['total'] > 0:
            success_rate = (results['successful'] / results['total']) * 100
            self.logger.info(
                f"派發完成: 成功 {results['successful']}/{results['total']} "
                f"({success_rate:.1f}%), 耗時 {execution_time:.2f}秒"
            )

    def get_dispatch_statistics(self) -> Dict[str, Any]:
        """取得派發統計資訊"""
        stats = self.dispatch_stats.copy()
        
        # 計算成功率
        if stats['total_dispatches'] > 0:
            stats['success_rate'] = (stats['successful_dispatches'] / stats['total_dispatches']) * 100
        else:
            stats['success_rate'] = 0.0
        
        return stats

    def reset_statistics(self):
        """重置統計資訊"""
        self.dispatch_stats = {
            'total_dispatches': 0,
            'successful_dispatches': 0,
            'failed_dispatches': 0,
            'retry_attempts': 0,
            'last_dispatch_time': None
        }
        self.logger.info("派發統計資訊已重置")

    def get_system_status(self) -> Dict[str, Any]:
        """取得系統狀態"""
        try:
            available_agvs = self._get_available_agvs()
            pending_tasks = self._get_pending_tasks()
            
            return {
                'timestamp': datetime.now(timezone.utc).isoformat(),
                'available_agvs': len(available_agvs),
                'pending_tasks': len(pending_tasks),
                'kuka_agvs_loaded': len(self.kuka_agvs),
                'statistics': self.get_dispatch_statistics(),
                'api_retry_config': self.api_retry_config
            }
            
        except Exception as e:
            self.logger.error(f"取得系統狀態時發生錯誤: {e}")
            return {'error': str(e)}


# 為了向後相容，提供別名
KukaDispatcher = KukaDispatcherV2