"""
統一任務管理器
負責將決策引擎的決策轉換為具體的資料庫任務，並管理任務參數格式
"""

import logging
from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass, asdict
import json
from datetime import datetime, timezone

from .unified_decision_engine import TaskDecision, WorkIDCategory
from .enhanced_database_client import EnhancedDatabaseClient, TaskInfo


@dataclass
class TaskCreationResult:
    """任務創建結果"""
    success: bool
    task_id: Optional[int] = None
    error_message: str = ""
    created_at: datetime = None
    
    def __post_init__(self):
        if self.created_at is None:
            self.created_at = datetime.now(timezone.utc)


class WorkIDParameterManager:
    """Work ID 參數管理器 - 基於設計文檔的完整映射"""
    
    def __init__(self):
        # Work ID 定義映射 (基於實際資料庫定義)
        self.WORK_ID_MAPPINGS = {
            # === OPUI 操作員任務群組 ===
            "100001": {
                "name": "opui-call-empty",
                "description": "作業員從opui請求將空Rack派至[人工作業準備區]",
                "category": "opui_operations",
                "priority": 40,
                "function": "rack_move",
                "api": "submit_mission",
                "missionType": "RACK_MOVE"
            },
            "100002": {
                "name": "opui-dispatch-full", 
                "description": "作業員從opui請求將Rack派至[系統準備派車區]",
                "category": "opui_operations",
                "priority": 40,
                "function": "rack_move",
                "api": "submit_mission",
                "missionType": "RACK_MOVE"
            },
            
            # === KUKA 基礎移動任務群組 ===
            "210001": {
                "name": "kuka-移動",
                "description": "執行指定的from,to(nodes)移動至指定位置",
                "category": "basic_movement",
                "priority": 60,
                "function": "move",
                "api": "submit_mission",
                "missionType": "MOVE"
            },
            "220001": {
                "name": "kuka-移動貨架", 
                "description": "執行指定的from,to(nodes)將貨架搬至指定位置",
                "category": "rack_transport",
                "priority": 80,
                "function": "rack_move",
                "api": "submit_mission",
                "missionType": "RACK_MOVE"
            },
            "230001": {
                "name": "kuka-流程觸發",
                "description": "執行指定的workflow流程觸發", 
                "category": "workflow_trigger",
                "priority": 100,
                "function": "workflow",
                "api": "submit_mission",
                "missionType": "MOVE",
                "templateCode": "W000000001"
            },
            
            # === Cargo AGV 任務群組 ===
            "2000102": {
                "name": "CargoAGV放入口傳送箱",
                "description": "從料架拿carrier到入口傳送箱放",
                "category": "cargo_inlet",
                "priority": 80,
                "function": "rack_move",
                "api": "submit_mission",
                "missionType": "RACK_MOVE"
            },
            "2000201": {
                "name": "CargoAGV拿出口傳送箱",
                "description": "從出口傳送箱拿carrier到料架放", 
                "category": "cargo_outlet",
                "priority": 80,
                "function": "rack_move",
                "api": "submit_mission",
                "missionType": "RACK_MOVE"
            }
        }
        
        # 業務流程與Work ID映射
        self.BUSINESS_FLOW_WORK_IDS = {
            # WCS 決策引擎使用的 Work IDs (大部分使用 kuka-移動貨架)
            'agv_rotation': "220001",              # AGV旋轉檢查 → kuka-移動貨架 (改用3節點移動)
            'ng_rack_recycling': "220001",         # NG料架回收 → kuka-移動貨架
            'full_rack_to_manual': "220001",       # 滿料架到人工收料區 → kuka-移動貨架
            'manual_area_transport': "220001",     # 人工收料區搬運 → kuka-移動貨架
            'system_to_room': "220001",           # 系統準備區到房間 → kuka-移動貨架
            'empty_rack_transfer': "220001",      # 空料架搬運 → kuka-移動貨架
            'manual_empty_recycling': "230001",   # 人工回收空料架 → kuka-流程觸發 ⭐唯一特殊例外
            
            # OPUI 手動任務
            'opui_call_empty': "100001",          # OPUI叫空車
            'opui_dispatch_full': "100002",       # OPUI派滿車
            
            # Cargo AGV 任務
            'cargo_inlet': "2000102",             # CargoAGV放入口傳送箱
            'cargo_outlet': "2000201",            # CargoAGV拿出口傳送箱
        }
        
        # Machine停車格配置 (實際初始化資料)
        self.MACHINE_PARKING_CONFIG = {
            1: {"parking_space_1": 95, "parking_space_2": 96, "name": "射出機1"},
            2: {"parking_space_1": 97, "parking_space_2": 98, "name": "射出機2"}, 
            3: {"parking_space_1": 1005, "parking_space_2": 1006, "name": "射出機3"},
            4: {"parking_space_1": 1007, "parking_space_2": 1008, "name": "射出機4"}
        }
    
    def get_work_id_info(self, work_id: str) -> Dict[str, Any]:
        """獲取Work ID資訊"""
        return self.WORK_ID_MAPPINGS.get(work_id, {})
    
    def build_kuka_rack_move_parameters(self, decision: TaskDecision) -> Dict[str, Any]:
        """建立KUKA移動貨架任務參數 (work_id: 220001)"""
        work_info = self.get_work_id_info(decision.work_id)
        
        base_params = {
            "function": work_info.get("function", "rack_move"),
            "model": "KUKA400i",
            "work_id": int(decision.work_id),
            "api": work_info.get("api", "submit_mission"),
            "missionType": work_info.get("missionType", "RACK_MOVE"),
            "nodes": decision.nodes if decision.nodes else [decision.source_location, decision.target_location],
            "task_category": decision.task_type,
            "priority_level": decision.priority,
            "source_location": decision.source_location,
            "target_location": decision.target_location
        }
        
        # 根據任務類型添加特殊參數
        if decision.task_type == "agv_rotation":
            base_params.update({
                "rotation_type": "3_node_movement",
                "location_type": self._get_location_type(decision.target_location),
                "agv_id": decision.agv_id,
                "parent_task_id": decision.parent_task_id,
                "description": "AGV在房間入口/出口執行旋轉動作"
            })
        elif decision.rack_id:
            base_params["rack_id"] = decision.rack_id
            
        if decision.room_id:
            base_params["room_id"] = decision.room_id
            
        # 合併自定義參數
        base_params.update(decision.parameters)
        
        return base_params
    
    def build_kuka_workflow_parameters(self, decision: TaskDecision) -> Dict[str, Any]:
        """建立KUKA流程觸發任務參數 (work_id: 230001) - 人工回收空料架專用"""
        work_info = self.get_work_id_info(decision.work_id)
        
        params = {
            "function": work_info.get("function", "workflow"),
            "model": "KUKA400i", 
            "work_id": int(decision.work_id),
            "api": work_info.get("api", "submit_mission"),
            "missionType": work_info.get("missionType", "MOVE"),
            "templateCode": work_info.get("templateCode", "W000000001"),
            "task_category": decision.task_type,
            "priority_level": decision.priority,
            "source_location": decision.source_location,
            "target_location": decision.target_location,
            "description": "人工回收空料架區搬運到系統空料架區"
        }
        
        # 合併自定義參數
        params.update(decision.parameters)
        
        return params
    
    def build_opui_call_empty_parameters(self, decision: TaskDecision) -> Dict[str, Any]:
        """建立OPUI叫空車任務參數 (work_id: 100001) - 基於實際OPUI邏輯"""
        work_info = self.get_work_id_info(decision.work_id)
        
        # 從decision.parameters中獲取OPUI特定參數
        machine_id = decision.parameters.get('machine_id')
        space_num = decision.parameters.get('space_num', 1)
        client_id = decision.parameters.get('client_id', 'clientId')
        
        # 獲取停車格對應的node_id
        machine_config = self.MACHINE_PARKING_CONFIG.get(machine_id, {})
        node_id = machine_config.get(f'parking_space_{space_num}')
        
        params = {
            "work_id": int(decision.work_id),
            "function": work_info.get("function", "rack_move"),
            "api": work_info.get("api", "submit_mission"),
            "missionType": work_info.get("missionType", "RACK_MOVE"),
            "model": "KUKA400i",
            "task_category": "opui_call_empty",
            "priority_level": decision.priority,
            
            # OPUI 特定參數
            "task_type": "call_empty",
            "machine_id": machine_id,
            "space_num": space_num,
            "node_id": node_id,
            "client_id": client_id,
            
            # KUKA 參數 - 移動路徑：取空車位置 → 中間點 → 目標停車格
            "nodes": decision.nodes if decision.nodes else [91, 76, node_id],
            
            # 停車格狀態管理
            "parking_space_status": 1  # 設置為 PARKING_TASK_ACTIVE (任務進行中)
        }
        
        # 合併其他參數
        params.update(decision.parameters)
        
        return params
    
    def build_opui_dispatch_full_parameters(self, decision: TaskDecision) -> Dict[str, Any]:
        """建立OPUI派滿車任務參數 (work_id: 100002) - 基於實際OPUI邏輯"""
        work_info = self.get_work_id_info(decision.work_id)
        
        # 從decision.parameters中獲取OPUI特定參數
        rack_id = decision.parameters.get('rack_id', decision.rack_id)
        room_id = decision.parameters.get('room_id', decision.room_id)
        machine_id = decision.parameters.get('machine_id')
        side = decision.parameters.get('side', 'left')  # left=space_1, right=space_2
        client_id = decision.parameters.get('client_id', 'clientId')
        product_name = decision.parameters.get('product_name', 'ABC121345')
        count = decision.parameters.get('count', 32)
        
        params = {
            "work_id": int(decision.work_id),
            "function": work_info.get("function", "rack_move"),
            "api": work_info.get("api", "submit_mission"),
            "missionType": work_info.get("missionType", "RACK_MOVE"),
            "model": "KUKA400i", 
            "task_category": "opui_dispatch_full",
            "priority_level": decision.priority,
            
            # OPUI 特定參數
            "task_type": "dispatch_full",
            "rack_id": rack_id,
            "room_id": room_id,
            "machine_id": machine_id,
            "side": side,
            "client_id": client_id,
            
            # 產品資訊
            "product_name": product_name,
            "count": count,
            
            # KUKA 參數 - 移動路徑：停車格 → 中間點 → 系統準備派車區
            "nodes": decision.nodes if decision.nodes else [95, 74, 72, 15],
            
            # WCS 決策參數
            "target_area": "system_prep_area"  # 系統準備派車區 (位置11-18)
        }
        
        # 合併其他參數
        params.update(decision.parameters)
        
        return params
    
    def build_cargo_agv_parameters(self, decision: TaskDecision) -> Dict[str, Any]:
        """建立CargoAGV任務參數"""
        work_info = self.get_work_id_info(decision.work_id)
        
        params = {
            "function": work_info.get("function", "rack_move"),
            "model": "KUKA400i",
            "work_id": int(decision.work_id),
            "api": work_info.get("api", "submit_mission"),
            "missionType": work_info.get("missionType", "RACK_MOVE"), 
            "nodes": decision.nodes,
            "task_category": work_info.get("category"),
            "priority_level": decision.priority,
            "room_id": decision.room_id,
            "description": work_info.get("description", "")
        }
        
        # 合併自定義參數
        params.update(decision.parameters)
        
        return params
    
    def _get_location_type(self, location_id: int) -> str:
        """根據location_id判斷位置類型"""
        if location_id % 10000 == 1 and location_id >= 10001:
            return "inlet"
        elif location_id % 10000 == 2 and location_id >= 10002:
            return "outlet"
        return "general"


class UnifiedTaskManager:
    """統一任務管理器"""
    
    def __init__(self, logger=None):
        self.logger = logger or logging.getLogger('unified_task_manager')
        self.logger.info('統一任務管理器啟動 - 直接連接模式')
        
        # 初始化組件
        self.db_client = EnhancedDatabaseClient(logger=self.logger)
        self.param_manager = WorkIDParameterManager()
        
        # 任務狀態追蹤
        self.created_tasks: Dict[int, TaskInfo] = {}
        self.failed_creations: List[TaskDecision] = []
        self.active_tasks: Dict[int, TaskInfo] = {}  # 修正：添加缺少的active_tasks屬性
        
        # 統計資料
        self.task_stats = {
            'created': 0,
            'failed': 0,
            'by_work_id': {},
            'by_priority': {}
        }
    
    def create_tasks_from_decisions(self, decisions: List[TaskDecision]) -> List[TaskCreationResult]:
        """批次創建任務從決策列表 (修正：改為同步方法以符合ai_wcs_node調用)"""
        results = []
        
        self.logger.info(f'開始批次創建 {len(decisions)} 個任務')
        
        for decision in decisions:
            result = self.create_task_from_decision(decision)
            results.append(result)
            
            # 更新統計
            if result.success:
                self.task_stats['created'] += 1
                work_id = decision.work_id
                priority = decision.priority
                
                self.task_stats['by_work_id'][work_id] = self.task_stats['by_work_id'].get(work_id, 0) + 1
                self.task_stats['by_priority'][priority] = self.task_stats['by_priority'].get(priority, 0) + 1
            else:
                self.task_stats['failed'] += 1
                self.failed_creations.append(decision)
        
        success_count = sum(1 for r in results if r.success)
        self.logger.info(f'批次任務創建完成: 成功 {success_count}/{len(decisions)}')
        
        return results
    
    def create_task_from_decision(self, decision: TaskDecision) -> TaskCreationResult:
        """從單一決策創建任務"""
        try:
            # 根據work_id建立對應參數
            parameters = self._build_task_parameters(decision)
            
            # 準備任務字典
            task_dict = {
                'work_id': decision.work_id,
                'status_id': 0,  # REQUESTING
                'room_id': decision.room_id,
                'node_id': decision.target_location,
                'name': self._generate_task_name(decision),
                'description': decision.reason,
                'agv_id': decision.agv_id,
                'parent_task_id': decision.parent_task_id,
                'priority': decision.priority,
                'parameters': parameters
            }
            
            # 創建任務到資料庫
            task_id = self.db_client.create_task_from_decision(task_dict)
            
            if task_id:
                self.logger.info(
                    f'成功創建任務: ID={task_id}, work_id={decision.work_id}, '
                    f'type={decision.task_type}, priority={decision.priority}'
                )
                
                # 🔗 OPUI停車格狀態同步
                if decision.work_id in ['100001', '100002']:
                    sync_success = self.sync_opui_parking_status_for_task(decision, task_id)
                    if sync_success:
                        self.logger.info(f'📋 OPUI停車格狀態同步成功: task_id={task_id}')
                    else:
                        self.logger.warning(f'📋 OPUI停車格狀態同步失敗: task_id={task_id}')
                
                return TaskCreationResult(
                    success=True,
                    task_id=task_id
                )
            else:
                error_msg = f"資料庫創建任務失敗: {decision.task_type}"
                self.logger.error(error_msg)
                
                return TaskCreationResult(
                    success=False,
                    error_message=error_msg
                )
                
        except Exception as e:
            error_msg = f"創建任務異常: {e}"
            self.logger.error(error_msg)
            
            return TaskCreationResult(
                success=False,
                error_message=error_msg
            )
    
    def _build_task_parameters(self, decision: TaskDecision) -> Dict[str, Any]:
        """根據work_id建立任務參數"""
        work_id = decision.work_id
        
        if work_id == "220001":
            # kuka-移動貨架 (大部分業務流程)
            return self.param_manager.build_kuka_rack_move_parameters(decision)
        elif work_id == "230001":
            # kuka-流程觸發 (人工回收空料架專用)
            return self.param_manager.build_kuka_workflow_parameters(decision)
        elif work_id == "100001":
            # OPUI叫空車
            return self.param_manager.build_opui_call_empty_parameters(decision)
        elif work_id == "100002":
            # OPUI派滿車
            return self.param_manager.build_opui_dispatch_full_parameters(decision)
        elif work_id in ["2000102", "2000201"]:
            # CargoAGV任務
            return self.param_manager.build_cargo_agv_parameters(decision)
        else:
            # 預設參數 (基於220001格式)
            return self.param_manager.build_kuka_rack_move_parameters(decision)
    
    def _generate_task_name(self, decision: TaskDecision) -> str:
        """生成任務名稱"""
        work_info = self.param_manager.get_work_id_info(decision.work_id)
        work_name = work_info.get('name', f'work_{decision.work_id}')
        
        return f"{work_name}_{decision.task_type}_{decision.source_location}_to_{decision.target_location}"
    
    def update_opui_parking_status(self, machine_id: int, space_num: int, status: int) -> bool:
        """更新OPUI停車格狀態 - 完整實現"""
        try:
            # 使用增強資料庫客戶端更新停車格狀態
            success = self.db_client.update_machine_parking_status(machine_id, space_num, status)
            
            if success:
                self.logger.info(
                    f'✅ OPUI停車格狀態更新成功: machine_id={machine_id}, space_{space_num}={status}'
                )
            else:
                self.logger.error(
                    f'❌ OPUI停車格狀態更新失敗: machine_id={machine_id}, space_{space_num}={status}'
                )
            
            return success
            
        except Exception as e:
            self.logger.error(f'更新OPUI停車格狀態異常: {e}')
            return False
    
    def sync_opui_parking_status_for_task(self, decision: TaskDecision, task_id: int) -> bool:
        """為創建的任務同步OPUI停車格狀態"""
        try:
            # 只處理OPUI相關任務
            if decision.work_id not in ['100001', '100002']:
                return True  # 非OPUI任務，不需要同步停車格狀態
            
            # 從任務參數中獲取machine_id和space_num
            machine_id = decision.parameters.get('machine_id')
            space_num = decision.parameters.get('space_num')
            
            if not machine_id or not space_num:
                self.logger.warning(
                    f'OPUI任務缺少必要參數: machine_id={machine_id}, space_num={space_num}'
                )
                return False
            
            # 根據work_id決定停車格狀態
            if decision.work_id == '100001':  # opui-call-empty
                # 叫空車：設置狀態為任務進行中
                new_status = 1  # PARKING_TASK_ACTIVE
                action = "叫空車任務創建"
            elif decision.work_id == '100002':  # opui-dispatch-full
                # 派滿車：通常不改變停車格狀態，因為是將現有料架移走
                return True  # 派滿車不需要更新停車格狀態
            else:
                return True
            
            # 更新停車格狀態
            success = self.update_opui_parking_status(machine_id, space_num, new_status)
            
            if success:
                self.logger.info(
                    f'🔗 OPUI停車格狀態同步成功: {action}, task_id={task_id}, '\
                    f'machine_id={machine_id}, space_{space_num}={new_status}'
                )
            else:
                self.logger.error(
                    f'🔗 OPUI停車格狀態同步失敗: {action}, task_id={task_id}'
                )
            
            return success
            
        except Exception as e:
            self.logger.error(f'OPUI停車格狀態同步異常: {e}')
            return False
    
    def handle_opui_task_completion(self, task_id: int, machine_id: int, space_num: int) -> bool:
        """處理OPUI任務完成後的停車格狀態更新"""
        try:
            # 任務完成：設置狀態為任務完成
            completed_status = 2  # PARKING_TASK_COMPLETED
            
            success = self.update_opui_parking_status(machine_id, space_num, completed_status)
            
            if success:
                self.logger.info(
                    f'🎯 OPUI任務完成，停車格狀態更新: task_id={task_id}, '\
                    f'machine_id={machine_id}, space_{space_num}={completed_status}'
                )
            else:
                self.logger.error(
                    f'🎯 OPUI任務完成，停車格狀態更新失敗: task_id={task_id}'
                )
            
            return success
            
        except Exception as e:
            self.logger.error(f'處理OPUI任務完成異常: {e}')
            return False
    
    def reset_opui_parking_status(self, machine_id: int, space_num: int) -> bool:
        """重置OPUI停車格狀態為可用 - 確認送達後調用"""
        try:
            # 重置為可用狀態
            available_status = 0  # PARKING_AVAILABLE
            
            success = self.update_opui_parking_status(machine_id, space_num, available_status)
            
            if success:
                self.logger.info(
                    f'🔄 OPUI停車格狀態重置為可用: machine_id={machine_id}, space_{space_num}={available_status}'
                )
            else:
                self.logger.error(
                    f'🔄 OPUI停車格狀態重置失敗: machine_id={machine_id}, space_{space_num}'
                )
            
            return success
            
        except Exception as e:
            self.logger.error(f'重置OPUI停車格狀態異常: {e}')
            return False
    
    def get_opui_machine_status(self, machine_id: int) -> Optional[Dict[str, Any]]:
        """獲取OPUI機台停車格狀態 - 用於狀態查詢和驗證"""
        try:
            parking_info = self.db_client.get_machine_parking_info(machine_id)
            
            if parking_info:
                self.logger.debug(
                    f'📋 OPUI機台狀態查詢成功: machine_id={machine_id}, '\
                    f'space_1_status={parking_info["parking_spaces"]["space_1"]["status"]}, '\
                    f'space_2_status={parking_info["parking_spaces"]["space_2"]["status"]}'
                )
            else:
                self.logger.warning(f'📋 OPUI機台狀態查詢失敗: machine_id={machine_id}')
            
            return parking_info
            
        except Exception as e:
            self.logger.error(f'獲取OPUI機台狀態異常: {e}')
            return None
    
    def get_task_statistics(self) -> Dict[str, Any]:
        """獲取任務統計資料"""
        return {
            'stats': self.task_stats,
            'failed_creations': len(self.failed_creations),
            'created_tasks': len(self.created_tasks),
            'last_update': datetime.now(timezone.utc).isoformat()
        }
    
    def get_logger(self):
        """取得logger實例"""
        if self.logger:
            return self.logger
        # 建立預設logger
        import logging
        logger = logging.getLogger('unified_task_manager')
        return logger
    
    def destroy_node(self):
        """修正：添加destroy_node方法以符合ai_wcs_node調用"""
        if self.logger:
            self.logger.info('🔚 統一任務管理器正在關閉...')
        # 清理資源
        pass


def main():
    """主函數 - 直接連接模式測試"""
    def test_unified_task_manager():
        """測試統一任務管理器"""
        from .unified_decision_engine import TaskDecision
        
        manager = UnifiedTaskManager()
        
        # 創建測試決策
        test_decision = TaskDecision(
            work_id="220001",
            task_type="test_task",
            priority=80,
            source_location=91,
            target_location=76,
            room_id=1,
            reason="測試任務創建"
        )
        
        try:
            # 測試單一任務創建
            result = manager.create_task_from_decision(test_decision)
            print(f"任務創建結果: {result.success}, task_id: {result.task_id}")
            
            # 測試批次任務創建
            decisions = [test_decision]
            results = manager.create_tasks_from_decisions(decisions)
            print(f"批次任務創建完成: {len(results)} 個結果")
            
            # 獲取統計資料
            stats = manager.get_task_statistics()
            print(f"任務統計: {stats}")
            
        except Exception as e:
            print(f"測試異常: {e}")
    
    # 執行測試
    test_unified_task_manager()


if __name__ == '__main__':
    main()