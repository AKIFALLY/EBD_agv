"""
WCS 統一決策引擎
實現基於設計文檔的七大業務流程統一調度系統

七大業務流程優先度架構：
🔴 第1級：AGV旋轉檢查 (Priority: 100)
🟠 第2級：NG料架回收 (Priority: 90)
🟡 第3級：滿料架到人工收料區 (Priority: 80)
🟡 第4級：人工收料區搬運 (Priority: 80)
🟢 第5級：系統準備區到房間 (Priority: 60)
🔵 第6級：空料架搬運 (Priority: 40)
🔵 第7級：人工回收空料架 (Priority: 40)
"""

import logging
from typing import Dict, List, Optional, Any, Tuple, Union, ClassVar
from dataclasses import dataclass, field
from enum import Enum, IntEnum
import json
# import asyncio  # 移除異步依賴
from datetime import datetime, timezone, timedelta

from .enhanced_database_client import EnhancedDatabaseClient
from db_proxy.models.agvc_location import Location
from db_proxy.models.room import Room
from db_proxy.models.agvc_product import Product
from .rack_analyzer import RackAnalyzer, RackStatus, CarrierInfo


class BusinessFlowPriority(IntEnum):
    """業務流程優先度定義 - 基於設計文檔的完整七大流程"""
    AGV_ROTATION = 100                  # AGV旋轉檢查
    NG_RECYCLING = 90                   # NG料架回收
    MANUAL_TRANSPORT = 80               # 人工收料區相關
    FULL_RACK_TO_MANUAL = 80            # 滿料架到人工收料區
    SYSTEM_TO_ROOM = 60                 # 系統準備區到房間
    EMPTY_RACK_TRANSFER = 40            # 空料架搬運
    MANUAL_EMPTY_RECYCLING = 40         # 人工回收空料架
    EMPTY_OPERATIONS = 40               # 通用空料架操作
    OPUI_OPERATIONS = 40                # OPUI 叫車/派車操作


class WorkIDCategory(Enum):
    """Work ID 分類系統 - 基於實際資料庫定義"""
    MAIN_RACK_OPERATIONS = "220001"     # kuka-移動貨架 (大部分業務流程)
    WORKFLOW_OPERATIONS = "230001"      # kuka-流程觸發 (人工回收空料架專用)
    OPUI_CALL_EMPTY = "100001"          # opui-call-empty
    OPUI_DISPATCH_FULL = "100002"       # opui-dispatch-full
    CARGO_INLET = "2000102"             # CargoAGV放入口傳送箱
    CARGO_OUTLET = "2000201"            # CargoAGV拿出口傳送箱


@dataclass
class TaskDecision:
    """任務決策結果"""
    work_id: str
    task_type: str
    priority: int
    source_location: int
    target_location: int
    room_id: Optional[int] = None
    rack_id: Optional[int] = None
    agv_id: Optional[int] = None
    parent_task_id: Optional[int] = None
    nodes: List[int] = field(default_factory=list)
    parameters: Dict[str, Any] = field(default_factory=dict)
    reason: str = ""
    created_at: datetime = field(default_factory=lambda: datetime.now(timezone.utc))
    
    def to_task_dict(self) -> Dict[str, Any]:
        """轉換為 Task 表格式"""
        return {
            'work_id': self.work_id,
            'priority': self.priority,
            'room_id': self.room_id,
            'node_id': self.target_location,
            'name': f"{self.task_type}_{self.source_location}_to_{self.target_location}",
            'description': self.reason,
            'agv_id': self.agv_id,
            'parent_task_id': self.parent_task_id,
            'parameters': {
                **self.parameters,
                'function': self._get_function_by_work_id(),
                'model': 'KUKA400i',
                'api': 'submit_mission',
                'missionType': self._get_mission_type_by_work_id(),
                'nodes': self.nodes,
                'source_location': self.source_location,
                'target_location': self.target_location,
                'task_category': self.task_type
            }
        }
    
    def _get_function_by_work_id(self) -> str:
        """根據 work_id 獲取 function 參數"""
        work_id_functions = {
            "220001": "rack_move",      # kuka-移動貨架
            "230001": "workflow",       # kuka-流程觸發
            "100001": "rack_move",      # opui-call-empty
            "100002": "rack_move",      # opui-dispatch-full
            "2000102": "rack_move",     # CargoAGV入口
            "2000201": "rack_move",     # CargoAGV出口
        }
        return work_id_functions.get(self.work_id, "rack_move")
    
    def _get_mission_type_by_work_id(self) -> str:
        """根據 work_id 獲取 missionType 參數"""
        if self.work_id == "230001":
            return "MOVE"  # workflow 使用 MOVE
        return "RACK_MOVE"  # 其他都使用 RACK_MOVE


class UnifiedWCSDecisionEngine:
    """統一的WCS決策引擎 - 整合所有業務流程"""
    
    def __init__(self, logger=None):
        self.logger = logger or logging.getLogger('unified_decision_engine')
        self.logger.info('WCS 統一決策引擎啟動 - 直接連接模式')
        
        # 初始化組件
        self.db_client = EnhancedDatabaseClient(logger=self.logger)
        self.rack_analyzer = RackAnalyzer(logger)
        
        # Work ID 配置系統
        self.work_ids = {
            # 主要料架搬運作業 (大部分業務流程)
            'MAIN_RACK_OPERATIONS': '220001',    # kuka-移動貨架 (流程1,2,3,4,5,6)
            'WORKFLOW_OPERATIONS': '230001',     # kuka-流程觸發 (流程7專用)
            
            # OPUI操作員任務
            'OPUI_OPERATIONS': ['100001', '100002'],  # opui-call-empty, opui-dispatch-full
            
            # Cargo AGV專業任務
            'CARGO_OPERATIONS': ['2000102', '2000201']  # CargoAGV入口/出口傳送箱
        }
        
        # 條件檢查使用的Work ID映射
        self.condition_work_ids = {
            'agv_rotation': '220001',           # AGV旋轉 → kuka-移動貨架 (改用3節點移動)
            'ng_rack_recycling': '220001',       # NG料架回收 → kuka-移動貨架
            'full_rack_to_manual': '220001',     # 滿料架搬運 → kuka-移動貨架
            'manual_area_transport': '220001',   # 人工收料區搬運 → kuka-移動貨架
            'system_to_room': '220001',         # 系統準備區搬運 → kuka-移動貨架
            'empty_rack_transfer': '220001',    # 空料架搬運 → kuka-移動貨架
            'manual_empty_recycling': '230001', # 人工回收空料架 → kuka-流程觸發 ⭐唯一特殊
        }
        
        self.priority_levels = {
            'AGV_ROTATION': 100,        # AGV旋轉檢查
            'NG_RECYCLING': 90,         # NG料架回收
            'MANUAL_TRANSPORT': 80,     # 人工收料區相關
            'SYSTEM_TO_ROOM': 60,       # 系統準備區到房間
            'EMPTY_OPERATIONS': 40      # 空料架和人工回收
        }
        
        self.location_mappings = {
            'ng_recycling_area': [71, 72],          # NG回收區
            'manual_area': [51, 52, 53, 54, 55],    # 人工收料區
            'system_empty_area': [31, 32, 33, 34],  # 系統空架區
            'system_prep_area': [11, 12, 13, 14, 15, 16, 17, 18],  # 系統準備區
            'manual_empty_area': [91, 92],          # 人工回收空料架區
            'empty_recycling_area': [51, 52, 53, 54]  # 空料架回收區
        }
        
        # 系統狀態追蹤
        self.pending_decisions: List[TaskDecision] = []
        self.active_tasks: Dict[str, TaskDecision] = {}
        self.occupied_locations: set = set()
        
        # 統計資料
        self.decision_stats = {
            'agv_rotation_tasks': 0,
            'ng_recycling_tasks': 0,
            'manual_transport_tasks': 0,
            'system_to_room_tasks': 0,
            'empty_operations_tasks': 0,
            'total_decisions': 0,
            'cycles_completed': 0
        }
    
    def run_unified_decision_cycle(self) -> List[TaskDecision]:
        """執行統一決策週期 (修正：改為同步方法以符合ai_wcs_node調用)"""
        all_decisions = []
        
        if self.get_logger:
            self.get_logger().info('開始執行統一WCS決策週期')
        
        try:
            # 🔴 Priority 100: AGV旋轉檢查
            decisions = self.check_agv_rotation_flow()
            all_decisions.extend(decisions)
            self.decision_stats['agv_rotation_tasks'] += len(decisions)
            
            # 🟠 Priority 90: NG料架回收
            decisions = self.check_ng_rack_recycling_flow()
            all_decisions.extend(decisions)
            self.decision_stats['ng_recycling_tasks'] += len(decisions)
            
            # 🟡 Priority 80: 人工收料區相關流程
            decisions = self.check_full_rack_to_manual_flow()
            all_decisions.extend(decisions)
            
            decisions = self.check_manual_area_transport_flow()
            all_decisions.extend(decisions)
            self.decision_stats['manual_transport_tasks'] += len(decisions)
            
            # 🟢 Priority 60: 系統準備區到房間
            decisions = self.check_system_to_room_flow()
            all_decisions.extend(decisions)
            self.decision_stats['system_to_room_tasks'] += len(decisions)
            
            # 🔵 Priority 40: 空料架相關流程
            decisions = self.check_empty_rack_transfer_flow()
            all_decisions.extend(decisions)
            
            decisions = self.check_manual_empty_recycling_flow()
            all_decisions.extend(decisions)
            self.decision_stats['empty_operations_tasks'] += len(decisions)
            
            # 🔷 Priority 40: OPUI操作員請求處理
            decisions = self.check_opui_requests_flow()
            all_decisions.extend(decisions)
            self.decision_stats['opui_requests_tasks'] = self.decision_stats.get('opui_requests_tasks', 0) + len(decisions)
            
            # 依優先度排序並調度
            scheduled_decisions = self._prioritize_and_schedule(all_decisions)
            
            # 更新統計
            self.decision_stats['total_decisions'] += len(scheduled_decisions)
            self.decision_stats['cycles_completed'] += 1
            
            self.get_logger().info(
                f'統一決策週期完成: 產生 {len(scheduled_decisions)} 個任務決策'
            )
            
            return scheduled_decisions
            
        except Exception as e:
            self.get_logger().error(f'統一決策週期執行失敗: {e}')
            return []
    
    def check_agv_rotation_flow(self) -> List[TaskDecision]:
        """AGV旋轉狀態檢查 - 使用3節點移動方式"""
        decisions = []
        
        try:
            # 獲取等待旋轉狀態的AGV (需要實作AGV狀態查詢)
            waiting_agvs = self._get_agvs_by_state('wait_rotation_state')
            
            for agv_context in waiting_agvs:
                agv_tasks = self._get_tasks_by_agv(agv_context.get('agv_id'))
                
                for task in agv_tasks:
                    # 檢查是否無子任務 (防重複發送)
                    child_tasks = self._get_child_tasks(task.get('id'))
                    if not child_tasks:
                        # 檢查是否有重複的旋轉任務
                        duplicate_check = self._has_active_task('220001', task.get('node_id'))
                        if not duplicate_check:
                            # 創建使用 nodes 移動的旋轉任務
                            rotation_nodes = self._generate_rotation_nodes(
                                task.get('node_id'), 
                                agv_context.get('current_location')
                            )
                            
                            decision = TaskDecision(
                                work_id='220001',  # 改用 kuka-移動貨架
                                task_type='agv_rotation',
                                priority=self.priority_levels['AGV_ROTATION'],
                                source_location=agv_context.get('current_location'),
                                target_location=task.get('node_id'),
                                agv_id=agv_context.get('agv_id'),
                                parent_task_id=task.get('id'),
                                nodes=rotation_nodes,  # 3個節點的旋轉路徑
                                reason=f"AGV旋轉任務 - 3節點移動方式"
                            )
                            decisions.append(decision)
                            
            return decisions
            
        except Exception as e:
            self.get_logger().error(f'AGV旋轉檢查失敗: {e}')
            return []
    
    def _generate_rotation_nodes(self, target_location: int, current_location: int) -> List[int]:
        """生成AGV旋轉的3個節點路徑"""
        # 根據目標位置類型生成旋轉節點
        if self._is_room_inlet(target_location):
            # 入口旋轉：當前位置 → 旋轉中間點 → 旋轉完成位置
            return [current_location, self._get_rotation_intermediate_point(target_location), target_location]
        elif self._is_room_outlet(target_location):
            # 出口旋轉：當前位置 → 旋轉中間點 → 旋轉完成位置  
            return [current_location, self._get_rotation_intermediate_point(target_location), target_location]
        else:
            # 一般旋轉
            return [current_location, target_location, target_location]
    
    def _is_room_inlet(self, location_id: int) -> bool:
        """判斷是否為房間入口位置"""
        # 房間入口位置格式：X0001 (room_id * 10000 + 1)
        return location_id % 10000 == 1 and location_id >= 10001
    
    def _is_room_outlet(self, location_id: int) -> bool:
        """判斷是否為房間出口位置"""
        # 房間出口位置格式：X0002 (room_id * 10000 + 2)
        return location_id % 10000 == 2 and location_id >= 10002
    
    def _get_rotation_intermediate_point(self, location_id: int) -> int:
        """獲取旋轉中間點位置"""
        # 根據實際地圖配置返回旋轉中間點
        room_id = location_id // 10000
        if self._is_room_inlet(location_id):
            return room_id * 10000 + 10  # 入口旋轉中間點
        elif self._is_room_outlet(location_id):
            return room_id * 10000 + 20  # 出口旋轉中間點
        return location_id + 1000  # 預設中間點
    
    def check_ng_rack_recycling_flow(self) -> List[TaskDecision]:
        """NG料架回收 - 三階段條件檢查 (房間擴展支援)"""
        decisions = []
        
        try:
            # 條件 6: 檢查NG回收區是否有空位
            ng_space_available = self._check_locations_available([71, 72], status=2)
            if not ng_space_available:
                return decisions  # NG回收區無空位
            
            ng_target_location = ng_space_available[0]['id']
            
            # 遍歷所有房間 (支援房間1-10擴展)
            for room_id in range(1, 11):
                inlet_location = room_id * 10000 + 1  # 房間入口位置
                
                # 條件 X20: 檢查房間X入口傳送箱NG料架
                has_ng_rack = self._check_ng_rack_at_location(inlet_location)
                if not has_ng_rack:
                    continue  # 該房間無NG料架
                
                # 條件 X21: 檢查是否有重複執行任務
                has_duplicate = self._has_active_task('220001', inlet_location)
                if not has_duplicate:
                    # 該房間條件滿足，創建NG料架回收任務
                    decision = TaskDecision(
                        work_id='220001',  # 使用 kuka-移動貨架
                        task_type='ng_rack_recycling',
                        priority=self.priority_levels['NG_RECYCLING'],
                        source_location=inlet_location,
                        target_location=ng_target_location,
                        room_id=room_id,
                        reason=f"NG料架回收：房間{room_id}入口 → NG回收區"
                    )
                    decisions.append(decision)
                    
                    # NG回收區位置已被分配，不再處理其他房間
                    break
            
            return decisions
            
        except Exception as e:
            self.get_logger().error(f'NG料架回收檢查失敗: {e}')
            return []
    
    def check_full_rack_to_manual_flow(self) -> List[TaskDecision]:
        """滿料架到人工收料區"""
        decisions = []
        
        try:
            # 檢查系統空架區空料架
            empty_locations = self._check_locations_available([31, 32, 33, 34], status=3)
            if not empty_locations:
                return decisions
            
            for room_id in range(1, 11):
                # 檢查房間內是否有carrier需要搬運
                has_carriers = self._check_carriers_in_room(room_id)
                if has_carriers:
                    outlet_location = room_id * 10000 + 2
                    has_conflict = self._has_active_task('220001', outlet_location)
                    if not has_conflict:
                        decision = TaskDecision(
                            work_id='220001',
                            task_type='full_rack_to_manual',
                            priority=self.priority_levels['MANUAL_TRANSPORT'],
                            source_location=outlet_location,
                            target_location=empty_locations[0]['id'],
                            room_id=room_id,
                            reason=f"滿料架搬運：房間{room_id}出口 → 人工收料區"
                        )
                        decisions.append(decision)
                        break  # 一次只處理一個
            
            return decisions
            
        except Exception as e:
            self.get_logger().error(f'滿料架搬運檢查失敗: {e}')
            return []
    
    def check_manual_area_transport_flow(self) -> List[TaskDecision]:
        """人工收料區搬運"""
        decisions = []
        
        try:
            # 檢查人工收料區是否有空位
            manual_spaces = self._check_locations_available([51, 52, 53, 54, 55], status=2)
            if not manual_spaces:
                return decisions
            
            for room_id in range(1, 11):
                outlet_location = room_id * 10000 + 2
                
                # 檢查房間出口是否有滿料架
                has_full_racks = self._check_racks_at_location(outlet_location, status=[2, 3, 6])
                
                if has_full_racks:
                    # 有滿料架，檢查重複任務
                    has_conflict = self._has_active_task('220001', outlet_location)
                    if not has_conflict:
                        decision = TaskDecision(
                            work_id='220001',
                            task_type='manual_area_transport',
                            priority=self.priority_levels['MANUAL_TRANSPORT'],
                            source_location=outlet_location,
                            target_location=manual_spaces[0]['id'],
                            room_id=room_id,
                            rack_id=has_full_racks[0]['id'],
                            reason=f"人工收料區搬運：房間{room_id}滿料架 → 人工收料區"
                        )
                        decisions.append(decision)
                else:
                    # 無滿料架，檢查cargo任務
                    cargo_work_id = room_id * 1000000 + 201
                    has_completed_cargo = self._has_completed_task(cargo_work_id)
                    has_conflict = self._has_active_task('220001', outlet_location)
                    
                    if has_completed_cargo and not has_conflict:
                        decision = TaskDecision(
                            work_id='220001',
                            task_type='cargo_followup_transport',
                            priority=self.priority_levels['MANUAL_TRANSPORT'],
                            source_location=outlet_location,
                            target_location=manual_spaces[0]['id'],
                            room_id=room_id,
                            reason=f"Cargo後續搬運：房間{room_id}出口 → 人工收料區"
                        )
                        decisions.append(decision)
            
            return decisions
            
        except Exception as e:
            self.get_logger().error(f'人工收料區搬運檢查失敗: {e}')
            return []
    
    def check_system_to_room_flow(self) -> List[TaskDecision]:
        """系統準備區到房間入口"""
        decisions = []
        
        try:
            # 檢查系統準備區是否有料架
            system_racks = self._check_locations_available([11, 12, 13, 14, 15, 16, 17, 18], status=3)
            if not system_racks:
                return decisions
            
            for room_id in range(1, 11):
                inlet_location = room_id * 10000 + 1
                
                # 檢查房間入口是否無料架佔用
                is_occupied = self._check_racks_at_location(inlet_location)
                has_conflict = self._has_active_task('220001', inlet_location)
                
                if not is_occupied and not has_conflict:
                    decision = TaskDecision(
                        work_id='220001',
                        task_type='system_to_room',
                        priority=self.priority_levels['SYSTEM_TO_ROOM'],
                        source_location=system_racks[0]['id'],
                        target_location=inlet_location,
                        room_id=room_id,
                        reason=f"系統準備區搬運：準備區 → 房間{room_id}入口"
                    )
                    decisions.append(decision)
                    break  # 一次只處理一個
            
            return decisions
            
        except Exception as e:
            self.get_logger().error(f'系統準備區搬運檢查失敗: {e}')
            return []
    
    def check_empty_rack_transfer_flow(self) -> List[TaskDecision]:
        """空料架搬運"""
        decisions = []
        
        try:
            for room_id in range(1, 11):
                inlet_location = room_id * 10000 + 1
                outlet_location = room_id * 10000 + 2
                
                # 檢查房間入口是否有空料架
                empty_racks = self._check_racks_at_location(inlet_location, status=[1])
                
                if empty_racks:
                    # 檢查房間出口是否無料架佔用
                    outlet_occupied = self._check_racks_at_location(outlet_location)
                    has_conflict = self._has_active_task('220001', outlet_location)
                    
                    if not outlet_occupied and not has_conflict:
                        decision = TaskDecision(
                            work_id='220001',
                            task_type='empty_rack_transfer',
                            priority=self.priority_levels['EMPTY_OPERATIONS'],
                            source_location=inlet_location,
                            target_location=outlet_location,
                            room_id=room_id,
                            rack_id=empty_racks[0]['id'],
                            reason=f"空料架搬運：房間{room_id}入口 → 出口"
                        )
                        decisions.append(decision)
            
            return decisions
            
        except Exception as e:
            self.get_logger().error(f'空料架搬運檢查失敗: {e}')
            return []
    
    def check_manual_empty_recycling_flow(self) -> List[TaskDecision]:
        """人工回收空料架搬運 - 三階段條件檢查"""
        decisions = []
        
        try:
            # 條件 7: 檢查人工回收空料架區是否有料架
            manual_empty_racks = self._check_locations_available([91, 92], status=3)
            if not manual_empty_racks:
                return decisions  # 無空料架需回收
            
            # 條件 8: 檢查空料架回收區是否有空位  
            empty_spaces = self._check_locations_available([51, 52, 53, 54], status=2)
            if not empty_spaces:
                return decisions  # 回收區無空位
            
            # 條件 9: 檢查是否有重複執行任務 (特殊work_id='230001')
            has_duplicate = self._has_active_task_by_work_id('230001', status_list=[0, 1, 2])
            if not has_duplicate:
                # 三個條件都滿足，創建人工回收空料架任務
                decision = TaskDecision(
                    work_id='230001',  # 使用 kuka-流程觸發
                    task_type='manual_empty_recycling',
                    priority=self.priority_levels['EMPTY_OPERATIONS'],
                    source_location=manual_empty_racks[0]['id'],
                    target_location=empty_spaces[0]['id'],
                    reason="人工回收空料架：人工回收區 → 空料架回收區",
                    parameters={
                        'templateCode': 'W000000001'  # workflow 模板代碼
                    }
                )
                decisions.append(decision)
            
            return decisions
            
        except Exception as e:
            self.get_logger().error(f'人工回收空料架檢查失敗: {e}')
            return []
    
    def check_opui_requests_flow(self) -> List[TaskDecision]:
        """OPUI操作員請求處理 - 叫空車和派滿車"""
        decisions = []
        
        try:
            # 獲取OPUI待處理請求
            opui_requests = self._get_opui_pending_requests()
            
            for request in opui_requests:
                # 處理OPUI叫空車請求 (work_id: 100001)
                if request.get('work_id') == '100001':
                    decision = self._process_opui_call_empty_request(request)
                    if decision:
                        decisions.append(decision)
                
                # 處理OPUI派滿車請求 (work_id: 100002)
                elif request.get('work_id') == '100002':
                    decision = self._process_opui_dispatch_full_request(request)
                    if decision:
                        decisions.append(decision)
            
            return decisions
            
        except Exception as e:
            self.get_logger().error(f'OPUI請求處理檢查失敗: {e}')
            return []
    
    def _process_opui_call_empty_request(self, request: Dict[str, Any]) -> Optional[TaskDecision]:
        """處理OPUI叫空車請求"""
        try:
            machine_id = request.get('machine_id')
            parameters = request.get('parameters', {})
            space_num = parameters.get('space_num', 1)
            
            if not machine_id:
                return None
            
            # 檢查停車格狀態是否允許叫車
            machine_info = self._get_machine_parking_info(machine_id)
            if not machine_info:
                return None
            
            space_key = f'space_{space_num}'
            parking_space = machine_info['parking_spaces'].get(space_key)
            
            if not parking_space or not parking_space['is_available']:
                self.get_logger().debug(f'停車格不可用: machine_id={machine_id}, space={space_num}')
                return None
            
            # 獲取目標node_id
            target_node_id = parking_space['node_id']
            
            # 找到可用的空料架
            empty_rack_location = self._find_available_empty_rack()
            if not empty_rack_location:
                self.get_logger().debug('無可用空料架，無法處理叫空車請求')
                return None
            
            # 創建叫空車決策
            decision = TaskDecision(
                work_id='100001',
                task_type='opui_call_empty',
                priority=self.priority_levels['EMPTY_OPERATIONS'],
                source_location=empty_rack_location,
                target_location=target_node_id,
                parameters={
                    'machine_id': machine_id,
                    'space_num': space_num,
                    'task_type': 'call_empty',
                    'client_id': parameters.get('client_id', 'clientId')
                },
                reason=f"OPUI叫空車：machine_id={machine_id}, space_{space_num}"
            )
            
            return decision
            
        except Exception as e:
            self.get_logger().error(f'處理OPUI叫空車請求異常: {e}')
            return None
    
    def _process_opui_dispatch_full_request(self, request: Dict[str, Any]) -> Optional[TaskDecision]:
        """處理OPUI派滿車請求"""
        try:
            machine_id = request.get('machine_id')
            parameters = request.get('parameters', {})
            rack_id = parameters.get('rack_id')
            room_id = parameters.get('room_id')
            
            if not all([machine_id, rack_id, room_id]):
                return None
            
            # 檢查系統準備派車區是否有空位
            prep_area_spaces = self._check_locations_available([11, 12, 13, 14, 15, 16, 17, 18], status=2)
            if not prep_area_spaces:
                self.get_logger().debug('系統準備派車區無空位，無法處理派滿車請求')
                return None
            
            # 獲取機台停車格位置作為起始位置
            machine_info = self._get_machine_parking_info(machine_id)
            if not machine_info:
                return None
            
            # 假設使用第一個停車格作為起始位置
            space_1_node = machine_info['parking_spaces']['space_1']['node_id']
            
            # 創建派滿車決策
            decision = TaskDecision(
                work_id='100002',
                task_type='opui_dispatch_full',
                priority=self.priority_levels['EMPTY_OPERATIONS'],
                source_location=space_1_node,
                target_location=prep_area_spaces[0]['id'],
                rack_id=rack_id,
                room_id=room_id,
                parameters={
                    'machine_id': machine_id,
                    'rack_id': rack_id,
                    'room_id': room_id,
                    'task_type': 'dispatch_full',
                    'client_id': parameters.get('client_id', 'clientId'),
                    'product_name': parameters.get('product_name', 'ABC121345'),
                    'count': parameters.get('count', 32)
                },
                reason=f"OPUI派滿車：machine_id={machine_id}, rack_id={rack_id} → 系統準備派車區"
            )
            
            return decision
            
        except Exception as e:
            self.get_logger().error(f'處理OPUI派滿車請求異常: {e}')
            return None
    
    def _prioritize_and_schedule(self, decisions: List[TaskDecision]) -> List[TaskDecision]:
        """優先度排序和調度衝突解決"""
        # 按優先度排序
        decisions.sort(key=lambda d: d.priority, reverse=True)
        
        # 解決資源衝突
        scheduled = []
        occupied_locations = set()
        
        for decision in decisions:
            if decision.target_location not in occupied_locations:
                scheduled.append(decision)
                occupied_locations.add(decision.target_location)
                self.get_logger().info(
                    f'調度任務: {decision.task_type} (優先級: {decision.priority}) '
                    f'{decision.source_location} → {decision.target_location}'
                )
        
        return scheduled
    
    def get_room_location_info(self, room_id: int) -> Dict[str, int]:
        """取得房間位置資訊"""
        return {
            'inlet_location': room_id * 10000 + 1,    # 房間入口
            'outlet_location': room_id * 10000 + 2,   # 房間出口
            'cargo_work_id': room_id * 1000000 + 201, # Cargo任務ID
            'node_prefix': room_id * 10000             # 節點ID前綴
        }
    
    # === 資料庫查詢輔助方法 (待實作) ===
    
    def _get_agvs_by_state(self, state: str) -> List[Dict[str, Any]]:
        """獲取特定狀態的AGV"""
        try:
            agv_infos = self.db_client.get_agvs_by_state(state)
            
            # 轉換為字典格式
            agvs = []
            for agv_info in agv_infos:
                agv_dict = {
                    'id': agv_info.id,
                    'name': agv_info.name,
                    'state': agv_info.state,
                    'current_location': agv_info.current_location,
                    'is_available': agv_info.is_available
                }
                agvs.append(agv_dict)
            
            return agvs
            
        except Exception as e:
            self.get_logger().error(f'獲取AGV狀態失敗: state={state}, error={e}')
            return []
    
    def _get_tasks_by_agv(self, agv_id: int) -> List[Dict[str, Any]]:
        """獲取AGV的任務列表"""
        try:
            task_infos = self.db_client.get_tasks_by_agv(agv_id)
            
            # 轉換為字典格式
            tasks = []
            for task_info in task_infos:
                task_dict = {
                    'id': task_info.id,
                    'work_id': task_info.work_id,
                    'status_id': task_info.status_id,
                    'room_id': task_info.room_id,
                    'node_id': task_info.node_id,
                    'name': task_info.name,
                    'description': task_info.description,
                    'agv_id': task_info.agv_id,
                    'parent_task_id': task_info.parent_task_id,
                    'priority': task_info.priority,
                    'parameters': task_info.parameters,
                    'created_at': task_info.created_at
                }
                tasks.append(task_dict)
            
            return tasks
            
        except Exception as e:
            self.get_logger().error(f'獲取AGV任務失敗: agv_id={agv_id}, error={e}')
            return []
    
    def _get_child_tasks(self, task_id: int) -> List[Dict[str, Any]]:
        """獲取子任務列表"""
        try:
            task_infos = self.db_client.get_child_tasks(task_id)
            
            # 轉換為字典格式
            tasks = []
            for task_info in task_infos:
                task_dict = {
                    'id': task_info.id,
                    'work_id': task_info.work_id,
                    'status_id': task_info.status_id,
                    'room_id': task_info.room_id,
                    'node_id': task_info.node_id,
                    'name': task_info.name,
                    'description': task_info.description,
                    'agv_id': task_info.agv_id,
                    'parent_task_id': task_info.parent_task_id,
                    'priority': task_info.priority,
                    'parameters': task_info.parameters,
                    'created_at': task_info.created_at
                }
                tasks.append(task_dict)
            
            return tasks
            
        except Exception as e:
            self.get_logger().error(f'獲取子任務失敗: task_id={task_id}, error={e}')
            return []
    
    def _has_active_task(self, work_id: str, location_id: int) -> bool:
        """檢查是否有重複的活動任務"""
        try:
            has_task = self.db_client.has_active_task(work_id, location_id)
            return has_task
            
        except Exception as e:
            self.get_logger().error(f'檢查活動任務失敗: work_id={work_id}, location_id={location_id}, error={e}')
            return False
    
    def _has_active_task_by_work_id(self, work_id: str, status_list: List[int]) -> bool:
        """檢查特定work_id是否有活動任務"""
        try:
            has_task = self.db_client.has_active_task_by_work_id(work_id, status_list)
            return has_task
            
        except Exception as e:
            self.get_logger().error(f'檢查work_id任務失敗: work_id={work_id}, status_list={status_list}, error={e}')
            return False
    
    def _has_completed_task(self, work_id: Union[str, int]) -> bool:
        """檢查是否有已完成的任務"""
        try:
            has_task = self.db_client.has_completed_task(work_id)
            return has_task
            
        except Exception as e:
            self.get_logger().error(f'檢查完成任務失敗: work_id={work_id}, error={e}')
            return False
    
    def _check_locations_available(self, location_ids: List[int], status: int) -> List[Dict[str, Any]]:
        """檢查位置可用性"""
        try:
            locations = self.db_client.check_locations_available(location_ids, status)
            return locations
            
        except Exception as e:
            self.get_logger().error(f'檢查位置可用性失敗: location_ids={location_ids}, status={status}, error={e}')
            return []
    
    def _check_ng_rack_at_location(self, location_id: int) -> bool:
        """檢查位置是否有NG料架"""
        try:
            has_ng_rack = self.db_client.check_ng_rack_at_location(location_id)
            return has_ng_rack
            
        except Exception as e:
            self.get_logger().error(f'檢查NG料架失敗: location_id={location_id}, error={e}')
            return False
    
    def _check_carriers_in_room(self, room_id: int) -> bool:
        """檢查房間是否有carrier"""
        try:
            has_carriers = self.db_client.check_carriers_in_room(room_id)
            return has_carriers
            
        except Exception as e:
            self.get_logger().error(f'檢查房間carrier失敗: room_id={room_id}, error={e}')
            return False
    
    def _check_racks_at_location(self, location_id: int, status: List[int] = None) -> List[Dict[str, Any]]:
        """檢查位置的料架狀態"""
        try:
            racks = self.db_client.check_racks_at_location(location_id, status)
            return racks
            
        except Exception as e:
            self.get_logger().error(f'檢查料架狀態失敗: location_id={location_id}, status={status}, error={e}')
            return []
    
    def _get_opui_pending_requests(self) -> List[Dict[str, Any]]:
        """獲取OPUI待處理請求 - 基於machine parking space狀態和待處理任務"""
        try:
            # 使用增強資料庫客戶端獲取OPUI請求
            requests = self.db_client.get_opui_pending_requests()
            
            self.get_logger().debug(f'查詢OPUI待處理請求: {len(requests)}個')
            
            # 格式化為統一格式
            formatted_requests = []
            for request in requests:
                if request.get('task_id'):
                    # 有實際任務的請求
                    formatted_request = {
                        'machine_id': request['machine_id'],
                        'machine_name': request['machine_name'],
                        'task_id': request['task_id'],
                        'work_id': request['work_id'],
                        'parameters': request.get('parameters', {}),
                        'created_at': request.get('created_at'),
                        'type': 'existing_task'
                    }
                    formatted_requests.append(formatted_request)
                else:
                    # 基於停車格狀態的潛在請求
                    space_1_status = request.get('parking_space_1_status', 0)
                    space_2_status = request.get('parking_space_2_status', 0)
                    
                    if space_1_status != 0 or space_2_status != 0:
                        formatted_request = {
                            'machine_id': request['machine_id'],
                            'machine_name': request['machine_name'],
                            'parking_space_1_status': space_1_status,
                            'parking_space_2_status': space_2_status,
                            'type': 'status_based'
                        }
                        formatted_requests.append(formatted_request)
            
            return formatted_requests
            
        except Exception as e:
            self.get_logger().error(f'獲取OPUI待處理請求失敗: {e}')
            return []
    
    def _get_machine_parking_info(self, machine_id: int) -> Optional[Dict[str, Any]]:
        """獲取機台停車格資訊 - 用於OPUI叫空車和派滿車邏輯"""
        try:
            # 使用增強資料庫客戶端獲取機台停車格資訊
            parking_info = self.db_client.get_machine_parking_info(machine_id)
            
            if parking_info:
                self.get_logger().debug(
                    f'機台停車格資訊: machine_id={machine_id}, '
                    f'space_1_available={parking_info["parking_spaces"]["space_1"]["is_available"]}, '
                    f'space_2_available={parking_info["parking_spaces"]["space_2"]["is_available"]}'
                )
            else:
                self.get_logger().warning(f'未找到機台停車格資訊: machine_id={machine_id}')
            
            return parking_info
            
        except Exception as e:
            self.get_logger().error(f'獲取機台停車格資訊失敗: machine_id={machine_id}, error={e}')
            return None
    
    def _find_available_empty_rack(self) -> Optional[int]:
        """尋找可用的空料架位置"""
        # 檢查系統空架區是否有可用的空料架
        empty_racks = self._check_locations_available([31, 32, 33, 34], status=3)
        if empty_racks:
            return empty_racks[0]['id']
        return None
    
    def get_decision_statistics(self) -> Dict[str, Any]:
        """獲取決策統計資料"""
        return {
            'stats': self.decision_stats,
            'pending_decisions': len(self.pending_decisions),
            'active_tasks': len(self.active_tasks),
            'occupied_locations': len(self.occupied_locations),
            'work_id_mappings': self.work_ids,
            'last_update': datetime.now(timezone.utc).isoformat()
        }
    
    def destroy_node(self):
        """修正：添加destroy_node方法以符合ai_wcs_node調用"""
        if self.get_logger:
            self.get_logger().info('🔚 WCS統一決策引擎正在關閉...')
        # 清理資源
        pass
    
    def get_logger(self):
        """取得logger實例"""
        if self.logger:
            return self.logger
        # 建立預設logger
        import logging
        logger = logging.getLogger('unified_wcs_engine')
        return logger


def main(args=None):
    """主函數 - 獨立運行時使用"""
    import rclpy
    from rclpy.node import Node
    
    class StandaloneUnifiedEngine(Node):
        def __init__(self):
            super().__init__('unified_wcs_decision_engine')
            self.engine = UnifiedWCSDecisionEngine(self.get_logger())
            
            # 創建定時器執行決策週期
            self.timer = self.create_timer(10.0, self.run_decision_cycle)
            
        def run_decision_cycle(self):
            """執行決策週期"""
            try:
                # 直接同步執行
                decisions = self.engine.run_unified_decision_cycle()
                
                self.get_logger().info(f'決策週期完成，產生 {len(decisions)} 個任務')
                
            except Exception as e:
                self.get_logger().error(f'決策週期執行錯誤: {e}')
                
    rclpy.init(args=args)
    node = StandaloneUnifiedEngine()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()