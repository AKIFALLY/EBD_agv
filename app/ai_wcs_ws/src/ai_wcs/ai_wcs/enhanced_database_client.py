"""
增強的資料庫查詢客戶端
專為統一決策引擎設計，實現批次查詢最佳化和完整業務流程支援
直接使用 db_proxy 連接池，無 ROS 2 依賴
"""

import os
import logging
from typing import Dict, List, Optional, Any, Tuple, Union
from dataclasses import dataclass
import json
# import asyncio  # 移除異步依賴
from datetime import datetime, timezone, timedelta

# 直接導入 db_proxy 組件
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.crud.task_crud import task_crud
from db_proxy.crud.agv_crud import agv_crud
from db_proxy.crud.rack_crud import rack_crud
from db_proxy.crud.location_crud import location_crud
from db_proxy.crud.machine_crud import machine_crud
from db_proxy.crud.carrier_crud import carrier_crud
from db_proxy.models.agvc_task import Task
from db_proxy.models.agvc_rcs import AGV
from db_proxy.models.rack import Rack as RackModel
from db_proxy.models.agvc_location import Location
from db_proxy.models.machine import Machine
from db_proxy.models.carrier import Carrier as CarrierModel
from sqlmodel import select, and_, or_, func
from sqlalchemy import cast, Integer

from .rack_analyzer import CarrierInfo


@dataclass
class BatchQueryResult:
    """批次查詢結果"""
    success: bool
    data: Dict[str, Any]
    error_message: str = ""
    query_count: int = 0
    execution_time: float = 0.0


@dataclass
class TaskInfo:
    """任務資訊"""
    id: int
    work_id: str
    status_id: int
    room_id: Optional[int]
    node_id: Optional[int]
    name: str
    description: Optional[str]
    agv_id: Optional[int]
    parent_task_id: Optional[int]
    priority: int
    parameters: Dict[str, Any]
    created_at: datetime


@dataclass
class AGVInfo:
    """AGV資訊"""
    id: int
    name: str
    state: str
    current_location: Optional[int]
    is_available: bool


class EnhancedDatabaseClient:
    """增強的資料庫查詢客戶端 - 直接使用 db_proxy 連接池"""
    
    def __init__(self, logger=None):
        self.logger = logger or logging.getLogger('enhanced_database_client')
        self.logger.info('增強版 AI WCS 資料庫客戶端啟動 - 直接連接模式')
        
        # 初始化資料庫連接池
        db_url = self._build_database_url()
        self.connection_pool = ConnectionPoolManager(db_url)
        
        # 統計資料
        self.query_stats = {
            'total_queries': 0,
            'batch_queries': 0,
            'cache_hits': 0,
            'cache_misses': 0,
            'avg_query_time': 0.0
        }
        
        # 批次查詢快取 - 同步優化
        self.batch_cache = {}
        self.batch_cache_ttl = 25  # 25秒快取 - 適配同步執行
        
        # 一般查詢快取 - 同步優化  
        self.cache = {}
        self.cache_ttl = 45  # 45秒快取 - 平衡性能與實時性
    
    def _build_database_url(self) -> str:
        """建構資料庫連接 URL - 使用正確的AGVC資料庫設定"""
        host = os.getenv('DB_HOST', 'postgres')
        port = os.getenv('DB_PORT', '5432')
        database = os.getenv('DB_NAME', 'agvc')
        username = os.getenv('DB_USER', 'agvc')
        password = os.getenv('DB_PASSWORD', 'password')
        
        db_url = f"postgresql://{username}:{password}@{host}:{port}/{database}"
        self.logger.info(f'資料庫連接: {host}:{port}/{database}')
        return db_url
    
    def batch_check_locations_status(self, 
                                    location_groups: Dict[str, List[int]], 
                                    status_filter: int) -> BatchQueryResult:
        """批次檢查多組位置狀態"""
        start_time = datetime.now()
        
        try:
            results = {}
            
            with self.connection_pool.get_session() as session:
                for group_name, location_ids in location_groups.items():
                    # 使用 SQLModel 查詢
                    stmt = select(Location).where(
                        and_(
                            Location.id.in_(location_ids),
                            Location.location_status_id == status_filter
                        )
                    )
                    
                    locations = session.exec(stmt).all()
                    
                    if locations:
                        results[group_name] = {
                            'available': True,
                            'location_id': locations[0].id,
                            'locations': [
                                {
                                    'id': loc.id,
                                    'name': loc.name,
                                    'status_id': loc.location_status_id
                                } for loc in locations
                            ]
                        }
                    else:
                        results[group_name] = {
                            'available': False,
                            'location_id': None,
                            'locations': []
                        }
            
            execution_time = (datetime.now() - start_time).total_seconds()
            self.query_stats['total_queries'] += 1
            self.query_stats['batch_queries'] += 1
            
            return BatchQueryResult(
                success=True,
                data=results,
                query_count=len(location_groups),
                execution_time=execution_time
            )
            
        except Exception as e:
            error_msg = f"批次位置狀態查詢失敗: {e}"
            self.logger.error(error_msg)
            return BatchQueryResult(
                success=False,
                data={},
                error_message=error_msg,
                query_count=1
            )
    
    def batch_check_task_conflicts(self, 
                                  work_location_pairs: List[Tuple[str, int]]) -> BatchQueryResult:
        """批次檢查任務衝突"""
        start_time = datetime.now()
        
        try:
            results = {}
            
            with self.connection_pool.get_session() as session:
                for work_id, location_id in work_location_pairs:
                    check_key = f'{work_id}_{location_id}'
                    
                    # 使用 SQLModel 查詢衝突任務
                    stmt = select(func.count(Task.id)).where(
                        and_(
                            Task.work_id == work_id,
                            or_(
                                Task.node_id == location_id,
                                Task.status_id.in_([0, 1, 2])
                            )
                        )
                    )
                    
                    conflict_count = session.exec(stmt).one()
                    
                    results[check_key] = {
                        'has_conflict': conflict_count > 0,
                        'no_conflict': conflict_count == 0,
                        'conflict_count': conflict_count
                    }
            
            execution_time = (datetime.now() - start_time).total_seconds()
            self.query_stats['total_queries'] += 1
            self.query_stats['batch_queries'] += 1
            
            return BatchQueryResult(
                success=True,
                data=results,
                query_count=len(work_location_pairs),
                execution_time=execution_time
            )
            
        except Exception as e:
            error_msg = f"批次任務衝突檢查失敗: {e}"
            self.logger.error(error_msg)
            return BatchQueryResult(
                success=False,
                data={},
                error_message=error_msg
            )
    
    def get_agvs_by_state(self, state: str) -> List[AGVInfo]:
        """獲取特定狀態的AGV"""
        try:
            with self.connection_pool.get_session() as session:
                stmt = select(AGV).where(
                    and_(
                        AGV.state == state,
                        AGV.enable == 1
                    )
                )
                
                agv_models = session.exec(stmt).all()
                
                agvs = []
                for agv_model in agv_models:
                    agv = AGVInfo(
                        id=agv_model.id,
                        name=agv_model.name,
                        state=agv_model.state,
                        current_location=agv_model.current_location,
                        is_available=getattr(agv_model, 'is_available', True)
                    )
                    agvs.append(agv)
                
                return agvs
            
        except Exception as e:
            self.logger.error(f'查詢AGV狀態異常: {e}')
            return []
    
    def get_tasks_by_agv(self, agv_id: int) -> List[TaskInfo]:
        """獲取AGV的任務列表"""
        try:
            with self.connection_pool.get_session() as session:
                stmt = select(Task).where(
                    and_(
                        Task.agv_id == agv_id,
                        Task.status_id.in_([0, 1, 2])
                    )
                ).order_by(Task.priority.desc(), Task.created_at.asc())
                
                task_models = session.exec(stmt).all()
                
                tasks = []
                for task_model in task_models:
                    task = TaskInfo(
                        id=task_model.id,
                        work_id=task_model.work_id,
                        status_id=task_model.status_id,
                        room_id=task_model.room_id,
                        node_id=task_model.node_id,
                        name=task_model.name,
                        description=task_model.description,
                        agv_id=task_model.agv_id,
                        parent_task_id=task_model.parent_task_id,
                        priority=task_model.priority,
                        parameters=task_model.parameters or {},
                        created_at=task_model.created_at
                    )
                    tasks.append(task)
                
                return tasks
            
        except Exception as e:
            self.logger.error(f'查詢AGV任務異常: {e}')
            return []
    
    def get_child_tasks(self, parent_task_id: int) -> List[TaskInfo]:
        """獲取子任務列表"""
        try:
            with self.connection_pool.get_session() as session:
                stmt = select(Task).where(
                    Task.parent_task_id == parent_task_id
                ).order_by(Task.created_at.asc())
                
                task_models = session.exec(stmt).all()
                
                tasks = []
                for task_model in task_models:
                    task = TaskInfo(
                        id=task_model.id,
                        work_id=task_model.work_id,
                        status_id=task_model.status_id,
                        room_id=task_model.room_id,
                        node_id=task_model.node_id,
                        name=task_model.name,
                        description=task_model.description,
                        agv_id=task_model.agv_id,
                        parent_task_id=task_model.parent_task_id,
                        priority=task_model.priority,
                        parameters=task_model.parameters or {},
                        created_at=task_model.created_at
                    )
                    tasks.append(task)
                
                return tasks
            
        except Exception as e:
            self.logger.error(f'查詢子任務異常: {e}')
            return []
    
    def has_active_task(self, work_id: str, location_id: int) -> bool:
        """檢查是否有重複的活動任務"""
        try:
            with self.connection_pool.get_session() as session:
                stmt = select(func.count(Task.id)).where(
                    and_(
                        Task.work_id == work_id,
                        Task.node_id == location_id,
                        Task.status_id.in_([0, 1, 2])
                    )
                )
                
                count = session.exec(stmt).one()
                return count > 0
            
        except Exception as e:
            self.logger.error(f'檢查活動任務異常: {e}')
            return False
    
    def has_active_task_by_work_id(self, work_id: str, status_list: List[int]) -> bool:
        """檢查特定work_id是否有活動任務"""
        try:
            with self.connection_pool.get_session() as session:
                stmt = select(func.count(Task.id)).where(
                    and_(
                        Task.work_id == work_id,
                        Task.status_id.in_(status_list)
                    )
                )
                
                count = session.exec(stmt).one()
                return count > 0
            
        except Exception as e:
            self.logger.error(f'檢查work_id任務異常: {e}')
            return False
    
    def has_completed_task(self, work_id: Union[str, int]) -> bool:
        """檢查是否有已完成的任務"""
        try:
            with self.connection_pool.get_session() as session:
                stmt = select(func.count(Task.id)).where(
                    and_(
                        Task.work_id == str(work_id),
                        Task.status_id == 4
                    )
                )
                
                count = session.exec(stmt).one()
                return count > 0
            
        except Exception as e:
            self.logger.error(f'檢查完成任務異常: {e}')
            return False
    
    def check_locations_available(self, location_ids: List[int], status: int) -> List[Dict[str, Any]]:
        """檢查位置可用性"""
        try:
            with self.connection_pool.get_session() as session:
                stmt = select(Location).where(
                    and_(
                        Location.id.in_(location_ids),
                        Location.location_status_id == status
                    )
                ).order_by(Location.id)
                
                location_models = session.exec(stmt).all()
                
                locations = []
                for loc in location_models:
                    location_data = {
                        'id': loc.id,
                        'name': loc.name,
                        'node_id': loc.node_id,
                        'room_id': loc.room_id,
                        'location_status_id': loc.location_status_id
                    }
                    locations.append(location_data)
                
                return locations
            
        except Exception as e:
            self.logger.error(f'檢查位置可用性異常: {e}')
            return []
    
    def check_ng_rack_at_location(self, location_id: int) -> bool:
        """檢查位置是否有NG料架"""
        try:
            with self.connection_pool.get_session() as session:
                stmt = select(func.count(RackModel.id)).where(
                    and_(
                        RackModel.location_id == location_id,
                        RackModel.status_id == 7  # NG狀態
                    )
                )
                
                count = session.exec(stmt).one()
                return count > 0
            
        except Exception as e:
            self.logger.error(f'檢查NG料架異常: {e}')
            return False
    
    def check_carriers_in_room(self, room_id: int) -> bool:
        """檢查房間是否有carrier"""
        try:
            with self.connection_pool.get_session() as session:
                stmt = select(func.count(CarrierModel.id)).where(
                    CarrierModel.room_id == room_id
                )
                
                count = session.exec(stmt).one()
                return count > 0
            
        except Exception as e:
            self.logger.error(f'檢查房間carrier異常: {e}')
            return False
    
    def check_racks_at_location(self, location_id: int, status: List[int] = None) -> List[Dict[str, Any]]:
        """檢查位置的料架狀態"""
        try:
            with self.connection_pool.get_session() as session:
                stmt = select(RackModel).where(
                    RackModel.location_id == location_id
                )
                
                if status:
                    stmt = stmt.where(RackModel.status_id.in_(status))
                
                rack_models = session.exec(stmt).all()
                
                racks = []
                for rack in rack_models:
                    rack_data = {
                        'id': rack.id,
                        'name': rack.name,
                        'room_id': rack.room_id,
                        'location_id': rack.location_id,
                        'product_id': rack.product_id,
                        'status_id': rack.status_id,
                        'direction': rack.direction,
                        'agv_id': rack.agv_id
                    }
                    racks.append(rack_data)
                
                return racks
            
        except Exception as e:
            self.logger.error(f'檢查位置料架異常: {e}')
            return []
    
    def create_task_from_decision(self, decision_dict: Dict[str, Any]) -> Optional[int]:
        """從決策創建任務"""
        try:
            with self.connection_pool.get_session() as session:
                # 創建新任務實例
                task = Task(
                    work_id=decision_dict.get('work_id'),
                    status_id=decision_dict.get('status_id', 0),
                    room_id=decision_dict.get('room_id'),
                    node_id=decision_dict.get('node_id'),
                    name=decision_dict.get('name'),
                    description=decision_dict.get('description'),
                    agv_id=decision_dict.get('agv_id'),
                    parent_task_id=decision_dict.get('parent_task_id'),
                    priority=decision_dict.get('priority'),
                    parameters=decision_dict.get('parameters'),
                    created_at=datetime.now(timezone.utc)
                )
                
                session.add(task)
                session.commit()
                session.refresh(task)
                
                task_id = task.id
                
                self.logger.info(f'成功創建任務 ID: {task_id}')
                return task_id
            
        except Exception as e:
            self.logger.error(f'創建任務異常: {e}')
            return None
    
    def update_machine_parking_status(self, machine_id: int, space_num: int, status: int) -> bool:
        """更新機台停車格狀態 - OPUI停車格狀態同步"""
        try:
            # 根據停車格編號決定更新哪個欄位
            if space_num == 1:
                status_field = 'parking_space_1_status'
            elif space_num == 2:
                status_field = 'parking_space_2_status'
            else:
                self.logger.error(f'無效的停車格編號: {space_num}')
                return False
            
            with self.connection_pool.get_session() as session:
                # 查找機台
                stmt = select(Machine).where(
                    and_(
                        Machine.id == machine_id,
                        Machine.enable == 1
                    )
                )
                
                machine = session.exec(stmt).first()
                if not machine:
                    self.logger.error(f'找不到機台: machine_id={machine_id}')
                    return False
                
                # 更新停車格狀態
                setattr(machine, status_field, status)
                session.add(machine)
                session.commit()
                session.refresh(machine)
                
                updated_status = getattr(machine, status_field)
                
                self.logger.info(
                    f'✅ 停車格狀態更新成功: machine_id={machine_id}, space_{space_num}_status={updated_status}'
                )
                
                # 更新統計
                self.query_stats['total_queries'] += 1
                
                return True
            
        except Exception as e:
            self.logger.error(f'更新停車格狀態異常: {e}')
            return False
    
    def get_machine_parking_info(self, machine_id: int) -> Optional[Dict[str, Any]]:
        """獲取機台停車格完整資訊 - OPUI狀態查詢"""
        try:
            with self.connection_pool.get_session() as session:
                stmt = select(Machine).where(Machine.id == machine_id)
                
                machine = session.exec(stmt).first()
                if not machine:
                    self.logger.error(f'查詢機台停車格資訊失敗: machine_id={machine_id}')
                    return None
                
                # 構建完整的停車格資訊
                parking_info = {
                    'machine_id': machine.id,
                    'machine_name': machine.name,
                    'enabled': machine.enable == 1,
                    'parking_spaces': {
                        'space_1': {
                            'node_id': machine.parking_space_1,
                            'status': getattr(machine, 'parking_space_1_status', 0),
                            'is_available': getattr(machine, 'parking_space_1_status', 0) == 0,
                            'status_info': self._get_parking_status_description(getattr(machine, 'parking_space_1_status', 0))
                        },
                        'space_2': {
                            'node_id': machine.parking_space_2,
                            'status': getattr(machine, 'parking_space_2_status', 0),
                            'is_available': getattr(machine, 'parking_space_2_status', 0) == 0,
                            'status_info': self._get_parking_status_description(getattr(machine, 'parking_space_2_status', 0))
                        }
                    }
                }
                
                return parking_info
            
        except Exception as e:
            self.logger.error(f'查詢機台停車格資訊異常: {e}')
            return None
    
    def _get_parking_status_description(self, status: int) -> str:
        """獲取停車格狀態描述"""
        status_map = {
            0: "可用 (PARKING_AVAILABLE)",
            1: "任務進行中 (PARKING_TASK_ACTIVE)", 
            2: "任務完成 (PARKING_TASK_COMPLETED)"
        }
        return status_map.get(status, f"未知狀態 ({status})")
    
    def batch_update_parking_status(self, updates: List[Dict[str, Any]]) -> Dict[str, Any]:
        """批次更新停車格狀態 - 性能最佳化"""
        try:
            successful_updates = []
            failed_updates = []
            
            for update in updates:
                machine_id = update['machine_id']
                space_num = update['space_num']
                status = update['status']
                
                success = self.update_machine_parking_status(machine_id, space_num, status)
                
                if success:
                    successful_updates.append(update)
                else:
                    failed_updates.append(update)
            
            return {
                'total_updates': len(updates),
                'successful': len(successful_updates),
                'failed': len(failed_updates),
                'successful_updates': successful_updates,
                'failed_updates': failed_updates
            }
            
        except Exception as e:
            self.logger.error(f'批次更新停車格狀態異常: {e}')
            return {
                'total_updates': len(updates),
                'successful': 0,
                'failed': len(updates),
                'error': str(e)
            }
    
    def get_opui_pending_requests(self) -> List[Dict[str, Any]]:
        """獲取OPUI待處理請求 (基於machine parking space狀態) - 使用 SQLModel 查詢"""
        try:
            with self.connection_pool.get_session() as session:
                # 步驟1: 查詢有狀態變化的機台
                machines_stmt = select(Machine).where(
                    and_(
                        Machine.enable == 1,
                        or_(
                            Machine.parking_space_1_status != 0,
                            Machine.parking_space_2_status != 0
                        )
                    )
                )
                machines = session.exec(machines_stmt).all()
                
                # 步驟2: 查詢相關的 OPUI 任務
                opui_tasks_stmt = select(Task).where(
                    and_(
                        Task.work_id.in_(['100001', '100002']),
                        Task.status_id == 0
                    )
                ).order_by(Task.created_at.asc())
                
                opui_tasks = session.exec(opui_tasks_stmt).all()
                
                # 步驟3: 在 Python 中處理 JSON 參數匹配和組合結果
                requests = []
                
                # 為機台狀態變化創建請求記錄
                for machine in machines:
                    machine_data = {
                        'machine_id': machine.id,
                        'machine_name': machine.name,
                        'parking_space_1': machine.parking_space_1,
                        'parking_space_2': machine.parking_space_2,
                        'parking_space_1_status': getattr(machine, 'parking_space_1_status', 0),
                        'parking_space_2_status': getattr(machine, 'parking_space_2_status', 0),
                        'task_id': None,
                        'work_id': None,
                        'parameters': None,
                        'created_at': None
                    }
                    requests.append(machine_data)
                
                # 為 OPUI 任務創建請求記錄
                for task in opui_tasks:
                    # 從 JSON 參數中提取 machine_id
                    task_machine_id = None
                    if task.parameters and isinstance(task.parameters, dict):
                        task_machine_id = task.parameters.get('machine_id')
                    elif task.parameters and isinstance(task.parameters, str):
                        import json
                        try:
                            params = json.loads(task.parameters)
                            task_machine_id = params.get('machine_id')
                        except (json.JSONDecodeError, TypeError):
                            continue
                    
                    if task_machine_id:
                        # 查找對應的機台資訊
                        machine = session.get(Machine, task_machine_id)
                        if machine and machine.enable == 1:
                            task_data = {
                                'machine_id': machine.id,
                                'machine_name': machine.name,
                                'parking_space_1': machine.parking_space_1,
                                'parking_space_2': machine.parking_space_2,
                                'parking_space_1_status': getattr(machine, 'parking_space_1_status', 0),
                                'parking_space_2_status': getattr(machine, 'parking_space_2_status', 0),
                                'task_id': task.id,
                                'work_id': task.work_id,
                                'parameters': task.parameters,
                                'created_at': task.created_at
                            }
                            requests.append(task_data)
                
                # 去重和排序
                unique_requests = []
                seen_combinations = set()
                
                for req in requests:
                    # 使用 (machine_id, task_id) 作為唯一標識
                    key = (req['machine_id'], req['task_id'])
                    if key not in seen_combinations:
                        seen_combinations.add(key)
                        unique_requests.append(req)
                
                # 按任務創建時間排序，無任務的排在前面
                unique_requests.sort(key=lambda x: (
                    x['created_at'] is None,  # None 值排在前面
                    x['created_at'] if x['created_at'] is not None else ''
                ))
                
                return unique_requests
            
        except Exception as e:
            self.logger.error(f'查詢OPUI請求異常: {e}')
            return []
    
    def get_query_statistics(self) -> Dict[str, Any]:
        """獲取查詢統計資料"""
        cache_total = self.query_stats['cache_hits'] + self.query_stats['cache_misses']
        cache_hit_rate = self.query_stats['cache_hits'] / cache_total if cache_total > 0 else 0
        
        return {
            'stats': self.query_stats,
            'cache_hit_rate': cache_hit_rate,
            'cache_size': len(self.cache),
            'batch_cache_size': len(self.batch_cache),
            'last_update': datetime.now(timezone.utc).isoformat()
        }


def main():
    """主函數 - 直接連接模式測試"""
    
    def test_enhanced_client():
        """測試增強資料庫客戶端"""
        client = EnhancedDatabaseClient()
        
        # 測試基本連接
        try:
            agvs = client.get_agvs_by_state('idle')
            print(f"找到 {len(agvs)} 個空閒AGV")
            
            # 測試批次查詢
            location_groups = {
                'ng_area': [71, 72],
                'manual_area': [51, 52, 53, 54, 55]
            }
            result = client.batch_check_locations_status(location_groups, 0)
            print(f"批次查詢結果: {result.success}, 查詢數量: {result.query_count}")
            
            # 獲取統計資料
            stats = client.get_query_statistics()
            print(f"查詢統計: {stats}")
            
        except Exception as e:
            print(f"測試異常: {e}")
    
    # 執行測試
    test_enhanced_client()


if __name__ == '__main__':
    main()