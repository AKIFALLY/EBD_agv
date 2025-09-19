#!/usr/bin/env python3
"""
Database manager for flow_wcs - 使用 db_proxy 統一模型版本
這個檔案展示如何讓 flow_wcs 使用 db_proxy 的統一模型
"""

from typing import Dict, Any, List, Optional
from datetime import datetime
from contextlib import contextmanager
import os
import sys

# Add db_proxy to path for model imports
sys.path.append('/app/db_proxy_ws/src/db_proxy')

# Import unified models from db_proxy instead of defining our own
from db_proxy.models.agvc_location import Location
from db_proxy.models.rack import Rack
from db_proxy.models.agvc_task import Task, Work, FlowLog
from db_proxy.models.agvc_rcs import AGV
from db_proxy.connection_pool_manager import ConnectionPoolManager

# Database configuration
DATABASE_URL = os.getenv(
    'DATABASE_URL',
    'postgresql://agvc:password@192.168.100.254:5432/agvc'
)


class DatabaseManager:
    """Database manager for Flow WCS - 使用 db_proxy 統一模型"""
    
    def __init__(self):
        """Initialize database manager with db_proxy connection pool"""
        # 使用 db_proxy 的 ConnectionPoolManager 而非自己的連線管理
        self.pool_manager = ConnectionPoolManager(DATABASE_URL)
        
    @contextmanager
    def get_session(self):
        """Get database session using db_proxy's pool manager"""
        with self.pool_manager.get_session() as session:
            yield session
    
    def query_locations(self, type: Optional[str] = None, rooms: Optional[List[int]] = None, 
                       has_rack: Optional[bool] = None) -> List[Dict[str, Any]]:
        """Query locations from database using unified model"""
        # Handle unresolved variable references
        if isinstance(rooms, str):
            if rooms.startswith('${') and rooms.endswith('}'):
                rooms = [1, 2, 3, 4, 5]  # Use default rooms
            else:
                try:
                    import json
                    rooms = json.loads(rooms)
                except:
                    rooms = None
        
        # 保存原始的 has_rack 參數，避免被後面的循環變數覆蓋
        has_rack_filter = has_rack
        
        with self.get_session() as session:
            # 簡化查詢，只查詢 Location 表，避免複雜的 JOIN
            query = session.query(Location)
            
            if type:
                query = query.filter(Location.type == type)
            if rooms:
                query = query.filter(Location.room_id.in_(rooms))
            # Note: has_rack filter needs to be applied after joining with Rack table
            # We'll filter this in post-processing since we need to check rack.location_id
            
            locations = query.all()
            
            # 如果需要 Node 和 LocationStatus 的資料，分別查詢
            from db_proxy.models.node import Node
            from db_proxy.models.agvc_location import LocationStatus
            
            # 批量獲取所有需要的 node 和 status
            node_ids = [loc.node_id for loc in locations if loc.node_id]
            nodes_dict = {}
            if node_ids:
                nodes = session.query(Node).filter(Node.id.in_(node_ids)).all()
                nodes_dict = {node.id: node for node in nodes}
            
            status_ids = [loc.location_status_id for loc in locations if loc.location_status_id]
            status_dict = {}
            if status_ids:
                statuses = session.query(LocationStatus).filter(LocationStatus.id.in_(status_ids)).all()
                status_dict = {status.id: status for status in statuses}
            
            # 批量獲取所有 location 上的 rack (正確的關係: rack.location_id = location.id)
            location_ids = [loc.id for loc in locations]
            racks_dict = {}
            if location_ids:
                racks = session.query(Rack).filter(Rack.location_id.in_(location_ids)).all()
                # 使用 location_id 作為 key 來建立對應關係
                racks_dict = {rack.location_id: rack for rack in racks}
            
            # 手動轉換為 flow_wcs 期望的格式
            locations_list = []
            for loc in locations:
                # 從 Node 獲取 x, y 座標，如果沒有 node 則使用預設值
                node = nodes_dict.get(loc.node_id) if loc.node_id else None
                x = node.x if node else 0.0
                y = node.y if node else 0.0
                
                # 從 LocationStatus 獲取狀態名稱
                status = 'unknown'
                loc_status = status_dict.get(loc.location_status_id) if loc.location_status_id else None
                if loc_status:
                    if loc_status.id == LocationStatus.UNOCCUPIED:
                        status = 'available'
                    elif loc_status.id == LocationStatus.OCCUPIED:
                        status = 'occupied'
                    else:
                        status = loc_status.name.lower() if loc_status.name else 'unknown'
                
                # 檢查此位置是否有架台 (使用正確的關係)
                rack_at_location = racks_dict.get(loc.id)
                has_rack = rack_at_location is not None
                rack_id = rack_at_location.id if rack_at_location else None
                
                locations_list.append({
                    'id': loc.id,
                    'name': loc.name if hasattr(loc, 'name') else f'Location_{loc.id}',
                    'type': loc.type if hasattr(loc, 'type') else 'enter_or_exit',
                    'room_id': loc.room_id if hasattr(loc, 'room_id') else None,
                    'x': x,
                    'y': y,
                    'has_rack': has_rack,
                    'rack_id': rack_id,
                    'status': status,
                    'node_id': loc.node_id  # 也返回 node_id 供參考
                })
            
            # Apply has_rack filter if specified
            print(f"DEBUG: Before filter: {len(locations_list)} locations")
            if has_rack_filter is not None:
                print(f"DEBUG: Applying has_rack filter: has_rack={has_rack_filter}")
                locations_list = [loc for loc in locations_list if loc['has_rack'] == has_rack_filter]
                print(f"DEBUG: After filter: {len(locations_list)} locations")
                for loc in locations_list:
                    print(f"DEBUG:   Location {loc['id']}: has_rack={loc['has_rack']}")
            
            return locations_list
    
    def query_racks(self, location_id: Optional[int] = None, 
                   status: Optional[str] = None) -> List[Dict[str, Any]]:
        """Query racks from database using unified model"""
        with self.get_session() as session:
            query = session.query(Rack)
            
            if location_id:
                query = query.filter(Rack.location_id == location_id)
            # Rack 模型可能沒有 status 欄位，使用 side_a_status 或 side_b_status
            
            racks = query.all()
            
            # 手動轉換為 flow_wcs 期望的格式
            return [
                {
                    'id': rack.id,
                    'rack_id': getattr(rack, 'rack_id', f'RACK_{rack.id}'),
                    'location_id': rack.location_id if hasattr(rack, 'location_id') else None,
                    'side_a_status': rack.side_a_status if hasattr(rack, 'side_a_status') else 'empty',
                    'side_b_status': rack.side_b_status if hasattr(rack, 'side_b_status') else 'empty',
                    'side_a_carrier_id': rack.side_a_carrier_id if hasattr(rack, 'side_a_carrier_id') else None,
                    'side_b_carrier_id': rack.side_b_carrier_id if hasattr(rack, 'side_b_carrier_id') else None,
                    'rotation_angle': rack.rotation_angle if hasattr(rack, 'rotation_angle') else 0.0,
                    'status': getattr(rack, 'status', 'available')
                }
                for rack in racks
            ]
    
    def query_tasks(self, status: Optional[str] = None, type: Optional[str] = None,
                   limit: Optional[int] = None) -> List[Dict[str, Any]]:
        """Query tasks from database using unified model"""
        with self.get_session() as session:
            query = session.query(Task)
            
            if status:
                # 需要將狀態名稱轉換為 status_id
                from db_proxy.models.agvc_task import TaskStatus
                status_map = {
                    'pending': TaskStatus.PENDING,
                    'assigned': TaskStatus.READY_TO_EXECUTE,
                    'executing': TaskStatus.EXECUTING,
                    'completed': TaskStatus.COMPLETED,
                    'failed': TaskStatus.ERROR
                }
                status_id = status_map.get(status)
                if status_id:
                    query = query.filter(Task.status_id == status_id)
            
            if type:
                query = query.filter(Task.type == type)
            
            if limit:
                query = query.limit(limit)
            
            tasks = query.all()
            
            # 手動轉換為 flow_wcs 期望的格式
            result = []
            for task in tasks:
                # 將 status_id 轉換為 status 字串
                status_map = {
                    1: 'pending',
                    2: 'assigned',
                    3: 'executing',
                    4: 'completed',
                    5: 'cancelling',
                    6: 'failed'
                }
                status = status_map.get(task.status_id, 'unknown')
                
                result.append({
                    'id': task.id,
                    'task_id': task.task_id or f'TASK_{task.id}',
                    'type': task.type or 'default',
                    'work_id': task.work_id,
                    'location_id': task.location_id,
                    'rack_id': task.rack_id,
                    'agv_id': task.agv_id,
                    'status': status,
                    'priority': task.priority,
                    'metadata': task.parameters or {},
                    'created_at': task.created_at.isoformat() if task.created_at else None
                })
            return result
    
    def query_agvs(self, status: Optional[str] = None, type: Optional[str] = None) -> List[Dict[str, Any]]:
        """Query AGVs from database using unified model"""
        with self.get_session() as session:
            query = session.query(AGV)
            
            if type:
                # AGV model 使用 model 欄位，需要轉換
                model_map = {
                    'cargo': 'Cargo',
                    'loader': 'Loader',
                    'unloader': 'Unloader'
                }
                model = model_map.get(type, type.title())
                query = query.filter(AGV.model == model)
            
            if status:
                # 需要轉換狀態到 status_id
                status_map = {
                    'idle': 1,
                    'busy': 2,
                    'charging': 3,
                    'error': 4
                }
                status_id = status_map.get(status)
                if status_id:
                    query = query.filter(AGV.status_id == status_id)
            
            agvs = query.all()
            
            # 手動轉換為 flow_wcs 期望的格式
            result = []
            for agv in agvs:
                # 將 status_id 轉換為 status 字串
                status = 'unknown'
                if hasattr(agv, 'status_id'):
                    status_map = {
                        1: 'idle',
                        2: 'busy',
                        3: 'charging',
                        4: 'error'
                    }
                    status = status_map.get(agv.status_id, 'unknown')
                elif hasattr(agv, 'status'):
                    status = agv.status
                
                # 將 model 轉換為 type
                agv_type = 'unknown'
                if hasattr(agv, 'model'):
                    model_to_type = {
                        'Cargo': 'cargo',
                        'Loader': 'loader',
                        'Unloader': 'unloader'
                    }
                    agv_type = model_to_type.get(agv.model, agv.model.lower())
                elif hasattr(agv, 'type'):
                    agv_type = agv.type
                
                result.append({
                    'id': agv.id,
                    'agv_id': agv.agv_id if hasattr(agv, 'agv_id') else f'AGV_{agv.id}',
                    'type': agv_type,
                    'status': status,
                    'battery_level': agv.battery_level if hasattr(agv, 'battery_level') else 100.0,
                    'current_location': agv.current_location if hasattr(agv, 'current_location') else None,
                    'current_task_id': agv.current_task_id if hasattr(agv, 'current_task_id') else None,
                    'x': agv.x if hasattr(agv, 'x') else 0.0,
                    'y': agv.y if hasattr(agv, 'y') else 0.0,
                    'theta': agv.theta if hasattr(agv, 'theta') else 0.0
                })
            return result
    
    def check_rack_status(self, rack_id: int, side: str, check_type: str) -> bool:
        """Check rack status using unified model
        
        Args:
            rack_id: Rack ID
            side: 'a' or 'b' (目前架台方向 direction: 0=A面朝外, 180=B面朝外)
            check_type: 'has_work', 'all_complete', 'empty' 等
            
        Returns:
            Boolean based on check type
        """
        with self.get_session() as session:
            rack = session.query(Rack).filter(Rack.id == rack_id).first()
            
            if not rack:
                return False
            
            # 查詢架台上的載具
            from db_proxy.models.carrier import Carrier
            carriers = session.query(Carrier).filter(Carrier.rack_id == rack_id).all()
            
            # 查詢產品資訊以決定rack_index範圍
            from db_proxy.models.agvc_product import Product
            product = None
            if rack.product_id:
                product = session.query(Product).filter(Product.id == rack.product_id).first()
            
            # 根據產品尺寸決定A/B面的rack_index範圍
            # 根據 docs-ai 文檔定義:
            # S產品: 32個載具，A面 1-16 (4row×4col), B面 17-32 (4row×4col)
            # L產品: 16個載具，A面 1-8 (2row×4col), B面 9-16 (2row×4col)
            if product and product.size == 'L':
                # L尺寸產品 (總共16個載具)
                if side == 'a':
                    side_carriers = [c for c in carriers if c.rack_index and c.rack_index <= 8]
                elif side == 'b':
                    side_carriers = [c for c in carriers if c.rack_index and c.rack_index > 8 and c.rack_index <= 16]
                else:
                    return False
            else:
                # S尺寸產品 (總共32個載具，預設)
                if side == 'a':
                    side_carriers = [c for c in carriers if c.rack_index and c.rack_index <= 16]
                elif side == 'b':
                    side_carriers = [c for c in carriers if c.rack_index and c.rack_index > 16 and c.rack_index <= 32]
                else:
                    return False
            
            # 根據檢查類型返回結果
            if check_type == 'has_work':
                # 檢查該面是否有待作業載具 (狀態不是 completed)
                # 狀態 8 = 已完成, 其他狀態都視為有工作待做
                # 狀態 1 = 空閒, 5 = 處理中, 101-603 = 各種處理中狀態
                print(f"🔍 DEBUG: check_rack_status rack_id={rack_id}, side={side}")
                print(f"  - Total carriers: {len(carriers)}")
                print(f"  - Side carriers: {len(side_carriers)}")
                print(f"  - Side carrier statuses: {[c.status_id for c in side_carriers]}")
                result = any(c.status_id != 8 for c in side_carriers)  # 8 = 已完成
                print(f"  - Has work result: {result}")
                return result
                
            elif check_type == 'all_complete':
                # ⚠️ 注意：all_complete 實際上檢查該面是否為空（所有載具都被拿走了）
                # 入口處的翻轉邏輯：A面空了（載具都被拿走）才需要翻轉
                # 建議使用 check_type='empty' 來避免誤解
                return len(side_carriers) == 0  # 沒有載具才是"完成"狀態
                
            elif check_type == 'empty':
                # 檢查該面是否沒有載具
                return len(side_carriers) == 0
                
            elif check_type == 'occupied':
                # 檢查該面是否有載具
                return len(side_carriers) > 0
            
            return False
    
    def check_task_exists(self, type: Optional[str] = None, 
                         location_id: Optional[int] = None,
                         rack_id: Optional[int] = None,
                         status: Optional[str] = None) -> bool:
        """Check if task exists using unified model"""
        with self.get_session() as session:
            query = session.query(Task)
            
            if type:
                query = query.filter(Task.type == type)
            if location_id:
                query = query.filter(Task.location_id == location_id)
            if rack_id:
                query = query.filter(Task.rack_id == rack_id)
            
            # 如果沒有指定狀態，預設檢查所有未完成的狀態
            if status:
                # 轉換狀態
                from db_proxy.models.agvc_task import TaskStatus
                status_map = {
                    'pending': TaskStatus.PENDING,
                    'executing': TaskStatus.EXECUTING
                }
                status_id = status_map.get(status)
                if status_id:
                    query = query.filter(Task.status_id == status_id)
            else:
                # 檢查所有未完成的任務 (狀態 0-3 都是未完成)
                from db_proxy.models.agvc_task import TaskStatus
                incomplete_statuses = [
                    TaskStatus.REQUESTING,        # 0
                    TaskStatus.PENDING,           # 1
                    TaskStatus.READY_TO_EXECUTE,  # 2
                    TaskStatus.EXECUTING          # 3
                ]
                query = query.filter(Task.status_id.in_(incomplete_statuses))
            
            return query.first() is not None
    
    def create_task(self, type: str, work_id: str, location_id: int = None,
                   rack_id: int = None, room_id: int = None, agv_id: str = None,
                   priority: int = 100, metadata: Dict = None) -> str:
        """Create a new task using unified model
        
        Args:
            type: Task type (e.g., 'RACK_ROTATION')
            work_id: Work ID (e.g., '220001')
            location_id: Location ID where task occurs
            rack_id: Rack ID involved in task
            room_id: Room ID where task belongs (important for identifying which room's entrance/exit)
            agv_id: AGV ID assigned to task (optional)
            priority: Task priority
            metadata: Additional parameters for the task
        """
        with self.get_session() as session:
            # work_id 直接就是 Work 表的 ID (如 220001)
            # 不需要查詢 work_code，直接使用數字 ID
            work_id_int = int(work_id)
            
            # 查找 Work 記錄，確認它存在
            work = session.query(Work).filter(Work.id == work_id_int).first()
            if not work:
                # 如果沒有找到對應的 Work，記錄錯誤並返回
                print(f"❌ Error: Work ID {work_id_int} not found in database")
                return None
            
            # 合併 Work 的 parameters 和傳入的 metadata
            # Work.parameters 優先級較低，metadata 可以覆蓋
            task_parameters = {}
            
            # 先加入 Work 的 parameters（如果有）
            if work.parameters:
                task_parameters.update(work.parameters)
            
            # 再加入傳入的 metadata，可能會覆蓋某些值
            if metadata:
                task_parameters.update(metadata)
            
            # 創建 Task，使用正確的 work_id (整數)和合併的 parameters
            task = Task(
                type=type,
                work_id=work_id_int,  # 直接使用整數 work_id (220001)
                location_id=location_id,
                rack_id=rack_id,
                room_id=room_id,  # 設定房間ID - 標識任務屬於哪個房間
                priority=priority,
                parameters=task_parameters,  # 使用合併後的 parameters
                status_id=1,  # PENDING
                name=f"Task {type} for {work.name}"  # 使用實際的 Work 名稱
            )
            
            # 生成 task_id
            task.task_id = task.generate_task_id(str(work_id_int))
            
            session.add(task)
            session.commit()
            
            return task.task_id
    
    def update_task(self, task_id: str, status: str) -> bool:
        """Update task status using unified model"""
        with self.get_session() as session:
            task = session.query(Task).filter(Task.task_id == task_id).first()
            
            if not task:
                return False
            
            # 轉換狀態
            from db_proxy.models.agvc_task import TaskStatus
            status_map = {
                'assigned': TaskStatus.READY_TO_EXECUTE,
                'executing': TaskStatus.EXECUTING,
                'completed': TaskStatus.COMPLETED,
                'cancelled': TaskStatus.CANCELLED,
                'failed': TaskStatus.ERROR
            }
            
            status_id = status_map.get(status)
            if status_id:
                task.status_id = status_id
                
                if status == 'completed':
                    task.completed_at = datetime.now()
                
                session.commit()
                return True
            
            return False
    
    def assign_task(self, task_id: str, agv_id: str) -> Dict[str, Any]:
        """Assign task to AGV using unified model"""
        with self.get_session() as session:
            # 查找 Task
            task = session.query(Task).filter(Task.task_id == task_id).first()
            if not task:
                return {'success': False, 'message': 'Task not found'}
            
            # 查找 AGV
            agv = session.query(AGV).filter(AGV.agv_id == agv_id).first()
            if not agv:
                return {'success': False, 'message': 'AGV not found'}
            
            # 分配任務
            task.agv_id = agv.id
            task.status_id = 2  # READY_TO_EXECUTE
            
            # 更新 AGV 狀態
            agv.current_task_id = task.task_id
            agv.status_id = 2  # busy
            
            session.commit()
            
            return {
                'success': True,
                'task_id': task.task_id,
                'agv_id': agv.agv_id
            }
    
    def rotate_rack(self, rack_id: int, angle: float) -> bool:
        """Rotate rack using unified model's rotate method"""
        with self.get_session() as session:
            rack = session.query(Rack).filter(Rack.id == rack_id).first()
            
            if not rack:
                return False
            
            # 使用統一模型的 rotate 方法
            rack.rotate(angle)
            session.commit()
            
            return True
    
    def log_flow_execution(self, flow_id: str, flow_name: str, work_id: str,
                          section: str, step_id: str, function: str,
                          params: Dict = None, result: Any = None,
                          status: str = 'success', error_message: str = None,
                          duration: float = None):
        """Log flow execution using FlowLog model"""
        with self.get_session() as session:
            # work_id 直接就是 Work 表的 ID (如 220001)
            work_id_int = None
            if work_id:
                work_id_int = int(work_id)
                # 確認 Work 記錄存在，但不創建新的
                work = session.query(Work).filter(Work.id == work_id_int).first()
                if not work:
                    print(f"⚠️ Warning: Work ID {work_id_int} not found for logging")
                    # 不創建新的 Work 記錄，只記錄警告
            
            log = FlowLog(
                flow_id=flow_id,
                flow_name=flow_name,
                work_id=work_id_int,  # 直接使用整數 work_id
                section=section,
                step_id=step_id,
                function=function,
                params=params,
                result={'value': result} if result else None,
                status=status,
                error_message=error_message,
                duration=duration
            )
            
            session.add(log)
            session.commit()


# Create global database manager instance
db_manager = DatabaseManager()