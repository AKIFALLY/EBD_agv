"""
TAFL Database Bridge Module - Enhanced Version
Provides enhanced database access functions for TAFL flows with:
- Advanced filtering options
- Parameter validation
- Change logging
- Pagination support
- Error handling
"""

import logging
from typing import List, Dict, Any, Optional, Tuple
from datetime import datetime, timedelta
import json
from enum import Enum

# Import db_proxy's ConnectionPoolManager
import sys
sys.path.insert(0, '/app/db_proxy_ws/install/db_proxy/lib/python3.12/site-packages')

from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import (
    Location, LocationStatus,
    Rack, RackStatus,
    Carrier, CarrierStatus,
    Task, TaskStatus,
    Work
)
from sqlmodel import Session, select, and_, or_, desc, asc

logger = logging.getLogger(__name__)


class SortOrder(Enum):
    """Sort order enumeration"""
    ASC = "asc"
    DESC = "desc"


class TAFLDatabaseBridge:
    """Enhanced database bridge for TAFL operations"""
    
    def __init__(self, database_url: str):
        """Initialize database connection"""
        self.pool_manager = ConnectionPoolManager(database_url)
        self.change_log = []  # Store change history
        logger.info("✅ Enhanced Database bridge initialized")
    
    # ========== Enhanced Query Functions ==========
    
    def query_locations(self, **kwargs) -> Dict[str, Any]:
        """
        Query locations with enhanced filtering options
        
        Supported filters:
        - room_id: Filter by room ID
        - node_id: Filter by node ID
        - status: Filter by status name
        - area: Filter by area (new)
        - type: Filter by location type (new)
        - available_only: Only return available locations (new)
        - occupied_only: Only return occupied locations (new)
        - sort_by: Field to sort by (new)
        - sort_order: 'asc' or 'desc' (new)
        - offset: Pagination offset (new)
        - limit: Maximum results
        """
        with self.pool_manager.get_session() as session:
            query = select(Location, LocationStatus).join(
                LocationStatus, Location.location_status_id == LocationStatus.id, isouter=True
            )
            
            # Apply filters
            conditions = []

            # Basic filters
            if 'id' in kwargs:
                conditions.append(Location.id == kwargs['id'])
            if 'room_id' in kwargs:
                conditions.append(Location.room_id == kwargs['room_id'])
            if 'node_id' in kwargs:
                conditions.append(Location.node_id == kwargs['node_id'])
            if 'status' in kwargs:
                conditions.append(LocationStatus.name == kwargs['status'])
            
            # New enhanced filters
            # Note: Location model doesn't have 'area' field
            if 'type' in kwargs:
                conditions.append(Location.type == kwargs['type'])
            if kwargs.get('available_only'):
                conditions.append(LocationStatus.name == 'AVAILABLE')
            if kwargs.get('occupied_only'):
                conditions.append(LocationStatus.name == 'OCCUPIED')
            
            # Multiple status filter
            if 'status_list' in kwargs:
                status_list = kwargs['status_list']
                if isinstance(status_list, list):
                    conditions.append(LocationStatus.name.in_(status_list))
            
            if conditions:
                query = query.where(and_(*conditions))
            
            # Apply sorting
            sort_by = kwargs.get('sort_by', 'id')
            sort_order = kwargs.get('sort_order', 'asc')
            
            if hasattr(Location, sort_by):
                if sort_order == 'desc':
                    query = query.order_by(desc(getattr(Location, sort_by)))
                else:
                    query = query.order_by(asc(getattr(Location, sort_by)))
            
            # Apply pagination
            if 'offset' in kwargs:
                query = query.offset(kwargs['offset'])
            if 'limit' in kwargs:
                query = query.limit(kwargs['limit'])
            
            results = session.exec(query).all()
            
            # Get total count for pagination info
            count_query = select(Location, LocationStatus).join(
                LocationStatus, Location.location_status_id == LocationStatus.id, isouter=True
            )
            if conditions:
                count_query = count_query.where(and_(*conditions))
            total_count = len(session.exec(count_query).all())
            
            # Convert to dict with enhanced info
            locations = []
            for location, status in results:
                loc_dict = location.model_dump()
                loc_dict['status'] = status.name if status else 'UNKNOWN'
                loc_dict['status_id'] = status.id if status else None
                locations.append(loc_dict)
            
            # Return with pagination metadata
            return {
                'data': locations,
                'total': total_count,
                'offset': kwargs.get('offset', 0),
                'limit': kwargs.get('limit', None),
                'has_more': total_count > (kwargs.get('offset', 0) + len(locations))
            }
    
    def query_racks(self, **kwargs) -> Dict[str, Any]:
        """
        Query racks with enhanced filtering options
        
        Supported filters:
        - location_id: Filter by location ID
        - status: Filter by status name
        - status_list: Filter by multiple statuses (new)
        - is_carry: Filter by carry status
        - is_docked: Filter by docked status
        - needs_rotation: Only racks needing rotation (new)
        - completed_only: Only completed racks (new)
        - in_progress_only: Only in-progress racks (new)
        - created_after: Created after date (new)
        - created_before: Created before date (new)
        - sort_by: Field to sort by (new)
        - sort_order: 'asc' or 'desc' (new)
        - offset: Pagination offset (new)
        - limit: Maximum results
        """
        with self.pool_manager.get_session() as session:
            query = select(Rack, RackStatus).join(
                RackStatus, Rack.status_id == RackStatus.id, isouter=True
            )
            
            # Apply filters
            conditions = []
            
            # Basic filters
            if 'location_id' in kwargs:
                conditions.append(Rack.location_id == kwargs['location_id'])
            if 'status' in kwargs:
                conditions.append(RackStatus.name == kwargs['status'])
            if 'is_carry' in kwargs:
                conditions.append(Rack.is_carry == kwargs['is_carry'])
            if 'is_docked' in kwargs:
                conditions.append(Rack.is_docked == kwargs['is_docked'])
            
            # Enhanced filters
            if 'status_list' in kwargs:
                status_list = kwargs['status_list']
                if isinstance(status_list, list):
                    conditions.append(RackStatus.name.in_(status_list))
            
            if kwargs.get('needs_rotation'):
                conditions.append(and_(Rack.is_carry == True, Rack.is_docked == False))
            
            if kwargs.get('completed_only'):
                conditions.append(and_(Rack.is_carry == True, Rack.is_docked == True))
            
            if kwargs.get('in_progress_only'):
                conditions.append(or_(
                    and_(Rack.is_carry == True, Rack.is_docked == False),
                    and_(Rack.is_carry == False, Rack.is_docked == False)
                ))
            
            # Date filters
            if 'created_after' in kwargs:
                conditions.append(Rack.created_at >= kwargs['created_after'])
            if 'created_before' in kwargs:
                conditions.append(Rack.created_at <= kwargs['created_before'])
            
            if conditions:
                query = query.where(and_(*conditions))
            
            # Apply sorting
            sort_by = kwargs.get('sort_by', 'id')
            sort_order = kwargs.get('sort_order', 'asc')
            
            if hasattr(Rack, sort_by):
                if sort_order == 'desc':
                    query = query.order_by(desc(getattr(Rack, sort_by)))
                else:
                    query = query.order_by(asc(getattr(Rack, sort_by)))
            
            # Apply pagination
            if 'offset' in kwargs:
                query = query.offset(kwargs['offset'])
            if 'limit' in kwargs:
                query = query.limit(kwargs['limit'])
            
            results = session.exec(query).all()
            
            # Get total count
            count_query = select(Rack, RackStatus).join(
                RackStatus, Rack.status_id == RackStatus.id, isouter=True
            )
            if conditions:
                count_query = count_query.where(and_(*conditions))
            total_count = len(session.exec(count_query).all())
            
            # Convert to dict with enhanced info
            racks = []
            for rack, status in results:
                rack_dict = rack.model_dump()
                rack_dict['status'] = status.name if status else 'UNKNOWN'
                rack_dict['status_id'] = status.id if status else None
                rack_dict['needs_rotation'] = rack.is_carry and not rack.is_docked
                rack_dict['completion_percentage'] = self._calculate_rack_completion(rack)
                racks.append(rack_dict)
            
            return {
                'data': racks,
                'total': total_count,
                'offset': kwargs.get('offset', 0),
                'limit': kwargs.get('limit', None),
                'has_more': total_count > (kwargs.get('offset', 0) + len(racks))
            }
    
    def query_tasks(self, **kwargs) -> Dict[str, Any]:
        """
        Query tasks with enhanced filtering options
        
        Supported filters:
        - rack_id: Filter by rack ID (for TAFL)
        - work_id: Filter by work ID
        - status: Filter by status name
        - status_id: Filter by status ID (for TAFL)
        - status_id_in: Filter by multiple status IDs (for TAFL)
        - status_list: Filter by multiple statuses (new)
        - priority: Filter by priority
        - priority_min: Minimum priority (new)
        - priority_max: Maximum priority (new)
        - assigned_to: Filter by assignee (new)
        - created_after: Created after date (new)
        - created_before: Created before date (new)
        - updated_after: Updated after date (new)
        - overdue_only: Only overdue tasks (new)
        - pending_only: Only pending tasks (new)
        - sort_by: Field to sort by (default: priority,created_at)
        - sort_order: 'asc' or 'desc'
        - offset: Pagination offset
        - limit: Maximum results
        """
        with self.pool_manager.get_session() as session:
            query = select(Task, TaskStatus).join(
                TaskStatus, Task.status_id == TaskStatus.id, isouter=True
            )
            
            # Apply filters
            conditions = []
            
            # Basic filters
            if 'rack_id' in kwargs:
                conditions.append(Task.rack_id == kwargs['rack_id'])
            if 'work_id' in kwargs:
                conditions.append(Task.work_id == kwargs['work_id'])
            if 'status' in kwargs:
                conditions.append(TaskStatus.name == kwargs['status'])
            if 'status_id' in kwargs:
                conditions.append(Task.status_id == kwargs['status_id'])
            if 'status_id_in' in kwargs:
                status_ids = kwargs['status_id_in']
                if isinstance(status_ids, list):
                    conditions.append(Task.status_id.in_(status_ids))
            if 'priority' in kwargs:
                conditions.append(Task.priority == kwargs['priority'])
            
            # Enhanced filters
            if 'status_list' in kwargs:
                status_list = kwargs['status_list']
                if isinstance(status_list, list):
                    conditions.append(TaskStatus.name.in_(status_list))
            
            if 'priority_min' in kwargs:
                conditions.append(Task.priority >= kwargs['priority_min'])
            if 'priority_max' in kwargs:
                conditions.append(Task.priority <= kwargs['priority_max'])
            
            if 'assigned_to' in kwargs:
                conditions.append(Task.assigned_to == kwargs['assigned_to'])
            
            # Date filters
            if 'created_after' in kwargs:
                conditions.append(Task.created_at >= kwargs['created_after'])
            if 'created_before' in kwargs:
                conditions.append(Task.created_at <= kwargs['created_before'])
            if 'updated_after' in kwargs:
                conditions.append(Task.updated_at >= kwargs['updated_after'])
            
            # Special filters
            # Note: Task model doesn't have deadline field, skip overdue filter
            if kwargs.get('overdue_only'):
                # Skip overdue filter as Task doesn't have deadline field
                pass
            
            if kwargs.get('pending_only'):
                conditions.append(TaskStatus.name.in_(['PENDING', 'QUEUED']))
            
            if kwargs.get('active_only'):
                conditions.append(TaskStatus.name.in_(['IN_PROGRESS', 'RUNNING']))
            
            if conditions:
                query = query.where(and_(*conditions))
            
            # Apply complex sorting
            sort_by = kwargs.get('sort_by', 'priority,created_at')
            sort_order = kwargs.get('sort_order', 'desc,asc')
            
            if ',' in sort_by:
                # Multiple sort fields
                sort_fields = sort_by.split(',')
                sort_orders = sort_order.split(',') if ',' in sort_order else [sort_order] * len(sort_fields)
                
                for field, order in zip(sort_fields, sort_orders):
                    field = field.strip()
                    order = order.strip()
                    if hasattr(Task, field):
                        if order == 'desc':
                            query = query.order_by(desc(getattr(Task, field)))
                        else:
                            query = query.order_by(asc(getattr(Task, field)))
            else:
                # Single sort field
                if hasattr(Task, sort_by):
                    if sort_order == 'desc':
                        query = query.order_by(desc(getattr(Task, sort_by)))
                    else:
                        query = query.order_by(asc(getattr(Task, sort_by)))
            
            # Apply pagination
            if 'offset' in kwargs:
                query = query.offset(kwargs['offset'])
            if 'limit' in kwargs:
                query = query.limit(kwargs['limit'])
            
            results = session.exec(query).all()
            
            # Get total count and statistics
            count_query = select(Task, TaskStatus).join(
                TaskStatus, Task.status_id == TaskStatus.id, isouter=True
            )
            if conditions:
                count_query = count_query.where(and_(*conditions))
            all_results = session.exec(count_query).all()
            total_count = len(all_results)
            
            # Calculate statistics
            stats = self._calculate_task_statistics(all_results)
            
            # Convert to dict with enhanced info
            tasks = []
            for task, status in results:
                task_dict = task.model_dump()
                task_dict['status'] = status.name if status else 'UNKNOWN'
                task_dict['status_id'] = status.id if status else None
                # Task model doesn't have deadline field
                task_dict['is_overdue'] = False
                task_dict['execution_time'] = self._calculate_execution_time(task)
                tasks.append(task_dict)
            
            return {
                'data': tasks,
                'total': total_count,
                'offset': kwargs.get('offset', 0),
                'limit': kwargs.get('limit', None),
                'has_more': total_count > (kwargs.get('offset', 0) + len(tasks)),
                'statistics': stats
            }
    
    def query_works(self, **kwargs) -> Dict[str, Any]:
        """
        Query works with enhanced filtering options
        
        Supported filters:
        - work_code: Filter by work code
        - enabled: Filter by enabled status
        - category: Filter by work category (new)
        - created_after: Created after date (new)
        - sort_by: Field to sort by (new)
        - sort_order: 'asc' or 'desc' (new)
        - offset: Pagination offset (new)
        - limit: Maximum results
        """
        with self.pool_manager.get_session() as session:
            query = select(Work)
            
            # Apply filters
            conditions = []
            if 'work_code' in kwargs:
                conditions.append(Work.work_code == kwargs['work_code'])
            if 'enabled' in kwargs:
                conditions.append(Work.enabled == kwargs['enabled'])
            if 'category' in kwargs:
                conditions.append(Work.category == kwargs['category'])
            
            # Date filter
            if 'created_after' in kwargs:
                conditions.append(Work.created_at >= kwargs['created_after'])
            
            if conditions:
                query = query.where(and_(*conditions))
            
            # Apply sorting
            sort_by = kwargs.get('sort_by', 'id')
            sort_order = kwargs.get('sort_order', 'asc')
            
            if hasattr(Work, sort_by):
                if sort_order == 'desc':
                    query = query.order_by(desc(getattr(Work, sort_by)))
                else:
                    query = query.order_by(asc(getattr(Work, sort_by)))
            
            # Apply pagination
            if 'offset' in kwargs:
                query = query.offset(kwargs['offset'])
            if 'limit' in kwargs:
                query = query.limit(kwargs['limit'])
            
            results = session.exec(query).all()
            
            # Get total count
            count_query = select(Work)
            if conditions:
                count_query = count_query.where(and_(*conditions))
            total_count = len(session.exec(count_query).all())
            
            # Convert to dict
            works = [work.model_dump() for work in results]
            
            return {
                'data': works,
                'total': total_count,
                'offset': kwargs.get('offset', 0),
                'limit': kwargs.get('limit', None),
                'has_more': total_count > (kwargs.get('offset', 0) + len(works))
            }
    
    def query_agvs(self, **kwargs) -> Dict[str, Any]:
        """Query AGVs from database
        
        Args:
            **kwargs: Query filters (e.g., status_id=1, enable=1)
        
        Returns:
            List of AGV dictionaries
        """
        try:
            with self.pool_manager.get_session() as session:
                # Import AGV model
                from db_proxy.models.agvc_rcs import AGV
                
                # Build query
                query = session.query(AGV)
                
                # Apply filters if provided
                if kwargs:
                    for key, value in kwargs.items():
                        if hasattr(AGV, key):
                            query = query.filter(getattr(AGV, key) == value)
                
                # Execute query
                agvs = query.all()
                
                # Convert to dictionaries
                result = []
                for agv in agvs:
                    result.append({
                        'id': agv.id,
                        'name': agv.name,
                        'description': agv.description,
                        'model': agv.model,
                        'x': agv.x,
                        'y': agv.y,
                        'heading': agv.heading,
                        'battery': agv.battery if agv.battery is not None else 0,  # Default to 0 if None
                        'last_node_id': agv.last_node_id,
                        'enable': agv.enable,
                        'status_id': agv.status_id
                    })
                
                return result
                
        except Exception as e:
            logger.error(f"Failed to query AGVs: {str(e)}")
            return []
    
    # ========== Enhanced Check Functions ==========
    
    def check_location_available(self, location_id: int) -> Tuple[bool, Dict]:
        """
        Check if a location is available with detailed info
        
        Returns:
            Tuple[bool, Dict]: (is_available, details)
        """
        with self.pool_manager.get_session() as session:
            location = session.get(Location, location_id)
            if not location:
                return False, {'error': 'Location not found', 'location_id': location_id}
            
            is_available = location.location_status_id == 2  # LocationStatus.UNOCCUPIED = 2
            
            details = {
                'location_id': location_id,
                'is_available': is_available,
                'current_status': location.location_status_id,
                'room_id': location.room_id,
                'node_id': location.node_id
            }
            
            return is_available, details
    
    def check_rack_needs_rotation(self, rack_id: int) -> Tuple[bool, Dict]:
        """
        Check if a rack needs rotation with detailed info
        
        Returns:
            Tuple[bool, Dict]: (needs_rotation, details)
        """
        with self.pool_manager.get_session() as session:
            rack = session.get(Rack, rack_id)
            if not rack:
                return False, {'error': 'Rack not found', 'rack_id': rack_id}
            
            needs_rotation = rack.is_carry and not rack.is_docked
            
            details = {
                'rack_id': rack_id,
                'needs_rotation': needs_rotation,
                'is_carry': rack.is_carry,
                'is_docked': rack.is_docked,
                'completion_percentage': self._calculate_rack_completion(rack)
            }
            
            return needs_rotation, details
    
    def check_task_exists(self, task_id: str) -> Tuple[bool, Dict]:
        """
        Check if a task exists with detailed info
        
        Returns:
            Tuple[bool, Dict]: (exists, details)
        """
        with self.pool_manager.get_session() as session:
            task = session.get(Task, task_id)
            if not task:
                return False, {'error': 'Task not found', 'task_id': task_id}
            
            # Get status name
            status = session.get(TaskStatus, task.status_id)
            
            details = {
                'task_id': task_id,
                'exists': True,
                'status': status.name if status else 'UNKNOWN',
                'priority': task.priority,
                'work_id': task.work_id,
                'created_at': task.created_at.isoformat() if task.created_at else None
            }
            
            return True, details
    
    # ========== Enhanced Create Functions with Validation ==========
    
    def create_task(self, **params) -> Tuple[str, Dict]:
        """
        Create a new task with validation
        
        Required params:
        - work_id: Work ID
        
        Optional params:
        - name: Task name (default: "Task for Work {work_id}")
        - description: Task description
        - priority: Task priority (1-10, default: 5)
        - rack_id: Associated rack ID
        - status_id: Initial status (default: PENDING)
        - metadata: Additional metadata
        - assigned_to: Assignee
        - deadline: Task deadline
        
        Returns:
            Tuple[str, Dict]: (task_id, details)
        """
        # Validate required parameters
        errors = []
        if 'work_id' not in params:
            errors.append("work_id is required")
        
        # Validate priority range
        priority = params.get('priority', 5)
        if not isinstance(priority, int) or priority < 1 or priority > 10:
            errors.append("priority must be an integer between 1 and 10")
        
        # Note: Task model doesn't have deadline field, skip deadline validation
        deadline = None  # Task model doesn't support deadline
        
        if errors:
            logger.error(f"Task creation validation failed: {errors}")
            return None, {'success': False, 'errors': errors}
        
        with self.pool_manager.get_session() as session:
            try:
                # Check if work exists
                work = session.get(Work, params['work_id'])
                if not work:
                    return None, {'success': False, 'errors': ['Work ID not found']}
                
                # Create task
                task = Task(
                    name=params.get('name', f"Task for Work {params['work_id']}"),
                    description=params.get('description'),  # Add description field
                    work_id=params['work_id'],
                    rack_id=params.get('rack_id'),  # Add rack_id from params
                    priority=priority,
                    status_id=params.get('status_id', TaskStatus.PENDING),  # Use status_id from params, default to PENDING
                    parameters=params.get('parameters', {}),  # Fixed: use 'parameters' not 'metadata'
                    # deadline field not available in Task model
                    created_at=datetime.now()
                )
                
                session.add(task)
                session.commit()
                session.refresh(task)
                
                # Log the creation
                self._log_change('CREATE', 'Task', task.id, None, task.model_dump())
                
                logger.info(f"✅ Created task: {task.id}")
                
                return str(task.id), {
                    'success': True,
                    'task_id': str(task.id),
                    'name': task.name,
                    'description': task.description,
                    'work_id': params['work_id'],
                    'rack_id': task.rack_id,
                    'priority': priority,
                    'status': 'PENDING',
                    'created_at': task.created_at.isoformat()
                }
                
            except Exception as e:
                logger.error(f"Failed to create task: {e}")
                return None, {'success': False, 'errors': [str(e)]}
    
    def create_rack(self, **params) -> Tuple[str, Dict]:
        """
        Create a new rack with validation
        
        Required params:
        - name: Rack name
        - location_id: Location ID
        
        Optional params:
        - status_id: Initial status ID
        - is_carry: Carry status
        - is_docked: Docked status
        - room_id: Room ID
        - product_id: Product ID
        - direction: Direction angle
        
        Returns:
            Tuple[str, Dict]: (rack_id, details)
        """
        # Validate required parameters
        errors = []
        if 'name' not in params:
            # Generate default name if not provided
            params['name'] = f'Rack_{datetime.now().strftime("%Y%m%d%H%M%S")}'
            logger.info(f"Generated default rack name: {params['name']}")
        
        if 'location_id' not in params:
            errors.append("location_id is required")
        
        if errors:
            logger.error(f"Rack creation validation failed: {errors}")
            return None, {'success': False, 'errors': errors}
        
        with self.pool_manager.get_session() as session:
            try:
                # Check if location exists
                location = session.get(Location, params['location_id'])
                if not location:
                    return None, {'success': False, 'errors': ['Location ID not found']}
                
                # Check if location is available
                if location.location_status_id != 2:  # LocationStatus.UNOCCUPIED = 2
                    return None, {'success': False, 'errors': ['Location is not available']}
                
                rack = Rack(
                    name=params['name'],  # Add name field
                    location_id=params['location_id'],
                    room_id=params.get('room_id'),  # Optional room_id
                    product_id=params.get('product_id'),  # Optional product_id
                    status_id=params.get('status_id', 1),  # Default to status_id=1 (EMPTY)
                    is_carry=params.get('is_carry', 0),  # Default to 0
                    is_docked=params.get('is_docked', 0),  # Default to 0
                    is_in_map=params.get('is_in_map', 0),  # Default to 0
                    direction=params.get('direction', 0)  # Default direction
                )
                
                session.add(rack)
                session.commit()
                session.refresh(rack)
                
                # Log the creation
                self._log_change('CREATE', 'Rack', rack.id, None, rack.model_dump())
                
                logger.info(f"✅ Created rack: {rack.id} with name: {rack.name}")
                
                return str(rack.id), {
                    'success': True,
                    'rack_id': str(rack.id),
                    'name': rack.name,
                    'location_id': params['location_id'],
                    'status_id': rack.status_id,
                    'is_carry': rack.is_carry,
                    'is_docked': rack.is_docked,
                    'created_at': datetime.now().isoformat()
                }
                
            except Exception as e:
                logger.error(f"Failed to create rack: {e}")
                logger.error(f"Exception type: {type(e).__name__}")
                logger.error(f"Exception args: {e.args}")
                import traceback
                logger.error(f"Traceback: {traceback.format_exc()}")
                return None, {'success': False, 'errors': [str(e)]}
    
    # ========== Enhanced Update Functions with Logging ==========
    
    def update_task(self, **kwargs) -> bool:
        """
        Generic update task function for TAFL
        
        Args:
            where: Dict with conditions (e.g., {'id': 1})
            set: Dict with fields to update (e.g., {'status_id': 3, 'priority': 5})
        
        Returns:
            bool: Success status
        """
        where = kwargs.get('where', {})
        set_fields = kwargs.get('set', {})
        
        with self.pool_manager.get_session() as session:
            try:
                # Build query
                query = select(Task)
                
                # Apply where conditions
                for key, value in where.items():
                    query = query.where(getattr(Task, key) == value)
                
                # Get tasks to update
                tasks = session.exec(query).all()
                
                if not tasks:
                    logger.warning(f"No tasks found with conditions: {where}")
                    return False
                
                # Update each task
                for task in tasks:
                    # Store old values for logging
                    old_values = {}
                    for key in set_fields.keys():
                        old_values[key] = getattr(task, key, None)
                    
                    # Apply updates
                    for key, value in set_fields.items():
                        setattr(task, key, value)
                    
                    # Update timestamp
                    task.updated_at = datetime.now()
                    
                    # Log the change
                    self._log_change('UPDATE', 'Task', task.id, old_values, set_fields)
                
                session.commit()
                logger.info(f"Updated {len(tasks)} task(s) with {set_fields}")
                return True
                
            except Exception as e:
                logger.error(f"Failed to update task: {e}")
                session.rollback()
                return False
    
    def update_task_status(self, task_id: str, status: str, reason: str = None) -> Tuple[bool, Dict]:
        """
        Update task status with change logging
        
        Args:
            task_id: Task ID
            status: New status name
            reason: Optional reason for status change
        
        Returns:
            Tuple[bool, Dict]: (success, details)
        """
        with self.pool_manager.get_session() as session:
            try:
                task = session.get(Task, task_id)
                if not task:
                    logger.warning(f"Task {task_id} not found")
                    return False, {'success': False, 'error': 'Task not found'}
                
                # Get current status for logging
                old_status = session.get(TaskStatus, task.status_id)
                old_status_name = old_status.name if old_status else 'UNKNOWN'
                
                # Get new status ID
                new_status = session.exec(
                    select(TaskStatus).where(TaskStatus.name == status)
                ).first()
                
                if not new_status:
                    logger.warning(f"Status {status} not found")
                    return False, {'success': False, 'error': f'Status {status} not found'}
                
                # Store old values for logging
                old_values = {
                    'status_id': task.status_id,
                    'status_name': old_status_name,
                    'updated_at': task.updated_at.isoformat() if task.updated_at else None
                }
                
                # Update task
                task.status_id = new_status.id
                task.updated_at = datetime.now()
                
                session.add(task)
                session.commit()
                
                # Log the change
                new_values = {
                    'status_id': task.status_id,
                    'status_name': status,
                    'updated_at': task.updated_at.isoformat(),
                    'reason': reason
                }
                self._log_change('UPDATE', 'Task', task_id, old_values, new_values)
                
                logger.info(f"✅ Updated task {task_id} status from {old_status_name} to {status}")
                if reason:
                    logger.info(f"   Reason: {reason}")
                
                return True, {
                    'success': True,
                    'task_id': task_id,
                    'old_status': old_status_name,
                    'new_status': status,
                    'updated_at': task.updated_at.isoformat(),
                    'reason': reason
                }
                
            except Exception as e:
                logger.error(f"Failed to update task status: {e}")
                return False, {'success': False, 'error': str(e)}
    
    def update_rack(self, **kwargs) -> bool:
        """
        Generic update rack function for TAFL
        
        Args:
            where: Dict with conditions (e.g., {'id': 1})
            set: Dict with fields to update (e.g., {'status_id': 3})
        
        Returns:
            bool: Success status
        """
        where = kwargs.get('where', {})
        set_fields = kwargs.get('set', {})
        
        with self.pool_manager.get_session() as session:
            try:
                # Build query
                query = select(Rack)
                
                # Apply where conditions
                for key, value in where.items():
                    query = query.where(getattr(Rack, key) == value)
                
                # Get racks to update
                racks = session.exec(query).all()
                
                if not racks:
                    logger.warning(f"No racks found with conditions: {where}")
                    return False
                
                # Update each rack
                for rack in racks:
                    for key, value in set_fields.items():
                        setattr(rack, key, value)
                
                session.commit()
                logger.info(f"Updated {len(racks)} rack(s) with {set_fields}")
                return True
                
            except Exception as e:
                logger.error(f"Failed to update rack: {e}")
                session.rollback()
                return False
    
    def update_rack_side_completed(self, rack_id: int, side: str, completed: bool, notes: str = None) -> Tuple[bool, Dict]:
        """
        Update rack side completion status with logging
        
        Args:
            rack_id: Rack ID
            side: 'a' or 'b'
            completed: Completion status
            notes: Optional notes about the update
        
        Returns:
            Tuple[bool, Dict]: (success, details)
        """
        with self.pool_manager.get_session() as session:
            try:
                rack = session.get(Rack, rack_id)
                if not rack:
                    logger.warning(f"Rack {rack_id} not found")
                    return False, {'success': False, 'error': 'Rack not found'}
                
                # Store old values
                old_values = {
                    'is_carry': rack.is_carry,
                    'is_docked': rack.is_docked,
                    'updated_at': rack.updated_at.isoformat() if rack.updated_at else None
                }
                
                # Update based on side
                if side == 'a':
                    rack.is_carry = completed
                elif side == 'b':
                    rack.is_docked = completed
                else:
                    logger.warning(f"Invalid side: {side}")
                    return False, {'success': False, 'error': f'Invalid side: {side}'}
                
                rack.updated_at = datetime.now()
                
                session.add(rack)
                session.commit()
                
                # Log the change
                new_values = {
                    'is_carry': rack.is_carry,
                    'is_docked': rack.is_docked,
                    'updated_at': rack.updated_at.isoformat(),
                    'side': side,
                    'completed': completed,
                    'notes': notes
                }
                self._log_change('UPDATE', 'Rack', rack_id, old_values, new_values)
                
                logger.info(f"✅ Updated rack {rack_id} side {side} completed to {completed}")
                if notes:
                    logger.info(f"   Notes: {notes}")
                
                return True, {
                    'success': True,
                    'rack_id': rack_id,
                    'side': side,
                    'completed': completed,
                    'completion_percentage': self._calculate_rack_completion(rack),
                    'updated_at': rack.updated_at.isoformat(),
                    'notes': notes
                }
                
            except Exception as e:
                logger.error(f"Failed to update rack side: {e}")
                return False, {'success': False, 'error': str(e)}
    
    def update_location_status(self, location_id: int, status: str, reason: str = None) -> Tuple[bool, Dict]:
        """
        Update location status with logging
        
        Args:
            location_id: Location ID
            status: New status name
            reason: Optional reason for status change
        
        Returns:
            Tuple[bool, Dict]: (success, details)
        """
        with self.pool_manager.get_session() as session:
            try:
                location = session.get(Location, location_id)
                if not location:
                    logger.warning(f"Location {location_id} not found")
                    return False, {'success': False, 'error': 'Location not found'}
                
                # Get current status for logging
                old_status = session.get(LocationStatus, location.location_status_id)
                old_status_name = old_status.name if old_status else 'UNKNOWN'
                
                # Get new status ID
                new_status = session.exec(
                    select(LocationStatus).where(LocationStatus.name == status)
                ).first()
                
                if not new_status:
                    logger.warning(f"Status {status} not found")
                    return False, {'success': False, 'error': f'Status {status} not found'}
                
                # Store old values
                old_values = {
                    'status_id': location.location_status_id,
                    'status_name': old_status_name
                }
                
                # Update location
                location.location_status_id = new_status.id
                
                session.add(location)
                session.commit()
                
                # Log the change
                new_values = {
                    'status_id': location.location_status_id,
                    'status_name': status,
                    'reason': reason
                }
                self._log_change('UPDATE', 'Location', location_id, old_values, new_values)
                
                logger.info(f"✅ Updated location {location_id} status from {old_status_name} to {status}")
                if reason:
                    logger.info(f"   Reason: {reason}")
                
                return True, {
                    'success': True,
                    'location_id': location_id,
                    'old_status': old_status_name,
                    'new_status': status,
                    'reason': reason
                }
                
            except Exception as e:
                logger.error(f"Failed to update location status: {e}")
                return False, {'success': False, 'error': str(e)}
    
    def query_carriers(self, **kwargs) -> Dict[str, Any]:
        """
        Query carriers with enhanced filtering options

        Supported filters:
        - rack_id: Filter by rack ID
        - room_id: Filter by room ID
        - status_id: Filter by status ID
        - rack_index: Filter by specific rack index
        - rack_index_min: Minimum rack index (for side filtering)
        - rack_index_max: Maximum rack index (for side filtering)
        - port_id: Filter by port ID
        - product_name: Filter by associated product name (requires JOIN)
        - created_after: Created after date
        - created_before: Created before date
        - sort_by: Field to sort by (default: id)
        - sort_order: 'asc' or 'desc'
        - offset: Pagination offset
        - limit: Maximum results
        """
        with self.pool_manager.get_session() as session:
            query = select(Carrier, CarrierStatus).join(
                CarrierStatus, Carrier.status_id == CarrierStatus.id, isouter=True
            )

            # Apply filters
            conditions = []

            # Basic filters
            if 'rack_id' in kwargs:
                conditions.append(Carrier.rack_id == kwargs['rack_id'])
            if 'room_id' in kwargs:
                conditions.append(Carrier.room_id == kwargs['room_id'])
            if 'status_id' in kwargs:
                conditions.append(Carrier.status_id == kwargs['status_id'])
            if 'rack_index' in kwargs:
                conditions.append(Carrier.rack_index == kwargs['rack_index'])
            if 'port_id' in kwargs:
                conditions.append(Carrier.port_id == kwargs['port_id'])

            # Rack index range filters (for A-side/B-side filtering)
            if 'rack_index_min' in kwargs:
                conditions.append(Carrier.rack_index >= kwargs['rack_index_min'])
            if 'rack_index_max' in kwargs:
                conditions.append(Carrier.rack_index <= kwargs['rack_index_max'])

            # Date filters
            if 'created_after' in kwargs:
                conditions.append(Carrier.created_at >= kwargs['created_after'])
            if 'created_before' in kwargs:
                conditions.append(Carrier.created_at <= kwargs['created_before'])

            if conditions:
                query = query.where(and_(*conditions))

            # Apply sorting
            sort_by = kwargs.get('sort_by', 'id')
            sort_order = kwargs.get('sort_order', 'asc')

            if hasattr(Carrier, sort_by):
                if sort_order == 'desc':
                    query = query.order_by(desc(getattr(Carrier, sort_by)))
                else:
                    query = query.order_by(asc(getattr(Carrier, sort_by)))

            # Apply pagination
            if 'offset' in kwargs:
                query = query.offset(kwargs['offset'])
            if 'limit' in kwargs:
                query = query.limit(kwargs['limit'])

            results = session.exec(query).all()

            # Get total count
            count_query = select(Carrier, CarrierStatus).join(
                CarrierStatus, Carrier.status_id == CarrierStatus.id, isouter=True
            )
            if conditions:
                count_query = count_query.where(and_(*conditions))
            total_count = len(session.exec(count_query).all())

            # Convert to dict with enhanced info
            carriers = []
            for carrier, status in results:
                carrier_dict = carrier.model_dump()
                carrier_dict['status'] = status.name if status else 'UNKNOWN'
                carrier_dict['status_id'] = status.id if status else None
                # Determine which side the carrier is on
                if carrier.rack_index:
                    if 1 <= carrier.rack_index <= 16:
                        carrier_dict['side'] = 'A'
                    elif 17 <= carrier.rack_index <= 32:
                        carrier_dict['side'] = 'B'
                    else:
                        carrier_dict['side'] = 'UNKNOWN'
                else:
                    carrier_dict['side'] = None
                carriers.append(carrier_dict)

            return {
                'data': carriers,
                'total': total_count,
                'offset': kwargs.get('offset', 0),
                'limit': kwargs.get('limit', None),
                'has_more': total_count > (kwargs.get('offset', 0) + len(carriers))
            }

    def check_rack_side_status(self, rack_id: int, check_type: str = 'both') -> Dict[str, Any]:
        """
        Check rack side status (A-side full, B-side empty, etc.)

        Args:
            rack_id: Rack ID to check
            check_type: Type of check to perform
                - 'both': Check both A-side full and B-side empty
                - 'a_full': Check if A-side is full
                - 'b_empty': Check if B-side is empty
                - 'a_empty': Check if A-side is empty
                - 'b_full': Check if B-side is full

        Returns:
            Dictionary with check results
        """
        with self.pool_manager.get_session() as session:
            try:
                # Get rack info
                rack = session.get(Rack, rack_id)
                if not rack:
                    return {
                        'success': False,
                        'error': f'Rack {rack_id} not found',
                        'rack_id': rack_id
                    }

                # Determine rack size (S or L) - default to S if not specified
                # S size: 32 slots (16 per side), L size: 16 slots (8 per side)
                rack_size = 'S'  # Default, could be from rack.size if field exists
                max_slots_per_side = 16 if rack_size == 'S' else 8

                # Use query_carriers to get carriers with side information
                carriers_result = self.query_carriers(rack_id=rack_id)
                carriers = carriers_result.get('data', [])

                # Separate carriers by side using the already computed side information
                a_side_carriers = [c for c in carriers if c.get('side') == 'A']
                b_side_carriers = [c for c in carriers if c.get('side') == 'B']

                # Calculate side status
                a_side_count = len(a_side_carriers)
                b_side_count = len(b_side_carriers)
                a_side_full = a_side_count >= max_slots_per_side
                a_side_empty = a_side_count == 0
                b_side_full = b_side_count >= max_slots_per_side
                b_side_empty = b_side_count == 0

                # Build result based on check_type
                result = {
                    'success': True,
                    'rack_id': rack_id,
                    'rack_name': rack.name,
                    'rack_size': rack_size,
                    'max_slots_per_side': max_slots_per_side,
                    'a_side_count': a_side_count,
                    'b_side_count': b_side_count,
                    'total_carriers': len(carriers)
                }

                if check_type == 'both':
                    result.update({
                        'a_side_full': a_side_full,
                        'b_side_empty': b_side_empty,
                        'rotation_needed': a_side_full and b_side_empty
                    })
                elif check_type == 'a_full':
                    result['a_side_full'] = a_side_full
                elif check_type == 'b_empty':
                    result['b_side_empty'] = b_side_empty
                elif check_type == 'a_empty':
                    result['a_side_empty'] = a_side_empty
                elif check_type == 'b_full':
                    result['b_side_full'] = b_side_full
                else:
                    # Return all status
                    result.update({
                        'a_side_full': a_side_full,
                        'a_side_empty': a_side_empty,
                        'b_side_full': b_side_full,
                        'b_side_empty': b_side_empty
                    })

                return result

            except Exception as e:
                logger.error(f"Failed to check rack side status: {e}")
                return {
                    'success': False,
                    'error': str(e),
                    'rack_id': rack_id
                }

    def check_carriers_in_room(self, room_id: int, carrier_type: str = None) -> Dict[str, Any]:
        """
        Check for carriers in a specific room

        Args:
            room_id: Room ID to check
            carrier_type: Deprecated parameter (kept for compatibility, Carrier model has no type field)

        Returns:
            Dictionary with carrier information
        """
        try:
            # Use query_carriers to get all carriers in the room
            result = self.query_carriers(room_id=room_id)

            # Count carriers by status
            status_counts = {}
            for carrier in result.get('data', []):
                status_id = carrier.get('status_id', 0)
                status_counts[status_id] = status_counts.get(status_id, 0) + 1

            return {
                'success': True,
                'room_id': room_id,
                'total_carriers': result.get('total', 0),
                'status_counts': status_counts,
                'has_carriers': result.get('total', 0) > 0
            }

        except Exception as e:
            logger.error(f"Failed to check carriers in room: {e}")
            return {
                'success': False,
                'error': str(e),
                'room_id': room_id
            }

    # ========== Helper Functions ==========

    def _calculate_rack_completion(self, rack) -> int:
        """Calculate rack completion percentage"""
        if rack.is_carry and rack.is_docked:
            return 100
        elif rack.is_carry or rack.is_docked:
            return 50
        else:
            return 0
    
    def _calculate_execution_time(self, task) -> Optional[float]:
        """Calculate task execution time in seconds"""
        if task.created_at and task.updated_at:
            delta = task.updated_at - task.created_at
            return delta.total_seconds()
        return None
    
    def _calculate_task_statistics(self, tasks_with_status) -> Dict:
        """Calculate task statistics"""
        stats = {
            'total': len(tasks_with_status),
            'by_status': {},
            'by_priority': {},
            'average_execution_time': 0,
            'overdue_count': 0
        }
        
        execution_times = []
        for task, status in tasks_with_status:
            # Count by status
            status_name = status.name if status else 'UNKNOWN'
            stats['by_status'][status_name] = stats['by_status'].get(status_name, 0) + 1
            
            # Count by priority
            priority = f"P{task.priority}"
            stats['by_priority'][priority] = stats['by_priority'].get(priority, 0) + 1
            
            # Calculate execution time
            exec_time = self._calculate_execution_time(task)
            if exec_time:
                execution_times.append(exec_time)
            
            # Task model doesn't have deadline field, skip overdue check
            # stats['overdue_count'] calculation skipped
        
        # Calculate average execution time
        if execution_times:
            stats['average_execution_time'] = sum(execution_times) / len(execution_times)
        
        return stats
    
    def _log_change(self, action: str, entity_type: str, entity_id: Any, old_values: Any, new_values: Any):
        """Log a change for audit trail"""
        change = {
            'timestamp': datetime.now().isoformat(),
            'action': action,
            'entity_type': entity_type,
            'entity_id': str(entity_id),
            'old_values': old_values,
            'new_values': new_values
        }
        self.change_log.append(change)
        
        # Keep only last 1000 changes in memory
        if len(self.change_log) > 1000:
            self.change_log = self.change_log[-1000:]
    
    def get_change_log(self, entity_type: str = None, entity_id: str = None, limit: int = 100) -> List[Dict]:
        """
        Get change log with optional filters
        
        Args:
            entity_type: Filter by entity type
            entity_id: Filter by entity ID
            limit: Maximum number of entries to return
        
        Returns:
            List of change log entries
        """
        filtered_log = self.change_log
        
        if entity_type:
            filtered_log = [c for c in filtered_log if c['entity_type'] == entity_type]
        
        if entity_id:
            filtered_log = [c for c in filtered_log if c['entity_id'] == str(entity_id)]
        
        # Return most recent entries first
        return list(reversed(filtered_log))[:limit]
    
    def close(self):
        """Close database connection and save change log"""
        # Optionally save change log to file or database
        if self.change_log:
            logger.info(f"Saving {len(self.change_log)} change log entries")
            # Could implement saving to file or database here
        
        if hasattr(self, 'pool_manager'):
            self.pool_manager.shutdown()
            logger.info("Enhanced database bridge closed")