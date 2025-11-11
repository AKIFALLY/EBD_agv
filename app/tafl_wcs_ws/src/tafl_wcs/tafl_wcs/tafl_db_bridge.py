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
sys.path.insert(0, '/app/web_api_ws/install/agvcui/lib/python3.12/site-packages')
sys.path.insert(0, '/app/kuka_fleet_ws/install/kuka_fleet_adapter/lib/python3.12/site-packages')

from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import (
    Location, LocationStatus,
    Rack, RackStatus,
    Carrier, CarrierStatus,
    Task, TaskStatus,
    Work,
    Room,
    Product, ProcessSettings
)

# KUKA 同步服務（用於 TAFL 流程中的 rack 更新）
try:
    from agvcui.services.kuka_sync_service import KukaContainerSyncService
    KUKA_SYNC_AVAILABLE = True
except ImportError as e:
    logging.warning(f"KUKA sync service not available: {e}")
    KUKA_SYNC_AVAILABLE = False
from db_proxy.models.agvc_eqp import Eqp, EqpPort, EqpSignal
from db_proxy.models.agvc_eqp import Eqp, EqpPort, EqpSignal
from db_proxy.models.agvc_eqp import Eqp, EqpPort, EqpSignal
from sqlmodel import Session, select, and_, or_, desc, asc

logger = logging.getLogger(__name__)


class SortOrder(Enum):
    """Sort order enumeration"""
    ASC = "asc"
    DESC = "desc"


class BaseQueryMixin:
    """Base mixin class for unified query filtering logic"""

    def _apply_status_filters(self, model, status_model, conditions, kwargs):
        """
        Apply unified status filtering

        Args:
            model: The main model (e.g., Task, Carrier, Rack)
            status_model: The status model (e.g., TaskStatus, CarrierStatus) or None
            conditions: List to append conditions to
            kwargs: Query parameters
        """
        # Single status ID
        if 'status_id' in kwargs:
            conditions.append(model.status_id == kwargs['status_id'])

        # Multiple status IDs (include)
        if 'status_id_in' in kwargs:
            status_ids = kwargs['status_id_in']
            if isinstance(status_ids, list):
                conditions.append(model.status_id.in_(status_ids))

        # Single status ID (exclude)
        if 'status_id_not' in kwargs:
            conditions.append(model.status_id != kwargs['status_id_not'])

        # Multiple status IDs (exclude)
        if 'status_id_not_in' in kwargs:
            status_ids = kwargs['status_id_not_in']
            if isinstance(status_ids, list):
                conditions.append(~model.status_id.in_(status_ids))

        # Status name filters (if status model exists)
        if status_model:
            # Single status name
            if 'status' in kwargs:
                conditions.append(status_model.name == kwargs['status'])

            # Multiple status names (include)
            if 'status_list' in kwargs:
                status_list = kwargs['status_list']
                if isinstance(status_list, list):
                    conditions.append(status_model.name.in_(status_list))

            # Single status name (exclude)
            if 'status_not' in kwargs:
                conditions.append(status_model.name != kwargs['status_not'])

            # Multiple status names (exclude)
            if 'status_not_list' in kwargs:
                status_list = kwargs['status_not_list']
                if isinstance(status_list, list):
                    conditions.append(~status_model.name.in_(status_list))

    def _apply_date_filters(self, model, conditions, kwargs):
        """
        Apply unified date filtering

        Args:
            model: The model with date fields
            conditions: List to append conditions to
            kwargs: Query parameters
        """
        # Created date filters
        if 'created_after' in kwargs:
            conditions.append(model.created_at >= kwargs['created_after'])
        if 'created_before' in kwargs:
            conditions.append(model.created_at <= kwargs['created_before'])

        # Updated date filters (if model has updated_at)
        if hasattr(model, 'updated_at'):
            if 'updated_after' in kwargs:
                conditions.append(model.updated_at >= kwargs['updated_after'])
            if 'updated_before' in kwargs:
                conditions.append(model.updated_at <= kwargs['updated_before'])

    def _apply_pagination_and_sorting(self, query, model, kwargs):
        """
        Apply unified pagination and sorting

        Args:
            query: SQLAlchemy query
            model: The model being queried
            kwargs: Query parameters containing sort_by, sort_order, offset, limit

        Returns:
            Modified query with sorting and pagination applied
        """
        # Sorting
        sort_by = kwargs.get('sort_by', 'id')
        sort_order = kwargs.get('sort_order', 'asc')

        if hasattr(model, sort_by):
            sort_column = getattr(model, sort_by)
            if sort_order == 'desc':
                query = query.order_by(desc(sort_column))
            else:
                query = query.order_by(asc(sort_column))

        # Pagination
        if 'offset' in kwargs:
            query = query.offset(kwargs['offset'])
        if 'limit' in kwargs:
            query = query.limit(kwargs['limit'])

        return query

    def _apply_numeric_range_filters(self, model, field_name, conditions, kwargs):
        """
        Apply numeric range filters (min/max)

        Args:
            model: The model being queried
            field_name: Field name to filter on
            conditions: List to append conditions to
            kwargs: Query parameters
        """
        min_key = f"{field_name}_min"
        max_key = f"{field_name}_max"

        if min_key in kwargs and hasattr(model, field_name):
            field = getattr(model, field_name)
            conditions.append(field >= kwargs[min_key])

        if max_key in kwargs and hasattr(model, field_name):
            field = getattr(model, field_name)
            conditions.append(field <= kwargs[max_key])


class TAFLDatabaseBridge(BaseQueryMixin):
    """Enhanced database bridge for TAFL operations"""
    
    def __init__(self, database_url: str):
        """Initialize database connection"""
        self.pool_manager = ConnectionPoolManager(database_url)
        self.change_log = []  # Store change history
        self.logger = logger  # Add logger attribute
        logger.info("✅ Enhanced Database bridge initialized")
    
    # ========== Enhanced Query Functions ==========
    
    def query_locations(self, **kwargs) -> Dict[str, Any]:
        """
        Query locations with unified enhanced filtering options

        Basic filters:
        - id: Filter by single location ID
        - id_in: Filter by multiple location IDs (list)
        - room_id: Filter by room ID
        - node_id: Filter by node ID
        - type: Filter by location type (e.g., 'room_inlet', 'room_outlet')
        - available_only: Only return available locations
        - occupied_only: Only return occupied locations

        Enhanced status filters (NEW - unified standard):
        - status_id: Single status ID
        - status_id_in: List of status IDs to include [1, 2, 3]
        - status_id_not: Single status ID to exclude
        - status_id_not_in: List of status IDs to exclude [7, 8]
        - status: Single status name
        - status_list: List of status names to include
        - status_not: Single status name to exclude
        - status_not_list: List of status names to exclude

        Date filters:
        - created_after: Created after date
        - created_before: Created before date
        - updated_after: Updated after date
        - updated_before: Updated before date

        Pagination and sorting:
        - sort_by: Field to sort by (default: id)
        - sort_order: 'asc' or 'desc'
        - offset: Pagination offset
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
            if 'id_in' in kwargs:
                id_list = kwargs['id_in']
                if isinstance(id_list, (list, tuple)):
                    conditions.append(Location.id.in_(id_list))
            if 'room_id' in kwargs:
                if kwargs['room_id'] is None:
                    conditions.append(Location.room_id.is_(None))
                else:
                    conditions.append(Location.room_id == kwargs['room_id'])
            if 'node_id' in kwargs:
                if kwargs['node_id'] is None:
                    conditions.append(Location.node_id.is_(None))
                else:
                    conditions.append(Location.node_id == kwargs['node_id'])
            if 'type' in kwargs:
                conditions.append(Location.type == kwargs['type'])

            # Direct location_status_id filter (for Location model)
            if 'location_status_id' in kwargs:
                conditions.append(Location.location_status_id == kwargs['location_status_id'])

            # Special filters
            if kwargs.get('available_only'):
                conditions.append(LocationStatus.name == 'AVAILABLE')
            if kwargs.get('occupied_only'):
                conditions.append(LocationStatus.name == 'OCCUPIED')

            # Apply unified status filters
            self._apply_status_filters(Location, LocationStatus, conditions, kwargs)

            # Apply unified date filters
            self._apply_date_filters(Location, conditions, kwargs)

            if conditions:
                query = query.where(and_(*conditions))

            # Apply unified pagination and sorting
            query = self._apply_pagination_and_sorting(query, Location, kwargs)
            
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
        Query racks with unified enhanced filtering options

        Basic filters:
        - location_id: Filter by single location ID
        - location_id_in: Filter by multiple location IDs (list)
        - product_id: Filter by product ID
        - room_id: Filter by room ID
        - is_carry: Filter by carry status
        - is_docked: Filter by docked status
        - needs_rotation: Only racks needing rotation
        - completed_only: Only completed racks
        - in_progress_only: Only in-progress racks

        Enhanced status filters (NEW - unified standard):
        - status_id: Single status ID
        - status_id_in: List of status IDs to include [1, 2, 3]
        - status_id_not: Single status ID to exclude
        - status_id_not_in: List of status IDs to exclude [7, 8]
        - status: Single status name
        - status_list: List of status names to include
        - status_not: Single status name to exclude
        - status_not_list: List of status names to exclude

        Date filters:
        - created_after: Created after date
        - created_before: Created before date
        - updated_after: Updated after date
        - updated_before: Updated before date

        Pagination and sorting:
        - sort_by: Field to sort by (default: id)
        - sort_order: 'asc' or 'desc'
        - offset: Pagination offset
        - limit: Maximum results
        """
        with self.pool_manager.get_session() as session:
            query = select(Rack, RackStatus).join(
                RackStatus, Rack.status_id == RackStatus.id, isouter=True
            )

            # Apply filters
            conditions = []

            # Basic filters
            if 'id' in kwargs:
                conditions.append(Rack.id == kwargs['id'])
            if 'location_id' in kwargs:
                conditions.append(Rack.location_id == kwargs['location_id'])
            if 'location_id_in' in kwargs:
                location_ids = kwargs['location_id_in']
                if isinstance(location_ids, list):
                    conditions.append(Rack.location_id.in_(location_ids))
            if 'product_id' in kwargs:
                if kwargs['product_id'] is None:
                    conditions.append(Rack.product_id.is_(None))
                else:
                    conditions.append(Rack.product_id == kwargs['product_id'])
            if 'room_id' in kwargs:
                self.logger.info(f"DEBUG query_racks: room_id filter = {kwargs['room_id']} (type: {type(kwargs['room_id'])})")
                if kwargs['room_id'] is None:
                    self.logger.info("DEBUG query_racks: Adding condition Rack.room_id IS NULL")
                    conditions.append(Rack.room_id.is_(None))
                else:
                    self.logger.info(f"DEBUG query_racks: Adding condition Rack.room_id = {kwargs['room_id']}")
                    conditions.append(Rack.room_id == kwargs['room_id'])
            if 'is_carry' in kwargs:
                conditions.append(Rack.is_carry == kwargs['is_carry'])
            if 'is_docked' in kwargs:
                conditions.append(Rack.is_docked == kwargs['is_docked'])

            # Special filters
            if kwargs.get('needs_rotation'):
                conditions.append(and_(Rack.is_carry == True, Rack.is_docked == False))

            if kwargs.get('completed_only'):
                conditions.append(and_(Rack.is_carry == True, Rack.is_docked == True))

            if kwargs.get('in_progress_only'):
                conditions.append(or_(
                    and_(Rack.is_carry == True, Rack.is_docked == False),
                    and_(Rack.is_carry == False, Rack.is_docked == False)
                ))

            # Apply unified status filters
            self._apply_status_filters(Rack, RackStatus, conditions, kwargs)

            # Apply unified date filters
            self._apply_date_filters(Rack, conditions, kwargs)

            if conditions:
                query = query.where(and_(*conditions))

            # Apply unified pagination and sorting
            query = self._apply_pagination_and_sorting(query, Rack, kwargs)
            
            results = session.exec(query).all()

            self.logger.info(f"DEBUG query_racks: Query returned {len(results)} results")
            for rack, status in results[:3]:  # Log first 3 for debugging
                self.logger.info(f"DEBUG query_racks: Result - Rack {rack.id}: room_id={rack.room_id}")

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
        Query tasks with unified enhanced filtering options

        Basic filters:
        - rack_id: Filter by rack ID
        - work_id: Filter by work ID
        - room_id: Filter by room ID
        - room_id: Filter by room ID
        - priority: Filter by priority
        - assigned_to: Filter by assignee

        Enhanced status filters (NEW - unified standard):
        - status_id: Single status ID
        - status_id_in: List of status IDs to include [1, 2, 3]
        - status_id_not: Single status ID to exclude
        - status_id_not_in: List of status IDs to exclude [7, 8]
        - status: Single status name
        - status_list: List of status names to include
        - status_not: Single status name to exclude
        - status_not_list: List of status names to exclude

        Date filters:
        - created_after: Created after date
        - created_before: Created before date
        - updated_after: Updated after date
        - updated_before: Updated before date

        Numeric range filters:
        - priority_min: Minimum priority
        - priority_max: Maximum priority

        Special filters:
        - pending_only: Only pending tasks (PENDING, QUEUED)
        - active_only: Only active tasks (IN_PROGRESS, RUNNING)

        Pagination and sorting:
        - sort_by: Field to sort by (default: priority,created_at)
        - sort_order: 'asc' or 'desc' (default: desc,asc)
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
            if 'room_id' in kwargs:
                if kwargs['room_id'] is None:
                    conditions.append(Task.room_id.is_(None))
                else:
                    conditions.append(Task.room_id == kwargs['room_id'])
            if 'room_id' in kwargs:
                if kwargs['room_id'] is None:
                    conditions.append(Task.room_id.is_(None))
                else:
                    conditions.append(Task.room_id == kwargs['room_id'])
            if 'priority' in kwargs:
                conditions.append(Task.priority == kwargs['priority'])
            if 'assigned_to' in kwargs:
                conditions.append(Task.assigned_to == kwargs['assigned_to'])

            # Apply numeric range filters for priority
            self._apply_numeric_range_filters(Task, 'priority', conditions, kwargs)

            # Apply unified status filters
            self._apply_status_filters(Task, TaskStatus, conditions, kwargs)

            # Apply unified date filters
            self._apply_date_filters(Task, conditions, kwargs)
            
            # Special filters
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
        Query works with unified enhanced filtering options

        Basic filters:
        - id: Filter by work ID
        - enabled: Filter by enabled status
        - category: Filter by work category

        Date filters:
        - created_after: Created after date
        - created_before: Created before date
        - updated_after: Updated after date (if Work has updated_at)
        - updated_before: Updated before date (if Work has updated_at)

        Pagination and sorting:
        - sort_by: Field to sort by (default: id)
        - sort_order: 'asc' or 'desc'
        - offset: Pagination offset
        - limit: Maximum results

        Note: Work model doesn't have status_id field, only 'enabled' field
        """
        with self.pool_manager.get_session() as session:
            query = select(Work)

            # Apply filters
            conditions = []

            # Basic filters
            if 'id' in kwargs:
                conditions.append(Work.id == kwargs['id'])
            if 'enabled' in kwargs:
                conditions.append(Work.enabled == kwargs['enabled'])
            if 'category' in kwargs:
                conditions.append(Work.category == kwargs['category'])

            # Apply unified date filters
            self._apply_date_filters(Work, conditions, kwargs)

            if conditions:
                query = query.where(and_(*conditions))

            # Apply unified pagination and sorting
            query = self._apply_pagination_and_sorting(query, Work, kwargs)
            
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
        """
        Query AGVs with unified enhanced filtering options

        Basic filters:
        - id: Filter by AGV ID
        - name: Filter by AGV name
        - model: Filter by AGV model
        - enable: Filter by enable status (0/1)
        - last_node_id: Filter by last node ID

        Enhanced status filters (NEW - unified standard):
        - status_id: Single status ID
        - status_id_in: List of status IDs to include [1, 2, 3]
        - status_id_not: Single status ID to exclude
        - status_id_not_in: List of status IDs to exclude [7, 8]

        Location filters:
        - x_min/x_max: X coordinate range
        - y_min/y_max: Y coordinate range
        - battery_min/battery_max: Battery level range

        Date filters:
        - created_after: Created after date
        - created_before: Created before date
        - updated_after: Updated after date
        - updated_before: Updated before date

        Pagination and sorting:
        - sort_by: Field to sort by (default: id)
        - sort_order: 'asc' or 'desc'
        - offset: Pagination offset
        - limit: Maximum results

        Returns:
            Dictionary with 'data', 'total', pagination info
        """
        with self.pool_manager.get_session() as session:
            # Import models
            from db_proxy.models.agvc_rcs import AGV
            from db_proxy.models.agv_status import AgvStatus

            # Build modern query with optional status join
            query = select(AGV, AgvStatus).join(
                AgvStatus, AGV.status_id == AgvStatus.id, isouter=True
            )

            # Apply filters
            conditions = []

            # Basic filters
            if 'id' in kwargs:
                conditions.append(AGV.id == kwargs['id'])
            if 'name' in kwargs:
                conditions.append(AGV.name == kwargs['name'])
            if 'model' in kwargs:
                conditions.append(AGV.model == kwargs['model'])
            if 'enable' in kwargs:
                # Handle boolean to integer conversion for enable field
                # PostgreSQL expects integer (0/1) but TAFL may pass boolean
                enable_value = kwargs['enable']
                if isinstance(enable_value, bool):
                    enable_value = 1 if enable_value else 0
                elif isinstance(enable_value, str):
                    # Handle string representations of boolean
                    enable_value = 1 if enable_value.lower() in ['true', '1', 'yes'] else 0
                conditions.append(AGV.enable == enable_value)
            if 'last_node_id' in kwargs:
                conditions.append(AGV.last_node_id == kwargs['last_node_id'])

            # Apply unified status filters
            self._apply_status_filters(AGV, AgvStatus, conditions, kwargs)

            # Apply numeric range filters
            self._apply_numeric_range_filters(AGV, 'x', conditions, kwargs)
            self._apply_numeric_range_filters(AGV, 'y', conditions, kwargs)
            self._apply_numeric_range_filters(AGV, 'battery', conditions, kwargs)

            # Apply unified date filters
            self._apply_date_filters(AGV, conditions, kwargs)

            if conditions:
                query = query.where(and_(*conditions))

            # Apply unified pagination and sorting
            query = self._apply_pagination_and_sorting(query, AGV, kwargs)

            results = session.exec(query).all()

            # Get total count
            count_query = select(AGV, AgvStatus).join(
                AgvStatus, AGV.status_id == AgvStatus.id, isouter=True
            )
            if conditions:
                count_query = count_query.where(and_(*conditions))
            total_count = len(session.exec(count_query).all())

            # Convert to dictionaries with enhanced info
            agvs = []
            for agv, status in results:
                agv_dict = {
                    'id': agv.id,
                    'name': agv.name,
                    'description': agv.description,
                    'model': agv.model,
                    'x': agv.x,
                    'y': agv.y,
                    'heading': agv.heading,
                    'battery': agv.battery if agv.battery is not None else 0,
                    'last_node_id': agv.last_node_id,
                    'enable': agv.enable,
                    'status_id': agv.status_id,
                    'status': status.name if status else 'UNKNOWN'
                }
                agvs.append(agv_dict)

            return {
                'data': agvs,
                'total': total_count,
                'offset': kwargs.get('offset', 0),
                'limit': kwargs.get('limit', None),
                'has_more': total_count > (kwargs.get('offset', 0) + len(agvs))
            }
    
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

        Args:
            task_id: Task primary key ID (as string)

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
        - room_id: Associated room ID
        - room_id: Associated room ID
        - status_id: Initial status (default: PENDING)
        - metadata: Additional metadata
        - assigned_to: Assignee
        - deadline: Task deadline

        Returns:
            Tuple[str, Dict]: (task.id as string, details)
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
                
                # Prepare parameters to store extra fields
                task_params = params.get('parameters', {})

                # Store source and target location IDs in parameters if provided
                if 'source_location_id' in params:
                    task_params['source_location_id'] = params['source_location_id']
                if 'target_location_id' in params:
                    task_params['target_location_id'] = params['target_location_id']

                # Create task
                task = Task(
                    name=params.get('name', f"Task for Work {params['work_id']}"),
                    description=params.get('description'),  # Add description field
                    type=params.get('type'),  # Add type field for task categorization
                    work_id=params['work_id'],
                    rack_id=params.get('rack_id'),  # Add rack_id from params
                    room_id=params.get('room_id'),  # Add room_id from params
                    location_id=params.get('target_location_id'),  # Use target as main location
                    priority=priority,
                    status_id=params.get('status_id', TaskStatus.PENDING),  # Use status_id from params, default to PENDING
                    parameters=task_params,  # Store extended parameters including source/target locations
                    # deadline field not available in Task model
                    created_at=datetime.now()
                )
                
                session.add(task)
                session.commit()
                session.refresh(task)

                # Auto-update location status for KUKA move rack tasks
                # Work ID 210001 = KUKA move rack (room dispatch)
                if params.get('work_id') == 210001 and params.get('target_location_id'):
                    try:
                        target_location = session.get(Location, params['target_location_id'])
                        if target_location and target_location.location_status_id == 2:  # IDLE
                            old_status_id = target_location.location_status_id
                            target_location.location_status_id = 3  # OCCUPIED
                            session.commit()
                            logger.info(f"✅ Auto-updated location {params['target_location_id']} status from {old_status_id} to 3 (OCCUPIED) for KUKA move rack task")
                    except Exception as e:
                        logger.warning(f"Failed to auto-update location status: {e}")
                        # Don't fail task creation if location update fails

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
            task_id: Task primary key ID (as string)
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
        Generic update rack function for TAFL（整合 KUKA Container 同步）

        自動同步到 KUKA Fleet Manager：
        - is_in_map: 0→1 時調用 container_in（入場）
        - is_in_map: 1→0 時調用 container_out（出場）
        - location_id 變更時調用 update_container（位置更新）

        Args:
            where: Dict with conditions (e.g., {'id': 1})
            set/data: Dict with fields to update (e.g., {'status_id': 3})

        Returns:
            bool: Success status
        """
        where = kwargs.get('where', {})
        # Support both 'set' and 'data' for TAFL compatibility
        set_fields = kwargs.get('set', kwargs.get('data', {}))

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

                # 儲存舊狀態用於比對變更
                old_racks = {rack.id: {
                    'location_id': rack.location_id,
                    'is_in_map': rack.is_in_map
                } for rack in racks}

                # Update each rack
                for rack in racks:
                    for key, value in set_fields.items():
                        setattr(rack, key, value)

                session.commit()
                logger.info(f"Updated {len(racks)} rack(s) with {set_fields}")

                # KUKA 同步（錯誤不影響資料庫更新）
                if KUKA_SYNC_AVAILABLE:
                    try:
                        kuka_sync = KukaContainerSyncService()

                        for rack in racks:
                            # 重建舊 rack 物件用於比對
                            old_rack_data = old_racks[rack.id]
                            old_rack = Rack(
                                id=rack.id,
                                name=rack.name,
                                location_id=old_rack_data['location_id'],
                                is_in_map=old_rack_data['is_in_map']
                            )

                            # 同步到 KUKA
                            sync_result = kuka_sync.sync_rack_to_kuka(old_rack, rack, session)

                            if sync_result["success"] and sync_result.get("action") != "skip":
                                logger.info(
                                    f"✅ TAFL→KUKA sync: Rack(id={rack.id}) | "
                                    f"Action: {sync_result.get('action')}"
                                )
                            elif not sync_result["success"]:
                                logger.warning(
                                    f"⚠️ TAFL→KUKA sync failed (DB updated): Rack(id={rack.id}) | "
                                    f"Error: {sync_result.get('error')}"
                                )
                    except Exception as e:
                        # KUKA 同步失敗不影響 TAFL 流程執行
                        logger.error(
                            f"❌ TAFL→KUKA sync error (DB updated): {str(e)}",
                            exc_info=True
                        )

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
    
    def update_location_status(self, location_id: int, status, reason: str = None) -> Tuple[bool, Dict]:
        """
        Update location status with logging

        Args:
            location_id: Location ID
            status: New status - can be status ID (int) or status name (str)
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

                # Get new status - support both ID (int) and name (str)
                if isinstance(status, int):
                    # If status is an integer, treat it as status ID
                    new_status = session.get(LocationStatus, status)
                    if not new_status:
                        logger.warning(f"Status ID {status} not found")
                        return False, {'success': False, 'error': f'Status ID {status} not found'}
                else:
                    # If status is a string, treat it as status name
                    new_status = session.exec(
                        select(LocationStatus).where(LocationStatus.name == str(status))
                    ).first()
                    if not new_status:
                        logger.warning(f"Status name '{status}' not found")
                        return False, {'success': False, 'error': f'Status name "{status}" not found'}
                
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
                    'status_name': new_status.name,  # Use the actual status name from database
                    'reason': reason
                }
                self._log_change('UPDATE', 'Location', location_id, old_values, new_values)
                
                logger.info(f"✅ Updated location {location_id} status from {old_status_name} to {new_status.name}")
                if reason:
                    logger.info(f"   Reason: {reason}")

                return True, {
                    'success': True,
                    'location_id': location_id,
                    'old_status': old_status_name,
                    'new_status': new_status.name,  # Return actual status name
                    'reason': reason
                }
                
            except Exception as e:
                logger.error(f"Failed to update location status: {e}")
                return False, {'success': False, 'error': str(e)}
    
    def query_carriers(self, **kwargs) -> Dict[str, Any]:
        """
        Query carriers with unified enhanced filtering options

        Basic filters:
        - rack_id: Filter by rack ID
        - room_id: Filter by room ID
        - rack_index: Filter by specific rack index
        - rack_index_min: Minimum rack index (for side filtering)
        - rack_index_max: Maximum rack index (for side filtering)
        - port_id: Filter by single port ID
        - port_in: Filter by multiple port IDs (list)
        - agv_id: Filter by AGV ID (carrier on AGV)
        - port_id: Filter by single port ID
        - port_in: Filter by multiple port IDs (list)
        - agv_id: Filter by AGV ID (carrier on AGV)
        - product_name: Filter by associated product name (requires JOIN)

        Enhanced status filters (NEW - unified standard):
        - status_id: Single status ID
        - status_id_in: List of status IDs to include [1, 2, 3]
        - status_id_not: Single status ID to exclude
        - status_id_not_in: List of status IDs to exclude [7, 8]
        - status: Single status name
        - status_list: List of status names to include
        - status_not: Single status name to exclude
        - status_not_list: List of status names to exclude

        Date filters:
        - created_after: Created after date
        - created_before: Created before date
        - updated_after: Updated after date
        - updated_before: Updated before date

        Pagination and sorting:
        - sort_by: Field to sort by (default: id)
        - sort_order: 'asc' or 'desc'
        - offset: Pagination offset
        - limit: Maximum results
        """
        with self.pool_manager.get_session() as session:
            # Simple query without unnecessary joins
            # Simple query without unnecessary joins
            query = select(Carrier, CarrierStatus).join(
                CarrierStatus, Carrier.status_id == CarrierStatus.id, isouter=True
            )

            # Apply filters
            conditions = []

            # Basic filters
            if 'rack_id' in kwargs:
                if kwargs['rack_id'] is None:
                    conditions.append(Carrier.rack_id.is_(None))
                else:
                    conditions.append(Carrier.rack_id == kwargs['rack_id'])
            if 'room_id' in kwargs:
                if kwargs['room_id'] is None:
                    conditions.append(Carrier.room_id.is_(None))
                else:
                    conditions.append(Carrier.room_id == kwargs['room_id'])
            if 'rack_index' in kwargs:
                conditions.append(Carrier.rack_index == kwargs['rack_index'])
            if 'port_id' in kwargs:
                conditions.append(Carrier.port_id == kwargs['port_id'])
            if 'port_in' in kwargs:
                port_list = kwargs['port_in']
                if isinstance(port_list, (list, tuple)):
                    conditions.append(Carrier.port_id.in_(port_list))
            if 'agv_id' in kwargs:
                # Filter by Carrier.agv_id directly (not through Rack)
                if kwargs['agv_id'] is None:
                    conditions.append(Carrier.agv_id.is_(None))
                else:
                    conditions.append(Carrier.agv_id == kwargs['agv_id'])
            if 'port_in' in kwargs:
                port_list = kwargs['port_in']
                if isinstance(port_list, (list, tuple)):
                    conditions.append(Carrier.port_id.in_(port_list))
            if 'agv_id' in kwargs:
                # Filter by Carrier.agv_id directly (not through Rack)
                if kwargs['agv_id'] is None:
                    conditions.append(Carrier.agv_id.is_(None))
                else:
                    conditions.append(Carrier.agv_id == kwargs['agv_id'])

            # Rack index range filters (for A-side/B-side filtering)
            self._apply_numeric_range_filters(Carrier, 'rack_index', conditions, kwargs)

            # Apply unified status filters
            self._apply_status_filters(Carrier, CarrierStatus, conditions, kwargs)

            # Apply unified date filters
            self._apply_date_filters(Carrier, conditions, kwargs)

            if conditions:
                query = query.where(and_(*conditions))

            # Apply unified pagination and sorting
            query = self._apply_pagination_and_sorting(query, Carrier, kwargs)

            results = session.exec(query).all()

            # Get total count - use same simple query structure
            # Get total count - use same simple query structure
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
    
    def query_rooms(self, **kwargs) -> Dict[str, Any]:
        """
        Query rooms with enhanced filtering options

        Basic filters:
        - id: Room ID
        - enable: Filter by enabled status (0 or 1)
        - process_settings_id: Filter by process settings ID
        - name: Filter by room name

        Pagination and sorting:
        - sort_by: Field to sort by (default: id)
        - sort_order: 'asc' or 'desc'
        - offset: Pagination offset
        - limit: Maximum results
        """
        with self.pool_manager.get_session() as session:
            query = select(Room)

            # Apply filters
            conditions = []

            # Basic filters
            if 'id' in kwargs:
                conditions.append(Room.id == kwargs['id'])
            if 'enable' in kwargs:
                conditions.append(Room.enable == kwargs['enable'])
            if 'process_settings_id' in kwargs:
                conditions.append(Room.process_settings_id == kwargs['process_settings_id'])
            if 'name' in kwargs:
                conditions.append(Room.name == kwargs['name'])

            # Apply conditions
            if conditions:
                query = query.where(and_(*conditions))

            # Apply sorting
            sort_by = kwargs.get('sort_by', 'id')
            sort_order = kwargs.get('sort_order', 'asc')

            if hasattr(Room, sort_by):
                order_column = getattr(Room, sort_by)
                if sort_order.lower() == 'desc':
                    query = query.order_by(desc(order_column))
                else:
                    query = query.order_by(asc(order_column))

            # Apply pagination
            if 'offset' in kwargs:
                query = query.offset(kwargs['offset'])
            if 'limit' in kwargs:
                query = query.limit(kwargs['limit'])

            # Execute query
            results = session.exec(query).all()

            # Get total count
            count_query = select(Room)
            if conditions:
                count_query = count_query.where(and_(*conditions))
            total_count = len(session.exec(count_query).all())

            # Convert to dict
            rooms = [room.model_dump() for room in results]

            return {
                'data': rooms,
                'total': total_count,
                'offset': kwargs.get('offset', 0),
                'limit': kwargs.get('limit', None),
                'has_more': total_count > (kwargs.get('offset', 0) + len(rooms))
            }

    def query_machine(self, **kwargs) -> Dict[str, Any]:
        """
        Query machines with filtering options

        Basic filters:
        - id: Machine ID
        - enable: Filter by enabled status (0 or 1)
        - name: Filter by machine name
        - parking_space_1: Filter by parking space 1 location ID
        - parking_space_2: Filter by parking space 2 location ID

        Array filters:
        - id_in: List of machine IDs

        Pagination and sorting:
        - sort_by: Field to sort by (default: id)
        - sort_order: 'asc' or 'desc'
        - offset: Pagination offset
        - limit: Maximum results
        """
        # Import Machine model
        from db_proxy.models.machine import Machine

        with self.pool_manager.get_session() as session:
            query = select(Machine)

            # Apply filters
            conditions = []

            # Basic filters
            if 'id' in kwargs:
                conditions.append(Machine.id == kwargs['id'])
            if 'enable' in kwargs:
                conditions.append(Machine.enable == kwargs['enable'])
            if 'name' in kwargs:
                conditions.append(Machine.name == kwargs['name'])
            if 'parking_space_1' in kwargs:
                conditions.append(Machine.parking_space_1 == kwargs['parking_space_1'])
            if 'parking_space_2' in kwargs:
                conditions.append(Machine.parking_space_2 == kwargs['parking_space_2'])

            # Array filters
            if 'id_in' in kwargs:
                id_list = kwargs['id_in']
                if isinstance(id_list, (list, tuple)):
                    conditions.append(Machine.id.in_(id_list))

            # Apply conditions
            if conditions:
                query = query.where(and_(*conditions))

            # Sorting
            sort_by = kwargs.get('sort_by', 'id')
            sort_order = kwargs.get('sort_order', 'asc')
            if hasattr(Machine, sort_by):
                order_field = getattr(Machine, sort_by)
                query = query.order_by(order_field if sort_order == 'asc' else order_field.desc())

            # Apply limit and offset
            if 'offset' in kwargs:
                query = query.offset(kwargs['offset'])
            if 'limit' in kwargs:
                query = query.limit(kwargs['limit'])

            # Execute query
            results = session.exec(query).all()

            # Get total count
            count_query = select(Machine)
            if conditions:
                count_query = count_query.where(and_(*conditions))
            total_count = len(session.exec(count_query).all())

            # Convert to dict
            machines = [machine.model_dump() for machine in results]

            return {
                'data': machines,
                'total': total_count,
                'offset': kwargs.get('offset', 0),
                'limit': kwargs.get('limit'),
                'success': True
            }

    # Alias for TAFL compatibility
    query_machines = query_machine  # Support both singular and plural forms

    def query_products(self, **kwargs) -> Dict[str, Any]:
        """
        Query products with enhanced filtering options

        Basic filters:
        - id: Product ID
        - process_settings_id: Filter by process settings ID
        - name: Filter by product name
        - size: Filter by product size

        Date filters:
        - created_after: Created after date
        - created_before: Created before date
        - updated_after: Updated after date
        - updated_before: Updated before date

        Pagination and sorting:
        - sort_by: Field to sort by (default: id)
        - sort_order: 'asc' or 'desc'
        - offset: Pagination offset
        - limit: Maximum results
        """
        with self.pool_manager.get_session() as session:
            query = select(Product, ProcessSettings).join(
                ProcessSettings, Product.process_settings_id == ProcessSettings.id, isouter=True
            )

            # Apply filters
            conditions = []

            # Basic filters
            if 'id' in kwargs:
                conditions.append(Product.id == kwargs['id'])
            if 'process_settings_id' in kwargs:
                conditions.append(Product.process_settings_id == kwargs['process_settings_id'])
            if 'name' in kwargs:
                conditions.append(Product.name == kwargs['name'])
            if 'size' in kwargs:
                conditions.append(Product.size == kwargs['size'])

            # Date filters
            self._apply_date_filters(Product, conditions, kwargs)

            # Apply conditions
            if conditions:
                query = query.where(and_(*conditions))

            # Apply sorting
            sort_by = kwargs.get('sort_by', 'id')
            sort_order = kwargs.get('sort_order', 'asc')

            if hasattr(Product, sort_by):
                order_column = getattr(Product, sort_by)
                if sort_order.lower() == 'desc':
                    query = query.order_by(desc(order_column))
                else:
                    query = query.order_by(asc(order_column))

            # Apply pagination
            if 'offset' in kwargs:
                query = query.offset(kwargs['offset'])
            if 'limit' in kwargs:
                query = query.limit(kwargs['limit'])

            # Execute query
            results = session.exec(query).all()

            # Get total count
            count_query = select(Product, ProcessSettings).join(
                ProcessSettings, Product.process_settings_id == ProcessSettings.id, isouter=True
            )
            if conditions:
                count_query = count_query.where(and_(*conditions))
            total_count = len(session.exec(count_query).all())

            # Convert to dict with process settings info
            products = []
            for product, process_settings in results:
                product_dict = product.model_dump()
                if process_settings:
                    product_dict['process_settings'] = {
                        'id': process_settings.id,
                        'soaking_times': process_settings.soaking_times,
                        'description': process_settings.description
                    }
                products.append(product_dict)

            return {
                'data': products,
                'total': total_count,
                'offset': kwargs.get('offset', 0),
                'limit': kwargs.get('limit', None),
                'has_more': total_count > (kwargs.get('offset', 0) + len(products))
            }

    def query_eqps(self, **kwargs) -> Dict[str, Any]:
        """
        Query equipment (eqp) with filtering options

        Basic filters:
        - id: Equipment ID
        - name: Equipment name
        - location_id: Filter by location ID
        - id_in: List of equipment IDs

        Pagination and sorting:
        - sort_by: Field to sort by (default: id)
        - sort_order: 'asc' or 'desc'
        - offset: Pagination offset
        - limit: Maximum results
        """
        with self.pool_manager.get_session() as session:
            query = select(Eqp)

            # Apply filters
            conditions = []

            # Basic filters
            if 'id' in kwargs:
                conditions.append(Eqp.id == kwargs['id'])
            if 'name' in kwargs:
                conditions.append(Eqp.name == kwargs['name'])
            if 'location_id' in kwargs:
                if kwargs['location_id'] is None:
                    conditions.append(Eqp.location_id.is_(None))
                else:
                    conditions.append(Eqp.location_id == kwargs['location_id'])

            # Array filters
            if 'id_in' in kwargs:
                id_list = kwargs['id_in']
                if isinstance(id_list, (list, tuple)):
                    conditions.append(Eqp.id.in_(id_list))

            # Apply conditions
            if conditions:
                query = query.where(and_(*conditions))

            # Apply unified pagination and sorting
            query = self._apply_pagination_and_sorting(query, Eqp, kwargs)

            # Execute query
            results = session.exec(query).all()

            # Get total count
            count_query = select(Eqp)
            if conditions:
                count_query = count_query.where(and_(*conditions))
            total_count = len(session.exec(count_query).all())

            # Convert to dict
            eqps = [eqp.model_dump() for eqp in results]

            return {
                'data': eqps,
                'total': total_count,
                'offset': kwargs.get('offset', 0),
                'limit': kwargs.get('limit', None),
                'has_more': total_count > (kwargs.get('offset', 0) + len(eqps))
            }

    def query_eqp_ports(self, **kwargs) -> Dict[str, Any]:
        """
        Query equipment ports (eqp_port) with filtering options

        Basic filters:
        - equipment_id: Filter by equipment ID (eqp_id)
        - port: Single port ID
        - port_in: List of port IDs
        - eqp_id: Alias for equipment_id
        - id: Filter by port ID
        - id_in: List of port IDs
        - name: Filter by port name

        Status filters (requires JOIN with carrier table):
        - status: 'empty' or 'occupied'
          - 'empty': Ports with no carriers (LEFT JOIN carrier WHERE carrier.id IS NULL)
          - 'occupied': Ports with carriers (INNER JOIN carrier)

        Pagination and sorting:
        - sort_by: Field to sort by (default: id)
        - sort_order: 'asc' or 'desc'
        - offset: Pagination offset
        - limit: Maximum results
        """
        with self.pool_manager.get_session() as session:
            # Base query - start with EqpPort only
            query = select(EqpPort)

            # Apply filters
            conditions = []

            # Equipment ID filter (support both equipment_id and eqp_id)
            eqp_id_value = kwargs.get('equipment_id') or kwargs.get('eqp_id')
            if eqp_id_value is not None:
                conditions.append(EqpPort.eqp_id == eqp_id_value)

            # Port ID filters
            if 'id' in kwargs:
                conditions.append(EqpPort.id == kwargs['id'])
            if 'port' in kwargs:
                conditions.append(EqpPort.id == kwargs['port'])
            if 'id_in' in kwargs:
                id_list = kwargs['id_in']
                if isinstance(id_list, (list, tuple)):
                    conditions.append(EqpPort.id.in_(id_list))
            if 'port_in' in kwargs:
                port_list = kwargs['port_in']
                if isinstance(port_list, (list, tuple)):
                    conditions.append(EqpPort.id.in_(port_list))

            # Name filter
            if 'name' in kwargs:
                conditions.append(EqpPort.name == kwargs['name'])

            # Status filter - requires JOIN with carrier table
            status = kwargs.get('status')
            if status:
                if status == 'empty':
                    # LEFT JOIN carrier and filter where carrier is NULL
                    query = select(EqpPort, Carrier).outerjoin(
                        Carrier, EqpPort.id == Carrier.port_id
                    )
                    conditions.append(Carrier.id.is_(None))
                elif status == 'occupied':
                    # INNER JOIN carrier to get only occupied ports
                    query = select(EqpPort, Carrier).join(
                        Carrier, EqpPort.id == Carrier.port_id
                    )
                else:
                    logger.warning(f"Unknown status filter: {status}. Valid values: 'empty', 'occupied'")

            # Apply conditions
            if conditions:
                query = query.where(and_(*conditions))

            # Apply unified pagination and sorting
            query = self._apply_pagination_and_sorting(query, EqpPort, kwargs)

            # Execute query
            results = session.exec(query).all()

            # Get total count
            if status:
                if status == 'empty':
                    count_query = select(EqpPort, Carrier).outerjoin(
                        Carrier, EqpPort.id == Carrier.port_id
                    )
                elif status == 'occupied':
                    count_query = select(EqpPort, Carrier).join(
                        Carrier, EqpPort.id == Carrier.port_id
                    )
            else:
                count_query = select(EqpPort)

            if conditions:
                count_query = count_query.where(and_(*conditions))
            total_count = len(session.exec(count_query).all())

            # Convert to dict
            eqp_ports = []
            for result in results:
                # Handle both single EqpPort and (EqpPort, Carrier) tuple
                # Also handle SQLAlchemy Row objects
                if isinstance(result, tuple):
                    # Tuple: (EqpPort, Carrier) or Row-like object
                    port = result[0]
                    carrier = result[1] if len(result) > 1 else None
                elif hasattr(result, '_mapping'):
                    # SQLAlchemy Row object - extract first entity
                    port = result[0]
                    carrier = result[1] if len(result) > 1 else None
                else:
                    # Single EqpPort object
                    port = result
                    carrier = None

                # Ensure port is a model object, not a proxy
                if hasattr(port, 'model_dump'):
                    port_dict = port.model_dump()
                else:
                    # Fallback: manually create dict from Row object
                    port_dict = {
                        'id': port.id,
                        'eqp_id': port.eqp_id,
                        'name': port.name if hasattr(port, 'name') else None,
                        'description': port.description if hasattr(port, 'description') else None
                    }

                # Add status information
                if status:
                    port_dict['status'] = status
                else:
                    # Determine status by checking if carrier exists
                    port_dict['status'] = 'occupied' if carrier else 'empty'

                # Add carrier info if available
                if carrier:
                    port_dict['carrier_id'] = carrier.id

                eqp_ports.append(port_dict)

            return {
                'data': eqp_ports,
                'total': total_count,
                'offset': kwargs.get('offset', 0),
                'limit': kwargs.get('limit', None),
                'has_more': total_count > (kwargs.get('offset', 0) + len(eqp_ports))
            }

    def query_eqp_signals(self, **kwargs) -> Dict[str, Any]:
        """
        Query equipment signals (eqp_signal) with filtering options

        Basic filters:
        - eqp_id: Filter by equipment ID
        - eqp_port_id: Filter by port ID
        - eqp_port_id_in: List of port IDs
        - id: Filter by signal ID
        - id_in: List of signal IDs
        - name: Exact name match
        - name_like: Pattern match (e.g., "%Presence")
        - value: Exact value match
        - type_of_value: Filter by value type (e.g., "bool", "int", "float", "string")
        - dm_address: Filter by DM address

        Pagination and sorting:
        - sort_by: Field to sort by (default: id)
        - sort_order: 'asc' or 'desc'
        - offset: Pagination offset
        - limit: Maximum results
        """
        with self.pool_manager.get_session() as session:
            # Base query
            query = select(EqpSignal)

            # Apply filters
            conditions = []

            # Equipment ID filter
            if 'eqp_id' in kwargs:
                conditions.append(EqpSignal.eqp_id == kwargs['eqp_id'])

            # Port ID filters
            if 'eqp_port_id' in kwargs:
                conditions.append(EqpSignal.eqp_port_id == kwargs['eqp_port_id'])
            if 'eqp_port_id_in' in kwargs:
                port_list = kwargs['eqp_port_id_in']
                if isinstance(port_list, (list, tuple)):
                    conditions.append(EqpSignal.eqp_port_id.in_(port_list))

            # Signal ID filters
            if 'id' in kwargs:
                conditions.append(EqpSignal.id == kwargs['id'])
            if 'id_in' in kwargs:
                id_list = kwargs['id_in']
                if isinstance(id_list, (list, tuple)):
                    conditions.append(EqpSignal.id.in_(id_list))

            # Name filters
            if 'name' in kwargs:
                conditions.append(EqpSignal.name == kwargs['name'])
            if 'name_like' in kwargs:
                pattern = kwargs['name_like']
                # Convert TAFL pattern to SQL LIKE pattern
                # TAFL uses % wildcards, same as SQL
                conditions.append(EqpSignal.name.like(pattern))

            # Value filter
            if 'value' in kwargs:
                conditions.append(EqpSignal.value == str(kwargs['value']))

            # Type of value filter
            if 'type_of_value' in kwargs:
                conditions.append(EqpSignal.type_of_value == kwargs['type_of_value'])

            # DM address filter
            if 'dm_address' in kwargs:
                conditions.append(EqpSignal.dm_address == kwargs['dm_address'])

            # Apply conditions
            if conditions:
                query = query.where(and_(*conditions))

            # Apply unified pagination and sorting
            query = self._apply_pagination_and_sorting(query, EqpSignal, kwargs)

            # Execute query
            results = session.exec(query).all()

            # Get total count
            count_query = select(EqpSignal)
            if conditions:
                count_query = count_query.where(and_(*conditions))
            total_count = len(session.exec(count_query).all())

            # Convert to dict
            eqp_signals = []
            for signal in results:
                if hasattr(signal, 'model_dump'):
                    signal_dict = signal.model_dump()
                else:
                    # Fallback: manually create dict
                    signal_dict = {
                        'id': signal.id,
                        'eqp_id': signal.eqp_id,
                        'eqp_port_id': signal.eqp_port_id,
                        'name': signal.name if hasattr(signal, 'name') else None,
                        'description': signal.description if hasattr(signal, 'description') else None,
                        'value': signal.value if hasattr(signal, 'value') else None,
                        'type_of_value': signal.type_of_value if hasattr(signal, 'type_of_value') else None,
                        'dm_address': signal.dm_address if hasattr(signal, 'dm_address') else None
                    }

                eqp_signals.append(signal_dict)

            return {
                'data': eqp_signals,
                'total': total_count,
                'offset': kwargs.get('offset', 0),
                'limit': kwargs.get('limit', None),
                'has_more': total_count > (kwargs.get('offset', 0) + len(eqp_signals))
            }

    def close(self):
        """Close database connection and save change log"""
        # Optionally save change log to file or database
        if self.change_log:
            logger.info(f"Saving {len(self.change_log)} change log entries")
            # Could implement saving to file or database here

        if hasattr(self, 'pool_manager'):
            self.pool_manager.shutdown()
            logger.info("Enhanced database bridge closed")