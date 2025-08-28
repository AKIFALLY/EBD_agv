"""
TAFL Database Bridge Module
Provides database access functions for TAFL flows
"""

import logging
from typing import List, Dict, Any, Optional
from datetime import datetime
import json

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
from sqlmodel import Session, select, and_, or_

logger = logging.getLogger(__name__)


class TAFLDatabaseBridge:
    """Database bridge for TAFL operations"""
    
    def __init__(self, database_url: str):
        """Initialize database connection"""
        self.pool_manager = ConnectionPoolManager(database_url)
        logger.info("✅ Database bridge initialized")
    
    # ========== Query Functions ==========
    
    def query_locations(self, **kwargs) -> List[Dict]:
        """Query locations with optional filters"""
        with self.pool_manager.get_session() as session:
            query = select(Location, LocationStatus).join(
                LocationStatus, Location.location_status_id == LocationStatus.id, isouter=True
            )
            
            # Apply filters
            conditions = []
            if 'room_id' in kwargs:
                conditions.append(Location.room_id == kwargs['room_id'])
            if 'node_id' in kwargs:
                conditions.append(Location.node_id == kwargs['node_id'])
            if 'status' in kwargs:
                conditions.append(LocationStatus.name == kwargs['status'])
            
            if conditions:
                query = query.where(and_(*conditions))
            
            # Apply limit
            if 'limit' in kwargs:
                query = query.limit(kwargs['limit'])
            
            results = session.exec(query).all()
            
            # Convert to dict
            locations = []
            for location, status in results:
                loc_dict = location.model_dump()
                loc_dict['status'] = status.name if status else 'UNKNOWN'
                locations.append(loc_dict)
            
            return locations
    
    def query_racks(self, **kwargs) -> List[Dict]:
        """Query racks with optional filters"""
        with self.pool_manager.get_session() as session:
            query = select(Rack, RackStatus).join(
                RackStatus, Rack.status_id == RackStatus.id, isouter=True
            )
            
            # Apply filters
            conditions = []
            if 'location_id' in kwargs:
                conditions.append(Rack.location_id == kwargs['location_id'])
            if 'status' in kwargs:
                conditions.append(RackStatus.name == kwargs['status'])
            if 'is_carry' in kwargs:
                conditions.append(Rack.is_carry == kwargs['is_carry'])
            if 'is_docked' in kwargs:
                conditions.append(Rack.is_docked == kwargs['is_docked'])
            
            if conditions:
                query = query.where(and_(*conditions))
            
            # Apply limit
            if 'limit' in kwargs:
                query = query.limit(kwargs['limit'])
            
            results = session.exec(query).all()
            
            # Convert to dict
            racks = []
            for rack, status in results:
                rack_dict = rack.model_dump()
                rack_dict['status'] = status.name if status else 'UNKNOWN'
                racks.append(rack_dict)
            
            return racks
    
    def query_tasks(self, **kwargs) -> List[Dict]:
        """Query tasks with optional filters"""
        with self.pool_manager.get_session() as session:
            query = select(Task, TaskStatus).join(
                TaskStatus, Task.status_id == TaskStatus.id, isouter=True
            )
            
            # Apply filters
            conditions = []
            if 'work_id' in kwargs:
                conditions.append(Task.work_id == kwargs['work_id'])
            if 'status' in kwargs:
                conditions.append(TaskStatus.name == kwargs['status'])
            if 'priority' in kwargs:
                conditions.append(Task.priority == kwargs['priority'])
            
            if conditions:
                query = query.where(and_(*conditions))
            
            # Apply ordering
            query = query.order_by(Task.priority.desc(), Task.created_at)
            
            # Apply limit
            if 'limit' in kwargs:
                query = query.limit(kwargs['limit'])
            
            results = session.exec(query).all()
            
            # Convert to dict
            tasks = []
            for task, status in results:
                task_dict = task.model_dump()
                task_dict['status'] = status.name if status else 'UNKNOWN'
                tasks.append(task_dict)
            
            return tasks
    
    def query_works(self, **kwargs) -> List[Dict]:
        """Query works with optional filters"""
        with self.pool_manager.get_session() as session:
            query = select(Work)
            
            # Apply filters
            conditions = []
            if 'work_code' in kwargs:
                conditions.append(Work.work_code == kwargs['work_code'])
            if 'enabled' in kwargs:
                conditions.append(Work.enabled == kwargs['enabled'])
            
            if conditions:
                query = query.where(and_(*conditions))
            
            # Apply limit
            if 'limit' in kwargs:
                query = query.limit(kwargs['limit'])
            
            results = session.exec(query).all()
            
            # Convert to dict
            return [work.model_dump() for work in results]
    
    # ========== Check Functions ==========
    
    def check_location_available(self, location_id: int) -> bool:
        """Check if a location is available (not occupied)"""
        with self.pool_manager.get_session() as session:
            location = session.get(Location, location_id)
            if not location:
                return False
            return location.location_status_id == LocationStatus.UNOCCUPIED
    
    def check_rack_needs_rotation(self, rack_id: int) -> bool:
        """Check if a rack needs rotation"""
        with self.pool_manager.get_session() as session:
            rack = session.get(Rack, rack_id)
            if not rack:
                return False
            return rack.is_carry and not rack.is_docked
    
    def check_task_exists(self, task_id: str) -> bool:
        """Check if a task exists"""
        with self.pool_manager.get_session() as session:
            task = session.get(Task, task_id)
            return task is not None
    
    # ========== Create Functions ==========
    
    def create_task(self, **params) -> str:
        """Create a new task"""
        with self.pool_manager.get_session() as session:
            # Extract parameters
            work_id = params.get('work_id')
            priority = params.get('priority', 5)
            metadata = params.get('metadata', {})
            
            # Create task
            task = Task(
                work_id=work_id,
                priority=priority,
                status_id=TaskStatus.PENDING,  # Assuming PENDING = 1
                parameters=metadata,
                created_at=datetime.now()
            )
            
            session.add(task)
            session.commit()
            session.refresh(task)
            
            logger.info(f"✅ Created task: {task.id}")
            return str(task.id)
    
    def create_rack(self, **params) -> str:
        """Create a new rack"""
        with self.pool_manager.get_session() as session:
            rack = Rack(
                location_id=params.get('location_id'),
                status_id=params.get('status_id', RackStatus.EMPTY),
                is_carry=params.get('is_carry', False),
                is_docked=params.get('is_docked', False),
                created_at=datetime.now()
            )
            
            session.add(rack)
            session.commit()
            session.refresh(rack)
            
            logger.info(f"✅ Created rack: {rack.id}")
            return str(rack.id)
    
    # ========== Update Functions ==========
    
    def update_task_status(self, task_id: str, status: str) -> bool:
        """Update task status"""
        with self.pool_manager.get_session() as session:
            task = session.get(Task, task_id)
            if not task:
                logger.warning(f"Task {task_id} not found")
                return False
            
            # Get status ID
            status_obj = session.exec(
                select(TaskStatus).where(TaskStatus.name == status)
            ).first()
            
            if not status_obj:
                logger.warning(f"Status {status} not found")
                return False
            
            task.status_id = status_obj.id
            task.updated_at = datetime.now()
            
            session.add(task)
            session.commit()
            
            logger.info(f"✅ Updated task {task_id} status to {status}")
            return True
    
    def update_rack_side_completed(self, rack_id: int, side: str, completed: bool) -> bool:
        """Update rack side completion status"""
        with self.pool_manager.get_session() as session:
            rack = session.get(Rack, rack_id)
            if not rack:
                logger.warning(f"Rack {rack_id} not found")
                return False
            
            if side == 'a':
                rack.is_carry = completed
            elif side == 'b':
                rack.is_docked = completed
            else:
                logger.warning(f"Invalid side: {side}")
                return False
            
            rack.updated_at = datetime.now()
            
            session.add(rack)
            session.commit()
            
            logger.info(f"✅ Updated rack {rack_id} side {side} completed to {completed}")
            return True
    
    def update_location_status(self, location_id: int, status: str) -> bool:
        """Update location status"""
        with self.pool_manager.get_session() as session:
            location = session.get(Location, location_id)
            if not location:
                logger.warning(f"Location {location_id} not found")
                return False
            
            # Get status ID
            status_obj = session.exec(
                select(LocationStatus).where(LocationStatus.name == status)
            ).first()
            
            if not status_obj:
                logger.warning(f"Status {status} not found")
                return False
            
            location.location_status_id = status_obj.id
            
            session.add(location)
            session.commit()
            
            logger.info(f"✅ Updated location {location_id} status to {status}")
            return True
    
    def close(self):
        """Close database connection"""
        if hasattr(self, 'pool_manager'):
            self.pool_manager.shutdown()
            logger.info("Database bridge closed")
