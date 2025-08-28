#!/usr/bin/env python3
"""
Query functions for Flow WCS with systematic default value handling
"""

from typing import Dict, Any, List, Optional
from .base import FlowFunctionBase
from ..decorators import flow_function


class QueryFunctions(FlowFunctionBase):
    """Query functions for database operations"""
    
    @flow_function("query", "查詢位置資料", ["type", "rooms", "has_rack"], "array",
                   defaults={"type": "enter_or_exit", "rooms": [1,2,3,4,5], "has_rack": True})
    def locations(self, params: Dict) -> List[Dict]:
        """
        Query locations with systematic default value handling.
        Prevents variable name conflicts and ensures proper defaults.
        """
        # Extract parameters with systematic defaults
        location_type = self.get_param(params, 'type', default='enter_or_exit')
        rooms = self.get_param(params, 'rooms', default=[1, 2, 3, 4, 5])
        has_rack = self.get_param(params, 'has_rack', default=True)
        
        # Ensure proper types
        rooms = self.safe_cast(rooms, list, default=[1, 2, 3, 4, 5])
        has_rack = self.safe_cast(has_rack, bool, default=True)
        
        self.logger.info(f"Querying locations: type={location_type}, rooms={rooms}, has_rack={has_rack}")
        
        # Use database manager with protected parameter names
        result = self.executor.db_manager.query_locations(
            type=location_type,
            rooms=rooms,
            has_rack=has_rack  # Now properly protected from variable conflicts
        )
        
        self.log_execution('query.locations', params, f"{len(result)} locations")
        return result
    
    @flow_function("query", "查詢架台資料", ["location_id", "status"], "array",
                   defaults={"location_id": None, "status": None})
    def racks(self, params: Dict) -> List[Dict]:
        """
        Query racks with systematic default value handling
        """
        # Extract parameters with defaults
        location_id = self.get_param(params, 'location_id', default=None)
        status = self.get_param(params, 'status', default=None)
        
        # Type casting with safety
        if location_id is not None:
            location_id = self.safe_cast(location_id, int, default=None)
        
        self.logger.info(f"Querying racks: location_id={location_id}, status={status}")
        
        result = self.executor.db_manager.query_racks(
            location_id=location_id,
            status=status
        )
        
        self.log_execution('query.racks', params, f"{len(result)} racks")
        return result
    
    @flow_function("query", "查詢任務資料", ["status", "type", "limit"], "array",
                   defaults={"status": None, "type": None, "limit": 100})
    def tasks(self, params: Dict) -> List[Dict]:
        """
        Query tasks with systematic default value handling
        """
        # Extract parameters with defaults
        status = self.get_param(params, 'status', default=None)
        task_type = self.get_param(params, 'type', default=None)
        limit = self.get_param(params, 'limit', default=100)
        
        # Type casting
        limit = self.safe_cast(limit, int, default=100)
        
        self.logger.info(f"Querying tasks: status={status}, type={task_type}, limit={limit}")
        
        result = self.executor.db_manager.query_tasks(
            status=status,
            type=task_type,
            limit=limit
        )
        
        self.log_execution('query.tasks', params, f"{len(result)} tasks")
        return result
    
    @flow_function("query", "查詢 AGV 資料", ["status", "type"], "array",
                   defaults={"status": None, "type": None})
    def agvs(self, params: Dict) -> List[Dict]:
        """
        Query AGVs with systematic default value handling
        """
        # Extract parameters with defaults
        status = self.get_param(params, 'status', default=None)
        agv_type = self.get_param(params, 'type', default=None)
        
        self.logger.info(f"Querying AGVs: status={status}, type={agv_type}")
        
        result = self.executor.db_manager.query_agvs(
            status=status,
            type=agv_type
        )
        
        self.log_execution('query.agvs', params, f"{len(result)} AGVs")
        return result
    
    @flow_function("query", "查詢房間資訊", ["room_id", "production_status"], "array",
                   defaults={"room_id": None, "production_status": None})
    def rooms(self, params: Dict) -> List[Dict]:
        """
        Query room information with systematic default value handling
        """
        # Extract parameters with defaults
        room_id = self.get_param(params, 'room_id', default=None)
        production_status = self.get_param(params, 'production_status', default=None)
        
        # Type casting
        if room_id is not None:
            room_id = self.safe_cast(room_id, int, default=None)
        
        self.logger.info(f"Querying rooms: room_id={room_id}, production_status={production_status}")
        
        # Mock implementation - replace with actual database query
        result = [
            {
                'id': 1, 'name': 'Entrance 1', 'type': 'entrance',
                'production_status': 'active', 'rack_count': 5
            },
            {
                'id': 2, 'name': 'Exit 2', 'type': 'exit',
                'production_status': 'active', 'rack_count': 3
            },
            {
                'id': 3, 'name': 'Entrance 3', 'type': 'entrance',
                'production_status': 'inactive', 'rack_count': 0
            },
            {
                'id': 4, 'name': 'Exit 4', 'type': 'exit',
                'production_status': 'active', 'rack_count': 2
            },
            {
                'id': 5, 'name': 'Mixed 5', 'type': 'both',
                'production_status': 'active', 'rack_count': 4
            }
        ]
        
        # Apply filters
        if room_id is not None:
            result = [r for r in result if r['id'] == room_id]
        if production_status is not None:
            result = [r for r in result if r['production_status'] == production_status]
        
        self.log_execution('query.rooms', params, f"{len(result)} rooms")
        return result
