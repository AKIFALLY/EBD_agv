#!/usr/bin/env python3
"""
Check functions for Flow WCS with systematic default value handling
"""

from typing import Dict, Any, List, Optional
from .base import FlowFunctionBase
from ..decorators import flow_function


class CheckFunctions(FlowFunctionBase):
    """Check functions for condition validation"""
    
    @flow_function("check", "檢查資料是否為空", ["data"], "boolean",
                   defaults={"data": None})
    def empty(self, params: Dict) -> bool:
        """
        Check if data is empty with systematic default value handling
        """
        # Extract parameter - support both 'data' and 'collection' parameter names
        data = self.get_param(params, 'data', default=None)
        if data is None:
            # Also check for 'collection' parameter for compatibility
            data = self.get_param(params, 'collection', default=None)
        
        self.logger.info(f"Checking if data is empty: {type(data)}")
        
        # Check various empty conditions
        if data is None:
            result = True
        elif isinstance(data, (list, dict, str)):
            result = len(data) == 0
        elif isinstance(data, bool):
            result = not data
        elif isinstance(data, (int, float)):
            result = data == 0
        else:
            result = not bool(data)
        
        self.log_execution('check.empty', params, result)
        return result
    
    @flow_function("check", "檢查架台狀態", ["rack_id", "side", "check_type"], "boolean",
                   defaults={"rack_id": None, "side": "a", "check_type": "has_work"})
    def rack_status(self, params: Dict) -> bool:
        """
        Check rack status with systematic default value handling.
        Prevents variable name conflicts in database operations.
        """
        # Extract parameters with defaults
        rack_id = self.get_param(params, 'rack_id', required=True)
        side = self.get_param(params, 'side', default='a')
        check_type = self.get_param(params, 'check_type', default='has_work')
        
        # Type validation
        rack_id = self.safe_cast(rack_id, int, default=None)
        if rack_id is None:
            self.logger.error("Invalid rack_id for rack status check")
            return False
        
        # Validate side parameter
        if side not in ['a', 'b']:
            self.logger.warning(f"Invalid side '{side}', using 'a'")
            side = 'a'
        
        # Validate check_type
        valid_check_types = ['has_work', 'all_complete', 'empty', 'occupied']
        if check_type not in valid_check_types:
            self.logger.warning(f"Invalid check_type '{check_type}', using 'has_work'")
            check_type = 'has_work'
        
        self.logger.info(f"Checking rack {rack_id} side {side} for {check_type}")
        
        # Use database manager with protected parameters
        result = self.executor.db_manager.check_rack_status(
            rack_id=rack_id,
            side=side,
            check_type=check_type
        )
        
        self.log_execution('check.rack_status', params, result)
        return result
    
    @flow_function("check", "檢查任務是否存在", ["type", "location_id", "rack_id", "status"], "boolean",
                   defaults={"type": None, "location_id": None, "rack_id": None, "status": None})
    def task_exists(self, params: Dict) -> bool:
        """
        Check if task exists with systematic default value handling
        """
        # Extract parameters with defaults
        task_type = self.get_param(params, 'type', default=None)
        location_id = self.get_param(params, 'location_id', default=None)
        rack_id = self.get_param(params, 'rack_id', default=None)
        status = self.get_param(params, 'status', default=None)
        
        # Type casting
        if location_id is not None:
            location_id = self.safe_cast(location_id, int, default=None)
        if rack_id is not None:
            rack_id = self.safe_cast(rack_id, int, default=None)
        
        self.logger.info(f"Checking task existence: type={task_type}, location={location_id}, "
                        f"rack={rack_id}, status={status}")
        
        result = self.executor.db_manager.check_task_exists(
            type=task_type,
            location_id=location_id,
            rack_id=rack_id,
            status=status
        )
        
        self.log_execution('check.task_exists', params, result)
        return result
    
    @flow_function("check", "檢查位置是否可用", ["location_id"], "boolean",
                   defaults={"location_id": None})
    def location_available(self, params: Dict) -> bool:
        """
        Check if location is available with systematic default value handling
        """
        # Extract parameter
        location_id = self.get_param(params, 'location_id', required=True)
        
        # Type casting
        location_id = self.safe_cast(location_id, int, default=None)
        if location_id is None:
            self.logger.error("Invalid location_id for availability check")
            return False
        
        self.logger.info(f"Checking location {location_id} availability")
        
        # Query location status
        locations = self.executor.db_manager.query_locations()
        location = next((loc for loc in locations if loc['id'] == location_id), None)
        
        if location:
            # Check if location is available (not occupied)
            result = location.get('status', 'unknown') == 'available'
        else:
            self.logger.warning(f"Location {location_id} not found")
            result = False
        
        self.log_execution('check.location_available', params, result)
        return result
    
    @flow_function("check", "檢查系統就緒狀態", ["components"], "boolean",
                   defaults={"components": ["database", "plc", "zenoh"]})
    def system_ready(self, params: Dict) -> bool:
        """
        Check if system components are ready with systematic default value handling
        """
        # Extract parameters with defaults
        components = self.get_param(params, 'components', default=['database', 'plc', 'zenoh'])
        
        # Ensure it's a list
        components = self.safe_cast(components, list, default=['database', 'plc', 'zenoh'])
        
        self.logger.info(f"Checking system readiness for components: {components}")
        
        all_ready = True
        
        for component in components:
            if component == 'database':
                # Check database connection
                try:
                    with self.executor.db_manager.get_session() as session:
                        session.execute('SELECT 1')
                    self.logger.info("✓ Database is ready")
                except Exception as e:
                    self.logger.error(f"✗ Database not ready: {e}")
                    all_ready = False
            
            elif component == 'plc':
                # Mock PLC check - implement actual check if needed
                self.logger.info("✓ PLC assumed ready")
            
            elif component == 'zenoh':
                # Mock Zenoh check - implement actual check if needed
                self.logger.info("✓ Zenoh assumed ready")
            
            else:
                self.logger.warning(f"Unknown component: {component}")
        
        self.log_execution('check.system_ready', params, all_ready)
        return all_ready
