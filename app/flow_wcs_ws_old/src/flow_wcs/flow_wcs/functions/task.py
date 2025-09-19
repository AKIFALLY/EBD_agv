#!/usr/bin/env python3
"""
Task functions for Flow WCS with systematic default value handling
"""

from typing import Dict, Any, Optional
from .base import FlowFunctionBase
from ..decorators import flow_function


class TaskFunctions(FlowFunctionBase):
    """Task management functions"""
    
    @flow_function("task", "建立新任務", ["type", "work_id", "location_id", "rack_id", "room_id", "priority", "metadata"], "string",
                   defaults={"type": "STANDARD", "work_id": None, "location_id": None, 
                            "rack_id": None, "room_id": None, "priority": 100, "metadata": {}})
    def create_task(self, params: Dict) -> str:
        """
        Create a new task with systematic default value handling.
        Ensures proper parameter merging and prevents circular overwrites.
        """
        # Extract parameters with systematic defaults
        task_type = self.get_param(params, 'type', default='STANDARD')
        work_id = self.get_param(params, 'work_id', required=True)
        location_id = self.get_param(params, 'location_id', default=None)
        rack_id = self.get_param(params, 'rack_id', default=None)
        room_id = self.get_param(params, 'room_id', default=None)
        priority = self.get_param(params, 'priority', default=100)
        metadata = self.get_param(params, 'metadata', default={})
        
        # Type validation and casting
        if work_id is None:
            self.logger.error("work_id is required for task creation")
            return None
        
        work_id = str(work_id)  # Ensure work_id is string
        
        if location_id is not None:
            location_id = self.safe_cast(location_id, int, default=None)
        if rack_id is not None:
            rack_id = self.safe_cast(rack_id, int, default=None)
        if room_id is not None:
            room_id = self.safe_cast(room_id, int, default=None)
        
        priority = self.safe_cast(priority, int, default=100)
        metadata = self.safe_cast(metadata, dict, default={})
        
        self.logger.info(f"Creating task: type={task_type}, work_id={work_id}, "
                        f"location={location_id}, rack={rack_id}, room={room_id}")
        
        # Create task with protected parameters
        task_id = self.executor.db_manager.create_task(
            type=task_type,
            work_id=work_id,
            location_id=location_id,
            rack_id=rack_id,
            room_id=room_id,
            priority=priority,
            metadata=metadata
        )
        
        if task_id:
            self.logger.info(f"Created task: {task_id}")
        else:
            self.logger.error("Failed to create task")
        
        self.log_execution('task.create_task', params, task_id)
        return task_id
    
    @flow_function("task", "更新任務狀態", ["task_id", "status"], "boolean",
                   defaults={"task_id": None, "status": "pending"})
    def update_task(self, params: Dict) -> bool:
        """
        Update task status with systematic default value handling
        """
        # Extract parameters
        task_id = self.get_param(params, 'task_id', required=True)
        status = self.get_param(params, 'status', default='pending')
        
        if task_id is None:
            self.logger.error("task_id is required for task update")
            return False
        
        task_id = str(task_id)
        
        # Validate status
        valid_statuses = ['pending', 'assigned', 'executing', 'completed', 'cancelled', 'failed']
        if status not in valid_statuses:
            self.logger.warning(f"Invalid status '{status}', using 'pending'")
            status = 'pending'
        
        self.logger.info(f"Updating task {task_id} to status: {status}")
        
        result = self.executor.db_manager.update_task(
            task_id=task_id,
            status=status
        )
        
        self.log_execution('task.update_task', params, result)
        return result
    
    @flow_function("task", "分配任務給 AGV", ["task_id", "agv_id"], "object",
                   defaults={"task_id": None, "agv_id": None})
    def assign_task_to_agv(self, params: Dict) -> Dict[str, Any]:
        """
        Assign task to AGV with systematic default value handling
        """
        # Extract parameters
        task_id = self.get_param(params, 'task_id', required=True)
        agv_id = self.get_param(params, 'agv_id', required=True)
        
        if task_id is None or agv_id is None:
            self.logger.error("Both task_id and agv_id are required for task assignment")
            return {'success': False, 'message': 'Missing required parameters'}
        
        task_id = str(task_id)
        agv_id = str(agv_id)
        
        self.logger.info(f"Assigning task {task_id} to AGV {agv_id}")
        
        result = self.executor.db_manager.assign_task(
            task_id=task_id,
            agv_id=agv_id
        )
        
        self.log_execution('task.assign_task_to_agv', params, result)
        return result
    
    @flow_function("task", "取消任務", ["task_id", "reason"], "boolean",
                   defaults={"task_id": None, "reason": "User cancelled"})
    def cancel_task(self, params: Dict) -> bool:
        """
        Cancel task with systematic default value handling
        """
        # Extract parameters
        task_id = self.get_param(params, 'task_id', required=True)
        reason = self.get_param(params, 'reason', default='User cancelled')
        
        if task_id is None:
            self.logger.error("task_id is required for task cancellation")
            return False
        
        task_id = str(task_id)
        
        self.logger.info(f"Cancelling task {task_id}: {reason}")
        
        # Update task status to cancelled
        result = self.executor.db_manager.update_task(
            task_id=task_id,
            status='cancelled'
        )
        
        if result:
            # Log cancellation reason (could be saved to database)
            self.logger.info(f"Task {task_id} cancelled: {reason}")
        
        self.log_execution('task.cancel_task', params, result)
        return result
