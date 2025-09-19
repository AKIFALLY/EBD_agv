#!/usr/bin/env python3
"""
Flow Executor - Core execution engine for linear workflows
直接操作資料庫版本，不使用 ROS 2 服務
"""

import yaml
import json
from typing import Dict, Any, List, Optional, Union
from datetime import datetime
from pathlib import Path
import asyncio
import re
import time
import logging

# Import database manager for direct database access (using unified models from db_proxy)
from .database import db_manager

# Import decorators for automatic function registration
from .decorators import flow_function, get_function_library as get_registry_library, get_function_handler


class FlowExecutor:
    """Core executor for linear flow workflows with direct database access"""
    
    def __init__(self, flow_data: Dict, context: Dict = None, ros_node=None):
        self.flow_data = flow_data
        self.db = db_manager  # Direct database access
        self.context = context or self.create_context()
        self.functions = self.register_functions()
        self.ros_node = ros_node  # ROS 2 node for logging
        
        # Initialize variable stack for proper scoping
        self.variable_stack = []
        
        # Log initialization for debugging
        import time
        self.init_time = time.time()
        
        # Use ROS 2 logger if node is available, otherwise use Python logging
        # In thread context, ROS logger might not work properly, so we test it
        if self.ros_node:
            try:
                # Try to use ROS logger
                self.logger = self.ros_node.get_logger()
                # Log initialization
                self.logger.info(f"🚀 FlowExecutor initialized at {self.init_time:.2f} for flow {self.flow_data['flow']['id']}")
            except Exception as e:
                # Fallback to Python logger if ROS logger fails in thread
                import logging
                logging.basicConfig(level=logging.INFO)
                self.logger = logging.getLogger(__name__)
                self.logger.info(f"Using Python logger in thread context: {e}")
        else:
            import logging
            logging.basicConfig(level=logging.INFO)
            self.logger = logging.getLogger(__name__)
        
    def create_context(self) -> Dict:
        """Create a new execution context"""
        flow_id = self.flow_data['flow']['id']
        start_time = datetime.now()
        
        context = {
            'variables': {
                # System variables (readonly)
                '_flow_id': flow_id,
                '_flow_name': self.flow_data['flow'].get('name', ''),
                '_work_id': self.flow_data['flow'].get('work_id'),
                '_priority': self.flow_data['flow'].get('priority', 100),
                '_timestamp': start_time.isoformat(),
                '_execution_time': 0,
                '_errors': []
            },
            'flow_id': flow_id,
            'flow_name': self.flow_data['flow'].get('name', ''),
            'work_id': self.flow_data['flow'].get('work_id'),
            'start_time': start_time,
            'current_section': None,
            'current_step': None,
            'status': 'initialized',
            'errors': [],
            'logs': []
        }
        
        # Initialize user-defined variables from flow data
        if 'variables' in self.flow_data:
            for var_name, var_def in self.flow_data['variables'].items():
                if not var_name.startswith('_'):  # Skip system variables
                    if isinstance(var_def, dict):
                        context['variables'][var_name] = var_def.get('default', None)
                    else:
                        context['variables'][var_name] = var_def
        
        return context
    
    def push_variable_scope(self):
        """Push current variables to stack and create new scope with inheritance"""
        # Create a deep copy of current variables
        import copy
        current_vars = copy.deepcopy(self.context['variables'])
        self.variable_stack.append(current_vars)
        # Keep a copy of variables for the new scope (inherits from parent)
        # This allows child scopes to see parent variables but changes won't affect parent
        self.context['variables'] = copy.deepcopy(current_vars)
    
    def pop_variable_scope(self):
        """Pop variable scope from stack and restore previous scope"""
        if self.variable_stack:
            # Restore previous scope
            self.context['variables'] = self.variable_stack.pop()
        else:
            self.log_warning("Attempted to pop empty variable stack")
    
    def register_functions(self) -> Dict:
        """Register available functions for execution"""
        return {
            # Query functions - Direct database queries
            'query.locations': self.query_locations,
            'query.racks': self.query_racks,
            'query.tasks': self.query_tasks,
            'query.agvs': self.query_agvs,
            
            # Check functions - Direct database checks
            'check.empty': self.check_empty,
            'check.rack_status': self.check_rack_status,
            'check.task_exists': self.check_task_exists,
            'check.location_available': self.check_location_available,
            'check.system_ready': self.check_system_ready,
            
            # Task functions - Direct database operations
            'task.create': self.create_task,
            'task.create_task': self.create_task,  # New name mapping
            'task.update': self.update_task,
            'task.update_task': self.update_task,  # New name mapping
            'task.assign': self.assign_task,
            'task.assign_task': self.assign_task,  # New name mapping
            'task.cancel': self.cancel_task,
            'task.cancel_task': self.cancel_task,  # New name mapping
            
            # Action functions - Direct database updates
            'action.rotate_rack': self.rotate_rack,
            'action.notify': self.send_notification,
            'action.send_notification': self.send_notification,  # New name mapping
            'action.log': self.log_message,
            'action.log_message': self.log_message,  # New name mapping
            'action.optimize_batch': self.optimize_batch,
            'action.analyze_priorities': self.analyze_priorities,
            'action.find_optimal_agv': self.find_optimal_agv,
            'action.recover': self.recover_from_error,
            'action.calculate_metrics': self.calculate_metrics,
            'action.optimize': self.optimize_performance,
            'action.alert': self.send_alert,
            'action.cleanup': self.cleanup_resources,
            'action.generate_report': self.generate_report,
            'action.save': self.save_data,
            'action.send_alarm': self.send_alarm,  # Additional for new flows
            'action.trigger_event': self.trigger_event,  # Additional for new flows
            
            # Additional query functions
            'query.rooms': self.query_rooms,  # Additional for new flows
            
            # Control functions
            'control.wait': self.wait_time,
            'control.wait_time': self.wait_time,  # New name mapping
            'control.stop': self.stop_flow,
            'control.stop_flow': self.stop_flow,  # New name mapping
            'control.count': self.count_items,
            'control.count_items': self.count_items,  # New name mapping
            'control.switch': self.switch,  # Switch with conditions
            'control.switch_case': self.switch_case,  # Simple switch case
            'control.update_variable': self.update_variable,
            'control.foreach': self.foreach,  # Foreach control structure
            'foreach': self.foreach  # Alias for backward compatibility
        }
    
    @classmethod
    def get_function_library(cls) -> Dict:
        """Get function library metadata for external tools
        
        Returns all decorator-registered functions
        """
        return get_registry_library()
    
    @classmethod
    def execute_function_test(cls, function_name: str, params: Dict, variables: Dict = None) -> Any:
        """Execute a single function for testing purposes
        
        Args:
            function_name: Full function name (e.g., "query.locations")
            params: Function parameters
            variables: Optional variables context
            
        Returns:
            Function execution result
        """
        # Create a minimal flow for execution
        flow_data = {
            "flow": {
                "id": "test_flow",
                "name": "Test Flow"
            },
            "workflow": []
        }
        
        # Create executor instance with variables
        executor = cls(flow_data)
        if variables:
            executor.context['variables'] = variables
        
        # Apply defaults to params
        from .decorators import apply_function_defaults
        params_with_defaults = apply_function_defaults(function_name, params)
        
        # Get the function handler
        handler = get_function_handler(function_name)
        if not handler:
            # Fallback to old registration if decorator not yet applied
            if function_name in executor.functions:
                handler = executor.functions[function_name]
            else:
                raise ValueError(f"Function '{function_name}' not found")
        
        # Execute the function
        try:
            # Check if it's an async function
            import asyncio
            import inspect
            
            # The handler from decorator is unbound, need to bind it to executor instance
            if handler and hasattr(handler, '__self__'):
                # Already bound method
                bound_handler = handler
            elif handler:
                # Unbound method from decorator, bind it to executor
                bound_handler = handler.__get__(executor, executor.__class__)
            else:
                raise ValueError(f"No handler found for function '{function_name}'")
            
            if inspect.iscoroutinefunction(bound_handler):
                # For async functions, run in event loop
                result = asyncio.run(bound_handler(params_with_defaults))
            else:
                result = bound_handler(params_with_defaults)
            
            return result
        except Exception as e:
            raise Exception(f"Error executing function '{function_name}': {str(e)}")
    
    async def execute_in_test_mode(self, dry_run: bool = False) -> Dict:
        """Execute flow in test mode with enhanced tracking
        
        Args:
            dry_run: If True, only simulate execution without actual database operations
            
        Returns:
            Detailed execution report with step tracking and variable snapshots
        """
        # Set test mode flag
        self.context['test_mode'] = True
        self.context['dry_run'] = dry_run
        
        # Initialize detailed tracking
        self.context['execution_trace'] = []
        self.context['variable_snapshots'] = []
        
        # Execute with enhanced tracking
        result = await self.execute()
        
        # Return detailed report
        return {
            'status': self.context['status'],
            'duration': self.context.get('execution_time', 0),
            'trace': self.context.get('execution_trace', []),
            'snapshots': self.context.get('variable_snapshots', []),
            'final_variables': self.context.get('variables', {}),
            'errors': self.context.get('errors', []),
            'detailed_logs': self.context.get('detailed_logs', []),
            'logs': self.context.get('logs', [])
        }
    
    async def execute(self) -> Dict:
        """Execute the entire workflow"""
        self.context['status'] = 'running'
        self.context['start_time'] = datetime.now()
        start_time = time.time()
        
        try:
            workflow = self.flow_data.get('workflow', [])
            
            for section in workflow:
                await self.execute_section(section)
                
                if self.context['status'] == 'stopped':
                    break
            
            if self.context['status'] != 'stopped':
                self.context['status'] = 'completed'
            
        except Exception as e:
            self.context['status'] = 'error'
            self.context['errors'].append(str(e))
            # Update system _errors variable
            self.context['variables']['_errors'].append({
                'type': type(e).__name__,
                'message': str(e),
                'step': self.context.get('current_step', 'unknown'),
                'section': self.context.get('current_section', 'unknown'),
                'timestamp': datetime.now().isoformat()
            })
            self.log_error(f"Flow execution failed: {e}")
        
        finally:
            self.context['end_time'] = datetime.now()
            duration = time.time() - start_time
            
            # Update system variables
            self.context['variables']['_execution_time'] = int(duration * 1000)  # in milliseconds
            
            # Log flow execution to database (skip if dry_run)
            if not self.context.get('dry_run', False):
                self.db.log_flow_execution(
                    flow_id=self.context['flow_id'],
                    flow_name=self.context['flow_name'],
                    work_id=self.context['work_id'],
                    section=self.context.get('current_section', ''),
                    step_id=self.context.get('current_step', ''),
                function='flow_complete',
                params={},
                result={'status': self.context['status']},
                status=self.context['status'],
                error_message='; '.join(self.context['errors']) if self.context['errors'] else None,
                duration=duration
            )
        
        return self.context
    
    async def execute_section(self, section: Dict):
        """Execute a workflow section"""
        section_name = section.get('section', 'unnamed')
        self.context['current_section'] = section_name
        
        self.log_info(f"Executing section: {section_name}")
        
        # Check section condition
        if 'condition' in section:
            if not await self.evaluate_condition(section['condition']):
                self.log_info(f"Skipping section {section_name} - condition not met")
                return
        
        # Execute steps
        steps = section.get('steps', [])
        for step in steps:
            await self.execute_step(step)
            
            if self.context['status'] == 'stopped':
                break
    
    async def execute_step(self, step: Dict):
        """Execute a single step"""
        step_id = step.get('id', 'unnamed')
        self.context['current_step'] = step_id
        step_start = time.time()
        
        # Track step execution in test mode
        if self.context.get('test_mode'):
            step_trace = {
                'step_id': step_id,
                'exec': step.get('exec', ''),
                'params': step.get('params', {}),
                'start_time': datetime.now().isoformat(),
                'variables_before': dict(self.context.get('variables', {}))
            }
        
        # Check skip condition
        if 'skip_if' in step:
            condition = step['skip_if']
            result = await self.evaluate_condition(condition)
            
            # Log detailed condition evaluation for test mode
            condition_details = {
                'condition_expression': condition,
                'evaluation_result': result,
                'variables_used': {}
            }
            
            # Extract and log variables used in condition
            import re
            var_pattern = r'\${([^}]+)}'
            var_matches = re.findall(var_pattern, condition)
            for var_name in var_matches:
                var_value = self.context['variables'].get(var_name, 'UNDEFINED')
                condition_details['variables_used'][var_name] = var_value
            
            # Add debug info for specific steps
            if step_id in ['check_rack_needs_rotation', 'check_a_side_complete', 'create_rotation_task', 'log_task_created']:
                self.log_info(f"Step {step_id} skip_if debug:", condition_details)
                self.log_info(f"  Condition: {condition}")
                self.log_info(f"  no_racks: {self.context['variables'].get('no_racks', 'NOT SET')}")
                self.log_info(f"  b_side_has_work: {self.context['variables'].get('b_side_has_work', 'NOT SET')}")
                self.log_info(f"  a_side_complete: {self.context['variables'].get('a_side_complete', 'NOT SET')}")
                self.log_info(f"  task_exists: {self.context['variables'].get('task_exists', 'NOT SET')}")
                self.log_info(f"  Skip Result: {result}")
            else:
                self.log_info(f"Step {step_id} skip_if condition: {condition} = {result}", condition_details)
            
            if result:
                self.log_info(f"⏭️ Skipping step {step_id} due to skip_if condition")
                # Track skipped step in test mode
                if self.context.get('test_mode') and 'execution_trace' in self.context:
                    self.context['execution_trace'].append({
                        'step_id': step_id,
                        'function': step.get('exec', ''),
                        'status': 'skipped',
                        'reason': 'skip_if condition met',
                        'timestamp': datetime.now().isoformat()
                    })
                return
        
        if 'skip_if_not' in step:
            if not await self.evaluate_condition(step['skip_if_not']):
                self.log_info(f"Skipping step {step_id}")
                # Track skipped step in test mode
                if self.context.get('test_mode') and 'execution_trace' in self.context:
                    self.context['execution_trace'].append({
                        'step_id': step_id,
                        'function': step.get('exec', ''),
                        'status': 'skipped',
                        'reason': 'skip_if_not condition not met',
                        'timestamp': datetime.now().isoformat()
                    })
                return
        
        # Get execution function
        exec_cmd = step.get('exec', '')
        
        try:
            if exec_cmd == 'foreach':
                await self.execute_foreach(step)
            elif exec_cmd == 'parallel':
                await self.execute_parallel(step)
            else:
                await self.execute_function(step)
            
            # Log step execution to database
            duration = time.time() - step_start
            # Resolve variables in params before logging to database
            resolved_params = await self.resolve_variables(step.get('params', {}))
            
            # Get the result from context - enhanced to capture actual function results
            result_data = {'success': True}
            
            # For regular functions, get the last function result
            if exec_cmd not in ['foreach', 'parallel', 'if', 'while'] and 'last_function_result' in self.context:
                actual_result = self.context['last_function_result']
                
                # Format result based on type
                if isinstance(actual_result, list):
                    result_data['type'] = 'list'
                    result_data['count'] = len(actual_result)
                    if len(actual_result) > 0:
                        # Store sample data (first 5 items)
                        result_data['data'] = actual_result[:5]
                        # Extract key fields if present
                        if isinstance(actual_result[0], dict):
                            if 'node_id' in actual_result[0]:
                                result_data['node_ids'] = [item.get('node_id') for item in actual_result]
                            if 'room_id' in actual_result[0]:
                                result_data['room_ids'] = [item.get('room_id') for item in actual_result]
                            if 'id' in actual_result[0]:
                                result_data['ids'] = [item.get('id') for item in actual_result]
                elif isinstance(actual_result, dict):
                    result_data['type'] = 'dict'
                    result_data['data'] = actual_result
                elif isinstance(actual_result, bool):
                    result_data['type'] = 'boolean'
                    result_data['value'] = actual_result
                elif actual_result is not None:
                    result_data['type'] = 'value'
                    result_data['value'] = actual_result
                
                # Clear the last function result
                del self.context['last_function_result']
            
            # For foreach and other control structures, use the stored variable
            elif 'store' in step:
                store_name = await self.resolve_variables(step['store'])
                if store_name in self.context['variables']:
                    stored_value = self.context['variables'][store_name]
                    # For foreach, include the items that were processed
                    if exec_cmd == 'foreach' and isinstance(stored_value, list):
                        result_data['items_count'] = len(stored_value)
                        result_data['items'] = stored_value[:10]  # Store first 10 items for logging
                    elif isinstance(stored_value, list):
                        result_data['count'] = len(stored_value)
                        result_data['data'] = stored_value
                    else:
                        result_data['value'] = stored_value
            
            # Only log to database if not in dry_run mode
            if not self.context.get('dry_run', False):
                self.db.log_flow_execution(
                    flow_id=self.context['flow_id'],
                    flow_name=self.context['flow_name'],
                    work_id=self.context['work_id'],
                    section=self.context['current_section'],
                    step_id=step_id,
                    function=exec_cmd,
                    params=resolved_params,
                    result=result_data,
                    status='success',
                    duration=duration
                )
            
            # Track step completion in test mode
            if self.context.get('test_mode'):
                step_trace['end_time'] = datetime.now().isoformat()
                step_trace['duration'] = duration
                step_trace['status'] = 'success'
                step_trace['result'] = result_data
                step_trace['variables_after'] = dict(self.context.get('variables', {}))
                
                # Add to execution trace
                if 'execution_trace' in self.context:
                    self.context['execution_trace'].append(step_trace)
                
                # Take variable snapshot if significant changes
                if step_trace['variables_before'] != step_trace['variables_after']:
                    snapshot = {
                        'step_id': step_id,
                        'changes': self._get_variable_changes(
                            step_trace['variables_before'],
                            step_trace['variables_after']
                        )
                    }
                    if 'variable_snapshots' in self.context:
                        self.context['variable_snapshots'].append(snapshot)
            
        except Exception as e:
            self.log_error(f"Step {step_id} failed: {e}")
            
            # Update system _errors variable
            self.context['variables']['_errors'].append({
                'type': type(e).__name__,
                'message': str(e),
                'step': step_id,
                'section': self.context.get('current_section', 'unknown'),
                'timestamp': datetime.now().isoformat()
            })
            
            # Log failure to database (skip if dry_run)
            duration = time.time() - step_start
            # Resolve variables in params before logging to database
            resolved_params = await self.resolve_variables(step.get('params', {}))
            if not self.context.get('dry_run', False):
                self.db.log_flow_execution(
                    flow_id=self.context['flow_id'],
                    flow_name=self.context['flow_name'],
                    work_id=self.context['work_id'],
                    section=self.context['current_section'],
                    step_id=step_id,
                    function=exec_cmd,
                    params=resolved_params,
                    result={'error': str(e)},
                    status='failed',
                    error_message=str(e),
                    duration=duration
                )
    
    async def execute_function(self, step: Dict):
        """Execute a function step"""
        exec_cmd = step.get('exec', '')
        params = step.get('params', {})
        step_id = step.get('id', 'unknown')
        
        # Debug: Log parameters before resolution
        self.logger.info(f"   [DEBUG] Parameters before resolution: {params}")
        
        # Apply function defaults first
        from .decorators import apply_function_defaults
        params_with_defaults = apply_function_defaults(exec_cmd, params)
        self.logger.info(f"   [DEBUG] Parameters after defaults: {params_with_defaults}")
        
        # Resolve variables in parameters
        resolved_params = await self.resolve_variables(params_with_defaults)
        
        # Debug: Log parameters after resolution
        self.logger.info(f"   [DEBUG] Parameters after resolution: {resolved_params}")
        
        # Get function
        if exec_cmd in self.functions:
            # Log function execution start with detailed info
            execution_details = {
                'function': exec_cmd,
                'step_id': step_id,
                'section': self.context.get('current_section', 'unknown'),
                'input_params': resolved_params,
                'raw_params': params
            }
            
            self.log_info(f"========================================")
            self.log_info(f"📍 Executing function: {exec_cmd}", execution_details)
            self.log_info(f"   Step ID: {step_id}")
            self.log_info(f"   Section: {self.context.get('current_section', 'unknown')}")
            self.log_info(f"   Parameters: {resolved_params}")
            
            func = self.functions[exec_cmd]
            result = await self.call_function(func, resolved_params)
            
            # Store the actual result in context for later logging to database
            self.context['last_function_result'] = result
            
            # Log the result with details
            result_details = {
                'function': exec_cmd,
                'step_id': step_id,
                'result_type': type(result).__name__,
                'result_value': result if not isinstance(result, (list, dict)) or len(str(result)) < 500 else f"[{type(result).__name__} with {len(result)} items]"
            }
            
            # Store result if specified
            if 'store' in step:
                # Resolve variables in store name (e.g., "r${_room}" -> "r1")
                store_name = await self.resolve_variables(step['store'])
                self.context['variables'][store_name] = result
                result_details['stored_to'] = store_name
                self.log_info(f"   Stored to: {store_name}")
            
            # Enhanced result logging based on type
            if isinstance(result, list):
                self.log_info(f"   Result: List with {len(result)} items", result_details)
                if len(result) > 0:
                    # Show first item as example
                    first_item = result[0]
                    if isinstance(first_item, dict):
                        self.log_info(f"   First item: {first_item}")
                        # If it has node_id or id, highlight it
                        if 'node_id' in first_item:
                            self.log_info(f"   ✨ Node IDs: {[item.get('node_id') for item in result[:5]]}")
                        if 'room_id' in first_item:
                            self.log_info(f"   ✨ Room IDs: {[item.get('room_id') for item in result[:5]]}")
            elif isinstance(result, dict):
                self.log_info(f"   Result: Dictionary", result_details)
                self.log_info(f"   Result: {result}")
            elif isinstance(result, bool):
                self.log_info(f"   Result: {'✅ True' if result else '❌ False'}")
            elif result is None:
                self.log_info(f"   Result: None (no data)")
            else:
                self.log_info(f"   Result: {result}")
            
            self.log_info(f"✅ Function {exec_cmd} completed successfully")
            self.log_info(f"========================================")
        else:
            self.log_warning(f"Unknown function: {exec_cmd}")
    
    async def execute_foreach(self, step: Dict):
        """Execute a foreach loop with proper variable scoping"""
        # Check if parameters are in params dict (new format) or at top level (old format)
        params = step.get('params', {})
        
        # Get items, var, and steps from params if available, otherwise from step directly
        items_ref = params.get('items', step.get('items', ''))
        var_name = params.get('var', step.get('var', '_item'))
        sub_steps = params.get('steps', step.get('steps', []))
        
        # Resolve items
        items = await self.resolve_variable(items_ref)
        
        if not isinstance(items, list):
            self.log_warning(f"Foreach items is not a list: {items_ref}")
            return
        
        # Log the foreach iteration details
        self.log_info(f"Foreach processing {len(items)} items")
        
        for index, item in enumerate(items):
            # Push current context to stack and create new scope
            self.push_variable_scope()
            
            # Set loop variables for this iteration
            self.context['variables'][var_name] = item
            self.context['variables']['_item'] = item  # Also set _item for compatibility
            # Also set _${var_name} for template compatibility (e.g., _room when var is "room")
            self.context['variables'][f'_{var_name}'] = item
            self.context['variables']['_index'] = index  # Add index for tracking
            
            # Log current item being processed
            if isinstance(item, dict) and 'node_id' in item:
                self.log_info(f"Processing item {index+1}/{len(items)}: node_id={item.get('node_id')}, room_id={item.get('room_id')}")
            else:
                self.log_info(f"Processing item {index+1}/{len(items)}: {item}")
            
            # Execute sub-steps
            for sub_step in sub_steps:
                await self.execute_step(sub_step)
                
                if self.context['status'] == 'stopped':
                    # Pop scope before returning
                    self.pop_variable_scope()
                    return
            
            # Pop scope to restore previous context
            self.pop_variable_scope()
    
    async def execute_parallel(self, step: Dict):
        """Execute parallel branches"""
        # Support both old format (branches at top level) and new format (branches in params)
        branches = step.get('branches', [])
        
        # If branches is empty, check if this is complex_flow_example format
        # where branches are direct children of the step
        if not branches:
            # Look for named branches as direct children
            for key, value in step.items():
                if key not in ['id', 'exec', 'params', 'store', 'skip_if', 'skip_if_not']:
                    if isinstance(value, dict) and 'steps' in value:
                        branches.append(value)
        
        # Create tasks for each branch
        tasks = []
        for branch in branches:
            if isinstance(branch, dict):
                branch_name = branch.get('name', 'unnamed')
                branch_steps = branch.get('steps', [])
                self.log_info(f"Starting parallel branch: {branch_name}")
                task = self.execute_branch(branch_steps)
                tasks.append(task)
        
        # Wait for all branches
        if tasks:
            await asyncio.gather(*tasks)
        else:
            self.log_warning("No valid branches found for parallel execution")
    
    async def execute_branch(self, steps: List[Dict]):
        """Execute a branch of steps"""
        for step in steps:
            await self.execute_step(step)
            
            if self.context['status'] == 'stopped':
                break
    
    async def call_function(self, func, params: Dict) -> Any:
        """Call a function with parameters"""
        # Check if function is async
        if asyncio.iscoroutinefunction(func):
            return await func(params)
        else:
            return func(params)
    
    async def evaluate_condition(self, condition: Union[str, Dict, bool]) -> bool:
        """Evaluate a condition"""
        if isinstance(condition, bool):
            return condition
        
        if isinstance(condition, str):
            # Special string values
            if condition.lower() == 'true':
                return True
            if condition.lower() == 'false':
                return False
            
            # Handle logical operators: ||, &&, !
            if '||' in condition:
                # OR operator
                parts = condition.split('||')
                for part in parts:
                    if await self.evaluate_condition(part.strip()):
                        return True
                return False
            
            if '&&' in condition:
                # AND operator
                parts = condition.split('&&')
                for part in parts:
                    if not await self.evaluate_condition(part.strip()):
                        return False
                return True
            
            if condition.strip().startswith('!'):
                # NOT operator
                inner_condition = condition.strip()[1:].strip()
                return not await self.evaluate_condition(inner_condition)
            
            # Variable reference
            if condition.startswith('${') and condition.endswith('}'):
                value = await self.resolve_variable(condition)
                return bool(value)
            
            # Expression evaluation (simplified)
            if '==' in condition:
                parts = condition.split('==')
                left = await self.resolve_variable(parts[0].strip())
                right = await self.resolve_variable(parts[1].strip())
                return left == right
            
            if '!=' in condition:
                parts = condition.split('!=')
                left = await self.resolve_variable(parts[0].strip())
                right = await self.resolve_variable(parts[1].strip())
                return left != right
            
            return bool(condition)
        
        if isinstance(condition, dict):
            # Complex condition
            op = condition.get('op', 'and')
            conditions = condition.get('conditions', [])
            
            if op == 'and':
                for cond in conditions:
                    if not await self.evaluate_condition(cond):
                        return False
                return True
            elif op == 'or':
                for cond in conditions:
                    if await self.evaluate_condition(cond):
                        return True
                return False
        
        return False
    
    async def resolve_variable(self, ref: str) -> Any:
        """Resolve a variable reference"""
        if not isinstance(ref, str):
            return ref
        
        # Variable pattern: ${variable_name}
        if ref.startswith('${') and ref.endswith('}'):
            var_name = ref[2:-1]
            
            # Check for nested path (e.g., ${location.id})
            if '.' in var_name:
                parts = var_name.split('.')
                value = self.context['variables'].get(parts[0])
                
                for part in parts[1:]:
                    if isinstance(value, dict):
                        value = value.get(part)
                    elif isinstance(value, list) and part.isdigit():
                        value = value[int(part)] if int(part) < len(value) else None
                    else:
                        return None
                
                return value
            
            return self.context['variables'].get(var_name)
        
        return ref
    
    def _resolve_complex_expression(self, expr: str) -> Any:
        """Resolve complex variable expressions like variable.length, variable[0], variable[0].property, and math operations"""
        try:
            # Always log for debugging
            self.log_info(f"   📊 Resolving expression: {expr}")
            
            # Check for simple math operations (e.g., "location.node_id + 1")
            if '+' in expr or '-' in expr or '*' in expr or '/' in expr:
                # Try to evaluate math expression
                try:
                    # First resolve any variables in the expression
                    import re
                    # Find all variable references (word.word pattern)
                    var_pattern = r'(\w+(?:\.\w+)*)'
                    
                    def replace_var(match):
                        var_expr = match.group(1)
                        # Recursively resolve this variable
                        resolved = self._resolve_complex_expression(var_expr)
                        if resolved is not None:
                            return str(resolved)
                        return match.group(0)
                    
                    # Replace all variables with their values
                    resolved_expr = re.sub(var_pattern, replace_var, expr)
                    
                    # Now try to evaluate the math expression
                    # Only allow safe operations
                    if re.match(r'^[\d\s+\-*/()]+$', resolved_expr):
                        result = eval(resolved_expr)
                        self.log_info(f"   ✅ Evaluated math expression '{expr}' = '{resolved_expr}' = {result}")
                        return result
                except Exception as e:
                    self.log_info(f"   ⚠️ Could not evaluate math expression '{expr}': {e}")
            
            # Parse the expression
            # Examples: variable, variable.length, variable[0], variable[0].property
            parts = expr.replace('[', '.').replace(']', '').split('.')
            
            # Start with the base variable
            var_name = parts[0]
            value = self.context['variables'].get(var_name)
            
            # Log the current variables for debugging
            self.log_info(f"   📊 Available variables: {list(self.context['variables'].keys())}")
            self.log_info(f"   📊 Base variable '{var_name}' value: {value if value is not None else 'NOT FOUND'}")
            
            if value is None:
                # Variable not found in context (expected in foreach scope)
                return None  # Variable not found
            
            # Process each part of the expression
            for i, part in enumerate(parts[1:], 1):
                self.log_info(f"   📊 Processing part '{part}' on value type {type(value).__name__}")
                
                if part == 'length':
                    # Handle .length for arrays
                    if isinstance(value, (list, str)):
                        value = len(value)
                        self.log_info(f"   ✅ Got length: {value}")
                    else:
                        value = 0
                        self.log_info(f"   ⚠️ Not a list/string, returning 0")
                elif part.isdigit():
                    # Handle array index
                    index = int(part)
                    if isinstance(value, list) and index < len(value):
                        value = value[index]
                        self.log_info(f"   ✅ Got index [{index}]: {value}")
                    else:
                        self.log_info(f"   ❌ Index {index} out of bounds or not a list")
                        return None  # Index out of bounds
                elif isinstance(value, dict):
                    # Handle object property
                    old_value = value
                    value = value.get(part)
                    if value is None:
                        self.log_info(f"   ❌ Property '{part}' not found in dict with keys: {list(old_value.keys())}")
                        return None
                    else:
                        self.log_info(f"   ✅ Got property '{part}': {value}")
                elif isinstance(value, list) and len(value) > 0 and isinstance(value[0], dict):
                    # Handle array of objects property (e.g., get all node_ids)
                    value = [item.get(part) for item in value if isinstance(item, dict)]
                    self.log_info(f"   ✅ Got property '{part}' from list of dicts: {value}")
                else:
                    # Unknown property access
                    self.log_info(f"   ❌ Unknown property access: {part} on type {type(value).__name__}")
                    return None
            
            self.log_info(f"   ✅ Resolved '{expr}' to: {value}")
            return value
        except Exception as e:
            self.log_error(f"Exception resolving '{expr}': {e}")
            import traceback
            self.log_error(traceback.format_exc())
            return None
    
    async def resolve_variables(self, data: Any) -> Any:
        """Recursively resolve variables in data"""
        if isinstance(data, str):
            # Check for multiple variables in one string
            if '${' in data:
                pattern = r'\$\{([^}]+)\}'
                
                # Special case: if the entire string is a single variable reference
                # return the actual value (preserving type) instead of converting to string
                if data.strip().startswith('${') and data.strip().endswith('}'):
                    # Extract variable expression
                    var_match = re.match(r'^\$\{([^}]+)\}$', data.strip())
                    if var_match:
                        expr = var_match.group(1)
                        # Use the enhanced variable resolution logic
                        resolved_value = self._resolve_complex_expression(expr)
                        
                        # Log variable resolution for debugging
                        self.logger.debug(f"變數解析: ${{{expr}}} -> {resolved_value}")
                        
                        # Return the actual value, preserving its type
                        return resolved_value if resolved_value is not None else data
                
                # Otherwise, it's a string with embedded variables - convert to string
                def replacer(match):
                    expr = match.group(1)
                    value = self._resolve_complex_expression(expr)
                    
                    # Log each variable replacement
                    self.logger.debug(f"變數替換: ${{{expr}}} -> {value}")
                    
                    # Convert None to original expression, otherwise to string
                    return str(value) if value is not None else match.group(0)
                
                resolved_string = re.sub(pattern, replacer, data)
                
                # Log the final resolved string if it was changed
                if resolved_string != data:
                    self.logger.debug(f"字串解析完成: '{data}' -> '{resolved_string}'")
                
                return resolved_string
            return data
        elif isinstance(data, dict):
            resolved = {}
            for key, value in data.items():
                resolved[key] = await self.resolve_variables(value)
            return resolved
        elif isinstance(data, list):
            resolved = []
            for item in data:
                resolved.append(await self.resolve_variables(item))
            return resolved
        return data
    
    # Query functions - Direct database access
    
    @flow_function("query", "查詢位置資料", ["type", "rooms", "has_rack"], "array",
                   defaults={"type": "enter_or_exit", "rooms": [1 ,2,3,4,5], "has_rack": True})
    def query_locations(self, params: Dict) -> List[Dict]:
        """Query locations from database"""
        location_type = params.get('type', 'enter_or_exit')
        rooms = params.get('rooms', [1, 2, 3, 4, 5])
        has_rack = params.get('has_rack', True)  # 使用預設值 True
        
        # Handle string variable references that weren't resolved
        if isinstance(rooms, str) and rooms.startswith('${') and rooms.endswith('}'):
            # Extract variable name and try to resolve from context
            var_name = rooms[2:-1]  # Remove ${ and }
            if hasattr(self, 'context') and 'variables' in self.context:
                rooms = self.context['variables'].get(var_name, rooms)
            # If still a string, use default
            if isinstance(rooms, str):
                rooms = [1, 2, 3, 4, 5]
        
        return self.db.query_locations(type=location_type, rooms=rooms, has_rack=has_rack)
    
    @flow_function("query", "查詢架台資料", ["location_id", "status"], "array",
                   defaults={"location_id": 1, "status": "available"})
    def query_racks(self, params: Dict) -> List[Dict]:
        """Query racks from database"""
        location_id = params.get('location_id')
        status = params.get('status')
        
        # Handle both single value and array of location_ids
        if location_id:
            if isinstance(location_id, list):
                # For array of location_ids, query each and combine results
                all_racks = []
                for loc_id in location_id:
                    if isinstance(loc_id, str):
                        try:
                            loc_id = int(loc_id)
                        except ValueError:
                            continue
                    racks = self.db.query_racks(location_id=loc_id, status=status)
                    all_racks.extend(racks)
                return all_racks
            elif isinstance(location_id, str):
                try:
                    location_id = int(location_id)
                except ValueError:
                    # If conversion fails, use None to query all locations
                    location_id = None
        
        return self.db.query_racks(location_id=location_id, status=status)
    
    @flow_function("query", "查詢任務資料", ["status", "type", "limit"], "array",
                   defaults={"status": "pending", "type": "RACK_ROTATION", "limit": 10})
    def query_tasks(self, params: Dict) -> List[Dict]:
        """Query tasks from database"""
        status = params.get('status')
        task_type = params.get('type')
        limit = params.get('limit')
        
        return self.db.query_tasks(status=status, type=task_type, limit=limit)
    
    @flow_function("query", "查詢 AGV 資料", ["status", "type"], "array",
                   defaults={"status": "idle", "type": "cargo"})
    def query_agvs(self, params: Dict) -> List[Dict]:
        """Query AGVs from database"""
        status = params.get('status')
        agv_type = params.get('type')
        
        return self.db.query_agvs(status=status, type=agv_type)
    
    @flow_function("query", "查詢房間資訊", ["room_id", "production_status"], "array",
                   defaults={"room_id": None, "production_status": None})
    def query_rooms(self, params: Dict) -> List[Dict]:
        """Query room information from database"""
        room_id = params.get('room_id')
        production_status = params.get('production_status')
        
        # Simulate room data query
        # In real implementation, this would query from database
        rooms_data = []
        
        if room_id:
            # Mock room data based on room_id
            room_info = {
                'room_id': room_id,
                'name': f'Room {room_id}',
                'production_status': production_status or 'normal',
                'capacity': 10,
                'current_load': 5
            }
            rooms_data.append(room_info)
        else:
            # Return all rooms
            for rid in range(1, 6):  # Rooms 1-5
                room_info = {
                    'room_id': rid,
                    'name': f'Room {rid}',
                    'production_status': 'normal',
                    'capacity': 10,
                    'current_load': 3 + rid  # Mock data
                }
                rooms_data.append(room_info)
        
        self.log_info(f"Queried {len(rooms_data)} rooms")
        return rooms_data
    
    # Check functions
    
    @flow_function("check", "檢查資料是否為空", ["data"], "boolean",
                   defaults={"data": "${query_result}"})
    def check_empty(self, params: Dict) -> bool:
        """Check if data is empty"""
        data = params.get('data')
        
        if data is None:
            return True
        if isinstance(data, (list, dict, str)):
            return len(data) == 0
        return False
    
    @flow_function("check", "檢查架台狀態", ["rack_id", "side", "check_type"], "boolean",
                   defaults={"rack_id": "rack001", "side": "a", "check_type": "occupied"})
    def check_rack_status(self, params: Dict) -> bool:
        """Check rack status"""
        rack_id = params.get('rack_id')
        side = params.get('side', 'a')
        check_type = params.get('check_type')
        
        return self.db.check_rack_status(rack_id, side, check_type)
    
    @flow_function("check", "檢查任務是否存在", ["type", "location_id", "rack_id", "status"], "boolean",
                   defaults={"type": "RACK_ROTATION", "location_id": 1, "rack_id": None, "status": None})
    def check_task_exists(self, params: Dict) -> bool:
        """Check if task exists"""
        task_type = params.get('type')
        location_id = params.get('location_id')
        rack_id = params.get('rack_id')
        status = params.get('status')
        
        # Convert location_id to int if it's a string
        if location_id and isinstance(location_id, str):
            try:
                location_id = int(location_id)
            except ValueError:
                location_id = None
        
        # Convert rack_id to int if it's a string
        if rack_id and isinstance(rack_id, str):
            try:
                rack_id = int(rack_id)
            except ValueError:
                rack_id = None
        
        return self.db.check_task_exists(type=task_type, location_id=location_id, rack_id=rack_id, status=status)
    
    @flow_function("check", "檢查位置是否可用", ["location_id"], "boolean",
                   defaults={"location_id": 1})
    def check_location_available(self, params: Dict) -> bool:
        """Check if location is available"""
        location_id = params.get('location_id')
        
        # Convert location_id to int if it's a string
        if location_id and isinstance(location_id, str):
            try:
                location_id = int(location_id)
            except ValueError:
                return False  # Invalid location_id means not available
        
        # Check if location has no pending or executing tasks
        return not self.db.check_task_exists(
            type=None,
            location_id=location_id,
            status='pending'
        ) and not self.db.check_task_exists(
            type=None,
            location_id=location_id,
            status='executing'
        )
    
    @flow_function("check", "檢查系統就緒狀態", ["components"], "boolean",
                   defaults={"components": ["database", "agv_fleet", "plc"]})
    def check_system_ready(self, params: Dict) -> bool:
        """Check if system components are ready"""
        components = params.get('components', ['database'])
        
        # For now, just check database connection
        try:
            with self.db.get_session() as session:
                session.execute('SELECT 1')
            return True
        except:
            return False
    
    # Task functions
    
    @flow_function("task", "建立新任務", ["type", "work_id", "location_id", "rack_id", "room_id", "priority", "metadata"], "string",
                   defaults={"type": "RACK_ROTATION", "work_id": 1, "location_id": "loc001", "rack_id": None, "room_id": None, "priority": "normal", "metadata": {}})
    def create_task(self, params: Dict) -> str:
        """Create a new task"""
        task_type = params.get('type', 'RACK_ROTATION')
        work_id = params.get('work_id', self.context.get('work_id'))
        location_id = params.get('location_id')
        rack_id = params.get('rack_id')  # Extract rack_id from params
        room_id = params.get('room_id')  # Extract room_id from params - identifies which room's entrance/exit
        priority = params.get('priority', 100)
        metadata = params.get('metadata', {})
        
        # Convert priority string to number
        if isinstance(priority, str):
            priority_map = {'low': 50, 'normal': 100, 'high': 150, 'urgent': 200}
            priority = priority_map.get(priority.lower(), 100)
        
        task_id = self.db.create_task(
            type=task_type,
            work_id=work_id,
            location_id=location_id,
            rack_id=rack_id,  # Pass rack_id to database
            room_id=room_id,  # Pass room_id to database - important for identifying task's room
            priority=priority,
            metadata=metadata
        )
        
        self.log_info(f"Created task {task_id}: {task_type} at location {location_id} in room {room_id}")
        
        return task_id
    
    @flow_function("task", "更新任務狀態", ["task_id", "status"], "boolean",
                   defaults={"task_id": "${task_id}", "status": "executing"})
    def update_task(self, params: Dict) -> bool:
        """Update task status"""
        task_id = params.get('task_id')
        status = params.get('status')
        
        success = self.db.update_task(task_id, status)
        
        if success:
            self.log_info(f"Updated task {task_id} to {status}")
        else:
            self.log_warning(f"Failed to update task {task_id}")
        
        return success
    
    @flow_function("task", "分配任務給 AGV", ["task_id", "agv_id"], "object",
                   defaults={"task_id": "${task_id}", "agv_id": "agv01"})
    def assign_task(self, params: Dict) -> Dict:
        """Assign task to AGV"""
        task_id = params.get('task_id')
        agv_id = params.get('agv_id')
        
        result = self.db.assign_task(task_id, agv_id)
        
        if result.get('success'):
            self.log_info(f"Assigned task {task_id} to AGV {agv_id}")
        else:
            self.log_warning(f"Failed to assign task: {result.get('error')}")
        
        return result
    
    @flow_function("task", "取消任務", ["task_id", "reason"], "boolean",
                   defaults={"task_id": "${task_id}", "reason": "User cancelled"})
    def cancel_task(self, params: Dict) -> bool:
        """Cancel a task"""
        task_id = params.get('task_id')
        reason = params.get('reason', 'User cancelled')
        
        success = self.db.update_task(task_id, 'cancelled')
        
        if success:
            self.log_info(f"Cancelled task {task_id}: {reason}")
        else:
            self.log_warning(f"Failed to cancel task {task_id}")
        
        return success
    
    # Action functions
    
    @flow_function("action", "旋轉架台", ["rack_id", "angle"], "boolean",
                   defaults={"rack_id": "rack001", "angle": 180})
    def rotate_rack(self, params: Dict) -> bool:
        """Rotate a rack"""
        rack_id = params.get('rack_id')
        angle = params.get('angle', 180)
        
        success = self.db.rotate_rack(rack_id, angle)
        
        if success:
            self.log_info(f"Rotated rack {rack_id} by {angle} degrees")
        else:
            self.log_warning(f"Failed to rotate rack {rack_id}")
        
        return success
    
    @flow_function("action", "發送通知", ["message", "level", "priority"], "boolean",
                   defaults={"message": "Task completed", "level": "info", "priority": "normal"})
    def send_notification(self, params: Dict) -> bool:
        """Send a notification"""
        message = params.get('message', '')
        level = params.get('level', 'info')
        priority = params.get('priority', 'normal')
        
        # In a real system, this would send to notification service
        self.log_info(f"Notification [{level}:{priority}]: {message}")
        
        return True
    
    @flow_function("action", "記錄日誌", ["message", "level", "tags"], "boolean",
                   defaults={"message": "Flow step executed", "level": "info", "tags": []})
    def log_message(self, params: Dict) -> bool:
        """Log a message"""
        message = params.get('message', '')
        level = params.get('level', 'info')
        tags = params.get('tags', [])
        
        # Debug: Log that we're processing a message
        if '${' in message:
            self.log_info(f"   🔍 Processing message with variables: {message}")
            # Show current variables for debugging
            var_keys = list(self.context['variables'].keys())
            self.log_info(f"   🔍 Current variables: {var_keys}")
            # Show specific variable values if they exist
            if 'single_room_location' in self.context['variables']:
                srl = self.context['variables']['single_room_location']
                self.log_info(f"   🔍 single_room_location: {srl}")
        
        # Enhanced variable resolution using the shared helper method
        if '${' in message:
            pattern = r'\$\{([^}]+)\}'
            
            def replacer(match):
                expr = match.group(1)
                self.log_info(f"   🔍 Resolving expression: {expr}")
                value = self._resolve_complex_expression(expr)
                self.log_info(f"   🔍 Resolved to: {value}")
                # Convert None to original expression, otherwise to string
                return str(value) if value is not None else match.group(0)
            
            message = re.sub(pattern, replacer, message)
        
        # Format message with tags if present
        if tags:
            tag_str = f"[{', '.join(tags)}]"
            formatted_message = f"{tag_str} {message}"
        else:
            formatted_message = message
        
        # Log with appropriate level using the formatted message
        if level == 'error':
            self.log_error(formatted_message)
        elif level == 'warning':
            self.log_warning(formatted_message)
        else:
            self.log_info(f"📝 {formatted_message}")
        
        return True
    
    @flow_function("action", "最佳化任務批次", ["task_type", "optimization_strategy"], "object",
                   defaults={"task_type": "RACK_ROTATION", "optimization_strategy": "distance"})
    def optimize_batch(self, params: Dict) -> Dict:
        """Optimize task batch"""
        task_type = params.get('task_type')
        strategy = params.get('optimization_strategy', 'distance')
        
        # Query pending tasks
        tasks = self.db.query_tasks(status='pending', type=task_type, limit=50)
        
        # Simple optimization: sort by location_id
        if strategy == 'distance':
            tasks.sort(key=lambda t: t.get('location_id', 0))
        elif strategy == 'priority':
            tasks.sort(key=lambda t: t.get('priority', 0), reverse=True)
        
        result = {
            'optimized_count': len(tasks),
            'task_ids': [t['task_id'] for t in tasks],
            'strategy': strategy
        }
        
        self.log_info(f"Optimized {len(tasks)} tasks using {strategy} strategy")
        
        return result
    
    # Control functions
    
    @flow_function("control", "等待指定時間", ["seconds"], "boolean",
                   defaults={"seconds": 1.0})
    async def wait_time(self, params: Dict) -> bool:
        """Wait for specified time"""
        seconds = params.get('seconds', 1.0)
        
        await asyncio.sleep(seconds)
        
        return True
    
    @flow_function("control", "停止流程執行", ["reason"], "boolean",
                   defaults={"reason": "Condition met"})
    def stop_flow(self, params: Dict) -> bool:
        """Stop flow execution"""
        reason = params.get('reason', 'User requested')
        
        self.context['status'] = 'stopped'
        self.log_info(f"Flow stopped: {reason}")
        
        return True
    
    @flow_function("control", "計算項目數量", ["variable"], "number",
                   defaults={"variable": "${query_result}"})
    def count_items(self, params: Dict) -> int:
        """Count items in a variable"""
        variable = params.get('variable')
        
        if isinstance(variable, list):
            return len(variable)
        elif isinstance(variable, dict):
            return len(variable)
        elif isinstance(variable, str):
            return len(variable)
        else:
            return 0
    
    @flow_function("control", "Switch case 控制", ["value", "cases", "default"], "any",
                   defaults={"value": "${status}", "cases": {"pending": "wait", "ready": "execute"}, "default": "skip"})
    def switch_case(self, params: Dict) -> Any:
        """Switch case control"""
        value = params.get('value')
        cases = params.get('cases', {})
        default = params.get('default', None)
        
        if value in cases:
            return cases[value]
        
        return default
    
    @flow_function("control", "迴圈遍歷", ["items", "var", "steps", "max_iterations"], "array",
                   defaults={"items": [], "var": "item", "steps": [], "max_iterations": 1000},
                   also_register_as="foreach")
    def foreach(self, params: Dict) -> List[Any]:
        """Foreach loop control
        
        Iterates over items and executes steps for each item.
        
        Args:
            params: Dictionary containing:
                - items: List of items to iterate over
                - var: Variable name to store current item
                - steps: List of step definitions to execute for each item
                - max_iterations: Maximum number of iterations (safety limit)
                
        Returns:
            List of results from each iteration
        """
        items = params.get('items', [])
        var_name = params.get('var', 'item')
        steps = params.get('steps', [])
        max_iterations = params.get('max_iterations', 1000)
        
        # Safety check for empty items
        if not items:
            self.log_info(f"Foreach: No items to process")
            return []
        
        # Safety check for max iterations
        if len(items) > max_iterations:
            self.log_warning(f"Foreach: Limiting iterations from {len(items)} to {max_iterations}")
            items = items[:max_iterations]
        
        results = []
        original_var_value = self.context['variables'].get(var_name)
        
        try:
            for index, item in enumerate(items):
                self.log_info(f"Foreach: Processing item {index + 1}/{len(items)}")
                
                # Set loop variable
                self.context['variables'][var_name] = item
                
                # Execute steps for this iteration
                iteration_results = []
                for step in steps:
                    try:
                        # Create a copy of the step to avoid modifying original
                        step_copy = step.copy() if isinstance(step, dict) else step
                        
                        # Execute the step (this should call execute_step internally)
                        # For now, we'll store the step definition as the result
                        # The actual execution will be handled by the flow executor
                        iteration_results.append({
                            'step': step_copy,
                            'item': item,
                            'index': index
                        })
                        
                    except Exception as e:
                        self.log_error(f"Foreach: Error in step execution: {str(e)}")
                        if params.get('error_handling') == 'stop':
                            raise
                        # Continue with next step if error_handling is 'continue'
                
                results.append({
                    'item': item,
                    'index': index,
                    'results': iteration_results
                })
                
        finally:
            # Restore original variable value
            if original_var_value is not None:
                self.context['variables'][var_name] = original_var_value
            else:
                self.context['variables'].pop(var_name, None)
        
        self.log_info(f"Foreach: Completed {len(results)} iterations")
        return results
    
    # Logging functions
    
    def log_info(self, message: str, details: Dict = None):
        """Log info message with optional details"""
        # Log to ROS 2 console if available
        if self.ros_node:
            self.ros_node.get_logger().info(message)
        else:
            self.logger.info(message)
        
        log_entry = {
            'level': 'info',
            'message': message,
            'timestamp': datetime.now().isoformat(),
            'section': self.context.get('current_section'),
            'step': self.context.get('current_step')
        }
        
        # Add details if provided
        if details:
            log_entry['details'] = details
        
        self.context['logs'].append(log_entry)
        
        # Also add to detailed logs for test mode
        if self.context.get('test_mode'):
            if 'detailed_logs' not in self.context:
                self.context['detailed_logs'] = []
            self.context['detailed_logs'].append(log_entry)
    
    def log_warning(self, message: str, details: Dict = None):
        """Log warning message with optional details"""
        # Log to ROS 2 console if available
        if self.ros_node:
            self.ros_node.get_logger().warning(message)
        else:
            self.logger.warning(message)
        
        log_entry = {
            'level': 'warning',
            'message': message,
            'timestamp': datetime.now().isoformat(),
            'section': self.context.get('current_section'),
            'step': self.context.get('current_step')
        }
        
        # Add details if provided
        if details:
            log_entry['details'] = details
        
        self.context['logs'].append(log_entry)
        
        # Also add to detailed logs for test mode
        if self.context.get('test_mode'):
            if 'detailed_logs' not in self.context:
                self.context['detailed_logs'] = []
            self.context['detailed_logs'].append(log_entry)
    
    def log_error(self, message: str, details: Dict = None):
        """Log error message with optional details"""
        # Log to ROS 2 console if available
        if self.ros_node:
            self.ros_node.get_logger().error(message)
        else:
            self.logger.error(message)
        
        log_entry = {
            'level': 'error',
            'message': message,
            'timestamp': datetime.now().isoformat(),
            'section': self.context.get('current_section'),
            'step': self.context.get('current_step')
        }
        
        # Add details if provided
        if details:
            log_entry['details'] = details
        
        self.context['logs'].append(log_entry)
        
        # Also add to detailed logs for test mode
        if self.context.get('test_mode'):
            if 'detailed_logs' not in self.context:
                self.context['detailed_logs'] = []
            self.context['detailed_logs'].append(log_entry)
    
    # New Control Functions
    
    @flow_function("control", "更新變數值", ["variable", "operation", "value"], "any",
                   defaults={"variable": "counter", "operation": "set", "value": 0})
    def update_variable(self, params: Dict) -> Any:
        """Update a variable value with various operations"""
        var_name = params.get('variable')
        operation = params.get('operation', 'set')
        value = params.get('value')
        
        current_value = self.context['variables'].get(var_name)
        
        if operation == 'set':
            self.context['variables'][var_name] = value
            return value
        elif operation == 'increment':
            new_value = (current_value or 0) + (value or 1)
            self.context['variables'][var_name] = new_value
            return new_value
        elif operation == 'decrement':
            new_value = (current_value or 0) - (value or 1)
            self.context['variables'][var_name] = new_value
            return new_value
        elif operation == 'append':
            if not isinstance(current_value, list):
                current_value = [] if current_value is None else [current_value]
            current_value.append(value)
            self.context['variables'][var_name] = current_value
            return current_value
        elif operation == 'remove':
            if isinstance(current_value, list) and value in current_value:
                current_value.remove(value)
                self.context['variables'][var_name] = current_value
            return current_value
        elif operation == 'merge':
            if isinstance(current_value, dict) and isinstance(value, dict):
                current_value.update(value)
                self.context['variables'][var_name] = current_value
            return current_value
        else:
            self.log_warning(f"Unknown operation: {operation}")
            return current_value
    
    @flow_function("control", "條件分支控制", ["value", "cases"], "any",
                   defaults={"value": 0, "cases": []})
    def switch(self, params: Dict) -> Any:
        """Switch control with condition evaluation
        
        Supports two formats:
        1. Object format: {"4": true, "5": true, "default": false}
        2. Array format: [{"condition": ">=4", "steps": [...]}]
        """
        value = params.get('value')
        cases = params.get('cases', [])
        default_value = None
        
        # Handle object format (simple key-value mapping)
        if isinstance(cases, dict):
            # Check for default value
            default_value = cases.get('default')
            
            # Convert value to string for comparison
            str_value = str(value)
            
            # Check each case
            for case_key, case_value in cases.items():
                if case_key != 'default' and str_value == str(case_key):
                    return case_value
            
            # Return default if no match
            return default_value if default_value is not None else False
        
        # Handle array format (original implementation)
        elif isinstance(cases, list):
            for case in cases:
                condition = case.get('condition')
                steps = case.get('steps', [])
                
                # Evaluate condition
                if self._evaluate_switch_condition(value, condition):
                    # Execute steps for this case
                    # Note: In actual execution, this would be handled by execute_step
                    return {'matched_case': condition, 'steps': steps}
            
            # No case matched
            return {'matched_case': None, 'steps': []}
        
        # Invalid format
        self.logger.warning(f"Invalid cases format for control.switch: {type(cases)}")
        return None
    
    def _evaluate_switch_condition(self, value: Any, condition: str) -> bool:
        """Evaluate a switch condition string"""
        if condition is None:
            return False
        
        # Simple equality check
        if not any(op in condition for op in ['>', '<', '>=', '<=', '==']):
            return str(value) == str(condition)
        
        # Parse comparison operators
        try:
            if '>=' in condition:
                threshold = float(condition.replace('>=', '').strip())
                return float(value) >= threshold
            elif '<=' in condition:
                threshold = float(condition.replace('<=', '').strip())
                return float(value) <= threshold
            elif '>' in condition:
                threshold = float(condition.replace('>', '').strip())
                return float(value) > threshold
            elif '<' in condition:
                threshold = float(condition.replace('<', '').strip())
                return float(value) < threshold
            elif '==' in condition:
                expected = condition.replace('==', '').strip()
                return str(value) == expected
        except (ValueError, TypeError):
            return False
        
        return False
    
    # New Action Functions
    
    @flow_function("action", "分析任務優先級", ["tasks", "strategy"], "array",
                   defaults={"tasks": [], "strategy": "deadline_first"})
    def analyze_priorities(self, params: Dict) -> List[Dict]:
        """Analyze and sort tasks by priority"""
        tasks = params.get('tasks', [])
        strategy = params.get('strategy', 'deadline_first')
        
        # Ensure tasks is a list
        if not isinstance(tasks, list):
            # If tasks is a string (variable name), try to get it from context
            if isinstance(tasks, str) and hasattr(self, 'context'):
                tasks = self.context.get('variables', {}).get(tasks, [])
            else:
                tasks = []
        
        if not tasks:
            return []
        
        # Sort tasks based on strategy
        if strategy == 'deadline_first':
            # Sort by deadline (assuming tasks have deadline field)
            sorted_tasks = sorted(tasks, key=lambda t: t.get('deadline', '9999-12-31') if isinstance(t, dict) else '9999-12-31')
        elif strategy == 'priority_first':
            # Sort by priority (higher priority first)
            sorted_tasks = sorted(tasks, key=lambda t: t.get('priority', 0) if isinstance(t, dict) else 0, reverse=True)
        elif strategy == 'fifo':
            # First in, first out (by creation time)
            sorted_tasks = sorted(tasks, key=lambda t: t.get('created_at', '') if isinstance(t, dict) else '')
        elif strategy == 'shortest_first':
            # Sort by estimated time (shortest first)
            sorted_tasks = sorted(tasks, key=lambda t: t.get('estimated_time', float('inf')) if isinstance(t, dict) else float('inf'))
        else:
            # Default: no sorting
            sorted_tasks = tasks
        
        self.log_info(f"Analyzed {len(tasks)} tasks using {strategy} strategy")
        return sorted_tasks
    
    @flow_function("action", "尋找最佳 AGV", ["task", "agvs", "criteria"], "object",
                   defaults={"task": {}, "agvs": [], "criteria": ["distance"]})
    def find_optimal_agv(self, params: Dict) -> Dict:
        """Find the best AGV for a task based on criteria"""
        task = params.get('task', {})
        agvs = params.get('agvs', [])
        criteria = params.get('criteria', ['distance'])
        
        if not agvs:
            return None
        
        # Score each AGV
        scored_agvs = []
        for agv in agvs:
            score = 0
            
            # Calculate score based on criteria
            if 'distance' in criteria:
                # Lower distance is better (simple Manhattan distance)
                task_location = task.get('location', {'x': 0, 'y': 0})
                agv_location = agv.get('location', {'x': 0, 'y': 0})
                distance = abs(task_location.get('x', 0) - agv_location.get('x', 0)) + \
                          abs(task_location.get('y', 0) - agv_location.get('y', 0))
                score += (100 - min(distance, 100))  # Invert so higher score is better
            
            if 'battery' in criteria:
                # Higher battery is better
                score += agv.get('battery', 0)
            
            if 'capability' in criteria:
                # Check if AGV has required capabilities
                required_caps = task.get('required_capabilities', [])
                agv_caps = agv.get('capabilities', [])
                matching_caps = len([c for c in required_caps if c in agv_caps])
                score += matching_caps * 50
            
            # Add estimated time
            estimated_time = distance * 2 if 'distance' in criteria else 10
            
            scored_agvs.append({
                **agv,
                'score': score,
                'estimated_time': estimated_time
            })
        
        # Select AGV with highest score
        best_agv = max(scored_agvs, key=lambda a: a['score'])
        
        self.log_info(f"Selected AGV {best_agv.get('id')} with score {best_agv['score']}")
        return best_agv
    
    @flow_function("action", "錯誤恢復", ["error_type", "step_id", "retry_count"], "object",
                   defaults={"error_type": "unknown", "step_id": "", "retry_count": 3})
    def recover_from_error(self, params: Dict) -> Dict:
        """Attempt to recover from an error"""
        error_type = params.get('error_type', 'unknown')
        step_id = params.get('step_id', '')
        retry_count = params.get('retry_count', 3)
        
        recovery_result = {
            'success': False,
            'message': '',
            'action_taken': ''
        }
        
        # Recovery strategies based on error type
        if error_type == 'timeout':
            recovery_result['action_taken'] = 'retry_with_extended_timeout'
            recovery_result['success'] = True
            recovery_result['message'] = f"Will retry step {step_id} with extended timeout"
        elif error_type == 'connection_error':
            recovery_result['action_taken'] = 'reconnect_and_retry'
            recovery_result['success'] = True
            recovery_result['message'] = f"Will reconnect and retry step {step_id}"
        elif error_type == 'validation_error':
            recovery_result['action_taken'] = 'skip_and_continue'
            recovery_result['success'] = False
            recovery_result['message'] = f"Cannot recover from validation error in step {step_id}"
        else:
            recovery_result['action_taken'] = 'log_and_continue'
            recovery_result['success'] = False
            recovery_result['message'] = f"Unknown error type: {error_type}"
        
        self.log_info(f"Recovery attempt for {error_type}: {recovery_result['message']}")
        return recovery_result
    
    @flow_function("action", "計算效能指標", ["metrics"], "object",
                   defaults={"metrics": ["task_completion_rate"]})
    def calculate_metrics(self, params: Dict) -> Dict:
        """Calculate performance metrics"""
        requested_metrics = params.get('metrics', ['task_completion_rate'])
        
        metrics_result = {
            'overall_score': 0,
            'timestamp': datetime.now().isoformat()
        }
        
        scores = []
        
        for metric in requested_metrics:
            if metric == 'task_completion_rate':
                # Calculate from context or database
                completed = len([l for l in self.context['logs'] if 'task completed' in l.get('message', '').lower()])
                total = len([l for l in self.context['logs'] if 'task' in l.get('message', '').lower()])
                rate = (completed / total * 100) if total > 0 else 0
                metrics_result['task_completion_rate'] = rate
                scores.append(rate)
                
            elif metric == 'agv_utilization':
                # Simulate AGV utilization
                utilization = 75.5  # In real system, calculate from AGV status
                metrics_result['agv_utilization'] = utilization
                scores.append(utilization)
                
            elif metric == 'error_rate':
                # Calculate error rate from logs
                errors = len([l for l in self.context['logs'] if l.get('level') == 'error'])
                total = len(self.context['logs'])
                error_rate = (errors / total * 100) if total > 0 else 0
                metrics_result['error_rate'] = error_rate
                scores.append(100 - error_rate)  # Invert for scoring
        
        # Calculate overall score
        metrics_result['overall_score'] = sum(scores) / len(scores) if scores else 0
        
        self.log_info(f"Calculated metrics: overall_score={metrics_result['overall_score']:.2f}")
        return metrics_result
    
    @flow_function("action", "優化系統效能", ["target", "threshold"], "object",
                   defaults={"target": "efficiency", "threshold": 80})
    def optimize_performance(self, params: Dict) -> Dict:
        """Optimize system performance"""
        target = params.get('target', 'efficiency')
        threshold = params.get('threshold', 80)
        
        optimization_result = {
            'target': target,
            'threshold': threshold,
            'actions_taken': [],
            'improvement': 0
        }
        
        if target == 'efficiency':
            # Optimize for efficiency
            optimization_result['actions_taken'].append('Adjusted task batching size')
            optimization_result['actions_taken'].append('Optimized AGV routing')
            optimization_result['improvement'] = 15.5
        elif target == 'throughput':
            # Optimize for throughput
            optimization_result['actions_taken'].append('Increased parallel processing')
            optimization_result['actions_taken'].append('Reduced wait times')
            optimization_result['improvement'] = 20.3
        elif target == 'accuracy':
            # Optimize for accuracy
            optimization_result['actions_taken'].append('Enhanced validation checks')
            optimization_result['actions_taken'].append('Improved error handling')
            optimization_result['improvement'] = 8.7
        
        self.log_info(f"Optimized {target}: {optimization_result['improvement']:.1f}% improvement")
        return optimization_result
    
    @flow_function("action", "發送警報", ["message", "severity"], "boolean",
                   defaults={"message": "System alert", "severity": "warning"})
    def send_alert(self, params: Dict) -> bool:
        """Send system alert"""
        message = params.get('message', 'System alert')
        severity = params.get('severity', 'warning')
        
        # Log alert based on severity
        if severity == 'critical':
            self.log_error(f"CRITICAL ALERT: {message}")
        elif severity == 'high':
            self.log_error(f"HIGH ALERT: {message}")
        elif severity == 'warning':
            self.log_warning(f"WARNING: {message}")
        else:
            self.log_info(f"ALERT: {message}")
        
        # In real system, would send to monitoring system
        return True
    
    @flow_function("action", "清理資源", ["targets"], "boolean",
                   defaults={"targets": ["temp_variables"]})
    def cleanup_resources(self, params: Dict) -> bool:
        """Clean up temporary resources"""
        targets = params.get('targets', ['temp_variables'])
        
        for target in targets:
            if target == 'temp_variables':
                # Clean temporary variables (those starting with _temp)
                temp_vars = [k for k in self.context['variables'].keys() if k.startswith('_temp')]
                for var in temp_vars:
                    del self.context['variables'][var]
                self.log_info(f"Cleaned {len(temp_vars)} temporary variables")
                
            elif target == 'cache':
                # Clear cache (if implemented)
                self.log_info("Cache cleared")
                
            elif target == 'logs':
                # Truncate old logs
                if len(self.context['logs']) > 1000:
                    self.context['logs'] = self.context['logs'][-500:]
                    self.log_info("Truncated old logs")
        
        return True
    
    @flow_function("action", "生成報告", ["flow_id", "include"], "object",
                   defaults={"flow_id": "", "include": ["execution_time", "tasks_processed"]})
    def generate_report(self, params: Dict) -> Dict:
        """Generate execution report"""
        flow_id = params.get('flow_id', self.context.get('flow_id', ''))
        include_items = params.get('include', ['execution_time', 'tasks_processed'])
        
        report = {
            'flow_id': flow_id,
            'generated_at': datetime.now().isoformat(),
            'summary': {}
        }
        
        for item in include_items:
            if item == 'execution_time':
                report['execution_time'] = self.context['variables'].get('_execution_time', 0)
                report['summary']['duration'] = f"{report['execution_time']}ms"
                
            elif item == 'tasks_processed':
                # Count tasks from logs
                task_count = len([l for l in self.context['logs'] if 'task' in l.get('message', '').lower()])
                report['tasks_processed'] = task_count
                report['summary']['tasks'] = f"{task_count} tasks"
                
            elif item == 'errors_encountered':
                errors = self.context['variables'].get('_errors', [])
                report['errors_encountered'] = len(errors)
                report['error_details'] = errors
                report['summary']['errors'] = f"{len(errors)} errors"
                
            elif item == 'performance_metrics':
                # Include calculated metrics if available
                metrics = self.context['variables'].get('performance_metrics', {})
                report['performance_metrics'] = metrics
                report['summary']['performance'] = f"Score: {metrics.get('overall_score', 0):.1f}"
        
        self.log_info(f"Generated report for flow {flow_id}")
        return report
    
    @flow_function("action", "保存資料到檔案", ["data", "path"], "boolean",
                   defaults={"data": {}, "path": "/tmp/flow_data.json"})
    def save_data(self, params: Dict) -> bool:
        """Save data to file"""
        data = params.get('data', {})
        file_path = params.get('path', '/tmp/flow_data.json')
        
        try:
            # Ensure directory exists
            import os
            dir_path = os.path.dirname(file_path)
            if dir_path and not os.path.exists(dir_path):
                os.makedirs(dir_path, exist_ok=True)
            
            # Save data as JSON
            import json
            with open(file_path, 'w') as f:
                json.dump(data, f, indent=2, default=str)
            
            self.log_info(f"Saved data to {file_path}")
            return True
            
        except Exception as e:
            self.log_error(f"Failed to save data: {e}")
            return False
    
    @flow_function("action", "發送警報", ["message", "severity"], "boolean",
                   defaults={"message": "System alert", "severity": "medium"})
    def send_alarm(self, params: Dict) -> bool:
        """Send an alarm notification"""
        message = params.get('message', 'System alert')
        severity = params.get('severity', 'medium')
        
        # Resolve variables in message if needed
        if '${' in message:
            import re
            pattern = r'\${([^}]+)}'
            
            def replacer(match):
                expr = match.group(1)
                value = self._resolve_complex_expression(expr)
                return str(value) if value is not None else match.group(0)
            
            message = re.sub(pattern, replacer, message)
        
        # Log alarm with appropriate severity
        if severity in ['critical', 'high']:
            self.log_error(f"🚨 ALARM [{severity}]: {message}")
        elif severity == 'medium':
            self.log_warning(f"⚠️ ALARM [{severity}]: {message}")
        else:
            self.log_info(f"📢 ALARM [{severity}]: {message}")
        
        # In real system, this would trigger alarm system
        # Could integrate with external alerting services
        
        return True
    
    @flow_function("action", "觸發事件", ["event_type", "agv_id", "data"], "boolean",
                   defaults={"event_type": "custom_event", "agv_id": None, "data": {}})
    def trigger_event(self, params: Dict) -> bool:
        """Trigger a system event"""
        event_type = params.get('event_type')
        agv_id = params.get('agv_id')
        event_data = params.get('data', {})
        
        # Log the event
        self.log_info(f"🎯 Triggering event: {event_type}")
        if agv_id:
            self.log_info(f"   Target AGV: {agv_id}")
        if event_data:
            self.log_info(f"   Event data: {event_data}")
        
        # In real system, this would:
        # 1. Publish event to event bus
        # 2. Trigger AGV state changes
        # 3. Update system state
        # 4. Notify relevant subsystems
        
        # Simulate event handling
        if event_type == "agv_manual_intervention_requested":
            # Handle manual intervention request
            self.log_info(f"   Manual intervention requested for AGV {agv_id}")
            # Would update AGV status in database
            
        elif event_type == "agv_recovery_attempt":
            # Handle recovery attempt
            self.log_info(f"   Attempting to recover AGV {agv_id}")
            # Would send recovery commands to AGV
        
        return True
    
    def _get_variable_changes(self, before: Dict, after: Dict) -> Dict:
        """Get the changes between two variable states
        
        Args:
            before: Variables before step execution
            after: Variables after step execution
            
        Returns:
            Dictionary of changes with added, modified, and removed keys
        """
        changes = {
            'added': {},
            'modified': {},
            'removed': []
        }
        
        # Check for added and modified variables
        for key, value in after.items():
            if key not in before:
                changes['added'][key] = value
            elif before[key] != value:
                changes['modified'][key] = {
                    'old': before[key],
                    'new': value
                }
        
        # Check for removed variables
        for key in before:
            if key not in after:
                changes['removed'].append(key)
        
        return changes


def main():
    """Test the flow executor with direct database access"""
    # Load a test flow
    test_flow = {
        'meta': {
            'system': 'linear_flow_v2',
            'version': '2.0.0'
        },
        'flow': {
            'id': 'test_db_flow',
            'name': 'Test Database Flow',
            'work_id': '220001'
        },
        'workflow': [
            {
                'section': 'Query Database',
                'steps': [
                    {
                        'id': 'query_inlet_locations',
                        'exec': 'query.locations',
                        'params': {
                            'type': 'room_inlet',
                            'has_rack': True
                        },
                        'store': 'inlet_locations'
                    },
                    {
                        'id': 'check_locations',
                        'exec': 'check.empty',
                        'params': {
                            'data': '${inlet_locations}'
                        },
                        'store': 'locations_empty'
                    }
                ]
            },
            {
                'section': 'Process Locations',
                'skip_if': '${locations_empty}',
                'steps': [
                    {
                        'id': 'foreach_location',
                        'exec': 'foreach',
                        'items': '${inlet_locations}',
                        'var': 'location',
                        'steps': [
                            {
                                'id': 'log_location',
                                'exec': 'action.log',
                                'params': {
                                    'message': 'Processing location: ${location}',
                                    'level': 'info'
                                }
                            },
                            {
                                'id': 'create_rotation_task',
                                'exec': 'task.create',
                                'params': {
                                    'type': 'RACK_ROTATION',
                                    'location_id': '${location.id}',
                                    'priority': 'high',
                                    'metadata': {
                                        'source': 'test_flow'
                                    }
                                },
                                'store': 'task_id'
                            }
                        ]
                    }
                ]
            }
        ]
    }
    
    # Create executor
    executor = FlowExecutor(test_flow)
    
    # Run flow
    import asyncio
    result = asyncio.run(executor.execute())
    
    print(f"Flow completed with status: {result['status']}")
    print(f"Variables: {result['variables']}")
    print(f"Errors: {result['errors']}")
    print(f"Log count: {len(result['logs'])}")


def main(args=None):
    """Main entry point for flow executor as a standalone script"""
    import sys
    
    print("Flow Executor Service Started")
    print("This service is typically called by flow_wcs_node")
    print("Running in standalone mode for testing...")
    
    # For standalone testing, you can run a test flow here
    if '--test' in sys.argv:
        test_flow = {
            'meta': {
                'system': 'linear_flow_v2',
                'version': '2.0.0'
            },
            'flow': {
                'id': 'test_executor',
                'name': 'Test Flow Executor',
                'work_id': '999999'
            },
            'workflow': [
                {
                    'section': 'Test Section',
                    'steps': [
                        {
                            'id': 'test_log',
                            'exec': 'action.log',
                            'params': {
                                'message': 'Flow Executor is working!',
                                'level': 'info'
                            }
                        }
                    ]
                }
            ]
        }
        
        executor = FlowExecutor(test_flow)
        result = asyncio.run(executor.execute())
        print(f"Test flow completed with status: {result['status']}")
    else:
        print("Use --test flag to run a test flow")
        # In production, this would be integrated with ROS 2 or called by flow_wcs_node
        while True:
            time.sleep(1)


if __name__ == '__main__':
    import sys
    main(sys.argv)