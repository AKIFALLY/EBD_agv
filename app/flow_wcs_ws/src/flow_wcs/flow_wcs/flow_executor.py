#!/usr/bin/env python3
"""
Flow Executor - Improved version combining best features from old and new versions

This version includes:
1. Complete trace logging system from old version
2. Detailed execution logging with context
3. Proper variable change tracking
4. Enhanced foreach execution tracking
5. All original functionality preserved
6. TAFL v1.0 integration support
"""

import re
import time
import logging
import copy
import json
import asyncio
from typing import Dict, Any, List, Optional, Union
from datetime import datetime

# Import modular functions
from .functions import (
    QueryFunctions,
    CheckFunctions,
    TaskFunctions,
    ActionFunctions,
    ControlFunctions
)

# Import database manager
from .database import db_manager

# Import decorator system
from .decorators import flow_function

# Import TAFL integration
try:
    from .tafl_integration import TAFLIntegration, HybridFlowExecutor
    TAFL_AVAILABLE = True
except ImportError:
    TAFL_AVAILABLE = False
    logger.warning("TAFL integration not available")

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class FlowExecutor:
    """
    Linear Flow v2 Executor with enhanced trace logging and monitoring.
    
    Combines the best features from both old and new versions:
    - Modular function organization from new version
    - Complete trace system from old version
    - Enhanced logging with context
    - Proper variable change tracking
    """
    
    def __init__(self, flow_config: Dict[str, Any] = None, context: Dict = None, ros_node=None, flow_data: Dict[str, Any] = None):
        """
        Initialize FlowExecutor with flow configuration.
        
        Args:
            flow_config: The flow configuration dictionary (new parameter name)
            context: Optional initial context (for compatibility)
            ros_node: Optional ROS 2 node for logging
            flow_data: Alias for flow_config (for backward compatibility)
        """
        # Handle both parameter names for backward compatibility
        if flow_data is not None and flow_config is None:
            flow_config = flow_data
        elif flow_config is None:
            raise ValueError("Either flow_config or flow_data must be provided")
            
        self.flow_config = flow_config
        self.flow_data = flow_config  # Add alias for compatibility
        self.ros_node = ros_node  # Store ROS node reference for logging
        self.flow_id = flow_config.get('flow', {}).get('id', 'unknown')
        self.flow_name = flow_config.get('flow', {}).get('name', 'Unnamed Flow')
        self.work_id = flow_config.get('flow', {}).get('work_id', None)
        
        # Initialize variable stack for proper scoping
        self.variable_stack = []
        
        # Initialize execution tracking
        self.init_time = time.time()
        
        # Setup logger (prefer ROS logger if available)
        if self.ros_node:
            try:
                self.logger = self.ros_node.get_logger()
                self.logger.info(f"🚀 FlowExecutor initialized at {self.init_time:.2f} for flow {self.flow_id}")
            except Exception as e:
                self.logger = logger
                self.logger.info(f"Using Python logger in thread context: {e}")
        else:
            self.logger = logger
        
        # Initialize context with complete structure (like old version)
        if context:
            self.context = context
            # Ensure all required fields exist
            self.context.setdefault('variables', {})
            self.context.setdefault('flow_stopped', False)
            self.context.setdefault('stop_reason', None)
            self.context.setdefault('current_section', None)
            self.context.setdefault('current_step', None)
            self.context.setdefault('execution_history', [])
            self.context.setdefault('error_count', 0)
            self.context.setdefault('logs', [])
            self.context.setdefault('errors', [])
            self.context.setdefault('status', 'initialized')
        else:
            # Initialize with system variables like old version
            start_time = datetime.now()
            self.context = {
                'variables': {
                    # System variables (readonly)
                    '_flow_id': self.flow_id,
                    '_flow_name': self.flow_name,
                    '_work_id': self.work_id,
                    '_priority': flow_config.get('flow', {}).get('priority', 100),
                    '_timestamp': start_time.isoformat(),
                    '_execution_time': 0,
                    '_errors': []
                },
                'flow_id': self.flow_id,
                'flow_name': self.flow_name,
                'work_id': self.work_id,
                'start_time': start_time,
                'flow_stopped': False,
                'stop_reason': None,
                'current_section': None,
                'current_step': None,
                'execution_history': [],
                'error_count': 0,
                'logs': [],
                'errors': [],
                'status': 'initialized'
            }
            
            # Initialize user-defined variables from flow data
            if 'variables' in flow_config:
                for var_name, var_def in flow_config['variables'].items():
                    if not var_name.startswith('_'):  # Skip system variables
                        if isinstance(var_def, dict):
                            self.context['variables'][var_name] = var_def.get('default', None)
                        else:
                            self.context['variables'][var_name] = var_def
        
        # Initialize database manager
        self.db_manager = db_manager
        
        # Initialize function modules with reference to executor
        self.query_functions = QueryFunctions(self)
        self.check_functions = CheckFunctions(self)
        self.task_functions = TaskFunctions(self)
        self.action_functions = ActionFunctions(self)
        self.control_functions = ControlFunctions(self)
        
        # Map function categories to their handlers
        self.function_handlers = {
            'query': self.query_functions,
            'check': self.check_functions,
            'task': self.task_functions,
            'action': self.action_functions,
            'control': self.control_functions,
            'flow': self  # Flow functions are handled by executor itself
        }
        
        self.logger.info(f"FlowExecutor initialized for flow: {self.flow_name} (ID: {self.flow_id})")
    
    async def execute(self) -> Dict[str, Any]:
        """
        Execute the complete flow.
        
        Returns:
            Execution result dictionary
        """
        self.context['status'] = 'running'
        self.context['start_time'] = datetime.now()
        start_time = time.time()
        
        try:
            # Execute workflow sections
            workflow = self.flow_config.get('workflow', [])
            
            for section in workflow:
                if self.context['flow_stopped']:
                    break
                
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
                self.db_manager.log_flow_execution(
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
    
    async def execute_in_test_mode(self, dry_run: bool = False) -> Dict:
        """
        Execute flow in test mode with enhanced tracking.
        
        Args:
            dry_run: If True, only simulate execution without actual database operations
            
        Returns:
            Detailed execution report with step tracking and variable snapshots
        """
        # Set test mode flags
        self.context['test_mode'] = True
        self.context['dry_run'] = dry_run
        
        # Initialize detailed tracking
        self.context['execution_trace'] = []
        self.context['variable_snapshots'] = []
        self.context['detailed_logs'] = []
        
        # Execute flow asynchronously
        result = await self.execute()
        
        # Return detailed report
        return {
            'status': self.context['status'],
            'success': self.context['status'] == 'completed',
            'duration': self.context['variables'].get('_execution_time', 0) / 1000.0,
            'trace': self.context.get('execution_trace', []),
            'snapshots': self.context.get('variable_snapshots', []),
            'final_variables': self.context.get('variables', {}),
            'errors': self.context.get('errors', []),
            'detailed_logs': self.context.get('detailed_logs', []),
            'logs': self.context.get('logs', [])
        }
    
    async def execute_section(self, section: Dict[str, Any]) -> None:
        """
        Execute a workflow section.
        
        Args:
            section: Section configuration
        """
        section_name = section.get('section', 'unnamed')
        self.context['current_section'] = section_name
        
        self.log_info(f"Executing section: {section_name}")
        
        # Check section condition
        if 'condition' in section:
            if not self.evaluate_condition(section['condition']):
                self.log_info(f"Skipping section {section_name} - condition not met")
                return
        
        # Execute steps
        steps = section.get('steps', [])
        for step in steps:
            await self.execute_step(step)
            
            if self.context['status'] == 'stopped':
                break
    
    async def execute_step(self, step: Dict[str, Any]) -> Any:
        """
        Execute a single step with complete tracking.
        
        Args:
            step: Step configuration
            
        Returns:
            Step execution result
        """
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
                'variables_before': copy.deepcopy(self.context.get('variables', {}))
            }
        
        # Check skip condition
        if 'skip_if' in step:
            condition = step['skip_if']
            result = self.evaluate_condition(condition)
            
            # Log detailed condition evaluation for test mode
            condition_details = {
                'condition_expression': condition,
                'evaluation_result': result,
                'variables_used': {}
            }
            
            # Extract and log variables used in condition
            var_pattern = r'\\${([^}]+)}'
            var_matches = re.findall(var_pattern, str(condition))
            for var_name in var_matches:
                var_value = self.context['variables'].get(var_name, 'UNDEFINED')
                condition_details['variables_used'][var_name] = var_value
            
            self.log_info(f"Step {step_id} skip_if condition: {condition} = {result}", condition_details)
            
            if result:
                self.log_info(f"⏭️ Skipping step {step_id} due to skip_if condition")
                # Track skipped step in test mode
                if self.context.get('test_mode'):
                    self.context.setdefault('execution_trace', []).append({
                        'step_id': step_id,
                        'function': step.get('exec', ''),
                        'status': 'skipped',
                        'reason': 'skip_if condition met',
                        'timestamp': datetime.now().isoformat()
                    })
                return None
        
        # Get execution function
        exec_cmd = step.get('exec', '')
        
        try:
            if exec_cmd == 'foreach':
                result = await self.execute_foreach(step)
            elif exec_cmd == 'parallel':
                result = await self.execute_parallel(step)
            else:
                result = await self.execute_function(step)
            
            # Log step execution to database
            duration = time.time() - step_start
            resolved_params = self.resolve_parameters(step.get('params', {}))
            
            # Get the result from context
            result_data = {'success': True}
            
            # Store result if specified
            if 'store' in step:
                store_name = self.resolve_value(step['store'])
                self.context['variables'][store_name] = result
                result_data['stored_to'] = store_name
            
            # Only log to database if not in dry_run mode
            if not self.context.get('dry_run', False):
                self.db_manager.log_flow_execution(
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
                step_trace['status'] = 'completed'
                step_trace['result'] = result_data
                step_trace['variables_after'] = copy.deepcopy(self.context.get('variables', {}))
                
                # Add to execution trace
                self.context.setdefault('execution_trace', []).append(step_trace)
                
                # Take variable snapshot if significant changes
                if step_trace['variables_before'] != step_trace['variables_after']:
                    snapshot = {
                        'step_id': step_id,
                        'changes': self._get_variable_changes(
                            step_trace['variables_before'],
                            step_trace['variables_after']
                        )
                    }
                    self.context.setdefault('variable_snapshots', []).append(snapshot)
            
            return result
            
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
            resolved_params = self.resolve_parameters(step.get('params', {}))
            if not self.context.get('dry_run', False):
                self.db_manager.log_flow_execution(
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
            
            return None
    
    async def execute_function(self, step: Dict[str, Any]) -> Any:
        """
        Execute a function step with detailed logging.
        
        Args:
            step: Step configuration with 'exec' and 'params'
            
        Returns:
            Function execution result
        """
        exec_cmd = step.get('exec', '')
        params = step.get('params', {})
        step_id = step.get('id', 'unknown')
        
        # Resolve variables in parameters
        resolved_params = self.resolve_parameters(params)
        
        # Parse function name (e.g., "query.locations")
        parts = exec_cmd.split('.')
        if len(parts) != 2:
            self.log_warning(f"Invalid function name format: {exec_cmd}")
            return None
        
        category, method = parts
        
        # Get handler for category
        handler = self.function_handlers.get(category)
        if not handler:
            self.log_warning(f"Unknown function category: {category}")
            return None
        
        # Get method from handler
        if not hasattr(handler, method):
            self.log_warning(f"Function {method} not found in {category}")
            return None
        
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
        
        # Execute function
        func = getattr(handler, method)
        result = func(resolved_params)
        
        # Store result if specified
        if 'store' in step:
            store_name = self.resolve_value(step['store'])
            self.context['variables'][store_name] = result
            self.log_info(f"   Stored to: {store_name}")
        
        # Enhanced result logging based on type
        if isinstance(result, list):
            self.log_info(f"   Result: List with {len(result)} items")
            if len(result) > 0:
                first_item = result[0]
                if isinstance(first_item, dict):
                    self.log_info(f"   First item: {first_item}")
                    if 'node_id' in first_item:
                        self.log_info(f"   ✨ Node IDs: {[item.get('node_id') for item in result[:5]]}")
                    if 'room_id' in first_item:
                        self.log_info(f"   ✨ Room IDs: {[item.get('room_id') for item in result[:5]]}")
        elif isinstance(result, dict):
            self.log_info(f"   Result: Dictionary")
            self.log_info(f"   Result: {result}")
        elif isinstance(result, bool):
            self.log_info(f"   Result: {'✅ True' if result else '❌ False'}")
        elif result is None:
            self.log_info(f"   Result: None (no data)")
        else:
            self.log_info(f"   Result: {result}")
        
        self.log_info(f"✅ Function {exec_cmd} completed successfully")
        self.log_info(f"========================================")
        
        return result
    
    async def execute_foreach(self, step: Dict[str, Any]) -> List[Any]:
        """
        Execute a foreach loop with proper variable scoping.
        
        Args:
            step: Foreach step configuration
            
        Returns:
            List of results from each iteration
        """
        # Check if parameters are in params dict (new format) or at top level (old format)
        params = step.get('params', {})
        
        # Get items, var, and steps from params if available, otherwise from step directly
        items_ref = params.get('items', step.get('items', ''))
        var_name = params.get('var', step.get('var', '_item'))
        sub_steps = params.get('steps', step.get('steps', []))
        
        # Resolve items
        items = self.resolve_value(items_ref)
        
        if not isinstance(items, list):
            self.log_warning(f"Foreach items is not a list: {items_ref}")
            return []
        
        # Log the foreach iteration details
        self.log_info(f"Foreach processing {len(items)} items")
        
        results = []
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
                result = await self.execute_step(sub_step)
                if result is not None:
                    results.append(result)
                
                if self.context['status'] == 'stopped':
                    # Pop scope before returning
                    self.pop_variable_scope()
                    return results
            
            # Pop scope to restore previous context
            self.pop_variable_scope()
        
        self.log_info(f"Foreach completed {len(items)} iterations")
        return results
    
    async def execute_parallel(self, step: Dict[str, Any]) -> List[Any]:
        """
        Execute parallel branches (currently sequential).
        
        Args:
            step: Parallel step configuration
            
        Returns:
            List of results from each branch
        """
        # Support both old format (branches at top level) and new format (branches in params)
        branches = step.get('branches', [])
        
        # If branches is empty, check if this is complex_flow_example format
        if not branches:
            # Look for named branches as direct children
            for key, value in step.items():
                if key not in ['id', 'exec', 'params', 'store', 'skip_if', 'skip_if_not']:
                    if isinstance(value, dict) and 'steps' in value:
                        branches.append(value)
        
        # Execute each branch
        results = []
        for branch in branches:
            if isinstance(branch, dict):
                branch_name = branch.get('name', 'unnamed')
                branch_steps = branch.get('steps', [])
                self.log_info(f"Starting parallel branch: {branch_name}")
                for step in branch_steps:
                    result = await self.execute_step(step)
                    if result is not None:
                        results.append(result)
        
        if not branches:
            self.log_warning("No valid branches found for parallel execution")
        
        return results
    
    def evaluate_condition(self, condition: Union[str, Dict, bool]) -> bool:
        """
        Evaluate a condition with full logical operator support.
        
        Args:
            condition: Condition expression
            
        Returns:
            Boolean evaluation result
        """
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
                    if self.evaluate_condition(part.strip()):
                        return True
                return False
            
            if '&&' in condition:
                # AND operator
                parts = condition.split('&&')
                for part in parts:
                    if not self.evaluate_condition(part.strip()):
                        return False
                return True
            
            if condition.strip().startswith('!'):
                # NOT operator
                inner_condition = condition.strip()[1:].strip()
                return not self.evaluate_condition(inner_condition)
            
            # Variable reference
            if condition.startswith('${') and condition.endswith('}'):
                value = self.resolve_value(condition)
                return bool(value)
            
            # Expression evaluation (simplified)
            if '==' in condition:
                parts = condition.split('==')
                left = self.resolve_value(parts[0].strip())
                right = self.resolve_value(parts[1].strip())
                return left == right
            
            if '!=' in condition:
                parts = condition.split('!=')
                left = self.resolve_value(parts[0].strip())
                right = self.resolve_value(parts[1].strip())
                return left != right
            
            return bool(condition)
        
        if isinstance(condition, dict):
            # Complex condition
            op = condition.get('op', 'and')
            conditions = condition.get('conditions', [])
            
            if op == 'and':
                for cond in conditions:
                    if not self.evaluate_condition(cond):
                        return False
                return True
            elif op == 'or':
                for cond in conditions:
                    if self.evaluate_condition(cond):
                        return True
                return False
        
        return False
    
    def resolve_value(self, value: Any) -> Any:
        """
        Resolve a value that may contain variable references or expressions.
        
        Args:
            value: Value to resolve
            
        Returns:
            Resolved value
        """
        if value is None:
            return None
        
        if isinstance(value, str):
            # Check for variable reference
            if value.startswith('${') and value.endswith('}'):
                expr = value[2:-1]
                return self._resolve_complex_expression(expr)
            
            # Check for inline variables
            if '${' in value:
                return self.resolve_template(value)
        
        elif isinstance(value, dict):
            # Recursively resolve dictionary values
            return {k: self.resolve_value(v) for k, v in value.items()}
        
        elif isinstance(value, list):
            # Recursively resolve list items
            return [self.resolve_value(item) for item in value]
        
        return value
    
    def _resolve_complex_expression(self, expr: str) -> Any:
        """
        Resolve complex variable expressions like old version.
        
        Args:
            expr: Expression to resolve
            
        Returns:
            Resolved value
        """
        try:
            # Check for simple math operations
            if '+' in expr or '-' in expr or '*' in expr or '/' in expr:
                # Try to evaluate math expression
                try:
                    # First resolve any variables in the expression
                    var_pattern = r'(\\w+(?:\\.\\w+)*)'
                    
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
                    if re.match(r'^[\\d\\s+\\-*/()]+$', resolved_expr):
                        result = eval(resolved_expr)
                        return result
                except Exception:
                    pass
            
            # Parse the expression for property access
            # Examples: variable, variable.length, variable[0], variable[0].property
            parts = expr.replace('[', '.').replace(']', '').split('.')
            
            # Start with the base variable
            var_name = parts[0]
            value = self.context['variables'].get(var_name)
            
            if value is None:
                return None
            
            # Process each part of the expression
            for i, part in enumerate(parts[1:], 1):
                if part == 'length':
                    # Handle .length for arrays
                    if isinstance(value, (list, str)):
                        value = len(value)
                    else:
                        value = 0
                elif part.isdigit():
                    # Handle array index
                    index = int(part)
                    if isinstance(value, list) and index < len(value):
                        value = value[index]
                    else:
                        return None
                elif isinstance(value, dict):
                    # Handle object property
                    value = value.get(part)
                    if value is None:
                        return None
                elif isinstance(value, list) and len(value) > 0 and isinstance(value[0], dict):
                    # Handle array of objects property
                    value = [item.get(part) for item in value if isinstance(item, dict)]
                else:
                    return None
            
            return value
        except Exception:
            return None
    
    def resolve_template(self, template: str) -> str:
        """
        Resolve a template string with variable placeholders.
        
        Args:
            template: Template string
            
        Returns:
            Resolved string
        """
        def replacer(match):
            expr = match.group(1)
            value = self._resolve_complex_expression(expr)
            return str(value) if value is not None else match.group(0)
        
        return re.sub(r'\\${([^}]+)}', replacer, template)
    
    def resolve_parameters(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Resolve all parameters in a dictionary.
        
        Args:
            params: Parameters dictionary
            
        Returns:
            Resolved parameters
        """
        return {key: self.resolve_value(value) for key, value in params.items()}
    
    def push_variable_scope(self):
        """Push current variables to stack and create new scope with inheritance"""
        # Create a deep copy of current variables
        current_vars = copy.deepcopy(self.context['variables'])
        self.variable_stack.append(current_vars)
        # Keep a copy of variables for the new scope (inherits from parent)
        self.context['variables'] = copy.deepcopy(current_vars)
    
    def pop_variable_scope(self):
        """Pop variable scope from stack and restore previous scope"""
        if self.variable_stack:
            # Restore previous scope
            self.context['variables'] = self.variable_stack.pop()
        else:
            self.log_warning("Attempted to pop empty variable stack")
    
    # Logging functions (like old version)
    
    def log_info(self, message: str, details: Dict = None):
        """Log info message with optional details"""
        # Log to console
        if self.ros_node:
            self.ros_node.get_logger().info(message)
        else:
            self.logger.info(message)
        
        # Add to context logs
        log_entry = {
            'level': 'info',
            'message': message,
            'timestamp': datetime.now().isoformat(),
            'section': self.context.get('current_section'),
            'step': self.context.get('current_step')
        }
        
        if details:
            log_entry['details'] = details
        
        self.context['logs'].append(log_entry)
        
        # Also add to detailed logs for test mode
        if self.context.get('test_mode'):
            self.context.setdefault('detailed_logs', []).append(log_entry)
    
    def log_warning(self, message: str, details: Dict = None):
        """Log warning message with optional details"""
        # Log to console
        if self.ros_node:
            self.ros_node.get_logger().warning(message)
        else:
            self.logger.warning(message)
        
        # Add to context logs
        log_entry = {
            'level': 'warning',
            'message': message,
            'timestamp': datetime.now().isoformat(),
            'section': self.context.get('current_section'),
            'step': self.context.get('current_step')
        }
        
        if details:
            log_entry['details'] = details
        
        self.context['logs'].append(log_entry)
        
        # Also add to detailed logs for test mode
        if self.context.get('test_mode'):
            self.context.setdefault('detailed_logs', []).append(log_entry)
    
    def log_error(self, message: str, details: Dict = None):
        """Log error message with optional details"""
        # Log to console
        if self.ros_node:
            self.ros_node.get_logger().error(message)
        else:
            self.logger.error(message)
        
        # Add to context logs
        log_entry = {
            'level': 'error',
            'message': message,
            'timestamp': datetime.now().isoformat(),
            'section': self.context.get('current_section'),
            'step': self.context.get('current_step')
        }
        
        if details:
            log_entry['details'] = details
        
        self.context['logs'].append(log_entry)
        
        # Also add to detailed logs for test mode
        if self.context.get('test_mode'):
            self.context.setdefault('detailed_logs', []).append(log_entry)
    
    def _get_variable_changes(self, before: Dict, after: Dict) -> Dict:
        """
        Get the changes between two variable states.
        
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


# Direct function exports for compatibility
def execute_flow(flow_config: Dict[str, Any], context: Dict = None, ros_node=None) -> Dict[str, Any]:
    """
    Execute a flow configuration (Linear Flow v2 or TAFL).
    
    Args:
        flow_config: Flow configuration dictionary or TAFL YAML string
        context: Optional initial context
        ros_node: Optional ROS 2 node for logging
        
    Returns:
        Execution result
    """
    # Check if this is a TAFL string
    if isinstance(flow_config, str) and TAFL_AVAILABLE:
        # Try to detect format
        from .tafl_integration import TAFLIntegration
        integration = TAFLIntegration()
        format_type = integration.detect_format(flow_config)
        
        if format_type == 'tafl':
            # Execute as TAFL
            import asyncio
            return asyncio.run(integration.execute_tafl_string(flow_config, context))
    
    # Otherwise execute as Linear Flow v2
    executor = FlowExecutor(flow_config, context, ros_node)
    return executor.execute()

async def execute_tafl(tafl_yaml: str, context: Dict = None) -> Dict[str, Any]:
    """
    Execute a TAFL v1.0 flow.
    
    Args:
        tafl_yaml: TAFL YAML content as string
        context: Optional initial context
        
    Returns:
        Execution result
    """
    if not TAFL_AVAILABLE:
        raise ImportError("TAFL integration not available")
    
    from .tafl_integration import TAFLIntegration
    integration = TAFLIntegration()
    return await integration.execute_tafl_string(tafl_yaml, context)

async def execute_hybrid_flow(yaml_content: str, context: Dict = None, ros_node=None) -> Dict[str, Any]:
    """
    Execute a flow in either TAFL or Linear Flow v2 format.
    
    Args:
        yaml_content: YAML content (auto-detects format)
        context: Optional initial context
        ros_node: Optional ROS 2 node for logging
        
    Returns:
        Execution result
    """
    if not TAFL_AVAILABLE:
        # Fall back to Linear Flow v2
        import yaml
        flow_config = yaml.safe_load(yaml_content)
        executor = FlowExecutor(flow_config, context, ros_node)
        return await executor.execute()
    
    # Use hybrid executor
    from .tafl_integration import HybridFlowExecutor
    executor = FlowExecutor({}, context, ros_node)  # Create base executor
    hybrid = HybridFlowExecutor(executor)
    return await hybrid.execute(yaml_content, context)